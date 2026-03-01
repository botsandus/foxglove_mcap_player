// Foxglove MCAP Player with ROS 2 republishing.
//
// Reads an MCAP file (typically a ROS 2 bag) and:
//   1. Serves it to Foxglove with full ranged playback controls (play/pause/seek/speed)
//   2. Publishes each CDR-encoded message to the corresponding ROS 2 topic
//
// Usage:
//   ros2 run foxglove_mcap_player foxglove_mcap_player --ros-args \
//     -p file:=/path/to/bag.mcap \
//     -p port:=8765 \
//     -p host:=127.0.0.1

#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>

#include <foxglove/channel.hpp>
#include <foxglove/foxglove.hpp>
#include <foxglove/server.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/serialization.hpp>

#include <rosbag2_storage/qos.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>

using namespace std::chrono_literals;

static std::function<void()> g_sigint_handler;  // NOLINT

// ─── TimeTracker ─────────────────────────────────────────────────────────────
// Maps wall-clock time to log-time, with support for pause/resume and variable speed.

class TimeTracker {
public:
  TimeTracker(uint64_t offset_ns, double speed)
      : offset_ns_(offset_ns), speed_(clamp_speed(speed)) {
    start_ = std::chrono::steady_clock::now();
  }

  uint64_t current_log_time() const {
    if (paused_) {
      return offset_ns_ + paused_elapsed_ns_;
    }
    auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now() - start_)
                     .count();
    return offset_ns_ + paused_elapsed_ns_ + static_cast<uint64_t>(wall_ns * speed_);
  }

  // Returns seconds to wait until log_time is reached, or nullopt if ready now.
  std::optional<double> seconds_until(uint64_t log_time) const {
    auto current = current_log_time();
    if (log_time <= current) {
      return std::nullopt;
    }
    double log_diff_ns = static_cast<double>(log_time - current);
    double wall_diff_ns = (speed_ > 0) ? (log_diff_ns / speed_) : 1e9;
    return wall_diff_ns / 1e9;
  }

  void pause() {
    if (!paused_) {
      auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now() - start_)
                       .count();
      paused_elapsed_ns_ += static_cast<uint64_t>(wall_ns * speed_);
      paused_ = true;
    }
  }

  void resume() {
    if (paused_) {
      start_ = std::chrono::steady_clock::now();
      paused_ = false;
    }
  }

  void set_speed(double speed) {
    speed = clamp_speed(speed);
    if (!paused_) {
      auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now() - start_)
                       .count();
      paused_elapsed_ns_ += static_cast<uint64_t>(wall_ns * speed_);
      start_ = std::chrono::steady_clock::now();
    }
    speed_ = speed;
  }

  // Returns a timestamp to broadcast at ~60 Hz, or nullopt.
  std::optional<uint64_t> notify(uint64_t current_ns) {
    constexpr uint64_t kInterval = 1'000'000'000 / 60;
    if (current_ns - notify_last_ >= kInterval) {
      notify_last_ = current_ns;
      return current_ns;
    }
    return std::nullopt;
  }

private:
  static constexpr double kMinSpeed = 0.01;
  static double clamp_speed(double s) { return (s >= kMinSpeed) ? s : kMinSpeed; }

  uint64_t offset_ns_;
  double speed_;
  std::chrono::steady_clock::time_point start_;
  bool paused_ = false;
  uint64_t paused_elapsed_ns_ = 0;
  uint64_t notify_last_ = 0;
};

// ─── McapPlayer ──────────────────────────────────────────────────────────────
// Reads MCAP file, manages playback state, logs to Foxglove channels, and
// publishes to ROS topics.

class McapPlayer {
public:
  explicit McapPlayer(const std::string& path, rclcpp::Node::SharedPtr node)
      : path_(path), node_(std::move(node)) {
    // Read summary for time range
    auto status = reader_.open(path_);
    if (!status.ok()) {
      throw std::runtime_error("Failed to open MCAP: " + status.message);
    }
    status = reader_.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if (!status.ok()) {
      throw std::runtime_error("Failed to read MCAP summary: " + status.message);
    }
    auto stats = reader_.statistics();
    if (!stats.has_value()) {
      throw std::runtime_error("MCAP file has no statistics");
    }
    time_range_ = {stats->messageStartTime, stats->messageEndTime};
    current_time_ = stats->messageStartTime;

    // Pre-populate channels and ROS publishers from the summary
    for (const auto& [ch_id, ch_ptr] : reader_.channels()) {
      setup_channel(*ch_ptr, reader_.schema(ch_ptr->schemaId));
    }

    // Set up the message view
    reset_view(current_time_);
  }

  ~McapPlayer() { reader_.close(); }

  std::pair<uint64_t, uint64_t> time_range() const { return time_range_; }
  uint64_t current_time() const { return current_time_; }
  double playback_speed() const { return playback_speed_; }

  foxglove::PlaybackStatus status() const { return status_; }

  void set_playback_speed(double speed) {
    speed = std::max(0.01, speed);
    if (time_tracker_) {
      time_tracker_->set_speed(speed);
    }
    playback_speed_ = speed;
  }

  void play() {
    if (status_ == foxglove::PlaybackStatus::Ended) {
      return;
    }
    if (time_tracker_) {
      time_tracker_->resume();
    }
    status_ = foxglove::PlaybackStatus::Playing;
  }

  void pause() {
    if (time_tracker_) {
      time_tracker_->pause();
    }
    status_ = foxglove::PlaybackStatus::Paused;
  }

  void seek(uint64_t log_time) {
    log_time = std::clamp(log_time, time_range_.first, time_range_.second);
    reset_view(log_time);
    if (status_ == foxglove::PlaybackStatus::Ended) {
      status_ = foxglove::PlaybackStatus::Paused;
    }
  }

  // Returns seconds to sleep if caller should wait, or nullopt if a message was logged
  // (or playback is not active).
  std::optional<double> log_next_message(foxglove::WebSocketServer& server) {
    if (status_ != foxglove::PlaybackStatus::Playing) {
      return std::nullopt;
    }

    if (*view_iter_ == *view_end_) {
      status_ = foxglove::PlaybackStatus::Ended;
      current_time_ = time_range_.second;
      return std::nullopt;
    }

    const auto& msg_view = **view_iter_;
    const auto& msg = msg_view.message;

    // Create TimeTracker on first message
    if (!time_tracker_) {
      time_tracker_ = std::make_unique<TimeTracker>(msg.logTime, playback_speed_);
    }

    // Check if we need to wait
    auto sleep_secs = time_tracker_->seconds_until(msg.logTime);
    if (sleep_secs.has_value() && *sleep_secs > 0) {
      return sleep_secs;
    }

    current_time_ = msg.logTime;

    // Broadcast time at ~60 Hz
    auto notify_time = time_tracker_->notify(msg.logTime);
    if (notify_time.has_value()) {
      server.broadcastTime(*notify_time);
    }

    // Log to Foxglove channel
    auto ch_it = foxglove_channels_.find(msg.channelId);
    if (ch_it != foxglove_channels_.end()) {
      ch_it->second.log(msg.data, msg.dataSize, msg.logTime);
    }

    // Publish to ROS topic (zero-copy — raw CDR bytes)
    auto ros_it = ros_publishers_.find(msg.channelId);
    if (ros_it != ros_publishers_.end()) {
      auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(msg.dataSize);
      auto& rcl_msg = serialized_msg->get_rcl_serialized_message();
      memcpy(rcl_msg.buffer, msg.data, msg.dataSize);
      rcl_msg.buffer_length = msg.dataSize;
      ros_it->second->publish(*serialized_msg);
    }

    ++(*view_iter_);
    return std::nullopt;
  }

private:
  void reset_view(uint64_t start_time) {
    current_time_ = start_time;
    time_tracker_.reset();

    mcap::ReadMessageOptions opts;
    opts.startTime = start_time;
    opts.endTime = time_range_.second + 1;
    opts.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
    view_.emplace(
      reader_.readMessages([](const mcap::Status& s) {
        RCLCPP_WARN(rclcpp::get_logger("mcap_player"), "MCAP read problem: %s", s.message.c_str());
      }, opts)
    );
    view_iter_.emplace(view_->begin());
    view_end_.emplace(view_->end());
  }

  void setup_channel(const mcap::Channel& mcap_ch, mcap::SchemaPtr mcap_schema) {
    // Create Foxglove channel
    std::optional<foxglove::Schema> fg_schema;
    if (mcap_schema) {
      foxglove::Schema s;
      s.name = mcap_schema->name;
      s.encoding = mcap_schema->encoding;
      s.data = mcap_schema->data.empty()
                 ? nullptr
                 : reinterpret_cast<const std::byte*>(mcap_schema->data.data());
      s.data_len = mcap_schema->data.size();
      fg_schema = std::move(s);
    }

    auto channel_result =
      foxglove::RawChannel::create(mcap_ch.topic, mcap_ch.messageEncoding, std::move(fg_schema));
    if (channel_result.has_value()) {
      foxglove_channels_.emplace(mcap_ch.id, std::move(channel_result.value()));
    } else {
      RCLCPP_WARN(
        node_->get_logger(), "Failed to create Foxglove channel for '%s'", mcap_ch.topic.c_str());
    }

    // Create ROS generic publisher for CDR-encoded channels
    if (mcap_ch.messageEncoding == "cdr" && mcap_schema) {
      try {
        // Use QoS from recorded bag metadata if available
        rclcpp::QoS qos(10);
        auto qos_it = mcap_ch.metadata.find("offered_qos_profiles");
        if (qos_it != mcap_ch.metadata.end()) {
          try {
            auto profiles = rosbag2_storage::to_rclcpp_qos_vector(qos_it->second, 9);
            if (!profiles.empty()) {
              auto adapted = rosbag2_storage::Rosbag2QoS::adapt_offer_to_recorded_offers(
                mcap_ch.topic, rosbag2_storage::from_rclcpp_qos_vector(profiles));
              qos = adapted;
            }
          } catch (const std::exception& e) {
            RCLCPP_WARN(
              node_->get_logger(), "Failed to parse QoS for '%s': %s, using default",
              mcap_ch.topic.c_str(), e.what());
          }
        } else {
          RCLCPP_WARN(
            node_->get_logger(),
            "No 'offered_qos_profiles' metadata for '%s', using default QoS (reliability: reliable, durability: volatile, depth: 10)",
            mcap_ch.topic.c_str());
        }
        auto pub = node_->create_generic_publisher(
          mcap_ch.topic, mcap_schema->name, qos);
        ros_publishers_.emplace(mcap_ch.id, std::move(pub));
        RCLCPP_INFO(
          node_->get_logger(), "ROS publisher: %s [%s]", mcap_ch.topic.c_str(),
          mcap_schema->name.c_str());
      } catch (const std::exception& e) {
        RCLCPP_WARN(
          node_->get_logger(), "Failed to create ROS publisher for '%s': %s",
          mcap_ch.topic.c_str(), e.what());
      }
    }
  }

  std::string path_;
  rclcpp::Node::SharedPtr node_;
  mcap::McapReader reader_;
  std::pair<uint64_t, uint64_t> time_range_;
  uint64_t current_time_ = 0;
  double playback_speed_ = 1.0;
  foxglove::PlaybackStatus status_ = foxglove::PlaybackStatus::Paused;
  std::unique_ptr<TimeTracker> time_tracker_;

  // The LinearMessageView must outlive the iterators
  std::optional<mcap::LinearMessageView> view_;
  std::optional<mcap::LinearMessageView::Iterator> view_iter_;
  std::optional<mcap::LinearMessageView::Iterator> view_end_;

  std::unordered_map<mcap::ChannelId, foxglove::RawChannel> foxglove_channels_;
  std::unordered_map<mcap::ChannelId, rclcpp::GenericPublisher::SharedPtr> ros_publishers_;
};

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.enable_rosout(false);
  auto node = rclcpp::Node::make_shared("foxglove_mcap_player", node_options);
  node->declare_parameter<std::string>("file", "");
  node->declare_parameter<int>("port", 8765);
  node->declare_parameter<std::string>("host", "127.0.0.1");

  auto file = node->get_parameter("file").as_string();
  auto port = static_cast<uint16_t>(node->get_parameter("port").as_int());
  auto host = node->get_parameter("host").as_string();

  if (file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'file' is required: -p file:=/path/to/bag.mcap");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Loading MCAP: %s", file.c_str());
  McapPlayer player(file, node);
  auto [start_time, end_time] = player.time_range();
  RCLCPP_INFO(node->get_logger(), "Time range: [%lu, %lu] ns", start_time, end_time);

  std::mutex lock;

  // Set up Foxglove WebSocket server with ranged playback
  foxglove::WebSocketServerOptions options;
  options.name = file;
  options.host = host;
  options.port = port;
  options.capabilities =
    foxglove::WebSocketServerCapabilities::Time | foxglove::WebSocketServerCapabilities::PlaybackControl;
  options.playback_time_range = {start_time, end_time};

  options.callbacks.onPlaybackControlRequest =
    [&](const foxglove::PlaybackControlRequest& req) -> std::optional<foxglove::PlaybackState> {
    std::lock_guard<std::mutex> guard(lock);

    if (req.seek_time.has_value()) {
      try {
        player.seek(*req.seek_time);
      } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(), "Seek failed: %s", e.what());
      }
    }

    player.set_playback_speed(static_cast<double>(req.playback_speed));

    if (req.playback_command == foxglove::PlaybackCommand::Play) {
      player.play();
    } else if (req.playback_command == foxglove::PlaybackCommand::Pause) {
      player.pause();
    }

    return foxglove::PlaybackState{
      .status = player.status(),
      .current_time = player.current_time(),
      .playback_speed = static_cast<float>(player.playback_speed()),
      .did_seek = false,
      .request_id = req.request_id,
    };
  };

  auto server_result = foxglove::WebSocketServer::create(std::move(options));
  if (!server_result.has_value()) {
    RCLCPP_ERROR(
      node->get_logger(), "Failed to create server: %s",
      foxglove::strerror(server_result.error()));
    return 1;
  }
  auto server = std::move(server_result.value());
  RCLCPP_INFO(node->get_logger(), "Foxglove server listening on %s:%d", host.c_str(), port);

  std::atomic_bool done = false;
  g_sigint_handler = [&] {
    done = true;
  };
  std::signal(SIGINT, [](int) {
    if (g_sigint_handler) {
      g_sigint_handler();
    }
  });

  auto last_status = foxglove::PlaybackStatus::Paused;
  while (rclcpp::ok() && !done) {
    foxglove::PlaybackStatus current_status;

    // Check status & broadcast state changes
    {
      std::lock_guard<std::mutex> guard(lock);
      current_status = player.status();
      if (current_status == foxglove::PlaybackStatus::Ended &&
          last_status != foxglove::PlaybackStatus::Ended) {
        server.broadcastPlaybackState(foxglove::PlaybackState{
          .status = current_status,
          .current_time = player.current_time(),
          .playback_speed = static_cast<float>(player.playback_speed()),
          .did_seek = false,
          .request_id = std::nullopt,
        });
      }
    }
    last_status = current_status;

    if (current_status != foxglove::PlaybackStatus::Playing) {
      std::this_thread::sleep_for(10ms);
      continue;
    }

    // Log next message (holds lock briefly)
    std::optional<double> sleep_duration;
    {
      std::lock_guard<std::mutex> guard(lock);
      sleep_duration = player.log_next_message(server);
    }

    if (sleep_duration.has_value()) {
      auto sleep_ms = std::min(*sleep_duration * 1000.0, 1000.0);
      std::this_thread::sleep_for(std::chrono::microseconds(static_cast<int64_t>(sleep_ms * 1000)));
    }
  }

  server.stop();
  rclcpp::shutdown();
  return 0;
}
