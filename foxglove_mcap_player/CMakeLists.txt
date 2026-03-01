cmake_minimum_required(VERSION 3.16)
project(foxglove_mcap_player LANGUAGES CXX VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Avoid DOWNLOAD_EXTRACT_TIMESTAMP warning for CMake >= 3.24
if (POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

find_package(Threads REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# ── Foxglove SDK ──────────────────────────────────────────────────────────────
message(STATUS "Fetching Foxglove SDK artifacts from GitHub")
set(FOXGLOVE_SDK_VERSION "0.18.0")

if (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(FOXGLOVE_SDK_PLATFORM "aarch64-unknown-linux-gnu")
  set(FOXGLOVE_SDK_SHA "42c042365fef1e46b72a956ca37b7f6ebfaf93ce405e7bbd19f38d43c83a17a9")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(FOXGLOVE_SDK_PLATFORM "x86_64-unknown-linux-gnu")
  set(FOXGLOVE_SDK_SHA "a428aff940ddebaca617a42328d6245674a824607de9e3991c5291fc3fd4d408")
else()
  message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}")
endif()
set(FOXGLOVE_SDK_URL "https://github.com/foxglove/foxglove-sdk/releases/download/sdk%2Fv${FOXGLOVE_SDK_VERSION}/foxglove-v${FOXGLOVE_SDK_VERSION}-cpp-${FOXGLOVE_SDK_PLATFORM}.zip")

include(FetchContent)
FetchContent_Declare(
  foxglove_sdk
  URL ${FOXGLOVE_SDK_URL}
  URL_HASH SHA256=${FOXGLOVE_SDK_SHA}
)
FetchContent_MakeAvailable(foxglove_sdk)
add_library(foxglove_cpp_static STATIC)
target_include_directories(foxglove_cpp_static SYSTEM
  PUBLIC
    $<BUILD_INTERFACE:${foxglove_sdk_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
file(GLOB_RECURSE FOXGLOVE_SDK_SOURCES CONFIGURE_DEPENDS "${foxglove_sdk_SOURCE_DIR}/src/*.cpp")
target_sources(foxglove_cpp_static PRIVATE ${FOXGLOVE_SDK_SOURCES})
set_target_properties(foxglove_cpp_static PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(foxglove_cpp_static PRIVATE ${foxglove_sdk_SOURCE_DIR}/lib/libfoxglove.a)

# ── MCAP (header-only) ───────────────────────────────────────────────────────
set(MCAP_VERSION "1.4.1")
FetchContent_Declare(
  mcap
  URL "https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v${MCAP_VERSION}.tar.gz"
)
FetchContent_MakeAvailable(mcap)
add_library(mcap_headers INTERFACE)
target_include_directories(mcap_headers SYSTEM INTERFACE
  ${mcap_SOURCE_DIR}/cpp/mcap/include
)

# MCAP depends on lz4 and zstd for decompression
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED IMPORTED_TARGET liblz4)
pkg_check_modules(ZSTD REQUIRED IMPORTED_TARGET libzstd)

# ── ROS 2 ────────────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(foxglove_mcap_player
  src/main.cpp
)

target_include_directories(foxglove_mcap_player PRIVATE
  $<BUILD_INTERFACE:${foxglove_sdk_SOURCE_DIR}/include>
)

target_link_libraries(foxglove_mcap_player
  foxglove_cpp_static
  mcap_headers
  rclcpp::rclcpp
  ${rosbag2_storage_LIBRARIES}
  PkgConfig::LZ4
  PkgConfig::ZSTD
  Threads::Threads
)
target_include_directories(foxglove_mcap_player PRIVATE
  ${rosbag2_storage_INCLUDE_DIRS}
)

install(TARGETS foxglove_mcap_player
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
