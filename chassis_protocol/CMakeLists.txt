cmake_minimum_required(VERSION 3.14)
project(chassis_protocol VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Protobuf QUIET)
find_package(ament_cmake_gtest REQUIRED)

# ── Optional: protobuf code generation ───────────────────────────────────────
if(Protobuf_FOUND)
  protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS proto/chassis.proto)
  add_library(chassis_proto STATIC ${PROTO_SRCS})
  target_include_directories(chassis_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(chassis_proto PUBLIC protobuf::libprotobuf)
  target_compile_features(chassis_proto PUBLIC cxx_std_17)
endif()

# ── Frame codec library ───────────────────────────────────────────────────────
add_library(chassis_protocol_frame STATIC
  frame/frame_codec.cpp
)
target_include_directories(chassis_protocol_frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(chassis_protocol_frame PUBLIC cxx_std_17)

# ── Transport library ─────────────────────────────────────────────────────────
add_library(chassis_protocol_transport STATIC
  transport/itransport.cpp
  transport/tcp_transport.cpp
  transport/udp_transport.cpp
  transport/rs485_transport.cpp
)
target_include_directories(chassis_protocol_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(chassis_protocol_transport PUBLIC chassis_protocol_frame)
target_compile_features(chassis_protocol_transport PUBLIC cxx_std_17)

# POSIX thread support
find_package(Threads REQUIRED)
target_link_libraries(chassis_protocol_transport PUBLIC Threads::Threads)

# ── HAL library ───────────────────────────────────────────────────────────────
add_library(chassis_protocol_hal STATIC
  chassis/version_negotiator.cpp
  chassis/chassis_hal.cpp
  chassis/differential/differential_controller.cpp
  chassis/mecanum/mecanum_controller.cpp
)
target_include_directories(chassis_protocol_hal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(chassis_protocol_hal PUBLIC
  chassis_protocol_frame
  chassis_protocol_transport
)
target_compile_features(chassis_protocol_hal PUBLIC cxx_std_17)

# ── ROS2 adapter library ──────────────────────────────────────────────────────
add_library(chassis_protocol_ros2_adapter STATIC
  ros2_adapter/chassis_node.cpp
)
target_include_directories(chassis_protocol_ros2_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(chassis_protocol_ros2_adapter PUBLIC chassis_protocol_hal)
ament_target_dependencies(chassis_protocol_ros2_adapter
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs diagnostic_msgs)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(chassis_node ros2_adapter/chassis_node_main.cpp)
target_link_libraries(chassis_node PRIVATE chassis_protocol_ros2_adapter)
ament_target_dependencies(chassis_node rclcpp)

# ── Tests ─────────────────────────────────────────────────────────────────────
ament_add_gtest(frame_codec_test tests/frame_codec_test.cpp)
target_link_libraries(frame_codec_test chassis_protocol_frame)

ament_add_gtest(transport_mock_test tests/transport_mock_test.cpp)
target_link_libraries(transport_mock_test chassis_protocol_hal)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  chassis_protocol_frame
  chassis_protocol_transport
  chassis_protocol_hal
  chassis_protocol_ros2_adapter
  chassis_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY frame/ transport/ chassis/ ros2_adapter/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
