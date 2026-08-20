# Copyright 2016 The Cartographer Authors
# Copyright 2022 Wyca Robotics (for the ROS2 conversion)
# Copyright 2025 Unified package for Livox Mid360 2D SLAM
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)

project(cartographer_slam)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(absl REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cartographer REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_environment REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(livox_driver REQUIRED)

# glog version detection for code path selection
find_package(glog CONFIG QUIET)
if(DEFINED glog_VERSION)
  if(NOT glog_VERSION VERSION_LESS 0.7.0)
    add_definitions(-DROS_CARTOGRAPHER_GLOG_GE_070)
  endif()
endif()

include(FindPkgConfig)
pkg_search_module(CAIRO REQUIRED cairo>=1.12.16)

# ==============================================================================
# Message and Service Generation
# ==============================================================================
set(msg_files
  "msg/HistogramBucket.msg"
  "msg/LandmarkEntry.msg"
  "msg/LandmarkList.msg"
  "msg/MetricFamily.msg"
  "msg/MetricLabel.msg"
  "msg/Metric.msg"
  "msg/StatusCode.msg"
  "msg/StatusResponse.msg"
  "msg/SubmapEntry.msg"
  "msg/SubmapList.msg"
  "msg/SubmapTexture.msg"
  "msg/TrajectoryStates.msg"
)

set(srv_files
  "srv/FinishTrajectory.srv"
  "srv/GetTrajectoryStates.srv"
  "srv/ReadMetrics.srv"
  "srv/StartTrajectory.srv"
  "srv/TrajectoryQuery.srv"
  "srv/SubmapQuery.srv"
  "srv/WriteState.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

# ==============================================================================
# Library
# ==============================================================================
add_library(${PROJECT_NAME}_lib SHARED
  src/map_builder_bridge.cpp
  src/msg_conversion.cpp
  src/node_constants.cpp
  src/node.cpp
  src/node_options.cpp
  src/ros_log_sink.cpp
  src/ros_map.cpp
  src/sensor_bridge.cpp
  src/submap.cpp
  src/tf_bridge.cpp
  src/time_conversion.cpp
  src/trajectory_options.cpp
  src/metrics/family_factory.cpp
  src/metrics/internal/family.cpp
  src/metrics/internal/histogram.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${CAIRO_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

if ("$ENV{ROS_DISTRO}" STRLESS "kilted")
  # Compatibility for pre-Kilted pcl_conversions
  target_include_directories(${PROJECT_NAME}_lib PRIVATE ${pcl_conversions_INCLUDE_DIRS})
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE "-DUSE_URDF_H_FILES")
else()
  target_link_libraries(${PROJECT_NAME}_lib PRIVATE pcl_conversions::pcl_conversions)
endif()

target_link_libraries(${PROJECT_NAME}_lib PUBLIC
  absl::synchronization
  ${builtin_interfaces_TARGETS}
  cartographer
  ${geometry_msgs_TARGETS}
  ${livox_driver_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib PRIVATE
  ${CAIRO_LIBRARIES}
  pcl_common
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  ${tf2_msgs_TARGETS}
)

# Link generated messages to library
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${cpp_typesupport_target})

# ==============================================================================
# Executables
# ==============================================================================
add_executable(cartographer_node src/node_main.cpp)
target_include_directories(cartographer_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_node PRIVATE
  ${PROJECT_NAME}_lib
  absl::memory
  cartographer
  gflags
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)

add_executable(cartographer_occupancy_grid_node src/occupancy_grid_node_main.cpp)
target_include_directories(cartographer_occupancy_grid_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${CAIRO_INCLUDE_DIRS}
)
target_link_libraries(cartographer_occupancy_grid_node PRIVATE
  ${PROJECT_NAME}_lib
  absl::synchronization
  cartographer
  Eigen3::Eigen
  gflags
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(cartographer_pbstream_map_publisher src/pbstream_map_publisher_main.cpp)
target_include_directories(cartographer_pbstream_map_publisher PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_pbstream_map_publisher PRIVATE
  ${PROJECT_NAME}_lib
  cartographer
  gflags
  glog::glog
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(cartographer_pbstream_to_ros_map src/pbstream_to_ros_map_main.cpp)
target_include_directories(cartographer_pbstream_to_ros_map PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(cartographer_pbstream_to_ros_map PRIVATE
  ${PROJECT_NAME}_lib
  cartographer
  gflags
  glog::glog
  rclcpp::rclcpp
)

# ==============================================================================
# Install
# ==============================================================================
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  cartographer_node
  cartographer_occupancy_grid_node
  cartographer_pbstream_map_publisher
  cartographer_pbstream_to_ros_map
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config rviz launch
  DESTINATION share/${PROJECT_NAME}/
)

# Python scripts
install(PROGRAMS
  scripts/trajectory_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ==============================================================================
# Export
# ==============================================================================
ament_export_targets(${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  absl
  builtin_interfaces
  cartographer
  geometry_msgs
  livox_driver
  nav_msgs
  rclcpp
  rosidl_default_runtime
  sensor_msgs
  tf2_ros
  visualization_msgs
)

ament_package()
