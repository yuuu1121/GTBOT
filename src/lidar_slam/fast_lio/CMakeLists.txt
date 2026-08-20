cmake_minimum_required(VERSION 3.8)
project(fast_lio)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Multi-processing setup for FAST-LIO
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor count: ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("Core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("Core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_driver REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find third-party packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenMP REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# Find Open3D for localization
find_package(Open3D QUIET)
if(NOT Open3D_FOUND)
  message(WARNING "Open3D not found. Trying pkg-config...")
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(Open3D Open3D)
endif()
# gtbot 시뮬 패치: Open3D 없으면 localization 타깃 생략(SLAM 노드는 무관)
if(NOT Open3D_FOUND)
  message(WARNING "Open3D unavailable - building fastlio_mapping only (localization skipped)")
endif()

# Fix Open3D libraries for Ubuntu 22.04+ compatibility
if(TARGET Open3D::Open3D)
  foreach(target Open3D::Open3D)
    if(TARGET ${target})
      get_target_property(LINK_LIBS ${target} INTERFACE_LINK_LIBRARIES)
      if(LINK_LIBS)
        set(FILTERED_LIBS "")
        foreach(lib ${LINK_LIBS})
          if(NOT "${lib}" MATCHES ".*librt\\.so.*" AND
             NOT "${lib}" MATCHES ".*libc\\+\\+\\.so.*" AND
             NOT "${lib}" MATCHES ".*libc\\+\\+abi\\.so.*")
            list(APPEND FILTERED_LIBS ${lib})
          endif()
        endforeach()
        set_target_properties(${target} PROPERTIES INTERFACE_LINK_LIBRARIES "${FILTERED_LIBS}")
      endif()
    endif()
  endforeach()
endif()

message(STATUS "Eigen: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "PCL: ${PCL_INCLUDE_DIRS}")
message(STATUS "Open3D: ${Open3D_INCLUDE_DIRS}")

# Include directories
include_directories(
  include
  include/fast_lio/slam
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Open3D_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# ========== Message Generation ==========
set(msg_files
  "msg/Pose6D.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)

# ========== SLAM Dependencies List ==========
set(slam_dependencies
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  livox_driver
)

# ========== Localization Dependencies List ==========
set(localization_dependencies
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  pcl_ros
  pcl_conversions
)

# ========== SLAM Executable: fastlio_mapping ==========
add_executable(fastlio_mapping
  src/slam/laserMapping.cpp
  src/slam/ikd_Tree.cpp
  src/slam/preprocess.cpp
)

target_include_directories(fastlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(fastlio_mapping
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Link Pose6D message support
list(APPEND EOL_LIST "foxy" "galactic" "eloquent" "dashing" "crystal")
if($ENV{ROS_DISTRO} IN_LIST EOL_LIST)
  rosidl_target_interfaces(fastlio_mapping ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
endif()

ament_target_dependencies(fastlio_mapping ${slam_dependencies})

if(Open3D_FOUND)
# ========== Localization Libraries ==========
add_library(open3d_registration
  src/localization/open3d_registration.cpp
)
target_link_libraries(open3d_registration
  ${Open3D_LIBRARIES}
)

add_library(open3d_conversions
  src/localization/open3d_conversions.cpp
)
target_link_libraries(open3d_conversions
  ${Open3D_LIBRARIES}
)
ament_target_dependencies(open3d_conversions
  rclcpp
  sensor_msgs
)

# ========== OccupancyGrid Generator Library ==========
add_library(occupancy_grid_generator
  src/localization/occupancy_grid_generator.cpp
)
target_link_libraries(occupancy_grid_generator
  ${Open3D_LIBRARIES}
)
ament_target_dependencies(occupancy_grid_generator
  rclcpp
  nav_msgs
)

# ========== Localization Pose Utilities Library ==========
add_library(localization_pose_utils
  src/localization/pose_utils.cpp
)
target_link_libraries(localization_pose_utils
  ${Open3D_LIBRARIES}
)
ament_target_dependencies(localization_pose_utils
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
)

# ========== Localization Executable: localization_node ==========
add_executable(localization_node
  src/localization/localization_node.cpp
  src/localization/localization_engine.cpp
  src/localization/tf_publisher.cpp
)
target_link_libraries(localization_node
  open3d_registration
  open3d_conversions
  occupancy_grid_generator
  localization_pose_utils
  ${Open3D_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(localization_node ${localization_dependencies})

install(TARGETS
  localization_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  open3d_registration
  open3d_conversions
  occupancy_grid_generator
  localization_pose_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
endif()

# ========== Install ==========
install(TARGETS
  fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
