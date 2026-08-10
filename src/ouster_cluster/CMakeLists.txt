cmake_minimum_required(VERSION 3.8)
project(ouster_cluster)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Eigen3 REQUIRED)  # headers use Eigen directly, not only via PCL

include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Optional CUDA front-end (conv+cut / crop / BEV density fused on the GPU).
# Built only when a CUDA toolchain is present; the node falls back to the
# CPU path otherwise (OUSTER_CLUSTER_HAS_CUDA guards all GPU code).
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 87)  # Jetson Orin; override with -DCMAKE_CUDA_ARCHITECTURES
  endif()
  find_package(CUDAToolkit REQUIRED)
  add_library(ouster_cluster_gpu STATIC
    src/gpu_frontend.cu
    src/gpu_frontend.cpp
  )
  target_include_directories(ouster_cluster_gpu PRIVATE
    include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  ament_target_dependencies(ouster_cluster_gpu sensor_msgs)
  target_link_libraries(ouster_cluster_gpu CUDA::cudart)
  set_target_properties(ouster_cluster_gpu PROPERTIES POSITION_INDEPENDENT_CODE ON)
  message(STATUS "ouster_cluster: CUDA front-end ENABLED (arch ${CMAKE_CUDA_ARCHITECTURES})")
else()
  message(STATUS "ouster_cluster: no CUDA toolchain, CPU-only build")
endif()

# Node executable
add_executable(ouster_cluster_node
  src/ouster_cluster_node.cpp
)
ament_target_dependencies(ouster_cluster_node
  rclcpp sensor_msgs visualization_msgs pcl_conversions
)
target_link_libraries(ouster_cluster_node ${PCL_LIBRARIES})
if(CMAKE_CUDA_COMPILER)
  target_compile_definitions(ouster_cluster_node PRIVATE OUSTER_CLUSTER_HAS_CUDA)
  target_link_libraries(ouster_cluster_node ouster_cluster_gpu)
endif()

install(TARGETS ouster_cluster_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cluster_logic test/test_cluster_logic.cpp)
  target_include_directories(test_cluster_logic PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_cluster_logic ${PCL_LIBRARIES})

  ament_add_gtest(test_bev_tracker test/test_bev_tracker.cpp)
  target_include_directories(test_bev_tracker PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_bev_tracker ${PCL_LIBRARIES})

  ament_add_gtest(test_ouster_convert test/test_ouster_convert.cpp)
  target_include_directories(test_ouster_convert PRIVATE include ${PCL_INCLUDE_DIRS})
  ament_target_dependencies(test_ouster_convert sensor_msgs)
  target_link_libraries(test_ouster_convert ${PCL_LIBRARIES})

  ament_add_gtest(test_plane_detect test/test_plane_detect.cpp)
  target_include_directories(test_plane_detect PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_plane_detect ${PCL_LIBRARIES})

  ament_add_gtest(test_bev_density test/test_bev_density.cpp)
  target_include_directories(test_bev_density PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_bev_density ${PCL_LIBRARIES})

  ament_add_gtest(test_bev_lines test/test_bev_lines.cpp)
  target_include_directories(test_bev_lines PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_bev_lines ${PCL_LIBRARIES})

  ament_add_gtest(test_elevation_filter test/test_elevation_filter.cpp)
  target_include_directories(test_elevation_filter PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_elevation_filter ${PCL_LIBRARIES})

  ament_add_gtest(test_refl_gate test/test_refl_gate.cpp)
  target_include_directories(test_refl_gate PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_refl_gate ${PCL_LIBRARIES})

  if(CMAKE_CUDA_COMPILER)
    ament_add_gtest(test_gpu_frontend test/test_gpu_frontend.cpp)
    target_include_directories(test_gpu_frontend PRIVATE include ${PCL_INCLUDE_DIRS})
    ament_target_dependencies(test_gpu_frontend sensor_msgs)
    target_link_libraries(test_gpu_frontend ${PCL_LIBRARIES} ouster_cluster_gpu)
  endif()
endif()

ament_package()
