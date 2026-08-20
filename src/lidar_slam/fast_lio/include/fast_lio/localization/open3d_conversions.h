// Based on Open3D Conversions by Autonomous Robots Lab, University of Nevada, Reno
// Licensed under Apache License 2.0

#ifndef OPEN3D_CONVERSIONS_HPP_
#define OPEN3D_CONVERSIONS_HPP_

#include <open3d/Open3D.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace open3d_conversions
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = std::shared_ptr<const PointCloud2>;

/// @brief Convert Open3D PointCloud to ROS2 PointCloud2
/// @param pointcloud Open3D point cloud
/// @param ros_pc2 Output ROS2 message
/// @param frame_id Frame ID for the message
void open3dToRos(
    const open3d::geometry::PointCloud& pointcloud,
    PointCloud2& ros_pc2,
    std::string frame_id = "open3d_pointcloud");

/// @brief Convert ROS2 PointCloud2 to Open3D PointCloud
/// @param ros_pc2 ROS2 point cloud message
/// @param o3d_pc Output Open3D point cloud
/// @param skip_colors If true, only xyz fields will be copied
void rosToOpen3d(
    const PointCloud2ConstPtr& ros_pc2,
    open3d::geometry::PointCloud& o3d_pc,
    bool skip_colors = false);

}  // namespace open3d_conversions

#endif  // OPEN3D_CONVERSIONS_HPP_
