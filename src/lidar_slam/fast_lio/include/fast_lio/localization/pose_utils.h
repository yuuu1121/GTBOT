#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <open3d/Open3D.h>
#include <rclcpp/time.hpp>

#include <memory>
#include <string>

namespace fast_lio::localization::pose_utils {

/**
 * @brief Convert a 4x4 transformation matrix to geometry_msgs::msg::Pose.
 */
geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& mat);

/**
 * @brief Build an Odometry message from a transform matrix.
 */
nav_msgs::msg::Odometry createOdometryMsg(
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const Eigen::Matrix4d& transform);

/**
 * @brief Construct an OrientedBoundingBox for point cloud cropping.
 */
std::shared_ptr<open3d::geometry::OrientedBoundingBox> createCropBox(
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& extent);

/**
 * @brief Limit a point cloud to a maximum number of points via random downsampling.
 *
 * The input cloud is returned unchanged if it already has at most max_points.
 */
std::shared_ptr<open3d::geometry::PointCloud> limitPointCloud(
    std::shared_ptr<open3d::geometry::PointCloud> cloud,
    std::size_t max_points);

}  // namespace fast_lio::localization::pose_utils
