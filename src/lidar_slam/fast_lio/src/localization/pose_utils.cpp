#include "fast_lio/localization/pose_utils.h"

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fast_lio::localization::pose_utils {

geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4d& mat)
{
    Eigen::Isometry3d iso;
    iso.matrix() = mat;
    return tf2::toMsg(iso);
}

nav_msgs::msg::Odometry createOdometryMsg(
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const Eigen::Matrix4d& transform)
{
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
    msg.pose.pose = matrixToPose(transform);
    return msg;
}

std::shared_ptr<open3d::geometry::OrientedBoundingBox> createCropBox(
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& extent)
{
    auto obb = std::make_shared<open3d::geometry::OrientedBoundingBox>();
    obb->center_ = center;
    obb->R_ = rotation;
    obb->extent_ = extent;
    return obb;
}

std::shared_ptr<open3d::geometry::PointCloud> limitPointCloud(
    std::shared_ptr<open3d::geometry::PointCloud> cloud,
    std::size_t max_points)
{
    if (cloud->points_.size() > max_points)
    {
        const double ratio = static_cast<double>(max_points) / cloud->points_.size();
        return cloud->RandomDownSample(ratio);
    }
    return cloud;
}

}  // namespace fast_lio::localization::pose_utils
