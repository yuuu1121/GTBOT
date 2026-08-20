#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>
#include <open3d/Open3D.h>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace fast_lio {
namespace localization {

struct TfPublisherConfig
{
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    bool occupancy_grid_publish = true;
    double occupancy_grid_resolution = 0.05;
    double occupancy_grid_z_min = -0.5;
    double occupancy_grid_z_max = 1.5;
    double tf_period_seconds = 0.02;  // 50 Hz
};

/**
 * @brief Publishes map -> odom TF, downsampled map cloud, and occupancy grid.
 *
 * Reads `mat_odom2map` by const reference (owner must outlive this object)
 * and protects access with `odom_mutex` provided by the owner. Spawns a
 * wall-clock timer (period from config) to publish TF at fixed rate.
 */
class TfPublisher
{
public:
    TfPublisher(rclcpp::Node* node,
                const TfPublisherConfig& config,
                const Eigen::Matrix4d& mat_odom2map,
                std::mutex& odom_mutex);

    void publishMap(const open3d::geometry::PointCloud& map);
    void publishOccupancyGrid(const open3d::geometry::PointCloud& map);

    /// Publishes initial TF and starts the periodic timer.
    void start();

    /// Publish current TF immediately (e.g. one-shot from constructor).
    void publishTF(const rclcpp::Time& stamp);

private:
    void tfTimerCallback();

    rclcpp::Node* node_;
    TfPublisherConfig config_;
    const Eigen::Matrix4d& mat_odom2map_;
    std::mutex& odom_mutex_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
};

}  // namespace localization
}  // namespace fast_lio
