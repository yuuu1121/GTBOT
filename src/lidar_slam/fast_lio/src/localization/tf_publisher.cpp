/**
 * @file tf_publisher.cpp
 * @brief TF + map + occupancy grid publisher for LocalizationNode.
 */

#include "fast_lio/localization/tf_publisher.h"

#include <chrono>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "fast_lio/localization/occupancy_grid_generator.hpp"
#include "fast_lio/localization/open3d_conversions.h"
#include "fast_lio/qos.hpp"

namespace fast_lio {
namespace localization {

TfPublisher::TfPublisher(rclcpp::Node* node,
                         const TfPublisherConfig& config,
                         const Eigen::Matrix4d& mat_odom2map,
                         std::mutex& odom_mutex)
    : node_(node),
      config_(config),
      mat_odom2map_(mat_odom2map),
      odom_mutex_(odom_mutex)
{
    auto map_qos = pkrc_qos::latched_qos();

    pub_map_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/slam/fast_lio_loc/map", map_qos);

    if (config_.occupancy_grid_publish)
    {
        pub_occupancy_grid_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/slam/fast_lio_loc/occupancy_grid", map_qos);
    }

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void TfPublisher::publishMap(const open3d::geometry::PointCloud& map)
{
    sensor_msgs::msg::PointCloud2 map_msg;
    auto map_coarse = map.VoxelDownSample(0.5);  // Coarse for visualization
    open3d_conversions::open3dToRos(*map_coarse, map_msg, config_.map_frame);
    map_msg.header.stamp = node_->now();
    pub_map_->publish(map_msg);
}

void TfPublisher::publishOccupancyGrid(const open3d::geometry::PointCloud& map)
{
    if (!config_.occupancy_grid_publish || !pub_occupancy_grid_)
    {
        return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "Generating OccupancyGrid from map (z: [%.2f, %.2f], res: %.3f)...",
                config_.occupancy_grid_z_min,
                config_.occupancy_grid_z_max,
                config_.occupancy_grid_resolution);

    occupancy_grid::OccupancyGridParams og_params;
    og_params.resolution = config_.occupancy_grid_resolution;
    og_params.z_min = config_.occupancy_grid_z_min;
    og_params.z_max = config_.occupancy_grid_z_max;
    og_params.frame_id = config_.map_frame;

    occupancy_grid::OccupancyGridGenerator generator(og_params);
    auto grid = generator.generate(map, node_->now());

    if (grid)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "OccupancyGrid generated: %u x %u cells (%.1f x %.1f m)",
                    grid->info.width, grid->info.height,
                    grid->info.width * config_.occupancy_grid_resolution,
                    grid->info.height * config_.occupancy_grid_resolution);
        pub_occupancy_grid_->publish(std::move(*grid));
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to generate OccupancyGrid (no points in Z range?)");
    }
}

void TfPublisher::start()
{
    publishTF(node_->now());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(config_.tf_period_seconds));
    // Use ROS clock (honors use_sim_time) instead of wall clock so the TF
    // timer fires in lockstep with bag replay; otherwise tf2 lookups during
    // sim-time replay observe stamps that diverge from bag clock.
    tf_timer_ = rclcpp::create_timer(
        node_, node_->get_clock(), rclcpp::Duration(period),
        std::bind(&TfPublisher::tfTimerCallback, this));
}

void TfPublisher::tfTimerCallback()
{
    publishTF(node_->now());
}

void TfPublisher::publishTF(const rclcpp::Time& stamp)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = config_.map_frame;
    tf_msg.child_frame_id = config_.odom_frame;
    tf_msg.transform.translation.x = mat_odom2map_(0, 3);
    tf_msg.transform.translation.y = mat_odom2map_(1, 3);
    tf_msg.transform.translation.z = mat_odom2map_(2, 3);
    Eigen::Quaterniond q(mat_odom2map_.block<3, 3>(0, 0));
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace localization
}  // namespace fast_lio
