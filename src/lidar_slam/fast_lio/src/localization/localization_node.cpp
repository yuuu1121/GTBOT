/**
 * @file localization_node.cpp
 * @brief Map-based localization using Open3D ICP registration (ROS2 plumbing).
 *
 * Subscribes to FAST-LIO odometry and point clouds, performs ICP-based
 * localization against a pre-built map, and publishes the map->odom transform.
 *
 * Algorithm method bodies (initializeLocalization, performLocalizationStep,
 * performGlobalLocalizationWithFitness, localizationLoop, ...) live in
 * localization_engine.cpp. This file contains the constructor, ROS2 callbacks,
 * and TF/map publishing only.
 *
 * Features:
 * - Multi-scale ICP registration
 * - FPFH + RANSAC global localization with multi-hypothesis
 * - Kalman filtering for pose smoothing (X, Y, Z axes)
 * - Distance-based submap update for efficiency
 */

#include "fast_lio/localization/localization_node.h"

#include <chrono>
#include <cmath>
#include <memory>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "fast_lio/localization/open3d_conversions.h"
#include "fast_lio/localization/pose_utils.h"
#include "fast_lio/localization/tf_publisher.h"
#include "fast_lio/qos.hpp"

namespace pose_utils = fast_lio::localization::pose_utils;

LocalizationNode::LocalizationNode()
    : Node("localization_node")
{
    // Initialize matrices
    mat_body2odom_ = Eigen::Matrix4d::Identity();
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_initialpose_ = Eigen::Matrix4d::Identity();
    last_loc_position_ = Eigen::Vector3d(0, 0, -5000);

    // Initialize point clouds
    pcd_map_ori_ = std::make_shared<open3d::geometry::PointCloud>();
    pcd_map_fine_ = std::make_shared<open3d::geometry::PointCloud>();
    pcd_scan_cur_ = std::make_shared<open3d::geometry::PointCloud>();

    // Declare parameters
    this->declare_parameter<std::string>("map_path", "");
    this->declare_parameter<double>("voxel_size_fine", 0.05);
    this->declare_parameter<double>("loc_frequency", 2.0);
    this->declare_parameter<double>("fitness_threshold", 0.5);
    this->declare_parameter<double>("submap_radius", 50.0);
    this->declare_parameter<int>("max_points_source", 100000);
    this->declare_parameter<int>("max_points_target", 500000);
    this->declare_parameter<std::vector<double>>("initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});

    // Global localization parameters
    this->declare_parameter<bool>("use_global_localization", true);
    this->declare_parameter<double>("voxel_size_global", 0.25);
    this->declare_parameter<double>("fitness_threshold_global", 0.15);

    // Kalman filter parameters [process_var, measurement_var]
    this->declare_parameter<std::vector<double>>("kalman_x", std::vector<double>{0.001, 0.002});
    this->declare_parameter<std::vector<double>>("kalman_y", std::vector<double>{0.001, 0.005});
    this->declare_parameter<std::vector<double>>("kalman_z", std::vector<double>{0.00001, 0.04});

    // Distance-based submap update
    this->declare_parameter<double>("dis_updatemap", 3.0);

    // Point cloud queue size
    this->declare_parameter<int>("pcd_queue_maxsize", 5);

    // Debug options
    this->declare_parameter<bool>("save_scan", false);
    this->declare_parameter<std::string>("save_scan_path", "/tmp/localization_scans/");

    // OccupancyGrid parameters
    this->declare_parameter<bool>("occupancy_grid.publish", true);
    this->declare_parameter<double>("occupancy_grid.resolution", 0.05);
    this->declare_parameter<double>("occupancy_grid.z_min", -0.5);
    this->declare_parameter<double>("occupancy_grid.z_max", 1.5);

    // TF frame names
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("body_frame", "base_link");

    // Get parameters
    std::string map_path;
    this->get_parameter("map_path", map_path);
    this->get_parameter("voxel_size_fine", voxel_size_fine_);
    this->get_parameter("loc_frequency", loc_frequency_);
    this->get_parameter("fitness_threshold", fitness_threshold_);
    this->get_parameter("submap_radius", submap_radius_);
    this->get_parameter("max_points_source", max_points_source_);
    this->get_parameter("max_points_target", max_points_target_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("body_frame", body_frame_);
    this->get_parameter("use_global_localization", use_global_localization_);
    this->get_parameter("voxel_size_global", voxel_size_global_);
    this->get_parameter("fitness_threshold_global", fitness_threshold_global_);

    // Kalman filter parameters
    std::vector<double> kf_x, kf_y, kf_z;
    this->get_parameter("kalman_x", kf_x);
    this->get_parameter("kalman_y", kf_y);
    this->get_parameter("kalman_z", kf_z);
    if (kf_x.size() >= 2) kalman_params_x_ = kf_x;
    if (kf_y.size() >= 2) kalman_params_y_ = kf_y;
    if (kf_z.size() >= 2) kalman_params_z_ = kf_z;

    // Distance-based submap update
    this->get_parameter("dis_updatemap", dis_updatemap_);

    // Point cloud queue size
    this->get_parameter("pcd_queue_maxsize", queue_max_size_);

    // Debug options
    this->get_parameter("save_scan", save_scan_);
    this->get_parameter("save_scan_path", save_scan_path_);

    // OccupancyGrid parameters
    fast_lio::localization::TfPublisherConfig tf_config;
    tf_config.map_frame = map_frame_;
    tf_config.odom_frame = odom_frame_;
    this->get_parameter("occupancy_grid.publish", tf_config.occupancy_grid_publish);
    this->get_parameter("occupancy_grid.resolution", tf_config.occupancy_grid_resolution);
    this->get_parameter("occupancy_grid.z_min", tf_config.occupancy_grid_z_min);
    this->get_parameter("occupancy_grid.z_max", tf_config.occupancy_grid_z_max);

    std::vector<double> initial_pose;
    this->get_parameter("initial_pose", initial_pose);
    if (initial_pose.size() >= 6)
    {
        setInitialPose(initial_pose[0], initial_pose[1], initial_pose[2],
                       initial_pose[3], initial_pose[4], initial_pose[5]);
    }

    // Load map
    if (map_path.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "map_path parameter is empty!");
        rclcpp::shutdown();
        return;
    }

    // Auto-append .pcd extension if missing
    if (map_path.size() < 4 || map_path.substr(map_path.size() - 4) != ".pcd") {
        map_path += ".pcd";
    }

    RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_path.c_str());
    if (!open3d::io::ReadPointCloud(map_path, *pcd_map_ori_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map from: %s", map_path.c_str());
        rclcpp::shutdown();
        return;
    }

    if (pcd_map_ori_->IsEmpty())
    {
        RCLCPP_ERROR(this->get_logger(), "Map is empty!");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Map loaded with %zu points", pcd_map_ori_->points_.size());

    // Downsample and compute normals
    pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxel_size_fine_);
    pcd_map_fine_->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(voxel_size_fine_ * 2, 30));
    RCLCPP_INFO(this->get_logger(), "Map downsampled to %zu points", pcd_map_fine_->points_.size());

    // Create publishers
    pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/slam/fast_lio_loc/odometry", pkrc_qos::reliable_qos());
    pub_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/slam/fast_lio_loc/confidence", pkrc_qos::reliable_qos());

    // Create subscribers
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/slam/fast_lio/odometry", pkrc_qos::reliable_qos(),
        std::bind(&LocalizationNode::odomCallback, this, std::placeholders::_1));

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/slam/fast_lio/points_body", pkrc_qos::sensor_qos(),
        std::bind(&LocalizationNode::cloudCallback, this, std::placeholders::_1));

    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", pkrc_qos::reliable_qos(),
        std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    // TF / map / occupancy grid publisher
    tf_publisher_ = std::make_unique<fast_lio::localization::TfPublisher>(
        this, tf_config, mat_odom2map_, odom_mutex_);
    tf_publisher_->publishMap(*pcd_map_ori_);
    tf_publisher_->publishOccupancyGrid(*pcd_map_ori_);
    tf_publisher_->start();

    // Start localization thread
    loc_thread_ = std::thread(&LocalizationNode::localizationLoop, this);

    RCLCPP_INFO(this->get_logger(), "Localization node initialized");
}

LocalizationNode::~LocalizationNode()
{
    running_ = false;
    if (loc_thread_.joinable())
    {
        loc_thread_.join();
    }
}

// ==================== Setup helpers ====================

void LocalizationNode::setInitialPose(double x, double y, double z, double roll, double pitch, double yaw)
{
    // Convert RPY (degrees) to rotation matrix
    Eigen::AngleAxisd rollAngle(roll * M_PI / 180.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    Eigen::Matrix3d rotation = (yawAngle * pitchAngle * rollAngle).matrix();

    mat_initialpose_.block<3, 3>(0, 0) = rotation;
    mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
    mat_odom2map_ = mat_initialpose_;

    RCLCPP_INFO(this->get_logger(), "Initial pose set: [%.2f, %.2f, %.2f] RPY: [%.1f, %.1f, %.1f]",
                x, y, z, roll, pitch, yaw);
}

// ==================== ROS callbacks ====================

void LocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_time_ = msg->header.stamp;

    // Convert pose to matrix
    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    tf2::fromMsg(msg->pose.pose, isometry);
    mat_body2odom_ = isometry.matrix();

    // Compute body to map
    Eigen::Matrix4d mat_body2map = mat_odom2map_ * mat_body2odom_;

    // TF is now published by tfTimerCallback at 50Hz

    // Publish odometry with Kalman filtering
    if (loc_initialized_)
    {
        // Apply Kalman filtering to body2map position
        Eigen::Matrix4d mat_body2map_kalman = mat_body2map;

        kf_x_.update(mat_body2map(0, 3));
        kf_y_.update(mat_body2map(1, 3));
        kf_z_.update(mat_body2map(2, 3));

        double filtered_x = kf_x_.getEstimate();
        double filtered_y = kf_y_.getEstimate();
        double filtered_z = kf_z_.getEstimate();

        // NaN protection
        if (!std::isnan(filtered_x)) mat_body2map_kalman(0, 3) = filtered_x;
        if (!std::isnan(filtered_y)) mat_body2map_kalman(1, 3) = filtered_y;
        if (!std::isnan(filtered_z)) mat_body2map_kalman(2, 3) = filtered_z;

        // Publish Kalman-filtered odometry
        pub_odometry_->publish(
            pose_utils::createOdometryMsg(msg->header.stamp, map_frame_, body_frame_, mat_body2map_kalman));

        // Publish confidence
        std_msgs::msg::Float32 conf_msg;
        conf_msg.data = current_fitness_;
        pub_confidence_->publish(conf_msg);
    }
}

void LocalizationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // Convert to Open3D
    open3d::geometry::PointCloud pcd_received;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr const_msg = msg;
    open3d_conversions::rosToOpen3d(const_msg, pcd_received, true);

    // Add to queue
    scan_queue_.push(pcd_received);
    if (scan_queue_.size() > static_cast<size_t>(queue_max_size_))
    {
        scan_queue_.pop();
    }

    // Merge queue into current scan
    pcd_scan_cur_->Clear();
    std::queue<open3d::geometry::PointCloud> temp_queue = scan_queue_;
    while (!temp_queue.empty())
    {
        *pcd_scan_cur_ += temp_queue.front();
        temp_queue.pop();
    }
}

void LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received initial pose from RVIZ");

    // Extract pose
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    std::lock_guard<std::mutex> lock(odom_mutex_);
    mat_initialpose_.block<3, 3>(0, 0) = q.matrix();
    mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    mat_odom2map_ = mat_initialpose_;
    // Signal localizationLoop to re-run initializeLocalization() at the top
    // of its next iteration. Do NOT touch loc_initialized_ here — the loop
    // owns that state and will reset it during the re-init handshake.
    reinit_requested_.store(true);
    RCLCPP_INFO(this->get_logger(),
                "Re-initialization requested by /initialpose hint at [%.2f, %.2f, %.2f]",
                msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
}

// ==================== main ====================

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
