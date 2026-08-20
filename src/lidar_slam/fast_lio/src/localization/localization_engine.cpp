/**
 * @file localization_engine.cpp
 * @brief Algorithmic core of LocalizationNode (Open3D ICP + global localization).
 *
 * Splits the algorithm method bodies out of localization_node.cpp without
 * changing the class hierarchy — `LocalizationNode` remains a single class
 * declared in `localization_node.h`. Method bodies in this file may freely
 * reference `this->get_logger()`, `this->get_clock()`, etc. because they are
 * still member functions of the same class.
 */

#include "fast_lio/localization/localization_node.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <thread>

#include <tf2_eigen/tf2_eigen.hpp>

#include "fast_lio/localization/open3d_registration.h"
#include "fast_lio/localization/pose_utils.h"

namespace pose_utils = fast_lio::localization::pose_utils;

// ==================== Global Localization Helpers ====================

std::optional<std::pair<std::shared_ptr<open3d::geometry::PointCloud>, Eigen::Matrix4d>>
LocalizationNode::prepareScanForGlobalLocalization()
{
    std::lock_guard<std::mutex> lock_odom(odom_mutex_);
    std::lock_guard<std::mutex> lock_cloud(cloud_mutex_);

    if (pcd_scan_cur_->IsEmpty())
    {
        RCLCPP_WARN(this->get_logger(), "No scan data for global localization");
        return std::nullopt;
    }

    auto pcd_scan = std::make_shared<open3d::geometry::PointCloud>(*pcd_scan_cur_);
    Eigen::Matrix4d mat_body2odom_cur = mat_body2odom_;

    // Transform scan to odom frame
    pcd_scan->Transform(mat_body2odom_cur);

    return std::make_pair(pcd_scan, mat_body2odom_cur);
}

void LocalizationNode::ensureMapFpfhComputed(double voxel_size)
{
    if (!map_fpfh_computed_)
    {
        RCLCPP_INFO(this->get_logger(), "Computing FPFH features for map...");
        std::tie(pcd_map_global_, map_fpfh_) = pcd_tools::PreprocessPointCloudForFpfh(
            pcd_map_ori_, voxel_size);
        map_fpfh_computed_ = true;
        RCLCPP_INFO(this->get_logger(), "Map FPFH computed (%zu points)", pcd_map_global_->points_.size());
    }
}

std::pair<std::vector<Eigen::Vector3d>, Eigen::Matrix4d>
LocalizationNode::generateTranslationCandidates(
    const open3d::pipelines::registration::RegistrationResult& fpfh_result,
    std::shared_ptr<open3d::geometry::PointCloud> scan_down)
{
    std::vector<Eigen::Vector3d> translation_candidates;
    Eigen::Matrix4d fpfh_transform = fpfh_result.transformation_;

    // If FPFH completely failed (fitness < 0.02), try grid search around map centroid
    if (fpfh_result.fitness_ < 0.02)
    {
        RCLCPP_WARN(this->get_logger(), "FPFH failed, trying grid search around map centroid");
        Eigen::Vector3d map_centroid = pcd_map_global_->GetCenter();
        Eigen::Vector3d scan_centroid = scan_down->GetCenter();
        Eigen::Vector3d base_translation = map_centroid - scan_centroid;

        // Get map bounding box to determine grid step
        auto bbox = pcd_map_global_->GetAxisAlignedBoundingBox();
        double map_extent = std::max({
            bbox.max_bound_.x() - bbox.min_bound_.x(),
            bbox.max_bound_.y() - bbox.min_bound_.y()
        });
        double grid_step = map_extent / 4.0;  // Divide map into 4x4 regions

        // Create 3x3 grid around centroid
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                Eigen::Vector3d offset(dx * grid_step, dy * grid_step, 0.0);
                translation_candidates.push_back(base_translation + offset);
            }
        }
        fpfh_transform = Eigen::Matrix4d::Identity();
        RCLCPP_INFO(this->get_logger(), "Grid search: %zu positions, step=%.2fm",
                    translation_candidates.size(), grid_step);
    }
    else
    {
        // Use FPFH translation
        translation_candidates.push_back(fpfh_transform.block<3, 1>(0, 3));
    }

    return {translation_candidates, fpfh_transform};
}

LocalizationNode::HypothesisResult LocalizationNode::evaluateSingleHypothesis(
    std::shared_ptr<open3d::geometry::PointCloud> scan_down,
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_original,
    const Eigen::Vector3d& translation,
    double yaw_offset,
    const Eigen::Matrix3d& base_rotation,
    double voxel_size)
{
    // Create rotation matrix for yaw offset around Z-axis
    Eigen::Matrix3d R_offset;
    R_offset = Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ());

    // Apply rotation offset to base rotation
    Eigen::Matrix4d hypothesis_transform = Eigen::Matrix4d::Identity();
    hypothesis_transform.block<3, 3>(0, 0) = R_offset * base_rotation;
    hypothesis_transform.block<3, 1>(0, 3) = translation;

    // Transform scan with this hypothesis
    auto scan_hypothesis = std::make_shared<open3d::geometry::PointCloud>(*scan_down);
    scan_hypothesis->Transform(hypothesis_transform);

    // Refine with ICP
    auto icp_result = pcd_tools::RegistrationMultiScaleIcp(
        scan_hypothesis, pcd_map_global_, voxel_size, 1, {1, 2, 3});

    // Compute final transformation for this hypothesis
    Eigen::Matrix4d final_transform = icp_result * hypothesis_transform;

    // Evaluate result
    auto eval = open3d::pipelines::registration::EvaluateRegistration(
        *pcd_scan_original, *pcd_map_ori_, voxel_size * 2, final_transform);

    return {eval.fitness_, final_transform};
}

// ==================== Main Algorithm Loop ====================

void LocalizationNode::localizationLoop()
{
    RCLCPP_INFO(this->get_logger(), "Waiting for odometry...");
    while (running_ && rclcpp::ok() && latest_odom_time_.seconds() == 0.0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Odometry received");

    // Wait for sufficient point cloud accumulation for global localization
    const size_t min_points_for_global = 5000;
    RCLCPP_INFO(this->get_logger(), "Waiting for point cloud (min %zu points for global loc)...",
                min_points_for_global);
    while (running_ && rclcpp::ok())
    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        if (!pcd_scan_cur_->IsEmpty())
        {
            size_t num_points = pcd_scan_cur_->points_.size();
            if (num_points >= min_points_for_global || !use_global_localization_)
            {
                RCLCPP_INFO(this->get_logger(), "Point cloud ready: %zu points", num_points);
                break;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Accumulating points: %zu / %zu", num_points, min_points_for_global);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Point cloud received");

    // Initialize localization
    RCLCPP_INFO(this->get_logger(), "Starting localization initialization...");
    initializeLocalization();

    // Initialize Kalman filters with current position
    Eigen::Matrix4d init_body2map;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        init_body2map = mat_odom2map_ * mat_body2odom_;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing Kalman filters with position: x=%.3f, y=%.3f, z=%.3f",
                init_body2map(0, 3), init_body2map(1, 3), init_body2map(2, 3));

    kf_x_.init(kalman_params_x_[0], kalman_params_x_[1], init_body2map(0, 3), 1.0);
    kf_y_.init(kalman_params_y_[0], kalman_params_y_[1], init_body2map(1, 3), 1.0);
    kf_z_.init(kalman_params_z_[0], kalman_params_z_[1], init_body2map(2, 3), 1.0);

    RCLCPP_INFO(this->get_logger(), "Kalman filters: x[%.6f,%.6f], y[%.6f,%.6f], z[%.6f,%.6f]",
                kalman_params_x_[0], kalman_params_x_[1],
                kalman_params_y_[0], kalman_params_y_[1],
                kalman_params_z_[0], kalman_params_z_[1]);

    loc_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "Localization initialized with Kalman filtering");

    // Main localization loop
    auto last_loc_time = std::chrono::high_resolution_clock::now();
    double loc_interval_sec = 1.0 / loc_frequency_;

    while (running_ && rclcpp::ok())
    {
        // Honor /initialpose hint before doing tracking work this tick.
        // exchange(false) atomically reads-and-clears so a callback firing
        // while we're inside initializeLocalization() queues another re-init
        // for the next iteration instead of being lost.
        if (reinit_requested_.exchange(false))
        {
            RCLCPP_INFO(this->get_logger(),
                        "Re-initializing localization from /initialpose hint");
            loc_initialized_ = false;
            initializeLocalization();

            Eigen::Matrix4d init_body2map;
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                init_body2map = mat_odom2map_ * mat_body2odom_;
            }
            kf_x_.init(kalman_params_x_[0], kalman_params_x_[1], init_body2map(0, 3), 1.0);
            kf_y_.init(kalman_params_y_[0], kalman_params_y_[1], init_body2map(1, 3), 1.0);
            kf_z_.init(kalman_params_z_[0], kalman_params_z_[1], init_body2map(2, 3), 1.0);
            loc_initialized_ = true;

            last_loc_time = std::chrono::high_resolution_clock::now();
            continue;
        }

        auto now = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_loc_time).count();

        if (elapsed < loc_interval_sec)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        performLocalization();
        last_loc_time = std::chrono::high_resolution_clock::now();
    }
}

void LocalizationNode::initializeLocalization()
{
    // Try global localization first if enabled and no valid initial pose
    bool need_global = use_global_localization_ &&
                      (mat_initialpose_.block<3, 1>(0, 3).norm() < 0.1);

    if (need_global)
    {
        RCLCPP_INFO(this->get_logger(), "Attempting FPFH-based global localization...");

        const int max_global_attempts = 10;
        const double good_fitness_threshold = 0.8;  // Target fitness for global loc
        double best_global_fitness = 0.0;
        Eigen::Matrix4d best_global_transform = mat_odom2map_;

        for (int attempt = 1; attempt <= max_global_attempts && running_ && rclcpp::ok(); attempt++)
        {
            RCLCPP_INFO(this->get_logger(), "Global localization attempt %d/%d...", attempt, max_global_attempts);

            double fitness = performGlobalLocalizationWithFitness();

            if (fitness > best_global_fitness)
            {
                best_global_fitness = fitness;
                std::lock_guard<std::mutex> lock(odom_mutex_);
                best_global_transform = mat_odom2map_;
            }

            RCLCPP_INFO(this->get_logger(), "Attempt %d fitness: %.3f, best so far: %.3f",
                        attempt, fitness, best_global_fitness);

            if (best_global_fitness >= good_fitness_threshold)
            {
                RCLCPP_INFO(this->get_logger(), "Good fitness achieved (%.3f >= %.3f), stopping search",
                            best_global_fitness, good_fitness_threshold);
                break;
            }

            // Wait a bit for new scan data
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // Apply best result
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            mat_odom2map_ = best_global_transform;
        }

        if (best_global_fitness >= good_fitness_threshold)
        {
            RCLCPP_INFO(this->get_logger(), "Global localization: GOOD (fitness %.3f >= %.1f)", best_global_fitness, good_fitness_threshold);
        }
        else if (best_global_fitness >= fitness_threshold_global_)
        {
            RCLCPP_WARN(this->get_logger(), "Global localization: MARGINAL (fitness %.3f), may need manual adjustment", best_global_fitness);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Global localization: FAILED (fitness %.3f < %.3f), use RVIZ 2D Pose Estimate",
                         best_global_fitness, fitness_threshold_global_);
        }
    }

    // Fine-tune with ICP
    int success_count = 0;
    const int required_success = 2;
    int fail_count = 0;
    const int max_fails = 10;

    double final_init_fitness = 0.0;
    while (running_ && rclcpp::ok() && success_count < required_success && fail_count < max_fails)
    {
        double fitness = performLocalizationStep(true);
        final_init_fitness = fitness;

        if (fitness > fitness_threshold_)
        {
            success_count++;
            fail_count = 0;
            RCLCPP_INFO(this->get_logger(), "Init ICP pass %d/%d (fitness: %.3f)",
                        success_count, required_success, fitness);
        }
        else
        {
            success_count = 0;
            fail_count++;
            RCLCPP_WARN(this->get_logger(), "Init ICP failed %d/%d (fitness: %.3f < %.3f)",
                        fail_count, max_fails, fitness, fitness_threshold_);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (fail_count >= max_fails)
    {
        RCLCPP_ERROR(this->get_logger(), "INITIALIZATION FAILED after %d attempts. Use RVIZ 2D Pose Estimate.", max_fails);
    }
    else if (final_init_fitness >= 0.7)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZATION: EXCELLENT (fitness %.3f)", final_init_fitness);
    }
    else if (final_init_fitness >= 0.5)
    {
        RCLCPP_INFO(this->get_logger(), "INITIALIZATION: GOOD (fitness %.3f)", final_init_fitness);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "INITIALIZATION: MARGINAL (fitness %.3f) - tracking may be unstable", final_init_fitness);
    }
}

// Returns the best fitness achieved (0.0 if failed)
double LocalizationNode::performGlobalLocalizationWithFitness()
{
    // Step 1: Prepare scan for global localization
    auto scan_result = prepareScanForGlobalLocalization();
    if (!scan_result.has_value())
    {
        return 0.0;
    }
    auto [pcd_scan, mat_body2odom_cur] = scan_result.value();

    double effective_voxel_size = voxel_size_global_;

    // Preprocess scan for FPFH
    auto [scan_down, scan_fpfh] = pcd_tools::PreprocessPointCloudForFpfh(pcd_scan, effective_voxel_size);

    if (scan_down->points_.size() < 500)
    {
        RCLCPP_WARN(this->get_logger(), "Not enough scan points: %zu < 500", scan_down->points_.size());
        return 0.0;
    }

    // Step 2: Ensure map FPFH is computed
    ensureMapFpfhComputed(effective_voxel_size);

    // Perform FPFH + RANSAC global registration (RANSAC has randomness, so results vary)
    auto fpfh_result = pcd_tools::RegistrationFpfh(
        scan_down, pcd_map_global_, scan_fpfh, map_fpfh_, effective_voxel_size, true);

    RCLCPP_INFO(this->get_logger(), "FPFH result: fitness=%.3f, rmse=%.3f",
                fpfh_result.fitness_, fpfh_result.inlier_rmse_);

    // Step 3: Generate translation candidates
    auto [translation_candidates, fpfh_transform] = generateTranslationCandidates(
        fpfh_result, scan_down);

    // Multi-Hypothesis: Test rotation hypotheses (45 degree increments for indoor)
    std::vector<double> yaw_hypotheses = {
        0.0, M_PI_4, M_PI_2, 3*M_PI_4,
        M_PI, -3*M_PI_4, -M_PI_2, -M_PI_4
    };  // 0, 45, 90, 135, 180, -135, -90, -45 degrees

    double best_fitness = 0.0;
    Eigen::Matrix4d best_transform = Eigen::Matrix4d::Identity();
    int best_trans_idx = -1;
    int best_rot_idx = -1;

    size_t total_hypotheses = translation_candidates.size() * yaw_hypotheses.size();
    RCLCPP_INFO(this->get_logger(), "Testing %zu hypotheses (%zu translations x %zu rotations)...",
                total_hypotheses, translation_candidates.size(), yaw_hypotheses.size());

    // Step 4: Evaluate all hypotheses
    Eigen::Matrix3d base_rotation = fpfh_transform.block<3, 3>(0, 0);

    for (size_t t = 0; t < translation_candidates.size(); t++)
    {
        const Eigen::Vector3d& translation = translation_candidates[t];

        for (size_t r = 0; r < yaw_hypotheses.size(); r++)
        {
            double yaw_offset = yaw_hypotheses[r];

            HypothesisResult result = evaluateSingleHypothesis(
                scan_down, pcd_scan, translation, yaw_offset,
                base_rotation, effective_voxel_size);

            // Only log if it's a good candidate or every 8th hypothesis
            if (result.fitness > 0.1 || (t * yaw_hypotheses.size() + r) % 8 == 0)
            {
                RCLCPP_INFO(this->get_logger(), "  Trans[%zu] Rot[%zu](%.0f°): fitness=%.3f",
                            t, r, yaw_offset * 180.0 / M_PI, result.fitness);
            }

            if (result.fitness > best_fitness)
            {
                best_fitness = result.fitness;
                best_transform = result.transform;
                best_trans_idx = static_cast<int>(t);
                best_rot_idx = static_cast<int>(r);
            }
        }
    }

    int best_hypothesis = best_rot_idx;

    if (best_hypothesis >= 0)
    {
        RCLCPP_INFO(this->get_logger(), "Best hypothesis: trans[%d] rot[%d](yaw=%.0f°) fitness=%.3f",
                    best_trans_idx, best_rot_idx,
                    yaw_hypotheses[best_hypothesis] * 180.0 / M_PI, best_fitness);

        // Always apply best transform (caller will decide if it's good enough)
        std::lock_guard<std::mutex> lock(odom_mutex_);
        mat_odom2map_ = best_transform;
    }

    return best_fitness;
}

void LocalizationNode::performLocalization()
{
    double fitness = performLocalizationStep(false);
    current_fitness_ = fitness;

    if (fitness < 0.3)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                              "LOCALIZATION LOST (fitness: %.3f) - consider re-initializing", fitness);
    }
    else if (fitness < fitness_threshold_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Low confidence (fitness: %.3f < %.3f)", fitness, fitness_threshold_);
    }
}

double LocalizationNode::performLocalizationStep(bool is_init)
{
    // Get current state
    Eigen::Matrix4d mat_body2odom_cur;
    Eigen::Matrix4d mat_body2map_cur;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan;

    {
        std::lock_guard<std::mutex> lock_odom(odom_mutex_);
        std::lock_guard<std::mutex> lock_cloud(cloud_mutex_);

        if (pcd_scan_cur_->IsEmpty())
        {
            return 0.0;
        }

        mat_body2odom_cur = mat_body2odom_;
        mat_body2map_cur = mat_odom2map_ * mat_body2odom_;
        pcd_scan = std::make_shared<open3d::geometry::PointCloud>(*pcd_scan_cur_);
    }

    // Current position in map frame
    Eigen::Vector3d cur_position = mat_body2map_cur.block<3, 1>(0, 3);

    // Distance-based submap update: only recompute when moved enough
    double motion_distance = (cur_position - last_submap_position_).norm();
    if (motion_distance > dis_updatemap_ || !pcd_submap_cached_)
    {
        RCLCPP_DEBUG(this->get_logger(), "Updating submap (moved %.2fm > %.2fm threshold)",
                     motion_distance, dis_updatemap_);

        auto OBB_map = pose_utils::createCropBox(
            cur_position,
            mat_body2map_cur.block<3, 3>(0, 0),
            Eigen::Vector3d(submap_radius_ * 2, submap_radius_ * 2, submap_radius_));

        pcd_submap_cached_ = pcd_map_fine_->Crop(*OBB_map);
        last_submap_position_ = cur_position;
    }

    if (!pcd_submap_cached_ || pcd_submap_cached_->IsEmpty())
    {
        RCLCPP_WARN(this->get_logger(), "No map points in current area");
        return 0.0;
    }

    auto map_crop = pcd_submap_cached_;

    // Crop scan
    auto OBB_scan = pose_utils::createCropBox(
        mat_body2odom_cur.block<3, 1>(0, 3),
        mat_body2odom_cur.block<3, 3>(0, 0),
        Eigen::Vector3d(submap_radius_ * 2, submap_radius_ * 2, submap_radius_));

    auto scan_crop = pcd_scan->Crop(*OBB_scan);
    if (scan_crop->IsEmpty())
    {
        RCLCPP_WARN(this->get_logger(), "No scan points");
        return 0.0;
    }

    // Prepare source (scan) and target (map)
    auto source = scan_crop->VoxelDownSample(voxel_size_fine_);
    auto target = map_crop;

    // Limit points
    source = pose_utils::limitPointCloud(source, static_cast<size_t>(max_points_source_));
    target = pose_utils::limitPointCloud(target, static_cast<size_t>(max_points_target_));

    // Save current odom2map for update calculation
    Eigen::Matrix4d mat_odom2map_cur;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        mat_odom2map_cur = mat_odom2map_;
    }

    // Transform source from body frame to map frame using current estimate
    // body → map = odom2map * body2odom
    Eigen::Matrix4d mat_body2map_init = mat_odom2map_cur * mat_body2odom_cur;
    source->Transform(mat_body2map_init);

    // Perform ICP
    std::vector<double> scales = is_init ? std::vector<double>{1, 2, 3} : std::vector<double>{1, 2};
    Eigen::Matrix4d icp_result = pcd_tools::RegistrationMultiScaleIcp(
        source, target, voxel_size_fine_, 1, scales);

    // Evaluate result
    source->Transform(icp_result);
    auto eval_result = open3d::pipelines::registration::EvaluateRegistration(
        *source, *target, voxel_size_fine_ * 3);

    double fitness = eval_result.fitness_;

    // Update odom2map if fitness is good
    // new_odom2map = icp_result * old_odom2map
    if (fitness > fitness_threshold_)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        mat_odom2map_ = icp_result * mat_odom2map_cur;
    }

    // Save scan for debugging if enabled
    if (save_scan_)
    {
        try
        {
            std::filesystem::create_directories(save_scan_path_);
            std::string filename = save_scan_path_ + std::to_string(scan_save_count_) + "_scan.ply";
            open3d::io::WritePointCloud(filename, *source);
            scan_save_count_++;
            RCLCPP_DEBUG(this->get_logger(), "Saved scan to: %s", filename.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Failed to save scan: %s", e.what());
        }
    }

    return fitness;
}
