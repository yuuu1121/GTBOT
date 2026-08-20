#pragma once

#include <iostream>
#include <thread>
#include <vector>
#include <chrono>

#include <open3d/Open3D.h>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace pcd_tools
{

/// @brief Compute FPFH features for a point cloud
/// @param pcd Point cloud
/// @param voxel_size Voxel size for feature computation
/// @return Tuple of (downsampled cloud with normals, FPFH features)
std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
           std::shared_ptr<open3d::pipelines::registration::Feature>>
PreprocessPointCloudForFpfh(
    std::shared_ptr<open3d::geometry::PointCloud> pcd,
    double voxel_size);

/// @brief FPFH + RANSAC based global registration
/// @param source Source point cloud
/// @param target Target point cloud
/// @param source_fpfh Source FPFH features
/// @param target_fpfh Target FPFH features
/// @param voxel_size Voxel size
/// @param mutual_filter Use mutual filter for correspondences
/// @return Registration result
open3d::pipelines::registration::RegistrationResult RegistrationFpfh(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh,
    std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh,
    float voxel_size,
    bool mutual_filter = true);

/// @brief Point-to-plane ICP registration
/// @param source Source point cloud
/// @param target Target point cloud (must have normals)
/// @param icp_distance_threshold Max correspondence distance
/// @param init_matrix Initial transformation guess
/// @param icp_method 0: point2point, 1: point2plane, 2: generalizedICP
/// @param icp_iteration Max iterations
/// @return Registration result
open3d::pipelines::registration::RegistrationResult RegistrationIcp(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    float icp_distance_threshold,
    Eigen::Matrix4d init_matrix = Eigen::Matrix4d::Identity(),
    int icp_method = 1,
    int icp_iteration = 30);

/// @brief Multi-scale ICP registration for coarse-to-fine alignment
/// @param source Source point cloud
/// @param target Target point cloud
/// @param voxel_size Base voxel size for downsampling
/// @param icp_method 0: point2point, 1: point2plane, 2: generalizedICP
/// @param scale Scale factors for multi-scale (largest to smallest)
/// @return Final transformation matrix
Eigen::Matrix4d RegistrationMultiScaleIcp(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    double voxel_size,
    int icp_method = 1,
    std::vector<double> scale = {1.5});

/// @brief Evaluate registration quality
/// @param src Source point cloud
/// @param tgt Target point cloud
/// @param voxel_size Voxel size for downsampling
/// @param transformation Transformation to evaluate
/// @return Registration result with fitness and RMSE
open3d::pipelines::registration::RegistrationResult RegistrationEvaluate(
    const std::shared_ptr<open3d::geometry::PointCloud> src,
    const std::shared_ptr<open3d::geometry::PointCloud> tgt,
    double voxel_size,
    Eigen::Matrix4d transformation);

}  // namespace pcd_tools
