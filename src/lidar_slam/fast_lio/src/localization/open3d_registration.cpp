#include "fast_lio/localization/open3d_registration.h"

namespace pcd_tools
{

std::tuple<std::shared_ptr<open3d::geometry::PointCloud>,
           std::shared_ptr<open3d::pipelines::registration::Feature>>
PreprocessPointCloudForFpfh(
    std::shared_ptr<open3d::geometry::PointCloud> pcd,
    double voxel_size)
{
    // Downsample
    auto pcd_down = pcd->VoxelDownSample(voxel_size);

    // Estimate normals
    pcd_down->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 2, 30));

    // Orient normals consistently
    pcd_down->OrientNormalsToAlignWithDirection();

    // Compute FPFH features
    auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(
        *pcd_down,
        open3d::geometry::KDTreeSearchParamHybrid(voxel_size * 5, 100));

    return std::make_tuple(pcd_down, pcd_fpfh);
}

open3d::pipelines::registration::RegistrationResult RegistrationFpfh(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh,
    std::shared_ptr<open3d::pipelines::registration::Feature> target_fpfh,
    float voxel_size,
    bool mutual_filter)
{
    float distance_threshold = 1.5 * voxel_size;

    // Prepare correspondence checkers
    std::vector<std::reference_wrapper<
        const open3d::pipelines::registration::CorrespondenceChecker>>
        correspondence_checker;

    auto correspondence_checker_edge_length =
        open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto correspondence_checker_distance =
        open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);

    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);

    // RANSAC registration with increased iterations for global search
    auto registration_result = open3d::pipelines::registration::
        RegistrationRANSACBasedOnFeatureMatching(
            *source, *target, *source_fpfh, *target_fpfh,
            mutual_filter, distance_threshold,
            open3d::pipelines::registration::TransformationEstimationPointToPoint(false),
            3,  // RANSAC n (minimum correspondences for transform estimation)
            correspondence_checker,
            open3d::pipelines::registration::RANSACConvergenceCriteria(4000000, 0.9999));

    return registration_result;
}

open3d::pipelines::registration::RegistrationResult RegistrationIcp(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    float icp_distance_threshold,
    Eigen::Matrix4d init_matrix,
    int icp_method,
    int icp_iteration)
{
    std::shared_ptr<open3d::geometry::PointCloud> source_transformed(new open3d::geometry::PointCloud);
    *source_transformed = *source;
    source_transformed->Transform(init_matrix);

    auto criteria_icp = open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, icp_iteration);
    open3d::pipelines::registration::RegistrationResult registration_result;

    if (icp_method == 0)
    {
        registration_result = open3d::pipelines::registration::RegistrationICP(
            *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPoint(), criteria_icp);
    }
    else if (icp_method == 1)
    {
        registration_result = open3d::pipelines::registration::RegistrationICP(
            *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPlane(), criteria_icp);
    }
    else if (icp_method == 2)
    {
        registration_result = open3d::pipelines::registration::RegistrationGeneralizedICP(
            *source_transformed, *target, icp_distance_threshold, Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(), criteria_icp);
    }
    return registration_result;
}

Eigen::Matrix4d RegistrationMultiScaleIcp(
    std::shared_ptr<open3d::geometry::PointCloud> source,
    std::shared_ptr<open3d::geometry::PointCloud> target,
    double voxel_size,
    int icp_method,
    std::vector<double> scale)
{
    struct PcdPair
    {
        std::shared_ptr<open3d::geometry::PointCloud> pcd_src;
        std::shared_ptr<open3d::geometry::PointCloud> pcd_tgt;
        double voxel_size_;
        double icp_threshold_;
    };

    std::vector<PcdPair> vec_pcd_pair;

    for (std::size_t scale_i = 0; scale_i < scale.size(); ++scale_i)
    {
        PcdPair pair_;
        pair_.voxel_size_ = voxel_size * scale[scale_i];
        pair_.icp_threshold_ = pair_.voxel_size_ * 1.3;
        vec_pcd_pair.push_back(pair_);
    }

    int num_pair = vec_pcd_pair.size();

    // Preprocess point clouds at each scale (can be parallelized)
    auto pcd_preprocess = [&](int pcd_i)
    {
        double vs = vec_pcd_pair[pcd_i].voxel_size_;
        vec_pcd_pair[pcd_i].pcd_src = source->VoxelDownSample(vs);
        vec_pcd_pair[pcd_i].pcd_tgt = target->VoxelDownSample(vs);
        vec_pcd_pair[pcd_i].pcd_tgt->EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(vs * 2, 30));
    };

    std::vector<std::thread> thread_preprocess;
    for (int i = 0; i < num_pair; ++i)
    {
        thread_preprocess.push_back(std::thread(pcd_preprocess, i));
    }
    for (auto& t : thread_preprocess)
    {
        t.join();
    }

    // Multi-scale ICP: coarse to fine
    open3d::pipelines::registration::RegistrationResult registration_result;
    Eigen::Matrix4d matrix_icp = Eigen::Matrix4d::Identity();

    for (int icp_i = 0; icp_i < num_pair; ++icp_i)
    {
        auto src = vec_pcd_pair[num_pair - 1 - icp_i].pcd_src;
        auto tgt = vec_pcd_pair[num_pair - 1 - icp_i].pcd_tgt;
        double icp_threshold_current = vec_pcd_pair[num_pair - 1 - icp_i].icp_threshold_;
        registration_result = RegistrationIcp(src, tgt, icp_threshold_current, matrix_icp, icp_method, 30);
        matrix_icp = registration_result.transformation_ * matrix_icp;
    }

    return matrix_icp;
}

open3d::pipelines::registration::RegistrationResult RegistrationEvaluate(
    const std::shared_ptr<open3d::geometry::PointCloud> src,
    const std::shared_ptr<open3d::geometry::PointCloud> tgt,
    double voxel_size,
    Eigen::Matrix4d transformation)
{
    auto src_down = src->VoxelDownSample(voxel_size);
    auto tgt_down = tgt->VoxelDownSample(voxel_size);
    return open3d::pipelines::registration::EvaluateRegistration(
        *src_down, *tgt_down, voxel_size * 1.5, transformation);
}

}  // namespace pcd_tools
