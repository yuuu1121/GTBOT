// Pure, ROS-independent core types + Mahalanobis cluster trim for
// ouster_cluster. Kept header-only so both the node and the gtest link the
// same code.
#ifndef OUSTER_CLUSTER__CLUSTER_LOGIC_HPP_
#define OUSTER_CLUSTER__CLUSTER_LOGIC_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>

namespace ouster_cluster
{

using Cloud = pcl::PointCloud<pcl::PointXYZI>;

/// Refine one cluster by Mahalanobis distance: keep points whose squared
/// Mahalanobis distance from the cluster mean (w.r.t. its covariance) is
/// <= threshold. Trims points that lie off the cluster's shape.
/// Clusters with fewer than 4 points are returned unchanged (covariance
/// is not meaningful). A tiny ridge is added to the covariance diagonal to
/// keep the inverse well-defined for near-degenerate (planar/linear) clusters.
inline pcl::PointIndices refineClusterMahalanobis(
  const Cloud & cloud, const pcl::PointIndices & idx, double threshold)
{
  if (idx.indices.size() < 4) {
    return idx;
  }
  // mean
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int i : idx.indices) {
    mean += Eigen::Vector3d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
  }
  mean /= static_cast<double>(idx.indices.size());
  // covariance
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (int i : idx.indices) {
    Eigen::Vector3d d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    d -= mean;
    cov += d * d.transpose();
  }
  cov /= static_cast<double>(idx.indices.size() - 1);
  cov += Eigen::Matrix3d::Identity() * 1e-6;  // ridge for degenerate clusters
  const Eigen::Matrix3d inv = cov.inverse();

  pcl::PointIndices out;
  out.header = idx.header;
  for (int i : idx.indices) {
    Eigen::Vector3d d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    d -= mean;
    const double m2 = d.transpose() * inv * d;
    if (m2 <= threshold) {
      out.indices.push_back(i);
    }
  }
  return out;
}

/// Oriented bounding box: a center, a rotation (columns of `rotation` are the
/// box's local axes), and half-extents along those axes. The 8 corners are
/// center +/- rotation * diag(half). Used to draw tracked plates as wireframes.
struct OBB
{
  Eigen::Vector3d center;
  Eigen::Matrix3d rotation;      // right-handed; columns = principal axes
  Eigen::Vector3d half_extents;  // half-size along each principal axis
};

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__CLUSTER_LOGIC_HPP_
