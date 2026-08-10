// ROS-independent geometric detection of an upright rectangular plate in a BEV
// (top-down) sense: preprocessing crop, a PCA test that a cluster is a thin,
// ~14cm-wide, ~10cm-tall plate, and the seed-grown recovery clustering.
#ifndef OUSTER_CLUSTER__PLANE_DETECT_HPP_
#define OUSTER_CLUSTER__PLANE_DETECT_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/voxel_grid.h>

#include "ouster_cluster/cluster_logic.hpp"  // ouster_cluster::Cloud

namespace ouster_cluster
{

/// Parameters for the cheap front-of-pipeline cull that keeps the heavy stages
/// (RANSAC, clustering) fast enough to run in real time.
struct PreprocessParams
{
  double max_range = 3.0;    // drop points farther than this from the sensor (m); <=0 disables
  double z_min = -1.0;       // drop points below this height (m, sensor frame)
  double z_max = 1.0;        // drop points above this height (m, sensor frame)
  double voxel_leaf = 0.02;  // voxel edge for downsampling (m); <=0 disables
  double min_intensity = 0.0;  // drop points with scalar below this; <=0 disables
                               // (retroreflector plates pop vs background — cheap
                               //  material cut before geometry stages)
};

/// Range/height-crop then voxel-downsample `cloud`. Points farther than
/// `max_range` from the origin, or outside the [z_min, z_max] height band, are
/// dropped first (cheap, big cut), then a VoxelGrid collapses each
/// `voxel_leaf`-sized cell to one point. The z-band removes floor/ceiling so a
/// small cluster tolerance keeps objects separate. Range crop is skipped when
/// max_range <= 0; voxel when voxel_leaf <= 0. Returns a new cloud.
inline Cloud::Ptr preprocessCloud(const Cloud::Ptr & cloud, const PreprocessParams & p)
{
  auto out = std::make_shared<Cloud>();
  if (!cloud || cloud->empty()) {
    if (cloud) {*out = *cloud;}
    return out;
  }

  // 1) range + height crop
  const bool do_range = p.max_range > 0.0;
  const double r2 = p.max_range * p.max_range;
  const Cloud::Ptr cropped = std::make_shared<Cloud>();
  cropped->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    if (pt.z < p.z_min || pt.z > p.z_max) {continue;}
    if (do_range && pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > r2) {continue;}
    if (p.min_intensity > 0.0 && pt.intensity < p.min_intensity) {continue;}
    cropped->push_back(pt);
  }
  cropped->width = cropped->size();
  cropped->height = 1;
  cropped->is_dense = true;

  // 2) voxel downsample
  if (p.voxel_leaf > 0.0 && !cropped->empty()) {
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cropped);
    const float leaf = static_cast<float>(p.voxel_leaf);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*out);
  } else {
    *out = *cropped;
  }
  return out;
}

/// Tunable thresholds for the plate test.
struct PlateParams
{
  double thickness_max = 0.03;  // max spread along the thinnest axis (m)
  double width = 0.14;          // target BEV width (m)
  double height = 0.10;         // target height = z range (m)
  double size_tol = 0.03;       // +/- tolerance on width and height (m)
  int min_points = 10;          // reject clusters with too few points
};

/// Result of testing a cluster against the plate model.
struct PlateFit
{
  bool is_plate = false;
  double bev_width = 0.0;                          // xy principal-axis length
  double height = 0.0;                             // z_max - z_min
  double thickness = 0.0;                          // spread along 3D min axis
  double yaw = 0.0;                                // xy principal-axis heading
  Eigen::Vector2d center_xy = Eigen::Vector2d::Zero();
  double z_min = 0.0;
  double z_max = 0.0;
};

/// Test whether cluster `idx` of `cloud` is an upright ~width x ~height thin
/// plate. BEV width uses a 2D PCA on xy (z dropped) so the vertical extent does
/// not pollute the width; height is the z range; thickness is the smallest
/// spread of the full 3D PCA (planarity).
inline PlateFit fitRectangularPlate(
  const Cloud & cloud, const pcl::PointIndices & idx, const PlateParams & p)
{
  PlateFit f;
  const int n = static_cast<int>(idx.indices.size());
  if (n < p.min_points) {return f;}

  // ---- z range (height) + xy centroid ----
  double zmn = std::numeric_limits<double>::max();
  double zmx = std::numeric_limits<double>::lowest();
  Eigen::Vector2d mean_xy = Eigen::Vector2d::Zero();
  Eigen::Vector3d mean_3d = Eigen::Vector3d::Zero();
  for (int i : idx.indices) {
    const auto & pt = cloud.points[i];
    zmn = std::min<double>(zmn, pt.z);
    zmx = std::max<double>(zmx, pt.z);
    mean_xy += Eigen::Vector2d(pt.x, pt.y);
    mean_3d += Eigen::Vector3d(pt.x, pt.y, pt.z);
  }
  mean_xy /= n;
  mean_3d /= n;
  f.center_xy = mean_xy;
  f.z_min = zmn; f.z_max = zmx;
  f.height = zmx - zmn;

  // ---- thickness: smallest eigenvalue axis of 3D covariance ----
  Eigen::Matrix3d cov3 = Eigen::Matrix3d::Zero();
  for (int i : idx.indices) {
    Eigen::Vector3d d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    d -= mean_3d;
    cov3 += d * d.transpose();
  }
  cov3 /= (n - 1);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es3(cov3);
  const Eigen::Vector3d axis_min = es3.eigenvectors().col(0);  // smallest eigenvalue
  double tmn = std::numeric_limits<double>::max();
  double tmx = std::numeric_limits<double>::lowest();
  for (int i : idx.indices) {
    Eigen::Vector3d d(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    d -= mean_3d;
    const double t = d.dot(axis_min);
    tmn = std::min(tmn, t); tmx = std::max(tmx, t);
  }
  f.thickness = tmx - tmn;

  // ---- BEV width: 2D PCA on xy, length along the dominant xy axis ----
  Eigen::Matrix2d cov2 = Eigen::Matrix2d::Zero();
  for (int i : idx.indices) {
    Eigen::Vector2d d(cloud.points[i].x, cloud.points[i].y);
    d -= mean_xy;
    cov2 += d * d.transpose();
  }
  cov2 /= (n - 1);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(cov2);
  const Eigen::Vector2d axis_xy = es2.eigenvectors().col(1);  // largest
  f.yaw = std::atan2(axis_xy.y(), axis_xy.x());
  double wmn = std::numeric_limits<double>::max();
  double wmx = std::numeric_limits<double>::lowest();
  for (int i : idx.indices) {
    Eigen::Vector2d d(cloud.points[i].x, cloud.points[i].y);
    d -= mean_xy;
    const double w = d.dot(axis_xy);
    wmn = std::min(wmn, w); wmx = std::max(wmx, w);
  }
  f.bev_width = wmx - wmn;

  // ---- decision ----
  const bool thin = f.thickness <= p.thickness_max;
  const bool width_ok = std::abs(f.bev_width - p.width) <= p.size_tol;
  const bool height_ok = std::abs(f.height - p.height) <= p.size_tol;
  f.is_plate = thin && width_ok && height_ok;
  return f;
}

/// Concatenate a set of clouds into one (no transform; sensor is static).
/// Used to accumulate the last N preprocessed frames so the plate's scan rings
/// interleave — at long range one frame's rings are too sparse to cluster, but
/// their union fills the gaps. Empty input -> empty cloud.
inline Cloud::Ptr accumulateClouds(const std::vector<Cloud::Ptr> & clouds)
{
  auto out = std::make_shared<Cloud>();
  std::size_t total = 0;
  for (const auto & c : clouds) {if (c) {total += c->size();}}
  out->reserve(total);
  for (const auto & c : clouds) {
    if (!c) {continue;}
    for (const auto & pt : c->points) {out->push_back(pt);}
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

/// Exact euclidean clustering via a spatial hash instead of a KD-tree: the
/// same transitive "within `tol`" connectivity as PCL's
/// EuclideanClusterExtraction (cluster order unspecified; callers here pick by
/// centroid distance, not order). Points hash into cells of edge `tol`, so any
/// within-`tol` neighbour lies in the 27-cell cube around a point — checked
/// with a real distance test. No tree build, O(1) neighbour lookup: several
/// times faster than the KD-tree on the dense plate crops recovery feeds it.
/// Clusters smaller than `min_points` are dropped.
inline std::vector<pcl::PointIndices> clusterEuclideanGrid(
  const Cloud & cloud, double tol, int min_points)
{
  std::vector<pcl::PointIndices> out;
  const int n = static_cast<int>(cloud.size());
  if (n == 0 || tol <= 0.0) {return out;}

  const double inv = 1.0 / tol;
  // 21-bit packed cell key (same trick as bev_density's key3): collisions need
  // coordinates 2^21 cells apart — kilometres at plate-crop scale.
  const auto pack = [](std::int64_t cx, std::int64_t cy, std::int64_t cz) {
      return ((cx & 0x1fffffLL) << 42) | ((cy & 0x1fffffLL) << 21) | (cz & 0x1fffffLL);
    };
  const auto cellOf = [inv](float v) {
      return static_cast<std::int64_t>(std::floor(v * inv));
    };

  std::unordered_map<std::int64_t, std::vector<int>> grid;
  grid.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const auto & p = cloud.points[i];
    grid[pack(cellOf(p.x), cellOf(p.y), cellOf(p.z))].push_back(i);
  }

  const float tol2 = static_cast<float>(tol * tol);
  std::vector<char> used(n, 0);
  std::vector<int> stack;
  for (int i = 0; i < n; ++i) {
    if (used[i]) {continue;}
    used[i] = 1;
    stack.assign(1, i);
    pcl::PointIndices comp;
    while (!stack.empty()) {
      const int j = stack.back();
      stack.pop_back();
      comp.indices.push_back(j);
      const auto & pj = cloud.points[j];
      const std::int64_t cx = cellOf(pj.x), cy = cellOf(pj.y), cz = cellOf(pj.z);
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        for (std::int64_t dy = -1; dy <= 1; ++dy) {
          for (std::int64_t dz = -1; dz <= 1; ++dz) {
            const auto it = grid.find(pack(cx + dx, cy + dy, cz + dz));
            if (it == grid.end()) {continue;}
            for (const int k : it->second) {
              if (used[k]) {continue;}
              const auto & pk = cloud.points[k];
              const float ddx = pk.x - pj.x, ddy = pk.y - pj.y, ddz = pk.z - pj.z;
              if (ddx * ddx + ddy * ddy + ddz * ddz <= tol2) {
                used[k] = 1;
                stack.push_back(k);
              }
            }
          }
        }
      }
    }
    if (static_cast<int>(comp.indices.size()) >= min_points) {
      out.push_back(std::move(comp));
    }
  }
  return out;
}

/// Parameters for the seed-grown final plate clustering.
struct PlateRecoverParams
{
  double xy_half = 0.12;         // crop half-size around the seed centre (m):
                                 // plate physical half-width + margin, so a
                                 // shortened segment can still recover the
                                 // full plate from the raw points
  double z_margin_below = 0.0;   // crop below the seed z_min (m) — keep tight:
                                 // this is the floor guard
  double z_margin_above = 0.02;  // crop above the seed z_max (m)
  double tolerance = 0.03;       // euclidean cluster tolerance (m)
  int min_points = 10;           // minimum crop/cluster size
  double maha_threshold = 7.815;  // squared Mahalanobis cutoff for the final
                                  // shape trim (chi^2 dof=3, 95%): strays that
                                  // rode into the cluster within tolerance but
                                  // sit off its shape are dropped; <= 0 disables
};

/// Final precision stage for a green (plate-classified) segment: crop the RAW
/// preprocessed cloud around the seed (segment centre + its z-band), euclidean
/// -cluster the crop, and return the cluster nearest the seed as a new cloud.
/// Because it runs on raw points, cells the density gates dropped do not
/// matter — a segment measured short recovers the plate's full extent. The
/// z-crop keeps the floor out of the input so the cluster cannot bridge
/// through it. Returns an empty cloud when nothing qualifies.
inline Cloud::Ptr recoverPlateCluster(
  const Cloud::Ptr & cloud, double cx, double cy, double z_lo, double z_hi,
  const PlateRecoverParams & p)
{
  auto out = std::make_shared<Cloud>();
  if (!cloud || cloud->empty()) {return out;}

  auto crop = std::make_shared<Cloud>();
  crop->reserve(cloud->size());
  const float zmin = static_cast<float>(z_lo - p.z_margin_below);
  const float zmax = static_cast<float>(z_hi + p.z_margin_above);
  const float half = static_cast<float>(p.xy_half);
  for (const auto & pt : cloud->points) {
    if (std::fabs(pt.x - static_cast<float>(cx)) <= half &&
      std::fabs(pt.y - static_cast<float>(cy)) <= half &&
      pt.z >= zmin && pt.z <= zmax)
    {
      crop->push_back(pt);
    }
  }
  if (static_cast<int>(crop->size()) < p.min_points) {return out;}
  crop->width = crop->size();
  crop->height = 1;
  crop->is_dense = true;

  // exact euclidean clustering on a spatial hash — same connectivity as the
  // old KD-tree EuclideanClusterExtraction, several times faster on the dense
  // accumulated crop (no tree build, O(1) neighbour lookup)
  const auto clusters = clusterEuclideanGrid(*crop, p.tolerance, p.min_points);
  if (clusters.empty()) {return out;}

  // the seed sits at (cx, cy): take the cluster whose centroid is nearest —
  // NOT the biggest, which may be a denser neighbour inside the crop
  int best = -1;
  double best_d = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    double mx = 0.0, my = 0.0;
    for (int idx : clusters[i].indices) {
      mx += crop->points[idx].x;
      my += crop->points[idx].y;
    }
    const double n = static_cast<double>(clusters[i].indices.size());
    const double d = std::hypot(mx / n - cx, my / n - cy);
    if (d < best_d) {best_d = d; best = static_cast<int>(i);}
  }
  // final shape trim: points that chained into the cluster within tolerance
  // but lie off its estimated shape (mean + covariance) are dropped
  pcl::PointIndices keep = clusters[best];
  if (p.maha_threshold > 0.0) {
    keep = refineClusterMahalanobis(*crop, keep, p.maha_threshold);
  }
  out->reserve(keep.indices.size());
  for (int idx : keep.indices) {out->push_back(crop->points[idx]);}
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__PLANE_DETECT_HPP_
