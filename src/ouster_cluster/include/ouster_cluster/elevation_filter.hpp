// Ray-elevation band filter: keep only points whose ray from the sensor origin
// has an elevation angle inside [min_deg, max_deg]. Replaces per-ring masking
// (the sensor firmware cannot disable vertical beams) with pure geometry, so it
// needs no beam-calibration table. Runs before preprocessCloud so every
// downstream stage sees fewer points.
#ifndef OUSTER_CLUSTER__ELEVATION_FILTER_HPP_
#define OUSTER_CLUSTER__ELEVATION_FILTER_HPP_

#include <cmath>
#include <memory>

#include "ouster_cluster/cluster_logic.hpp"  // ouster_cluster::Cloud

namespace ouster_cluster
{

struct ElevationFilterParams
{
  bool enable = false;
  double min_deg = -30.0;  // OS0 spans -45..+45; default band cuts the floor cone
  double max_deg = 15.0;   // ...and overhead clutter above the target height
};

/// Keep points with elevation angle in [min_deg, max_deg]. Compares
/// z against r_xy * tan(bound) instead of calling atan2 per point.
inline Cloud::Ptr filterElevation(const Cloud::Ptr & cloud, const ElevationFilterParams & prm)
{
  if (!prm.enable) {return cloud;}
  const float tan_min = static_cast<float>(std::tan(prm.min_deg * M_PI / 180.0));
  const float tan_max = static_cast<float>(std::tan(prm.max_deg * M_PI / 180.0));
  auto out = std::make_shared<Cloud>();
  out->reserve(cloud->size());
  for (const auto & p : *cloud) {
    const float r_xy = std::hypot(p.x, p.y);
    if (p.z >= r_xy * tan_min && p.z <= r_xy * tan_max) {
      out->push_back(p);
    }
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = cloud->is_dense;
  return out;
}

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__ELEVATION_FILTER_HPP_
