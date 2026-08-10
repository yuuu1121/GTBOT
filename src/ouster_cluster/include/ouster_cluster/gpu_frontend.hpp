// CUDA front-end: fuses convert + NaN/elevation cut + range/z crop + BEV
// density (fromOusterMsg -> preprocessCloud -> computeBevDensity) into one
// GPU pass over the raw PointCloud2 buffer. Cell keys, bit packing and gate
// logic mirror bev_density.hpp exactly; per-cell r^2 sums use fixed-point
// integer atomics so results are deterministic run-to-run (float sums may
// differ from the CPU path in the last bits — density values agree to ~1e-6
// relative, so gates only diverge for cells sitting exactly on a threshold).
// Point order of every output cloud matches the CPU path (stable compaction),
// so downstream stages (bev_lines component order, max_segments cutoff,
// recover accumulation) see identical inputs.
//
// This header is CUDA-free; it is only compiled into the node when CMake
// found a CUDA toolchain (OUSTER_CLUSTER_HAS_CUDA). Voxel downsampling is not
// ported (unused in the tuned config) — process() refuses when voxel_leaf > 0
// and the caller falls back to the CPU path.
#ifndef OUSTER_CLUSTER__GPU_FRONTEND_HPP_
#define OUSTER_CLUSTER__GPU_FRONTEND_HPP_

#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ouster_cluster/cluster_logic.hpp"    // ouster_cluster::Cloud
#include "ouster_cluster/elevation_filter.hpp"  // ElevationFilterParams
#include "ouster_cluster/plane_detect.hpp"      // PreprocessParams
#include "ouster_cluster/bev_density.hpp"       // BevDensityParams

namespace ouster_cluster
{

struct GpuFrontendResult
{
  Cloud::Ptr conv;      // conv+cut cloud (pubF debug) — only when want_conv
  Cloud::Ptr prep;      // range/z-cropped cloud (reflectivity gate input)
  Cloud::Ptr density;   // computeBevDensity-equivalent output
  std::size_t n_conv = 0;  // points after conv+cut (timing_log "n=")
  std::size_t n_prep = 0;  // points after range/z crop
};

class GpuFrontend
{
public:
  /// True if a CUDA device is present (checked once).
  static bool available();

  GpuFrontend();
  ~GpuFrontend();

  /// Run the fused front-end. Returns false (leaving `out` untouched) when
  /// the msg lacks x/y/z/scalar fields, the scalar datatype is unsupported,
  /// voxel_leaf > 0, or a CUDA error occurred — caller falls back to CPU.
  /// `out.conv` is only produced when `want_conv` (it costs an extra
  /// compaction + copy-back of the ~26k-point debug cloud).
  bool process(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::string & scalar_field,
    const ElevationFilterParams & elev,
    const PreprocessParams & pre,
    const BevDensityParams & bev,
    bool want_conv,
    GpuFrontendResult & out);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__GPU_FRONTEND_HPP_
