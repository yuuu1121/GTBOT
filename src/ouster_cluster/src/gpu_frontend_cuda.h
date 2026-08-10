// Plain-C++ bridge between the host adapter (gpu_frontend.cpp, ROS/PCL side)
// and the CUDA implementation (gpu_frontend.cu). No ROS, PCL or CUDA types
// cross this boundary so the .cu never parses ROS/PCL headers.
#ifndef OUSTER_CLUSTER__GPU_FRONTEND_CUDA_H_
#define OUSTER_CLUSTER__GPU_FRONTEND_CUDA_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace ouster_cluster
{
namespace gpu
{

struct GpuPoint  // matches float4 layout on device
{
  float x, y, z, intensity;
};

struct FrontendParams
{
  // raw buffer layout
  std::uint32_t point_step = 0;
  std::uint32_t off_x = 0, off_y = 0, off_z = 0, off_scalar = 0;
  bool scalar_is_u16 = false;  // else float32
  // conv+cut (elevation band; disabled -> NaN cut only)
  bool elev_enable = false;
  float tan_min = 0.f, tan_max = 0.f;
  // preprocess crop
  double max_range = 0.0;  // <= 0 disables the range gate
  double z_min = 0.0, z_max = 0.0;
  // BEV density (semantics of BevDensityParams)
  double cell_size = 0.05;
  double min_density = 0.0, max_density = 0.0;
  double z_bin = 0.05;
  int min_pts_per_bin = 2;
  int min_z_bins = 0;
  bool dual_grid = false;
  bool drop_lonely = false;
  bool want_conv = false;
};

class CudaFrontend
{
public:
  static bool deviceAvailable();

  CudaFrontend();
  ~CudaFrontend();

  /// Returns false on any CUDA error. On success the out_* vectors are
  /// host-side copies; order matches the raw point order (stable compaction).
  bool run(
    const std::uint8_t * raw, std::size_t n_points,
    const FrontendParams & p,
    std::vector<GpuPoint> & out_conv,     // filled only when p.want_conv
    std::vector<GpuPoint> & out_prep,     // range/z-cropped cloud (refl gate)
    std::vector<GpuPoint> & out_density,
    std::size_t & n_conv, std::size_t & n_prep);

private:
  struct State;
  std::unique_ptr<State> s_;
};

}  // namespace gpu
}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__GPU_FRONTEND_CUDA_H_
