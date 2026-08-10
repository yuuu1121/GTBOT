// Host adapter for the CUDA front-end: resolves PointCloud2 field offsets,
// mirrors the CPU precomputations (tan of the elevation band), and converts
// the raw GPU output into PCL clouds. See gpu_frontend.hpp for semantics.
#include "ouster_cluster/gpu_frontend.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "gpu_frontend_cuda.h"

namespace ouster_cluster
{

namespace
{

/// Byte offset + datatype of the named field, or false if absent.
bool fieldInfo(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & name,
  std::uint32_t & offset, std::uint8_t & datatype)
{
  for (const auto & f : msg.fields) {
    if (f.name == name) {
      offset = f.offset;
      datatype = f.datatype;
      return true;
    }
  }
  return false;
}

Cloud::Ptr toCloud(const std::vector<gpu::GpuPoint> & pts)
{
  auto out = std::make_shared<Cloud>();
  out->resize(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    auto & q = out->points[i];
    q.x = pts[i].x;
    q.y = pts[i].y;
    q.z = pts[i].z;
    q.intensity = pts[i].intensity;
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

}  // namespace

struct GpuFrontend::Impl
{
  gpu::CudaFrontend cuda;
  std::vector<gpu::GpuPoint> conv_buf, prep_buf, dens_buf;
};

bool GpuFrontend::available()
{
  static const bool ok = gpu::CudaFrontend::deviceAvailable();
  return ok;
}

GpuFrontend::GpuFrontend()
: impl_(new Impl) {}

GpuFrontend::~GpuFrontend() = default;

bool GpuFrontend::process(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::string & scalar_field,
  const ElevationFilterParams & elev,
  const PreprocessParams & pre,
  const BevDensityParams & bev,
  bool want_conv,
  GpuFrontendResult & out)
{
  if (pre.voxel_leaf > 0.0 || bev.cell_size <= 0.0) {return false;}  // not ported

  gpu::FrontendParams p;
  std::uint8_t dt_x = 0, dt_y = 0, dt_z = 0, dt_s = 0;
  if (!fieldInfo(msg, "x", p.off_x, dt_x) ||
    !fieldInfo(msg, "y", p.off_y, dt_y) ||
    !fieldInfo(msg, "z", p.off_z, dt_z) ||
    !fieldInfo(msg, scalar_field, p.off_scalar, dt_s))
  {
    return false;
  }
  const auto f32 = sensor_msgs::msg::PointField::FLOAT32;
  const auto u16 = sensor_msgs::msg::PointField::UINT16;
  if (dt_x != f32 || dt_y != f32 || dt_z != f32) {return false;}
  if (dt_s == u16) {p.scalar_is_u16 = true;} else if (dt_s == f32) {
    p.scalar_is_u16 = false;
  } else {return false;}

  p.point_step = msg.point_step;
  p.elev_enable = elev.enable;
  p.tan_min = static_cast<float>(std::tan(elev.min_deg * M_PI / 180.0));
  p.tan_max = static_cast<float>(std::tan(elev.max_deg * M_PI / 180.0));
  p.max_range = pre.max_range;
  p.z_min = pre.z_min;
  p.z_max = pre.z_max;
  p.cell_size = bev.cell_size;
  p.min_density = bev.min_density;
  p.max_density = bev.max_density;
  p.z_bin = bev.z_bin;
  p.min_pts_per_bin = bev.min_pts_per_bin;
  p.min_z_bins = bev.min_z_bins;
  p.dual_grid = bev.dual_grid;
  p.drop_lonely = bev.drop_lonely;
  p.want_conv = want_conv;

  const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
  if (msg.point_step == 0 ||
    static_cast<std::size_t>(msg.point_step) * n > msg.data.size())
  {
    return false;  // truncated/malformed buffer — refuse instead of reading past it
  }
  impl_->conv_buf.clear();
  impl_->prep_buf.clear();
  impl_->dens_buf.clear();
  std::size_t n_conv = 0, n_prep = 0;
  if (n == 0) {
    out.conv = want_conv ? std::make_shared<Cloud>() : nullptr;
    out.prep = std::make_shared<Cloud>();
    out.density = std::make_shared<Cloud>();
    out.n_conv = out.n_prep = 0;
    return true;
  }
  if (!impl_->cuda.run(msg.data.data(), n, p, impl_->conv_buf, impl_->prep_buf,
    impl_->dens_buf, n_conv, n_prep))
  {
    return false;
  }
  out.conv = want_conv ? toCloud(impl_->conv_buf) : nullptr;
  out.prep = toCloud(impl_->prep_buf);
  out.density = toCloud(impl_->dens_buf);
  out.n_conv = n_conv;
  out.n_prep = n_prep;
  return true;
}

}  // namespace ouster_cluster
