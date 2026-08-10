// GPU front-end vs CPU pipeline equivalence. Builds a synthetic Ouster-style
// PointCloud2, runs fromOusterMsg -> preprocessCloud -> computeBevDensity on
// the CPU and GpuFrontend::process on the GPU, and requires identical point
// sequences. Density intensities are fixed-point on the GPU, so they are
// compared with a small relative tolerance; test geometry keeps every cell
// away from the density/z-bin gate thresholds so rounding cannot flip a gate.
// All tests skip on machines without a CUDA device.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ouster_cluster/gpu_frontend.hpp"
#include "ouster_cluster/ouster_convert.hpp"
#include "ouster_cluster/plane_detect.hpp"
#include "ouster_cluster/bev_density.hpp"

namespace oc = ouster_cluster;

namespace
{

// xyzi float32 PointCloud2 (layout of the driver's point_type:=xyzi)
sensor_msgs::msg::PointCloud2 makeMsg(const std::vector<std::array<float, 4>> & pts)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(pts.size());
  msg.point_step = 16;
  msg.row_step = msg.point_step * msg.width;
  msg.is_bigendian = false;
  msg.is_dense = false;
  const char * names[4] = {"x", "y", "z", "intensity"};
  for (int i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField f;
    f.name = names[i];
    f.offset = 4 * i;
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.count = 1;
    msg.fields.push_back(f);
  }
  msg.data.resize(msg.row_step);
  std::memcpy(msg.data.data(), pts.data(), msg.row_step);
  return msg;
}

// dense synthetic scene: two vertical "plates" + floor + far wall + NaNs,
// with a few thousand jittered points so hash tables and compaction get real
// load. Deterministic seed.
std::vector<std::array<float, 4>> makeScene(std::size_t n_extra)
{
  std::vector<std::array<float, 4>> pts;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> j(-0.004f, 0.004f);
  // plate A: x ~ 1.5, y in [-0.2, 0.2], z in [-0.45, -0.05] (inside crop)
  for (int yi = 0; yi < 40; ++yi) {
    for (int zi = 0; zi < 30; ++zi) {
      pts.push_back(
        {1.5f + j(rng), -0.2f + 0.01f * yi + j(rng), -0.45f + 0.0135f * zi, 100.f});
    }
  }
  // plate B: y ~ -1.2
  for (int xi = 0; xi < 30; ++xi) {
    for (int zi = 0; zi < 25; ++zi) {
      pts.push_back(
        {0.6f + 0.012f * xi + j(rng), -1.2f + j(rng), -0.4f + 0.014f * zi, 80.f});
    }
  }
  // floor sheet at z=-0.48 (single z-bin -> killed by min_z_bins)
  for (int xi = 0; xi < 40; ++xi) {
    for (int yi = 0; yi < 40; ++yi) {
      pts.push_back({0.5f + 0.05f * xi, -1.0f + 0.05f * yi, -0.48f, 10.f});
    }
  }
  // far wall beyond max_range
  for (int i = 0; i < 500; ++i) {
    pts.push_back({8.f, -2.f + 0.008f * i, -0.2f, 50.f});
  }
  // NaNs and out-of-elevation-band points
  for (int i = 0; i < 200; ++i) {
    pts.push_back({NAN, NAN, NAN, 0.f});
    pts.push_back({0.5f, 0.5f, 2.0f, 5.f});   // way above elev band
  }
  // random scatter inside the crop volume
  std::uniform_real_distribution<float> rx(0.3f, 2.8f), ry(-2.5f, 2.5f),
  rz(-0.5f, 0.0f);
  for (std::size_t i = 0; i < n_extra; ++i) {
    pts.push_back({rx(rng), ry(rng), rz(rng), 42.f});
  }
  return pts;
}

struct PipelineCfg
{
  oc::ElevationFilterParams elev;
  oc::PreprocessParams pre;
  oc::BevDensityParams bev;
  PipelineCfg()
  {
    elev.enable = true;
    elev.min_deg = -30.0;
    elev.max_deg = 15.0;
    pre.max_range = 3.0;
    pre.z_min = -0.5;
    pre.z_max = 0.0;
    pre.voxel_leaf = 0.0;
    bev.cell_size = 0.02;
    bev.min_density = 0.5;
    bev.max_density = 0.0;
    bev.z_bin = 0.05;
    bev.min_pts_per_bin = 2;
    bev.min_z_bins = 2;
    bev.dual_grid = true;
    bev.drop_lonely = true;
  }
};

oc::Cloud::Ptr cpuPipeline(
  const sensor_msgs::msg::PointCloud2 & msg, const PipelineCfg & s, oc::Cloud::Ptr * conv_out)
{
  auto cloud = oc::fromOusterMsg(msg, "intensity", s.elev);
  if (conv_out) {*conv_out = cloud;}
  cloud = oc::preprocessCloud(cloud, s.pre);
  return oc::computeBevDensity(cloud, s.bev);
}

void expectSameCloud(
  const oc::Cloud & a, const oc::Cloud & b, double int_rel_tol, const char * what)
{
  ASSERT_EQ(a.size(), b.size()) << what;
  for (std::size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a.points[i].x, b.points[i].x) << what << " @" << i;
    EXPECT_EQ(a.points[i].y, b.points[i].y) << what << " @" << i;
    EXPECT_EQ(a.points[i].z, b.points[i].z) << what << " @" << i;
    const double ia = a.points[i].intensity, ib = b.points[i].intensity;
    EXPECT_NEAR(ia, ib, std::max(1e-3, std::abs(ia)) * int_rel_tol)
      << what << " intensity @" << i;
  }
}

}  // namespace

TEST(GpuFrontend, MatchesCpuPipeline)
{
  if (!oc::GpuFrontend::available()) {GTEST_SKIP() << "no CUDA device";}
  const PipelineCfg s;
  const auto msg = makeMsg(makeScene(4000));

  oc::Cloud::Ptr cpu_conv;
  const auto cpu_density = cpuPipeline(msg, s, &cpu_conv);
  ASSERT_GT(cpu_density->size(), 0u);

  oc::GpuFrontend gpu;
  oc::GpuFrontendResult r;
  ASSERT_TRUE(gpu.process(msg, "intensity", s.elev, s.pre, s.bev, true, r));

  // conv+cut cloud must match exactly (pure per-point gates, no accumulation)
  ASSERT_TRUE(r.conv != nullptr);
  expectSameCloud(*cpu_conv, *r.conv, 0.0, "conv");
  EXPECT_EQ(r.n_conv, cpu_conv->size());

  // density cloud: same points in the same order; intensity to 1e-4 relative
  expectSameCloud(*cpu_density, *r.density, 1e-4, "density");
}

TEST(GpuFrontend, MatchesCpuSingleGridNoLonely)
{
  if (!oc::GpuFrontend::available()) {GTEST_SKIP() << "no CUDA device";}
  PipelineCfg s;
  s.bev.dual_grid = false;
  s.bev.drop_lonely = false;
  s.bev.min_z_bins = 0;  // exercises the no-z-histogram path
  const auto msg = makeMsg(makeScene(2000));

  const auto cpu_density = cpuPipeline(msg, s, nullptr);
  oc::GpuFrontend gpu;
  oc::GpuFrontendResult r;
  ASSERT_TRUE(gpu.process(msg, "intensity", s.elev, s.pre, s.bev, false, r));
  EXPECT_TRUE(r.conv == nullptr);
  expectSameCloud(*cpu_density, *r.density, 1e-4, "density(single grid)");
}

TEST(GpuFrontend, RefusesVoxelLeaf)
{
  if (!oc::GpuFrontend::available()) {GTEST_SKIP() << "no CUDA device";}
  PipelineCfg s;
  s.pre.voxel_leaf = 0.03;  // not ported -> must refuse so the node falls back
  const auto msg = makeMsg(makeScene(100));
  oc::GpuFrontend gpu;
  oc::GpuFrontendResult r;
  EXPECT_FALSE(gpu.process(msg, "intensity", s.elev, s.pre, s.bev, false, r));
}

TEST(GpuFrontend, EmptyAndAllNaN)
{
  if (!oc::GpuFrontend::available()) {GTEST_SKIP() << "no CUDA device";}
  const PipelineCfg s;
  oc::GpuFrontend gpu;

  const auto empty_msg = makeMsg({});
  oc::GpuFrontendResult r0;
  ASSERT_TRUE(gpu.process(empty_msg, "intensity", s.elev, s.pre, s.bev, false, r0));
  EXPECT_EQ(r0.n_conv, 0u);
  EXPECT_TRUE(r0.density->empty());

  std::vector<std::array<float, 4>> nans(64, {NAN, NAN, NAN, 0.f});
  oc::GpuFrontendResult r1;
  ASSERT_TRUE(gpu.process(makeMsg(nans), "intensity", s.elev, s.pre, s.bev, false, r1));
  EXPECT_EQ(r1.n_conv, 0u);
  EXPECT_TRUE(r1.density->empty());
}
