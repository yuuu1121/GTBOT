#include <gtest/gtest.h>
#include <cstdint>
#include <cstring>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ouster_cluster/ouster_convert.hpp"

using ouster_cluster::fromOusterMsg;

namespace
{
// Build a minimal Ouster-style PointCloud2 with fields x,y,z (float32) and
// reflectivity (uint16), matching the real /ouster/points offsets loosely
// (we let PointCloud2Modifier lay them out; the helper reads by field name).
sensor_msgs::msg::PointCloud2 makeCloud(
  const std::vector<std::array<float, 3>> & xyz,
  const std::vector<uint16_t> & refl)
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "reflectivity", 1, sensor_msgs::msg::PointField::UINT16);
  mod.resize(xyz.size());
  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint16_t> ir(msg, "reflectivity");
  for (size_t i = 0; i < xyz.size(); ++i, ++ix, ++iy, ++iz, ++ir) {
    *ix = xyz[i][0]; *iy = xyz[i][1]; *iz = xyz[i][2]; *ir = refl[i];
  }
  return msg;
}
}  // namespace

// reflectivity value lands in the PointXYZI intensity slot; xyz preserved.
TEST(OusterConvert, ReflectivityToIntensity)
{
  auto msg = makeCloud(
    {{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}},
    {150, 42});
  auto cloud = fromOusterMsg(msg);
  ASSERT_EQ(cloud->size(), 2u);
  EXPECT_FLOAT_EQ(cloud->points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloud->points[0].y, 2.0f);
  EXPECT_FLOAT_EQ(cloud->points[0].z, 3.0f);
  EXPECT_FLOAT_EQ(cloud->points[0].intensity, 150.0f);  // reflectivity
  EXPECT_FLOAT_EQ(cloud->points[1].intensity, 42.0f);
}

// Selecting the float32 "intensity" field puts its value in the intensity slot.
TEST(OusterConvert, IntensityFieldFloat32)
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(5,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "reflectivity", 1, sensor_msgs::msg::PointField::UINT16,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(2);
  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
  sensor_msgs::PointCloud2Iterator<uint16_t> ir(msg, "reflectivity");
  sensor_msgs::PointCloud2Iterator<float> ii(msg, "intensity");
  float xs[2] = {1.0f, 4.0f};
  uint16_t rs[2] = {150, 42};
  float is[2] = {2500.5f, 88.0f};
  for (size_t i = 0; i < 2; ++i, ++ix, ++iy, ++iz, ++ir, ++ii) {
    *ix = xs[i]; *iy = 0.0f; *iz = 0.0f; *ir = rs[i]; *ii = is[i];
  }
  auto cloud = fromOusterMsg(msg, "intensity");
  ASSERT_EQ(cloud->size(), 2u);
  EXPECT_FLOAT_EQ(cloud->points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloud->points[0].intensity, 2500.5f);  // intensity, not reflectivity
  EXPECT_FLOAT_EQ(cloud->points[1].intensity, 88.0f);
}

// Missing reflectivity field -> empty cloud (no crash).
TEST(OusterConvert, MissingFieldReturnsEmpty)
{
  sensor_msgs::msg::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(1);
  auto cloud = fromOusterMsg(msg);
  EXPECT_TRUE(cloud->empty());
}

// Empty input -> empty output.
TEST(OusterConvert, EmptyInput)
{
  auto msg = makeCloud({}, {});
  auto cloud = fromOusterMsg(msg);
  EXPECT_TRUE(cloud->empty());
}

// Fused overload: one pass drops NaN points and applies the elevation band.
// Band [-30, +15] deg: elevation 0 kept, +45 and -45 dropped, NaN dropped.
TEST(OusterConvert, FusedDropsNaNAndOutOfBand)
{
  const float nanv = std::numeric_limits<float>::quiet_NaN();
  auto msg = makeCloud(
    {{1.0f, 0.0f, 0.0f},    // elevation 0 deg -> keep
     {nanv, 0.0f, 0.0f},    // NaN -> drop
     {1.0f, 0.0f, 1.0f},    // +45 deg -> drop (above +15)
     {1.0f, 0.0f, -1.0f}},  // -45 deg -> drop (below -30)
    {150, 1, 2, 3});
  ouster_cluster::ElevationFilterParams elev;
  elev.enable = true;
  elev.min_deg = -30.0;
  elev.max_deg = 15.0;
  auto cloud = fromOusterMsg(msg, "reflectivity", elev);
  ASSERT_EQ(cloud->size(), 1u);
  EXPECT_FLOAT_EQ(cloud->points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloud->points[0].intensity, 150.0f);
  EXPECT_TRUE(cloud->is_dense);  // NaN already removed
}

// Fused overload with the band disabled: NaN still dropped, band ignored.
TEST(OusterConvert, FusedDisabledKeepsBandDropsNaN)
{
  const float nanv = std::numeric_limits<float>::quiet_NaN();
  auto msg = makeCloud(
    {{1.0f, 0.0f, 0.0f},    // keep
     {nanv, 0.0f, 0.0f},    // NaN -> drop even with band off
     {1.0f, 0.0f, 1.0f}},   // +45 deg -> kept (band disabled)
    {150, 1, 2});
  ouster_cluster::ElevationFilterParams elev;
  elev.enable = false;
  auto cloud = fromOusterMsg(msg, "reflectivity", elev);
  ASSERT_EQ(cloud->size(), 2u);
  EXPECT_FLOAT_EQ(cloud->points[1].z, 1.0f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
