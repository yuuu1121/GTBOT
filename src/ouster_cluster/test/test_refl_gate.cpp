// Tests for the segment reflectivity gate: a real plate is a single material,
// so its supporting points must have uniform reflectivity (small p10-p90
// spread) and a median inside the plate material's absolute band.
#include <gtest/gtest.h>

#include "ouster_cluster/bev_lines.hpp"

namespace oc = ouster_cluster;

namespace
{

// n points spread evenly along the segment [x0,y0]->[x1,y1] at height z,
// all with the given reflectivity.
void addLinePoints(
  oc::Cloud & cloud, float x0, float y0, float x1, float y1,
  float z, float refl, int n)
{
  for (int i = 0; i < n; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(n - 1);
    pcl::PointXYZI p;
    p.x = x0 + t * (x1 - x0);
    p.y = y0 + t * (y1 - y0);
    p.z = z;
    p.intensity = refl;
    cloud.push_back(p);
  }
}

oc::BevSegment segment()
{
  // 14cm segment along x at y=1, z band [0.0, 0.1]
  return oc::BevSegment{1.0f, 1.0f, 1.14f, 1.0f, 6, 0.0f, 0.1f};
}

}  // namespace

TEST(ReflGate, DisabledPassesEverything)
{
  auto cloud = std::make_shared<oc::Cloud>();
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 10.f, 10);
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 200.f, 10);
  oc::BevReflGateParams prm;
  prm.enable = false;
  EXPECT_TRUE(oc::passesReflectivityGate(segment(), cloud, prm));
}

TEST(ReflGate, UniformReflectivityPasses)
{
  auto cloud = std::make_shared<oc::Cloud>();
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 50.f, 20);
  oc::BevReflGateParams prm;
  prm.enable = true;
  EXPECT_TRUE(oc::passesReflectivityGate(segment(), cloud, prm));
}

TEST(ReflGate, MixedReflectivityFails)
{
  // half dark (10), half bright (150): spread 140 > default 60 -> not one material
  auto cloud = std::make_shared<oc::Cloud>();
  addLinePoints(*cloud, 1.0f, 1.0f, 1.07f, 1.0f, 0.05f, 10.f, 10);
  addLinePoints(*cloud, 1.07f, 1.0f, 1.14f, 1.0f, 0.05f, 150.f, 10);
  oc::BevReflGateParams prm;
  prm.enable = true;
  EXPECT_FALSE(oc::passesReflectivityGate(segment(), cloud, prm));
}

TEST(ReflGate, MedianBelowBandFails)
{
  // uniform but nearly black (splash noise measures ~1)
  auto cloud = std::make_shared<oc::Cloud>();
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 1.f, 20);
  oc::BevReflGateParams prm;
  prm.enable = true;
  prm.min_median = 5.0;
  EXPECT_FALSE(oc::passesReflectivityGate(segment(), cloud, prm));
}

TEST(ReflGate, MedianAboveBandFails)
{
  auto cloud = std::make_shared<oc::Cloud>();
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 240.f, 20);
  oc::BevReflGateParams prm;
  prm.enable = true;
  prm.max_median = 200.0;
  EXPECT_FALSE(oc::passesReflectivityGate(segment(), cloud, prm));
}

TEST(ReflGate, TooFewSupportingPointsPasses)
{
  // below min_points the cell evidence already vouched; do not judge
  auto cloud = std::make_shared<oc::Cloud>();
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 10.f, 3);
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 150.f, 3);
  oc::BevReflGateParams prm;
  prm.enable = true;
  EXPECT_TRUE(oc::passesReflectivityGate(segment(), cloud, prm));
}

TEST(ReflGate, IgnoresPointsOutsideBand)
{
  auto cloud = std::make_shared<oc::Cloud>();
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.05f, 50.f, 20);
  // wild reflectivity far away in XY and at a different height: must not count
  addLinePoints(*cloud, 3.0f, 3.0f, 3.14f, 3.0f, 0.05f, 250.f, 20);
  addLinePoints(*cloud, 1.0f, 1.0f, 1.14f, 1.0f, 0.9f, 250.f, 20);
  oc::BevReflGateParams prm;
  prm.enable = true;
  EXPECT_TRUE(oc::passesReflectivityGate(segment(), cloud, prm));
}

TEST(ReflGate, EmptyCloudPasses)
{
  auto cloud = std::make_shared<oc::Cloud>();
  oc::BevReflGateParams prm;
  prm.enable = true;
  EXPECT_TRUE(oc::passesReflectivityGate(segment(), cloud, prm));
}
