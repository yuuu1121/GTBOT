// Tests for the ray-elevation band filter (keep points whose elevation angle
// from the sensor origin lies in [min_deg, max_deg]).
#include <gtest/gtest.h>

#include <cmath>

#include "ouster_cluster/elevation_filter.hpp"

using ouster_cluster::Cloud;
using ouster_cluster::ElevationFilterParams;
using ouster_cluster::filterElevation;

namespace
{

// A point at horizontal distance 1m whose ray elevation is `deg`.
pcl::PointXYZI atElevation(float deg, float x = 1.0f, float y = 0.0f)
{
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = std::hypot(x, y) * std::tan(deg * static_cast<float>(M_PI) / 180.0f);
  p.intensity = 1.0f;
  return p;
}

}  // namespace

TEST(ElevationFilter, DisabledIsPassthrough)
{
  auto cloud = std::make_shared<Cloud>();
  cloud->push_back(atElevation(-40.0f));  // outside band, but filter is off
  ElevationFilterParams prm;
  prm.enable = false;
  auto out = filterElevation(cloud, prm);
  EXPECT_EQ(out->size(), 1u);
}

TEST(ElevationFilter, KeepsBandDropsOutside)
{
  auto cloud = std::make_shared<Cloud>();
  cloud->push_back(atElevation(-40.0f));  // below band -> drop
  cloud->push_back(atElevation(-29.0f));  // inside -> keep
  cloud->push_back(atElevation(0.0f));    // inside -> keep
  cloud->push_back(atElevation(14.0f));   // inside -> keep
  cloud->push_back(atElevation(16.0f));   // above band -> drop
  cloud->push_back(atElevation(40.0f));   // above band -> drop
  ElevationFilterParams prm;
  prm.enable = true;
  prm.min_deg = -30.0;
  prm.max_deg = 15.0;
  auto out = filterElevation(cloud, prm);
  ASSERT_EQ(out->size(), 3u);
  for (const auto & p : *out) {
    const float elev = std::atan2(p.z, std::hypot(p.x, p.y)) * 180.0f / static_cast<float>(M_PI);
    EXPECT_GE(elev, -30.5f);
    EXPECT_LE(elev, 15.5f);
  }
}

TEST(ElevationFilter, IndependentOfAzimuth)
{
  auto cloud = std::make_shared<Cloud>();
  cloud->push_back(atElevation(-20.0f, -0.7f, -0.7f));  // behind-left, inside band
  cloud->push_back(atElevation(-40.0f, -0.7f, 0.7f));   // behind-right, below band
  ElevationFilterParams prm;
  prm.enable = true;
  auto out = filterElevation(cloud, prm);
  ASSERT_EQ(out->size(), 1u);
  EXPECT_LT(out->front().z, 0.0f);
}

TEST(ElevationFilter, EmptyCloud)
{
  auto cloud = std::make_shared<Cloud>();
  ElevationFilterParams prm;
  prm.enable = true;
  auto out = filterElevation(cloud, prm);
  EXPECT_TRUE(out->empty());
}
