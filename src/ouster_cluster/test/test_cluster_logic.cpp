#include <gtest/gtest.h>

#include "ouster_cluster/cluster_logic.hpp"

using ouster_cluster::Cloud;

// Mahalanobis trims a point lying far off an elongated cluster's axis.
TEST(ClusterLogic, MahalanobisTrimsOffAxis)
{
  Cloud cloud;
  // elongated cluster along x: 40 points, tightly packed in y,z
  for (int i = 0; i < 40; ++i) {
    pcl::PointXYZI p;
    p.x = 0.02f * i;  // spans 0..0.78 in x
    p.y = 0.0f;
    p.z = 0.0f;
    p.intensity = 200.0f;
    cloud.push_back(p);
  }
  // one outlier far off the y axis (perpendicular to the cluster's spread)
  pcl::PointXYZI out;
  out.x = 0.4f; out.y = 1.0f; out.z = 0.0f; out.intensity = 200.0f;
  cloud.push_back(out);
  const int outlier_idx = static_cast<int>(cloud.size()) - 1;

  pcl::PointIndices all;
  for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {all.indices.push_back(i);}

  auto refined = ouster_cluster::refineClusterMahalanobis(cloud, all, 7.815);
  // the off-axis point must be dropped
  EXPECT_EQ(refined.indices.size(), all.indices.size() - 1);
  for (int i : refined.indices) {
    EXPECT_NE(i, outlier_idx);
  }
}

// Mahalanobis on a tiny cluster (<4 pts) returns it unchanged.
TEST(ClusterLogic, MahalanobisSkipsTinyCluster)
{
  Cloud cloud;
  for (int i = 0; i < 3; ++i) {
    pcl::PointXYZI p;
    p.x = i; p.y = 0; p.z = 0; p.intensity = 200.0f;
    cloud.push_back(p);
  }
  pcl::PointIndices all;
  for (int i = 0; i < 3; ++i) {all.indices.push_back(i);}
  auto refined = ouster_cluster::refineClusterMahalanobis(cloud, all, 7.815);
  EXPECT_EQ(refined.indices.size(), 3u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
