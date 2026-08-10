#include <gtest/gtest.h>
#include <cmath>

#include "ouster_cluster/plane_detect.hpp"

using ouster_cluster::Cloud;
using ouster_cluster::PlateParams;
using ouster_cluster::fitRectangularPlate;
using ouster_cluster::PreprocessParams;
using ouster_cluster::preprocessCloud;
using ouster_cluster::clusterEuclideanGrid;

namespace
{
// Small grid blob around (cx, cy, cz): nx x ny points spaced `step`.
void addBlob(Cloud & c, float cx, float cy, float cz, int nx, int ny, float step)
{
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      pcl::PointXYZI p;
      p.x = cx + i * step;
      p.y = cy + j * step;
      p.z = cz;
      p.intensity = 100.0f;
      c.push_back(p);
    }
  }
}
}  // namespace

// Two blobs 1m apart with tolerance 0.05 -> exactly two clusters.
TEST(GridCluster, SeparatesTwoBlobs)
{
  Cloud c;
  addBlob(c, 0.0f, 0.0f, 0.0f, 5, 4, 0.02f);   // 20 pts
  addBlob(c, 1.0f, 0.0f, 0.0f, 5, 4, 0.02f);   // 20 pts
  auto clusters = clusterEuclideanGrid(c, 0.05, 5);
  ASSERT_EQ(clusters.size(), 2u);
  EXPECT_EQ(clusters[0].indices.size(), 20u);
  EXPECT_EQ(clusters[1].indices.size(), 20u);
}

// A chain of points each 0.9 x tolerance apart connects transitively into ONE
// cluster (euclidean clustering semantics, same as PCL EC).
TEST(GridCluster, ChainConnectsTransitively)
{
  Cloud c;
  for (int i = 0; i < 10; ++i) {
    pcl::PointXYZI p;
    p.x = i * 0.027f;  // 0.9 x tol
    p.y = 0.0f; p.z = 0.0f; p.intensity = 100.0f;
    c.push_back(p);
  }
  auto clusters = clusterEuclideanGrid(c, 0.03, 10);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].indices.size(), 10u);
}

// Clusters below min_points are dropped; empty input yields no clusters.
TEST(GridCluster, MinPointsDropsSmallCluster)
{
  Cloud c;
  addBlob(c, 0.0f, 0.0f, 0.0f, 2, 2, 0.01f);  // 4 pts < min 5
  auto clusters = clusterEuclideanGrid(c, 0.05, 5);
  EXPECT_TRUE(clusters.empty());
  Cloud empty;
  EXPECT_TRUE(clusterEuclideanGrid(empty, 0.05, 5).empty());
}

// Two points in DIAGONAL hash cells but within tolerance still connect
// (the neighbour sweep must cover the full 27-cell cube, not just faces).
TEST(GridCluster, DiagonalCellNeighborConnects)
{
  Cloud c;
  pcl::PointXYZI a, b;
  a.x = 0.028f; a.y = 0.028f; a.z = 0.0f; a.intensity = 100.0f;  // cell (0,0,0)
  b.x = 0.031f; b.y = 0.031f; b.z = 0.0f; b.intensity = 100.0f;  // cell (1,1,0)
  c.push_back(a); c.push_back(b);
  auto clusters = clusterEuclideanGrid(c, 0.03, 2);
  ASSERT_EQ(clusters.size(), 1u);
  EXPECT_EQ(clusters[0].indices.size(), 2u);
}

namespace
{
// Add a filled rectangle of points on a plane. `origin` is a corner; `u`,`v`
// span the rectangle; steps control density.
void addRect(Cloud & c, const Eigen::Vector3f & origin,
  const Eigen::Vector3f & u, const Eigen::Vector3f & v, int nu, int nv)
{
  for (int i = 0; i <= nu; ++i) {
    for (int j = 0; j <= nv; ++j) {
      const float a = static_cast<float>(i) / nu, b = static_cast<float>(j) / nv;
      Eigen::Vector3f p = origin + a * u + b * v;
      pcl::PointXYZI pt; pt.x = p.x(); pt.y = p.y(); pt.z = p.z(); pt.intensity = 0;
      c.push_back(pt);
    }
  }
}
}  // namespace

// Helper: make a PointIndices covering all points of a cloud.
static pcl::PointIndices allIdx(const Cloud & c)
{
  pcl::PointIndices idx;
  for (int i = 0; i < static_cast<int>(c.size()); ++i) {idx.indices.push_back(i);}
  return idx;
}

// A 14cm-wide (y), 10cm-tall (z), thin (x~0) upright plate at x=1 -> is_plate.
TEST(PlaneDetect, AcceptsTargetPlate)
{
  Cloud c;
  addRect(c, {1.0f, -0.07f, 0.0f}, {0, 0.14f, 0}, {0, 0, 0.10f}, 14, 10);
  PlateParams p;
  auto f = fitRectangularPlate(c, allIdx(c), p);
  EXPECT_TRUE(f.is_plate);
  EXPECT_NEAR(f.bev_width, 0.14, 0.03);
  EXPECT_NEAR(f.height, 0.10, 0.03);
  EXPECT_LT(f.thickness, 0.03);
}

// Too wide in BEV (30cm) -> rejected.
TEST(PlaneDetect, RejectsWidePlate)
{
  Cloud c;
  addRect(c, {1.0f, -0.15f, 0.0f}, {0, 0.30f, 0}, {0, 0, 0.10f}, 30, 10);
  PlateParams p;
  EXPECT_FALSE(fitRectangularPlate(c, allIdx(c), p).is_plate);
}

// Too tall (30cm) -> rejected.
TEST(PlaneDetect, RejectsTallPlate)
{
  Cloud c;
  addRect(c, {1.0f, -0.07f, 0.0f}, {0, 0.14f, 0}, {0, 0, 0.30f}, 14, 30);
  PlateParams p;
  EXPECT_FALSE(fitRectangularPlate(c, allIdx(c), p).is_plate);
}

// Thick box (10cm in x) -> not a plane, rejected.
TEST(PlaneDetect, RejectsThickBox)
{
  Cloud c;
  // volume: x in [1,1.1], y 14cm, z 10cm
  for (int i = 0; i <= 5; ++i) {
    addRect(c, {1.0f + 0.02f * i, -0.07f, 0.0f}, {0, 0.14f, 0}, {0, 0, 0.10f}, 14, 10);
  }
  PlateParams p;
  EXPECT_FALSE(fitRectangularPlate(c, allIdx(c), p).is_plate);
}

// Lying flat on the ground (z ~ 0 range) -> height fails, rejected.
TEST(PlaneDetect, RejectsFlatPlate)
{
  Cloud c;
  // horizontal 14x10 plate at z=0: x spans 0.14, y spans 0.10, z~0
  addRect(c, {1.0f, -0.05f, 0.0f}, {0.14f, 0, 0}, {0, 0.10f, 0}, 14, 10);
  PlateParams p;
  EXPECT_FALSE(fitRectangularPlate(c, allIdx(c), p).is_plate);
}

// preprocessCloud: the z-band crop drops points outside [z_min, z_max].
TEST(Preprocess, ZBandDropsFloorAndCeiling)
{
  auto c = std::make_shared<Cloud>();
  c->push_back(pcl::PointXYZI());               // floor point, z far below
  c->back().x = 1.0f; c->back().y = 0.0f; c->back().z = -2.0f;
  c->push_back(pcl::PointXYZI());               // keeper, z inside band
  c->back().x = 1.0f; c->back().y = 0.0f; c->back().z = -0.2f;
  c->push_back(pcl::PointXYZI());               // ceiling point, z far above
  c->back().x = 1.0f; c->back().y = 0.0f; c->back().z = 2.0f;

  PreprocessParams p;
  p.z_min = -1.0; p.z_max = 1.0; p.voxel_leaf = 0.0; p.max_range = 0.0;
  auto out = preprocessCloud(c, p);
  ASSERT_EQ(out->size(), 1u);
  EXPECT_NEAR(out->points[0].z, -0.2f, 1e-4);
}

// preprocessCloud: the range crop drops points beyond max_range.
TEST(Preprocess, RangeCropDropsFarPoints)
{
  auto c = std::make_shared<Cloud>();
  c->push_back(pcl::PointXYZI());               // near
  c->back().x = 1.0f; c->back().y = 0.0f; c->back().z = 0.0f;
  c->push_back(pcl::PointXYZI());               // far (5m)
  c->back().x = 5.0f; c->back().y = 0.0f; c->back().z = 0.0f;

  PreprocessParams p;
  p.max_range = 3.0; p.z_min = -1.0; p.z_max = 1.0; p.voxel_leaf = 0.0;
  auto out = preprocessCloud(c, p);
  ASSERT_EQ(out->size(), 1u);
  EXPECT_NEAR(out->points[0].x, 1.0f, 1e-4);
}

// --- recoverPlateCluster: seed-grown final clustering ------------------------

// Seeded at the (possibly shortened) green segment's centre, the recovery
// clusters the RAW points in a crop box sized by the plate's physical size —
// so it returns the FULL plate even when the density gates shortened the
// segment. Points outside the crop never enter.
TEST(PlateRecover, RecoversFullPlateAndIgnoresFarNoise)
{
  auto cloud = std::make_shared<Cloud>();
  // full plate: 0.14 along y at x=1.0, z 0.00..0.10, 1cm spacing
  addRect(*cloud, {1.0f, -0.07f, 0.0f}, {0, 0.14f, 0}, {0, 0, 0.10f}, 14, 10);
  const std::size_t plate_pts = cloud->size();
  // far clutter, outside the crop box
  addRect(*cloud, {2.0f, 2.0f, 0.0f}, {0, 0.1f, 0}, {0, 0, 0.1f}, 10, 10);

  ouster_cluster::PlateRecoverParams p;
  p.xy_half = 0.12;
  p.tolerance = 0.03;
  p.min_points = 10;
  // seed: as if the segment came out short — centre is right, z-band correct
  auto rec = ouster_cluster::recoverPlateCluster(cloud, 1.0, 0.0, 0.0, 0.10, p);
  EXPECT_EQ(rec->size(), plate_pts);  // full plate back, clutter out
}

// The z-crop is the floor guard: the plate floats at z 0.03..0.13 above a
// floor at z=0 that IS within clustering tolerance of the plate's bottom.
// With a tight lower margin the floor never enters the input; with a huge
// lower margin (control) the cluster bridges into the floor.
TEST(PlateRecover, ZCropKeepsFloorOut)
{
  auto cloud = std::make_shared<Cloud>();
  addRect(*cloud, {1.0f, -0.07f, 0.03f}, {0, 0.14f, 0}, {0, 0, 0.10f}, 14, 10);
  const std::size_t plate_pts = cloud->size();
  // floor at z=0 covering the whole crop box, 1cm spacing
  addRect(*cloud, {0.85f, -0.15f, 0.0f}, {0.3f, 0, 0}, {0, 0.3f, 0}, 30, 30);

  ouster_cluster::PlateRecoverParams p;
  p.xy_half = 0.12;
  p.tolerance = 0.04;        // plate bottom (0.03) is within 0.04 of the floor
  p.min_points = 10;
  p.z_margin_below = 0.0;    // crop floor: z >= 0.03 -> floor (z=0) excluded
  auto rec = ouster_cluster::recoverPlateCluster(cloud, 1.0, 0.0, 0.03, 0.13, p);
  EXPECT_EQ(rec->size(), plate_pts);

  p.z_margin_below = 0.10;   // control: crop reaches the floor -> bridges
  auto bad = ouster_cluster::recoverPlateCluster(cloud, 1.0, 0.0, 0.03, 0.13, p);
  EXPECT_GT(bad->size(), plate_pts);
}

// Two disconnected clusters inside the crop: the plate at the seed and a
// DENSER blob near the crop edge. The recovery returns the cluster nearest
// the seed, not the bigger one.
TEST(PlateRecover, PicksSeedClusterNotTheBiggerOne)
{
  auto cloud = std::make_shared<Cloud>();
  addRect(*cloud, {1.0f, -0.07f, 0.0f}, {0, 0.14f, 0}, {0, 0, 0.10f}, 14, 10);
  const std::size_t plate_pts = cloud->size();
  // denser blob: y 0.105..0.115 (gap 0.035 > tolerance), inside xy_half 0.12
  addRect(*cloud, {1.0f, 0.105f, 0.0f}, {0, 0.01f, 0}, {0, 0, 0.10f}, 4, 40);

  ouster_cluster::PlateRecoverParams p;
  p.xy_half = 0.12;
  p.tolerance = 0.03;
  p.min_points = 10;
  auto rec = ouster_cluster::recoverPlateCluster(cloud, 1.0, 0.0, 0.0, 0.10, p);
  EXPECT_EQ(rec->size(), plate_pts);
}

// Mahalanobis trim: stray points chained onto the plate within clustering
// tolerance (so euclidean keeps them in the SAME cluster) but sticking out in
// the thickness direction are off the cluster's shape -> trimmed. With the
// trim disabled (threshold 0) they stay.
TEST(PlateRecover, MahalanobisTrimsOffShapeStrays)
{
  auto cloud = std::make_shared<Cloud>();
  // plate in the y-z plane at x=1.0 (thickness ~0)
  addRect(*cloud, {1.0f, -0.07f, 0.0f}, {0, 0.14f, 0}, {0, 0, 0.10f}, 14, 10);
  const std::size_t plate_pts = cloud->size();
  // stray chain off the plate face, 2cm spacing (within tolerance 0.03)
  for (float dx : {0.02f, 0.04f, 0.06f}) {
    pcl::PointXYZI pt; pt.x = 1.0f + dx; pt.y = 0.0f; pt.z = 0.05f; pt.intensity = 0;
    cloud->push_back(pt);
  }

  ouster_cluster::PlateRecoverParams p;
  p.xy_half = 0.12;
  p.tolerance = 0.03;
  p.min_points = 10;
  p.maha_threshold = 0.0;    // trim off: strays ride along
  auto raw = ouster_cluster::recoverPlateCluster(cloud, 1.0, 0.0, 0.0, 0.10, p);
  EXPECT_EQ(raw->size(), plate_pts + 3);

  p.maha_threshold = 7.815;  // chi^2 dof=3, 95%: off-shape strays trimmed
  auto trimmed = ouster_cluster::recoverPlateCluster(cloud, 1.0, 0.0, 0.0, 0.10, p);
  EXPECT_EQ(trimmed->size(), plate_pts);
}

// accumulateClouds concatenates a ring of frames into one cloud. Two frames
// whose scan rings are INTERLEAVED (5cm apart each, offset 2.5cm) are each too
// sparse to cluster at tolerance 0.03, but their union fills the gaps into one
// connected plate.
TEST(PlateRecover, AccumulatedFramesFillSparseRings)
{
  // a horizontal ring of 15 points along y at x=1.0, at height z
  auto addRing = [](Cloud & c, float z) {
      for (int i = 0; i <= 14; ++i) {
        pcl::PointXYZI p; p.x = 1.0f; p.y = -0.07f + 0.01f * i; p.z = z; p.intensity = 0;
        c.push_back(p);
      }
    };
  // frame A: rings at z 0, 0.05, 0.10 (5cm apart)
  auto a = std::make_shared<Cloud>();
  for (float z : {0.00f, 0.05f, 0.10f}) {addRing(*a, z);}
  // frame B: interleaved rings at z 0.025, 0.075 (fills A's gaps)
  auto b = std::make_shared<Cloud>();
  for (float z : {0.025f, 0.075f}) {addRing(*b, z);}

  ouster_cluster::PlateRecoverParams p;
  p.xy_half = 0.12;
  p.tolerance = 0.03;
  p.min_points = 10;
  p.maha_threshold = 0.0;  // isolate the accumulation effect

  // single frame A: rings 5cm apart > tolerance 0.03 -> split, no full plate
  auto single = ouster_cluster::recoverPlateCluster(a, 1.0, 0.0, 0.0, 0.10, p);
  EXPECT_LT(single->size(), a->size());  // could not connect all rings

  // accumulate A + B: interleaved -> gaps 2.5cm < tolerance -> one cluster
  auto acc = ouster_cluster::accumulateClouds({a, b});
  auto both = ouster_cluster::recoverPlateCluster(acc, 1.0, 0.0, 0.0, 0.10, p);
  EXPECT_EQ(both->size(), a->size() + b->size());  // whole plate recovered
}

// accumulateClouds on an empty list -> empty cloud, no crash.
TEST(PlateRecover, AccumulateEmptyList)
{
  auto out = ouster_cluster::accumulateClouds({});
  EXPECT_TRUE(out->empty());
}

// Too few points in the crop -> empty result, no crash.
TEST(PlateRecover, EmptyWhenTooFewPoints)
{
  auto cloud = std::make_shared<Cloud>();
  addRect(*cloud, {1.0f, 0.0f, 0.0f}, {0, 0.02f, 0}, {0, 0, 0.02f}, 1, 1);  // 4 pts

  ouster_cluster::PlateRecoverParams p;
  p.min_points = 10;
  auto rec = ouster_cluster::recoverPlateCluster(cloud, 1.0, 0.0, 0.0, 0.10, p);
  EXPECT_TRUE(rec->empty());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
