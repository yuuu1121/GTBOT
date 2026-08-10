#include <gtest/gtest.h>

#include "ouster_cluster/bev_density.hpp"

using ouster_cluster::Cloud;
using ouster_cluster::BevDensityParams;
using ouster_cluster::computeBevDensity;

namespace
{
void addPoint(Cloud & c, float x, float y, float z)
{
  pcl::PointXYZI p; p.x = x; p.y = y; p.z = z; p.intensity = 0;
  c.push_back(p);
}
}  // namespace

// Density = per-cell point count x squared range (sum of r^2 over the cell's
// points). A vertical structure stacks many points into one XY cell; the floor
// spreads its points across many cells, so per-cell density stays low.
TEST(BevDensity, StackedColumnBeatsSpreadFloorAtSameRange)
{
  auto cloud = std::make_shared<Cloud>();
  // vertical column at (1.0, 0.5): 20 points stacked in z -> one XY cell
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.0f, 0.5f, -0.5f + 0.05f * i);}
  // floor at similar range: 20 points spread over 20 distinct cells
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.0f + 0.10f * i, -1.0f, -0.7f);}

  BevDensityParams p;
  p.cell_size = 0.05;
  p.min_density = 0.0;
  auto out = computeBevDensity(cloud, p);
  ASSERT_EQ(out->size(), cloud->size());

  float column = 0.f, floor_max = 0.f;
  for (const auto & pt : out->points) {
    if (pt.y > 0.f) {column = pt.intensity;} else {floor_max = std::max(floor_max, pt.intensity);}
  }
  EXPECT_GT(column, 2.f * floor_max);  // stacked cell dominates any single floor cell
}

// count x dist^2 is distance-invariant: the same physical structure yields a
// similar density near and far, even though the far one has ~1/r^2 fewer points.
TEST(BevDensity, NormalizedDensityIsDistanceInvariant)
{
  auto cloud = std::make_shared<Cloud>();
  // near column at r=1.0m: 25 points
  for (int i = 0; i < 25; ++i) {addPoint(*cloud, 1.0f, 0.0f, -0.5f + 0.04f * i);}
  // far column at r=2.5m, same physical size: 1/r^2 scaling -> 4 points
  for (int i = 0; i < 4; ++i) {addPoint(*cloud, 2.5f, 0.0f, -0.5f + 0.25f * i);}

  BevDensityParams p;
  p.cell_size = 0.05;
  auto out = computeBevDensity(cloud, p);

  float near_d = -1.f, far_d = -1.f;
  for (const auto & pt : out->points) {
    if (pt.x < 2.0f) {near_d = pt.intensity;} else {far_d = pt.intensity;}
  }
  ASSERT_GT(near_d, 0.f);
  ASSERT_GT(far_d, 0.f);
  // 25 x 1.0^2 = 25 vs 4 x 2.5^2 = 25
  EXPECT_NEAR(near_d, far_d, 0.15f * near_d);
}

// A single stray noise point sits alone in its cell: density = 1 x r^2, far
// below a real structure's cell, so min_density removes it. (This is the
// failure mode of the z-extent metric, where one stray point inflated a cell.)
TEST(BevDensity, MinDensityDropsLoneNoisePoint)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.0f, 0.5f, -0.5f + 0.05f * i);}  // structure
  addPoint(*cloud, 2.0f, 1.0f, 0.9f);  // lone stray: density = 1 x 5.0 = 5

  BevDensityParams p;
  p.cell_size = 0.05;
  p.min_density = 10.0;  // structure cell: 20 x 1.25 = 25 -> kept
  auto out = computeBevDensity(cloud, p);

  EXPECT_EQ(out->size(), 20u);  // stray dropped, structure intact
  for (const auto & pt : out->points) {EXPECT_GE(pt.intensity, 10.f);}
}

// max_density drops over-dense cells (band-pass: keep plate-like mid densities,
// cut big face-on walls that would otherwise dominate the line extraction).
TEST(BevDensity, MaxDensityDropsOverDenseCells)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.0f, 0.5f, -0.5f + 0.05f * i);}  // plate: 20x1.25=25
  for (int i = 0; i < 100; ++i) {addPoint(*cloud, 2.0f, -0.5f, -0.5f + 0.01f * i);}  // wall: 100x4.25=425

  BevDensityParams p;
  p.cell_size = 0.05;
  p.min_density = 5.0;
  p.max_density = 90.0;
  auto out = computeBevDensity(cloud, p);

  EXPECT_EQ(out->size(), 20u);  // wall cell cut by the max, plate kept
  for (const auto & pt : out->points) {EXPECT_LT(pt.x, 1.5f);}
}

// z-bin occupancy: a floor cell has all its points at one height -> only ONE
// occupied z-bin -> dropped, no matter how dense it is. A vertical structure
// spans several z-bins -> kept. This is what separates structures from floor.
TEST(BevDensity, ZOccupancyDropsFloorKeepsColumn)
{
  auto cloud = std::make_shared<Cloud>();
  // vertical column at (1.0, 0.5): 20 points over z=-0.5..0.45 (many z-bins)
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.0f, 0.5f, -0.5f + 0.05f * i);}
  // dense floor cell at (1.0, -0.5): 30 points, all z=-0.7 (one z-bin)
  for (int i = 0; i < 30; ++i) {addPoint(*cloud, 1.0f, -0.5f, -0.7f);}

  BevDensityParams p;
  p.cell_size = 0.05;
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;
  auto out = computeBevDensity(cloud, p);

  EXPECT_EQ(out->size(), 20u);  // column kept, dense floor cell dropped
  for (const auto & pt : out->points) {EXPECT_GT(pt.y, 0.f);}
}

// A lone stray point above a floor cell does NOT rescue it: its z-bin has only
// 1 point (< min_pts_per_bin) so it never counts as occupied. (This was the
// failure mode that killed the raw z-extent metric.)
TEST(BevDensity, LoneStrayDoesNotFakeVerticality)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 30; ++i) {addPoint(*cloud, 1.0f, -0.5f, -0.7f);}  // floor cell
  addPoint(*cloud, 1.0f, -0.5f, 0.3f);  // one stray 1m above, SAME cell

  BevDensityParams p;
  p.cell_size = 0.05;
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;
  auto out = computeBevDensity(cloud, p);

  EXPECT_TRUE(out->empty());  // still one occupied bin -> whole cell dropped
}

// min_z_bins = 0 disables the occupancy check entirely (density band only).
TEST(BevDensity, ZOccupancyDisabledKeepsFloor)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 30; ++i) {addPoint(*cloud, 1.0f, -0.5f, -0.7f);}  // floor cell

  BevDensityParams p;
  p.cell_size = 0.05;
  p.min_z_bins = 0;
  auto out = computeBevDensity(cloud, p);
  EXPECT_EQ(out->size(), 30u);
}

// Empty input -> empty output, no crash.
TEST(BevDensity, EmptyInput)
{
  auto cloud = std::make_shared<Cloud>();
  BevDensityParams p;
  auto out = computeBevDensity(cloud, p);
  EXPECT_TRUE(out->empty());
}

// A structure straddling a cell edge splits its points across two cells; each
// half can miss min_density even though the whole passes. The half-cell-shifted
// second grid holds the whole structure in one cell and rescues it (OR-combine).
TEST(BevDensity, BoundarySplitClusterRescuedByDualGrid)
{
  auto cloud = std::make_shared<Cloud>();
  // 12 points at r~1m straddling the x=1.000 cell edge (cell 0.025): 6 left,
  // 6 right, each side spanning 2 z-bins so only the density gate can fail.
  for (int i = 0; i < 3; ++i) {
    addPoint(*cloud, 0.999f, 0.01f, 0.01f + 0.001f * i);
    addPoint(*cloud, 0.999f, 0.01f, 0.06f + 0.001f * i);
    addPoint(*cloud, 1.001f, 0.01f, 0.01f + 0.001f * i);
    addPoint(*cloud, 1.001f, 0.01f, 0.06f + 0.001f * i);
  }

  BevDensityParams p;
  p.cell_size = 0.025;
  p.min_density = 10.0;  // each split half: 6 x r^2 ~ 6 < 10; whole: 12 >= 10
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;

  p.dual_grid = false;
  EXPECT_TRUE(computeBevDensity(cloud, p)->empty());  // split kills both halves

  p.dual_grid = true;
  auto out = computeBevDensity(cloud, p);
  EXPECT_EQ(out->size(), 12u);  // shifted grid sees the whole cluster
  for (const auto & pt : out->points) {
    EXPECT_GE(pt.intensity, 10.f);  // intensity = density of the passing grid
  }
}

// Dual grid must not resurrect genuinely weak cells: a lone point fails in
// both grids.
TEST(BevDensity, DualGridStillDropsLoneNoise)
{
  auto cloud = std::make_shared<Cloud>();
  addPoint(*cloud, 1.5f, 0.7f, -0.2f);

  BevDensityParams p;
  p.cell_size = 0.025;
  p.min_density = 10.0;
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;
  p.dual_grid = true;
  EXPECT_TRUE(computeBevDensity(cloud, p)->empty());
}

// With dual_grid off the behavior is the original single-grid one: a cell that
// passes on the primary grid keeps exactly its own points.
TEST(BevDensity, DualGridOffIsUnchangedForIntactCell)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 6; ++i) {
    addPoint(*cloud, 1.01f, 0.01f, 0.01f + 0.001f * i);
    addPoint(*cloud, 1.01f, 0.01f, 0.06f + 0.001f * i);
  }
  BevDensityParams p;
  p.cell_size = 0.025;
  p.min_density = 10.0;
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;
  p.dual_grid = false;
  EXPECT_EQ(computeBevDensity(cloud, p)->size(), 12u);
}

// drop_lonely: a surviving cell with NO surviving cell in its 8-neighbourhood
// is isolated noise and is removed; a real structure spans several adjacent
// cells, so its cells keep each other alive.
TEST(BevDensity, DropLonelyRemovesIsolatedCellKeepsStructure)
{
  auto cloud = std::make_shared<Cloud>();
  // structure: two ADJACENT cells (x cells 20 and 21 at cell 0.05), each a
  // proper column (passes density + z-bin gates on its own)
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.01f, 0.51f, -0.5f + 0.05f * i);}
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.06f, 0.51f, -0.5f + 0.05f * i);}
  // lonely column far away: passes the same gates but has no neighbour cell
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 2.01f, -1.49f, -0.5f + 0.05f * i);}

  BevDensityParams p;
  p.cell_size = 0.05;
  p.min_density = 10.0;
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;
  p.drop_lonely = true;
  auto out = computeBevDensity(cloud, p);

  EXPECT_EQ(out->size(), 40u);  // lonely cell dropped, both structure cells kept
  for (const auto & pt : out->points) {EXPECT_LT(pt.x, 1.5f);}
}

// drop_lonely off (default): the isolated cell survives as before.
TEST(BevDensity, LonelyCellKeptWhenDisabled)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 2.01f, -1.49f, -0.5f + 0.05f * i);}

  BevDensityParams p;
  p.cell_size = 0.05;
  p.min_density = 10.0;
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;
  auto out = computeBevDensity(cloud, p);
  EXPECT_EQ(out->size(), 20u);
}

// A diagonal neighbour counts as one of the 8: two cells touching only at a
// corner still keep each other alive.
TEST(BevDensity, DiagonalNeighbourSavesCell)
{
  auto cloud = std::make_shared<Cloud>();
  // cells (20,10) and (21,11): diagonal contact
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.01f, 0.51f, -0.5f + 0.05f * i);}
  for (int i = 0; i < 20; ++i) {addPoint(*cloud, 1.06f, 0.56f, -0.5f + 0.05f * i);}

  BevDensityParams p;
  p.cell_size = 0.05;
  p.min_density = 10.0;
  p.z_bin = 0.05;
  p.min_pts_per_bin = 2;
  p.min_z_bins = 2;
  p.drop_lonely = true;
  auto out = computeBevDensity(cloud, p);
  EXPECT_EQ(out->size(), 40u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
