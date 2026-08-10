#include <gtest/gtest.h>

#include <cmath>

#include "ouster_cluster/bev_lines.hpp"

using ouster_cluster::Cloud;
using ouster_cluster::BevLineParams;
using ouster_cluster::BevSegment;
using ouster_cluster::extractBevLineSegments;

namespace
{
void addPoint(Cloud & c, float x, float y, float z = 0.f)
{
  pcl::PointXYZI p; p.x = x; p.y = y; p.z = z; p.intensity = 0;
  c.push_back(p);
}

double segLength(const BevSegment & s)
{
  return std::hypot(s.x1 - s.x0, s.y1 - s.y0);
}
}  // namespace

// A contiguous straight run of cells (a plate/wall in BEV) yields one segment
// of roughly the right length and position.
TEST(BevLines, StraightRunYieldsOneSegment)
{
  auto cloud = std::make_shared<Cloud>();
  // 5 adjacent cells in a row along +x (mid-cell coords, 5cm apart)
  for (int i = 0; i < 5; ++i) {addPoint(*cloud, 1.02f + 0.05f * i, 0.52f);}

  BevLineParams p;
  p.cell_size = 0.05;
  auto segs = extractBevLineSegments(cloud, p);

  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(segLength(segs[0]), 0.20, 0.06);  // 5 cells -> ~20cm span
  EXPECT_NEAR(0.5 * (segs[0].y0 + segs[0].y1), 0.5, 0.05);
}

// Two well-separated contiguous runs yield two segments.
TEST(BevLines, TwoRunsYieldTwoSegments)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 5; ++i) {addPoint(*cloud, 1.02f + 0.05f * i, 0.52f);}   // run A along x
  for (int i = 0; i < 5; ++i) {addPoint(*cloud, 2.02f, -0.98f + 0.05f * i);}  // run B along y

  BevLineParams p;
  p.cell_size = 0.05;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_EQ(segs.size(), 2u);
}

// A gap breaks connectivity: one collinear line with a hole becomes two
// separate segments (components), not one long one.
TEST(BevLines, GapSplitsCollinearRun)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 4; ++i) {addPoint(*cloud, 1.02f + 0.05f * i, 0.02f);}
  for (int i = 0; i < 4; ++i) {addPoint(*cloud, 2.02f + 0.05f * i, 0.02f);}  // 85cm hole

  BevLineParams p;
  p.cell_size = 0.05;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_EQ(segs.size(), 2u);
  for (const auto & s : segs) {EXPECT_LT(segLength(s), 0.4);}
}

// THE bug this redesign fixes: cells that happen to be collinear but are far
// apart (scattered floor residue) must NOT form a segment — they are not one
// connected structure.
TEST(BevLines, CollinearButDisconnectedYieldsNothing)
{
  auto cloud = std::make_shared<Cloud>();
  addPoint(*cloud, 1.02f, 0.02f);   // three cells on the exact same line
  addPoint(*cloud, 1.52f, 0.02f);   // but 50cm apart from each other
  addPoint(*cloud, 2.02f, 0.02f);

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 3;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_TRUE(segs.empty());
}

// A fat blob (floor residue patch) is connected but not line-like: its width
// across the major axis exceeds max_width -> rejected.
TEST(BevLines, BlobYieldsNothing)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f + 0.05f * j);  // 5x5 = 25cm x 25cm patch
    }
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.max_width = 0.12;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_TRUE(segs.empty());
}

// --- point-level majority-touch connectivity (touch_connect) ----------------

// Same object, but the lower half is occluded along one section: the per-cell
// z-MEDIAN jumps there and the median rule cuts the line in two. The touch
// rule connects because every point of the occluded cell finds a companion
// (within touch_gap in z) in the fully visible neighbour -> ONE segment.
TEST(BevLines, TouchConnectSurvivesMedianJump)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 5; ++i) {   // full-height section, median 0.10
    for (float z : {0.00f, 0.05f, 0.10f, 0.15f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }
  for (int i = 5; i < 10; ++i) {  // occluded section, median 0.15
    for (float z : {0.10f, 0.15f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.top_tol = 0.0;
  p.z_med_gap = 0.04;   // median diff 0.05 > 0.04 -> the old rule splits
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 2u);

  p.touch_connect = true;  // occluded side: 2/2 points matched -> connected
  p.touch_gap = 0.03;
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 1u);
}

// Two runs at clearly different heights: no point of either cell finds a
// companion within touch_gap -> they stay separate.
TEST(BevLines, TouchConnectKeepsDifferentHeightsApart)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 4; ++i) {   // low run
    for (float z : {0.00f, 0.05f}) {addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);}
  }
  for (int i = 4; i < 8; ++i) {   // high run
    for (float z : {0.20f, 0.25f}) {addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);}
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.top_tol = 0.0;
  p.touch_connect = true;
  p.touch_gap = 0.03;
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 2u);
}

// touch_xy_gap adds a horizontal-distance condition to the companion check:
// two cells that share a z-band but whose matching points sit far apart in
// XY (e.g. across a gap that the coarse 2cm grid still calls "adjacent" at
// a cell corner) must NOT connect. Companions now need BOTH |dz| <= touch_gap
// AND horizontal distance <= touch_xy_gap.
TEST(BevLines, TouchXyGapRejectsHorizontallyDistantCompanions)
{
  auto cloud = std::make_shared<Cloud>();
  // left run: same z-band, clustered near x=1.02..1.17
  for (int i = 0; i < 4; ++i) {
    for (float z : {0.00f, 0.05f, 0.10f, 0.15f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }
  // right run: adjacent cell in the grid (dx=1), SAME z-band, but its points
  // sit at the far edge of that cell -> horizontally distant from the left
  // run's nearest points despite being nominally "adjacent"
  for (int i = 4; i < 8; ++i) {
    for (float z : {0.00f, 0.05f, 0.10f, 0.15f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.top_tol = 0.0;
  p.min_cells = 1;    // isolate the connectivity question from min_cells
  p.min_length = 0.0; // ...and from min_length (a 1-cell component is a point)
  p.touch_connect = true;
  p.touch_gap = 0.03;
  p.touch_min_matched = 2;
  p.touch_xy_gap = 0.0;   // 0 = disabled -> same z-band alone connects
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 1u);

  p.touch_xy_gap = 0.02;  // tight: matching points must also be close in XY.
                          // Consecutive points here are 0.05m apart in x, so
                          // no companion satisfies both conditions -> the
                          // run splits into per-cell singletons
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 8u);
}

// touch_min_ratio makes the "majority" fraction itself tunable (default 0.5
// preserves the old strict-majority behaviour). 2 of 5 matched (0.4) fails at
// ratio 0.5 but passes once the ratio is lowered to 0.3.
TEST(BevLines, TouchMinRatioIsTunable)
{
  auto cloud = std::make_shared<Cloud>();
  // left run: 5 points per cell, full height
  for (int i = 0; i < 4; ++i) {
    for (float z : {0.00f, 0.05f, 0.10f, 0.15f, 0.20f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }
  // right run: overlaps the left run's z-band at only 2 of its 5 levels
  // (0.00, 0.05) -> A->B matched = 2/5 = 0.4
  for (int i = 4; i < 8; ++i) {
    for (float z : {0.00f, 0.05f, 0.30f, 0.35f, 0.40f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.top_tol = 0.0;
  p.top_step = 0.0;  // disable top-step splitting — isolate the connectivity
                     // question from it (the two runs' tops differ by 0.2m,
                     // far more than the default 0.07 split threshold)
  p.touch_connect = true;
  p.touch_gap = 0.01;
  p.touch_min_matched = 2;
  p.touch_min_ratio = 0.5;   // default: needs > 50% -> 2/5 (0.4) fails
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 2u);

  p.touch_min_ratio = 0.3;   // relaxed: needs > 30% -> 2/5 (0.4) passes
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 1u);
}

// A SINGLE-point cell trivially satisfies "majority" (1/1) and would chain two
// structures as a stepping stone. touch_min_matched(2) removes its vote: a
// direction also needs at least 2 matched points, so the bridge stays isolated
// and the two runs stay apart. (The occluded 2-point cell in
// TouchConnectSurvivesMedianJump still passes: both its points match.)
TEST(BevLines, TouchConnectSinglePointCellCannotBridge)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 4; ++i) {   // left run, full height
    for (float z : {0.00f, 0.05f, 0.10f, 0.15f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }
  addPoint(*cloud, 1.22f, 0.02f, 0.15f);  // lone stepping-stone cell
  for (int i = 5; i < 9; ++i) {   // right run, different band but shares 0.15
    for (float z : {0.15f, 0.25f, 0.35f}) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);
    }
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.top_tol = 0.0;
  p.touch_connect = true;
  p.touch_gap = 0.03;
  p.touch_min_matched = 2;
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 2u);
}

// MAJORITY rule: one touching pair is not enough. Both runs overlap only at
// z=0.10 (1 of 3 points matched in each direction, half or less) -> cut. A
// min-pair rule would have glued them through that single stratum.
TEST(BevLines, TouchConnectNeedsMajorityNotOnePair)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 4; ++i) {
    for (float z : {0.00f, 0.05f, 0.10f}) {addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);}
  }
  for (int i = 4; i < 8; ++i) {
    for (float z : {0.10f, 0.20f, 0.30f}) {addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, z);}
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.top_tol = 0.0;
  p.touch_connect = true;
  p.touch_gap = 0.03;
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 2u);
}

namespace
{
BevSegment seg(float x0, float y0, float x1, float y1)
{
  return ouster_cluster::BevSegment{x0, y0, x1, y1, 5};
}
}  // namespace

// A segment is only published after confirm_frames consecutive appearances.
TEST(BevSegFilter, NotShownUntilConfirmed)
{
  ouster_cluster::BevSegmentFilter f;
  ouster_cluster::BevSegFilterParams p;
  p.confirm_frames = 3;
  f.setParams(p);

  const auto s = seg(1.0f, 0.0f, 1.14f, 0.0f);
  EXPECT_TRUE(f.update({s}).empty());       // frame 1
  EXPECT_TRUE(f.update({s}).empty());       // frame 2
  EXPECT_EQ(f.update({s}).size(), 1u);      // frame 3: confirmed
}

// A confirmed segment survives short dropouts (held while missed <= max_missed).
TEST(BevSegFilter, ConfirmedSegmentHeldThroughDropout)
{
  ouster_cluster::BevSegmentFilter f;
  ouster_cluster::BevSegFilterParams p;
  p.confirm_frames = 3;
  p.max_missed = 2;
  f.setParams(p);

  const auto s = seg(1.0f, 0.0f, 1.14f, 0.0f);
  f.update({s}); f.update({s}); f.update({s});  // confirmed
  EXPECT_EQ(f.update({}).size(), 1u);           // missed 1: still shown
  EXPECT_EQ(f.update({}).size(), 1u);           // missed 2: still shown
  EXPECT_EQ(f.update({s}).size(), 1u);          // reappears: keeps going
}

// A flickering noise segment (1-2 frames, then gone) is never published.
TEST(BevSegFilter, FlickerNeverConfirmed)
{
  ouster_cluster::BevSegmentFilter f;
  ouster_cluster::BevSegFilterParams p;
  p.confirm_frames = 3;
  f.setParams(p);

  const auto s = seg(2.0f, 1.0f, 2.1f, 1.0f);
  EXPECT_TRUE(f.update({s}).empty());
  EXPECT_TRUE(f.update({}).empty());
  EXPECT_TRUE(f.update({s}).empty());  // non-consecutive: counter reset
  EXPECT_TRUE(f.update({}).empty());
}

// A confirmed segment absent for more than max_missed frames expires.
TEST(BevSegFilter, ExpiresAfterMaxMissed)
{
  ouster_cluster::BevSegmentFilter f;
  ouster_cluster::BevSegFilterParams p;
  p.confirm_frames = 3;
  p.max_missed = 2;
  f.setParams(p);

  const auto s = seg(1.0f, 0.0f, 1.14f, 0.0f);
  f.update({s}); f.update({s}); f.update({s});
  f.update({}); f.update({});
  EXPECT_TRUE(f.update({}).empty());   // missed 3 > max_missed: gone
  EXPECT_TRUE(f.update({s}).empty());  // and must re-confirm from scratch
}

// A segment that drifts slightly (< match_distance) stays the same track.
TEST(BevSegFilter, SmallDriftKeepsTrack)
{
  ouster_cluster::BevSegmentFilter f;
  ouster_cluster::BevSegFilterParams p;
  p.confirm_frames = 3;
  p.match_distance = 0.15;
  f.setParams(p);

  f.update({seg(1.00f, 0.00f, 1.14f, 0.00f)});
  f.update({seg(1.03f, 0.02f, 1.17f, 0.02f)});
  EXPECT_EQ(f.update({seg(1.01f, -0.02f, 1.15f, -0.02f)}).size(), 1u);
}

// XY-adjacent cells at clearly different heights must NOT merge into one
// component: a thin plate passing near a lower structure keeps its own
// line-shaped component (this is the "plate near a low line" failure).
TEST(BevLines, DifferentHeightNeighboursStaySeparate)
{
  auto cloud = std::make_shared<Cloud>();
  // plate row: 6 cells along x at y=0.52, points UP HIGH (z ~ 0.0..0.1)
  for (int i = 0; i < 6; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.0f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.1f);
  }
  // low blob DIRECTLY ADJACENT (rows y=0.57..0.67), points LOW (z ~ -0.6)
  for (int i = 0; i < 6; ++i) {
    for (int j = 1; j <= 3; ++j) {
      addPoint(*cloud, 1.02f + 0.05f * i, 0.52f + 0.05f * j, -0.6f);
    }
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 5;
  p.max_width = 0.07;
  p.z_gap = 0.1;  // 0.1m max height gap to connect
  auto segs = extractBevLineSegments(cloud, p);

  // with height-aware connectivity the plate row is its own thin component
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(0.5 * (segs[0].y0 + segs[0].y1), 0.52, 0.05);
  // and the segment carries its height band
  EXPECT_NEAR(segs[0].z_min, 0.0, 0.03);
  EXPECT_NEAR(segs[0].z_max, 0.1, 0.03);
}

// Cells at the SAME height connect as before (one segment across them).
TEST(BevLines, SameHeightNeighboursStillConnect)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 6; ++i) {addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.05f);}

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 5;
  p.z_gap = 0.1;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_EQ(segs.size(), 1u);
}

// Plate standing ON the desk, its end at the desk edge, roughly collinear:
// z-ranges touch so the cells merge into ONE component. The top-height
// profile along the line has a persistent 10cm step there — the component
// is SPLIT at that step and each side is judged on its own.
TEST(BevLines, TopStepSplitsMergedPlateAndDeskEdge)
{
  auto cloud = std::make_shared<Cloud>();
  // plate: 6 cells, points z=-0.10..0.00 (top 0.00)
  for (int i = 0; i < 6; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, -0.10f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, 0.00f);
  }
  // boundary cell: points of both (top = plate top)
  addPoint(*cloud, 1.32f, 0.02f, 0.00f);
  addPoint(*cloud, 1.32f, 0.02f, -0.20f);
  // desk edge continuing the line: 6 cells, points z=-0.20..-0.10 (top -0.10)
  for (int i = 0; i < 6; ++i) {
    addPoint(*cloud, 1.37f + 0.05f * i, 0.02f, -0.20f);
    addPoint(*cloud, 1.37f + 0.05f * i, 0.02f, -0.10f);
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 5;
  p.z_gap = 0.05;
  p.top_tol = 0.06;
  p.top_step = 0.07;
  auto segs = extractBevLineSegments(cloud, p);

  ASSERT_EQ(segs.size(), 2u);  // plate(+boundary) and desk edge, separately
  for (const auto & s : segs) {
    EXPECT_LT(std::hypot(s.x1 - s.x0, s.y1 - s.y0), 0.40);  // neither spans both
  }
}

// A MIXED boundary cell (plate + desk points in one cell) has the plate's top,
// so no top-based check can see it — but it must not drag the segment's z-band
// down (that flipped the plate verdict green->magenta). The band uses per-cell
// medians, so one polluted cell changes nothing.
TEST(BevLines, MixedCellDoesNotDragZBand)
{
  auto cloud = std::make_shared<Cloud>();
  // plate: 12 cells, z=-0.28..-0.18 (the real captured values)
  for (int i = 0; i < 12; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, -0.28f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, -0.18f);
  }
  // one mixed cell at the end: plate points AND desk points down to -0.48
  addPoint(*cloud, 1.62f, 0.02f, -0.18f);
  addPoint(*cloud, 1.62f, 0.02f, -0.48f);

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 5;
  p.top_tol = 0.06;
  p.top_step = 0.07;
  auto segs = extractBevLineSegments(cloud, p);

  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(segs[0].z_min, -0.28, 0.02);  // NOT -0.48
  EXPECT_NEAR(segs[0].z_max, -0.18, 0.02);
}

// The same step detector must NOT split on a single cell whose top dips
// (frame-to-frame ring dropout): the median filter absorbs isolated jitter.
TEST(BevLines, SingleTopDipDoesNotSplit)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 7; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, -0.10f);
    if (i != 3) {addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, 0.00f);}
    // cell 3 misses its top row this frame -> its top is 10cm lower
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 5;
  p.top_tol = 0.06;
  p.top_step = 0.07;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_EQ(segs.size(), 1u);  // still one intact segment
}

// A component whose per-cell TOP heights rise gradually (a staircase chain —
// each neighbour within z_gap, but the top drifts along the line) is NOT a
// real flat-topped structure: rejected by the top-uniformity check.
TEST(BevLines, SlopedTopChainRejected)
{
  auto cloud = std::make_shared<Cloud>();
  // 6 cells along x; cell i holds a 10cm-tall stack whose top rises 4cm/cell
  for (int i = 0; i < 6; ++i) {
    const float top = -0.40f + 0.04f * i;   // tops: -0.40 .. -0.20 (spread 0.20)
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, top - 0.10f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, top - 0.05f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, top);
  }

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 5;
  p.z_gap = 0.05;      // neighbours still connect (4cm steps overlap)
  p.top_tol = 0.06;    // but the top drifts 20cm along the line -> reject
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_TRUE(segs.empty());
}

// A flat-topped run passes even when ONE cell carries a stray high point:
// the trimmed spread ignores a single outlier cell.
TEST(BevLines, FlatTopWithOneStrayCellStillPasses)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 6; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, -0.50f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.02f, -0.40f);  // flat top at -0.40
  }
  addPoint(*cloud, 1.02f, 0.02f, -0.10f);  // stray 30cm above, first cell only

  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 5;
  p.top_tol = 0.06;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_EQ(segs.size(), 1u);
}

// --- point-level refinement -------------------------------------------------

// Refit on raw points recovers the true line from a coarse (cell-quantized,
// slightly offset) first-pass segment.
TEST(BevRefine, RefitRecoversTrueLine)
{
  auto cloud = std::make_shared<Cloud>();
  // 70 points on the true line y=0, x in [1.0, 1.14], tiny alternating jitter
  for (int i = 0; i < 70; ++i) {
    addPoint(*cloud, 1.0f + 0.002f * i, (i % 2 ? 0.004f : -0.004f));
  }
  // coarse segment: offset 2cm in y and slightly short (cell quantization)
  const auto coarse = seg(1.02f, 0.02f, 1.12f, 0.02f);

  ouster_cluster::BevRefineParams p;
  const auto r = ouster_cluster::refineBevSegment(coarse, cloud, p);

  EXPECT_NEAR(0.5 * (r.y0 + r.y1), 0.0, 0.005);   // pulled onto the true line
  EXPECT_NEAR(std::min(r.x0, r.x1), 1.0, 0.02);   // endpoints span the real extent
  EXPECT_NEAR(std::max(r.x0, r.x1), 1.14, 0.02);
}

// A clump of outliers at one end (inside the gate band) must not tilt the
// line: MAD trimming removes them before the final fit.
TEST(BevRefine, OutlierClumpDoesNotTiltLine)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 60; ++i) {addPoint(*cloud, 1.0f + 0.0024f * i, 0.0f);}  // true line
  for (int i = 0; i < 10; ++i) {addPoint(*cloud, 1.13f + 0.001f * i, 0.045f);}  // clump at end

  const auto coarse = seg(1.0f, 0.0f, 1.14f, 0.0f);
  ouster_cluster::BevRefineParams p;
  p.band = 0.05;
  const auto r = ouster_cluster::refineBevSegment(coarse, cloud, p);

  EXPECT_NEAR(r.y0, 0.0, 0.01);
  EXPECT_NEAR(r.y1, 0.0, 0.01);  // untrimmed fit would tilt towards the clump
}

// A lone collinear point beyond the segment end must not stretch it:
// percentile endpoints ignore it.
TEST(BevRefine, LoneExtensionDoesNotStretchSegment)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 50; ++i) {addPoint(*cloud, 1.0f + 0.0028f * i, 0.0f);}  // [1.0, 1.14]
  addPoint(*cloud, 1.30f, 0.0f);  // lone stray on the same line, 16cm past the end

  const auto coarse = seg(1.0f, 0.0f, 1.35f, 0.0f);  // coarse seg reaches the stray
  ouster_cluster::BevRefineParams p;
  const auto r = ouster_cluster::refineBevSegment(coarse, cloud, p);

  EXPECT_LT(std::max(r.x0, r.x1), 1.20f);  // percentile endpoint cuts the stray
}

// Too few points near the segment -> fall back to the coarse segment unchanged.
TEST(BevRefine, FallbackWhenTooFewPoints)
{
  auto cloud = std::make_shared<Cloud>();
  addPoint(*cloud, 1.05f, 0.0f);  // 1 point < min_points

  const auto coarse = seg(1.0f, 0.0f, 1.14f, 0.0f);
  ouster_cluster::BevRefineParams p;
  const auto r = ouster_cluster::refineBevSegment(coarse, cloud, p);

  EXPECT_FLOAT_EQ(r.x0, coarse.x0);
  EXPECT_FLOAT_EQ(r.y0, coarse.y0);
  EXPECT_FLOAT_EQ(r.x1, coarse.x1);
  EXPECT_FLOAT_EQ(r.y1, coarse.y1);
}

// --- target plate shape check -----------------------------------------------

namespace
{
BevSegment segWithZ(float len, float z_min, float z_max)
{
  return ouster_cluster::BevSegment{1.0f, 0.0f, 1.0f + len, 0.0f, 6, z_min, z_max};
}
}  // namespace

// A segment matching the 14cm x 10cm target passes; wrong length or wrong
// height fails; sizes within tolerance pass.
TEST(BevPlate, MatchesTargetSize)
{
  ouster_cluster::BevPlateParams p;  // 0.14 x 0.10, tol_width 0.05 / tol_height 0.02
  EXPECT_TRUE(isPlateSegment(segWithZ(0.14f, -0.30f, -0.20f), p));   // exact
  EXPECT_TRUE(isPlateSegment(segWithZ(0.12f, -0.30f, -0.22f), p));   // within tol
  EXPECT_FALSE(isPlateSegment(segWithZ(0.30f, -0.30f, -0.20f), p));  // too long (wall piece)
  EXPECT_FALSE(isPlateSegment(segWithZ(0.14f, -0.30f, 0.20f), p));   // too tall
  EXPECT_FALSE(isPlateSegment(segWithZ(0.14f, -0.30f, -0.29f), p));  // too flat
}

// The plate is a thin sheet (<= 3cm): a segment whose perpendicular extent
// (width) exceeds thickness_max is a box/blob, not the plate. <= 0 disables.
TEST(BevPlate, ThickSegmentRejected)
{
  ouster_cluster::BevPlateParams p;  // thickness_max 0.05
  auto seg = segWithZ(0.14f, -0.30f, -0.20f);   // size matches the target
  seg.width = 0.03f;
  EXPECT_TRUE(isPlateSegment(seg, p));    // thin -> plate
  seg.width = 0.07f;
  EXPECT_FALSE(isPlateSegment(seg, p));   // thick -> not a plate
  p.thickness_max = 0.0;
  EXPECT_TRUE(isPlateSegment(seg, p));    // gate disabled
}

// Extraction records the component's cell width into the segment, and refine
// replaces it with the point-level perpendicular spread (tighter than cells).
TEST(BevLines, SegmentCarriesWidth)
{
  auto cloud = std::make_shared<Cloud>();
  // thin vertical wall along x at y=0.51: 10 cells long, 1 cell wide
  for (int i = 0; i < 10; ++i) {
    for (int k = 0; k < 4; ++k) {
      addPoint(*cloud, 1.01f + 0.05f * i, 0.51f, -0.10f + 0.05f * k);
    }
  }
  BevLineParams p;
  p.cell_size = 0.05;
  p.min_cells = 3;
  p.max_width = 0.12;
  p.min_length = 0.05;
  auto segs = extractBevLineSegments(cloud, p);
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_LE(segs[0].width, 0.06f);   // coarse: about one cell
  EXPECT_GE(segs[0].width, 0.0f);

  ouster_cluster::BevRefineParams rp;
  auto refined = refineBevSegment(segs[0], cloud, rp);
  EXPECT_LE(refined.width, 0.02f);   // points are perfectly collinear
}

// Length (BEV extent) and height (z-band) tolerances are INDEPENDENT: the
// length may drift +/-tol_width while the height must stay within the tighter
// +/-tol_height. One loose gate must not loosen the other.
TEST(BevPlate, SeparateLengthAndHeightTolerances)
{
  ouster_cluster::BevPlateParams p;
  p.tol_width = 0.05;
  p.tol_height = 0.02;
  EXPECT_TRUE(isPlateSegment(segWithZ(0.18f, -0.30f, -0.20f), p));    // len +0.04 <= 0.05
  EXPECT_FALSE(isPlateSegment(segWithZ(0.20f, -0.30f, -0.20f), p));   // len +0.06 >  0.05
  EXPECT_TRUE(isPlateSegment(segWithZ(0.14f, -0.30f, -0.185f), p));   // h +0.015 <= 0.02
  EXPECT_FALSE(isPlateSegment(segWithZ(0.14f, -0.30f, -0.17f), p));   // h +0.03  >  0.02
  // a length deviation acceptable to tol_width must not be judged by tol_height
  EXPECT_TRUE(isPlateSegment(segWithZ(0.10f, -0.30f, -0.20f), p));    // len -0.04 <= 0.05
}

// Empty input -> no segments, no crash.
TEST(BevLines, EmptyInput)
{
  auto cloud = std::make_shared<Cloud>();
  BevLineParams p;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_TRUE(segs.empty());
}

// Median-z connectivity: a single stray high point stretches a cell's z-RANGE
// enough to overlap a neighbour at a different height (the old range rule
// merges them), but the cell's MEDIAN height is unmoved -> stays split.
TEST(BevLines, MedianGapIgnoresStrayPointRangeRuleDoesNot)
{
  auto cloud = std::make_shared<Cloud>();
  // run A along x at low height (z ~ 0.00..0.10, median ~0.05)
  for (int i = 0; i < 5; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.00f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.05f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.10f);
  }
  // run B: the ADJACENT row of cells at a clearly different height
  // (z ~ 0.30..0.40, median ~0.35)
  for (int i = 0; i < 5; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.57f, 0.30f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.57f, 0.35f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.57f, 0.40f);
  }
  // one stray point in run A's middle cell reaching up to run B's band:
  // A's z-range becomes [0, 0.35] -> range rule sees overlap and merges
  addPoint(*cloud, 1.12f, 0.52f, 0.35f);

  BevLineParams p;
  p.cell_size = 0.05;
  p.z_gap = 0.05;

  // old range rule: the stray point bridges A and B -> one wide component,
  // which fails the width gate -> nothing (or a merged mess)
  auto segs_range = extractBevLineSegments(cloud, p);
  EXPECT_LT(segs_range.size(), 2u);

  // median rule: |0.05 - 0.35| = 0.30 > 0.05 -> A and B stay separate lines
  p.z_med_gap = 0.05;
  auto segs_med = extractBevLineSegments(cloud, p);
  EXPECT_EQ(segs_med.size(), 2u);
}

// Same height -> the median rule must keep normal connectivity intact.
TEST(BevLines, MedianGapKeepsSameHeightRunConnected)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 5; ++i) {
    addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.05f);
    addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.08f);
  }
  BevLineParams p;
  p.cell_size = 0.05;
  p.z_med_gap = 0.05;
  auto segs = extractBevLineSegments(cloud, p);
  ASSERT_EQ(segs.size(), 1u);
  EXPECT_NEAR(segLength(segs[0]), 0.20, 0.06);
}

// z_med_gap = 0 disables the median rule: behavior is the old range rule.
TEST(BevLines, MedianGapZeroFallsBackToRangeRule)
{
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 5; ++i) {addPoint(*cloud, 1.02f + 0.05f * i, 0.52f, 0.0f);}
  BevLineParams p;
  p.cell_size = 0.05;
  p.z_med_gap = 0.0;
  auto segs = extractBevLineSegments(cloud, p);
  EXPECT_EQ(segs.size(), 1u);
}

// Corner-touch guard: two runs that touch ONLY at a diagonal corner (both
// flanking orthogonal cells empty) must stay separate with the gate on;
// without the gate the plain 8-neighbourhood merges them.
TEST(BevLines, DiagFlankGateBlocksCornerOnlyTouch)
{
  auto cloud = std::make_shared<Cloud>();
  // run A: cells (40..44, 10); run B: cells (45..49, 11) — A's last cell and
  // B's first cell touch only corner-to-corner, flanks (45,10)/(44,11) empty
  for (int i = 0; i < 5; ++i) {addPoint(*cloud, 2.02f + 0.05f * i, 0.52f);}
  for (int i = 0; i < 5; ++i) {addPoint(*cloud, 2.27f + 0.05f * i, 0.57f);}

  BevLineParams p;
  p.cell_size = 0.05;

  p.diag_flank_gate = false;
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 1u);  // merged run

  p.diag_flank_gate = true;
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 2u);  // corner cut
}

// A two-cell-wide staircase keeps its orthogonal links, so the gate must NOT
// break a genuinely connected diagonal structure.
TEST(BevLines, DiagFlankGateKeepsThickStaircaseConnected)
{
  auto cloud = std::make_shared<Cloud>();
  for (int k = 0; k < 6; ++k) {
    addPoint(*cloud, 1.02f + 0.05f * k, 1.02f + 0.05f * k);
    addPoint(*cloud, 1.07f + 0.05f * k, 1.02f + 0.05f * k);
  }
  BevLineParams p;
  p.cell_size = 0.05;
  p.diag_flank_gate = true;
  EXPECT_EQ(extractBevLineSegments(cloud, p).size(), 1u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
