#include <gtest/gtest.h>
#include <cmath>

#include "ouster_cluster/bev_tracker.hpp"

using ouster_cluster::BevMeasurement;
using ouster_cluster::BevTrack;
using ouster_cluster::BevTracker;
using ouster_cluster::BevTrackerParams;

namespace
{
BevMeasurement measAt(double x, double y, double yaw = 0.0)
{
  BevMeasurement m;
  m.center = Eigen::Vector2d(x, y);
  m.half = Eigen::Vector2d(0.1, 0.05);
  m.yaw = yaw;
  m.z_min = 0.0;
  m.z_max = 0.3;
  return m;
}
}  // namespace

// Confirmation gate.
TEST(BevTracker, ConfirmationGate)
{
  BevTrackerParams p; p.confirm_frames = 2;
  BevTracker tr(p);
  EXPECT_TRUE(tr.update({measAt(1, 0)}, 0.1).empty());
  EXPECT_EQ(tr.update({measAt(1, 0)}, 0.1).size(), 1u);
}

// Stationary object: center converges, velocity stays ~0.
TEST(BevTracker, StationaryConverges)
{
  BevTrackerParams p; p.pos_alpha = 0.5; p.vel_alpha = 0.3; p.confirm_frames = 1;
  BevTracker tr(p);
  const double nz[6] = {0.02, -0.02, 0.01, -0.015, 0.012, -0.008};
  BevTrack last;
  for (double n : nz) {
    auto out = tr.update({measAt(2.0 + n, 3.0)}, 0.1);
    ASSERT_EQ(out.size(), 1u);
    last = out[0];
  }
  EXPECT_NEAR(last.center.x(), 2.0, 0.03);
  EXPECT_NEAR(last.center.y(), 3.0, 0.03);
  EXPECT_LT(last.vel.norm(), 0.15);  // essentially still
}

// KEY TEST: constant-velocity motion has no lag once velocity converges.
// Object moves +1 m/s in x (0.1 m per 0.1 s frame). After several frames the
// tracked center should sit essentially ON the latest measurement, not behind.
TEST(BevTracker, ConstantVelocityNoLag)
{
  BevTrackerParams p; p.pos_alpha = 0.5; p.vel_alpha = 0.5; p.confirm_frames = 1;
  BevTracker tr(p);
  const double dt = 0.1, speed = 1.0, step = speed * dt;  // 0.1 m/frame
  double x = 0.0;
  BevTrack last;
  for (int i = 0; i < 20; ++i) {
    x += step;
    auto out = tr.update({measAt(x, 0.0)}, dt);
    ASSERT_EQ(out.size(), 1u);
    last = out[0];
  }
  // velocity converged near true speed
  EXPECT_NEAR(last.vel.x(), speed, 0.15);
  // tracked center is at (not behind) the latest observation — lag < 3 cm
  EXPECT_NEAR(last.center.x(), x, 0.03);
}

// A plain-EMA (no prediction) would lag here; confirm our predictor beats that.
// Sanity: with prediction the residual is far smaller than one step.
TEST(BevTracker, PredictionBeatsPlainLag)
{
  BevTrackerParams p; p.pos_alpha = 0.3; p.vel_alpha = 0.5; p.confirm_frames = 1;
  BevTracker tr(p);
  const double dt = 0.1, step = 0.1;
  double x = 0.0;
  BevTrack last;
  for (int i = 0; i < 25; ++i) {
    x += step;
    last = tr.update({measAt(x, 0.0)}, dt).at(0);
  }
  const double lag = std::abs(x - last.center.x());
  EXPECT_LT(lag, step);  // less than one full step of lag
}

// Track dies after max_missed consecutive misses.
TEST(BevTracker, TrackDies)
{
  BevTrackerParams p; p.confirm_frames = 1; p.max_missed = 3;
  BevTracker tr(p);
  tr.update({measAt(1, 0)}, 0.1);
  ASSERT_EQ(tr.tracks().size(), 1u);
  for (int i = 0; i < 3; ++i) {tr.update({}, 0.1);}
  EXPECT_EQ(tr.tracks().size(), 1u);
  tr.update({}, 0.1);
  EXPECT_TRUE(tr.tracks().empty());
}

// dt is clamped so a huge gap does not fling the prediction across the map.
TEST(BevTracker, DtClamped)
{
  BevTrackerParams p; p.confirm_frames = 1; p.pos_alpha = 0.5; p.vel_alpha = 0.5;
  p.max_dt = 0.3;
  BevTracker tr(p);
  // build up a velocity of ~1 m/s
  double x = 0.0;
  for (int i = 0; i < 10; ++i) {x += 0.1; tr.update({measAt(x, 0)}, 0.1);}
  // now a miss with a huge dt: prediction advances by vel*clamped_dt, not vel*100
  const double before = tr.tracks().at(0).center.x();
  tr.update({}, 100.0);   // would be +100 m if unclamped
  const double after = tr.tracks().at(0).center.x();
  EXPECT_LT(after - before, 1.0);  // clamped to <= ~vel*0.3
}

// Two tracks, two measurements, no crossover.
TEST(BevTracker, MatchingNoCrossover)
{
  BevTrackerParams p; p.confirm_frames = 1; p.match_distance = 0.5;
  BevTracker tr(p);
  tr.update({measAt(0, 0), measAt(3, 0)}, 0.1);
  ASSERT_EQ(tr.tracks().size(), 2u);
  tr.update({measAt(0.05, 0), measAt(3.05, 0)}, 0.1);
  EXPECT_EQ(tr.tracks().size(), 2u);
}

// Yaw with noise converges smoothly and stays near the true heading.
TEST(BevTracker, YawConverges)
{
  BevTrackerParams p; p.confirm_frames = 1; p.yaw_alpha = 0.3;
  BevTracker tr(p);
  const double truth = 30.0 * M_PI / 180.0;
  const double nz[6] = {0.05, -0.05, 0.03, -0.04, 0.02, -0.03};
  BevTrack last;
  for (double n : nz) {
    last = tr.update({measAt(0, 0, truth + n)}, 0.1).at(0);
  }
  EXPECT_NEAR(ouster_cluster::bevWrapAngle(last.yaw - truth), 0.0, 0.05);
}

// PCA sign flip: yaw alternating theta and theta+pi is the SAME box; folding
// must keep the tracked yaw steady, not oscillate.
TEST(BevTracker, YawPiFlipStable)
{
  BevTrackerParams p; p.confirm_frames = 1; p.yaw_alpha = 0.3;
  BevTracker tr(p);
  const double theta = 20.0 * M_PI / 180.0;
  BevTrack last;
  for (int i = 0; i < 8; ++i) {
    double meas = (i % 2 == 0) ? theta : theta + M_PI;  // identical footprint
    last = tr.update({measAt(0, 0, meas)}, 0.1).at(0);
  }
  const double diff = std::abs(ouster_cluster::bevWrapAngle(last.yaw - theta));
  EXPECT_LT(diff, 10.0 * M_PI / 180.0);  // stayed near theta, no 180 jump
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
