// ROS-independent BEV (top-down 2D) multi-object tracker with a constant-
// velocity predictor. Tracks xy position/size only; z range is carried through
// for display but never smoothed. Velocity prediction cancels the lag that a
// plain EMA leaves on moving objects.
#ifndef OUSTER_CLUSTER__BEV_TRACKER_HPP_
#define OUSTER_CLUSTER__BEV_TRACKER_HPP_

#include <algorithm>
#include <vector>

#include <Eigen/Dense>

namespace ouster_cluster
{

/// Wrap an angle to (-pi, pi].
inline double bevWrapAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a <= -M_PI) {a += 2.0 * M_PI;}
  return a;
}

/// A box is symmetric under a pi rotation (yaw and yaw+pi are the same box) and
/// a BEV footprint is also symmetric under pi/2 if we allow swapping the x/y
/// extents. Fold the measured yaw to within (-pi/2, pi/2] of `ref` so it never
/// jumps by ~90/180 deg between frames.
inline double bevFoldYaw(double obs, double ref)
{
  double d = bevWrapAngle(obs - ref);
  while (d > M_PI_2) {d -= M_PI;}
  while (d <= -M_PI_2) {d += M_PI;}
  return ref + d;
}

/// EMA of two angles via unit vectors (avoids the 179<->-179 averaging error).
inline double bevEmaAngle(double prev, double meas, double alpha)
{
  const double s = (1.0 - alpha) * std::sin(prev) + alpha * std::sin(meas);
  const double c = (1.0 - alpha) * std::cos(prev) + alpha * std::cos(meas);
  return std::atan2(s, c);
}

/// One cluster reduced to a BEV measurement this frame.
struct BevMeasurement
{
  Eigen::Vector2d center;  // xy centroid
  Eigen::Vector2d half;    // half-size in the yaw-rotated frame
  double yaw = 0.0;        // heading of the xy principal axis (rad)
  double z_min = 0.0;      // display only, not tracked
  double z_max = 0.0;
};

/// A tracked object in the BEV plane.
struct BevTrack
{
  int id = 0;
  Eigen::Vector2d center = Eigen::Vector2d::Zero();  // filtered xy position
  Eigen::Vector2d vel = Eigen::Vector2d::Zero();     // xy velocity (m/s)
  Eigen::Vector2d half = Eigen::Vector2d::Zero();    // filtered half-size (yaw frame)
  double yaw = 0.0;                                   // filtered heading (rad)
  double z_min = 0.0;
  double z_max = 0.0;
  int hits = 0;
  int misses = 0;
  bool confirmed = false;
};

struct BevTrackerParams
{
  double match_distance = 0.5;  // max center distance to associate (m)
  double pos_alpha = 0.5;       // position/size EMA (prediction handles lag)
  double vel_alpha = 0.3;       // velocity EMA (smaller = smoother velocity)
  double yaw_alpha = 0.2;       // yaw EMA (small = steady heading, no spinning)
  int confirm_frames = 2;
  int max_missed = 3;
  double max_dt = 0.3;          // clamp dt to avoid runaway prediction (s)
};

class BevTracker
{
public:
  explicit BevTracker(const BevTrackerParams & prm = {})
  : prm_(prm) {}

  void setParams(const BevTrackerParams & prm) {prm_ = prm;}

  /// Advance one frame. `dt` is the time since the last update (seconds);
  /// clamped to (0, max_dt]. Returns confirmed tracks at their predicted
  /// (current) position.
  std::vector<BevTrack> update(const std::vector<BevMeasurement> & meas, double dt)
  {
    // max() keeps clamp's lo <= hi precondition even if max_dt is set below 1ms
    dt = std::clamp(dt, 1e-3, std::max(prm_.max_dt, 1e-3));

    // 1) Predict: advance every track by its velocity.
    for (auto & t : tracks_) {
      t.center += t.vel * dt;
    }

    const int nt = static_cast<int>(tracks_.size());
    const int nm = static_cast<int>(meas.size());
    std::vector<bool> track_used(nt, false), meas_used(nm, false);

    // 2) Greedy nearest association against predicted positions.
    while (true) {
      double best = prm_.match_distance;
      int bt = -1, bm = -1;
      for (int t = 0; t < nt; ++t) {
        if (track_used[t]) {continue;}
        for (int m = 0; m < nm; ++m) {
          if (meas_used[m]) {continue;}
          const double d = (tracks_[t].center - meas[m].center).norm();
          if (d < best) {best = d; bt = t; bm = m;}
        }
      }
      if (bt < 0) {break;}
      track_used[bt] = true;
      meas_used[bm] = true;
      correct(tracks_[bt], meas[bm], dt);
    }

    // 3) Unmatched tracks: coast (already predicted), count a miss.
    for (int t = 0; t < nt; ++t) {
      if (!track_used[t]) {tracks_[t].misses++;}
    }
    // 4) Unmatched measurements: spawn new tracks (vel = 0).
    for (int m = 0; m < nm; ++m) {
      if (meas_used[m]) {continue;}
      BevTrack tr;
      tr.id = next_id_++;
      tr.center = meas[m].center;
      tr.half = meas[m].half;
      tr.yaw = meas[m].yaw;
      tr.z_min = meas[m].z_min;
      tr.z_max = meas[m].z_max;
      tr.hits = 1;
      tr.confirmed = (prm_.confirm_frames <= 1);
      tracks_.push_back(tr);
    }

    // 5) Remove dead tracks.
    std::vector<BevTrack> alive;
    alive.reserve(tracks_.size());
    for (auto & t : tracks_) {
      if (t.misses <= prm_.max_missed) {alive.push_back(t);}
    }
    tracks_.swap(alive);

    // 6) Emit confirmed tracks.
    std::vector<BevTrack> out;
    for (const auto & t : tracks_) {
      if (t.confirmed) {out.push_back(t);}
    }
    return out;
  }

  const std::vector<BevTrack> & tracks() const {return tracks_;}
  void reset() {tracks_.clear(); next_id_ = 0;}

private:
  void correct(BevTrack & t, const BevMeasurement & m, double dt)
  {
    // Velocity implied by how far the measurement is from the prediction,
    // plus the velocity we already carried (prediction moved us by vel*dt).
    const Eigen::Vector2d predicted = t.center;               // after predict step
    const Eigen::Vector2d v_meas = t.vel + (m.center - predicted) / dt;

    const double pa = prm_.pos_alpha;
    const double va = prm_.vel_alpha;
    t.center = pa * m.center + (1.0 - pa) * predicted;
    t.vel = va * v_meas + (1.0 - va) * t.vel;
    // Fold the measured yaw next to the track's yaw (kills 90/180-deg jumps),
    // then EMA-smooth it so the heading is steady, not spinning.
    const double folded = bevFoldYaw(m.yaw, t.yaw);
    t.yaw = bevEmaAngle(t.yaw, folded, prm_.yaw_alpha);
    t.half = pa * m.half + (1.0 - pa) * t.half;
    t.z_min = m.z_min;
    t.z_max = m.z_max;
    t.hits++;
    t.misses = 0;
    if (t.hits >= prm_.confirm_frames) {t.confirmed = true;}
  }

  BevTrackerParams prm_;
  std::vector<BevTrack> tracks_;
  int next_id_ = 0;
};

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__BEV_TRACKER_HPP_
