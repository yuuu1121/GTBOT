// ROS-independent BEV line-segment extraction. Takes the band-passed BEV
// density cloud (survivor cells = likely vertical structures), dedupes it to
// unique grid cells, groups cells into 8-neighbour CONNECTED COMPONENTS, and
// keeps only components shaped like a line (long along one axis, thin across
// it). A planar structure seen from above is a thin connected run of cells;
// scattered floor residue is either disconnected (too few cells per component)
// or a fat blob (too wide). Cells that merely happen to be collinear but sit
// apart can never form a segment — unlike a global RANSAC fit.
#ifndef OUSTER_CLUSTER__BEV_LINES_HPP_
#define OUSTER_CLUSTER__BEV_LINES_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ouster_cluster/cluster_logic.hpp"  // ouster_cluster::Cloud

namespace ouster_cluster
{

/// Parameters for BEV line extraction.
struct BevLineParams
{
  double cell_size = 0.05;   // must match the density map's cell size (m)
  int min_cells = 3;         // a component needs at least this many cells
  double max_width = 0.12;   // max extent across the line axis (m) — wider = blob
  double min_length = 0.05;  // discard components shorter than this (m)
  double max_length = 0.0;   // if > 0, discard components longer than this (m)
  double z_gap = 0.1;        // neighbours connect only if their cell z-ranges are
                             // within this gap (m) — a plate passing near a LOWER
                             // structure keeps its own component
  double z_med_gap = 0.0;    // if > 0, connect on MEDIAN cell height instead:
                             // |median_a - median_b| <= this. Robust where the
                             // range rule is not — one stray point stretches a
                             // cell's z-range into anything, but not its median
  bool diag_flank_gate = false;  // diagonal neighbours connect only when BOTH
                                 // flanking orthogonal cells exist and pass the
                                 // z check — a corner-only touch cannot join
                                 // (kills stray-cell leaks through corners)
  bool touch_connect = false;  // connect neighbours by point-level MAJORITY
                               // overlap: in at least one direction, more than
                               // half of a cell's points must find a companion
                               // point within touch_gap in z in the other cell.
                               // Robust where a summary statistic (median)
                               // jumps when the visible part of the same
                               // object changes along the line, and a single
                               // touching pair cannot glue two structures.
                               // Overrides the z_med_gap / z_gap rules when on.
  double touch_gap = 0.03;     // max |z| difference for a companion point (m)
  double touch_xy_gap = 0.0;   // max horizontal (XY) distance for a companion
                               // point (m); <= 0 disables (z alone decides) —
                               // without it, two points 5cm+ apart in XY but
                               // in the same z-band still count as touching
  int touch_min_matched = 2;   // a direction also needs at least this many
                               // matched points — a single-point cell passes
                               // "majority" trivially (1/1) and would chain
                               // two structures as a stepping stone
  double touch_min_ratio = 0.5;  // fraction of a cell's points that must find
                                 // a companion for the direction to pass
                                 // (strictly greater than); 0.5 = old "more
                                 // than half" rule. Lower = more permissive.
  double top_tol = 0.06;     // max spread of per-cell TOP heights along the line (m)
                             // — a real structure has a flat top edge; staircase
                             // chains / mixed structures drift and are rejected
  double top_step = 0.07;    // split a component where the median-smoothed top
                             // profile steps more than this along the line (m);
                             // 0 = no splitting
  int max_segments = 10;     // stop after this many segments
};

/// One BEV line segment (metres, sensor frame).
struct BevSegment
{
  float x0, y0, x1, y1;
  int cells;  // number of grid cells supporting it
  float z_min = 0.f, z_max = 0.f;  // height band of the supporting cells
  float width = 0.f;  // extent ACROSS the line (m): cell width from extraction,
                      // replaced by the point-level spread after refine
};

/// Extract line segments from the BEV density cloud. `cloud` is the output of
/// computeBevDensity (already band-passed); only each point's XY cell matters,
/// duplicates collapse to one cell. Deterministic.
inline std::vector<BevSegment> extractBevLineSegments(
  const Cloud::Ptr & cloud, const BevLineParams & p)
{
  std::vector<BevSegment> segs;
  if (!cloud || cloud->empty() || p.cell_size <= 0.0) {return segs;}

  // collapse points to unique cells; keep integer coords for adjacency and
  // the per-cell z-range for height-aware connectivity
  const double inv = 1.0 / p.cell_size;
  struct CellInfo
  {
    std::int32_t cx, cy;
    float zmin, zmax;
    float zmed = 0.f;
  };
  std::unordered_map<std::int64_t, int> cell_idx;  // packed (cx,cy) -> index
  std::vector<CellInfo> cells;
  // per-cell point samples for the median/touch rules. touch also needs x,y
  // (not just z) when touch_xy_gap gates companions on horizontal distance.
  struct Pt3 {float x, y, z;};
  std::vector<std::vector<Pt3>> cell_pts;
  const bool need_pts = p.z_med_gap > 0.0 || p.touch_connect;
  auto pack = [](std::int64_t cx, std::int64_t cy) -> std::int64_t {
      return ((cx & 0xffffffffLL) << 32) | (cy & 0xffffffffLL);
    };
  for (const auto & pt : cloud->points) {
    const std::int64_t cx = static_cast<std::int64_t>(std::floor(pt.x * inv));
    const std::int64_t cy = static_cast<std::int64_t>(std::floor(pt.y * inv));
    const auto ins = cell_idx.emplace(pack(cx, cy), static_cast<int>(cells.size()));
    if (ins.second) {
      cells.push_back(
        CellInfo{static_cast<std::int32_t>(cx), static_cast<std::int32_t>(cy), pt.z, pt.z});
      if (need_pts) {cell_pts.push_back({{pt.x, pt.y, pt.z}});}
    } else {
      CellInfo & c = cells[ins.first->second];
      c.zmin = std::min(c.zmin, pt.z);
      c.zmax = std::max(c.zmax, pt.z);
      if (need_pts) {cell_pts[ins.first->second].push_back({pt.x, pt.y, pt.z});}
    }
  }
  if (need_pts) {
    for (std::size_t i = 0; i < cells.size(); ++i) {
      auto & pts = cell_pts[i];
      std::sort(pts.begin(), pts.end(), [](const Pt3 & a, const Pt3 & b) {return a.z < b.z;});
      cells[i].zmed = pts[pts.size() / 2].z;  // median = middle of the z-sorted list
    }
  }

  // connected components over the 8-neighbourhood (BFS on the grid hash);
  // neighbours only connect when their heights agree — XY-adjacent cells at
  // clearly different heights belong to different structures. Rules, by
  // priority: point-level majority touch > z-median gap > z-range gap.
  // majority touch: a point "touches" the neighbour when the neighbour has a
  // point within touch_gap in z (AND, if touch_xy_gap > 0, within touch_xy_gap
  // horizontally); the direction passes when MORE than half of the cell's
  // points touch AND at least touch_min_matched points touch. One stray pair
  // cannot glue two structures, and a single-point cell cannot vote itself
  // into a bridge.
  auto majority_touch = [&p, &cell_pts](int ia, int ib) {
      const auto & A = cell_pts[ia];
      const auto & B = cell_pts[ib];  // both z-sorted ascending
      const float gap = static_cast<float>(p.touch_gap);
      const float xy_gap = static_cast<float>(p.touch_xy_gap);
      const bool check_xy = p.touch_xy_gap > 0.0;
      std::size_t matched = 0, lo = 0;
      for (const auto & a : A) {
        while (lo < B.size() && B[lo].z < a.z - gap) {++lo;}
        bool found = false;
        for (std::size_t j = lo; j < B.size() && B[j].z <= a.z + gap; ++j) {
          if (!check_xy || std::hypot(B[j].x - a.x, B[j].y - a.y) <= xy_gap) {
            found = true;
            break;
          }
        }
        if (found) {++matched;}
      }
      return static_cast<double>(matched) > p.touch_min_ratio * A.size() &&
             matched >= static_cast<std::size_t>(std::max(p.touch_min_matched, 0));
    };
  auto z_connected = [&p, &cells, &majority_touch](int ia, int ib) {
      if (p.touch_connect) {
        // one passing direction suffices: an occluded (few-point) section
        // still connects to its fully visible continuation
        return majority_touch(ia, ib) || majority_touch(ib, ia);
      }
      const CellInfo & a = cells[ia];
      const CellInfo & b = cells[ib];
      if (p.z_med_gap > 0.0) {
        return std::fabs(a.zmed - b.zmed) <= static_cast<float>(p.z_med_gap);
      }
      const float gap = std::max(a.zmin, b.zmin) - std::min(a.zmax, b.zmax);
      return gap <= static_cast<float>(p.z_gap);
    };
  std::vector<int> comp(cells.size(), -1);
  int n_comp = 0;
  std::vector<int> stack;
  for (std::size_t s = 0; s < cells.size(); ++s) {
    if (comp[s] >= 0) {continue;}
    comp[s] = n_comp;
    stack.assign(1, static_cast<int>(s));
    while (!stack.empty()) {
      const int i = stack.back();
      stack.pop_back();
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {continue;}
          if (dx != 0 && dy != 0 && p.diag_flank_gate) {
            // corner-touch guard: the two orthogonal cells flanking this
            // diagonal must themselves exist and pass the z check
            const auto f1 = cell_idx.find(pack(cells[i].cx + dx, cells[i].cy));
            const auto f2 = cell_idx.find(pack(cells[i].cx, cells[i].cy + dy));
            if (f1 == cell_idx.end() || !z_connected(i, f1->second) ||
              f2 == cell_idx.end() || !z_connected(i, f2->second))
            {
              continue;
            }
          }
          const auto it = cell_idx.find(pack(cells[i].cx + dx, cells[i].cy + dy));
          if (it != cell_idx.end() && comp[it->second] < 0 &&
            z_connected(i, it->second))
          {
            comp[it->second] = n_comp;
            stack.push_back(it->second);
          }
        }
      }
    }
    ++n_comp;
  }

  // group cell indices by component
  std::vector<std::vector<int>> groups(n_comp);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    groups[comp[i]].push_back(static_cast<int>(i));
  }

  auto centre = [&p, &cells](int i) {
      return std::pair<float, float>(
        static_cast<float>((cells[i].cx + 0.5) * p.cell_size),
        static_cast<float>((cells[i].cy + 0.5) * p.cell_size));
    };

  // emit one group of cells as a segment if it passes the line-ness checks:
  // PCA direction, then length/width extents + top-edge uniformity
  auto emitGroup = [&](const std::vector<int> & g) {
      if (static_cast<int>(g.size()) < p.min_cells) {return;}
      if (static_cast<int>(segs.size()) >= p.max_segments) {return;}

      float mx = 0.f, my = 0.f;
      for (int i : g) {const auto c = centre(i); mx += c.first; my += c.second;}
      mx /= g.size(); my /= g.size();
      float sxx = 0.f, sxy = 0.f, syy = 0.f;
      for (int i : g) {
        const auto c = centre(i);
        const float dx = c.first - mx, dy = c.second - my;
        sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
      }
      // principal axis of the 2x2 covariance (analytic, no eigen solver needed)
      const float theta = 0.5f * std::atan2(2.f * sxy, sxx - syy);
      const float ux = std::cos(theta), uy = std::sin(theta);

      float tmin = 1e9f, tmax = -1e9f, wmin = 1e9f, wmax = -1e9f;
      for (int i : g) {
        const auto c = centre(i);
        const float dx = c.first - mx, dy = c.second - my;
        const float t = ux * dx + uy * dy;    // along the line axis
        const float w = -uy * dx + ux * dy;   // across it
        tmin = std::min(tmin, t); tmax = std::max(tmax, t);
        wmin = std::min(wmin, w); wmax = std::max(wmax, w);
      }
      // robust z-band: MEDIAN of the per-cell extremes, not min/max — a mixed
      // boundary cell (plate + desk points in one cell, its top camouflaged as
      // the plate's) cannot drag the band down and flip the plate verdict
      std::vector<float> zlo, zhi;
      zlo.reserve(g.size()); zhi.reserve(g.size());
      for (int i : g) {zlo.push_back(cells[i].zmin); zhi.push_back(cells[i].zmax);}
      std::nth_element(zlo.begin(), zlo.begin() + zlo.size() / 2, zlo.end());
      std::nth_element(zhi.begin(), zhi.begin() + zhi.size() / 2, zhi.end());
      const float zmin = zlo[zlo.size() / 2];
      const float zmax = zhi[zhi.size() / 2];
      const float length = tmax - tmin;
      const float width = wmax - wmin;
      if (width > static_cast<float>(p.max_width)) {return;}   // blob, not a line
      if (length < static_cast<float>(p.min_length)) {return;}
      if (p.max_length > 0.0 && length > static_cast<float>(p.max_length)) {return;}

      // top-edge uniformity: per-cell top heights must agree along the line.
      // Trimmed spread (drop the single highest/lowest cell when >=5 cells) so
      // one cell with a stray point cannot veto the whole component.
      if (p.top_tol > 0.0) {
        std::vector<float> tops;
        tops.reserve(g.size());
        for (int i : g) {tops.push_back(cells[i].zmax);}
        std::sort(tops.begin(), tops.end());
        const std::size_t trim = tops.size() >= 5 ? 1 : 0;
        const float spread = tops[tops.size() - 1 - trim] - tops[trim];
        if (spread > static_cast<float>(p.top_tol)) {return;}
      }

      segs.push_back(
        BevSegment{
          mx + ux * tmin, my + uy * tmin,
          mx + ux * tmax, my + uy * tmax,
          static_cast<int>(g.size()), zmin, zmax, width});
    };

  // per component: split at persistent steps in the top-height profile, then
  // emit each piece. A plate standing ON a desk merges with the desk edge into
  // one component (their z-ranges touch), but the top profile steps ~10cm at
  // the boundary. A 3-tap median over the profile absorbs isolated per-cell
  // top jitter (ring dropout), so only spatially persistent steps split.
  for (const auto & g : groups) {
    if (static_cast<int>(g.size()) < p.min_cells) {continue;}
    if (p.top_step <= 0.0) {emitGroup(g); continue;}

    // order cells along the component's principal axis
    float mx = 0.f, my = 0.f;
    for (int i : g) {const auto c = centre(i); mx += c.first; my += c.second;}
    mx /= g.size(); my /= g.size();
    float sxx = 0.f, sxy = 0.f, syy = 0.f;
    for (int i : g) {
      const auto c = centre(i);
      const float dx = c.first - mx, dy = c.second - my;
      sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const float theta = 0.5f * std::atan2(2.f * sxy, sxx - syy);
    const float ux = std::cos(theta), uy = std::sin(theta);
    std::vector<std::pair<float, int>> order;
    order.reserve(g.size());
    for (int i : g) {
      const auto c = centre(i);
      order.emplace_back(ux * (c.first - mx) + uy * (c.second - my), i);
    }
    std::sort(order.begin(), order.end());

    const std::size_t n = order.size();
    std::vector<float> smooth(n);
    for (std::size_t j = 0; j < n; ++j) {
      const float a = cells[order[j > 0 ? j - 1 : j].second].zmax;
      const float b = cells[order[j].second].zmax;
      const float c = cells[order[j + 1 < n ? j + 1 : j].second].zmax;
      smooth[j] = std::max(std::min(a, b), std::min(std::max(a, b), c));  // median of 3
    }
    std::vector<int> run;
    for (std::size_t j = 0; j < n; ++j) {
      if (j > 0 && std::fabs(smooth[j] - smooth[j - 1]) > static_cast<float>(p.top_step)) {
        emitGroup(run);
        run.clear();
      }
      run.push_back(order[j].second);
    }
    emitGroup(run);
  }
  return segs;
}

/// Parameters for point-level segment refinement.
struct BevRefineParams
{
  bool enable = true;            // off: keep the coarse cell segments untouched
  double band = 0.05;            // gate: max perpendicular distance to the coarse line (m)
  double mad_k = 2.5;            // trim points whose residual exceeds mad_k x robust sigma
  double end_percentile = 0.02;  // endpoints at [p, 1-p] percentile along the line
  int min_points = 10;           // below this, keep the coarse segment as-is
};

/// Refine a coarse (cell-based) segment against the raw points: gate points to
/// a band around the coarse line, robust-fit (PCA + MAD trimming, 2 rounds),
/// and place endpoints at along-line percentiles. Removes the cell-grid
/// quantization jitter; a segment with too few nearby points is returned
/// unchanged (the cell evidence already vouched for it).
inline BevSegment refineBevSegment(
  const BevSegment & seg, const Cloud::Ptr & cloud, const BevRefineParams & p)
{
  if (!cloud || cloud->empty()) {return seg;}
  float ux = seg.x1 - seg.x0, uy = seg.y1 - seg.y0;
  const float len = std::hypot(ux, uy);
  if (len < 1e-6f) {return seg;}
  ux /= len; uy /= len;

  // gate: points within `band` of the coarse line, within its extent + band,
  // and within the segment's own height band (a lower structure's points
  // nearby in XY must not pollute the fit)
  const float band = static_cast<float>(p.band);
  std::vector<std::pair<float, float>> pts;
  for (const auto & pt : cloud->points) {
    const float dx = pt.x - seg.x0, dy = pt.y - seg.y0;
    const float t = ux * dx + uy * dy;
    const float w = -uy * dx + ux * dy;
    if (std::fabs(w) <= band && t >= -band && t <= len + band &&
      pt.z >= seg.z_min - band && pt.z <= seg.z_max + band)
    {
      pts.emplace_back(pt.x, pt.y);
    }
  }
  if (static_cast<int>(pts.size()) < p.min_points) {return seg;}

  // robust fit: trim against the CURRENT line, then PCA-refit the survivors.
  // Round 1 trims against the coarse cell line — it is majority-voted and
  // untilted, so an outlier clump separates cleanly; a first-pass PCA fit
  // would already lean towards the clump and blunt the MAD threshold.
  float mx = seg.x0, my = seg.y0, dirx = ux, diry = uy;
  for (int round = 0; round < 2; ++round) {
    std::vector<float> res;
    res.reserve(pts.size());
    for (const auto & q : pts) {
      res.push_back(-diry * (q.first - mx) + dirx * (q.second - my));
    }
    std::vector<float> tmp = res;
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    const float med = tmp[tmp.size() / 2];
    for (auto & r : tmp) {r = std::fabs(r - med);}
    std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
    // robust sigma with a floor so perfectly collinear points don't trim everything
    const float sigma = std::max(1.4826f * tmp[tmp.size() / 2], 0.005f);

    std::vector<std::pair<float, float>> kept;
    kept.reserve(pts.size());
    for (std::size_t i = 0; i < pts.size(); ++i) {
      if (std::fabs(res[i] - med) <= static_cast<float>(p.mad_k) * sigma) {
        kept.push_back(pts[i]);
      }
    }
    if (static_cast<int>(kept.size()) < p.min_points) {break;}
    pts = std::move(kept);

    mx = 0.f; my = 0.f;
    for (const auto & q : pts) {mx += q.first; my += q.second;}
    mx /= pts.size(); my /= pts.size();
    float sxx = 0.f, sxy = 0.f, syy = 0.f;
    for (const auto & q : pts) {
      const float dx = q.first - mx, dy = q.second - my;
      sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
    }
    const float theta = 0.5f * std::atan2(2.f * sxy, sxx - syy);
    dirx = std::cos(theta); diry = std::sin(theta);
  }

  // endpoints at along-line percentiles (a lone stray cannot stretch the segment)
  std::vector<float> proj;
  proj.reserve(pts.size());
  for (const auto & q : pts) {
    proj.push_back(dirx * (q.first - mx) + diry * (q.second - my));
  }
  std::sort(proj.begin(), proj.end());
  const std::size_t n = proj.size();
  const std::size_t lo = static_cast<std::size_t>(p.end_percentile * (n - 1));
  const std::size_t hi = (n - 1) - lo;
  // point-level width: perpendicular spread of the kept points, same
  // percentile trim as the endpoints (replaces the fatter cell-grid width)
  std::vector<float> perp;
  perp.reserve(pts.size());
  for (const auto & q : pts) {
    perp.push_back(-diry * (q.first - mx) + dirx * (q.second - my));
  }
  std::sort(perp.begin(), perp.end());
  return BevSegment{
    mx + dirx * proj[lo], my + diry * proj[lo],
    mx + dirx * proj[hi], my + diry * proj[hi],
    seg.cells, seg.z_min, seg.z_max, perp[hi] - perp[lo]};
}

/// Refine every segment in `segs` against `cloud`.
inline std::vector<BevSegment> refineBevSegments(
  const std::vector<BevSegment> & segs, const Cloud::Ptr & cloud, const BevRefineParams & p)
{
  if (!p.enable) {return segs;}
  std::vector<BevSegment> out;
  out.reserve(segs.size());
  for (const auto & s : segs) {out.push_back(refineBevSegment(s, cloud, p));}
  return out;
}

/// Target plate dimensions for the shape check.
struct BevPlateParams
{
  double width = 0.14;       // target plate width  = BEV segment length (m)
  double height = 0.10;      // target plate height = segment z-band (m)
  double tol_width = 0.05;   // +/- tolerance on the segment length (m)
  double tol_height = 0.02;  // +/- tolerance on the z-band height (m)
  double thickness_max = 0.05;  // reject segments whose perpendicular extent
                                // exceeds this (the plate sheet is <= 3cm);
                                // <= 0 disables
};

/// True if a segment's length and height band match the target plate size.
/// Length and height are judged against their own tolerances; a segment
/// thicker than thickness_max across the line is a box/blob, not the plate.
inline bool isPlateSegment(const BevSegment & s, const BevPlateParams & p)
{
  if (p.thickness_max > 0.0 && s.width > static_cast<float>(p.thickness_max)) {
    return false;
  }
  const float length = std::hypot(s.x1 - s.x0, s.y1 - s.y0);
  const float height = s.z_max - s.z_min;
  return std::fabs(length - static_cast<float>(p.width)) <=
         static_cast<float>(p.tol_width) &&
         std::fabs(height - static_cast<float>(p.height)) <=
         static_cast<float>(p.tol_height);
}

/// Parameters for the segment reflectivity gate. Works on whatever scalar the
/// pipeline carries in `intensity` (reflectivity or signal, per scalar_field).
struct BevReflGateParams
{
  bool enable = false;
  double band = 0.05;        // XY gate around the segment line (m), like refine
  double spread_max = 60.0;  // max p90-p10 scalar spread (one material = uniform)
  double min_median = 0.0;   // absolute band on the median scalar: reject
  double max_median = 255.0; // segments made of the wrong material
  int min_points = 10;       // fewer supporting points -> keep (cells vouched)
};

/// A real plate is a single material: the scalar values of the points that
/// support a segment must be uniform (small p10-p90 spread) and their median
/// inside the material's band. `cloud` must carry the raw scalar in intensity
/// (the preprocessed cloud, NOT the density cloud, whose intensity is density).
inline bool passesReflectivityGate(
  const BevSegment & seg, const Cloud::Ptr & cloud, const BevReflGateParams & p)
{
  if (!p.enable) {return true;}
  if (!cloud || cloud->empty()) {return true;}
  float ux = seg.x1 - seg.x0, uy = seg.y1 - seg.y0;
  const float len = std::hypot(ux, uy);
  if (len < 1e-6f) {return true;}
  ux /= len; uy /= len;

  const float band = static_cast<float>(p.band);
  std::vector<float> vals;
  for (const auto & pt : cloud->points) {
    const float dx = pt.x - seg.x0, dy = pt.y - seg.y0;
    const float t = ux * dx + uy * dy;
    const float w = -uy * dx + ux * dy;
    if (std::fabs(w) <= band && t >= -band && t <= len + band &&
      pt.z >= seg.z_min - band && pt.z <= seg.z_max + band)
    {
      vals.push_back(pt.intensity);
    }
  }
  if (static_cast<int>(vals.size()) < p.min_points) {return true;}

  std::sort(vals.begin(), vals.end());
  const std::size_t n = vals.size();
  const float p10 = vals[static_cast<std::size_t>(0.1 * (n - 1))];
  const float med = vals[n / 2];
  const float p90 = vals[static_cast<std::size_t>(0.9 * (n - 1))];
  return (p90 - p10) <= static_cast<float>(p.spread_max) &&
         med >= static_cast<float>(p.min_median) &&
         med <= static_cast<float>(p.max_median);
}

/// Parameters for the temporal segment filter.
struct BevSegFilterParams
{
  double match_distance = 0.15;  // max center-to-center distance to be the same segment (m)
  int confirm_frames = 3;        // consecutive appearances before a segment is published
  int max_missed = 2;            // a confirmed segment survives this many missed frames
};

/// Temporal hysteresis over extracted segments: a segment is published only
/// after appearing confirm_frames frames in a row (kills flickering noise),
/// and once confirmed it is held through up to max_missed dropped frames
/// (kills flicker-OFF of real structures). Matching: each track, visited in
/// storage order, claims its nearest unclaimed segment by centre distance (no
/// global nearest-first pass — segments here are few and well separated).
class BevSegmentFilter
{
public:
  void setParams(const BevSegFilterParams & p) {prm_ = p;}

  std::vector<BevSegment> update(const std::vector<BevSegment> & segs)
  {
    // per-track greedy matching: each track claims its nearest unclaimed segment
    std::vector<char> seg_used(segs.size(), 0);
    for (auto & t : tracks_) {
      const float tx = 0.5f * (t.seg.x0 + t.seg.x1);
      const float ty = 0.5f * (t.seg.y0 + t.seg.y1);
      int best = -1;
      float best_d = static_cast<float>(prm_.match_distance);
      for (std::size_t i = 0; i < segs.size(); ++i) {
        if (seg_used[i]) {continue;}
        const float sx = 0.5f * (segs[i].x0 + segs[i].x1);
        const float sy = 0.5f * (segs[i].y0 + segs[i].y1);
        const float d = std::hypot(sx - tx, sy - ty);
        if (d <= best_d) {best_d = d; best = static_cast<int>(i);}
      }
      if (best >= 0) {
        seg_used[best] = 1;
        t.seg = segs[best];
        ++t.hits;
        t.missed = 0;
      } else {
        ++t.missed;
      }
    }
    // expire: unconfirmed tracks die on the first miss (consecutive rule);
    // confirmed tracks survive up to max_missed
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [this](const Track & t) {
          return t.missed > 0 &&
                 (t.hits < prm_.confirm_frames || t.missed > prm_.max_missed);
        }),
      tracks_.end());
    // new tracks for unmatched segments
    for (std::size_t i = 0; i < segs.size(); ++i) {
      if (!seg_used[i]) {tracks_.push_back(Track{segs[i], 1, 0});}
    }
    // publish confirmed tracks (including ones held through a dropout)
    std::vector<BevSegment> out;
    for (const auto & t : tracks_) {
      if (t.hits >= prm_.confirm_frames) {out.push_back(t.seg);}
    }
    return out;
  }

private:
  struct Track
  {
    BevSegment seg;
    int hits;
    int missed;
  };
  BevSegFilterParams prm_;
  std::vector<Track> tracks_;
};

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__BEV_LINES_HPP_
