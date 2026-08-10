// ROS-independent BEV (bird's-eye-view) density map. Bins points onto an XY
// grid; each cell's density is its point count normalized by squared range
// (sum of r^2 over the cell's points == count x dist^2). A vertical structure
// stacks many points into one XY cell -> high density. The r^2 factor cancels
// the LiDAR's ~1/r^2 point-density falloff, so the same structure scores
// similarly near and far. Count-based density is noise-robust: one stray point
// only adds 1 x r^2, unlike a z-extent metric where it inflates the whole cell.
#ifndef OUSTER_CLUSTER__BEV_DENSITY_HPP_
#define OUSTER_CLUSTER__BEV_DENSITY_HPP_

#include <cmath>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <vector>

#include "ouster_cluster/cluster_logic.hpp"  // ouster_cluster::Cloud

namespace ouster_cluster
{

namespace detail
{

inline std::size_t hashCellKey(std::int64_t k)
{
  std::uint64_t x = static_cast<std::uint64_t>(k);
  x ^= x >> 33; x *= 0xff51afd7ed558ccdULL;
  x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL;
  x ^= x >> 33;
  return static_cast<std::size_t>(x);
}

/// Open-addressing (linear probe) int64->V accumulate map, sized once for a
/// known upper bound of unique keys (load factor <= 0.5 so probes stay short).
/// Replaces std::unordered_map in the per-frame hot path: same operator[]
/// semantics (default-construct on first access), contiguous storage, no
/// per-node allocation. Iteration order differs from unordered_map, which is
/// fine here — iteration only feeds set membership and integer counts, never
/// float accumulation order (that follows point order via operator[]).
template<typename V>
class FlatMap
{
public:
  explicit FlatMap(std::size_t expected)
  {
    std::size_t cap = 16;
    while (cap < expected * 2) {cap <<= 1;}
    mask_ = cap - 1;
    keys_.resize(cap);
    vals_.resize(cap);
    used_.assign(cap, 0);
  }
  V & operator[](std::int64_t k)
  {
    std::size_t i = hashCellKey(k) & mask_;
    while (used_[i] && keys_[i] != k) {i = (i + 1) & mask_;}
    if (!used_[i]) {used_[i] = 1; keys_[i] = k; vals_[i] = V{}; ++size_;}
    return vals_[i];
  }
  template<typename F>
  void forEach(F && f) const
  {
    for (std::size_t i = 0; i <= mask_; ++i) {
      if (used_[i]) {f(keys_[i], vals_[i]);}
    }
  }
  std::size_t size() const {return size_;}

private:
  std::vector<std::int64_t> keys_;
  std::vector<V> vals_;
  std::vector<char> used_;
  std::size_t mask_ = 0, size_ = 0;
};

/// Companion int64 set with the same layout (insert / count only).
class FlatSet
{
public:
  explicit FlatSet(std::size_t expected)
  {
    std::size_t cap = 16;
    while (cap < expected * 2) {cap <<= 1;}
    mask_ = cap - 1;
    keys_.resize(cap);
    used_.assign(cap, 0);
  }
  void insert(std::int64_t k)
  {
    std::size_t i = hashCellKey(k) & mask_;
    while (used_[i] && keys_[i] != k) {i = (i + 1) & mask_;}
    if (!used_[i]) {used_[i] = 1; keys_[i] = k; ++size_;}
  }
  bool count(std::int64_t k) const
  {
    std::size_t i = hashCellKey(k) & mask_;
    while (used_[i]) {
      if (keys_[i] == k) {return true;}
      i = (i + 1) & mask_;
    }
    return false;
  }
  template<typename F>
  void forEach(F && f) const
  {
    for (std::size_t i = 0; i <= mask_; ++i) {
      if (used_[i]) {f(keys_[i]);}
    }
  }
  std::size_t size() const {return size_;}

private:
  std::vector<std::int64_t> keys_;
  std::vector<char> used_;
  std::size_t mask_ = 0, size_ = 0;
};

}  // namespace detail

/// Parameters for the BEV density map.
struct BevDensityParams
{
  double cell_size = 0.05;   // XY grid resolution (m)
  double min_density = 0.0;  // cells whose normalized density is below this are dropped
  double max_density = 0.0;  // if > 0, cells above this are dropped too (band-pass:
                             // cuts big face-on walls, keeps plate-like structures)
  double z_bin = 0.05;       // z-histogram bin height per cell (m)
  int min_pts_per_bin = 2;   // a z-bin counts as occupied only with this many points
                             // (a lone stray point cannot fake verticality)
  int min_z_bins = 0;        // if > 0, a cell needs this many occupied z-bins to
                             // survive (floor = 1 bin -> dropped; upright = several)
  bool dual_grid = false;    // run a second grid shifted half a cell in x/y and
                             // OR-combine: rescues structures split by a cell edge
  bool drop_lonely = false;  // drop surviving cells with NO surviving cell in
                             // their 8-neighbourhood (isolated noise; a real
                             // structure spans several adjacent cells)
};

/// Bin `cloud` onto an XY grid of `cell_size`. Each cell's density is the sum
/// of r^2 (= x^2 + y^2) over its points, i.e. point count x squared range —
/// distance-invariant and noise-robust. Returns a copy of the input where each
/// point's `intensity` is its cell's density; points in cells below
/// `min_density` are dropped. Empty input -> empty output.
inline Cloud::Ptr computeBevDensity(const Cloud::Ptr & cloud, const BevDensityParams & p)
{
  auto out = std::make_shared<Cloud>();
  if (!cloud || cloud->empty() || p.cell_size <= 0.0) {
    return out;
  }

  const double inv = 1.0 / p.cell_size;
  auto key = [inv](float x, float y) -> std::int64_t {
      const std::int64_t cx = static_cast<std::int64_t>(std::floor(x * inv));
      const std::int64_t cy = static_cast<std::int64_t>(std::floor(y * inv));
      return (cx << 32) ^ (cy & 0xffffffffLL);
    };
  const double zinv = (p.z_bin > 0.0) ? 1.0 / p.z_bin : 0.0;
  auto key3 = [inv, zinv](float x, float y, float z) -> std::int64_t {
      const std::int64_t cx = static_cast<std::int64_t>(std::floor(x * inv));
      const std::int64_t cy = static_cast<std::int64_t>(std::floor(y * inv));
      const std::int64_t cz = static_cast<std::int64_t>(std::floor(z * zinv));
      return ((cx & 0x1fffffLL) << 42) | ((cy & 0x1fffffLL) << 21) | (cz & 0x1fffffLL);
    };

  // pass 1 per grid: per-cell sum of squared range (count x dist^2), plus a
  // per-cell z-histogram (cell+z-bin -> point count) for the verticality
  // check. Grid 1 is shifted by half a cell in x/y, so a structure split by a
  // cell edge of grid 0 lies whole inside one cell of grid 1 (and vice versa).
  // Density stays the physical r^2 of the points — only the binning shifts.
  const int grids = p.dual_grid ? 2 : 1;
  const float off[2] = {0.f, static_cast<float>(0.5 * p.cell_size)};
  // flat open-addressing tables (see detail::FlatMap): unique keys are bounded
  // by the point count, so sizing them once up front keeps the load factor
  // <= 0.5 with zero rehashing. The unused second grid gets a minimal table.
  const std::size_t n_pts = cloud->size();
  const std::size_t n_zk = (p.min_z_bins > 0) ? n_pts : 0;
  detail::FlatMap<float> density[2] = {
    detail::FlatMap<float>(n_pts), detail::FlatMap<float>(grids > 1 ? n_pts : 0)};
  detail::FlatMap<int> zbin_pts[2] = {
    detail::FlatMap<int>(n_zk), detail::FlatMap<int>(grids > 1 ? n_zk : 0)};
  detail::FlatMap<int> occupied[2] = {
    detail::FlatMap<int>(n_zk), detail::FlatMap<int>(grids > 1 ? n_zk : 0)};
  for (int g = 0; g < grids; ++g) {
    for (const auto & pt : cloud->points) {
      density[g][key(pt.x + off[g], pt.y + off[g])] += pt.x * pt.x + pt.y * pt.y;
      if (p.min_z_bins > 0) {++zbin_pts[g][key3(pt.x + off[g], pt.y + off[g], pt.z)];}
    }
    // per-cell count of occupied z-bins (bins holding >= min_pts_per_bin points)
    if (p.min_z_bins > 0) {
      zbin_pts[g].forEach(
        [&](std::int64_t k, int cnt) {
          if (cnt >= p.min_pts_per_bin) {
            ++occupied[g][k >> 21];  // strip the z-bin bits -> XY cell id
          }
        });
    }
  }

  // per-grid set of cells passing every per-cell gate (density band + z-bin
  // occupancy). Cell survival only depends on the cell, so deciding it once
  // here lets the lonely check below see the final neighbourhood.
  auto packKey = [](std::int64_t cx, std::int64_t cy) -> std::int64_t {
      return (cx << 32) ^ (cy & 0xffffffffLL);  // == key() for that cell
    };
  detail::FlatSet keep[2] = {
    detail::FlatSet(density[0].size()), detail::FlatSet(density[1].size())};
  for (int g = 0; g < grids; ++g) {
    density[g].forEach(
      [&](std::int64_t k, float dens) {
        if (dens < static_cast<float>(p.min_density)) {return;}
        if (p.max_density > 0.0 && dens > static_cast<float>(p.max_density)) {return;}
        const std::int64_t cx = k >> 32;
        const std::int64_t cy = static_cast<std::int32_t>(
          static_cast<std::uint32_t>(k & 0xffffffffLL));
        if (p.min_z_bins > 0 &&
          occupied[g][((cx & 0x1fffffLL) << 21) | (cy & 0x1fffffLL)] < p.min_z_bins)
        {
          return;
        }
        keep[g].insert(k);
      });
    // drop lonely cells: no surviving cell in the 8-neighbourhood -> isolated
    // noise (a real structure spans several adjacent cells)
    if (p.drop_lonely) {
      detail::FlatSet connected(keep[g].size());
      keep[g].forEach(
        [&](std::int64_t k) {
          const std::int64_t cx = k >> 32;
          const std::int64_t cy = static_cast<std::int32_t>(
            static_cast<std::uint32_t>(k & 0xffffffffLL));
          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              if (dx == 0 && dy == 0) {continue;}
              if (keep[g].count(packKey(cx + dx, cy + dy))) {
                connected.insert(k);
                return;
              }
            }
          }
        });
      keep[g] = std::move(connected);
    }
  }

  // pass 2: a point survives if its cell passes in ANY grid (OR-combine);
  // intensity = the passing grid's density, primary grid first
  out->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    for (int g = 0; g < grids; ++g) {
      const std::int64_t k = key(pt.x + off[g], pt.y + off[g]);
      if (!keep[g].count(k)) {continue;}
      pcl::PointXYZI q = pt;
      q.intensity = density[g][k];
      out->push_back(q);
      break;
    }
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

}  // namespace ouster_cluster

#endif  // OUSTER_CLUSTER__BEV_DENSITY_HPP_
