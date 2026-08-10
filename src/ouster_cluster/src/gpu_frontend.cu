// CUDA implementation of the fused front-end (see gpu_frontend.hpp).
//
// Determinism: per-cell r^2 sums accumulate in FIXED-POINT (u64, 2^20 scale)
// via atomicAdd, so the result is independent of thread scheduling. Every
// point-cloud compaction goes flag -> exclusive scan -> scatter, preserving
// raw point order exactly like the sequential CPU loops.
//
// Hash tables are open-addressing int64-key arrays (same murmur mix + probe
// as detail::FlatMap on the CPU); EMPTY is a byte-repeating sentinel so the
// per-frame reset is a plain cudaMemsetAsync. Real cell keys can never equal
// it (their magnitudes are bounded by sensor range / cell_size).
#include "gpu_frontend_cuda.h"

#include <cuda_runtime.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ouster_cluster
{
namespace gpu
{

namespace
{

constexpr long long kEmpty = 0x8080808080808080LL;  // memset-able sentinel
constexpr double kScale = 1048576.0;                // 2^20 fixed-point scale
constexpr int kBlock = 256;

#define CUDA_OK(call) \
  do { \
    cudaError_t e_ = (call); \
    if (e_ != cudaSuccess) { \
      std::fprintf( \
        stderr, "[gpu_frontend] %s:%d %s\n", __FILE__, __LINE__, cudaGetErrorString(e_)); \
      return false; \
    } \
  } while (0)

__host__ __device__ inline std::size_t hashCellKey(long long k)
{
  unsigned long long x = static_cast<unsigned long long>(k);
  x ^= x >> 33; x *= 0xff51afd7ed558ccdULL;
  x ^= x >> 33; x *= 0xc4ceb9fe1a85ec53ULL;
  x ^= x >> 33;
  return static_cast<std::size_t>(x);
}

// --- key packing: bit-for-bit identical to bev_density.hpp ---
__device__ inline long long keyXY(float x, float y, double inv)
{
  const long long cx = static_cast<long long>(floor(static_cast<double>(x) * inv));
  const long long cy = static_cast<long long>(floor(static_cast<double>(y) * inv));
  return (cx << 32) ^ (cy & 0xffffffffLL);
}
__device__ inline long long keyXYZ(float x, float y, float z, double inv, double zinv)
{
  const long long cx = static_cast<long long>(floor(static_cast<double>(x) * inv));
  const long long cy = static_cast<long long>(floor(static_cast<double>(y) * inv));
  const long long cz = static_cast<long long>(floor(static_cast<double>(z) * zinv));
  return ((cx & 0x1fffffLL) << 42) | ((cy & 0x1fffffLL) << 21) | (cz & 0x1fffffLL);
}
__device__ inline long long packXY(long long cx, long long cy)
{
  return (cx << 32) ^ (cy & 0xffffffffLL);
}
__device__ inline long long xyToOccKey(long long k)
{
  const long long cx = k >> 32;
  const long long cy = static_cast<int>(static_cast<unsigned int>(k & 0xffffffffLL));
  return ((cx & 0x1fffffLL) << 21) | (cy & 0x1fffffLL);
}

// insert-or-find `k`, then atomically add `add` to its value slot
__device__ inline void accumAdd(
  long long * keys, unsigned long long * vals, std::size_t mask,
  long long k, unsigned long long add)
{
  std::size_t i = hashCellKey(k) & mask;
  while (true) {
    const long long cur = keys[i];
    if (cur == k) {
      atomicAdd(&vals[i], add);
      return;
    }
    if (cur == kEmpty) {
      const long long old = static_cast<long long>(
        atomicCAS(
          reinterpret_cast<unsigned long long *>(&keys[i]),
          static_cast<unsigned long long>(kEmpty),
          static_cast<unsigned long long>(k)));
      if (old == kEmpty || old == k) {
        atomicAdd(&vals[i], add);
        return;
      }
    }
    i = (i + 1) & mask;
  }
}

// find-only: slot index of `k`, or SIZE_MAX
__device__ inline std::size_t findSlot(
  const long long * keys, std::size_t mask, long long k)
{
  std::size_t i = hashCellKey(k) & mask;
  while (true) {
    const long long cur = keys[i];
    if (cur == k) {return i;}
    if (cur == kEmpty) {return static_cast<std::size_t>(-1);}
    i = (i + 1) & mask;
  }
}

struct RawLayout
{
  std::uint32_t step, ox, oy, oz, os;
  bool s_u16;
};

__device__ inline void readPoint(
  const std::uint8_t * raw, std::size_t i, const RawLayout & L, GpuPoint & p)
{
  const std::uint8_t * b = raw + i * L.step;
  p.x = *reinterpret_cast<const float *>(b + L.ox);
  p.y = *reinterpret_cast<const float *>(b + L.oy);
  p.z = *reinterpret_cast<const float *>(b + L.oz);
  p.intensity = L.s_u16 ?
    static_cast<float>(*reinterpret_cast<const std::uint16_t *>(b + L.os)) :
    *reinterpret_cast<const float *>(b + L.os);
}

// conv+cut gate (fromOusterMsg fused overload) — float math as on CPU
__device__ inline bool convKeep(
  const GpuPoint & p, bool band, float tan_min, float tan_max)
{
  if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z)) {return false;}
  if (!band) {return true;}
  const float r_xy = hypotf(p.x, p.y);
  return p.z >= r_xy * tan_min && p.z <= r_xy * tan_max;
}

// preprocess crop gate (preprocessCloud step 1) — float sum, double compare
__device__ inline bool prepKeep(
  const GpuPoint & p, double z_min, double z_max, bool do_range, double r2)
{
  if (p.z < z_min || p.z > z_max) {return false;}
  if (do_range && p.x * p.x + p.y * p.y + p.z * p.z > r2) {return false;}
  return true;
}

__global__ void flagKernel(
  const std::uint8_t * raw, std::size_t n, RawLayout L,
  bool band, float tan_min, float tan_max,
  double z_min, double z_max, bool do_range, double r2,
  int * flag_conv, int * flag_prep)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {return;}
  GpuPoint p;
  readPoint(raw, i, L, p);
  const bool c = convKeep(p, band, tan_min, tan_max);
  if (flag_conv) {flag_conv[i] = c ? 1 : 0;}
  flag_prep[i] = (c && prepKeep(p, z_min, z_max, do_range, r2)) ? 1 : 0;
}

__global__ void scatterKernel(
  const std::uint8_t * raw, std::size_t n, RawLayout L,
  const int * flags, const int * pos, GpuPoint * out)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n || !flags[i]) {return;}
  GpuPoint p;
  readPoint(raw, i, L, p);
  out[pos[i]] = p;
}

__global__ void densAccumKernel(
  const GpuPoint * pts, std::size_t n, int grids, float off1,
  double inv, double zinv, int min_z_bins,
  long long * xy_keys0, unsigned long long * xy_vals0,
  long long * xy_keys1, unsigned long long * xy_vals1,
  long long * z_keys0, unsigned long long * z_vals0,
  long long * z_keys1, unsigned long long * z_vals1,
  std::size_t mask)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {return;}
  const GpuPoint p = pts[i];
  const double r2 = static_cast<double>(p.x * p.x + p.y * p.y);  // float mult as CPU
  const unsigned long long fx =
    static_cast<unsigned long long>(llrint(r2 * kScale));
  for (int g = 0; g < grids; ++g) {
    const float off = g ? off1 : 0.f;
    long long * xk = g ? xy_keys1 : xy_keys0;
    unsigned long long * xv = g ? xy_vals1 : xy_vals0;
    accumAdd(xk, xv, mask, keyXY(p.x + off, p.y + off, inv), fx);
    if (min_z_bins > 0) {
      long long * zk = g ? z_keys1 : z_keys0;
      unsigned long long * zv = g ? z_vals1 : z_vals0;
      accumAdd(zk, zv, mask, keyXYZ(p.x + off, p.y + off, p.z, inv, zinv), 1ULL);
    }
  }
}

__global__ void occupiedKernel(
  const long long * z_keys, const unsigned long long * z_vals, std::size_t cap,
  int min_pts_per_bin,
  long long * occ_keys, unsigned long long * occ_vals, std::size_t mask)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= cap || z_keys[i] == kEmpty) {return;}
  if (static_cast<int>(z_vals[i]) >= min_pts_per_bin) {
    accumAdd(occ_keys, occ_vals, mask, z_keys[i] >> 21, 1ULL);
  }
}

__global__ void keepKernel(
  const long long * xy_keys, const unsigned long long * xy_vals, std::size_t cap,
  unsigned long long min_fx, unsigned long long max_fx, bool has_max,
  int min_z_bins,
  const long long * occ_keys, const unsigned long long * occ_vals, std::size_t mask,
  unsigned char * keep)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= cap || xy_keys[i] == kEmpty) {return;}
  const unsigned long long v = xy_vals[i];
  if (v < min_fx) {return;}
  if (has_max && v > max_fx) {return;}
  if (min_z_bins > 0) {
    const std::size_t s = findSlot(occ_keys, mask, xyToOccKey(xy_keys[i]));
    const int cnt = (s == static_cast<std::size_t>(-1)) ? 0 :
      static_cast<int>(occ_vals[s]);
    if (cnt < min_z_bins) {return;}
  }
  keep[i] = 1;
}

__global__ void lonelyKernel(
  const long long * xy_keys, std::size_t cap, std::size_t mask,
  const unsigned char * keep, unsigned char * connected)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= cap || !keep[i]) {return;}
  const long long k = xy_keys[i];
  const long long cx = k >> 32;
  const long long cy = static_cast<int>(static_cast<unsigned int>(k & 0xffffffffLL));
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {continue;}
      const std::size_t s = findSlot(xy_keys, mask, packXY(cx + dx, cy + dy));
      if (s != static_cast<std::size_t>(-1) && keep[s]) {
        connected[i] = 1;
        return;
      }
    }
  }
}

// pass 2: point survives if its cell is kept in ANY grid (primary first);
// intensity = that grid's density. Writes flag + intensity per point.
__global__ void pass2FlagKernel(
  const GpuPoint * pts, std::size_t n, int grids, float off1, double inv,
  const long long * xy_keys0, const unsigned long long * xy_vals0,
  const unsigned char * keep0,
  const long long * xy_keys1, const unsigned long long * xy_vals1,
  const unsigned char * keep1,
  std::size_t mask, int * flags, float * dens_out)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) {return;}
  const GpuPoint p = pts[i];
  flags[i] = 0;
  for (int g = 0; g < grids; ++g) {
    const float off = g ? off1 : 0.f;
    const long long * xk = g ? xy_keys1 : xy_keys0;
    const unsigned long long * xv = g ? xy_vals1 : xy_vals0;
    const unsigned char * kp = g ? keep1 : keep0;
    const std::size_t s = findSlot(xk, mask, keyXY(p.x + off, p.y + off, inv));
    if (s == static_cast<std::size_t>(-1) || !kp[s]) {continue;}
    flags[i] = 1;
    dens_out[i] = static_cast<float>(static_cast<double>(xv[s]) / kScale);
    return;
  }
}

__global__ void pass2ScatterKernel(
  const GpuPoint * pts, std::size_t n, const int * flags, const int * pos,
  const float * dens, GpuPoint * out)
{
  const std::size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n || !flags[i]) {return;}
  GpuPoint p = pts[i];
  p.intensity = dens[i];
  out[pos[i]] = p;
}

// single-block exclusive scan over int flags (n <= a few hundred k): simple
// grid-stride Hillis-Steele on a work buffer would need multiple launches;
// instead use a two-level scan: per-block sums then offset add.
__global__ void blockScanKernel(const int * in, int * out, int * block_sums, std::size_t n)
{
  __shared__ int sh[kBlock];
  const std::size_t gi = blockIdx.x * blockDim.x + threadIdx.x;
  int v = (gi < n) ? in[gi] : 0;
  const int orig = v;
  sh[threadIdx.x] = v;
  __syncthreads();
  for (int ofs = 1; ofs < kBlock; ofs <<= 1) {
    int add = (threadIdx.x >= ofs) ? sh[threadIdx.x - ofs] : 0;
    __syncthreads();
    sh[threadIdx.x] += add;
    __syncthreads();
  }
  if (gi < n) {out[gi] = sh[threadIdx.x] - orig;}  // exclusive
  if (threadIdx.x == kBlock - 1) {block_sums[blockIdx.x] = sh[threadIdx.x];}
}

__global__ void addBlockOffsetsKernel(int * out, const int * block_offsets, std::size_t n)
{
  const std::size_t gi = blockIdx.x * blockDim.x + threadIdx.x;
  if (gi < n) {out[gi] += block_offsets[blockIdx.x];}
}

}  // namespace

struct CudaFrontend::State
{
  cudaStream_t stream = nullptr;
  std::size_t n_cap = 0;        // allocated point capacity
  std::size_t table_cap = 0;    // hash table capacity (pow2)
  // device buffers
  std::uint8_t * d_raw = nullptr;
  std::size_t raw_bytes = 0;
  int * d_flag_conv = nullptr;
  int * d_flag_prep = nullptr;
  int * d_pos = nullptr;
  int * d_block_sums = nullptr;
  int * d_block_offsets = nullptr;
  std::size_t n_blocks_cap = 0;
  GpuPoint * d_conv = nullptr;   // compacted conv+cut points
  GpuPoint * d_prep = nullptr;   // compacted preprocessed points
  GpuPoint * d_out = nullptr;    // final density cloud
  float * d_dens_pt = nullptr;   // per-point density (pass 2)
  long long * d_xy_keys[2] = {nullptr, nullptr};
  unsigned long long * d_xy_vals[2] = {nullptr, nullptr};
  long long * d_z_keys[2] = {nullptr, nullptr};
  unsigned long long * d_z_vals[2] = {nullptr, nullptr};
  long long * d_occ_keys[2] = {nullptr, nullptr};
  unsigned long long * d_occ_vals[2] = {nullptr, nullptr};
  unsigned char * d_keep[2] = {nullptr, nullptr};
  unsigned char * d_conn[2] = {nullptr, nullptr};
  // pinned host staging
  std::uint8_t * h_raw = nullptr;
  GpuPoint * h_out = nullptr;

  ~State()
  {
    if (stream) {cudaStreamDestroy(stream);}
    cudaFree(d_raw); cudaFree(d_flag_conv); cudaFree(d_flag_prep); cudaFree(d_pos);
    cudaFree(d_block_sums); cudaFree(d_block_offsets);
    cudaFree(d_conv); cudaFree(d_prep); cudaFree(d_out); cudaFree(d_dens_pt);
    for (int g = 0; g < 2; ++g) {
      cudaFree(d_xy_keys[g]); cudaFree(d_xy_vals[g]);
      cudaFree(d_z_keys[g]); cudaFree(d_z_vals[g]);
      cudaFree(d_occ_keys[g]); cudaFree(d_occ_vals[g]);
      cudaFree(d_keep[g]); cudaFree(d_conn[g]);
    }
    cudaFreeHost(h_raw); cudaFreeHost(h_out);
  }
};

bool CudaFrontend::deviceAvailable()
{
  int n = 0;
  return cudaGetDeviceCount(&n) == cudaSuccess && n > 0;
}

CudaFrontend::CudaFrontend()
: s_(new State) {}

CudaFrontend::~CudaFrontend() = default;

namespace
{

// exclusive scan of d_flags[n] into d_pos[n]; `total` = sum of flags.
// Two-level: per-block scan, host scan of block sums (block count is tiny),
// then a device offset add. Stable — order of set flags is preserved.
bool exclusiveScan(
  int * d_block_sums, int * d_block_offsets,
  const int * d_flags, int * d_pos, std::size_t n,
  cudaStream_t st, std::size_t & total)
{
  const std::size_t blocks = (n + kBlock - 1) / kBlock;
  blockScanKernel<<<blocks, kBlock, 0, st>>>(d_flags, d_pos, d_block_sums, n);
  std::vector<int> sums(blocks);
  CUDA_OK(
    cudaMemcpyAsync(
      sums.data(), d_block_sums, blocks * sizeof(int), cudaMemcpyDeviceToHost, st));
  CUDA_OK(cudaStreamSynchronize(st));
  std::vector<int> offs(blocks);
  int acc = 0;
  for (std::size_t b = 0; b < blocks; ++b) {offs[b] = acc; acc += sums[b];}
  total = static_cast<std::size_t>(acc);
  CUDA_OK(
    cudaMemcpyAsync(
      d_block_offsets, offs.data(), blocks * sizeof(int), cudaMemcpyHostToDevice, st));
  addBlockOffsetsKernel<<<blocks, kBlock, 0, st>>>(d_pos, d_block_offsets, n);
  return true;
}

}  // namespace

bool CudaFrontend::run(
  const std::uint8_t * raw, std::size_t n_points,
  const FrontendParams & p,
  std::vector<GpuPoint> & out_conv,
  std::vector<GpuPoint> & out_prep,
  std::vector<GpuPoint> & out_density,
  std::size_t & n_conv, std::size_t & n_prep)
{
  State * s = s_.get();
  if (!s->stream) {CUDA_OK(cudaStreamCreate(&s->stream));}
  cudaStream_t st = s->stream;

  // (re)allocate for this point count
  if (n_points > s->n_cap) {
    const std::size_t n = n_points;
    std::size_t cap = 16;
    while (cap < n * 2) {cap <<= 1;}
    const std::size_t blocks = (n + kBlock - 1) / kBlock;
    // free old buffers, nulling so a failed re-alloc cannot double-free later
    auto refree = [](auto & ptr) {cudaFree(ptr); ptr = nullptr;};
    refree(s->d_flag_conv); refree(s->d_flag_prep); refree(s->d_pos);
    refree(s->d_block_sums); refree(s->d_block_offsets);
    refree(s->d_conv); refree(s->d_prep); refree(s->d_out); refree(s->d_dens_pt);
    for (int g = 0; g < 2; ++g) {
      refree(s->d_xy_keys[g]); refree(s->d_xy_vals[g]);
      refree(s->d_z_keys[g]); refree(s->d_z_vals[g]);
      refree(s->d_occ_keys[g]); refree(s->d_occ_vals[g]);
      refree(s->d_keep[g]); refree(s->d_conn[g]);
    }
    cudaFreeHost(s->h_out); s->h_out = nullptr;
    CUDA_OK(cudaMalloc(&s->d_flag_conv, n * sizeof(int)));
    CUDA_OK(cudaMalloc(&s->d_flag_prep, n * sizeof(int)));
    CUDA_OK(cudaMalloc(&s->d_pos, n * sizeof(int)));
    CUDA_OK(cudaMalloc(&s->d_block_sums, blocks * sizeof(int)));
    CUDA_OK(cudaMalloc(&s->d_block_offsets, blocks * sizeof(int)));
    CUDA_OK(cudaMalloc(&s->d_conv, n * sizeof(GpuPoint)));
    CUDA_OK(cudaMalloc(&s->d_prep, n * sizeof(GpuPoint)));
    CUDA_OK(cudaMalloc(&s->d_out, n * sizeof(GpuPoint)));
    CUDA_OK(cudaMalloc(&s->d_dens_pt, n * sizeof(float)));
    for (int g = 0; g < 2; ++g) {
      CUDA_OK(cudaMalloc(&s->d_xy_keys[g], cap * sizeof(long long)));
      CUDA_OK(cudaMalloc(&s->d_xy_vals[g], cap * sizeof(unsigned long long)));
      CUDA_OK(cudaMalloc(&s->d_z_keys[g], cap * sizeof(long long)));
      CUDA_OK(cudaMalloc(&s->d_z_vals[g], cap * sizeof(unsigned long long)));
      CUDA_OK(cudaMalloc(&s->d_occ_keys[g], cap * sizeof(long long)));
      CUDA_OK(cudaMalloc(&s->d_occ_vals[g], cap * sizeof(unsigned long long)));
      CUDA_OK(cudaMalloc(&s->d_keep[g], cap));
      CUDA_OK(cudaMalloc(&s->d_conn[g], cap));
    }
    CUDA_OK(cudaMallocHost(&s->h_out, n * sizeof(GpuPoint)));
    s->n_cap = n;
    s->table_cap = cap;
    s->n_blocks_cap = blocks;
  }
  const std::size_t bytes = n_points * p.point_step;
  if (bytes > s->raw_bytes) {
    cudaFree(s->d_raw); cudaFreeHost(s->h_raw);
    CUDA_OK(cudaMalloc(&s->d_raw, bytes));
    CUDA_OK(cudaMallocHost(&s->h_raw, bytes));
    s->raw_bytes = bytes;
  }

  // upload raw buffer via pinned staging
  memcpy(s->h_raw, raw, bytes);
  CUDA_OK(cudaMemcpyAsync(s->d_raw, s->h_raw, bytes, cudaMemcpyHostToDevice, st));

  const RawLayout L{p.point_step, p.off_x, p.off_y, p.off_z, p.off_scalar, p.scalar_is_u16};
  const std::size_t blocks = (n_points + kBlock - 1) / kBlock;
  const bool do_range = p.max_range > 0.0;
  const double r2 = p.max_range * p.max_range;

  // stage 1: conv+cut and prep flags in one read of the raw buffer
  flagKernel<<<blocks, kBlock, 0, st>>>(
    s->d_raw, n_points, L, p.elev_enable, p.tan_min, p.tan_max,
    p.z_min, p.z_max, do_range, r2,
    s->d_flag_conv, s->d_flag_prep);

  // n_conv (empty-input warning + timing "n=") comes from the scan either way;
  // the scatter + copy-back of the ~26k debug cloud only runs when requested.
  if (!exclusiveScan(
      s->d_block_sums, s->d_block_offsets,
      s->d_flag_conv, s->d_pos, n_points, st, n_conv)) {return false;}
  if (p.want_conv && n_conv > 0) {
    scatterKernel<<<blocks, kBlock, 0, st>>>(
      s->d_raw, n_points, L, s->d_flag_conv, s->d_pos, s->d_conv);
    out_conv.resize(n_conv);
    CUDA_OK(
      cudaMemcpyAsync(
        s->h_out, s->d_conv, n_conv * sizeof(GpuPoint), cudaMemcpyDeviceToHost, st));
    CUDA_OK(cudaStreamSynchronize(st));
    memcpy(out_conv.data(), s->h_out, n_conv * sizeof(GpuPoint));
  }

  if (!exclusiveScan(
      s->d_block_sums, s->d_block_offsets,
      s->d_flag_prep, s->d_pos, n_points, st, n_prep)) {return false;}
  scatterKernel<<<blocks, kBlock, 0, st>>>(
    s->d_raw, n_points, L, s->d_flag_prep, s->d_pos, s->d_prep);

  if (n_prep == 0) {
    out_prep.clear();
    out_density.clear();
    return true;
  }

  // stage 2: density accumulation
  const int grids = p.dual_grid ? 2 : 1;
  const float off1 = static_cast<float>(0.5 * p.cell_size);
  const double inv = 1.0 / p.cell_size;
  const double zinv = (p.z_bin > 0.0) ? 1.0 / p.z_bin : 0.0;
  const std::size_t cap = s->table_cap;
  const std::size_t mask = cap - 1;
  for (int g = 0; g < grids; ++g) {
    CUDA_OK(cudaMemsetAsync(s->d_xy_keys[g], 0x80, cap * sizeof(long long), st));
    CUDA_OK(cudaMemsetAsync(s->d_xy_vals[g], 0, cap * sizeof(unsigned long long), st));
    CUDA_OK(cudaMemsetAsync(s->d_keep[g], 0, cap, st));
    if (p.min_z_bins > 0) {
      CUDA_OK(cudaMemsetAsync(s->d_z_keys[g], 0x80, cap * sizeof(long long), st));
      CUDA_OK(cudaMemsetAsync(s->d_z_vals[g], 0, cap * sizeof(unsigned long long), st));
      CUDA_OK(cudaMemsetAsync(s->d_occ_keys[g], 0x80, cap * sizeof(long long), st));
      CUDA_OK(cudaMemsetAsync(s->d_occ_vals[g], 0, cap * sizeof(unsigned long long), st));
    }
    if (p.drop_lonely) {
      CUDA_OK(cudaMemsetAsync(s->d_conn[g], 0, cap, st));
    }
  }
  const std::size_t pblocks = (n_prep + kBlock - 1) / kBlock;
  densAccumKernel<<<pblocks, kBlock, 0, st>>>(
    s->d_prep, n_prep, grids, off1, inv, zinv, p.min_z_bins,
    s->d_xy_keys[0], s->d_xy_vals[0], s->d_xy_keys[1], s->d_xy_vals[1],
    s->d_z_keys[0], s->d_z_vals[0], s->d_z_keys[1], s->d_z_vals[1], mask);

  const std::size_t tblocks = (cap + kBlock - 1) / kBlock;
  const unsigned long long min_fx =
    static_cast<unsigned long long>(llrint(p.min_density * kScale));
  const bool has_max = p.max_density > 0.0;
  const unsigned long long max_fx =
    has_max ? static_cast<unsigned long long>(llrint(p.max_density * kScale)) : ~0ULL;
  for (int g = 0; g < grids; ++g) {
    if (p.min_z_bins > 0) {
      occupiedKernel<<<tblocks, kBlock, 0, st>>>(
        s->d_z_keys[g], s->d_z_vals[g], cap, p.min_pts_per_bin,
        s->d_occ_keys[g], s->d_occ_vals[g], mask);
    }
    keepKernel<<<tblocks, kBlock, 0, st>>>(
      s->d_xy_keys[g], s->d_xy_vals[g], cap, min_fx, max_fx, has_max,
      p.min_z_bins, s->d_occ_keys[g], s->d_occ_vals[g], mask, s->d_keep[g]);
    if (p.drop_lonely) {
      lonelyKernel<<<tblocks, kBlock, 0, st>>>(
        s->d_xy_keys[g], cap, mask, s->d_keep[g], s->d_conn[g]);
      // swap: connected becomes the effective keep
      unsigned char * t = s->d_keep[g];
      s->d_keep[g] = s->d_conn[g];
      s->d_conn[g] = t;
    }
  }

  // stage 3: pass 2 — stable compaction of surviving points
  pass2FlagKernel<<<pblocks, kBlock, 0, st>>>(
    s->d_prep, n_prep, grids, off1, inv,
    s->d_xy_keys[0], s->d_xy_vals[0], s->d_keep[0],
    s->d_xy_keys[1], s->d_xy_vals[1], s->d_keep[1],
    mask, s->d_flag_prep, s->d_dens_pt);
  std::size_t n_out = 0;
  if (!exclusiveScan(
      s->d_block_sums, s->d_block_offsets,
      s->d_flag_prep, s->d_pos, n_prep, st, n_out)) {return false;}
  pass2ScatterKernel<<<pblocks, kBlock, 0, st>>>(
    s->d_prep, n_prep, s->d_flag_prep, s->d_pos, s->d_dens_pt, s->d_out);

  out_density.resize(n_out);
  if (n_out > 0) {
    CUDA_OK(
      cudaMemcpyAsync(
        s->h_out, s->d_out, n_out * sizeof(GpuPoint), cudaMemcpyDeviceToHost, st));
  }
  CUDA_OK(cudaStreamSynchronize(st));
  CUDA_OK(cudaGetLastError());
  if (n_out > 0) {
    memcpy(out_density.data(), s->h_out, n_out * sizeof(GpuPoint));
  }

  // preprocessed cloud (needed on host for the reflectivity gate) — d_prep is
  // final by now; reuse the pinned buffer after the density copy completed
  out_prep.resize(n_prep);
  CUDA_OK(
    cudaMemcpyAsync(
      s->h_out, s->d_prep, n_prep * sizeof(GpuPoint), cudaMemcpyDeviceToHost, st));
  CUDA_OK(cudaStreamSynchronize(st));
  memcpy(out_prep.data(), s->h_out, n_prep * sizeof(GpuPoint));
  return true;
}

}  // namespace gpu
}  // namespace ouster_cluster
