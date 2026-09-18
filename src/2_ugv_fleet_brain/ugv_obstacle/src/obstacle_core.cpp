#include "ugv_obstacle/obstacle_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <vector>

namespace ugv_obstacle {

namespace {
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Encode integer cell coords into one 64-bit key.
inline int64_t cell_key(int ix, int iy) {
  return (static_cast<int64_t>(ix) << 32) ^ (static_cast<int64_t>(iy) & 0xffffffff);
}
}  // namespace

void ObstacleCore::set_global_dem(const Eigen::MatrixXf& dem, double res,
                                  double origin_x, double origin_y) {
  dem_ = dem;
  res_ = res;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  rows_ = static_cast<int>(dem.rows());
  cols_ = static_cast<int>(dem.cols());
}

bool ObstacleCore::load_global_dem(const std::string& path, double res,
                                   double origin_x, double origin_y) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    return false;
  }
  uint32_t cols = 0, rows = 0;
  file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
  file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
  if (cols == 0 || rows == 0 || cols > 100000 || rows > 100000) {
    return false;
  }
  // TRN binary is column-major float32; Eigen default is column-major.
  Eigen::MatrixXf dem(rows, cols);
  file.read(reinterpret_cast<char*>(dem.data()),
            static_cast<std::streamsize>(rows) * cols * sizeof(float));
  if (!file) {
    return false;
  }
  set_global_dem(dem, res, origin_x, origin_y);
  return true;
}

float ObstacleCore::dem_at(double x, double y) const {
  if (rows_ < 2 || cols_ < 2) {
    return kNaN;
  }
  const double col = (x - origin_x_) / res_;  // X index
  const double row = (y - origin_y_) / res_;  // Y index
  if (col < 0.0 || row < 0.0 || col > cols_ - 1 || row > rows_ - 1) {
    return kNaN;
  }
  const int c0 = static_cast<int>(std::floor(col));
  const int r0 = static_cast<int>(std::floor(row));
  const int c1 = std::min(c0 + 1, cols_ - 1);
  const int r1 = std::min(r0 + 1, rows_ - 1);
  const double fc = col - c0;
  const double fr = row - r0;
  const float v00 = dem_(r0, c0);
  const float v01 = dem_(r0, c1);
  const float v10 = dem_(r1, c0);
  const float v11 = dem_(r1, c1);
  const double top = v00 * (1.0 - fc) + v01 * fc;
  const double bot = v10 * (1.0 - fc) + v11 * fc;
  return static_cast<float>(top * (1.0 - fr) + bot * fr);
}

std::vector<uint8_t> ObstacleCore::classify(const Eigen::MatrixXf& pts_map,
                                            double sensor_x, double sensor_y,
                                            double confidence) const {
  const int n = static_cast<int>(pts_map.rows());
  std::vector<uint8_t> mask(n, 0);
  if (n == 0) {
    return mask;
  }

  const double conf = std::max(0.0, std::min(1.0, confidence));
  const double tau_prior_eff = cfg_.tau_prior + (1.0 - conf) * cfg_.low_conf_relax;
  const double self_r2 = cfg_.self_radius * cfg_.self_radius;
  const double max_r2 = cfg_.max_range * cfg_.max_range;
  const double inv_cell = 1.0 / cfg_.cell_size;

  // Pass 1: per-cell minimum z and point count (for the local-jump signal).
  std::unordered_map<int64_t, float> cell_min;
  std::unordered_map<int64_t, int> cell_cnt;
  cell_min.reserve(n);
  cell_cnt.reserve(n);

  std::vector<uint8_t> valid(n, 0);  // passed self/range/band gates
  for (int i = 0; i < n; ++i) {
    const double x = pts_map(i, 0);
    const double y = pts_map(i, 1);
    const double z = pts_map(i, 2);
    const double dx = x - sensor_x;
    const double dy = y - sensor_y;
    const double r2 = dx * dx + dy * dy;
    if (r2 < self_r2 || r2 > max_r2) continue;
    if (z < cfg_.min_height || z > cfg_.max_height) continue;
    valid[i] = 1;
    const int ix = static_cast<int>(std::floor(x * inv_cell));
    const int iy = static_cast<int>(std::floor(y * inv_cell));
    const int64_t key = cell_key(ix, iy);
    auto it = cell_min.find(key);
    if (it == cell_min.end()) {
      cell_min.emplace(key, static_cast<float>(z));
      cell_cnt.emplace(key, 1);
    } else {
      if (z < it->second) it->second = static_cast<float>(z);
      cell_cnt[key] += 1;
    }
  }

  // Estimate the constant z-datum offset between map-frame LiDAR z and the
  // a-priori DEM. TRN matches terrain SHAPE via (mean-normalised) NCC, so the
  // map frame and the DEM need NOT share an absolute vertical datum. The
  // absolute DEM-prior test below (z - h > tau) would otherwise fire on flat
  // ground whenever that offset exceeds tau_prior, producing a phantom
  // robot-fixed obstacle blob. Using each cell's minimum z (a ground proxy),
  // the median of (z_min - dem) is a robust estimate of the offset; subtract
  // it so DEM-prior reacts only to genuine positive deviations. The local-jump
  // signal is neighbourhood-relative and already datum-free.
  double dem_offset = 0.0;
  if (rows_ >= 2 && cols_ >= 2) {
    std::vector<double> offs;
    offs.reserve(cell_min.size());
    for (const auto& kv : cell_min) {
      const int cx_i = static_cast<int>(kv.first >> 32);
      const int cy_i = static_cast<int>(static_cast<int32_t>(kv.first & 0xffffffff));
      const double cx = (cx_i + 0.5) * cfg_.cell_size;
      const double cy = (cy_i + 0.5) * cfg_.cell_size;
      const float h = dem_at(cx, cy);
      if (std::isfinite(h)) offs.push_back(static_cast<double>(kv.second) - h);
    }
    if (offs.size() >= 8) {
      const size_t mid = offs.size() / 2;
      std::nth_element(offs.begin(), offs.begin() + mid, offs.end());
      dem_offset = offs[mid];
    }
  }

  // Pass 2: classify.
  for (int i = 0; i < n; ++i) {
    if (!valid[i]) continue;
    const double x = pts_map(i, 0);
    const double y = pts_map(i, 1);
    const double z = pts_map(i, 2);
    const int ix = static_cast<int>(std::floor(x * inv_cell));
    const int iy = static_cast<int>(std::floor(y * inv_cell));

    // Local-jump: min z over the 3x3 cell neighbourhood (lets an occluded
    // obstacle cell borrow ground from neighbours).
    float ground = std::numeric_limits<float>::infinity();
    int cnt = 0;
    for (int a = -1; a <= 1; ++a) {
      for (int b = -1; b <= 1; ++b) {
        auto it = cell_min.find(cell_key(ix + a, iy + b));
        if (it != cell_min.end()) {
          ground = std::min(ground, it->second);
          if (a == 0 && b == 0) cnt = cell_cnt.at(cell_key(ix, iy));
        }
      }
    }
    const bool enough = cnt >= cfg_.min_points_per_cell;
    const bool local_obs =
        enough && std::isfinite(ground) && (z - ground > cfg_.tau_local);

    // DEM-prior difference (slope-invariant).
    bool dem_obs = false;
    const float h = dem_at(x, y);
    if (std::isfinite(h)) {
      dem_obs = enough && (z - (h + dem_offset) > tau_prior_eff);
    }

    // When the DEM prior is loaded and TRN confidence is reasonable (>0.5),
    // the DEM-prior signal alone is sufficient: it correctly passes dune slopes
    // (z ≈ dem → diff ≈ 0) while catching real obstacles that protrude above
    // the known terrain surface. The local-jump signal fires false positives
    // on ANY steep slope (dune face, hill crest) because it cannot distinguish
    // gradual terrain from a vertical obstacle within its 3×3 cell window.
    //
    // When confidence is low or no DEM is available, fall back to the local-jump
    // signal (with a relaxed tau_local) as a safety net.
    if (has_dem() && conf > 0.5) {
      mask[i] = dem_obs ? 1 : 0;
    } else {
      mask[i] = (local_obs || dem_obs) ? 1 : 0;
    }
  }
  return mask;
}

}  // namespace ugv_obstacle
