#include "slam/obstacle_grid.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <opencv2/imgproc.hpp>

#include "slam/sonar.h"

namespace slam {

namespace {
double logit(double p) { return std::log(p / (1 - p)); }
}  // namespace

// ------------------------------------------------------------------------------------------- sonar.h
bool SonarGeometry::parse(const uint8_t *d, size_t n, SonarGeometry &out) {
  if (n < 4 || d[0] != 1) return false;
  const int cnt = d[1];
  if (n < size_t(4 + 9 * cnt) || cnt > 16) return false;
  uint16_t mr;
  std::memcpy(&mr, d + 2, 2);
  SonarGeometry g;
  g.max_range = mr / 1000.0;
  for (int k = 0; k < cnt; k++) {
    const uint8_t *p = d + 4 + 9 * k;
    int16_t x, y, yaw;
    uint16_t stop;
    std::memcpy(&x, p, 2);
    std::memcpy(&y, p + 2, 2);
    std::memcpy(&yaw, p + 4, 2);
    std::memcpy(&stop, p + 7, 2);
    SonarSensor s;
    s.mount = Pose2(x / 1000.0, y / 1000.0, deg2rad(yaw / 100.0));
    s.half = deg2rad(p[6]);
    s.stop_mm = stop;
    g.sensors.push_back(s);
  }
  out = g;
  return true;
}

double frontClearance(const SonarGeometry &g, const std::vector<double> &dists, double corridor) {
  if (g.sensors.empty()) return std::numeric_limits<double>::infinity();
  double front = -1e9;
  for (const auto &s : g.sensors) front = std::max(front, s.mount.x);
  double best = std::numeric_limits<double>::infinity();
  for (size_t k = 0; k < g.sensors.size() && k < dists.size(); k++) {
    const double d = dists[k];
    if (!(d > 0)) continue;
    const auto &s = g.sensors[k];
    for (int i = 0; i < 9; i++) {
      const double a = s.mount.th - s.half + 2 * s.half * i / 8;
      const double px = s.mount.x + d * std::cos(a), py = s.mount.y + d * std::sin(a);
      if (px > 0 && std::fabs(py) <= corridor) best = std::min(best, px - front);
    }
  }
  return std::isfinite(best) ? std::max(0.0, best) : best;
}

// ------------------------------------------------------------------------------------------- log-odds
LogOddsParams LogOddsParams::from(const ObstacleCfg &c) {
  LogOddsParams p;
  p.hit = logit(c.p_hit);
  p.miss = logit(c.p_miss);
  p.lmin = logit(c.p_min);
  p.lmax = logit(c.p_max);
  p.locc = logit(c.p_occ);
  p.edge_weight = c.edge_weight;
  p.rays = std::max(1, c.rays);
  p.no_echo_free = c.no_echo_free;
  p.hit_range = c.hit_range;
  return p;
}

double LogOddsGrid::decayed(const Cell &c, double t) const {
  if (tau_ <= 0 || t <= c.t) return c.L;
  return c.L * std::exp(-(t - c.t) / tau_);
}

double LogOddsGrid::value(int i, int j, double t) const {
  auto it = cells_.find(key(i, j));
  return it == cells_.end() ? 0.0 : decayed(it->second, t);
}

void LogOddsGrid::integrate(const Pose2 &s, double half, double d, double max_range, double t,
                            const LogOddsParams &p) {
  const bool echo = d > 0 && d <= max_range;
  const bool hit_ok = echo && d <= p.hit_range;   // xa hơn: chỉ biết "trống tới d"
  const double reach = echo ? d : std::min(p.no_echo_free, max_range);
  struct Acc {
    int hits = 0, total = 0;
    double w = 0;
  };
  std::unordered_map<int64_t, Acc> acc;   // mỗi ô một lần mỗi số đo (cache như EGO)
  const int n = p.rays;
  const double step = res_ * 0.25;
  for (int r = 0; r < n; r++) {
    const double off = n == 1 ? 0.0 : -half + 2 * half * r / (n - 1);
    const double a = s.th + off, ca = std::cos(a), sa = std::sin(a);
    const double w = half > 0 ? 1.0 - (1.0 - p.edge_weight) * std::fabs(off) / half : 1.0;
    int64_t last = std::numeric_limits<int64_t>::min();
    for (double q = 0.5 * res_; q <= reach + 0.6 * res_; q += step) {
      const int i = int(std::floor((s.x + ca * q) / res_)), j = int(std::floor((s.y + sa * q) / res_));
      const int64_t k = key(i, j);
      if (k == last) continue;
      last = k;
      Acc &c = acc[k];
      if (hit_ok && std::fabs(q - d) <= 0.6 * res_) {
        ++c.hits, ++c.total;
        c.w = std::max(c.w, w);
      } else if (q < reach - res_) {
        ++c.total;
      }
    }
  }
  for (const auto &kv : acc) {
    const Acc &c = kv.second;
    if (c.total == 0) continue;
    const double du = c.hits >= c.total - c.hits ? p.hit * c.w : p.miss;
    Cell &cell = cells_[kv.first];
    const double L = std::clamp(decayed(cell, t) + du, p.lmin, p.lmax);
    cell.L = float(L);
    cell.t = t;
  }
}

std::vector<Eigen::Vector2d> LogOddsGrid::occupied(double t, double locc) const {
  std::vector<Eigen::Vector2d> out;
  for (const auto &kv : cells_)
    if (decayed(kv.second, t) > locc) {
      const int i = int(kv.first >> 32), j = int(int32_t(uint32_t(kv.first & 0xffffffff)));
      out.emplace_back((i + 0.5) * res_, (j + 0.5) * res_);
    }
  // thứ tự cố định (unordered_map không có thứ tự): kết quả lập đường + bản tin web xác định khi replay
  std::sort(out.begin(), out.end(), [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  return out;
}

void LogOddsGrid::prune(const Eigen::Vector2d &c, double half_size, double t) {
  for (auto it = cells_.begin(); it != cells_.end();) {
    const int i = int(it->first >> 32), j = int(int32_t(uint32_t(it->first & 0xffffffff)));
    const double x = (i + 0.5) * res_, y = (j + 0.5) * res_;
    if (std::fabs(x - c.x()) > half_size || std::fabs(y - c.y()) > half_size || std::fabs(decayed(it->second, t)) < 0.05)
      it = cells_.erase(it);
    else
      ++it;
  }
}

void LogOddsGrid::eraseDisc(const Eigen::Vector2d &c, double r, double lmin) {
  const int R = int(std::ceil(r / res_));
  const int ci = int(std::floor(c.x() / res_)), cj = int(std::floor(c.y() / res_));
  for (int dj = -R; dj <= R; dj++)
    for (int di = -R; di <= R; di++) {
      const Eigen::Vector2d p((ci + di + 0.5) * res_, (cj + dj + 0.5) * res_);
      if ((p - c).norm() > r) continue;
      auto it = cells_.find(key(ci + di, cj + dj));
      if (it != cells_.end()) it->second.L = float(lmin);
    }
}

// ------------------------------------------------------------------------------------------- lớp tĩnh
void StaticObstacles::clear() {
  grid_.clear();
  obs_.clear();
  erase_.clear();
}

void StaticObstacles::add(const SonarObs &o, const Keyframe &kf, const LogOddsParams &p) {
  obs_.push_back(o);
  const Pose2 s = kf.T_map * Pose2(o.rx, o.ry, o.rth);
  grid_.integrate(s, o.half, o.d, o.max_range, 0, p);
  for (const auto &e : erase_) grid_.eraseDisc({e[0], e[1]}, e[2], p.lmin);
}

void StaticObstacles::rebuild(const std::vector<Keyframe> &kfs, const LogOddsParams &p) {
  grid_.clear();
  for (const auto &o : obs_) {
    if (o.kf < 0 || o.kf >= int(kfs.size())) continue;
    grid_.integrate(kfs[o.kf].T_map * Pose2(o.rx, o.ry, o.rth), o.half, o.d, o.max_range, 0, p);
  }
  for (const auto &e : erase_) grid_.eraseDisc({e[0], e[1]}, e[2], p.lmin);
}

void StaticObstacles::erase(const Eigen::Vector2d &c, double r, const LogOddsParams &p) {
  erase_.push_back({float(c.x()), float(c.y()), float(r)});
  grid_.eraseDisc(c, r, p.lmin);
}

// sonar.bin: "SONAR1\0\0", u32 n_obs, n × SonarObs, u32 n_erase, n × 3 float
bool StaticObstacles::save(const std::string &file) const {
  std::ofstream f(file, std::ios::binary);
  if (!f) return false;
  f.write("SONAR1\0\0", 8);
  const uint32_t n = uint32_t(obs_.size()), m = uint32_t(erase_.size());
  f.write(reinterpret_cast<const char *>(&n), 4);
  if (n) f.write(reinterpret_cast<const char *>(obs_.data()), std::streamsize(n * sizeof(SonarObs)));
  f.write(reinterpret_cast<const char *>(&m), 4);
  if (m) f.write(reinterpret_cast<const char *>(erase_.data()), std::streamsize(m * sizeof(erase_[0])));
  return bool(f);
}

bool StaticObstacles::load(const std::string &file) {
  std::ifstream f(file, std::ios::binary);
  if (!f) return false;
  char magic[8];
  f.read(magic, 8);
  if (!f || std::memcmp(magic, "SONAR1", 6) != 0) return false;
  uint32_t n = 0, m = 0;
  f.read(reinterpret_cast<char *>(&n), 4);
  if (n > 10000000) return false;
  std::vector<SonarObs> o(n);
  if (n) f.read(reinterpret_cast<char *>(o.data()), std::streamsize(n * sizeof(SonarObs)));
  f.read(reinterpret_cast<char *>(&m), 4);
  if (m > 100000) return false;
  std::vector<std::array<float, 3>> e(m);
  if (m) f.read(reinterpret_cast<char *>(e.data()), std::streamsize(m * sizeof(e[0])));
  if (!f) return false;
  obs_ = std::move(o);
  erase_ = std::move(e);
  return true;
}

// ------------------------------------------------------------------------------------------- mặt nạ / chặn
cv::Mat obstacleMask(const TraversedGrid &g, const std::vector<Eigen::Vector2d> &pts, double inflate) {
  cv::Mat m = cv::Mat::zeros(std::max(g.ny, 1), std::max(g.nx, 1), CV_8U);
  if (g.empty()) return m;
  for (const auto &p : pts) {
    int i, j;
    if (g.toCell(p, i, j)) m.at<uchar>(j, i) = 255;
  }
  const int R = int(std::ceil(inflate / g.res));
  if (R > 0 && !pts.empty()) {
    const cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * R + 1, 2 * R + 1));
    cv::dilate(m, m, k);
  }
  return m;
}

double pathBlockedAt(const std::vector<Eigen::Vector2d> &poly, const std::vector<Eigen::Vector2d> &pts, double r,
                     bool end_is_goal) {
  if (poly.size() < 2 || pts.empty()) return -1;
  double along = 0, best = -1;
  for (size_t s = 1; s < poly.size(); s++) {
    const Eigen::Vector2d a = poly[s - 1], b = poly[s], ab = b - a;
    const double L2 = ab.squaredNorm(), L = std::sqrt(L2);
    for (const auto &p : pts) {
      double u = L2 > 1e-12 ? (p - a).dot(ab) / L2 : 0;
      if (s == 1 && u <= 0) continue;   // sau lưng xe
      if (end_is_goal && s + 1 == poly.size() && u >= 1) continue;   // sau đích cuối
      u = std::clamp(u, 0.0, 1.0);
      if ((a + u * ab - p).norm() < r) {
        const double at = along + u * L;
        if (best < 0 || at < best) best = at;
      }
    }
    if (best >= 0) return best;
    along += L;
  }
  return best;
}

}  // namespace slam
