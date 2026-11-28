#pragma once
// Bản đồ vật cản log-odds từ siêu âm, theo cách cập nhật của EGO-Planner (plan_env/grid_map.cpp), rút gọn cho
// cảm biến chùm rộng:
//   - một số đo d được rải thành `rays` tia trong chùm ±half (HC-SR04 không biết tia nào trúng);
//   - ô trên tia có r < d − 1 ô: TRỐNG; ô tại r ≈ d: TRÚNG (trọng số w = 1 ở trục, edge_weight ở mép);
//     "không có vang": chỉ trống tới no_echo_free; d > hit_range: chỉ trống tới d (cung quá rộng, không ghi trúng);
//   - mỗi ô chỉ cập nhật MỘT LẦN mỗi số đo: trúng nếu số tia trúng ≥ số tia trống (count_hit / count_hit_and_miss);
//   - L += logit(p_hit)·w hoặc logit(p_miss), kẹp [logit(p_min), logit(p_max)], vật cản khi L > logit(p_occ);
//   - lớp TẠM: L giảm dần về 0 với hằng số thời gian tau (cảm biến chỉ nhìn phía trước, phải tự quên);
//     lớp TĨNH (tau = 0): không quên, dựng lại từ số đo gắn keyframe (StaticObstacles).
// Ô mới bắt đầu ở L = 0 (chưa biết). Cổng "chỉ cập nhật khi xe đã dời / xoay" nằm ở SlamSystem.
#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "slam/config.h"
#include "slam/keyframe.h"
#include "slam/pose2.h"
#include "slam/traversed_grid.h"

namespace slam {

struct LogOddsParams {
  double hit = 0, miss = 0, lmin = 0, lmax = 0, locc = 0, edge_weight = 0.5;
  int rays = 7;
  double hit_range = 1.2;
  double no_echo_free = 1.2;
  static LogOddsParams from(const ObstacleCfg &c);
};

class LogOddsGrid {
public:
  explicit LogOddsGrid(double res = 0.05, double tau = 0) : res_(res), tau_(tau) {}
  void clear() { cells_.clear(); }
  // sensor: tư thế cảm biến trong khung của lưới; d > 0 có vang, d <= 0 không có vang
  void integrate(const Pose2 &sensor, double half, double d, double max_range, double t, const LogOddsParams &p);
  double value(int i, int j, double t) const;
  std::vector<Eigen::Vector2d> occupied(double t, double locc) const;
  void prune(const Eigen::Vector2d &center, double half_size, double t);   // bỏ ô ngoài cửa sổ hoặc |L| ≈ 0
  void eraseDisc(const Eigen::Vector2d &c, double r, double lmin);
  size_t size() const { return cells_.size(); }
  double res() const { return res_; }

private:
  struct Cell {
    float L = 0;
    double t = 0;
  };
  static int64_t key(int i, int j) { return (int64_t(i) << 32) ^ int64_t(uint32_t(j)); }
  double decayed(const Cell &c, double t) const;
  std::unordered_map<int64_t, Cell> cells_;
  double res_, tau_;
};

// Một số đo siêu âm neo vào keyframe: tư thế cảm biến tương đối keyframe + khoảng cách
struct SonarObs {
  int32_t kf = -1;
  float rx = 0, ry = 0, rth = 0;   // T_kf_sensor (khung odom của keyframe lúc tạo)
  float half = 0;                  // rad
  float d = 0;                     // m, <= 0 = không có vang
  float max_range = 2;
};

// Lớp vật cản tĩnh (ghi khi tạo / mở rộng bản đồ): giữ số đo gốc, dựng lại sau mỗi lần tối ưu pose graph
class StaticObstacles {
public:
  explicit StaticObstacles(double res = 0.05) : grid_(res, 0) {}
  void clear();
  void add(const SonarObs &o, const Keyframe &kf, const LogOddsParams &p);   // cộng ngay vào lưới
  void rebuild(const std::vector<Keyframe> &kfs, const LogOddsParams &p);
  void erase(const Eigen::Vector2d &c, double r, const LogOddsParams &p);    // công cụ tẩy (lưu cùng bản đồ)
  std::vector<Eigen::Vector2d> occupied(double locc) const { return grid_.occupied(0, locc); }
  const std::vector<SonarObs> &obs() const { return obs_; }
  const std::vector<std::array<float, 3>> &erasures() const { return erase_; }
  bool save(const std::string &file) const;
  bool load(const std::string &file);
  void setData(std::vector<SonarObs> o, std::vector<std::array<float, 3>> e) { obs_ = std::move(o), erase_ = std::move(e); }

private:
  LogOddsGrid grid_;
  std::vector<SonarObs> obs_;
  std::vector<std::array<float, 3>> erase_;   // x, y, r (map)
};

// Mặt nạ cấm (CV_8U, 255 = cấm) cùng lưới với g: các điểm vật cản (map) phình đĩa bán kính inflate
cv::Mat obstacleMask(const TraversedGrid &g, const std::vector<Eigen::Vector2d> &pts, double inflate);

// Đường (polyline từ vị trí xe) có điểm vật cản nào cách < r không; bỏ điểm nằm sau xe trên đoạn đầu.
// end_is_goal: điểm cuối là đích cuối -> vật nằm SAU đích (vd địa điểm sát tường) không tính.
// Trả khoảng cách dọc đường tới chỗ bị chặn, < 0 nếu không chặn.
double pathBlockedAt(const std::vector<Eigen::Vector2d> &poly, const std::vector<Eigen::Vector2d> &pts, double r,
                     bool end_is_goal = false);

}  // namespace slam
