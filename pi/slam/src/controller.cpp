#include "slam/controller.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace slam {

void Controller::start(const std::vector<Eigen::Vector2d> &wps, std::optional<double> goal_th, double t,
                       const std::vector<size_t> &stops) {
  wps_ = wps;
  stops_.clear();
  for (size_t k : stops)
    if (k + 1 < wps.size() && (stops_.empty() || k > stops_.back())) stops_.push_back(k);
  goal_th_ = goal_th;
  idx_ = 0;
  v_ = w_ = 0;
  state_ = wps_.empty() ? "arrived" : "following";
  reason_.clear();
  t0_ = t;
  best_ = 1e18;
  best_t_ = t;
}

void Controller::cancel(const std::string &reason) {
  if (active()) {
    state_ = "aborted";
    reason_ = reason;
  }
  v_ = w_ = 0;
}

void Controller::clear() {
  state_ = "idle";
  reason_.clear();
  wps_.clear();
  goal_th_.reset();
  v_ = w_ = 0;
}

ControlOutput Controller::tick(const ControlInput &in, double dt) {
  ControlOutput out;
  if (!active()) return out;
  const ControllerCfg &c = cfg_;

  // --- An toàn (bắt buộc) ---
  std::string why;
  char buf[96];
  if (!in.balancing)
    why = "Xe không còn cân bằng";
  else if (in.mcu_age > c.mcu_timeout)
    std::snprintf(buf, sizeof buf, "Mất dữ liệu từ ESP32 (%.2f s)", in.mcu_age), why = buf;
  else if (!in.localized)
    why = "Mất định vị trên bản đồ";
  else if (std::fabs(in.pitch) > deg2rad(c.max_pitch_deg))
    std::snprintf(buf, sizeof buf, "Góc nghiêng bất thường %.0f° (va chạm?)", rad2deg(in.pitch)), why = buf;
  else if (std::fabs(in.pitch_rate) > c.max_pitch_rate)
    std::snprintf(buf, sizeof buf, "Thân xe giật mạnh %.1f rad/s (va chạm?)", in.pitch_rate), why = buf;
  if (!why.empty()) {
    cancel(why);
    return out;
  }

  // --- Waypoint sang frame odom ---
  const Pose2 T_odom_map = in.T_map_odom.inverse();
  auto wp = [&](size_t k) { return T_odom_map * wps_[k]; };
  const Eigen::Vector2d p = in.odom.t();
  const double th = in.odom.th;

  // Tiến độ: chỉ tiến lần lượt (không nhảy cóc) khi đã tới gần waypoint hoặc đã vượt qua nó theo hướng đoạn
  // kế tiếp. Đường mission có thể quay đầu / đi lại đoạn cũ: waypoint "gần nhất" có thể nằm ở chặng sau.
  while (idx_ + 1 < wps_.size()) {
    const Eigen::Vector2d a = wp(idx_), b = wp(idx_ + 1);
    if (!stops_.empty() && idx_ == stops_.front()) {   // điểm nhiệm vụ: phải chạm mới đi tiếp
      if ((a - p).norm() >= c.goal_tol_xy) break;
      stops_.erase(stops_.begin());
      ++idx_;
    } else if ((a - p).norm() < 0.5 * c.lookahead || (p - a).dot(b - a) > 0) {
      ++idx_;
    } else {
      break;
    }
  }
  const Eigen::Vector2d goal = wp(wps_.size() - 1);
  const double goal_dist = (goal - p).norm();
  remaining_ = (wp(idx_) - p).norm();
  for (size_t k = idx_ + 1; k < wps_.size(); k++) remaining_ += (wps_[k] - wps_[k - 1]).norm();

  double v = 0, w = 0;
  const bool at_goal = goal_dist < c.goal_tol_xy && idx_ + 2 >= wps_.size();   // tới đích CUỐI (không phải đi ngang qua)
  if (state_ == "final_rotate" || at_goal) {
    if (!goal_th_) {
      state_ = "arrived";
      v_ = w_ = 0;
      return out;
    }
    state_ = "final_rotate";
    const double e = wrapAngle((T_odom_map.th + *goal_th_) - th);
    herr_ = e;
    if (std::fabs(e) < deg2rad(c.goal_tol_theta_deg)) {
      state_ = "arrived";
      v_ = w_ = 0;
      return out;
    }
    w = std::clamp(c.k_rot * e, -c.w_max, c.w_max);
    if (std::fabs(w) < c.w_min_rotate) w = std::copysign(c.w_min_rotate, e);
  } else {
    // Điểm lookahead: đi dọc đường từ waypoint hiện tại một quãng = lookahead (theo chiều dài đường, không theo
    // khoảng cách thẳng, để không cắt tắt qua chỗ quay đầu)
    Eigen::Vector2d la = goal;
    // không nhìn quá điểm nhiệm vụ chưa chạm (không cắt góc qua nó)
    const size_t lim = stops_.empty() ? wps_.size() - 1 : stops_.front();
    double acc = (wp(idx_) - p).norm();
    if (acc >= c.lookahead || idx_ >= lim) {
      la = wp(idx_);
    } else {
      la = wp(lim);
      for (size_t k = idx_ + 1; k <= lim; k++) {
        acc += (wps_[k] - wps_[k - 1]).norm();
        if (acc >= c.lookahead) {
          la = wp(k);
          break;
        }
      }
    }
    const Eigen::Vector2d d = la - p;
    const double e = wrapAngle(std::atan2(d.y(), d.x()) - th);
    herr_ = e;
    if (state_ == "following" && std::fabs(e) > deg2rad(c.rotate_in_place_deg)) state_ = "rotating";
    if (state_ == "rotating" && std::fabs(e) < deg2rad(c.rotate_exit_deg)) state_ = "following";
    if (state_ == "rotating") {
      w = std::clamp(c.k_rot * e, -c.w_max, c.w_max);
      if (std::fabs(w) < c.w_min_rotate) w = std::copysign(c.w_min_rotate, e);
    } else {
      const double Ld = std::max(d.norm(), 1e-3);
      v = std::clamp(c.k_v_goal * remaining_, c.v_min, c.v_max);   // chỉ chậm lại ở đích cuối
      const double kappa = 2.0 * std::sin(e) / Ld;
      w = kappa * v;
      if (std::fabs(w) > c.w_max) {
        w = std::copysign(c.w_max, w);
        v = std::fabs(w / kappa);
      }
    }
  }

  // Tiến triển: còn lại + lệch hướng quy ra mét. Lâu không giảm -> vướng / bánh kẹt
  const double prog = remaining_ + 0.15 * std::fabs(herr_);
  if (prog < best_ - c.progress_min) {
    best_ = prog;
    best_t_ = in.t;
  } else if (in.t - best_t_ > c.stuck_time) {
    std::snprintf(buf, sizeof buf, "Không tiến gần hơn trong %.0f s (vướng vật / bánh kẹt?)", c.stuck_time);
    cancel(buf);
    return out;
  }

  // Giới hạn gia tốc (xe cân bằng)
  const double dv = c.acc_v * dt, dw = c.acc_w * dt;
  v_ += std::clamp(v - v_, -dv, dv);
  w_ += std::clamp(w - w_, -dw, dw);
  out.v = v_;
  out.w = w_;
  out.active = true;
  return out;
}

}  // namespace slam
