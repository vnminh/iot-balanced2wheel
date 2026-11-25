#pragma once
// M10: pure pursuit trong frame odom (waypoint giữ trong map, đổi sang odom mỗi chu kỳ bằng T_map_odom⁻¹).
#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

#include "slam/config.h"
#include "slam/pose2.h"

namespace slam {

struct ControlInput {
  double t = 0;
  Pose2 odom;              // odom -> base_link
  Pose2 T_map_odom;
  bool localized = false;
  bool balancing = false;
  double pitch = 0, pitch_rate = 0;
  double mcu_age = 0;      // s kể từ mẫu MCU gần nhất
};

struct ControlOutput {
  double v = 0, w = 0;     // m/s, rad/s (+ = ngược chiều kim đồng hồ)
  bool active = false;
};

class Controller {
public:
  explicit Controller(const ControllerCfg &cfg) : cfg_(cfg) {}

  // stops: chỉ số waypoint phải CHẠM (trong goal_tol_xy) trước khi đi tiếp (điểm giữa của nhiệm vụ)
  void start(const std::vector<Eigen::Vector2d> &waypoints_map, std::optional<double> goal_theta_map, double t,
             const std::vector<size_t> &stops = {});
  void cancel(const std::string &reason);
  void clear();
  bool active() const { return state_ == "following" || state_ == "rotating" || state_ == "final_rotate"; }
  ControlOutput tick(const ControlInput &in, double dt);

  const std::string &state() const { return state_; }   // idle following rotating final_rotate arrived aborted
  const std::string &reason() const { return reason_; }
  const std::vector<Eigen::Vector2d> &waypoints() const { return wps_; }
  size_t index() const { return idx_; }
  double remaining() const { return remaining_; }
  double headingError() const { return herr_; }
  std::optional<double> goalTheta() const { return goal_th_; }
  double elapsed(double t) const { return active() ? t - t0_ : 0; }

private:
  ControllerCfg cfg_;
  std::string state_ = "idle", reason_;
  std::vector<Eigen::Vector2d> wps_;
  std::optional<double> goal_th_;
  std::vector<size_t> stops_;   // tăng dần; phần tử đầu = điểm phải chạm kế tiếp
  size_t idx_ = 0;
  double v_ = 0, w_ = 0;
  double remaining_ = 0, herr_ = 0;
  double t0_ = 0, best_ = 1e18, best_t_ = 0;
};

}  // namespace slam
