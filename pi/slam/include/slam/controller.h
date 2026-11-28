#pragma once
// M10: pure pursuit trong frame odom (waypoint giữ trong map, đổi sang odom mỗi chu kỳ bằng T_map_odom⁻¹).
// Vật cản (siêu âm):
//   - giảm tốc theo khoảng trống phía trước: v ≤ v_max·(clear − stop_dist)/(slow_dist − stop_dist)
//   - bị chặn (đường phía trước đè lên ô vật cản, hoặc siêu âm giữ xe đứng > hold_block_s): trạng thái
//     "blocked": dừng, quét ±scan_deg tại chỗ, chờ block_wait_s; hết chặn thì đi tiếp, còn chặn thì đặt cờ
//     replanRequested() cho SlamSystem chạy lại A*; không có đường thì thử lại mỗi replan_retry_s, quá
//     max_block_s thì huỷ.
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
  double clear_front = 1e9;   // m khoảng trống phía trước từ siêu âm (1e9 = không biết / không có)
  bool path_blocked = false;  // đường phía trước đè lên ô vật cản đã xác nhận
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
  bool active() const {
    return state_ == "following" || state_ == "rotating" || state_ == "final_rotate" || state_ == "blocked";
  }
  ControlOutput tick(const ControlInput &in, double dt);

  const std::string &state() const { return state_; }   // idle following rotating final_rotate arrived aborted
  const std::string &reason() const { return reason_; }
  const std::vector<Eigen::Vector2d> &waypoints() const { return wps_; }
  size_t index() const { return idx_; }
  double remaining() const { return remaining_; }
  double headingError() const { return herr_; }
  std::optional<double> goalTheta() const { return goal_th_; }
  double elapsed(double t) const { return active() ? t - t0_ : 0; }
  const std::vector<size_t> &stops() const { return stops_; }

  // ----- vật cản
  bool replanRequested() const { return replan_req_; }
  void replanFailed(double retry_at) { replan_req_ = false, replan_due_ = retry_at; }
  double blockedFor(double t) const { return state_ == "blocked" ? t - blk_t0_ : 0; }
  int scanPhase() const { return scan_phase_; }
  double nextReplanIn(double t) const { return state_ == "blocked" ? std::max(0.0, replan_due_ - t) : 0; }
  const std::string &blockCause() const { return blk_cause_; }

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
  // vật cản
  void enterBlocked(const ControlInput &in, const std::string &cause);
  double blk_t0_ = 0, replan_due_ = 0, hold_t_ = -1, look_until_ = -1, scan_ref_ = 0;
  int scan_phase_ = 0;
  bool replan_req_ = false;
  std::string blk_cause_;
};

}  // namespace slam
