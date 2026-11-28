#pragma once
// Điều phối toàn bộ SLAM 2D (SLAM-DESIGN.md mục 4).
//
// Luồng chính (gọi onMcu / onImage / command): EKF, chọn khung cần xử lý, controller, phát trạng thái.
//   Không bao giờ chờ tối ưu đồ thị (nguyên tắc 5).
// Luồng worker: ORB, khớp ảnh, keyframe, loop closure, pose graph, grid, lưu/tải bản đồ.
//   Worker là nơi DUY NHẤT sửa dữ liệu bản đồ (sửa dưới khoá m_); luồng chính chỉ đọc dưới khoá.
// Chế độ synchronous (replay / test): việc của worker chạy ngay trong luồng gọi -> kết quả xác định.
//
// Thời gian = timestamp dữ liệu (đồng hồ monotonic của Pi), không dùng đồng hồ hệ thống, nên replay
// cho cùng kết quả. Riêng wallTick() dùng đồng hồ thật để phát hiện mất dữ liệu ESP32 khi chạy thật.
#include <Eigen/Core>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "slam/config.h"
#include "slam/controller.h"
#include "slam/ekf2d.h"
#include "slam/frontend.h"
#include "slam/keyframe.h"
#include "slam/map_io.h"
#include "slam/matcher.h"
#include "slam/obstacle_grid.h"
#include "slam/sonar.h"
#include "slam/pose_graph.h"
#include "slam/protocol.h"
#include "slam/traversed_grid.h"
#include "slam/wheel_odometry.h"

namespace slam {

enum class Mode { Mapping, Localization };

class SlamSystem {
public:
  struct Options {
    std::string maps_dir = "maps";
    std::string shm_dir;        // ghi grid.png / frame.jpg cho web (trống = không ghi)
    bool synchronous = false;
  };
  using Emit = std::function<void(const std::string &)>;

  SlamSystem(const Config &cfg, const Options &opt, Emit emit);
  ~SlamSystem();
  SlamSystem(const SlamSystem &) = delete;
  SlamSystem &operator=(const SlamSystem &) = delete;

  void onMcu(const McuRecord &m);
  void onImage(double t, const cv::Mat &gray);
  void onSonar(const SonarRecord &r);                 // bản ghi 'S'
  void onSonarGeom(const uint8_t *d, size_t n);       // bản ghi 'G' (bảng vị trí cảm biến do ESP32 gửi)
  // Lệnh text (một dòng):
  //   goto <x> <y> [theta_deg]   goto_place <tên>   mission <x1> <y1> <x2> <y2> ...   cancel [lý do]   nav_clear
  //   new_map   load_map <id> [localization|mapping]   save_map <id> <tên hiển thị>
  //   set_mode localization|mapping   save_place <tên>   delete_place <tên>
  //   reloc   calib_ref   calib_measure   status
  //   obst_erase <x> <y> <r>   obst_clear_temp
  void command(const std::string &line);
  void wallTick(double now);   // chạy thật: kiểm tra mất dữ liệu MCU
  void flush();                // chờ worker làm hết việc đang xếp hàng

  // Truy cập trong tiến trình (simulator / test)
  ControlOutput lastCmd() const { return cmd_; }
  bool navActive() const { return ctrl_.active(); }
  std::string navState() const { return ctrl_.state(); }
  std::string navReason() const { return ctrl_.reason(); }
  int missionLeg() const { return leg_offset_ + currentLeg(); }
  int replans() const { return replans_; }
  double clearFront() const;
  std::vector<Eigen::Vector2d> obstaclesStatic() const;
  std::vector<Eigen::Vector2d> obstaclesTemp() const;   // frame map
  size_t staticObsCount() const;
  const SonarGeometry &sonarGeometry() const { return sonar_geom_; }
  Pose2 odomPose() const { return ekf_.pose(); }
  Pose2 mapPose() const;
  bool localized() const;
  Mode mode() const;
  std::vector<Keyframe> keyframes() const;
  int edgeCount(EdgeKind k) const;
  int slipCount() const;
  double heightUsed() const;
  std::vector<Place> places() const;
  double ekfBias() const { return ekf_.bias(); }
  std::vector<VisualEdge> visualEdges() const {
    std::lock_guard<std::mutex> lk(m_);
    return vis_edges_;
  }
  std::vector<OdomEdge> odomEdges() const {
    std::lock_guard<std::mutex> lk(m_);
    return odom_edges_;
  }
  const CameraModel &camera() const { return cam_; }

private:
  struct Hist {
    double t;
    Pose2 odom;
    double pitch, pitch_rate, yaw_rate;
  };
  struct ZSample {
    double t;
    int32_t sl, sr;
    double gz, prate;
  };
  // Trạng thái odometry tại một thời điểm: dùng tính độ trôi của T_map_odom kể từ lần sửa trước
  struct OdomSnap {
    Pose2 odom;
    Eigen::Matrix3d P = Eigen::Matrix3d::Zero();   // covariance tư thế của EKF
    double dist = 0;
  };
  struct Job {
    enum Type {
      KEYFRAME, LOCALIZE, CALIB_REF, CALIB_MEASURE, NEW_MAP, LOAD_MAP, SAVE_MAP, SET_MODE, SAVE_PLACE,
      DELETE_PLACE, RELOC
    } type = KEYFRAME;
    double t = 0;
    cv::Mat gray;
    Pose2 odom;
    double pitch = 0;
    Eigen::Matrix3d relcov = Eigen::Matrix3d::Zero();
    double dist = 0;
    Eigen::Matrix3d podom = Eigen::Matrix3d::Zero();
    std::string a, b;
    OdomSnap snap() const { return {odom, podom, dist}; }
    bool image_job = false;
  };

  // ----- luồng chính
  void handlePendingImage();
  Hist interp(double t) const;
  void maybeBlindKeyframe();
  void controlTick(double dt);
  void startRoute(const std::vector<Eigen::Vector2d> &goals, std::optional<double> theta, const std::string &label,
                  const char *cmd);
  struct Route {
    std::vector<Eigen::Vector2d> wps, snapped;
    std::vector<size_t> ends;
    double length = 0, max_snap = 0;
  };
  // A* qua lần lượt các điểm trên vùng đã đi TRỪ ô vật cản (đã phình); bad_leg = chặng lỗi
  bool planRoute(const TraversedGrid &g0, const Pose2 &start, const std::vector<Eigen::Vector2d> &goals,
                 double start_snap, Route &r, std::string &why, int &bad_leg);
  void replanFromHere();
  const std::vector<Eigen::Vector2d> &obstaclePointsMap();   // tĩnh + tạm (frame map), cache 0,2 s
  bool pathBlockedNow(const Pose2 &robot_map);
  void emitObstacles();
  int currentLeg() const;
  void emitState();
  void emitCmd(const ControlOutput &c);
  void emitPath();
  void submit(Job &&j);
  Job baseJob(Job::Type type) const;

  // ----- worker
  void workerLoop();
  void runJob(Job &j);
  void doKeyframe(Job &j);
  void doLocalize(Job &j);
  void doCalib(Job &j);
  void doNewMap(Job &j);
  void doLoadMap(Job &j);
  void doSaveMap(Job &j);
  void doSetMode(Job &j);
  void doPlace(Job &j);
  void optimizeGraph(const std::vector<size_t> &new_vis, const OdomSnap &snap);
  void rebuildGrid();
  void publishGraph();
  void publishGrid(bool write_file = true);
  void publishPlaces();
  void debugImage(const Job &j, const FrameFeatures &ff);
  bool updateMapOdom(const Pose2 &z, const Pose2 &meas_base, int inliers, const OdomSnap &snap);
  int lastKfOfSession() const;
  Eigen::Matrix3d visualCov(int inliers) const;
  GraphEdge toGraphEdge(const VisualEdge &e) const;
  std::string qualityJson() const;        // gọi khi đã giữ m_
  double heightUsedLocked() const;
  Eigen::Matrix3d processNoiseLocked(const OdomSnap &now) const;  // độ trôi T_map_odom từ lần sửa trước
  Eigen::Vector2d mapSigmaLocked(const OdomSnap &now) const;      // (σ_xy m, σ_θ rad)
  OdomSnap curSnap() const { return {ekf_.pose(), ekf_.P().topLeftCorner<3, 3>(), ekf_.distance()}; }
  void persistPlacesLocked();

  // ----- phát JSON (an toàn đa luồng)
  void emit(const std::string &s);
  void event(const std::string &level, const std::string &msg);
  void reply(const std::string &cmd, bool ok, const std::string &msg, const std::string &extra = "");

  Config cfg_;
  Options opt_;
  Emit emit_raw_;
  std::mutex emit_m_;
  CameraModel cam_;
  CeilingFrontend fe_;

  // ----- trạng thái luồng chính
  WheelOdometry wo_;
  PitchEstimator pe_;
  Ekf2d ekf_;
  McuRecord last_mcu_{};
  bool have_mcu_ = false;
  double t_ = 0;
  double pitch_ = 0, pitch_rate_ = 0, yaw_rate_ = 0;
  std::deque<Hist> hist_;
  std::deque<ZSample> zwin_;
  double last_zupt_t_ = -1e9;
  int zupt_count_ = 0;
  double wo_dth_ = 0, wo_gyro_ = 0, wo_dt_ = 0;
  int wo_ok_ = 0, wo_rej_ = 0;
  bool has_path_sample_ = false;
  Pose2 last_path_sample_;
  bool pend_ = false;
  double pend_t_ = 0;
  cv::Mat pend_img_, last_img_;
  double last_img_t_ = -1e9;
  double last_reloc_job_t_ = -1e9, last_loc_job_t_ = -1e9, last_loc_job_dist_ = 0;
  Pose2 last_loc_job_odom_;
  double next_state_t_ = 0, next_ctrl_t_ = 0;
  int frames_rx_ = 0, frames_skipped_motion_ = 0, frames_dropped_busy_ = 0;
  Controller ctrl_;
  ControlOutput cmd_;
  std::string nav_label_;
  std::vector<Eigen::Vector2d> mission_pts_;   // điểm nhiệm vụ (đã dời vào vùng đã đi), trống = goto 1 điểm
  std::vector<size_t> leg_ends_;               // chỉ số waypoint cuối của từng chặng
  int last_leg_ = 0;
  std::vector<Eigen::Vector2d> route_goals_;   // điểm đích gốc của goto / nhiệm vụ (để lập lại đường)
  std::optional<double> route_theta_;
  int leg_offset_ = 0;                          // số chặng đã xong trước lần lập lại đường gần nhất
  int replans_ = 0;
  std::string last_replan_why_;
  double last_replan_warn_t_ = -1e9;

  // ----- siêu âm / vật cản (luồng chính)
  SonarGeometry sonar_geom_;
  struct SonarLast {
    double t = -1e9;
    uint16_t mm = SONAR_FAULT;
    bool has_int = false;
    Pose2 last_int;            // tư thế cảm biến (odom) lần đưa vào lưới gần nhất
  };
  std::vector<SonarLast> sonar_last_;
  LogOddsParams lop_;
  LogOddsGrid temp_;            // lớp tạm, frame odom
  std::atomic<bool> temp_reset_{false};
  double last_prune_t_ = -1e9, last_obst_emit_t_ = -1e9, obst_cache_t_ = -1e9;
  std::vector<Eigen::Vector2d> obst_cache_;
  bool obst_emitted_nonempty_ = false;
  std::atomic<bool> rel_reset_{false};

  // ----- dữ liệu bản đồ (worker ghi dưới m_, luồng chính đọc dưới m_)
  mutable std::mutex m_;
  Mode mode_ = Mode::Mapping;
  std::string map_id_, map_name_ = "(chưa lưu)";
  int session_ = 0;
  std::vector<Keyframe> kfs_;
  std::vector<OdomEdge> odom_edges_;
  std::vector<bool> odom_slip_;          // song song odom_edges_: cạnh bị nghi trượt/kẹt bánh
  std::vector<VisualEdge> vis_edges_;
  std::vector<Place> places_;
  Pose2 T_map_odom_;
  Eigen::Matrix3d P_mo_ = Eigen::Matrix3d::Identity() * 1e-6;
  OdomSnap mo_snap_;                     // odometry lúc T_map_odom được sửa gần nhất
  bool localized_ = true;
  bool has_last_kf_ = false;
  Pose2 last_kf_odom_;
  int kf_reject_streak_ = 0;
  std::vector<std::pair<double, Pose2>> path_pending_;
  TraversedGrid grid_;
  int grid_version_ = 0, graph_version_ = 0;
  StaticObstacles static_;      // lớp tĩnh, frame map (neo keyframe)
  int last_kf_id_ = -1;
  std::vector<double> h_samples_;
  double loaded_height_ = 0;
  bool hyp_valid_ = false;
  Pose2 hyp_;
  int hyp_count_ = 0, hyp_kf_ = -1;
  double hyp_t_ = 0;
  int frames_processed_ = 0, kf_rejected_ = 0, last_feats_ = 0, slips_ = 0, rejected_loops_ = 0;
  MatchResult last_match_;
  std::string last_match_kind_;
  double worker_ms_ = 0, opt_ms_ = 0;
  bool calib_has_ = false;
  FrameFeatures calib_ff_;
  Pose2 calib_odom_;
  double last_debug_t_ = -1e9;

  // ----- hàng đợi việc
  std::deque<Job> jobs_;
  std::mutex jm_;
  std::condition_variable jcv_, idle_cv_;
  bool stop_ = false;
  int running_ = 0;
  std::atomic<bool> img_inflight_{false};
  std::thread worker_;
};

}  // namespace slam
