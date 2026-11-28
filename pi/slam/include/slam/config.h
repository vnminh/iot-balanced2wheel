#pragma once
// Cấu hình đọc từ pi/config/slam.yaml (xem SLAM-DESIGN.md mục 6).
// Mọi giá trị TBD có mặc định hợp lý; dùng mặc định thì có cảnh báo trong `warnings`.
#include <string>
#include <vector>

namespace slam {

struct RobotCfg {
  double wheel_diameter = 0.065;
  double wheel_base = 0.17;
  int steps_per_rev = 200;
  int microsteps = 16;
  double body_radius = 0.10;
  double metersPerStep() const;
};

struct EkfCfg {
  double sigma_d_per_m = 0.02;     // độ lệch chuẩn quãng đường, căn bậc hai theo mét (σ² = k²·|d|)
  double sigma_lat_per_m = 0.01;   // trượt ngang
  double sigma_gyro = 0.01;        // rad/s/√Hz
  double sigma_bias_rw = 0.0001;   // rad/s/√s (độ trôi bias MPU-6050 rất chậm)
  double init_sigma_bias = 0.02;   // rad/s
  double sat_noise_mult = 5.0;     // firmware báo chạm giới hạn -> nhiễu quãng đường ×
  double zupt_window = 1.0;        // s
  double zupt_disp = 0.005;        // m, dịch chuyển ròng mỗi bánh
  double zupt_gyro = 0.03;         // rad/s (firmware đã trừ offset gyro lúc khởi động -> bias còn lại nhỏ)
  double zupt_pitch_rate = 0.5;    // rad/s
  double zupt_sigma = 0.003;       // rad/s
  bool use_wheel_omega = false;   // cần wheel_base đo chính xác + hai bánh cùng tỉ lệ, không thì làm lệch bias
  double wheel_omega_window = 0.1; // s
  double wheel_omega_sigma = 0.15; // rad/s
  double pitch_filter_tau = 0.5;   // s, bộ lọc bù khi firmware cũ không gửi pitch
};

struct CameraCfg {
  int width = 640, height = 480;   // độ phân giải lúc hiệu chỉnh
  double fx = 635, fy = 635, cx = 320, cy = 240;
  std::vector<double> dist{0, 0, 0, 0, 0};  // k1 k2 p1 p2 k3
  double yaw_deg = 90;             // φ: trục x ảnh so với trục x xe (90 = mép trên ảnh hướng mũi xe)
  double mount_roll_deg = 0, mount_pitch_deg = 0;
  double offset_x = 0, offset_y = 0;   // vị trí camera so với tâm trục bánh (m, khung xe)
  double height_above_axle = 0.15;     // m, để bù dịch chuyển camera khi thân chúi
  double ceiling_height = 2.30;        // m từ camera tới trần; 0 = tự ước lượng
  double max_pitch_deg = 8;
  double max_pitch_rate = 1.5;     // rad/s
  double max_yaw_rate = 1.5;       // rad/s
};

struct FrontendCfg {
  int orb_features = 500;
  int orb_fast_threshold = 12;
  int min_features = 100;
  double kf_min_dist = 0.30;
  double kf_min_angle_deg = 15;
  double blind_kf_dist = 0.60;
  double path_sample = 0.02;
  double debug_jpg_period = 0.5;   // s, ghi ảnh đặc trưng cho web (0 = tắt)
};

struct LoopCfg {
  double search_radius = 1.0;
  int min_kf_gap = 20;
  int neighbor_edges = 3;
  int max_candidates = 3;
  double ratio_test = 0.7;
  double ransac_px = 3.0;
  int min_inliers = 30;
  double scale_min = 0.95, scale_max = 1.05;
  double max_pred_dist = 0.5;
  double max_pred_angle_deg = 20;
  double neighbor_max_dist = 1.0;      // m: cạnh lân cận lệch odometry hơn thế = bỏ (rộng: odometry có thể sai nhiều khi kẹt bánh)
  double neighbor_max_angle_deg = 30;
  double sigma_xy = 0.03;
  double sigma_th_deg = 1.5;
  double huber_k = 1.345;
  double reloc_period = 0.5;
  int reloc_confirm = 2;
  double reloc_agree_dist = 0.2;
  double reloc_agree_angle_deg = 10;
  double reloc_timeout = 4.0;      // s, giả thuyết cũ hơn thì bỏ
  double loc_period = 0.5;
  double loc_min_move = 0.10;
};

struct LocalizationCfg {
  double max_map_sigma = 0.30;     // m
  double drift_per_m = 0.05;       // m σ/√m quãng đường
  double drift_th_per_m_deg = 2.0;
  double chi2_gate = 11.34;        // 3 bậc tự do, 99%
};

struct GridCfg {
  double resolution = 0.05;
  double center_tolerance = 0.05;
  double margin = 0.5;
};

struct PlannerCfg {
  double edge_weight = 2.0;
  double waypoint_spacing = 0.15;
  double max_snap = 1.0;
};

struct ControllerCfg {
  double rate_hz = 20;
  double v_max = 0.15, v_min = 0.03;
  double w_max = 1.0, w_min_rotate = 0.25;
  double acc_v = 0.3, acc_w = 2.0;
  double lookahead = 0.30;
  double goal_tol_xy = 0.10;
  double goal_tol_theta_deg = 10;
  double rotate_in_place_deg = 50;
  double rotate_exit_deg = 15;
  double k_rot = 1.5;
  double k_v_goal = 0.8;
  double stuck_time = 8.0;
  double progress_min = 0.05;
  double mcu_timeout = 0.2;
  double max_pitch_deg = 20;
  double max_pitch_rate = 3.0;
  // Vật cản (siêu âm): giảm tốc theo khoảng trống phía trước, bị chặn -> quét, chờ, chạy lại A*
  double stop_dist = 0.15;         // m khoảng trống: v = 0 khi tiến
  double slow_dist = 0.50;         // m: bắt đầu giảm tốc
  double hold_block_s = 1.0;       // siêu âm giữ xe đứng lâu hơn -> coi như bị chặn
  double scan_deg = 25;            // quét ±góc tại chỗ khi bị chặn (log-odds thấy rõ vật lệch bên nào)
  double block_wait_s = 3.0;       // chờ vật tạm tự đi trước khi tìm đường khác
  double replan_retry_s = 2.0;
  double max_block_s = 30.0;       // không có đường lâu hơn -> huỷ
  double look_pause_s = 0.3;       // dừng sau khi xoay tại chỗ để siêu âm đo hướng mới
};

// Xử lý siêu âm (vị trí cảm biến DO ESP32 GỬI, không cấu hình ở đây)
struct SonarCfg {
  bool enable = true;
  double max_pitch_deg = 8;        // thân chúi hơn -> chùm có thể quét sàn: bỏ số đo
  double gate_move = 0.03;         // m: chỉ cập nhật lưới khi cảm biến đã dời...
  double gate_turn_deg = 3;        // ... hoặc xoay từ lần cập nhật trước (số đo đứng yên không độc lập)
  double fresh_s = 0.5;            // số đo cũ hơn không dùng cho khoảng trống phía trước
  double corridor = 0.15;          // m nửa bề rộng hành lang thân xe (thân 0,10 + dự phòng)
};

// Bản đồ vật cản log-odds (kiểu EGO-Planner)
struct ObstacleCfg {
  double resolution = 0.05;
  double p_hit = 0.65, p_miss = 0.35, p_min = 0.12, p_max = 0.90, p_occ = 0.80;
  double edge_weight = 0.5;        // trọng số "trúng" ở mép chùm (1 ở trục)
  int rays = 7;
  double hit_range = 1.2;          // m: chỉ ghi "trúng" khi d ≤ đây (xa hơn cung quá rộng, chỉ dùng để xoá trống)
  double no_echo_free = 1.2;       // m: "không có vang" chỉ xoá trống tới đây
  double temp_tau = 25.0;          // s: lớp tạm quên dần
  double temp_window = 3.0;        // m nửa cạnh cửa sổ lớp tạm quanh xe
  bool static_layer = true;        // ghi lớp tĩnh khi tạo / mở rộng bản đồ
  double inflate = 0.15;           // m phình ô vật cản (thân 0,10 + dự phòng)
  double check_ahead = 1.0;        // m đường phía trước để kiểm tra bị chặn
  double replan_start_snap = 0.30; // m: điểm đầu khi lập lại đường được dời tối đa
};

struct QualityCfg {
  int min_keyframes = 10;
  double min_visual_ratio = 0.6;
  double min_neighbor_ratio = 0.5;
  int min_loops = 1;
};

struct Config {
  RobotCfg robot;
  EkfCfg ekf;
  CameraCfg camera;
  FrontendCfg frontend;
  LoopCfg loop;
  LocalizationCfg loc;
  GridCfg grid;
  PlannerCfg planner;
  ControllerCfg controller;
  SonarCfg sonar;
  ObstacleCfg obstacles;
  QualityCfg quality;
  std::vector<std::string> warnings;  // giá trị TBD đang dùng mặc định
  std::string source_text;            // nội dung file (lưu vào config_snapshot.yaml)

  static Config load(const std::string &path);   // ném std::runtime_error nếu lỗi cú pháp
  static Config defaults();
};

}  // namespace slam
