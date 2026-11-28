#pragma once
// Thế giới giả lập cho test tích hợp (SLAM-DESIGN.md mục 9): trần nhà có hoa văn, xe cân bằng chúi tới lui,
// bánh có sai số tỉ lệ (odometry trôi), gyro có bias, có thể làm bánh "kẹt" (bánh báo quay mà xe đứng yên).
#include <array>
#include <functional>
#include <opencv2/core.hpp>
#include <random>
#include <utility>
#include <vector>

#include "slam/config.h"
#include "slam/frontend.h"
#include "slam/pose2.h"
#include "slam/protocol.h"
#include "slam/system.h"

#include <cstdio>

namespace sim {

// Hộp chắn (vật cản cho siêu âm), có thời gian xuất hiện / biến mất
struct SimBox {
  double x0, y0, x1, y1;
  double t_on = -1e18, t_off = 1e18;
};

struct SimParams {
  double ceiling_h = 2.30;       // m, trần so với camera
  double tex_res = 0.005;        // m / pixel kết cấu trần
  double world = 16.0;           // m, cạnh vùng trần (tâm ở gốc)
  int img_w = 640, img_h = 480;
  double scale_L = 1.015, scale_R = 0.99;   // bánh báo nhiều / ít hơn thực tế
  double gyro_bias = 0.008;      // rad/s
  double gyro_noise = 0.002;
  double pitch_amp_deg = 2.5, pitch_freq = 1.3;
  double balance_amp = 0.006, balance_freq = 1.7;   // m, xe đưa tới lui để giữ thăng bằng
  bool send_pitch = true;        // firmware 2.1 gửi pitch; false = firmware cũ
  unsigned seed = 1;             // nhiễu
  unsigned tex_seed = 1;         // hoa văn trần (giữ nguyên giữa các pha = cùng một căn phòng)
  std::vector<std::pair<double, double>> stuck;     // khoảng thời gian bánh kẹt (xe đứng yên, bánh báo chạy)
  // Siêu âm giả lập: đóng vai ESP32 (bảng vị trí gửi bằng bản ghi 'G', giống SONAR_SENSORS mặc định)
  bool sonar = true;
  std::vector<std::array<double, 4>> sonar_mount = {{0.08, 0.05, 40, 15}, {0.09, 0.0, 0, 15}, {0.08, -0.05, -40, 15}};
  double sonar_range = 2.0, sonar_period = 0.022, sonar_noise = 0.005;
  std::vector<SimBox> boxes;
};

class SimWorld {
public:
  SimWorld(const slam::Config &cfg, const SimParams &p, const slam::Pose2 &start = slam::Pose2(), double t0 = 100.0);

  // Tiến một bước với lệnh (v m/s, w rad/s); trễ bậc nhất như firmware làm mượt
  void step(double v_cmd, double w_cmd, double dt);
  slam::McuRecord mcu() const;
  cv::Mat render() const;

  double t() const { return t_; }
  slam::Pose2 truth() const { return truth_; }
  double pitch() const { return pitch_; }
  void teleport(const slam::Pose2 &p) { truth_ = p; }
  void addBox(const SimBox &b) { p_.boxes.push_back(b); }
  std::vector<SimBox> &boxes() { return p_.boxes; }
  const SimParams &params() const { return p_; }
  // Khoảng cách tới vật gần nhất trong chùm cảm biến k (m); < 0 = không có vang
  double sonarRange(int k) const;
  std::vector<uint8_t> sonarGeomBytes() const;   // khung SONAR_GEOM như firmware 2.2

private:
  slam::Config cfg_;
  SimParams p_;
  slam::CameraModel cam_;
  cv::Mat tex_;
  double t_;
  slam::Pose2 truth_;
  double v_ = 0, w_ = 0;
  double pitch_ = 0, pitch_rate_ = 0;
  double cumL_ = 0, cumR_ = 0;     // m bánh báo
  double w_true_ = 0;
  uint16_t seq_ = 0;
  mutable std::mt19937 rng_;
};

// Chạy SlamSystem (trong tiến trình, đồng bộ) với thế giới giả lập: MCU 200 Hz, ảnh 15 Hz.
// log != nullptr: ghi luồng vào file định dạng slamlog (để replay bằng slam_node).
class SimRunner {
public:
  SimRunner(slam::SlamSystem &sys, SimWorld &w, FILE *log = nullptr, double fps = 15, double rate = 200)
      : sys_(sys), w_(w), log_(log), fps_(fps), dt_(1.0 / rate), next_img_(w.t()) {}

  void tick(double v, double w);                 // một bước 5 ms
  void run(double v, double w, double secs);
  void forward(double dist, double v = 0.12);    // "người lái": đo bằng vị trí thật
  void rotate(double angle, double w = 0.5);
  void command(const std::string &c);
  bool navigate(double timeout, const std::function<void()> &each = nullptr);   // lái theo controller; true = tới nơi
  bool waitLocalized(double timeout, double w_rotate = 0.3);
  slam::Pose2 truthAt(double t) const;          // vị trí thật tại thời điểm t (để chấm điểm)
  double minDist(const Eigen::Vector2d &p, double t0, double t1) const;   // xe thật đi gần p nhất bao nhiêu

private:
  void sonarTick();
  std::vector<std::pair<double, slam::Pose2>> hist_;
  double next_sonar_ = -1e9;
  int sonar_k_ = 0;
  bool geom_sent_ = false;
  slam::SlamSystem &sys_;
  SimWorld &w_;
  FILE *log_;
  double fps_, dt_, next_img_;
};

}  // namespace sim
