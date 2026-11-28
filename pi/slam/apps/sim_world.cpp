#include "sim_world.h"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <opencv2/imgproc.hpp>

using slam::Pose2;

namespace sim {

SimWorld::SimWorld(const slam::Config &cfg, const SimParams &p, const Pose2 &start, double t0)
    : cfg_(cfg), p_(p), cam_(cfg.camera), t_(t0), truth_(start), rng_(p.seed) {
  // Trần: nền xám + đèn, ô trần, vết bẩn, đường dây... đủ đặc trưng ORB nhưng không lặp tuần hoàn
  const int N = int(p.world / p.tex_res);
  tex_ = cv::Mat(N, N, CV_8U, cv::Scalar(165));
  std::mt19937 g(p.tex_seed * 7919u + 13u);
  std::uniform_real_distribution<double> U(0, 1);
  cv::Mat noise(N / 8, N / 8, CV_8U);
  cv::RNG crng(p.tex_seed * 104729u + 1u);   // RNG riêng: cùng seed -> cùng trần (không dùng RNG toàn cục)
  crng.fill(noise, cv::RNG::UNIFORM, 0, 40);
  cv::resize(noise, noise, tex_.size(), 0, 0, cv::INTER_CUBIC);
  tex_ += noise;
  const int shapes = int(p.world * p.world * 9);
  for (int i = 0; i < shapes; i++) {
    const cv::Point c(int(U(g) * N), int(U(g) * N));
    const int col = int(U(g) * 255);
    const int sz = int((0.02 + 0.10 * U(g)) / p.tex_res);
    switch (int(U(g) * 4)) {
      case 0: cv::circle(tex_, c, sz / 2, col, -1, cv::LINE_AA); break;
      case 1: cv::rectangle(tex_, c, c + cv::Point(sz, int(sz * (0.3 + U(g)))), col, -1, cv::LINE_AA); break;
      case 2: cv::line(tex_, c, c + cv::Point(int((U(g) - 0.5) * 4 * sz), int((U(g) - 0.5) * 4 * sz)), col,
                       std::max(1, sz / 8), cv::LINE_AA); break;
      default: {
        std::vector<cv::Point> poly;
        for (int k = 0; k < 5; k++) poly.push_back(c + cv::Point(int((U(g) - 0.5) * sz), int((U(g) - 0.5) * sz)));
        cv::fillConvexPoly(tex_, poly, col, cv::LINE_AA);
      }
    }
  }
  cv::GaussianBlur(tex_, tex_, cv::Size(3, 3), 0);
}

void SimWorld::step(double v_cmd, double w_cmd, double dt) {
  const double a = dt / (0.15 + dt);
  v_ += a * (v_cmd - v_);
  w_ += a * (w_cmd - w_);
  const double tn = t_ + dt;
  // Dao động giữ thăng bằng: thân đưa tới lui quanh vị trí -> thêm vận tốc tiến
  const double osc_v = p_.balance_amp * 2 * M_PI * p_.balance_freq * std::cos(2 * M_PI * p_.balance_freq * tn);
  bool stuck = false;
  for (const auto &s : p_.stuck) stuck = stuck || (tn >= s.first && tn < s.second);
  const double v_true = stuck ? 0.0 : v_ + osc_v;
  const double w_true = stuck ? 0.0 : w_;
  const double thm = truth_.th + 0.5 * w_true * dt;
  truth_ = Pose2(truth_.x + v_true * dt * std::cos(thm), truth_.y + v_true * dt * std::sin(thm), truth_.th + w_true * dt);
  w_true_ = w_true;
  // Bánh báo: bánh kẹt thì vẫn quay theo lệnh (stepper không encoder)
  const double vr = v_ + osc_v, wr = w_;
  const double b = cfg_.robot.wheel_base;
  cumL_ += (vr - wr * b / 2) * dt * p_.scale_L;
  cumR_ += (vr + wr * b / 2) * dt * p_.scale_R;
  const double np = slam::deg2rad(p_.pitch_amp_deg) * std::sin(2 * M_PI * p_.pitch_freq * tn) + 0.2 * v_;
  pitch_rate_ = (np - pitch_) / dt;
  pitch_ = np;
  t_ = tn;
}

slam::McuRecord SimWorld::mcu() const {
  slam::McuRecord m{};
  std::normal_distribution<double> n(0, p_.gyro_noise);
  m.t = t_;
  m.t_us = uint32_t(uint64_t(t_ * 1e6) & 0xffffffffu);
  m.seq = const_cast<SimWorld *>(this)->seq_++;
  const double mps = cfg_.robot.metersPerStep();
  m.steps[0] = int32_t(std::lround(cumL_ / mps));
  m.steps[1] = int32_t(std::lround(cumR_ / mps));
  const double p = pitch_;
  m.gyro[0] = float(-std::sin(p) * w_true_ + n(rng_));
  m.gyro[1] = float(pitch_rate_ + n(rng_));
  m.gyro[2] = float(std::cos(p) * w_true_ + p_.gyro_bias + n(rng_));
  m.acc[0] = float(-9.81 * std::sin(p));
  m.acc[1] = 0;
  m.acc[2] = float(9.81 * std::cos(p));
  m.pitch = p_.send_pitch ? float(p) : NAN;
  m.state = slam::MCU_BALANCING;
  m.flags = slam::FLAG_CMD_ALIVE | slam::FLAG_MOTORS;
  return m;
}

cv::Mat SimWorld::render() const {
  const cv::Matx33d K = cam_.K(p_.img_w, p_.img_h);
  const Pose2 Twc = truth_ * cam_.T_b_c(pitch_);
  const double c = std::cos(Twc.th), s = std::sin(Twc.th);
  const double half = p_.world / 2;
  cv::Mat mx(p_.img_h, p_.img_w, CV_32F), my(p_.img_h, p_.img_w, CV_32F);
  for (int v = 0; v < p_.img_h; v++)
    for (int u = 0; u < p_.img_w; u++) {
      const cv::Vec3d rc((u - K(0, 2)) / K(0, 0), (v - K(1, 2)) / K(1, 1), 1.0);
      const cv::Vec3d rl = cam_.level(rc, pitch_);
      const double lx = p_.ceiling_h * rl[0] / rl[2], ly = p_.ceiling_h * rl[1] / rl[2];
      const double wx = Twc.x + c * lx - s * ly, wy = Twc.y + s * lx + c * ly;
      mx.at<float>(v, u) = float((wx + half) / p_.tex_res);
      my.at<float>(v, u) = float((wy + half) / p_.tex_res);
    }
  cv::Mat img;
  cv::remap(tex_, img, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(165));
  cv::Mat nz(img.size(), CV_8S);
  cv::randn(nz, 0, 3);
  cv::add(img, nz, img, cv::noArray(), CV_8U);
  return img;
}

double SimWorld::sonarRange(int k) const {
  const auto &m = p_.sonar_mount[k];
  const Pose2 s = truth_ * Pose2(m[0], m[1], slam::deg2rad(m[2]));
  const double half = slam::deg2rad(m[3]);
  double best = 1e9;
  for (int r = 0; r < 15; r++) {   // tiếng vang đầu tiên = vật gần nhất ở bất kỳ đâu trong chùm
    const double a = s.th - half + 2 * half * r / 14, dx = std::cos(a), dy = std::sin(a);
    for (const auto &b : p_.boxes) {
      if (t_ < b.t_on || t_ >= b.t_off) continue;
      double tmin = 0, tmax = 1e9;   // phương pháp slab
      const double o[2] = {s.x, s.y}, d[2] = {dx, dy}, lo[2] = {b.x0, b.y0}, hi[2] = {b.x1, b.y1};
      bool hit = true;
      for (int ax = 0; ax < 2 && hit; ax++) {
        if (std::fabs(d[ax]) < 1e-12) {
          if (o[ax] < lo[ax] || o[ax] > hi[ax]) hit = false;
        } else {
          double t1 = (lo[ax] - o[ax]) / d[ax], t2 = (hi[ax] - o[ax]) / d[ax];
          if (t1 > t2) std::swap(t1, t2);
          tmin = std::max(tmin, t1);
          tmax = std::min(tmax, t2);
          if (tmin > tmax) hit = false;
        }
      }
      if (hit && tmin > 0.02) best = std::min(best, tmin);
    }
  }
  if (best > p_.sonar_range) return -1;
  std::normal_distribution<double> n(0, p_.sonar_noise);
  return std::max(0.02, best + n(rng_));
}

std::vector<uint8_t> SimWorld::sonarGeomBytes() const {
  std::vector<uint8_t> b{1, uint8_t(p_.sonar_mount.size())};
  auto put16 = [&](int v) { b.push_back(uint8_t(v & 0xff)), b.push_back(uint8_t((v >> 8) & 0xff)); };
  put16(int(std::lround(p_.sonar_range * 1000)));
  for (const auto &m : p_.sonar_mount) {
    put16(int(std::lround(m[0] * 1000)));
    put16(int(std::lround(m[1] * 1000)));
    put16(int(std::lround(m[2] * 100)));
    b.push_back(uint8_t(m[3]));
    put16(m[2] == 0 ? 150 : 120);
  }
  return b;
}

// --------------------------------------------------------------------------------------------- runner
void SimRunner::sonarTick() {
  if (!w_.params().sonar || w_.params().sonar_mount.empty()) return;
  if (!geom_sent_) {
    const auto g = w_.sonarGeomBytes();
    if (log_) slam::writeRecord(log_, 'G', g.data(), uint32_t(g.size()));
    sys_.onSonarGeom(g.data(), g.size());
    geom_sent_ = true;
    next_sonar_ = w_.t();
  }
  if (w_.t() < next_sonar_) return;
  next_sonar_ += w_.params().sonar_period;   // lần lượt từng cảm biến như firmware
  const double d = w_.sonarRange(sonar_k_);
  slam::SonarRecord r{w_.t(), uint8_t(sonar_k_), uint16_t(d < 0 ? 0 : std::lround(d * 1000))};
  if (log_) slam::writeRecord(log_, 'S', &r, sizeof r);
  sys_.onSonar(r);
  sonar_k_ = (sonar_k_ + 1) % int(w_.params().sonar_mount.size());
}

void SimRunner::tick(double v, double w) {
  w_.step(v, w, dt_);
  hist_.emplace_back(w_.t(), w_.truth());
  const slam::McuRecord m = w_.mcu();
  if (log_) slam::writeRecord(log_, 'M', &m, sizeof m);
  sys_.onMcu(m);
  sonarTick();
  if (w_.t() >= next_img_) {
    next_img_ += 1.0 / fps_;
    const cv::Mat img = w_.render();
    // ảnh tới trễ một chút so với thời điểm chụp, như camera thật
    const double t_img = w_.t() - 0.004;
    if (log_) {
      std::vector<uint8_t> rec(12 + img.total());
      const uint16_t W = uint16_t(img.cols), H = uint16_t(img.rows);
      std::memcpy(rec.data(), &t_img, 8);
      std::memcpy(rec.data() + 8, &W, 2);
      std::memcpy(rec.data() + 10, &H, 2);
      std::memcpy(rec.data() + 12, img.data, img.total());
      slam::writeRecord(log_, 'I', rec.data(), uint32_t(rec.size()));
    }
    sys_.onImage(t_img, img);
  }
}

void SimRunner::run(double v, double w, double secs) {
  const int n = int(std::lround(secs / dt_));
  for (int i = 0; i < n; i++) tick(v, w);
}

void SimRunner::forward(double dist, double v) {
  const Pose2 s = w_.truth();
  const double sgn = dist >= 0 ? 1 : -1;
  for (int i = 0; i < 100000; i++) {
    const Pose2 d = Pose2::between(s, w_.truth());
    const double rem = std::fabs(dist) - sgn * d.x;
    if (rem <= 0.005) break;
    const double vc = sgn * std::min(v, std::max(0.03, 1.5 * rem));
    tick(vc, -1.5 * d.y * sgn);   // giữ thẳng như người lái
  }
  run(0, 0, 0.5);
}

void SimRunner::rotate(double angle, double wv) {
  const double target = w_.truth().th + angle;
  for (int i = 0; i < 100000; i++) {
    const double e = slam::wrapAngle(target - w_.truth().th);
    if (std::fabs(e) < slam::deg2rad(0.5)) break;
    tick(0, std::copysign(std::min(wv, std::max(0.1, 2.0 * std::fabs(e))), e));
  }
  run(0, 0, 0.5);
}

void SimRunner::command(const std::string &c) {
  if (log_) slam::writeRecord(log_, 'C', c.data(), uint32_t(c.size()));
  sys_.command(c);
}

bool SimRunner::navigate(double timeout, const std::function<void()> &each) {
  const double t_end = w_.t() + timeout;
  while (w_.t() < t_end) {
    const slam::ControlOutput c = sys_.lastCmd();
    tick(c.active ? c.v : 0, c.active ? c.w : 0);
    if (each) each();
    if (!sys_.navActive()) break;
  }
  run(0, 0, 0.3);
  return sys_.navState() == "arrived";
}

bool SimRunner::waitLocalized(double timeout, double w_rotate) {
  const double t_end = w_.t() + timeout;
  while (w_.t() < t_end && !sys_.localized()) tick(0, w_rotate);
  run(0, 0, 0.5);
  return sys_.localized();
}

double SimRunner::minDist(const Eigen::Vector2d &p, double t0, double t1) const {
  double best = 1e9;
  for (const auto &h : hist_)
    if (h.first >= t0 && h.first <= t1) best = std::min(best, (h.second.t() - p).norm());
  return best;
}

Pose2 SimRunner::truthAt(double t) const {
  if (hist_.empty()) return w_.truth();
  auto it = std::lower_bound(hist_.begin(), hist_.end(), t,
                             [](const std::pair<double, Pose2> &a, double v) { return a.first < v; });
  if (it == hist_.end()) return hist_.back().second;
  return it->second;
}

}  // namespace sim
