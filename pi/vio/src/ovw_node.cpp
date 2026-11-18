// =============================================================================
//  ovw_node: OpenVINS (không ROS) + cập nhật odometry bánh xe cho xe cân bằng.
//
//  Đầu vào (stdin, nhị phân, little-endian) - do robot/vio_bridge.py gửi:
//    'I' double t, float acc[3], float gyro[3], int32 steps[2], uint8 flags
//    'C' double t, uint16 w, uint16 h, uint8 gray[w*h]
//    'R' reset bộ lọc        'Q' thoát
//  Đầu ra (stdout, mỗi dòng một JSON):
//    {"type":"pose",...}  sau mỗi ảnh đã xử lý
//    {"type":"status",...}
//  Log của OpenVINS bị chuyển sang stderr để không lẫn vào JSON.
//
//  Odometry bánh xe (xe cân bằng, bánh không trượt):
//    tốc độ tâm trục so với đất  v = (bước/s trung bình)/STEPS_PER_METER + r * ω_chúi
//    (động cơ quay bánh SO VỚI THÂN; thân chúi tới cũng làm bánh lăn tới)
//  Đo lường cho EKF, với u = R_GtoI v_I + ω × p_OinI là vận tốc tâm trục trong khung IMU:
//    r1 = v - d·u        (d: hướng tới nằm ngang trong khung IMU)
//    r2 = 0 - a·u        (không trượt ngang; a: trục bánh)
//    r3 = 0 - e_z·v_O^G  (tâm trục không đi lên/xuống, nền phẳng)
// =============================================================================
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <boost/math/distributions/chi_squared.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

using namespace ov_msckf;

namespace {

FILE *g_out = nullptr;  // stdout gốc (JSON)

struct WheelParams {
  bool enable = true;
  double steps_per_meter = 15670.64;   // 3200 bước/vòng / (π * 0.065)
  double wheel_radius = 0.0325;
  Eigen::Vector3d axle_in_imu{0, 1, 0};   // trục bánh (chỉ sang TRÁI) trong khung IMU
  Eigen::Vector3d p_axle_in_imu{0, 0, -0.05};  // tâm trục trong khung IMU (m)
  double sigma_fwd = 0.03, sigma_lat = 0.02, sigma_vert = 0.03;  // m/s
  double chi2_mult = 3.0;
  double window = 0.03;  // s, nửa cửa sổ lấy đạo hàm số bước
};

struct WheelSample {
  double t;
  int32_t sl, sr;
  Eigen::Vector3d gyro;
  uint8_t flags;
};

struct Stats {
  int wheel_ok = 0, wheel_rej = 0, frames = 0, dropped = 0;
} g_stats;

bool readExact(void *dst, size_t n) {
  auto *p = static_cast<uint8_t *>(dst);
  while (n) {
    ssize_t r = ::read(0, p, n);
    if (r <= 0) return false;
    p += r;
    n -= (size_t)r;
  }
  return true;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &w) {
  Eigen::Matrix3d m;
  m << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return m;
}

WheelParams loadWheel(const std::string &path) {
  WheelParams w;
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    PRINT_WARNING("[wheel] khong mo duoc %s, dung mac dinh\n", path.c_str());
    return w;
  }
  auto rd = [&](const char *k, double &v) {
    if (!fs[k].empty()) fs[k] >> v;
  };
  auto rdv = [&](const char *k, Eigen::Vector3d &v) {
    if (!fs[k].empty()) {
      std::vector<double> x;
      fs[k] >> x;
      if (x.size() == 3) v << x[0], x[1], x[2];
    }
  };
  int en = 1;
  if (!fs["enable"].empty()) fs["enable"] >> en;
  w.enable = en != 0;
  rd("steps_per_meter", w.steps_per_meter);
  rd("wheel_radius", w.wheel_radius);
  rdv("axle_in_imu", w.axle_in_imu);
  rdv("p_axle_in_imu", w.p_axle_in_imu);
  rd("sigma_fwd", w.sigma_fwd);
  rd("sigma_lat", w.sigma_lat);
  rd("sigma_vert", w.sigma_vert);
  rd("chi2_mult", w.chi2_mult);
  rd("window", w.window);
  w.axle_in_imu.normalize();
  return w;
}

class Node {
public:
  Node(std::string cfg, std::string wheel_cfg) : cfg_(std::move(cfg)), wheel_(loadWheel(wheel_cfg)) { reset(); }

  void reset() {
    auto parser = std::make_shared<ov_core::YamlParser>(cfg_);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);
    params_ = VioManagerOptions();
    params_.print_and_load(parser);
    params_.use_multi_threading_subs = false;
    params_.use_multi_threading_pubs = false;
    if (!parser->successful()) {
      PRINT_ERROR("[ovw] loi doc cau hinh %s\n", cfg_.c_str());
      std::exit(2);
    }
    sys_ = std::make_shared<VioManager>(params_);
    cams_.clear();
    wheel_buf_.clear();
    last_imu_t_ = -1;
    last_out_ = 0;
    status("reset");
  }

  void onImu(double t, const float acc[3], const float gyr[3], int32_t sl, int32_t sr, uint8_t flags) {
    if (t <= last_imu_t_) return;  // không nhận mẫu đi lùi thời gian
    last_imu_t_ = t;
    ov_core::ImuData m;
    m.timestamp = t;
    m.wm << gyr[0], gyr[1], gyr[2];
    m.am << acc[0], acc[1], acc[2];
    sys_->feed_measurement_imu(m);
    wheel_buf_.push_back({t, sl, sr, m.wm, flags});
    while (!wheel_buf_.empty() && wheel_buf_.front().t < t - 3.0) wheel_buf_.pop_front();
    processCameras();
  }

  void onCamera(double t, cv::Mat img) {
    // Không kịp xử lý -> bỏ ảnh cũ, giữ thời gian thực
    while (cams_.size() >= 2) {
      cams_.pop_front();
      g_stats.dropped++;
    }
    cams_.push_back({t, std::move(img)});
    processCameras();
  }

private:
  struct Cam {
    double t;
    cv::Mat img;
  };

  void processCameras() {
    // Chỉ xử lý ảnh khi đã có IMU phủ quá thời điểm ảnh (+ cửa sổ bánh xe)
    const double need = wheel_.window + 0.01 + std::max(0.0, sys_->get_state()->_calib_dt_CAMtoIMU->value()(0));
    while (!cams_.empty() && last_imu_t_ >= cams_.front().t + need) {
      Cam c = std::move(cams_.front());
      cams_.pop_front();
      ov_core::CameraData msg;
      msg.timestamp = c.t;
      msg.sensor_ids.push_back(0);
      msg.images.push_back(c.img);
      msg.masks.push_back(cv::Mat::zeros(c.img.rows, c.img.cols, CV_8UC1));
      auto t0 = std::chrono::steady_clock::now();
      sys_->feed_measurement_camera(msg);
      if (sys_->initialized() && wheel_.enable) wheelUpdate();
      double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      g_stats.frames++;
      publish(ms);
    }
  }

  // Nội suy tổng số bước tại thời điểm t
  bool stepsAt(double t, double &sl, double &sr) const {
    if (wheel_buf_.size() < 2 || t < wheel_buf_.front().t || t > wheel_buf_.back().t) return false;
    for (size_t i = 1; i < wheel_buf_.size(); i++) {
      const auto &a = wheel_buf_[i - 1], &b = wheel_buf_[i];
      if (b.t >= t) {
        double k = (b.t > a.t) ? (t - a.t) / (b.t - a.t) : 0.0;
        sl = a.sl + k * (double)(b.sl - a.sl);
        sr = a.sr + k * (double)(b.sr - a.sr);
        return true;
      }
    }
    return false;
  }

  void wheelUpdate() {
    auto state = sys_->get_state();
    const double t = state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0);  // giờ IMU
    const double w = wheel_.window;
    double l0, r0, l1, r1;
    if (!stepsAt(t - w, l0, r0) || !stepsAt(t + w, l1, r1)) return;

    // Gyro trung bình trong cửa sổ, và động cơ phải đang bật suốt cửa sổ
    Eigen::Vector3d gsum = Eigen::Vector3d::Zero();
    int n = 0;
    for (const auto &s : wheel_buf_) {
      if (s.t < t - w || s.t > t + w) continue;
      if (!(s.flags & 0x02)) return;  // động cơ tắt (xe nằm / bị nhấc) -> bánh quay tự do, bỏ qua
      gsum += s.gyro;
      n++;
    }
    if (n == 0) return;

    const double rate = 0.5 * ((l1 - l0) + (r1 - r0)) / (2 * w);  // bước/s trung bình 2 bánh
    const Eigen::Matrix3d R = state->_imu->Rot();                  // R_GtoI
    const Eigen::Vector3d v = state->_imu->vel();
    const Eigen::Vector3d om = gsum / n - state->_imu->bias_g();
    const Eigen::Vector3d &a = wheel_.axle_in_imu;
    const Eigen::Vector3d &p = wheel_.p_axle_in_imu;
    const double v_meas = rate / wheel_.steps_per_meter + wheel_.wheel_radius * om.dot(a);

    const Eigen::Vector3d Rv = R * v;
    const Eigen::Vector3d wxp = om.cross(p);
    const Eigen::Vector3d u = Rv + wxp;                // vận tốc tâm trục, khung IMU
    const Eigen::Vector3d gI = R * Eigen::Vector3d::UnitZ();  // hướng lên của thế giới, khung IMU
    Eigen::Vector3d d = a.cross(gI);
    if (d.norm() < 0.3) return;  // xe gần như dựng đứng trục bánh -> bất thường
    d.normalize();

    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();  // [θ v bg]
    Eigen::Vector3d res;
    // r1: tiến
    H.block<1, 3>(0, 0) = d.transpose() * skew(Rv);
    H.block<1, 3>(0, 3) = d.transpose() * R;
    H.block<1, 3>(0, 6) = d.transpose() * skew(p);
    res(0) = v_meas - d.dot(u);
    // r2: ngang
    H.block<1, 3>(1, 0) = a.transpose() * skew(Rv);
    H.block<1, 3>(1, 3) = a.transpose() * R;
    H.block<1, 3>(1, 6) = a.transpose() * skew(p);
    res(1) = -a.dot(u);
    // r3: thẳng đứng (khung thế giới)
    const Eigen::RowVector3d ez = Eigen::Vector3d::UnitZ().transpose();
    H.block<1, 3>(2, 0) = -ez * R.transpose() * skew(wxp);
    H.block<1, 3>(2, 3) = ez;
    H.block<1, 3>(2, 6) = ez * R.transpose() * skew(p);
    res(2) = -(v + R.transpose() * wxp)(2);

    Eigen::Matrix3d Rn = Eigen::Vector3d(wheel_.sigma_fwd * wheel_.sigma_fwd, wheel_.sigma_lat * wheel_.sigma_lat,
                                         wheel_.sigma_vert * wheel_.sigma_vert)
                             .asDiagonal();
    std::vector<std::shared_ptr<ov_type::Type>> order = {state->_imu->q(), state->_imu->v(), state->_imu->bg()};
    Eigen::MatrixXd P = StateHelper::get_marginal_covariance(state, order);
    Eigen::Matrix3d S = H * P * H.transpose() + Rn;
    double chi2 = res.dot(S.llt().solve(res));
    static const double chi2_thr = boost::math::quantile(boost::math::chi_squared(3), 0.95);
    if (chi2 > wheel_.chi2_mult * chi2_thr) {
      g_stats.wheel_rej++;
      return;
    }
    StateHelper::EKFUpdate(state, order, H, res, Rn);
    g_stats.wheel_ok++;
    last_v_meas_ = v_meas;
  }

  void publish(double ms) {
    auto state = sys_->get_state();
    bool init = sys_->initialized();
    char buf[4096];
    int n;
    if (init) {
      Eigen::Matrix3d RtoG = state->_imu->Rot().transpose();
      Eigen::Vector3d p = state->_imu->pos(), v = state->_imu->vel();
      Eigen::Vector4d q = state->_imu->quat();  // JPL (x y z w) của R_GtoI
      double yaw = std::atan2(RtoG(1, 0), RtoG(0, 0));
      double pitch = std::asin(std::max(-1.0, std::min(1.0, -RtoG(2, 0))));
      n = snprintf(buf, sizeof(buf),
                   "{\"type\":\"pose\",\"init\":true,\"t\":%.6f,\"p\":[%.4f,%.4f,%.4f],\"q\":[%.6f,%.6f,%.6f,%.6f],"
                   "\"v\":[%.3f,%.3f,%.3f],\"yaw\":%.4f,\"pitch\":%.4f,\"ms\":%.1f,\"td\":%.4f,\"vw\":%.3f,"
                   "\"slam\":%zu,\"wheel_ok\":%d,\"wheel_rej\":%d,\"frames\":%d,\"dropped\":%d,\"pts\":[",
                   state->_timestamp, p(0), p(1), p(2), q(0), q(1), q(2), q(3), v(0), v(1), v(2), yaw, pitch, ms,
                   state->_calib_dt_CAMtoIMU->value()(0), last_v_meas_, state->_features_SLAM.size(), g_stats.wheel_ok,
                   g_stats.wheel_rej, g_stats.frames, g_stats.dropped);
      std::string s(buf, n);
      // Điểm 3D mới (MSCKF vừa tam giác hoá) -> bản đồ tích luỹ bên Python
      bool first = true;
      for (const auto &f : sys_->get_good_features_MSCKF()) {
        if (!f.allFinite() || (f - p).norm() > 15.0) continue;
        n = snprintf(buf, sizeof(buf), "%s[%.3f,%.3f,%.3f]", first ? "" : ",", f(0), f(1), f(2));
        s.append(buf, n);
        first = false;
      }
      s += "],\"slam_pts\":[";
      first = true;
      for (const auto &f : sys_->get_features_SLAM()) {
        if (!f.allFinite()) continue;
        n = snprintf(buf, sizeof(buf), "%s[%.3f,%.3f,%.3f]", first ? "" : ",", f(0), f(1), f(2));
        s.append(buf, n);
        first = false;
      }
      s += "]}\n";
      fputs(s.c_str(), g_out);
    } else {
      n = snprintf(buf, sizeof(buf), "{\"type\":\"pose\",\"init\":false,\"t\":%.6f,\"ms\":%.1f,\"frames\":%d}\n",
                   state->_timestamp, ms, g_stats.frames);
      fputs(buf, g_out);
    }
    fflush(g_out);

    // Ảnh theo dõi đặc trưng cho web (2 lần/giây), ghi vào RAM (/dev/shm)
    double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (now - last_out_ > 0.5) {
      last_out_ = now;
      cv::Mat viz = sys_->get_historical_viz_image();
      if (!viz.empty()) {
        cv::imwrite("/dev/shm/ovw_track.tmp.jpg", viz, {cv::IMWRITE_JPEG_QUALITY, 70});
        std::rename("/dev/shm/ovw_track.tmp.jpg", "/dev/shm/ovw_track.jpg");
      }
    }
  }

  void status(const char *what) {
    fprintf(g_out, "{\"type\":\"status\",\"msg\":\"%s\"}\n", what);
    fflush(g_out);
  }

  std::string cfg_;
  WheelParams wheel_;
  VioManagerOptions params_;
  std::shared_ptr<VioManager> sys_;
  std::deque<Cam> cams_;
  std::deque<WheelSample> wheel_buf_;
  double last_imu_t_ = -1;
  double last_out_ = 0;
  double last_v_meas_ = 0;
};

}  // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "dung: %s estimator_config.yaml wheel.yaml\n", argv[0]);
    return 1;
  }
  // stdout gốc dành cho JSON; mọi printf của OpenVINS đi sang stderr
  int out_fd = dup(1);
  dup2(2, 1);
  g_out = fdopen(out_fd, "w");

  Node node(argv[1], argv[2]);
  std::vector<uint8_t> img;
  while (true) {
    uint8_t type;
    if (!readExact(&type, 1)) break;
    if (type == 'I') {
#pragma pack(push, 1)
      struct {
        double t;
        float acc[3], gyr[3];
        int32_t steps[2];
        uint8_t flags;
      } m;
#pragma pack(pop)
      static_assert(sizeof(m) == 41, "goi IMU 41 byte");
      if (!readExact(&m, sizeof(m))) break;
      node.onImu(m.t, m.acc, m.gyr, m.steps[0], m.steps[1], m.flags);
    } else if (type == 'C') {
#pragma pack(push, 1)
      struct {
        double t;
        uint16_t w, h;
      } m;
#pragma pack(pop)
      if (!readExact(&m, sizeof(m))) break;
      cv::Mat im(m.h, m.w, CV_8UC1);
      if (!readExact(im.data, (size_t)m.w * m.h)) break;
      node.onCamera(m.t, im);
    } else if (type == 'R') {
      node.reset();
    } else if (type == 'Q') {
      break;
    } else {
      PRINT_ERROR("[ovw] goi tin la: 0x%02x\n", type);
      return 3;
    }
  }
  return 0;
}
