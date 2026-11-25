#pragma once
// M2: odometry từ số bước tích luỹ + các đại lượng IMU cần cho SLAM 2D.
#include <cstdint>

#include "slam/protocol.h"

namespace slam {

struct OdomDelta {
  double dt = 0;
  double dL = 0, dR = 0;  // m
  double d = 0;           // m, quãng đường tâm trục
  double dth_wheel = 0;   // rad, (dR − dL) / wheel_base
};

class WheelOdometry {
public:
  WheelOdometry(double meters_per_step, double wheel_base) : mps_(meters_per_step), base_(wheel_base) {}
  // Trả về false ở mẫu đầu tiên hoặc khi số bước nhảy bất thường (ESP32 khởi động lại).
  bool update(double t, const int32_t steps[2], OdomDelta &out);
  void reset() { has_ = false; }
  int resets() const { return resets_; }

private:
  double mps_, base_;
  bool has_ = false;
  double t_ = 0;
  int32_t s_[2] = {0, 0};
  int resets_ = 0;
};

// Góc chúi thân xe (rad, + = chúi tới). Ưu tiên pitch của firmware (đúng góc vòng cân bằng dùng);
// firmware cũ không gửi thì tự lọc bù từ acc + gyro như firmware.
class PitchEstimator {
public:
  explicit PitchEstimator(double tau) : tau_(tau) {}
  double update(const McuRecord &m, double dt);
  double pitch() const { return pitch_; }
  double rate() const { return rate_; }

private:
  double tau_;
  bool init_ = false;
  double pitch_ = 0, rate_ = 0;
};

// Tốc độ quay quanh trục THẲNG ĐỨNG của thế giới từ gyro khung MPU (X tới, Y trái, Z lên)
// khi thân chúi một góc pitch quanh trục Y: ω = −sin(p)·gx + cos(p)·gz.
double worldYawRate(const float gyro[3], double pitch);

}  // namespace slam
