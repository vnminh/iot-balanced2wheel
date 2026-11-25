#include "slam/wheel_odometry.h"

#include <cmath>
#include <cstdlib>

namespace slam {

bool WheelOdometry::update(double t, const int32_t steps[2], OdomDelta &out) {
  if (!has_) {
    has_ = true;
    t_ = t;
    s_[0] = steps[0];
    s_[1] = steps[1];
    return false;
  }
  const int64_t dl = int64_t(steps[0]) - s_[0], dr = int64_t(steps[1]) - s_[1];
  const double dt = t - t_;
  // 16000 bước/s tối đa -> mỗi mẫu 5 ms chỉ vài chục bước. Nhảy > 0,5 m = ESP32 reset (số bước về 0)
  // hoặc mất dữ liệu lâu: bỏ đoạn này thay vì cộng một bước nhảy giả.
  const double lim = 0.5 / mps_;
  t_ = t;
  s_[0] = steps[0];
  s_[1] = steps[1];
  if (std::llabs(dl) > lim || std::llabs(dr) > lim || dt <= 0 || dt > 1.0) {
    ++resets_;
    return false;
  }
  out.dt = dt;
  out.dL = dl * mps_;
  out.dR = dr * mps_;
  out.d = 0.5 * (out.dL + out.dR);
  out.dth_wheel = (out.dR - out.dL) / base_;
  return true;
}

double PitchEstimator::update(const McuRecord &m, double dt) {
  rate_ = m.gyro[1];  // quay quanh Y (trục bánh, sang trái) dương = chúi tới
  if (std::isfinite(m.pitch)) {
    pitch_ = m.pitch;
    init_ = true;
    return pitch_;
  }
  const double acc_pitch = std::atan2(-double(m.acc[0]), double(m.acc[2]));
  if (!init_ || dt <= 0 || dt > 0.5) {
    pitch_ = acc_pitch;
    init_ = true;
  } else {
    const double a = tau_ / (tau_ + dt);
    pitch_ = a * (pitch_ + rate_ * dt) + (1 - a) * acc_pitch;
  }
  return pitch_;
}

double worldYawRate(const float gyro[3], double pitch) {
  return -std::sin(pitch) * gyro[0] + std::cos(pitch) * gyro[2];
}

}  // namespace slam
