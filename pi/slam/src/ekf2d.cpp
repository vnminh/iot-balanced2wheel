#include "slam/ekf2d.h"

#include <cmath>

namespace slam {

Ekf2d::Ekf2d(const EkfCfg &cfg) : cfg_(cfg) { reset(); }

void Ekf2d::reset() {
  x_.setZero();
  P_.setZero();
  P_(0, 0) = P_(1, 1) = 1e-6;
  P_(2, 2) = 1e-6;
  P_(3, 3) = cfg_.init_sigma_bias * cfg_.init_sigma_bias;
  dist_ = 0;
  resetRelative();
}

void Ekf2d::resetRelative() {
  rel_ = Pose2();
  Prel_.setZero();
  rel_time_ = 0;
}

Eigen::Vector4d Ekf2d::propagate(const Eigen::Vector4d &x, double d, double omega_meas, double dt) {
  const double dth = (omega_meas - x(3)) * dt;
  const double thm = x(2) + 0.5 * dth;
  Eigen::Vector4d o = x;
  o(0) += d * std::cos(thm);
  o(1) += d * std::sin(thm);
  o(2) = wrapAngle(x(2) + dth);
  return o;
}

Eigen::Matrix4d Ekf2d::jacobian(const Eigen::Vector4d &x, double d, double omega_meas, double dt) {
  const double thm = x(2) + 0.5 * (omega_meas - x(3)) * dt;
  const double c = std::cos(thm), s = std::sin(thm);
  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0, 2) = -d * s;
  F(1, 2) = d * c;
  F(0, 3) = d * s * 0.5 * dt;
  F(1, 3) = -d * c * 0.5 * dt;
  F(2, 3) = -dt;
  return F;
}

void Ekf2d::predict(double d, double omega_meas, double dt, bool saturated) {
  const double dth = (omega_meas - x_(3)) * dt;
  const double thm = x_(2) + 0.5 * dth;
  const double c = std::cos(thm), s = std::sin(thm);
  const Eigen::Matrix4d F = jacobian(x_, d, omega_meas, dt);

  const double ad = std::fabs(d);
  const double kd = saturated ? cfg_.sat_noise_mult : 1.0;
  const double var_d = kd * kd * cfg_.sigma_d_per_m * cfg_.sigma_d_per_m * ad;
  const double var_lat = cfg_.sigma_lat_per_m * cfg_.sigma_lat_per_m * ad;
  const double var_th = cfg_.sigma_gyro * cfg_.sigma_gyro * dt;
  const double var_b = cfg_.sigma_bias_rw * cfg_.sigma_bias_rw * dt;

  auto noise = [&](double cc, double ss) {
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = cc * cc * var_d + ss * ss * var_lat;
    Q(1, 1) = ss * ss * var_d + cc * cc * var_lat;
    Q(0, 1) = Q(1, 0) = cc * ss * (var_d - var_lat);
    Q(2, 2) = var_th;
    Q(3, 3) = var_b;
    return Q;
  };

  x_ = propagate(x_, d, omega_meas, dt);
  P_ = F * P_ * F.transpose() + noise(c, s);

  // Chuyển động tương đối (không có bias trong trạng thái; phần bất định bias cộng ở relCov)
  const double rthm = rel_.th + 0.5 * dth;
  const double rc = std::cos(rthm), rs = std::sin(rthm);
  Eigen::Matrix3d Fr = Eigen::Matrix3d::Identity();
  Fr(0, 2) = -d * rs;
  Fr(1, 2) = d * rc;
  Prel_ = Fr * Prel_ * Fr.transpose() + noise(rc, rs).topLeftCorner<3, 3>();
  rel_ = Pose2(rel_.x + d * rc, rel_.y + d * rs, rel_.th + dth);
  rel_time_ += dt;
  dist_ += ad;
}

Eigen::Matrix3d Ekf2d::relCov() const {
  // Sai số bias là hằng trong đoạn -> sai số góc tăng tuyến tính theo thời gian của đoạn
  Eigen::Matrix3d C = Prel_;
  C(2, 2) += P_(3, 3) * rel_time_ * rel_time_;
  return C;
}

// Cập nhật CHỈ bias (Schmidt): vị trí / hướng odom không bao giờ nhảy (odom -> base_link phải liên tục).
// Gain chỉ có thành phần bias; dạng Joseph giữ P đối xứng, xác định dương.
void Ekf2d::biasUpdate(double residual, double h, double R) {
  const double S = h * P_(3, 3) * h + R;
  const double k = P_(3, 3) * h / S;
  Eigen::Vector4d K = Eigen::Vector4d::Zero();
  K(3) = k;
  x_(3) += k * residual;
  Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
  A.row(3) -= K(3) * Eigen::RowVector4d(0, 0, 0, h);
  P_ = A * P_ * A.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
}

void Ekf2d::updateZupt(double gyro_mean, double sigma) { biasUpdate(gyro_mean - x_(3), 1.0, sigma * sigma); }

bool Ekf2d::updateWheelOmega(double omega_wheel, double omega_gyro, double sigma, double gate) {
  // z = ω_bánh − ω_gyro = −b_g
  const double r = (omega_wheel - omega_gyro) + x_(3);
  const double S = P_(3, 3) + sigma * sigma;
  if (r * r / S > gate) return false;
  biasUpdate(r, -1.0, sigma * sigma);
  return true;
}

}  // namespace slam
