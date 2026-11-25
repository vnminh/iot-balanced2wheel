#pragma once
// M3: EKF 2D cho frame odom. Trạng thái x = [x, y, θ, b_g].
// Ngoài covariance toàn cục, giữ thêm chuyển động + covariance TƯƠNG ĐỐI kể từ mốc (keyframe gần nhất)
// để làm nhiễu cạnh odometry trong pose graph.
#include <Eigen/Core>

#include "slam/config.h"
#include "slam/pose2.h"

namespace slam {

class Ekf2d {
public:
  explicit Ekf2d(const EkfCfg &cfg);

  void reset();
  // d: quãng đường tâm trục (m), omega_meas: tốc độ quay trục đứng đo bởi gyro (rad/s, CHƯA trừ bias)
  void predict(double d, double omega_meas, double dt, bool saturated);
  // Xe đứng yên: gyro trung bình = bias (chỉ sửa bias, không làm odom nhảy)
  void updateZupt(double gyro_mean, double sigma);
  // ω_bánh − ω_gyro = −b_g (+ nhiễu lớn); có cổng χ² 1 bậc tự do (trả về false nếu bị loại)
  bool updateWheelOmega(double omega_wheel, double omega_gyro, double sigma, double gate = 6.63);

  // Mô hình chuyển động và Jacobian (tách riêng để test so với sai phân hữu hạn)
  static Eigen::Vector4d propagate(const Eigen::Vector4d &x, double d, double omega_meas, double dt);
  static Eigen::Matrix4d jacobian(const Eigen::Vector4d &x, double d, double omega_meas, double dt);
  void setState(const Eigen::Vector4d &x) { x_ = x; }
  const Eigen::Vector4d &state() const { return x_; }

  Pose2 pose() const { return Pose2(x_(0), x_(1), x_(2)); }
  double bias() const { return x_(3); }
  const Eigen::Matrix4d &P() const { return P_; }

  // Chuyển động tương đối từ mốc + covariance (đã gồm phần do bất định bias tích luỹ)
  Pose2 relPose() const { return rel_; }
  Eigen::Matrix3d relCov() const;
  void resetRelative();
  double distance() const { return dist_; }  // tổng quãng đường |d| (m), cho nhiễu localization

private:
  void biasUpdate(double residual, double h, double R);

  EkfCfg cfg_;
  Eigen::Vector4d x_;
  Eigen::Matrix4d P_;
  Pose2 rel_;
  Eigen::Matrix3d Prel_;
  double rel_time_ = 0;
  double dist_ = 0;
};

}  // namespace slam
