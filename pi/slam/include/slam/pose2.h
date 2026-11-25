#pragma once
// SE(2): (x, y, θ). Đơn vị SI, góc luôn wrap về (−π, π].
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

namespace slam {

inline double wrapAngle(double a) {
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0) a += 2.0 * M_PI;
  return a - M_PI;
}

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

struct Pose2 {
  double x = 0, y = 0, th = 0;

  Pose2() = default;
  Pose2(double x_, double y_, double th_) : x(x_), y(y_), th(wrapAngle(th_)) {}

  // Ghép: this ⊕ b  (b biểu diễn trong khung của this)
  Pose2 operator*(const Pose2 &b) const {
    const double c = std::cos(th), s = std::sin(th);
    return Pose2(x + c * b.x - s * b.y, y + s * b.x + c * b.y, th + b.th);
  }
  Pose2 inverse() const {
    const double c = std::cos(th), s = std::sin(th);
    return Pose2(-c * x - s * y, s * x - c * y, -th);
  }
  Eigen::Vector2d operator*(const Eigen::Vector2d &p) const {
    const double c = std::cos(th), s = std::sin(th);
    return {x + c * p.x() - s * p.y(), y + s * p.x() + c * p.y()};
  }
  Eigen::Vector2d t() const { return {x, y}; }
  Eigen::Vector3d vec() const { return {x, y, th}; }
  static Pose2 between(const Pose2 &a, const Pose2 &b) { return a.inverse() * b; }
  // Nội suy tuyến tính (góc theo đường ngắn nhất), u ∈ [0, 1]
  static Pose2 lerp(const Pose2 &a, const Pose2 &b, double u) {
    return Pose2(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y), a.th + u * wrapAngle(b.th - a.th));
  }
};

// Jacobian của phép đổi khung covariance: cov của (T ⊕ Δ) theo Δ khi T cố định = R3(T.th)
inline Eigen::Matrix3d rot3(double th) {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  const double c = std::cos(th), s = std::sin(th);
  R(0, 0) = c; R(0, 1) = -s;
  R(1, 0) = s; R(1, 1) = c;
  return R;
}

}  // namespace slam
