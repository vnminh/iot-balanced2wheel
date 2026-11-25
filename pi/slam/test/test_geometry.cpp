#include <gtest/gtest.h>

#include "slam/frontend.h"
#include "slam/pose2.h"
#include "slam/wheel_odometry.h"

using namespace slam;

TEST(Pose2, WrapRange) {
  EXPECT_NEAR(wrapAngle(M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(-M_PI), M_PI, 1e-12);
  EXPECT_NEAR(wrapAngle(3 * M_PI / 2), -M_PI / 2, 1e-12);
  EXPECT_NEAR(wrapAngle(-7.0), -7.0 + 2 * M_PI, 1e-12);
  for (double a = -20; a < 20; a += 0.37) {
    const double w = wrapAngle(a);
    EXPECT_GT(w, -M_PI);
    EXPECT_LE(w, M_PI);
    EXPECT_NEAR(std::sin(w), std::sin(a), 1e-9);
  }
}

TEST(Pose2, ComposeInverse) {
  const Pose2 a(1.0, -2.0, 0.7), b(-0.3, 0.5, -2.9);
  const Pose2 i = a * a.inverse();
  EXPECT_NEAR(i.x, 0, 1e-12);
  EXPECT_NEAR(i.y, 0, 1e-12);
  EXPECT_NEAR(i.th, 0, 1e-12);
  const Pose2 ab = a * b, back = Pose2::between(a, ab);
  EXPECT_NEAR(back.x, b.x, 1e-12);
  EXPECT_NEAR(back.y, b.y, 1e-12);
  EXPECT_NEAR(back.th, b.th, 1e-12);
  // Điểm: (a ⊕ b)·p = a·(b·p)
  const Eigen::Vector2d p(0.4, 0.9);
  EXPECT_NEAR(((a * b) * p - a * (b * p)).norm(), 0, 1e-12);
  // Quay 90°: điểm (1,0) -> (0,1)
  EXPECT_NEAR((Pose2(0, 0, M_PI / 2) * Eigen::Vector2d(1, 0) - Eigen::Vector2d(0, 1)).norm(), 0, 1e-12);
}

TEST(Pose2, LerpShortestAngle) {
  const Pose2 a(0, 0, deg2rad(170)), b(1, 2, deg2rad(-170));
  const Pose2 m = Pose2::lerp(a, b, 0.5);
  EXPECT_NEAR(std::fabs(m.th), M_PI, 1e-9);
  EXPECT_NEAR(m.x, 0.5, 1e-12);
}

TEST(WheelOdometry, DifferentialKinematics) {
  const double mps = M_PI * 0.065 / 3200, base = 0.17;
  WheelOdometry wo(mps, base);
  int32_t s[2] = {100000, 200000};
  OdomDelta d;
  EXPECT_FALSE(wo.update(0.0, s, d));
  s[0] += 100;   // trái 100 bước
  s[1] += 300;   // phải 300 bước
  ASSERT_TRUE(wo.update(0.005, s, d));
  EXPECT_NEAR(d.dL, 100 * mps, 1e-12);
  EXPECT_NEAR(d.dR, 300 * mps, 1e-12);
  EXPECT_NEAR(d.d, 200 * mps, 1e-12);
  EXPECT_NEAR(d.dth_wheel, 200 * mps / base, 1e-12);  // phải nhanh hơn -> quay trái (+)
  // ESP32 khởi động lại: số bước về 0 -> bỏ, không cộng bước nhảy
  int32_t z[2] = {0, 0};
  EXPECT_FALSE(wo.update(0.010, z, d));
  EXPECT_EQ(wo.resets(), 1);
}

TEST(WheelOdometry, MetersPerStep) {
  RobotCfg r;
  EXPECT_NEAR(r.metersPerStep(), M_PI * 0.065 / 3200, 1e-15);
  EXPECT_NEAR(1.0 / r.metersPerStep(), 15670.6, 0.1);   // khớp STEPS_PER_METER của firmware
}

TEST(Imu, WorldYawRateUnderPitch) {
  // Quay quanh trục đứng 0.5 rad/s khi thân chúi 10°: MPU thấy gx = −sin(p)·ω, gz = cos(p)·ω
  const double w = 0.5, p = deg2rad(10);
  const float g[3] = {float(-std::sin(p) * w), 0.f, float(std::cos(p) * w)};
  EXPECT_NEAR(worldYawRate(g, p), w, 1e-6);
  EXPECT_NEAR(worldYawRate(g, 0), std::cos(p) * w, 1e-6);  // không bù thì sai ~1.5%
}

TEST(Imu, PitchFallbackFromAccel) {
  PitchEstimator pe(0.5);
  McuRecord m{};
  const double p = deg2rad(5);
  m.acc[0] = float(-9.81 * std::sin(p));
  m.acc[2] = float(9.81 * std::cos(p));
  m.pitch = NAN;
  for (int i = 0; i < 400; i++) pe.update(m, 0.005);
  EXPECT_NEAR(pe.pitch(), p, 1e-4);
  m.pitch = 0.1f;   // firmware mới gửi pitch -> dùng thẳng
  EXPECT_NEAR(pe.update(m, 0.005), 0.1, 1e-6);
}

TEST(CameraModel, LevelUnlevelRoundTrip) {
  CameraCfg c;
  c.yaw_deg = 73;
  c.mount_roll_deg = 1.5;
  c.mount_pitch_deg = -2;
  CameraModel cam(c);
  const cv::Vec3d r(0.1, -0.2, 1.0);
  const cv::Vec3d back = cam.unlevel(cam.level(r, 0.12), 0.12);
  EXPECT_NEAR(cv::norm(back - r), 0, 1e-12);
}

TEST(CameraModel, PitchTiltsRayAlongBodyForward) {
  // Camera ảo: x ảnh = mũi xe khi yaw_deg = 0. Thân chúi tới: tia thẳng lên của camera thật
  // ngả về phía SAU xe (camera gắn vào thân, đỉnh thân ngả tới thì trục quang nghiêng tới -> tia "lên" của
  // camera thật trỏ về phía trước) -> trong khung camera ảo x > 0.
  CameraCfg c;
  c.yaw_deg = 0;
  CameraModel cam(c);
  const cv::Vec3d r = cam.level(cv::Vec3d(0, 0, 1), deg2rad(5));
  EXPECT_GT(r[0], 0.0);
  EXPECT_NEAR(r[1], 0.0, 1e-12);
}
