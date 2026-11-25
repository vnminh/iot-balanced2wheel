#include <gtest/gtest.h>

#include "slam/ekf2d.h"

using namespace slam;

TEST(Ekf2d, JacobianMatchesFiniteDifference) {
  const Eigen::Vector4d x(0.3, -1.2, 0.8, 0.01);
  const double d = 0.004, w = 0.4, dt = 0.005;
  const Eigen::Matrix4d F = Ekf2d::jacobian(x, d, w, dt);
  const double h = 1e-6;
  for (int k = 0; k < 4; k++) {
    Eigen::Vector4d xp = x, xm = x;
    xp(k) += h;
    xm(k) -= h;
    const Eigen::Vector4d df = (Ekf2d::propagate(xp, d, w, dt) - Ekf2d::propagate(xm, d, w, dt)) / (2 * h);
    for (int r = 0; r < 4; r++) EXPECT_NEAR(F(r, k), df(r), 1e-7) << "F(" << r << "," << k << ")";
  }
}

TEST(Ekf2d, StraightLineAndTurn) {
  EkfCfg c;
  Ekf2d ekf(c);
  for (int i = 0; i < 200; i++) ekf.predict(0.005, 0.0, 0.005, false);  // 1 m thẳng
  EXPECT_NEAR(ekf.pose().x, 1.0, 1e-9);
  EXPECT_NEAR(ekf.pose().y, 0.0, 1e-9);
  for (int i = 0; i < 200; i++) ekf.predict(0.0, M_PI / 2, 0.005, false);  // quay 90° tại chỗ
  EXPECT_NEAR(ekf.pose().th, M_PI / 2, 1e-9);
  for (int i = 0; i < 100; i++) ekf.predict(0.005, 0.0, 0.005, false);
  EXPECT_NEAR(ekf.pose().x, 1.0, 1e-9);
  EXPECT_NEAR(ekf.pose().y, 0.5, 1e-9);
  // Covariance tương đối dương, tăng theo quãng đường
  const Eigen::Matrix3d R = ekf.relCov();
  EXPECT_GT(R(0, 0) + R(1, 1), 0);
  EXPECT_NEAR(ekf.relPose().y, 0.5, 1e-9);
}

TEST(Ekf2d, ZuptEstimatesGyroBias) {
  EkfCfg c;
  Ekf2d ekf(c);
  const double bias = 0.007;
  for (int k = 0; k < 40; k++) {
    for (int i = 0; i < 100; i++) ekf.predict(0, bias, 0.005, false);
    ekf.updateZupt(bias, c.zupt_sigma);
  }
  EXPECT_NEAR(ekf.bias(), bias, 5e-4);
  // Đã biết bias -> đứng yên thêm 10 s hướng gần như không trôi
  const double th0 = ekf.pose().th;
  for (int i = 0; i < 2000; i++) ekf.predict(0, bias, 0.005, false);
  EXPECT_NEAR(ekf.pose().th - th0, 0, deg2rad(0.5));
}

TEST(Ekf2d, WheelOmegaGateRejectsSlip) {
  EkfCfg c;
  Ekf2d ekf(c);
  EXPECT_TRUE(ekf.updateWheelOmega(0.10, 0.10, c.wheel_omega_sigma));
  EXPECT_FALSE(ekf.updateWheelOmega(2.0, 0.0, c.wheel_omega_sigma));  // bánh trượt: chênh 2 rad/s
}
