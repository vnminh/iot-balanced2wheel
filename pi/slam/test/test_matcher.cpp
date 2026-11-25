// Đổi ảnh trần -> chuyển động xe: dựng hai ảnh giả lập ở tư thế biết trước, so kết quả.
#include <gtest/gtest.h>

#include "sim_world.h"
#include "slam/frontend.h"
#include "slam/matcher.h"

using namespace slam;

namespace {

struct Case {
  Pose2 a, b;
  double pa_deg, pb_deg;
};

void checkCase(const Config &cfg, const Case &c, double tol_xy = 0.02, double tol_th_deg = 0.7) {
  sim::SimParams sp;
  sp.pitch_amp_deg = 0;
  sp.balance_amp = 0;
  sim::SimWorld w(cfg, sp, c.a);
  CeilingFrontend fe(cfg);
  // pitch đặt trực tiếp qua bước mô phỏng: dùng render với pitch 0 rồi so; pitch khác 0 kiểm tra ở test riêng
  const cv::Mat ia = w.render();
  w.teleport(c.b);
  const cv::Mat ib = w.render();
  FrameFeatures fa, fb;
  fe.process(ia, 0, 0, fa);
  fe.process(ib, 0, 0, fb);
  ASSERT_GT(fa.size(), 100u);
  const MatchResult r = matchFrames(fa, fb, cfg.loop);
  ASSERT_TRUE(r.ok) << r.why;
  const Pose2 m = baseMotion(fe.camera(), r.alpha, r.tx, r.ty, fe.camera().f(640), sp.ceiling_h, 0, 0);
  const Pose2 t = Pose2::between(c.a, c.b);
  EXPECT_NEAR(m.x, t.x, tol_xy) << "y=" << m.y << " th=" << rad2deg(m.th);
  EXPECT_NEAR(m.y, t.y, tol_xy);
  EXPECT_NEAR(rad2deg(wrapAngle(m.th - t.th)), 0.0, tol_th_deg);
}

}  // namespace

TEST(Matcher, PureForward) {
  Config cfg = Config::defaults();
  checkCase(cfg, {Pose2(0, 0, 0), Pose2(0.3, 0, 0), 0, 0});
}

TEST(Matcher, Lateral) {
  Config cfg = Config::defaults();
  checkCase(cfg, {Pose2(0, 0, 0), Pose2(0, 0.25, 0), 0, 0});
}

TEST(Matcher, RotateInPlace) {
  Config cfg = Config::defaults();
  checkCase(cfg, {Pose2(1, 0.5, 0.3), Pose2(1, 0.5, 0.3 + deg2rad(25)), 0, 0});
}

TEST(Matcher, MixedWithCameraOffsetAndYaw) {
  Config cfg = Config::defaults();
  cfg.camera.yaw_deg = 30;
  cfg.camera.offset_x = 0.05;
  cfg.camera.offset_y = -0.03;
  checkCase(cfg, {Pose2(-1, 2, -0.7), Pose2(-0.8, 2.1, -0.7 + deg2rad(-20)), 0, 0});
}

TEST(Matcher, PitchCompensation) {
  // Cùng một tư thế, thân chúi 6°: sau khi bù pitch hai ảnh phải trùng nhau (chuyển động ≈ 0)
  Config cfg = Config::defaults();
  sim::SimParams sp;
  sp.balance_amp = 0;
  sp.pitch_amp_deg = 6;
  sp.pitch_freq = 0.25;            // t = 1 s -> sin(π/2) = 1 -> pitch 6°
  sim::SimWorld w(cfg, sp, Pose2(0.5, 0.2, 0.4), 0.0);
  CeilingFrontend fe(cfg);
  w.step(0, 0, 1.0);
  const double p1 = w.pitch();
  const cv::Mat i1 = w.render();
  sim::SimParams sp0 = sp;
  sp0.pitch_amp_deg = 0;
  sim::SimWorld w0(cfg, sp0, w.truth(), 0.0);
  const cv::Mat i0 = w0.render();
  ASSERT_GT(std::fabs(p1), deg2rad(5));
  FrameFeatures f0, f1;
  fe.process(i0, 0, 0, f0);
  fe.process(i1, 0, p1, f1);
  const MatchResult r = matchFrames(f0, f1, cfg.loop);
  ASSERT_TRUE(r.ok) << r.why;
  const Pose2 m = baseMotion(fe.camera(), r.alpha, r.tx, r.ty, fe.camera().f(640), sp.ceiling_h, 0, p1);
  EXPECT_NEAR(m.x, 0, 0.02);
  EXPECT_NEAR(m.y, 0, 0.02);
  EXPECT_NEAR(rad2deg(m.th), 0, 0.5);
  EXPECT_NEAR(r.scale, 1.0, 0.02);
}

TEST(Matcher, CeilingHeightEstimate) {
  Config cfg = Config::defaults();
  CameraModel cam(cfg.camera);
  // t = (f/H)·R(−ψB)·(C_A − C_B): xe tiến 0,4 m, φ = 90°
  const double H = 2.7, f = 635;
  const Pose2 od(0.4, 0, 0);
  const Pose2 cA = cam.T_b_c(0), cB = od * cam.T_b_c(0);
  const Eigen::Vector2d d = cA.t() - cB.t();
  const double c = std::cos(-cB.th), s = std::sin(-cB.th);
  const double tx = f / H * (c * d.x() - s * d.y()), ty = f / H * (s * d.x() + c * d.y());
  EXPECT_NEAR(estimateCeilingHeight(cam, od, tx, ty, f, 0, 0), H, 1e-6);
  const Pose2 back = baseMotion(cam, 0, tx, ty, f, H, 0, 0);
  EXPECT_NEAR(back.x, 0.4, 1e-9);
  EXPECT_NEAR(back.y, 0.0, 1e-9);
}
