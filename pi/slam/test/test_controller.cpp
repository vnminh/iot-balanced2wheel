// Controller pure pursuit trên xe lý tưởng (odometry = thật): đường có quay đầu + điểm nhiệm vụ phải chạm.
#include <gtest/gtest.h>

#include "slam/controller.h"

using namespace slam;

namespace {
struct DriveResult {
  bool arrived = false;
  std::vector<double> min_d;   // khoảng cách gần nhất tới từng điểm cần kiểm
};

DriveResult drive(Controller &c, const std::vector<Eigen::Vector2d> &check, Pose2 p = Pose2()) {
  DriveResult r;
  r.min_d.assign(check.size(), 1e9);
  const double dt = 0.05;
  for (double t = 0; t < 200; t += dt) {
    ControlInput in;
    in.t = t;
    in.odom = p;
    in.localized = in.balancing = true;
    const ControlOutput o = c.tick(in, dt);
    if (!c.active()) break;
    const double thm = p.th + 0.5 * o.w * dt;
    p = Pose2(p.x + o.v * dt * std::cos(thm), p.y + o.v * dt * std::sin(thm), p.th + o.w * dt);
    for (size_t k = 0; k < check.size(); k++) r.min_d[k] = std::min(r.min_d[k], (p.t() - check[k]).norm());
  }
  r.arrived = c.state() == "arrived";
  return r;
}

std::vector<Eigen::Vector2d> line(Eigen::Vector2d a, Eigen::Vector2d b, double step = 0.15) {
  std::vector<Eigen::Vector2d> o;
  const int n = std::max(1, int(std::round((b - a).norm() / step)));
  for (int i = 1; i <= n; i++) o.push_back(a + (b - a) * (double(i) / n));
  return o;
}
}  // namespace

TEST(Controller, UTurnAndMissionStops) {
  ControllerCfg cfg;
  Controller c(cfg);
  // (0,0) -> (2,0) -> quay đầu về (0.5,0) -> rẽ trái lên (0.5,1)
  std::vector<Eigen::Vector2d> w = line({0, 0}, {2, 0});
  const size_t s1 = w.size() - 1;
  for (auto &q : line({2, 0}, {0.5, 0})) w.push_back(q);
  const size_t s2 = w.size() - 1;
  for (auto &q : line({0.5, 0}, {0.5, 1})) w.push_back(q);
  c.start(w, std::nullopt, 0, {s1, s2});
  const DriveResult r = drive(c, {{2, 0}, {0.5, 0}, {0.5, 1}});
  ASSERT_TRUE(r.arrived) << c.reason();
  EXPECT_LT(r.min_d[0], cfg.goal_tol_xy);   // chạm điểm quay đầu, không cắt ngắn
  EXPECT_LT(r.min_d[1], cfg.goal_tol_xy);
  EXPECT_LT(r.min_d[2], cfg.goal_tol_xy);
}

TEST(Controller, SafetyAborts) {
  ControllerCfg cfg;
  Controller c(cfg);
  c.start(line({0, 0}, {1, 0}), std::nullopt, 0);
  ControlInput in;
  in.localized = true;
  in.balancing = false;   // xe ngã
  c.tick(in, 0.05);
  EXPECT_EQ(c.state(), "aborted");
  c.start(line({0, 0}, {1, 0}), std::nullopt, 0);
  in.balancing = true;
  in.mcu_age = 0.5;       // mất dữ liệu ESP32
  c.tick(in, 0.05);
  EXPECT_EQ(c.state(), "aborted");
  // Không tiến triển (bánh kẹt): xe đứng yên mãi
  c.start(line({0, 0}, {1, 0}), std::nullopt, 0);
  in.mcu_age = 0;
  for (double t = 0; t < cfg.stuck_time + 1; t += 0.05) {
    in.t = t;
    c.tick(in, 0.05);
  }
  EXPECT_EQ(c.state(), "aborted");
}
