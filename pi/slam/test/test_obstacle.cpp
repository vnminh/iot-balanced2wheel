// Siêu âm + bản đồ vật cản log-odds + controller bị chặn.
#include <gtest/gtest.h>

#include <cmath>

#include "slam/controller.h"
#include "slam/obstacle_grid.h"
#include "slam/planner.h"
#include "slam/sonar.h"

using namespace slam;

namespace {
LogOddsParams params() {
  ObstacleCfg c;
  return LogOddsParams::from(c);
}

// Khoảng cách tới điểm gần nhất của đĩa (tâm c, bán kính r) trong chùm ±half
double rangeTo(const Pose2 &s, double half, const Eigen::Vector2d &c, double r) {
  double best = -1;
  for (int k = 0; k < 31; k++) {
    const double a = s.th - half + 2 * half * k / 30;
    const Eigen::Vector2d d(std::cos(a), std::sin(a)), o = s.t();
    const double b = d.dot(o - c), cc = (o - c).squaredNorm() - r * r, disc = b * b - cc;
    if (disc < 0) continue;
    const double t = -b - std::sqrt(disc);
    if (t > 0 && (best < 0 || t < best)) best = t;
  }
  return best;
}
}  // namespace

TEST(Sonar, GeometryParseAndClearance) {
  // 3 cảm biến như firmware mặc định
  const uint8_t raw[] = {1, 3, 0xD0, 0x07,
                         80, 0, 50, 0, 0xA0, 0x0F, 15, 120, 0,                // (80, 50) mm, +40°
                         90, 0, 0, 0, 0, 0, 15, 150, 0,                       // (90, 0), 0°
                         80, 0, 0xCE, 0xFF, 0x60, 0xF0, 15, 120, 0};          // (80, −50), −40°
  SonarGeometry g;
  ASSERT_TRUE(SonarGeometry::parse(raw, sizeof raw, g));
  ASSERT_EQ(g.sensors.size(), 3u);
  EXPECT_NEAR(g.max_range, 2.0, 1e-9);
  EXPECT_NEAR(g.sensors[0].mount.th, deg2rad(40), 1e-6);
  EXPECT_NEAR(g.sensors[2].mount.y, -0.05, 1e-9);
  EXPECT_NEAR(g.sensors[1].half, deg2rad(15), 1e-9);
  // Chỉ cảm biến giữa thấy vật 40 cm: khoảng trống ≈ 0,40·cos15°
  EXPECT_NEAR(frontClearance(g, {-1, 0.40, -1}, 0.15), 0.40 * std::cos(deg2rad(15)), 0.01);
  // Cảm biến bên thấy vật xa (1 m ở 25..55°): nằm ngoài hành lang thân xe -> không chặn
  EXPECT_TRUE(std::isinf(frontClearance(g, {1.0, -1, -1}, 0.15)));
  // Cảm biến bên thấy vật sát (15 cm): có điểm trong hành lang
  EXPECT_LT(frontClearance(g, {0.15, -1, -1}, 0.15), 0.15);
  EXPECT_FALSE(SonarGeometry::parse(raw, 10, g));   // thiếu byte
}

TEST(Obstacle, ConvergesOnTrueObjectFromSeveralPoses) {
  const LogOddsParams p = params();
  LogOddsGrid g(0.05, 0);
  const Eigen::Vector2d obj(1.5, 0.0);
  const double half = deg2rad(15);
  // Xe tiến lại gần và lệch dần (mỗi vị trí đã qua cổng dời / xoay)
  for (int k = 0; k < 12; k++) {
    const Pose2 s(0.1 * k, -0.4 + 0.07 * k, std::atan2(obj.y() - (-0.4 + 0.07 * k), obj.x() - 0.1 * k) + deg2rad(5 * ((k % 3) - 1)));
    g.integrate(s, half, rangeTo(s, half, obj, 0.05), 2.0, 0, p);
  }
  const auto occ = g.occupied(0, p.locc);
  ASSERT_FALSE(occ.empty());
  int near = 0;
  for (const auto &c : occ) near += (c - obj).norm() < 0.12;
  EXPECT_GE(near, 1);
  EXPECT_LE(int(occ.size()) - near, 2);   // gần như không có ô sai
}

TEST(Obstacle, SingleReadingIsOnlyCandidate) {
  const LogOddsParams p = params();
  LogOddsGrid g(0.05, 0);
  g.integrate(Pose2(0, 0, 0), deg2rad(15), 1.0, 2.0, 0, p);
  EXPECT_TRUE(g.occupied(0, p.locc).empty());   // một lần đo: cả cung chỉ là ứng viên
  EXPECT_GT(g.value(20, 0, 0), 0);              // ô trên cung ở trục: có bằng chứng
  EXPECT_LT(g.value(10, 0, 0), 0);              // ô bên trong nón: trống
}

TEST(Obstacle, TemporaryLayerForgets) {
  const LogOddsParams p = params();
  LogOddsGrid g(0.05, 25.0);
  for (int k = 0; k < 8; k++) g.integrate(Pose2(0, 0.01 * k, 0), deg2rad(15), 1.0, 2.0, 0, p);
  EXPECT_FALSE(g.occupied(0, p.locc).empty());
  EXPECT_TRUE(g.occupied(60, p.locc).empty());   // 60 s sau (τ 25 s): đã quên
}

TEST(Obstacle, NoEchoClearsOnlyNearby) {
  const LogOddsParams p = params();
  LogOddsGrid g(0.05, 0);
  g.integrate(Pose2(0, 0, 0), deg2rad(15), 0.0, 2.0, 0, p);   // không có vang
  EXPECT_LT(g.value(10, 0, 0), 0);    // 0,5 m: trống
  EXPECT_EQ(g.value(30, 0, 0), 0);    // 1,5 m: không biết (sóng có thể dội lệch)
}

TEST(Obstacle, MaskAndPathBlocked) {
  TraversedGrid tg;
  tg.build({{{0, 0}, {3, 0}}}, 0.05, 0.10, 0.05, 0.5);
  const std::vector<Eigen::Vector2d> obst{{1.5, 0.02}};
  const cv::Mat m = obstacleMask(tg, obst, 0.15);
  int i, j;
  ASSERT_TRUE(tg.toCell({1.5, 0.0}, i, j));
  EXPECT_EQ(m.at<uchar>(j, i), 255);
  ASSERT_TRUE(tg.toCell({1.3, 0.0}, i, j));
  EXPECT_EQ(m.at<uchar>(j, i), 0);
  EXPECT_NEAR(pathBlockedAt({{0, 0}, {1, 0}, {2, 0}}, obst, 0.15), 1.5, 0.03);   // vị trí chiếu của vật lên đường
  EXPECT_LT(pathBlockedAt({{0, 0}, {1, 0}}, obst, 0.15), 0);           // ngoài 1 m phía trước
  EXPECT_LT(pathBlockedAt({{2, 0}, {3, 0}}, obst, 0.15), 0);           // vật ở sau lưng
  EXPECT_LT(pathBlockedAt({{1, 0}, {1.4, 0}}, {{1.5, 0}}, 0.15, true), 0);   // vật sau đích cuối
}

TEST(Obstacle, AStarAvoidsMaskedCells) {
  // Vòng chữ nhật 3 × 2 m: chắn cạnh dưới thì A* phải đi vòng 3 cạnh kia
  TraversedGrid tg;
  tg.build({{{0, 0}, {3, 0}, {3, 2}, {0, 2}, {0, 0}}}, 0.05, 0.10, 0.05, 0.5);
  PlannerCfg pc;
  PlanResult direct = planPath(tg, {0, 0}, {3, 0}, pc, 0.05);
  ASSERT_TRUE(direct.ok);
  EXPECT_LT(direct.length, 3.3);
  TraversedGrid blocked = tg;
  blocked.center = tg.center.clone();
  blocked.center.setTo(0, obstacleMask(tg, {{1.5, 0.0}}, 0.15));
  PlanResult around = planPath(blocked, {0, 0}, {3, 0}, pc, 0.05, 0.3);
  ASSERT_TRUE(around.ok);
  EXPECT_GT(around.length, 6.5);
  for (const auto &q : around.path) EXPECT_GT((q - Eigen::Vector2d(1.5, 0)).norm(), 0.15);
}

TEST(Obstacle, StaticLayerFollowsKeyframe) {
  const LogOddsParams p = params();
  StaticObstacles st(0.05);
  Keyframe kf;
  kf.id = 0;
  kf.T_odom = Pose2(0, 0, 0);
  kf.T_map = Pose2(0, 0, 0);
  const Eigen::Vector2d obj(1.0, 0.0);
  for (int k = 0; k < 10; k++) {
    const Pose2 s(0.05 * k, -0.3 + 0.06 * k, std::atan2(obj.y() + 0.3 - 0.06 * k, obj.x() - 0.05 * k));
    SonarObs o;
    o.kf = 0;
    o.rx = float(s.x), o.ry = float(s.y), o.rth = float(s.th);
    o.half = float(deg2rad(15));
    o.d = float(rangeTo(s, deg2rad(15), obj, 0.05));
    st.add(o, kf, p);
  }
  auto centroid = [&](const std::vector<Eigen::Vector2d> &v) -> Eigen::Vector2d {
    Eigen::Vector2d c(0, 0);
    for (const auto &q : v) c += q;
    return c / double(std::max<size_t>(1, v.size()));
  };
  const auto before = st.occupied(p.locc);
  ASSERT_FALSE(before.empty());
  // Tối ưu pose graph dời keyframe 0,3 m sang trái: vật cản dời theo
  std::vector<Keyframe> kfs{kf};
  kfs[0].T_map = Pose2(0, 0.3, 0);
  st.rebuild(kfs, p);
  const auto after = st.occupied(p.locc);
  ASSERT_FALSE(after.empty());
  EXPECT_NEAR((centroid(after) - centroid(before)).y(), 0.3, 0.06);
  // Tẩy
  st.erase(centroid(after), 0.3, p);
  EXPECT_TRUE(st.occupied(p.locc).empty());
}

TEST(ControllerBlocked, ScanWaitReplanResume) {
  ControllerCfg cfg;
  Controller c(cfg);
  std::vector<Eigen::Vector2d> wps;
  for (int k = 1; k <= 20; k++) wps.emplace_back(0.15 * k, 0);
  c.start(wps, std::nullopt, 0);
  Pose2 p;
  double t = 0;
  const double dt = 0.05;
  auto step = [&](bool blocked, double clear) {
    ControlInput in;
    in.t = t;
    in.odom = p;
    in.localized = in.balancing = true;
    in.path_blocked = blocked;
    in.clear_front = clear;
    const ControlOutput o = c.tick(in, dt);
    const double thm = p.th + 0.5 * o.w * dt;
    p = Pose2(p.x + o.v * dt * std::cos(thm), p.y + o.v * dt * std::sin(thm), p.th + o.w * dt);
    t += dt;
    return o;
  };
  for (int k = 0; k < 40; k++) step(false, 1e9);
  EXPECT_EQ(c.state(), "following");
  // Giảm tốc theo khoảng trống (có giới hạn gia tốc nên cần vài chu kỳ)
  ControlOutput o;
  for (int k = 0; k < 10; k++) o = step(false, 0.30);
  EXPECT_LT(o.v, cfg.v_max * 0.5);
  // Đường bị chặn -> blocked, quét ±25°
  step(true, 0.5);
  EXPECT_EQ(c.state(), "blocked");
  double max_th = -1, min_th = 1;
  bool req = false;
  for (int k = 0; k < 200 && !req; k++) {
    step(true, 0.5);
    max_th = std::max(max_th, p.th), min_th = std::min(min_th, p.th);
    req = c.replanRequested();
  }
  EXPECT_TRUE(req);
  EXPECT_GT(rad2deg(max_th), 20);
  EXPECT_LT(rad2deg(min_th), -20);
  EXPECT_GE(t, 2.0 + cfg.block_wait_s - 0.1);
  c.replanFailed(t + cfg.replan_retry_s);
  EXPECT_FALSE(c.replanRequested());
  // Hết chặn -> đi tiếp
  for (int k = 0; k < 5; k++) step(false, 1e9);
  EXPECT_EQ(c.state(), "following");
  // Siêu âm giữ xe đứng > 1 s cũng là bị chặn
  for (int k = 0; k < 40 && c.state() != "blocked"; k++) step(false, 0.10);
  EXPECT_EQ(c.state(), "blocked");
}
