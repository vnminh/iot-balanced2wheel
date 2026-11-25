#include <gtest/gtest.h>

#include <filesystem>

#include "slam/planner.h"
#include "slam/traversed_grid.h"

using namespace slam;

namespace {
std::vector<Eigen::Vector2d> line(Eigen::Vector2d a, Eigen::Vector2d b, double step = 0.02) {
  std::vector<Eigen::Vector2d> o;
  const int n = int((b - a).norm() / step);
  for (int i = 0; i <= n; i++) o.push_back(a + (b - a) * (double(i) / n));
  return o;
}
}  // namespace

TEST(Grid, StraightLineCenterLayerWidth) {
  // Quỹ đạo thẳng 1 m dọc tâm ô -> center_layer rộng 2·tol + 1 ô = 3 ô; mọi ô khác bị chặn
  TraversedGrid g;
  g.build({line({0.025, 0.025}, {1.025, 0.025})}, 0.05, 0.10, 0.05, 0.5);
  int i, j;
  ASSERT_TRUE(g.toCell({0.5, 0.025}, i, j));
  int width = 0;
  for (int jj = 0; jj < g.ny; jj++) width += g.free(i, jj);
  EXPECT_EQ(width, 3);
  // footprint rộng hơn (bán kính thân 0,10 m -> 5 ô)
  int fw = 0;
  for (int jj = 0; jj < g.ny; jj++) fw += g.footprint.at<uchar>(jj, i) > 0;
  EXPECT_EQ(fw, 5);
  // Ô xa quỹ đạo bị chặn, tổng số ô trống ≈ 3 × 21 (+ đầu tròn)
  EXPECT_FALSE(g.free(0, 0));
  EXPECT_GE(g.freeCells(), 63u);
  EXPECT_LE(g.freeCells(), 70u);
}

TEST(Grid, MapServerRoundTrip) {
  TraversedGrid g;
  g.build({line({0, 0}, {1, 0}), line({1, 0}, {1, 1})}, 0.05, 0.10, 0.05, 0.5);
  const std::string dir = std::filesystem::temp_directory_path() / "slam_grid_test";
  std::filesystem::create_directories(dir);
  ASSERT_TRUE(g.saveMapServer(dir));
  TraversedGrid h;
  ASSERT_TRUE(h.loadMapServer(dir));
  EXPECT_EQ(h.nx, g.nx);
  EXPECT_EQ(h.ny, g.ny);
  EXPECT_NEAR(h.ox, g.ox, 1e-9);
  EXPECT_EQ(cv::countNonZero(h.center != g.center), 0);
}

TEST(Planner, LShapeStaysInTraversedArea) {
  TraversedGrid g;
  g.build({line({0, 0}, {2, 0}), line({2, 0}, {2, 1.5})}, 0.05, 0.10, 0.05, 0.5);
  PlannerCfg pc;
  const PlanResult r = planPath(g, {0, 0}, {2, 1.5}, pc, 0.05);
  ASSERT_TRUE(r.ok) << r.why;
  EXPECT_NEAR(r.length, 3.5, 0.15);   // không đi tắt chéo qua vùng chưa đi
  for (const auto &p : r.path) {
    int i, j;
    ASSERT_TRUE(g.toCell(p, i, j));
    EXPECT_TRUE(g.free(i, j));
  }
  for (size_t k = 1; k + 1 < r.waypoints.size(); k++)
    EXPECT_NEAR((r.waypoints[k] - r.waypoints[k - 1]).norm(), pc.waypoint_spacing, 0.08);
  EXPECT_LT((r.waypoints.back() - Eigen::Vector2d(2, 1.5)).norm(), 0.05);
}

TEST(Planner, GoalSnapAndRejects) {
  TraversedGrid g;
  g.build({line({0, 0}, {2, 0}), line({0, 2}, {2, 2})}, 0.05, 0.10, 0.05, 0.5);
  PlannerCfg pc;
  // Đích lệch 0,3 m khỏi đường đã đi -> dời vào và báo
  PlanResult r = planPath(g, {0, 0}, {1.0, 0.3}, pc, 0.05);
  ASSERT_TRUE(r.ok);
  EXPECT_NEAR(r.goal_snap, 0.25, 0.04);
  // Hai đường song song không nối nhau -> không có đường
  r = planPath(g, {0, 0}, {1.0, 2.0}, pc, 0.05);
  EXPECT_FALSE(r.ok);
  // Đích quá xa vùng đã đi
  r = planPath(g, {0, 0}, {1.0, 5.0}, pc, 0.05);
  EXPECT_FALSE(r.ok);
}
