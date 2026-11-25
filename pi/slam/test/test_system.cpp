// Test tích hợp trên thế giới giả lập (mục 9): vòng kín + odometry trôi -> khép vòng; lưu / tải bản đồ.
#include <gtest/gtest.h>

#include <filesystem>

#include "sim_world.h"
#include "slam/map_io.h"
#include "slam/system.h"

using namespace slam;

TEST(MapIO, RoundTrip) {
  MapData m;
  for (int i = 0; i < 3; i++) {
    Keyframe k;
    k.id = i;
    k.stamp = 10 + i;
    k.T_odom = Pose2(i * 0.3, 0.01 * i, 0.1 * i);
    k.T_map = Pose2(i * 0.31, 0.02, 0.1 * i + 0.01);
    k.pitch = 0.02;
    k.session = i == 2 ? 1 : 0;
    k.pts = {{1.5f, -2.5f}, {3.f, 4.f}};
    k.desc = cv::Mat(2, 32, CV_8U, cv::Scalar(i + 7));
    k.path_rel = {Pose2(0.1, 0, 0), Pose2(0.2, 0.01, 0.02)};
    m.kfs.push_back(k);
  }
  m.odom.push_back({0, 1, Pose2(0.3, 0, 0.1), Eigen::Matrix3d::Identity() * 1e-4});
  m.vis.push_back({0, 2, 0.1, 5.0, -3.0, 635, 77, EdgeKind::Loop});
  m.places.push_back({"bếp", 1.0, 2.0, 0.5, true});
  m.places.push_back({"sạc", 0, 0, 0, false});
  m.height_est = 2.41;
  const std::string dir = std::filesystem::temp_directory_path() / "slam_mapio";
  std::filesystem::remove_all(dir);
  std::string err;
  ASSERT_TRUE(saveMap(dir, "t1", m, TraversedGrid(), "x: 1\n", "{\"name\":\"t\"}", err)) << err;
  MapData r;
  ASSERT_TRUE(loadMap(dir + "/t1", r, err)) << err;
  ASSERT_EQ(r.kfs.size(), 3u);
  EXPECT_NEAR(r.kfs[2].T_map.x, 0.62, 1e-12);
  EXPECT_EQ(r.kfs[2].session, 1);
  EXPECT_EQ(cv::countNonZero(r.kfs[1].desc != m.kfs[1].desc), 0);
  EXPECT_EQ(r.kfs[1].path_rel.size(), 2u);
  ASSERT_EQ(r.vis.size(), 1u);
  EXPECT_EQ(r.vis[0].inliers, 77);
  EXPECT_EQ(r.vis[0].kind, EdgeKind::Loop);
  ASSERT_EQ(r.places.size(), 2u);
  EXPECT_EQ(r.places[0].name, "bếp");
  EXPECT_TRUE(r.places[0].has_th);
  EXPECT_FALSE(r.places[1].has_th);
  EXPECT_NEAR(r.height_est, 2.41, 1e-9);
}

TEST(System, MappingLoopClosesAndStuckWheelDetected) {
  Config cfg = Config::defaults();
  cfg.camera.ceiling_height = 0;   // tự ước lượng
  sim::SimParams sp;
  sp.stuck = {{100.0 + 12.0, 100.0 + 13.5}};
  SlamSystem::Options opt;
  opt.synchronous = true;
  opt.maps_dir = std::filesystem::temp_directory_path() / "slam_sys_maps";
  SlamSystem sys(cfg, opt, nullptr);
  sim::SimWorld w(cfg, sp);
  sim::SimRunner r(sys, w);
  r.run(0, 0, 1.0);
  r.command("new_map");
  for (int k = 0; k < 4; k++) {
    r.forward(k % 2 ? 1.5 : 2.5);
    r.rotate(M_PI / 2);
  }
  r.forward(1.0);
  const Pose2 tr = w.truth(), mp = sys.mapPose(), od = sys.odomPose();
  EXPECT_GE(sys.edgeCount(EdgeKind::Loop), 1);
  EXPECT_GE(sys.slipCount(), 1);
  EXPECT_NEAR(sys.heightUsed(), sp.ceiling_h, 0.1);
  EXPECT_LT(std::hypot(mp.x - tr.x, mp.y - tr.y), 0.10)
      << "map " << mp.x << "," << mp.y << " thật " << tr.x << "," << tr.y << " odom " << od.x << "," << od.y;
  // Keyframe sau tối ưu so với vị trí thật (gốc map = keyframe đầu ≈ vị trí thật ban đầu)
  double sum = 0, worst = 0, worst_th = 0;
  const auto kfs = sys.keyframes();
  for (const auto &k : kfs) {
    const Pose2 t = r.truthAt(k.stamp);
    const double e = std::hypot(k.T_map.x - t.x, k.T_map.y - t.y);
    sum += e;
    worst = std::max(worst, e);
    worst_th = std::max(worst_th, std::fabs(rad2deg(wrapAngle(k.T_map.th - t.th))));
  }
  EXPECT_LT(sum / kfs.size(), 0.05);
  EXPECT_LT(worst, 0.10);
  EXPECT_LT(worst_th, 3.0);
}
