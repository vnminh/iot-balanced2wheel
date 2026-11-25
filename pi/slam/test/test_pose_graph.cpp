#include <gtest/gtest.h>

#include <random>

#include "slam/pose_graph.h"

using namespace slam;

namespace {
// Vòng vuông 4 m × 4 m, mỗi cạnh 8 node; odometry có trôi góc + tỉ lệ
struct LoopData {
  std::vector<Pose2> truth, init;
  std::vector<GraphEdge> edges;
};

// Covariance odometry khớp với độ trôi thật (σθ 0,5°/bước, σxy 1 cm/bước) như EKF báo
LoopData makeLoop(double drift_th = deg2rad(0.3), double scale = 1.01) {
  LoopData d;
  Pose2 p;
  for (int side = 0; side < 4; side++)
    for (int k = 0; k < 8; k++) {
      d.truth.push_back(p);
      p = p * (k == 7 ? Pose2(0.5, 0, M_PI / 2) : Pose2(0.5, 0, 0));
    }
  d.truth.push_back(p);   // ≈ node 0
  d.init.push_back(d.truth[0]);
  Eigen::Matrix3d cov = Eigen::Vector3d(1e-4, 1e-4, deg2rad(0.5) * deg2rad(0.5)).asDiagonal();
  for (size_t i = 1; i < d.truth.size(); i++) {
    const Pose2 t = Pose2::between(d.truth[i - 1], d.truth[i]);
    const Pose2 m(t.x * scale, t.y * scale, t.th + drift_th);
    d.init.push_back(d.init.back() * m);
    d.edges.push_back({int(i - 1), int(i), m, cov, false});
  }
  return d;
}
}  // namespace

TEST(PoseGraph, LoopClosureRemovesDrift) {
  LoopData d = makeLoop();
  const int last = int(d.truth.size()) - 1;
  const double before = (d.init[last].t() - d.init[0].t()).norm();
  ASSERT_GT(before, 0.3);
  GraphEdge loop{0, last, Pose2(), Eigen::Vector3d(4e-4, 4e-4, 1e-4).asDiagonal(), true};
  d.edges.push_back(loop);
  auto x = d.init;
  const auto r = optimizePoseGraph(x, d.edges, {0}, 1.345);
  ASSERT_TRUE(r.ok);
  EXPECT_LT((x[last].t() - x[0].t()).norm(), 0.05);   // khép vòng < 5 cm (giai đoạn 5)
}

TEST(PoseGraph, OdometryOnlyMatchesChain) {
  LoopData d = makeLoop();
  auto x = d.init;
  for (auto &p : x) p = Pose2(p.x + 0.1, p.y - 0.2, p.th + 0.05);   // lệch đầu vào
  x[0] = d.init[0];
  optimizePoseGraph(x, d.edges, {0}, 1.345);
  for (size_t i = 0; i < x.size(); i++) EXPECT_LT((x[i].t() - d.init[i].t()).norm(), 1e-4);
}

TEST(PoseGraph, HuberResistsWrongLoop) {
  // Không trôi; thêm một loop đúng và một loop SAI 1 m ở giữa -> không bị kéo lệch quá 5 cm
  LoopData d = makeLoop(0.0, 1.0);
  const int last = int(d.truth.size()) - 1;
  d.edges.push_back({0, last, Pose2(), Eigen::Vector3d(9e-4, 9e-4, 7e-4).asDiagonal(), true});
  const Pose2 truth_rel = Pose2::between(d.truth[4], d.truth[20]);
  d.edges.push_back({4, 20, Pose2(truth_rel.x + 1.0, truth_rel.y, truth_rel.th),
                     Eigen::Vector3d(9e-4, 9e-4, 7e-4).asDiagonal(), true});
  // Chỉ Huber: cạnh sai bị giảm trọng số nhưng vẫn kéo lệch đôi chút; cạnh sai vẫn còn χ² lớn
  auto x = d.init;
  optimizePoseGraph(x, d.edges, {0}, 1.345);
  EXPECT_GT(edgeChi2(x, d.edges.back()), 100.0);
  // Huber + loại cạnh χ² lớn (cách SlamSystem làm): bỏ đúng cạnh sai, giữ cạnh đúng, lệch < 5 cm
  std::vector<bool> rejectable(d.edges.size(), false);
  rejectable[d.edges.size() - 1] = rejectable[d.edges.size() - 2] = true;
  std::vector<size_t> rej;
  x = d.init;
  ASSERT_TRUE(optimizeWithRejection(x, d.edges, {0}, 1.345, 25.0, rejectable, rej).ok);
  ASSERT_EQ(rej.size(), 1u);
  EXPECT_EQ(rej[0], d.edges.size() - 1);
  double worst = 0;
  for (size_t i = 0; i < x.size(); i++) worst = std::max(worst, (x[i].t() - d.truth[i].t()).norm());
  EXPECT_LT(worst, 0.05);
}
