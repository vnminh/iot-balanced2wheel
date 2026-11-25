#pragma once
// M7: pose graph 2D bằng Ceres (thay GTSAM, xem SLAM-DESIGN.md 2.1).
#include <Eigen/Core>
#include <vector>

#include "slam/pose2.h"

namespace slam {

struct GraphEdge {
  int a = 0, b = 0;          // chỉ số node
  Pose2 meas;                // T_a_b đo được
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity();
  bool robust = false;       // bọc Huber (cạnh thị giác)
};

struct OptimizeResult {
  bool ok = false;
  int iterations = 0;
  double initial_cost = 0, final_cost = 0;
  double ms = 0;
};

// nodes: tư thế ban đầu (được cập nhật tại chỗ). fixed: chỉ số node giữ cố định (gốc bản đồ).
OptimizeResult optimizePoseGraph(std::vector<Pose2> &nodes, const std::vector<GraphEdge> &edges,
                                 const std::vector<int> &fixed, double huber_k, int max_iterations = 50);

// "Không ghép sai còn hơn ghép nhiều": tối ưu với Huber; cạnh được phép loại (rejectable) nào sau tối ưu vẫn
// có χ² > reject_chi2 thì bỏ cạnh tệ nhất và tối ưu lại từ đầu (lặp tới khi hết). rejected = chỉ số cạnh đã bỏ.
OptimizeResult optimizeWithRejection(std::vector<Pose2> &nodes, const std::vector<GraphEdge> &edges,
                                     const std::vector<int> &fixed, double huber_k, double reject_chi2,
                                     const std::vector<bool> &rejectable, std::vector<size_t> &rejected);

// Sai số đã chuẩn hoá (Mahalanobis) của một cạnh với tư thế hiện tại, để báo / loại cạnh xấu
double edgeChi2(const std::vector<Pose2> &nodes, const GraphEdge &e);

}  // namespace slam
