#pragma once
// M5: keyframe + các cạnh của pose graph.
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <vector>

#include "slam/frontend.h"
#include "slam/pose2.h"

namespace slam {

struct Keyframe {
  int id = 0;
  double stamp = 0;
  Pose2 T_odom;                   // pose EKF lúc tạo (bất biến)
  Pose2 T_map;                    // pose trong map (đổi sau mỗi lần tối ưu)
  double pitch = 0;
  bool blind = false;             // không đủ đặc trưng: chỉ có odometry
  int session = 0;                // lần chạy tạo ra keyframe (mở rộng bản đồ = session mới)
  std::vector<cv::Point2f> pts;   // toạ độ đã chỉnh (xem frontend.h)
  cv::Mat desc;
  std::vector<Pose2> path_rel;    // quỹ đạo từ keyframe này tới keyframe sau, tương đối với keyframe này

  FrameFeatures features() const {
    FrameFeatures f;
    f.t = stamp;
    f.pitch = pitch;
    f.pts = pts;
    f.desc = desc;
    return f;
  }
};

struct OdomEdge {
  int a = 0, b = 0;
  Pose2 meas;                     // inverse(T_odom_a) ⊕ T_odom_b
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1e-4;
};

enum class EdgeKind : uint8_t { Neighbor = 0, Loop = 1, Reloc = 2 };

// Cạnh thị giác lưu dạng pixel; đổi ra mét lúc dựng đồ thị (chiều cao trần có thể đổi khi tự ước lượng)
struct VisualEdge {
  int a = 0, b = 0;
  double alpha = 0, tx = 0, ty = 0;   // q_b = s·R(α)·q_a + t
  double f = 635;                     // tiêu cự (px) của toạ độ đã chỉnh
  int inliers = 0;
  EdgeKind kind = EdgeKind::Neighbor;
};

}  // namespace slam
