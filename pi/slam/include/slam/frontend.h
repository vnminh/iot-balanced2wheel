#pragma once
// M4: xử lý ảnh trần. ORB trên ảnh gốc, rồi undistort + bù pitch TOẠ ĐỘ keypoint (không warp cả ảnh).
//
// Toạ độ "đã chỉnh" q (pixel, gốc = tâm ảnh) là ảnh của một camera ảo nằm ngang tuyệt đối, trục quang
// hướng thẳng lên, hướng trục x ảnh = φ so với trục x xe. Mọi điểm trần P thoả
//     q = (f/H) · R(−ψ) · (P − C)        ψ = θ_xe + φ, C = vị trí camera, H = trần − camera
// nên hai ảnh liên hệ bằng một phép đồng dạng 2D (xem matcher.h).
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#include "slam/config.h"
#include "slam/pose2.h"

namespace slam {

struct FrameFeatures {
  double t = 0;
  double pitch = 0;                  // rad lúc chụp
  std::vector<cv::Point2f> pts;      // toạ độ đã chỉnh q (px, gốc tâm ảnh)
  std::vector<cv::Point2f> raw;      // toạ độ gốc trên ảnh (chỉ để vẽ debug)
  cv::Mat desc;                      // ORB, CV_8U N×32
  size_t size() const { return pts.size(); }
};

// Hình học camera dùng chung cho frontend, matcher, simulator
struct CameraModel {
  explicit CameraModel(const CameraCfg &c);
  // Ma trận K đã co giãn theo kích thước ảnh thực tế
  cv::Matx33d K(int width, int height) const;
  double f(int width) const;         // tiêu cự trung bình (px) ở độ rộng ảnh này
  // Tia camera thật (chuẩn hoá z=1) -> tia camera ảo nằm ngang, theo pitch thân xe
  cv::Vec3d level(const cv::Vec3d &r_cam, double pitch) const;
  cv::Vec3d unlevel(const cv::Vec3d &r_level, double pitch) const;  // ngược lại (simulator)
  // Tư thế camera ảo trong khung xe (phụ thuộc pitch vì camera cao hơn trục bánh)
  Pose2 T_b_c(double pitch) const;

  CameraCfg cfg;
  cv::Matx33d R_b_c;    // camera thật trong khung xe (thân thẳng)
  cv::Matx33d R_z_phi;  // camera ảo trong khung xe
};

class CeilingFrontend {
public:
  explicit CeilingFrontend(const Config &cfg);
  void process(const cv::Mat &gray, double t, double pitch, FrameFeatures &out);
  const CameraModel &camera() const { return cam_; }

private:
  Config cfg_;
  CameraModel cam_;
  cv::Ptr<cv::ORB> orb_;
};

}  // namespace slam
