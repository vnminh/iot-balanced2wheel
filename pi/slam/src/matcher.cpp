#include "slam/matcher.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>

namespace slam {

MatchResult matchFrames(const FrameFeatures &A, const FrameFeatures &B, const LoopCfg &cfg) {
  MatchResult res;
  if (int(A.size()) < cfg.min_inliers || int(B.size()) < cfg.min_inliers) {
    res.why = "ít đặc trưng";
    return res;
  }
  cv::BFMatcher bf(cv::NORM_HAMMING);
  std::vector<std::vector<cv::DMatch>> knn;
  bf.knnMatch(A.desc, B.desc, knn, 2);
  std::vector<cv::Point2f> pa, pb;
  for (const auto &m : knn) {
    if (m.size() == 2 && m[0].distance < cfg.ratio_test * m[1].distance) {
      pa.push_back(A.pts[m[0].queryIdx]);
      pb.push_back(B.pts[m[0].trainIdx]);
    }
  }
  res.matches = int(pa.size());
  if (res.matches < cfg.min_inliers) {
    res.why = "ít cặp khớp (" + std::to_string(res.matches) + ")";
    return res;
  }
  std::vector<uchar> inl;
  cv::Mat M = cv::estimateAffinePartial2D(pa, pb, inl, cv::RANSAC, cfg.ransac_px, 2000, 0.995, 10);
  if (M.empty()) {
    res.why = "RANSAC thất bại";
    return res;
  }
  res.inliers = cv::countNonZero(inl);
  const double a = M.at<double>(0, 0), b = M.at<double>(1, 0);
  res.scale = std::hypot(a, b);
  res.alpha = std::atan2(b, a);
  res.tx = M.at<double>(0, 2);
  res.ty = M.at<double>(1, 2);
  if (res.inliers < cfg.min_inliers) {
    res.why = "ít inlier (" + std::to_string(res.inliers) + ")";
    return res;
  }
  if (res.scale < cfg.scale_min || res.scale > cfg.scale_max) {
    res.why = "tỉ lệ " + std::to_string(res.scale).substr(0, 5) + " ngoài khoảng";
    return res;
  }
  res.ok = true;
  return res;
}

Pose2 cameraMotion(double alpha, double tx, double ty, double f, double H) {
  // d = −(H/f)·R(−α)·t
  const double c = std::cos(-alpha), s = std::sin(-alpha);
  const double k = -H / f;
  return Pose2(k * (c * tx - s * ty), k * (s * tx + c * ty), -alpha);
}

Pose2 baseMotion(const CameraModel &cam, double alpha, double tx, double ty, double f, double H, double pitchA,
                 double pitchB) {
  return cam.T_b_c(pitchA) * cameraMotion(alpha, tx, ty, f, H) * cam.T_b_c(pitchB).inverse();
}

double estimateCeilingHeight(const CameraModel &cam, const Pose2 &odom, double tx, double ty, double f,
                             double pitchA, double pitchB) {
  const Pose2 c = cam.T_b_c(pitchA).inverse() * odom * cam.T_b_c(pitchB);
  const double d = std::hypot(c.x, c.y), tp = std::hypot(tx, ty);
  if (d < 0.15 || tp < 20.0) return -1;
  return f * d / tp;
}

}  // namespace slam
