#include "slam/frontend.h"

#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace slam {

namespace {
cv::Matx33d Rx(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {1, 0, 0, 0, c, -s, 0, s, c};
}
cv::Matx33d Ry(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {c, 0, s, 0, 1, 0, -s, 0, c};
}
cv::Matx33d Rz(double a) {
  const double c = std::cos(a), s = std::sin(a);
  return {c, -s, 0, s, c, 0, 0, 0, 1};
}
}  // namespace

CameraModel::CameraModel(const CameraCfg &c) : cfg(c) {
  R_z_phi = Rz(deg2rad(c.yaw_deg));
  R_b_c = R_z_phi * Rx(deg2rad(c.mount_roll_deg)) * Ry(deg2rad(c.mount_pitch_deg));
}

cv::Matx33d CameraModel::K(int width, int height) const {
  const double sx = double(width) / cfg.width, sy = double(height) / cfg.height;
  return {cfg.fx * sx, 0, cfg.cx * sx, 0, cfg.fy * sy, cfg.cy * sy, 0, 0, 1};
}

double CameraModel::f(int width) const { return 0.5 * (cfg.fx + cfg.fy) * double(width) / cfg.width; }

// Thân chúi tới góc p = quay quanh trục Y (trục bánh) của xe: R_level_body = Ry(p).
// r_level (trong khung camera ảo) = R_z_phiᵀ · Ry(p) · R_b_c · r_cam
cv::Vec3d CameraModel::level(const cv::Vec3d &r, double pitch) const {
  return R_z_phi.t() * (Ry(pitch) * (R_b_c * r));
}

cv::Vec3d CameraModel::unlevel(const cv::Vec3d &r, double pitch) const {
  return R_b_c.t() * (Ry(pitch).t() * (R_z_phi * r));
}

Pose2 CameraModel::T_b_c(double pitch) const {
  return Pose2(cfg.offset_x + cfg.height_above_axle * std::sin(pitch), cfg.offset_y, deg2rad(cfg.yaw_deg));
}

CeilingFrontend::CeilingFrontend(const Config &cfg) : cfg_(cfg), cam_(cfg.camera) {
  orb_ = cv::ORB::create(cfg.frontend.orb_features, 1.2f, 8, 19, 0, 2, cv::ORB::HARRIS_SCORE, 31,
                         cfg.frontend.orb_fast_threshold);
}

void CeilingFrontend::process(const cv::Mat &gray, double t, double pitch, FrameFeatures &out) {
  out = FrameFeatures();
  out.t = t;
  out.pitch = pitch;
  std::vector<cv::KeyPoint> kps;
  orb_->detectAndCompute(gray, cv::noArray(), kps, out.desc);
  if (kps.empty()) return;

  std::vector<cv::Point2f> px(kps.size()), nrm;
  for (size_t i = 0; i < kps.size(); i++) px[i] = kps[i].pt;
  const cv::Matx33d K = cam_.K(gray.cols, gray.rows);
  cv::undistortPoints(px, nrm, K, cam_.cfg.dist);
  const double f = cam_.f(gray.cols);

  out.pts.reserve(kps.size());
  out.raw.reserve(kps.size());
  cv::Mat keep_desc;
  std::vector<int> keep;
  keep.reserve(kps.size());
  for (size_t i = 0; i < nrm.size(); i++) {
    const cv::Vec3d r = cam_.level(cv::Vec3d(nrm[i].x, nrm[i].y, 1.0), pitch);
    if (r[2] < 0.2) continue;  // tia gần nằm ngang (không xảy ra với camera hướng lên)
    out.pts.emplace_back(float(f * r[0] / r[2]), float(f * r[1] / r[2]));
    out.raw.push_back(px[i]);
    keep.push_back(int(i));
  }
  if (keep.size() != kps.size()) {
    cv::Mat d(int(keep.size()), out.desc.cols, out.desc.type());
    for (size_t i = 0; i < keep.size(); i++) out.desc.row(keep[i]).copyTo(d.row(int(i)));
    out.desc = d;
  }
}

}  // namespace slam
