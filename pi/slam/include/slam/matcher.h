#pragma once
// M6 (phần hình học): khớp hai khung ảnh trần -> chuyển động tương đối của xe.
//
// estimateAffinePartial2D trên toạ độ đã chỉnh (gốc = tâm ảnh) cho q_B = s·R(α)·q_A + t. Khi đó
//     Δψ = −α,      vị trí camera B trong khung camera A: d = −(H/f)·R(−α)·t
//     T_bA_bB = T_b_c(pitch_A) ⊕ (d, −α) ⊕ T_b_c(pitch_B)⁻¹
#include <string>

#include "slam/config.h"
#include "slam/frontend.h"
#include "slam/keyframe.h"

namespace slam {

struct MatchResult {
  bool ok = false;
  int matches = 0, inliers = 0;
  double alpha = 0, tx = 0, ty = 0, scale = 1;
  std::string why;
};

MatchResult matchFrames(const FrameFeatures &A, const FrameFeatures &B, const LoopCfg &cfg);

// Chuyển động camera ảo B trong khung camera ảo A (m, rad)
Pose2 cameraMotion(double alpha, double tx, double ty, double f, double H);
// Chuyển động xe (khung xe A -> khung xe B)
Pose2 baseMotion(const CameraModel &cam, double alpha, double tx, double ty, double f, double H, double pitchA,
                 double pitchB);
// Ước lượng H từ odometry: H = f·|d_odom| / |t|. Trả về <= 0 nếu không đủ chuyển động.
double estimateCeilingHeight(const CameraModel &cam, const Pose2 &odom_bA_bB, double tx, double ty, double f,
                             double pitchA, double pitchB);

}  // namespace slam
