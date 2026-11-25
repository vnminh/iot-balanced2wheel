#include "slam/config.h"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace slam {

double RobotCfg::metersPerStep() const { return M_PI * wheel_diameter / double(steps_per_rev * microsteps); }

namespace {

template <typename T>
void get(const YAML::Node &n, const char *key, T &v) {
  if (n && n[key]) v = n[key].as<T>();
}

}  // namespace

Config Config::defaults() {
  Config c;
  c.warnings = {"Đang dùng cấu hình mặc định (không có slam.yaml)"};
  return c;
}

Config Config::load(const std::string &path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Không mở được " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  Config c;
  c.source_text = ss.str();
  YAML::Node y;
  try {
    y = YAML::Load(c.source_text);
  } catch (const YAML::Exception &e) {
    throw std::runtime_error(path + ": " + e.what());
  }

  auto r = y["robot"];
  get(r, "wheel_diameter", c.robot.wheel_diameter);
  get(r, "wheel_base", c.robot.wheel_base);
  get(r, "steps_per_rev", c.robot.steps_per_rev);
  get(r, "microsteps", c.robot.microsteps);
  get(r, "body_radius", c.robot.body_radius);

  auto e = y["ekf"];
  get(e, "sigma_d_per_m", c.ekf.sigma_d_per_m);
  get(e, "sigma_lat_per_m", c.ekf.sigma_lat_per_m);
  get(e, "sigma_gyro", c.ekf.sigma_gyro);
  get(e, "sigma_bias_rw", c.ekf.sigma_bias_rw);
  get(e, "init_sigma_bias", c.ekf.init_sigma_bias);
  get(e, "sat_noise_mult", c.ekf.sat_noise_mult);
  get(e, "zupt_window", c.ekf.zupt_window);
  get(e, "zupt_disp", c.ekf.zupt_disp);
  get(e, "zupt_gyro", c.ekf.zupt_gyro);
  get(e, "zupt_pitch_rate", c.ekf.zupt_pitch_rate);
  get(e, "zupt_sigma", c.ekf.zupt_sigma);
  get(e, "use_wheel_omega", c.ekf.use_wheel_omega);
  get(e, "wheel_omega_window", c.ekf.wheel_omega_window);
  get(e, "wheel_omega_sigma", c.ekf.wheel_omega_sigma);
  get(e, "pitch_filter_tau", c.ekf.pitch_filter_tau);

  auto cam = y["camera"];
  get(cam, "width", c.camera.width);
  get(cam, "height", c.camera.height);
  get(cam, "fx", c.camera.fx);
  get(cam, "fy", c.camera.fy);
  get(cam, "cx", c.camera.cx);
  get(cam, "cy", c.camera.cy);
  get(cam, "dist", c.camera.dist);
  get(cam, "yaw_deg", c.camera.yaw_deg);
  get(cam, "mount_roll_deg", c.camera.mount_roll_deg);
  get(cam, "mount_pitch_deg", c.camera.mount_pitch_deg);
  get(cam, "offset_x", c.camera.offset_x);
  get(cam, "offset_y", c.camera.offset_y);
  get(cam, "height_above_axle", c.camera.height_above_axle);
  get(cam, "ceiling_height", c.camera.ceiling_height);
  get(cam, "max_pitch_deg", c.camera.max_pitch_deg);
  get(cam, "max_pitch_rate", c.camera.max_pitch_rate);
  get(cam, "max_yaw_rate", c.camera.max_yaw_rate);
  c.camera.dist.resize(5, 0.0);

  auto fe = y["frontend"];
  get(fe, "orb_features", c.frontend.orb_features);
  get(fe, "orb_fast_threshold", c.frontend.orb_fast_threshold);
  get(fe, "min_features", c.frontend.min_features);
  get(fe, "kf_min_dist", c.frontend.kf_min_dist);
  get(fe, "kf_min_angle_deg", c.frontend.kf_min_angle_deg);
  get(fe, "blind_kf_dist", c.frontend.blind_kf_dist);
  get(fe, "path_sample", c.frontend.path_sample);
  get(fe, "debug_jpg_period", c.frontend.debug_jpg_period);

  auto l = y["loop"];
  get(l, "search_radius", c.loop.search_radius);
  get(l, "min_kf_gap", c.loop.min_kf_gap);
  get(l, "neighbor_edges", c.loop.neighbor_edges);
  get(l, "max_candidates", c.loop.max_candidates);
  get(l, "ratio_test", c.loop.ratio_test);
  get(l, "ransac_px", c.loop.ransac_px);
  get(l, "min_inliers", c.loop.min_inliers);
  if (l && l["scale_range"]) {
    auto v = l["scale_range"].as<std::vector<double>>();
    if (v.size() == 2) c.loop.scale_min = v[0], c.loop.scale_max = v[1];
  }
  get(l, "max_pred_dist", c.loop.max_pred_dist);
  get(l, "max_pred_angle_deg", c.loop.max_pred_angle_deg);
  get(l, "neighbor_max_dist", c.loop.neighbor_max_dist);
  get(l, "neighbor_max_angle_deg", c.loop.neighbor_max_angle_deg);
  get(l, "sigma_xy", c.loop.sigma_xy);
  get(l, "sigma_th_deg", c.loop.sigma_th_deg);
  get(l, "huber_k", c.loop.huber_k);
  get(l, "reloc_period", c.loop.reloc_period);
  get(l, "reloc_confirm", c.loop.reloc_confirm);
  get(l, "reloc_agree_dist", c.loop.reloc_agree_dist);
  get(l, "reloc_agree_angle_deg", c.loop.reloc_agree_angle_deg);
  get(l, "reloc_timeout", c.loop.reloc_timeout);
  get(l, "loc_period", c.loop.loc_period);
  get(l, "loc_min_move", c.loop.loc_min_move);

  auto lc = y["localization"];
  get(lc, "max_map_sigma", c.loc.max_map_sigma);
  get(lc, "drift_per_m", c.loc.drift_per_m);
  get(lc, "drift_th_per_m_deg", c.loc.drift_th_per_m_deg);
  get(lc, "chi2_gate", c.loc.chi2_gate);

  auto g = y["grid"];
  get(g, "resolution", c.grid.resolution);
  get(g, "center_tolerance", c.grid.center_tolerance);
  get(g, "margin", c.grid.margin);

  auto p = y["planner"];
  get(p, "edge_weight", c.planner.edge_weight);
  get(p, "waypoint_spacing", c.planner.waypoint_spacing);
  get(p, "max_snap", c.planner.max_snap);

  auto ct = y["controller"];
  get(ct, "rate_hz", c.controller.rate_hz);
  get(ct, "v_max", c.controller.v_max);
  get(ct, "v_min", c.controller.v_min);
  get(ct, "w_max", c.controller.w_max);
  get(ct, "w_min_rotate", c.controller.w_min_rotate);
  get(ct, "acc_v", c.controller.acc_v);
  get(ct, "acc_w", c.controller.acc_w);
  get(ct, "lookahead", c.controller.lookahead);
  get(ct, "goal_tol_xy", c.controller.goal_tol_xy);
  get(ct, "goal_tol_theta_deg", c.controller.goal_tol_theta_deg);
  get(ct, "rotate_in_place_deg", c.controller.rotate_in_place_deg);
  get(ct, "rotate_exit_deg", c.controller.rotate_exit_deg);
  get(ct, "k_rot", c.controller.k_rot);
  get(ct, "k_v_goal", c.controller.k_v_goal);
  get(ct, "stuck_time", c.controller.stuck_time);
  get(ct, "progress_min", c.controller.progress_min);
  get(ct, "mcu_timeout", c.controller.mcu_timeout);
  get(ct, "max_pitch_deg", c.controller.max_pitch_deg);
  get(ct, "max_pitch_rate", c.controller.max_pitch_rate);

  auto q = y["quality"];
  get(q, "min_keyframes", c.quality.min_keyframes);
  get(q, "min_visual_ratio", c.quality.min_visual_ratio);
  get(q, "min_neighbor_ratio", c.quality.min_neighbor_ratio);
  get(q, "min_loops", c.quality.min_loops);

  // Giá trị TBD (mục 12 SLAM-DESIGN.md): báo khi chưa đánh dấu đã đo
  auto cal = y["calibrated"];
  auto flag = [&](const char *k) { return cal && cal[k] && cal[k].as<bool>(); };
  if (!flag("wheel_base"))
    c.warnings.push_back("wheel_base chưa đo (đang dùng " + std::to_string(c.robot.wheel_base) + " m)");
  if (!flag("camera_intrinsics")) c.warnings.push_back("Nội tham số camera chưa hiệu chỉnh (python3 -m robot.calibrate)");
  if (!flag("camera_mount")) c.warnings.push_back("Hướng gắn camera (camera.yaw_deg) chưa kiểm tra bằng calib_ref/calib_measure");
  if (c.camera.ceiling_height > 0 && !flag("ceiling_height"))
    c.warnings.push_back("Chiều cao trần chưa đo (đang dùng " + std::to_string(c.camera.ceiling_height) +
                         " m; đặt 0 để tự ước lượng)");
  return c;
}

}  // namespace slam
