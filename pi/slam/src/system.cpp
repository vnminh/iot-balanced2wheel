#include "slam/system.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

#include "slam/json.h"
#include "slam/planner.h"
#include "slam/pose_graph.h"

namespace fs = std::filesystem;

namespace slam {

namespace {

double nowMs() {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string poseJson(const Pose2 &p) { return "[" + jnum(p.x) + "," + jnum(p.y) + "," + jnum(p.th, 4) + "]"; }

std::string restOf(std::istringstream &ss) {
  std::string r;
  std::getline(ss, r);
  const auto b = r.find_first_not_of(" \t");
  if (b == std::string::npos) return "";
  const auto e = r.find_last_not_of(" \t\r\n");
  return r.substr(b, e - b + 1);
}

const char *kindName(EdgeKind k) {
  switch (k) {
    case EdgeKind::Neighbor: return "lân cận";
    case EdgeKind::Loop: return "loop";
    default: return "tái định vị";
  }
}

// Tách JSON tối giản: lấy giá trị chuỗi "key":"..." (đủ cho meta.json do chính ta ghi)
std::string jsonField(const std::string &js, const std::string &key) {
  const std::string k = "\"" + key + "\":\"";
  auto p = js.find(k);
  if (p == std::string::npos) return "";
  p += k.size();
  std::string out;
  for (; p < js.size() && js[p] != '"'; p++) {
    if (js[p] == '\\' && p + 1 < js.size()) p++;
    out += js[p];
  }
  return out;
}

}  // namespace

// =====================================================================================================
SlamSystem::SlamSystem(const Config &cfg, const Options &opt, Emit emit)
    : cfg_(cfg),
      opt_(opt),
      emit_raw_(std::move(emit)),
      cam_(cfg.camera),
      fe_(cfg),
      wo_(cfg.robot.metersPerStep(), cfg.robot.wheel_base),
      pe_(cfg.ekf.pitch_filter_tau),
      ekf_(cfg.ekf),
      ctrl_(cfg.controller),
      lop_(LogOddsParams::from(cfg.obstacles)),
      temp_(cfg.obstacles.resolution, cfg.obstacles.temp_tau),
      static_(cfg.obstacles.resolution) {
  if (!opt_.shm_dir.empty()) {
    std::error_code ec;
    fs::create_directories(opt_.shm_dir, ec);
  }
  if (!opt_.synchronous) worker_ = std::thread(&SlamSystem::workerLoop, this);
  for (const auto &w : cfg_.warnings) event("warn", w);
}

SlamSystem::~SlamSystem() {
  {
    std::lock_guard<std::mutex> lk(jm_);
    stop_ = true;
  }
  jcv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

// ----------------------------------------------------------------------------------------- phát JSON
void SlamSystem::emit(const std::string &s) {
  std::lock_guard<std::mutex> lk(emit_m_);
  if (emit_raw_) emit_raw_(s);
}

void SlamSystem::event(const std::string &level, const std::string &msg) {
  emit("{\"type\":\"event\",\"level\":" + jstr(level) + ",\"msg\":" + jstr(msg) + "}");
}

void SlamSystem::reply(const std::string &cmd, bool ok, const std::string &msg, const std::string &extra) {
  emit("{\"type\":\"reply\",\"cmd\":" + jstr(cmd) + ",\"ok\":" + jbool(ok) + ",\"msg\":" + jstr(msg) +
       (extra.empty() ? "" : "," + extra) + "}");
}

// ------------------------------------------------------------------------------------- truy cập
Pose2 SlamSystem::mapPose() const {
  std::lock_guard<std::mutex> lk(m_);
  return T_map_odom_ * ekf_.pose();
}
bool SlamSystem::localized() const {
  std::lock_guard<std::mutex> lk(m_);
  return localized_;
}
Mode SlamSystem::mode() const {
  std::lock_guard<std::mutex> lk(m_);
  return mode_;
}
std::vector<Keyframe> SlamSystem::keyframes() const {
  std::lock_guard<std::mutex> lk(m_);
  return kfs_;
}
int SlamSystem::edgeCount(EdgeKind k) const {
  std::lock_guard<std::mutex> lk(m_);
  return int(std::count_if(vis_edges_.begin(), vis_edges_.end(), [&](const VisualEdge &e) { return e.kind == k; }));
}
int SlamSystem::slipCount() const {
  std::lock_guard<std::mutex> lk(m_);
  return slips_;
}
double SlamSystem::heightUsed() const {
  std::lock_guard<std::mutex> lk(m_);
  return heightUsedLocked();
}
std::vector<Place> SlamSystem::places() const {
  std::lock_guard<std::mutex> lk(m_);
  return places_;
}

double SlamSystem::heightUsedLocked() const {
  if (cfg_.camera.ceiling_height > 0) return cfg_.camera.ceiling_height;
  if (h_samples_.size() >= 5) {
    std::vector<double> v = h_samples_;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }
  return loaded_height_ > 0 ? loaded_height_ : 2.30;
}

Eigen::Matrix3d SlamSystem::processNoiseLocked(const OdomSnap &now) const {
  // T_map_odom = T_map_base ⊕ T_odom_base⁻¹ trôi đúng bằng phần sai số odometry tích luỹ thêm từ lần sửa trước.
  // Sai số tuyệt đối (δx, δy, δθ) của odom tại (x, y) = nhiễu trái quanh gốc odom: (δx + δθ·y, δy − δθ·x, δθ),
  // rồi quay sang frame map.
  Eigen::Matrix3d dP = now.P - mo_snap_.P;
  Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; i++) D(i, i) = std::max(0.0, dP(i, i));
  Eigen::Matrix3d L = Eigen::Matrix3d::Identity();
  L(0, 2) = now.odom.y;
  L(1, 2) = -now.odom.x;
  const Eigen::Matrix3d M = rot3(T_map_odom_.th);
  Eigen::Matrix3d Q = M * L * D * L.transpose() * M.transpose();
  // Thêm trượt bánh không mô hình hoá được, theo quãng đường
  const double d = std::max(0.0, now.dist - mo_snap_.dist);
  const double dth = deg2rad(cfg_.loc.drift_th_per_m_deg);
  Q(0, 0) += cfg_.loc.drift_per_m * cfg_.loc.drift_per_m * d;
  Q(1, 1) += cfg_.loc.drift_per_m * cfg_.loc.drift_per_m * d;
  Q(2, 2) += dth * dth * d;
  return Q;
}

Eigen::Vector2d SlamSystem::mapSigmaLocked(const OdomSnap &now) const {
  // Độ bất định của VỊ TRÍ XE trong map (T_map_base = T_map_odom ⊕ T_odom_base), không phải của T_map_odom:
  //  - sai số T_map_odom: sai số góc quay quanh gốc odom -> ở vị trí xe thành sai số (−δθ·dy, δθ·dx)
  //  - odometry trôi thêm từ lần sửa trước: quay sang frame map
  const Pose2 mb = T_map_odom_ * now.odom;
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J(0, 2) = -(mb.y - T_map_odom_.y);
  J(1, 2) = mb.x - T_map_odom_.x;
  Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
  const Eigen::Matrix3d dP = now.P - mo_snap_.P;
  for (int i = 0; i < 3; i++) D(i, i) = std::max(0.0, dP(i, i));
  const double d = std::max(0.0, now.dist - mo_snap_.dist);
  const double dth = deg2rad(cfg_.loc.drift_th_per_m_deg);
  D(0, 0) += cfg_.loc.drift_per_m * cfg_.loc.drift_per_m * d;
  D(1, 1) += cfg_.loc.drift_per_m * cfg_.loc.drift_per_m * d;
  D(2, 2) += dth * dth * d;
  const Eigen::Matrix3d M = rot3(T_map_odom_.th);
  const Eigen::Matrix3d P = J * P_mo_ * J.transpose() + M * D * M.transpose();
  return {std::sqrt(0.5 * (P(0, 0) + P(1, 1))), std::sqrt(P(2, 2))};
}

Eigen::Matrix3d SlamSystem::visualCov(int inliers) const {
  const double s = std::sqrt(std::max(1.0, double(cfg_.loop.min_inliers) / std::max(1, inliers)));
  const double sxy = cfg_.loop.sigma_xy * s, sth = deg2rad(cfg_.loop.sigma_th_deg) * s;
  return Eigen::Vector3d(sxy * sxy, sxy * sxy, sth * sth).asDiagonal();
}

GraphEdge SlamSystem::toGraphEdge(const VisualEdge &e) const {
  GraphEdge g;
  g.a = e.a;
  g.b = e.b;
  g.meas = baseMotion(cam_, e.alpha, e.tx, e.ty, e.f, heightUsedLocked(), kfs_[e.a].pitch, kfs_[e.b].pitch);
  g.cov = visualCov(e.inliers);
  g.robust = true;
  return g;
}

// ===================================================================================== LUỒNG CHÍNH
void SlamSystem::onMcu(const McuRecord &m) {
  const double dt = have_mcu_ ? m.t - last_mcu_.t : 0.0;
  if (have_mcu_ && dt < 0) return;  // mẫu cũ / trùng
  pitch_ = pe_.update(m, dt);
  pitch_rate_ = pe_.rate();
  yaw_rate_ = worldYawRate(m.gyro, pitch_);
  if (rel_reset_.exchange(false)) ekf_.resetRelative();

  OdomDelta od;
  if (wo_.update(m.t, m.steps, od)) {
    const bool sat = m.flags & FLAG_SATURATED;
    ekf_.predict(od.d, yaw_rate_, od.dt, sat);
    if (cfg_.ekf.use_wheel_omega) {
      wo_dth_ += od.dth_wheel;
      wo_gyro_ += yaw_rate_ * od.dt;
      wo_dt_ += od.dt;
      if (wo_dt_ >= cfg_.ekf.wheel_omega_window) {
        if (!sat && (m.flags & FLAG_MOTORS)) {
          if (ekf_.updateWheelOmega(wo_dth_ / wo_dt_, wo_gyro_ / wo_dt_, cfg_.ekf.wheel_omega_sigma))
            ++wo_ok_;
          else
            ++wo_rej_;
        }
        wo_dth_ = wo_gyro_ = wo_dt_ = 0;
      }
    }
  }

  // ZUPT: xe không dời chỗ trong cửa sổ. Bánh vẫn đưa tới lui để cân bằng, nên so vị trí TRUNG BÌNH
  // nửa đầu với nửa sau cửa sổ (không so mẫu đầu với mẫu cuối), và giới hạn biên độ dao động.
  zwin_.push_back({m.t, m.steps[0], m.steps[1], yaw_rate_, pitch_rate_});
  while (!zwin_.empty() && zwin_.front().t < m.t - cfg_.ekf.zupt_window) zwin_.pop_front();
  if (zwin_.size() > 20 && zwin_.front().t <= m.t - 0.95 * cfg_.ekf.zupt_window && m.t - last_zupt_t_ > 0.25) {
    const double mps = cfg_.robot.metersPerStep();
    const size_t n = zwin_.size(), h = n / 2;
    double l1 = 0, l2 = 0, r1 = 0, r2 = 0, gsum = 0, pmax = 0;
    int32_t lmin = zwin_[0].sl, lmax = lmin, rmin = zwin_[0].sr, rmax = rmin;
    for (size_t i = 0; i < n; i++) {
      const ZSample &z = zwin_[i];
      (i < h ? l1 : l2) += z.sl;
      (i < h ? r1 : r2) += z.sr;
      lmin = std::min(lmin, z.sl), lmax = std::max(lmax, z.sl);
      rmin = std::min(rmin, z.sr), rmax = std::max(rmax, z.sr);
      gsum += z.gz;
      pmax = std::max(pmax, std::fabs(z.prate));
    }
    const double dl = std::fabs(l2 / (n - h) - l1 / h) * mps, dr = std::fabs(r2 / (n - h) - r1 / h) * mps;
    const double gmean = gsum / n;
    // xoay chậm tại chỗ: hai bánh ngược chiều -> chênh lệch tích luỹ (R − L) tăng; dao động cân bằng thì không
    const double rot_disp = std::fabs(double(zwin_.back().sr - zwin_.front().sr) - double(zwin_.back().sl - zwin_.front().sl)) * mps;
    const bool still = dl < cfg_.ekf.zupt_disp && dr < cfg_.ekf.zupt_disp && rot_disp < 0.002 &&
                       (lmax - lmin) * mps < 0.04 &&
                       (rmax - rmin) * mps < 0.04 && std::fabs(gmean) < cfg_.ekf.zupt_gyro &&
                       pmax < cfg_.ekf.zupt_pitch_rate;
    if (still) {
      ekf_.updateZupt(gmean, cfg_.ekf.zupt_sigma);
      last_zupt_t_ = m.t;
      ++zupt_count_;
    }
  }

  t_ = m.t;
  last_mcu_ = m;
  have_mcu_ = true;
  const Pose2 odom = ekf_.pose();
  hist_.push_back({t_, odom, pitch_, pitch_rate_, yaw_rate_});
  while (hist_.size() > 2 && hist_.front().t < t_ - 3.0) hist_.pop_front();

  if (!has_path_sample_ || std::hypot(odom.x - last_path_sample_.x, odom.y - last_path_sample_.y) >=
                               cfg_.frontend.path_sample) {
    has_path_sample_ = true;
    last_path_sample_ = odom;
    std::lock_guard<std::mutex> lk(m_);
    path_pending_.emplace_back(t_, odom);
    if (path_pending_.size() > 50000) path_pending_.erase(path_pending_.begin(), path_pending_.begin() + 10000);
  }

  if (pend_ && t_ >= pend_t_) handlePendingImage();
  maybeBlindKeyframe();

  const double cdt = 1.0 / cfg_.controller.rate_hz;
  if (t_ >= next_ctrl_t_) {
    next_ctrl_t_ = (t_ - next_ctrl_t_ > 1.0) ? t_ + cdt : next_ctrl_t_ + cdt;
    controlTick(cdt);
  }
  if (t_ >= next_state_t_) {
    next_state_t_ = (t_ - next_state_t_ > 1.0) ? t_ + 0.1 : next_state_t_ + 0.1;
    emitState();
  }
}

void SlamSystem::onImage(double t, const cv::Mat &gray) {
  ++frames_rx_;
  last_img_t_ = t;
  last_img_ = gray;
  pend_ = true;            // ảnh mới thay ảnh đang chờ (luôn xử lý ảnh mới nhất)
  pend_t_ = t;
  pend_img_ = gray;
  if (have_mcu_ && t_ >= pend_t_) handlePendingImage();
}

SlamSystem::Hist SlamSystem::interp(double t) const {
  if (hist_.empty()) return {t, ekf_.pose(), pitch_, pitch_rate_, yaw_rate_};
  if (t <= hist_.front().t) return hist_.front();
  if (t >= hist_.back().t) return hist_.back();
  auto it = std::lower_bound(hist_.begin(), hist_.end(), t, [](const Hist &h, double v) { return h.t < v; });
  const Hist &b = *it, &a = *(it - 1);
  const double u = (t - a.t) / std::max(1e-9, b.t - a.t);
  return {t, Pose2::lerp(a.odom, b.odom, u), a.pitch + u * (b.pitch - a.pitch),
          a.pitch_rate + u * (b.pitch_rate - a.pitch_rate), a.yaw_rate + u * (b.yaw_rate - a.yaw_rate)};
}

SlamSystem::Job SlamSystem::baseJob(Job::Type type) const {
  Job j;
  j.type = type;
  j.t = t_;
  j.odom = ekf_.pose();
  j.pitch = pitch_;
  j.relcov = ekf_.relCov();
  j.dist = ekf_.distance();
  j.podom = ekf_.P().topLeftCorner<3, 3>();
  return j;
}

void SlamSystem::handlePendingImage() {
  pend_ = false;
  cv::Mat img = pend_img_;
  pend_img_.release();
  const Hist h = interp(pend_t_);
  // Khung xấu: thân chúi quá nhiều / đang giật / quay nhanh (nhoè)
  if (std::fabs(h.pitch) > deg2rad(cfg_.camera.max_pitch_deg) || std::fabs(h.pitch_rate) > cfg_.camera.max_pitch_rate ||
      std::fabs(h.yaw_rate) > cfg_.camera.max_yaw_rate) {
    ++frames_skipped_motion_;
    return;
  }
  if (img_inflight_) {
    ++frames_dropped_busy_;
    return;
  }

  Mode mode;
  bool loc, has_kf;
  Pose2 last_kf;
  Eigen::Vector2d sig;
  {
    std::lock_guard<std::mutex> lk(m_);
    mode = mode_;
    has_kf = has_last_kf_;
    last_kf = last_kf_odom_;
    sig = mapSigmaLocked(curSnap());
    if (mode_ == Mode::Localization && localized_ && sig.x() > cfg_.loc.max_map_sigma) {
      localized_ = false;
      hyp_valid_ = false;
      emit("{\"type\":\"event\",\"level\":\"warn\",\"msg\":\"Mất định vị (độ bất định lớn): đang tái định vị\"}");
    }
    loc = localized_;
  }

  Job j;
  j.t = pend_t_;
  j.gray = img;
  j.odom = h.odom;
  j.pitch = h.pitch;
  j.relcov = ekf_.relCov();
  j.dist = ekf_.distance();
  j.podom = ekf_.P().topLeftCorner<3, 3>();
  j.image_job = true;
  if (mode == Mode::Mapping && loc) {
    const Pose2 d = Pose2::between(last_kf, h.odom);
    if (!has_kf || std::hypot(d.x, d.y) >= cfg_.frontend.kf_min_dist ||
        std::fabs(d.th) >= deg2rad(cfg_.frontend.kf_min_angle_deg)) {
      j.type = Job::KEYFRAME;
      submit(std::move(j));
    }
  } else if (!loc) {
    if (pend_t_ - last_reloc_job_t_ >= cfg_.loop.reloc_period) {
      last_reloc_job_t_ = pend_t_;
      j.type = Job::LOCALIZE;
      submit(std::move(j));
    }
  } else if (pend_t_ - last_loc_job_t_ >= cfg_.loop.loc_period &&
             (ekf_.distance() - last_loc_job_dist_ >= cfg_.loop.loc_min_move ||
              std::fabs(wrapAngle(h.odom.th - last_loc_job_odom_.th)) >= deg2rad(10) || pend_t_ - last_loc_job_t_ > 2.0)) {
    last_loc_job_t_ = pend_t_;
    last_loc_job_dist_ = ekf_.distance();
    last_loc_job_odom_ = h.odom;
    j.type = Job::LOCALIZE;
    submit(std::move(j));
  }
}

void SlamSystem::maybeBlindKeyframe() {
  // Không có ảnh (camera tắt / hỏng) vẫn dựng quỹ đạo + grid từ odometry
  if (t_ - last_img_t_ < 1.0 || img_inflight_) return;
  bool need;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (mode_ != Mode::Mapping || !localized_) return;
    const Pose2 d = Pose2::between(last_kf_odom_, ekf_.pose());
    need = !has_last_kf_ || std::hypot(d.x, d.y) >= cfg_.frontend.blind_kf_dist;
  }
  if (need) {
    Job j = baseJob(Job::KEYFRAME);
    j.image_job = true;
    submit(std::move(j));
  }
}

void SlamSystem::controlTick(double dt) {
  ControlInput in;
  in.t = t_;
  in.odom = ekf_.pose();
  {
    std::lock_guard<std::mutex> lk(m_);
    in.T_map_odom = T_map_odom_;
    in.localized = localized_ && mapSigmaLocked(curSnap()).x() <= cfg_.loc.max_map_sigma;
  }
  in.balancing = last_mcu_.state == MCU_BALANCING;
  in.pitch = pitch_;
  in.pitch_rate = pitch_rate_;
  in.mcu_age = 0;
  in.clear_front = clearFront();
  const bool was = ctrl_.active();
  if (was) in.path_blocked = pathBlockedNow(in.T_map_odom * in.odom);
  const std::string prev_state = ctrl_.state();
  cmd_ = ctrl_.tick(in, dt);
  if (ctrl_.state() == "blocked" && prev_state != "blocked")
    event("warn", ctrl_.blockCause() + ": dừng, quét ±" + jnum(cfg_.controller.scan_deg, 0) + "°, chờ " +
                      jnum(cfg_.controller.block_wait_s, 0) + " s");
  if (prev_state == "blocked" && ctrl_.state() == "following") event("info", "Hết vật cản, đi tiếp đường cũ");
  if (ctrl_.replanRequested()) replanFromHere();
  if (t_ - last_obst_emit_t_ >= 0.5) emitObstacles();
  if (mission_pts_.size() > 1) {
    const int leg = leg_offset_ + currentLeg();
    if (leg > last_leg_ && leg < int(mission_pts_.size())) event("info", "Qua điểm " + std::to_string(leg) + "/" +
                                                                         std::to_string(mission_pts_.size()));
    last_leg_ = std::max(last_leg_, leg);
  }
  if (cmd_.active) {
    emitCmd(cmd_);
  } else if (was) {
    emitCmd(cmd_);
    if (ctrl_.state() == "arrived")
      event("info", "Đã tới " + nav_label_);
    else
      event("warn", "Dừng goto: " + ctrl_.reason());
  }
}

void SlamSystem::wallTick(double now) {
  if (!ctrl_.active() || !have_mcu_) return;
  if (now - t_ > cfg_.controller.mcu_timeout) {
    char b[80];
    std::snprintf(b, sizeof b, "Mất dữ liệu từ ESP32 (%.2f s)", now - t_);
    ctrl_.cancel(b);
    cmd_ = ControlOutput();
    emitCmd(cmd_);
    event("warn", std::string("Dừng goto: ") + b);
  }
}

void SlamSystem::emitCmd(const ControlOutput &c) {
  const double t = -c.w * cfg_.robot.wheel_base / 2.0;  // firmware: t = chênh lệch mỗi bánh, + = quay phải
  emit("{\"type\":\"cmd\",\"v\":" + jnum(c.v, 4) + ",\"w\":" + jnum(c.w, 4) + ",\"t\":" + jnum(t, 4) +
       ",\"active\":" + jbool(c.active) + "}");
}

void SlamSystem::emitPath() {
  std::string s = "{\"type\":\"path\",\"label\":" + jstr(nav_label_) + ",\"pts\":[";
  const auto &w = ctrl_.waypoints();
  for (size_t i = 0; i < w.size(); i++) s += (i ? ",[" : "[") + jnum(w[i].x()) + "," + jnum(w[i].y()) + "]";
  s += "],\"mission\":[";
  for (size_t i = 0; i < mission_pts_.size(); i++)
    s += (i ? ",[" : "[") + jnum(mission_pts_[i].x()) + "," + jnum(mission_pts_[i].y()) + "]";
  emit(s + "]}");
}

int SlamSystem::currentLeg() const {
  // chặng đang đi (0-based): chặng đầu tiên có waypoint cuối chưa bị vượt
  if (leg_ends_.empty()) return 0;
  if (ctrl_.state() == "arrived") return int(leg_ends_.size());
  for (size_t k = 0; k < leg_ends_.size(); k++)
    if (ctrl_.index() <= leg_ends_[k]) return int(k);   // chỉ số chỉ vượt điểm nhiệm vụ khi đã chạm
  return int(leg_ends_.size()) - 1;
}

// Lập đường A* qua lần lượt các điểm (mỗi chặng bắt đầu từ đích chặng trước) rồi cho controller bám liền một mạch
void SlamSystem::startRoute(const std::vector<Eigen::Vector2d> &goals, std::optional<double> theta,
                            const std::string &label, const char *cmd) {
  if (!have_mcu_ || last_mcu_.state != MCU_BALANCING) {
    reply(cmd, false, "Xe chưa cân bằng: dựng xe lên trước");
    return;
  }
  TraversedGrid g;
  Pose2 start;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (!localized_) {
      reply(cmd, false, mode_ == Mode::Localization ? "Chưa tái định vị trên bản đồ: xoay xe / đẩy tới chỗ quen"
                                                    : "Chưa định vị");
      return;
    }
    if (mapSigmaLocked(curSnap()).x() > cfg_.loc.max_map_sigma) {
      reply(cmd, false, "Độ bất định vị trí quá lớn");
      return;
    }
    g = grid_;
    start = T_map_odom_ * ekf_.pose();
  }
  Route rt;
  std::string why;
  int bad = 0;
  if (!planRoute(g, start, goals, -1, rt, why, bad)) {
    reply(cmd, false, goals.size() > 1 ? "Điểm " + std::to_string(bad + 1) + ": " + why : why);
    return;
  }
  nav_label_ = label;
  mission_pts_ = goals.size() > 1 ? rt.snapped : std::vector<Eigen::Vector2d>();
  leg_ends_ = rt.ends;
  last_leg_ = 0;
  route_goals_ = goals;
  route_theta_ = theta;
  leg_offset_ = 0;
  ctrl_.start(rt.wps, theta, t_, std::vector<size_t>(rt.ends.begin(), rt.ends.end() - 1));
  emitPath();
  std::string msg = (goals.size() > 1 ? "Bắt đầu " : "Đi tới ") + label + ": " + jnum(rt.length, 2) + " m";
  if (rt.max_snap > 0.02) msg += " (điểm dời tối đa " + jnum(rt.max_snap * 100, 0) + " cm vào vùng đã đi)";
  reply(cmd, true, msg,
        "\"goal\":[" + jnum(rt.snapped.back().x()) + "," + jnum(rt.snapped.back().y()) + "],\"length\":" +
            jnum(rt.length, 2) + ",\"snap\":" + jnum(rt.max_snap, 3));
  event("info", msg);
}

bool SlamSystem::planRoute(const TraversedGrid &g0, const Pose2 &start, const std::vector<Eigen::Vector2d> &goals,
                           double start_snap, Route &r, std::string &why, int &bad_leg) {
  // Vùng đi được = vùng đã đi trừ ô vật cản (tĩnh + tạm) đã phình
  TraversedGrid g = g0;
  const std::vector<Eigen::Vector2d> &obst = obstaclePointsMap();
  if (!g.empty() && !obst.empty()) {
    g.center = g0.center.clone();
    g.center.setTo(0, obstacleMask(g, obst, cfg_.obstacles.inflate));
  }
  r = Route();
  Eigen::Vector2d from = start.t();
  for (size_t k = 0; k < goals.size(); k++) {
    const PlanResult p = planPath(g, from, goals[k], cfg_.planner, cfg_.grid.center_tolerance, k == 0 ? start_snap : -1);
    if (!p.ok) {
      why = p.why;
      bad_leg = int(k);
      return false;
    }
    for (const auto &w : p.waypoints)
      if (r.wps.empty() || (w - r.wps.back()).norm() > 1e-6) r.wps.push_back(w);
    if (r.wps.empty()) r.wps.push_back(p.goal);
    r.ends.push_back(r.wps.size() - 1);
    r.snapped.push_back(p.goal);
    r.length += p.length;
    r.max_snap = std::max(r.max_snap, p.goal_snap);
    from = p.goal;
  }
  return true;
}

void SlamSystem::replanFromHere() {
  const int done = leg_offset_ + currentLeg();   // số chặng đã xong (tính cả các lần lập lại trước)
  std::vector<Eigen::Vector2d> goals(route_goals_.begin() + std::min<size_t>(done, route_goals_.size()),
                                     route_goals_.end());
  if (goals.empty()) {
    ctrl_.replanFailed(t_ + cfg_.controller.replan_retry_s);
    return;
  }
  TraversedGrid g;
  Pose2 start;
  {
    std::lock_guard<std::mutex> lk(m_);
    g = grid_;
    start = T_map_odom_ * ekf_.pose();
  }
  obst_cache_t_ = -1e9;   // lấy vật cản mới nhất
  Route rt;
  std::string why;
  int bad = 0;
  if (!planRoute(g, start, goals, cfg_.obstacles.replan_start_snap, rt, why, bad)) {
    ctrl_.replanFailed(t_ + cfg_.controller.replan_retry_s);
    if (why != last_replan_why_ || t_ - last_replan_warn_t_ > 10) {
      event("warn", "Chưa có đường vòng: " + why + " (thử lại sau " + jnum(cfg_.controller.replan_retry_s, 0) + " s)");
      last_replan_why_ = why;
      last_replan_warn_t_ = t_;
    }
    return;
  }
  leg_offset_ = done;
  leg_ends_ = rt.ends;
  ctrl_.start(rt.wps, route_theta_, t_, std::vector<size_t>(rt.ends.begin(), rt.ends.end() - 1));
  ++replans_;
  last_replan_why_.clear();
  emitPath();
  event("info", "Đã tìm đường khác tránh vật cản: " + jnum(rt.length, 2) + " m");
}

// ------------------------------------------------------------------------------------- siêu âm / vật cản
void SlamSystem::onSonarGeom(const uint8_t *d, size_t n) {
  SonarGeometry g;
  if (!SonarGeometry::parse(d, n, g)) return;
  bool same = g.sensors.size() == sonar_geom_.sensors.size() && g.max_range == sonar_geom_.max_range;
  for (size_t k = 0; same && k < g.sensors.size(); k++)
    same = std::fabs(g.sensors[k].mount.x - sonar_geom_.sensors[k].mount.x) < 1e-6 &&
           std::fabs(g.sensors[k].mount.y - sonar_geom_.sensors[k].mount.y) < 1e-6 &&
           std::fabs(g.sensors[k].mount.th - sonar_geom_.sensors[k].mount.th) < 1e-6 &&
           std::fabs(g.sensors[k].half - sonar_geom_.sensors[k].half) < 1e-6;
  if (same) return;
  sonar_geom_ = g;
  sonar_last_.assign(g.sensors.size(), SonarLast());
  event("info", "Siêu âm: " + std::to_string(g.sensors.size()) + " cảm biến (bảng vị trí từ ESP32), tầm " +
                    jnum(g.max_range, 1) + " m");
}

void SlamSystem::onSonar(const SonarRecord &r) {
  if (temp_reset_.exchange(false)) temp_.clear();
  if (!cfg_.sonar.enable || r.idx >= sonar_geom_.sensors.size()) return;
  SonarLast &sl = sonar_last_[r.idx];
  sl.t = r.t;
  sl.mm = r.mm;
  if (r.mm == SONAR_FAULT || !have_mcu_) return;
  if (std::fabs(pitch_) > deg2rad(cfg_.sonar.max_pitch_deg)) return;   // chùm có thể đang quét sàn
  const SonarSensor &ss = sonar_geom_.sensors[r.idx];
  const Pose2 sensor = interp(r.t).odom * ss.mount;
  // Cổng: số đo khi cảm biến đứng yên lặp lại đúng một cung, không độc lập -> chỉ nhận khi đã dời / xoay
  if (sl.has_int) {
    const Pose2 dlt = Pose2::between(sl.last_int, sensor);
    if (std::hypot(dlt.x, dlt.y) < cfg_.sonar.gate_move && std::fabs(dlt.th) < deg2rad(cfg_.sonar.gate_turn_deg)) return;
  }
  sl.has_int = true;
  sl.last_int = sensor;
  const double d = r.mm == SONAR_NO_ECHO ? 0.0 : r.mm / 1000.0;
  temp_.integrate(sensor, ss.half, d, sonar_geom_.max_range, r.t, lop_);
  obst_cache_t_ = -1e9;
  if (cfg_.obstacles.static_layer) {
    std::lock_guard<std::mutex> lk(m_);
    if (mode_ == Mode::Mapping && localized_ && has_last_kf_ && last_kf_id_ >= 0 && last_kf_id_ < int(kfs_.size())) {
      const Pose2 rel = last_kf_odom_.inverse() * sensor;
      SonarObs o;
      o.kf = last_kf_id_;
      o.rx = float(rel.x), o.ry = float(rel.y), o.rth = float(rel.th);
      o.half = float(ss.half);
      o.d = float(d);
      o.max_range = float(sonar_geom_.max_range);
      static_.add(o, kfs_[last_kf_id_], lop_);
    }
  }
  if (r.t - last_prune_t_ > 2.0) {
    last_prune_t_ = r.t;
    temp_.prune(ekf_.pose().t(), cfg_.obstacles.temp_window, r.t);
  }
}

double SlamSystem::clearFront() const {
  if (!sonar_geom_.valid()) return 1e9;
  std::vector<double> d(sonar_geom_.sensors.size(), -1);
  for (size_t k = 0; k < d.size() && k < sonar_last_.size(); k++) {
    const auto &sl = sonar_last_[k];
    if (t_ - sl.t <= cfg_.sonar.fresh_s && sl.mm != SONAR_FAULT && sl.mm != SONAR_NO_ECHO) d[k] = sl.mm / 1000.0;
  }
  const double c = frontClearance(sonar_geom_, d, cfg_.sonar.corridor);
  return std::isfinite(c) ? c : 1e9;
}

std::vector<Eigen::Vector2d> SlamSystem::obstaclesStatic() const {
  std::lock_guard<std::mutex> lk(m_);
  return static_.occupied(lop_.locc);
}

std::vector<Eigen::Vector2d> SlamSystem::obstaclesTemp() const {
  Pose2 mo;
  {
    std::lock_guard<std::mutex> lk(m_);
    mo = T_map_odom_;
  }
  std::vector<Eigen::Vector2d> out = temp_.occupied(t_, lop_.locc);
  for (auto &p : out) p = mo * p;
  return out;
}

size_t SlamSystem::staticObsCount() const {
  std::lock_guard<std::mutex> lk(m_);
  return static_.obs().size();
}

const std::vector<Eigen::Vector2d> &SlamSystem::obstaclePointsMap() {
  if (t_ - obst_cache_t_ < 0.2) return obst_cache_;
  obst_cache_ = obstaclesStatic();
  const auto tmp = obstaclesTemp();
  obst_cache_.insert(obst_cache_.end(), tmp.begin(), tmp.end());
  obst_cache_t_ = t_;
  return obst_cache_;
}

bool SlamSystem::pathBlockedNow(const Pose2 &robot_map) {
  const auto &wps = ctrl_.waypoints();
  if (wps.empty() || ctrl_.state() == "final_rotate") return false;
  const auto &obst = obstaclePointsMap();
  if (obst.empty()) return false;
  std::vector<Eigen::Vector2d> poly{robot_map.t()};
  double len = 0;
  size_t k = std::min(ctrl_.index(), wps.size() - 1);
  for (; k < wps.size() && len < cfg_.obstacles.check_ahead; k++) {
    len += (wps[k] - poly.back()).norm();
    poly.push_back(wps[k]);
  }
  return pathBlockedAt(poly, obst, cfg_.obstacles.inflate, k == wps.size()) >= 0;
}

void SlamSystem::emitObstacles() {
  last_obst_emit_t_ = t_;
  const auto st = obstaclesStatic();
  const auto tp = obstaclesTemp();
  if (st.empty() && tp.empty() && !obst_emitted_nonempty_) return;
  obst_emitted_nonempty_ = !(st.empty() && tp.empty());
  auto arr = [](const std::vector<Eigen::Vector2d> &v) {
    std::string s = "[";
    const size_t n = std::min<size_t>(v.size(), 4000);
    for (size_t i = 0; i < n; i++) s += (i ? ",[" : "[") + jnum(v[i].x(), 3) + "," + jnum(v[i].y(), 3) + "]";
    return s + "]";
  };
  emit("{\"type\":\"obstacles\",\"res\":" + jnum(cfg_.obstacles.resolution, 3) + ",\"inflate\":" +
       jnum(cfg_.obstacles.inflate, 3) + ",\"static\":" + arr(st) + ",\"temp\":" + arr(tp) + "}");
}

void SlamSystem::command(const std::string &line) {
  std::istringstream ss(line);
  std::string cmd;
  ss >> cmd;
  if (cmd.empty()) return;
  if (cmd == "goto") {
    double x, y;
    if (!(ss >> x >> y) || !std::isfinite(x) || !std::isfinite(y)) {
      reply(cmd, false, "Toạ độ không hợp lệ");
      return;
    }
    std::optional<double> th;
    double thd;
    if (ss >> thd && std::isfinite(thd)) th = deg2rad(thd);
    char lb[64];
    std::snprintf(lb, sizeof lb, "(%.2f, %.2f)", x, y);
    startRoute({{x, y}}, th, lb, "goto");
  } else if (cmd == "goto_place") {
    const std::string name = restOf(ss);
    std::optional<Place> pl;
    {
      std::lock_guard<std::mutex> lk(m_);
      for (const auto &p : places_)
        if (p.name == name) pl = p;
    }
    if (!pl) {
      reply(cmd, false, "Không có địa điểm \"" + name + "\"");
      return;
    }
    startRoute({{pl->x, pl->y}}, pl->has_th ? std::optional<double>(pl->th) : std::nullopt, "\"" + name + "\"",
               "goto");
  } else if (cmd == "mission") {
    // mission x1 y1 x2 y2 ...: đi lần lượt qua các điểm, vận tốc / tốc độ quay cố định theo controller
    std::vector<Eigen::Vector2d> pts;
    double x, y;
    while (ss >> x >> y) {
      if (!std::isfinite(x) || !std::isfinite(y)) break;
      pts.emplace_back(x, y);
    }
    if (pts.empty() || pts.size() > 100) {
      reply(cmd, false, "Nhiệm vụ cần 1–100 điểm");
      return;
    }
    startRoute(pts, std::nullopt, "nhiệm vụ " + std::to_string(pts.size()) + " điểm", "mission");
  } else if (cmd == "cancel") {
    std::string why = restOf(ss);
    if (ctrl_.active()) {
      ctrl_.cancel(why.empty() ? "Đã huỷ" : why);
      cmd_ = ControlOutput();
      emitCmd(cmd_);
      event("info", "Dừng goto: " + ctrl_.reason());
    }
  } else if (cmd == "nav_clear") {
    if (ctrl_.active()) {
      cmd_ = ControlOutput();
      emitCmd(cmd_);
    }
    ctrl_.clear();
    nav_label_.clear();
    mission_pts_.clear();
    leg_ends_.clear();
    route_goals_.clear();
    leg_offset_ = 0;
    emitPath();
  } else if (cmd == "obst_erase") {
    double x, y, r;
    if (!(ss >> x >> y >> r) || !std::isfinite(x) || !std::isfinite(y) || !(r > 0 && r < 2)) {
      reply(cmd, false, "Cần: obst_erase x y r");
      return;
    }
    {
      std::lock_guard<std::mutex> lk(m_);
      static_.erase({x, y}, r, lop_);
    }
    Pose2 mo;
    {
      std::lock_guard<std::mutex> lk(m_);
      mo = T_map_odom_;
    }
    temp_.eraseDisc(mo.inverse() * Eigen::Vector2d(x, y), r, lop_.lmin);
    obst_cache_t_ = -1e9;
    last_obst_emit_t_ = -1e9;
    reply(cmd, true, "Đã tẩy vật cản quanh (" + jnum(x, 2) + ", " + jnum(y, 2) + ")");
  } else if (cmd == "obst_clear_temp") {
    temp_.clear();
    obst_cache_t_ = -1e9;
    last_obst_emit_t_ = -1e9;
    reply(cmd, true, "Đã xoá vật cản tạm");
  } else if (cmd == "status") {
    // publish* chỉ đọc dữ liệu dưới khoá: gọi từ luồng chính là an toàn
    publishGraph();
    publishGrid(false);
    publishPlaces();
    emitPath();
  } else {
    static const std::pair<const char *, Job::Type> MAP[] = {
        {"new_map", Job::NEW_MAP},       {"load_map", Job::LOAD_MAP},         {"save_map", Job::SAVE_MAP},
        {"set_mode", Job::SET_MODE},     {"save_place", Job::SAVE_PLACE},     {"delete_place", Job::DELETE_PLACE},
        {"reloc", Job::RELOC},           {"calib_ref", Job::CALIB_REF},       {"calib_measure", Job::CALIB_MEASURE}};
    for (const auto &kv : MAP)
      if (cmd == kv.first) {
        Job j = baseJob(kv.second);
        if (kv.second == Job::CALIB_REF || kv.second == Job::CALIB_MEASURE) {
          if (last_img_.empty()) {
            reply(cmd, false, "Chưa có ảnh camera");
            return;
          }
          const Hist h = interp(last_img_t_);
          j.gray = last_img_;
          j.t = last_img_t_;
          j.odom = h.odom;
          j.pitch = h.pitch;
        }
        if (kv.second == Job::LOAD_MAP || kv.second == Job::SAVE_MAP) {
          ss >> j.a;
          j.b = restOf(ss);
        } else {
          j.a = restOf(ss);
        }
        if (kv.second == Job::NEW_MAP || kv.second == Job::LOAD_MAP) {
          if (ctrl_.active()) {
            ctrl_.cancel("Đổi bản đồ");
            cmd_ = ControlOutput();
            emitCmd(cmd_);
          }
          ctrl_.clear();
          emitPath();
        }
        submit(std::move(j));
        return;
      }
    reply(cmd, false, "Lệnh lạ: " + cmd);
  }
}

void SlamSystem::emitState() {
  const Pose2 odom = ekf_.pose();
  const Eigen::Matrix4d &P = ekf_.P();
  std::string s = "{\"type\":\"state\",\"t\":" + jnum(t_, 3);
  {
    std::lock_guard<std::mutex> lk(m_);
    const Eigen::Vector2d sig = mapSigmaLocked(curSnap());
    int nb = 0, lp = 0, rl = 0, blind = 0;
    for (const auto &e : vis_edges_) (e.kind == EdgeKind::Neighbor ? nb : e.kind == EdgeKind::Loop ? lp : rl)++;
    for (const auto &k : kfs_) blind += k.blind;
    s += ",\"mode\":" + jstr(mode_ == Mode::Mapping ? "mapping" : "localization") + ",\"map\":" + jstr(map_id_) +
         ",\"map_name\":" + jstr(map_name_) + ",\"localized\":" + jbool(localized_) +
         ",\"reloc\":{\"count\":" + std::to_string(hyp_valid_ ? hyp_count_ : 0) +
         ",\"need\":" + std::to_string(cfg_.loop.reloc_confirm) + "}" + ",\"odom\":" + poseJson(odom) +
         ",\"pose\":" + poseJson(T_map_odom_ * odom) + ",\"sigma\":[" + jnum(sig.x(), 3) + "," +
         jnum(rad2deg(sig.y()), 2) + "]" + ",\"kfs\":" + std::to_string(kfs_.size()) +
         ",\"blind\":" + std::to_string(blind) + ",\"edges\":{\"odom\":" + std::to_string(odom_edges_.size()) +
         ",\"neighbor\":" + std::to_string(nb) + ",\"loop\":" + std::to_string(lp) + ",\"reloc\":" + std::to_string(rl) +
         "},\"slips\":" + std::to_string(slips_) + ",\"rejected_loops\":" + std::to_string(rejected_loops_) +
         ",\"height\":{\"used\":" + jnum(heightUsedLocked(), 3) + ",\"auto\":" +
         jbool(cfg_.camera.ceiling_height <= 0) + ",\"n\":" + std::to_string(h_samples_.size()) + "}" +
         ",\"frames\":{\"rx\":" + std::to_string(frames_rx_) + ",\"processed\":" + std::to_string(frames_processed_) +
         ",\"motion_skip\":" + std::to_string(frames_skipped_motion_) +
         ",\"busy_drop\":" + std::to_string(frames_dropped_busy_) + ",\"kf_rejected\":" + std::to_string(kf_rejected_) +
         "},\"feats\":" + std::to_string(last_feats_) + ",\"match\":{\"ok\":" + jbool(last_match_.ok) +
         ",\"kind\":" + jstr(last_match_kind_) + ",\"inliers\":" + std::to_string(last_match_.inliers) +
         ",\"why\":" + jstr(last_match_.why) + "}" + ",\"worker_ms\":" + jnum(worker_ms_, 1) +
         ",\"opt_ms\":" + jnum(opt_ms_, 1) + ",\"grid_v\":" + std::to_string(grid_version_) +
         ",\"graph_v\":" + std::to_string(graph_version_) + ",\"quality\":" + qualityJson();
  }
  const auto &st = ctrl_.state();
  s += ",\"ekf\":{\"sxy\":" + jnum(std::sqrt(0.5 * (P(0, 0) + P(1, 1))), 3) +
       ",\"sth\":" + jnum(rad2deg(std::sqrt(P(2, 2))), 2) + ",\"bias\":" + jnum(ekf_.bias(), 5) +
       ",\"bias_sigma\":" + jnum(std::sqrt(P(3, 3)), 5) + ",\"zupt\":" + std::to_string(zupt_count_) +
       ",\"womega\":[" + std::to_string(wo_ok_) + "," + std::to_string(wo_rej_) + "]" +
       ",\"odom_resets\":" + std::to_string(wo_.resets()) + "}";
  s += ",\"pitch\":" + jnum(rad2deg(pitch_), 2) + ",\"busy\":" + jbool(img_inflight_);
  {
    const double cf = clearFront();
    size_t nst, nobs;
    {
      std::lock_guard<std::mutex> lk(m_);
      nst = static_.occupied(lop_.locc).size();
      nobs = static_.obs().size();
    }
    s += ",\"sonar\":{\"n\":" + std::to_string(sonar_geom_.sensors.size()) +
         ",\"clear\":" + (std::isfinite(cf) && cf < 1e8 ? jnum(cf, 3) : std::string("null")) +
         ",\"temp\":" + std::to_string(temp_.occupied(t_, lop_.locc).size()) + ",\"static\":" + std::to_string(nst) +
         ",\"static_obs\":" + std::to_string(nobs) + "}";
  }
  s += ",\"nav\":{\"state\":" + jstr(st) + ",\"reason\":" + jstr(ctrl_.reason()) + ",\"label\":" + jstr(nav_label_);
  if (!ctrl_.waypoints().empty()) {
    const auto &g = ctrl_.waypoints().back();
    s += ",\"goal\":[" + jnum(g.x()) + "," + jnum(g.y()) + "],\"wp\":[" + std::to_string(ctrl_.index()) + "," +
         std::to_string(ctrl_.waypoints().size()) + "]";
  }
  if (!leg_ends_.empty()) {
    const int total = int(std::max(route_goals_.size(), leg_ends_.size()));
    const int leg = leg_offset_ + currentLeg() + (ctrl_.state() == "arrived" ? 0 : 1);
    s += ",\"leg\":[" + std::to_string(std::min(leg, total)) + "," + std::to_string(total) + "]";
  }
  if (ctrl_.state() == "blocked")
    s += ",\"blocked\":{\"t\":" + jnum(ctrl_.blockedFor(t_), 1) + ",\"scan\":" + std::to_string(ctrl_.scanPhase()) +
         ",\"next_replan\":" + jnum(ctrl_.nextReplanIn(t_), 1) + ",\"cause\":" + jstr(ctrl_.blockCause()) + "}";
  s += ",\"replans\":" + std::to_string(replans_);
  s += ",\"v_max\":" + jnum(cfg_.controller.v_max, 2) + ",\"w_max\":" + jnum(cfg_.controller.w_max, 2);
  if (ctrl_.active())
    s += ",\"remaining\":" + jnum(ctrl_.remaining(), 2) + ",\"herr_deg\":" + jnum(rad2deg(ctrl_.headingError()), 1) +
         ",\"elapsed\":" + jnum(ctrl_.elapsed(t_), 1);
  s += "},\"cmd\":[" + jnum(cmd_.v, 3) + "," + jnum(cmd_.w, 3) + "]}";
  emit(s);
}

std::string SlamSystem::qualityJson() const {
  const auto &q = cfg_.quality;
  const int n = int(kfs_.size());
  int blind = 0, lp = 0;
  for (const auto &k : kfs_) blind += k.blind;
  std::vector<char> has_nb(n, 0);
  for (const auto &e : vis_edges_) {
    if (e.kind == EdgeKind::Loop) ++lp;
    if (e.kind == EdgeKind::Neighbor && e.b < n) has_nb[e.b] = 1;
  }
  int nb = 0;
  int nonblind_after_first = 0;
  for (int i = 1; i < n; i++)
    if (!kfs_[i].blind) nb += has_nb[i], ++nonblind_after_first;
  const double vis_ratio = n ? double(n - blind) / n : 0;
  const double nb_ratio = nonblind_after_first ? double(nb) / nonblind_after_first : 0;
  struct C {
    const char *id, *label;
    bool ok;
    std::string val, lim;
  };
  const C cs[] = {
      {"kfs", "Số keyframe", n >= q.min_keyframes, std::to_string(n), "≥ " + std::to_string(q.min_keyframes)},
      {"visual", "Keyframe có đặc trưng trần", vis_ratio >= q.min_visual_ratio, jnum(vis_ratio * 100, 0) + "%",
       "≥ " + jnum(q.min_visual_ratio * 100, 0) + "%"},
      {"neighbor", "Camera bám liên tục (cạnh lân cận)", nb_ratio >= q.min_neighbor_ratio, jnum(nb_ratio * 100, 0) + "%",
       "≥ " + jnum(q.min_neighbor_ratio * 100, 0) + "%"},
      {"loops", "Loop closure (quay lại chỗ cũ)", lp >= q.min_loops, std::to_string(lp), "≥ " + std::to_string(q.min_loops)},
      {"localized", "Đang định vị trên bản đồ", localized_, localized_ ? "có" : "chưa", ""},
  };
  std::string s = "{\"checks\":[";
  bool ready = true;
  for (size_t i = 0; i < sizeof(cs) / sizeof(cs[0]); i++) {
    ready = ready && cs[i].ok;
    s += std::string(i ? "," : "") + "{\"id\":" + jstr(cs[i].id) + ",\"label\":" + jstr(cs[i].label) +
         ",\"ok\":" + jbool(cs[i].ok) + ",\"value\":" + jstr(cs[i].val) + ",\"limit\":" + jstr(cs[i].lim) + "}";
  }
  return s + "],\"ready\":" + jbool(ready) + "}";
}

// ========================================================================================= HÀNG ĐỢI
void SlamSystem::submit(Job &&j) {
  if (opt_.synchronous) {
    runJob(j);
    return;
  }
  if (j.image_job) img_inflight_ = true;
  {
    std::lock_guard<std::mutex> lk(jm_);
    jobs_.push_back(std::move(j));
  }
  jcv_.notify_one();
}

void SlamSystem::flush() {
  if (opt_.synchronous) return;
  std::unique_lock<std::mutex> lk(jm_);
  idle_cv_.wait(lk, [&] { return jobs_.empty() && running_ == 0; });
}

void SlamSystem::workerLoop() {
  for (;;) {
    Job j;
    {
      std::unique_lock<std::mutex> lk(jm_);
      jcv_.wait(lk, [&] { return stop_ || !jobs_.empty(); });
      if (stop_) return;
      j = std::move(jobs_.front());
      jobs_.pop_front();
      ++running_;
    }
    runJob(j);
    {
      std::lock_guard<std::mutex> lk(jm_);
      --running_;
    }
    idle_cv_.notify_all();
  }
}

void SlamSystem::runJob(Job &j) {
  const double t0 = nowMs();
  try {
    switch (j.type) {
      case Job::KEYFRAME: doKeyframe(j); break;
      case Job::LOCALIZE: doLocalize(j); break;
      case Job::CALIB_REF:
      case Job::CALIB_MEASURE: doCalib(j); break;
      case Job::NEW_MAP: doNewMap(j); break;
      case Job::LOAD_MAP: doLoadMap(j); break;
      case Job::SAVE_MAP: doSaveMap(j); break;
      case Job::SET_MODE: doSetMode(j); break;
      case Job::SAVE_PLACE:
      case Job::DELETE_PLACE: doPlace(j); break;
      case Job::RELOC: {
        std::lock_guard<std::mutex> lk(m_);
        if (kfs_.empty()) {
          reply("reloc", false, "Chưa có bản đồ");
          break;
        }
        localized_ = false;
        hyp_valid_ = false;
        reply("reloc", true, "Đang tái định vị");
        break;
      }
    }
  } catch (const std::exception &e) {
    event("error", std::string("Lỗi worker: ") + e.what());
  }
  if (j.image_job) {
    std::lock_guard<std::mutex> lk(m_);
    worker_ms_ = nowMs() - t0;
  }
  if (j.image_job) img_inflight_ = false;
}

// ============================================================================================ WORKER
int SlamSystem::lastKfOfSession() const {
  for (int i = int(kfs_.size()) - 1; i >= 0; i--)
    if (kfs_[i].session == session_) return i;
  return -1;
}

void SlamSystem::debugImage(const Job &j, const FrameFeatures &ff) {
  if (opt_.shm_dir.empty() || j.gray.empty() || cfg_.frontend.debug_jpg_period <= 0 ||
      j.t - last_debug_t_ < cfg_.frontend.debug_jpg_period)
    return;
  last_debug_t_ = j.t;
  cv::Mat img;
  cv::cvtColor(j.gray, img, cv::COLOR_GRAY2BGR);
  for (const auto &p : ff.raw) cv::circle(img, p, 3, cv::Scalar(80, 230, 120), 1, cv::LINE_AA);
  cv::putText(img, std::to_string(ff.size()) + " ORB", {8, 22}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {60, 200, 255}, 2);
  const std::string tmp = opt_.shm_dir + "/frame.tmp.jpg";
  if (cv::imwrite(tmp, img, {cv::IMWRITE_JPEG_QUALITY, 70})) {
    std::error_code ec;
    fs::rename(tmp, opt_.shm_dir + "/frame.jpg", ec);
  }
}

void SlamSystem::doKeyframe(Job &j) {
  FrameFeatures ff;
  const bool have_img = !j.gray.empty();
  if (have_img) fe_.process(j.gray, j.t, j.pitch, ff);
  debugImage(j, ff);
  {
    std::lock_guard<std::mutex> lk(m_);
    if (have_img) ++frames_processed_, last_feats_ = int(ff.size());
    // lệnh đổi chế độ / bản đồ có thể tới sau khi việc này được xếp hàng
    if (mode_ != Mode::Mapping || !localized_) return;
  }
  const double moved = has_last_kf_ ? std::hypot(Pose2::between(last_kf_odom_, j.odom).x,
                                                 Pose2::between(last_kf_odom_, j.odom).y)
                                    : 1e9;
  const bool blind = int(ff.size()) < cfg_.frontend.min_features;
  if (blind && have_img) {
    // Đợi khung sau; chỉ tạo keyframe mù khi đã đi quá xa (hoặc mãi không có ảnh tốt lúc bắt đầu)
    const bool give_up = has_last_kf_ ? moved >= cfg_.frontend.blind_kf_dist : kf_reject_streak_ >= 30;
    if (!give_up) {
      std::lock_guard<std::mutex> lk(m_);
      ++kf_rejected_;
      ++kf_reject_streak_;
      return;
    }
  }

  const double H = heightUsedLocked();
  Keyframe kf;
  kf.id = int(kfs_.size());
  kf.stamp = j.t;
  kf.T_odom = j.odom;
  kf.pitch = j.pitch;
  kf.blind = blind;
  kf.session = session_;
  if (!blind) kf.pts = ff.pts, kf.desc = ff.desc;
  const double f = cam_.f(have_img ? j.gray.cols : cfg_.camera.width);

  std::vector<OdomEdge> new_odom;
  const int prev = lastKfOfSession();
  if (prev >= 0) {
    OdomEdge e;
    e.a = prev;
    e.b = kf.id;
    e.meas = Pose2::between(kfs_[prev].T_odom, kf.T_odom);
    e.cov = j.relcov;
    e.cov.diagonal() = e.cov.diagonal().cwiseMax(Eigen::Vector3d(1e-6, 1e-6, 1e-7));
    new_odom.push_back(e);
    kf.T_map = kfs_[prev].T_map * e.meas;
  } else {
    kf.T_map = T_map_odom_ * kf.T_odom;
    if (!kfs_.empty()) {
      // Phiên mới trên bản đồ cũ (chuyển localization -> mapping): nối với keyframe gần nhất theo T_map_odom
      int near = 0;
      double bd = 1e18;
      for (const auto &k : kfs_) {
        const double d = (k.T_map.t() - kf.T_map.t()).norm();
        if (d < bd) bd = d, near = k.id;
      }
      OdomEdge e;
      e.a = near;
      e.b = kf.id;
      e.meas = Pose2::between(kfs_[near].T_map, kf.T_map);
      e.cov = P_mo_ + Eigen::Vector3d(1e-4, 1e-4, deg2rad(1) * deg2rad(1)).asDiagonal().toDenseMatrix();
      new_odom.push_back(e);
    }
  }

  // ---- cạnh thị giác: lân cận + loop
  std::vector<VisualEdge> new_vis;
  std::vector<double> new_h;
  bool slip = false;
  if (!blind) {
    std::vector<int> neigh;
    for (int i = prev; i >= 0 && int(neigh.size()) < cfg_.loop.neighbor_edges; i--)
      if (kfs_[i].session == session_ && !kfs_[i].blind) neigh.push_back(i);
    auto tryEdge = [&](int jdx, EdgeKind kind, double max_d, double max_a) -> bool {
      const MatchResult r = matchFrames(kfs_[jdx].features(), ff, cfg_.loop);
      {
        std::lock_guard<std::mutex> lk(m_);
        last_match_ = r;
        last_match_kind_ = kindName(kind);
      }
      if (!r.ok) return false;
      const Pose2 meas = baseMotion(cam_, r.alpha, r.tx, r.ty, f, H, kfs_[jdx].pitch, kf.pitch);
      const Pose2 pred = Pose2::between(kfs_[jdx].T_map, kf.T_map);
      const Pose2 diff = Pose2::between(pred, meas);
      if (std::hypot(diff.x, diff.y) > max_d || std::fabs(diff.th) > deg2rad(max_a)) {
        std::lock_guard<std::mutex> lk(m_);
        last_match_.ok = false;
        last_match_.why = "lệch dự đoán " + jnum(std::hypot(diff.x, diff.y), 2) + " m";
        return false;
      }
      VisualEdge e;
      e.a = jdx;
      e.b = kf.id;
      e.alpha = r.alpha, e.tx = r.tx, e.ty = r.ty, e.f = f;
      e.inliers = r.inliers;
      e.kind = kind;
      new_vis.push_back(e);
      if (kind == EdgeKind::Neighbor && kfs_[jdx].session == session_) {
        const Pose2 od = Pose2::between(kfs_[jdx].T_odom, kf.T_odom);
        const double h = estimateCeilingHeight(cam_, od, r.tx, r.ty, f, kfs_[jdx].pitch, kf.pitch);
        if (h > 0.5 && h < 10) new_h.push_back(h);
        // Thị giác và odometry bánh không khớp -> nghi bánh trượt / kẹt: hạ trọng số cạnh odometry
        if (jdx == prev && !new_odom.empty()) {
          // chỉ xét tịnh tiến: lệch hướng là do bias gyro, không phải bánh
          const Eigen::Vector2d dv(od.x - meas.x, od.y - meas.y);
          const Eigen::Matrix2d S = (new_odom[0].cov + visualCov(r.inliers)).topLeftCorner<2, 2>();
          if (dv.dot(S.inverse() * dv) > 16.0 && dv.norm() > 0.05) slip = true;
        }
      }
      return true;
    };
    for (int i : neigh) tryEdge(i, EdgeKind::Neighbor, cfg_.loop.neighbor_max_dist, cfg_.loop.neighbor_max_angle_deg);

    // Ứng viên loop: cách xa về thời gian (hoặc thuộc phiên khác), gần về vị trí theo map
    std::vector<std::pair<double, int>> cands;
    for (const auto &k : kfs_) {
      if (k.blind || std::find(neigh.begin(), neigh.end(), k.id) != neigh.end()) continue;
      if (k.session == session_ && k.id > kf.id - cfg_.loop.min_kf_gap) continue;
      const double d = (k.T_map.t() - kf.T_map.t()).norm();
      if (d < cfg_.loop.search_radius) cands.emplace_back(d, k.id);
    }
    std::sort(cands.begin(), cands.end());
    int tried = 0, accepted = 0;
    for (const auto &c : cands) {
      if (tried++ >= cfg_.loop.max_candidates || accepted >= 2) break;
      if (tryEdge(c.second, EdgeKind::Loop, cfg_.loop.max_pred_dist, cfg_.loop.max_pred_angle_deg)) ++accepted;
    }
  }
  if (slip) new_odom[0].cov = Eigen::Vector3d(0.25 * 0.25, 0.25 * 0.25, deg2rad(20) * deg2rad(20)).asDiagonal();

  std::vector<size_t> new_vis_idx;
  int prev_for_path = prev;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (prev_for_path >= 0) {
      auto &pk = kfs_[prev_for_path];
      const Pose2 inv = pk.T_odom.inverse();
      auto it = path_pending_.begin();
      for (; it != path_pending_.end() && it->first <= kf.stamp; ++it)
        if (it->first >= pk.stamp) pk.path_rel.push_back(inv * it->second);
      pk.path_rel.push_back(inv * kf.T_odom);
      path_pending_.erase(path_pending_.begin(), it);
    } else {
      auto it = std::find_if(path_pending_.begin(), path_pending_.end(),
                             [&](const std::pair<double, Pose2> &p) { return p.first > kf.stamp; });
      path_pending_.erase(path_pending_.begin(), it);
    }
    kfs_.push_back(kf);
    for (auto &e : new_odom) {
      odom_edges_.push_back(e);
      odom_slip_.push_back(slip && &e == &new_odom[0]);
    }
    if (slip) ++slips_;
    for (auto &e : new_vis) {
      new_vis_idx.push_back(vis_edges_.size());
      vis_edges_.push_back(e);
    }
    for (double h : new_h) h_samples_.push_back(h);
    if (h_samples_.size() > 400) h_samples_.erase(h_samples_.begin(), h_samples_.begin() + 100);
    has_last_kf_ = true;
    last_kf_odom_ = kf.T_odom;
    last_kf_id_ = kf.id;
    kf_reject_streak_ = 0;
    if (new_vis.empty()) {
      T_map_odom_ = kf.T_map * kf.T_odom.inverse();
      mo_snap_ = j.snap();
      ++graph_version_;
    }
  }
  rel_reset_ = true;
  if (slip) event("warn", "Bánh trượt / kẹt: odometry lệch ảnh trần ở keyframe " + std::to_string(kf.id));
  if (!new_vis.empty()) optimizeGraph(new_vis_idx, j.snap());
  for (const auto &e : new_vis)
    if (e.kind == EdgeKind::Loop)
      event("info", "Loop closure: keyframe " + std::to_string(e.a) + " ↔ " + std::to_string(e.b) + " (" +
                        std::to_string(e.inliers) + " inlier)");
  rebuildGrid();
  publishGraph();
  publishGrid();
}

void SlamSystem::optimizeGraph(const std::vector<size_t> &new_vis, const OdomSnap &snap) {
  std::vector<Pose2> nodes(kfs_.size());
  for (size_t i = 0; i < kfs_.size(); i++) nodes[i] = kfs_[i].T_map;
  auto build = [&](std::vector<GraphEdge> &edges, std::vector<int> &vis_of) {
    edges.clear();
    vis_of.clear();
    for (const auto &e : odom_edges_) {
      GraphEdge g;
      g.a = e.a, g.b = e.b, g.meas = e.meas, g.cov = e.cov;
      edges.push_back(g);
      vis_of.push_back(-1);
    }
    for (size_t i = 0; i < vis_edges_.size(); i++) {
      edges.push_back(toGraphEdge(vis_edges_[i]));
      vis_of.push_back(int(i));
    }
  };
  std::vector<GraphEdge> edges;
  std::vector<int> vis_of;
  build(edges, vis_of);
  // Chỉ cạnh loop MỚI được phép bị loại (cạnh cũ đã qua kiểm tra ở các lần trước)
  std::vector<bool> rejectable(edges.size(), false);
  for (size_t k = 0; k < edges.size(); k++)
    rejectable[k] = vis_of[k] >= 0 && vis_edges_[vis_of[k]].kind == EdgeKind::Loop &&
                    std::find(new_vis.begin(), new_vis.end(), size_t(vis_of[k])) != new_vis.end();
  std::vector<Pose2> x = nodes;
  std::vector<size_t> rej;
  const OptimizeResult r = optimizeWithRejection(x, edges, {0}, cfg_.loop.huber_k, 25.0, rejectable, rej);
  if (!rej.empty()) {
    std::vector<size_t> bad;
    for (size_t k : rej) bad.push_back(size_t(vis_of[k]));
    {
      std::lock_guard<std::mutex> lk(m_);
      std::sort(bad.rbegin(), bad.rend());
      for (size_t b : bad) vis_edges_.erase(vis_edges_.begin() + long(b));
      rejected_loops_ += int(bad.size());
    }
    event("warn", "Bỏ " + std::to_string(bad.size()) + " cạnh loop không nhất quán với đồ thị");
  }
  if (!r.ok) {
    event("error", "Tối ưu pose graph thất bại");
    return;
  }
  {
  std::lock_guard<std::mutex> lk(m_);
  for (size_t i = 0; i < kfs_.size(); i++) kfs_[i].T_map = x[i];
  for (auto &pl : places_)
    if (pl.kf >= 0 && pl.kf < int(kfs_.size())) {
      const Pose2 p = kfs_[pl.kf].T_map * pl.rel;
      pl.x = p.x, pl.y = p.y;
      if (pl.has_th) pl.th = p.th;
    }
  static_.rebuild(kfs_, lop_);   // số đo siêu âm neo keyframe: dời theo keyframe
  const int last = lastKfOfSession();
  if (last >= 0 && localized_) {
    T_map_odom_ = kfs_[last].T_map * kfs_[last].T_odom.inverse();
    P_mo_ = Eigen::Vector3d(1e-4, 1e-4, deg2rad(0.5) * deg2rad(0.5)).asDiagonal();
    mo_snap_ = snap;
  }
  opt_ms_ = r.ms;
  ++graph_version_;
  }
  if (!places_.empty()) publishPlaces();
}

bool SlamSystem::updateMapOdom(const Pose2 &z, const Pose2 &meas_base, int inliers, const OdomSnap &snap) {
  std::lock_guard<std::mutex> lk(m_);
  const Eigen::Matrix3d P = P_mo_ + processNoiseLocked(snap);
  // Phép đo z = T_map_base ⊕ T_odom_base⁻¹: sai số hướng của T_map_base kéo theo sai số vị trí của z
  Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
  J(0, 2) = -(z.y - meas_base.y);
  J(1, 2) = z.x - meas_base.x;
  const Eigen::Matrix3d Rb = rot3(meas_base.th);
  const Eigen::Matrix3d R = J * Rb * visualCov(inliers) * Rb.transpose() * J.transpose();
  const Eigen::Vector3d r(z.x - T_map_odom_.x, z.y - T_map_odom_.y, wrapAngle(z.th - T_map_odom_.th));
  const Eigen::Matrix3d S = P + R;
  if (r.dot(S.inverse() * r) > cfg_.loc.chi2_gate) return false;
  const Eigen::Matrix3d K = P * S.inverse();
  const Eigen::Vector3d dx = K * r;
  T_map_odom_ = Pose2(T_map_odom_.x + dx(0), T_map_odom_.y + dx(1), T_map_odom_.th + dx(2));
  P_mo_ = (Eigen::Matrix3d::Identity() - K) * P;
  P_mo_ = 0.5 * (P_mo_ + P_mo_.transpose());
  mo_snap_ = snap;
  return true;
}

void SlamSystem::doLocalize(Job &j) {
  FrameFeatures ff;
  fe_.process(j.gray, j.t, j.pitch, ff);
  debugImage(j, ff);
  bool loc;
  Pose2 Tmo;
  {
    std::lock_guard<std::mutex> lk(m_);
    ++frames_processed_;
    last_feats_ = int(ff.size());
    loc = localized_;
    Tmo = T_map_odom_;
    if (kfs_.empty()) return;
  }
  const double H = heightUsedLocked();
  const double f = cam_.f(j.gray.cols);
  auto setMatch = [&](const MatchResult &r, const char *kind) {
    std::lock_guard<std::mutex> lk(m_);
    last_match_ = r;
    last_match_kind_ = kind;
  };
  if (int(ff.size()) < cfg_.loop.min_inliers) {
    MatchResult r;
    r.why = "ít đặc trưng (" + std::to_string(ff.size()) + ")";
    setMatch(r, loc ? "bám bản đồ" : "tái định vị");
    return;
  }

  if (loc) {
    // Bám bản đồ: so với keyframe gần vị trí dự đoán
    const Pose2 pred = Tmo * j.odom;
    std::vector<std::pair<double, int>> cands;
    for (const auto &k : kfs_)
      if (!k.blind) {
        const double d = (k.T_map.t() - pred.t()).norm();
        if (d < cfg_.loop.search_radius) cands.emplace_back(d, k.id);
      }
    std::sort(cands.begin(), cands.end());
    int tried = 0;
    for (const auto &c : cands) {
      if (tried++ >= cfg_.loop.max_candidates) break;
      const Keyframe &k = kfs_[c.second];
      MatchResult r = matchFrames(k.features(), ff, cfg_.loop);
      if (!r.ok) {
        setMatch(r, "bám bản đồ");
        continue;
      }
      const Pose2 meas = k.T_map * baseMotion(cam_, r.alpha, r.tx, r.ty, f, H, k.pitch, j.pitch);
      const Pose2 diff = Pose2::between(pred, meas);
      if (std::hypot(diff.x, diff.y) > cfg_.loop.max_pred_dist || std::fabs(diff.th) > deg2rad(cfg_.loop.max_pred_angle_deg)) {
        r.ok = false;
        r.why = "lệch dự đoán " + jnum(std::hypot(diff.x, diff.y), 2) + " m";
        setMatch(r, "bám bản đồ");
        continue;
      }
      if (!updateMapOdom(meas * j.odom.inverse(), meas, r.inliers, j.snap())) {
        r.ok = false;
        r.why = "bị cổng χ² loại";
      }
      setMatch(r, "bám bản đồ");
      if (r.ok) break;
    }
    return;
  }

  // Tái định vị: so với mọi keyframe, lấy ứng viên nhiều inlier nhất
  MatchResult best;
  int best_kf = -1;
  for (const auto &k : kfs_) {
    if (k.blind) continue;
    const MatchResult r = matchFrames(k.features(), ff, cfg_.loop);
    if (r.ok && r.inliers > best.inliers) best = r, best_kf = k.id;
    else if (best_kf < 0 && r.inliers > best.inliers) best = r;
  }
  if (best_kf < 0) {
    if (best.why.empty()) best.why = "không khớp keyframe nào";
    setMatch(best, "tái định vị");
    return;
  }
  const Keyframe &k = kfs_[best_kf];
  const Pose2 rel = baseMotion(cam_, best.alpha, best.tx, best.ty, f, H, k.pitch, j.pitch);
  const Pose2 meas = k.T_map * rel;
  const Pose2 h = meas * j.odom.inverse();
  bool confirmed = false;
  Mode mode;
  {
    std::lock_guard<std::mutex> lk(m_);
    last_match_ = best;
    last_match_kind_ = "tái định vị";
    const Pose2 d = Pose2::between(hyp_, h);
    if (hyp_valid_ && j.t - hyp_t_ < cfg_.loop.reloc_timeout && std::hypot(d.x, d.y) < cfg_.loop.reloc_agree_dist &&
        std::fabs(d.th) < deg2rad(cfg_.loop.reloc_agree_angle_deg))
      ++hyp_count_;
    else
      hyp_count_ = 1;
    hyp_ = h;
    hyp_t_ = j.t;
    hyp_valid_ = true;
    hyp_kf_ = best_kf;
    mode = mode_;
    if (hyp_count_ >= cfg_.loop.reloc_confirm) {
      confirmed = true;
      localized_ = true;
      T_map_odom_ = h;
      Eigen::Matrix3d J = Eigen::Matrix3d::Identity();
      J(0, 2) = -(h.y - meas.y);
      J(1, 2) = h.x - meas.x;
      const Eigen::Matrix3d Rb = rot3(meas.th);
      P_mo_ = J * Rb * visualCov(best.inliers) * Rb.transpose() * J.transpose();
      mo_snap_ = j.snap();
      hyp_valid_ = false;
      ++graph_version_;
    }
  }
  if (!confirmed) return;
  event("info", "Đã tái định vị (khớp keyframe " + std::to_string(best_kf) + ", " + std::to_string(best.inliers) +
                    " inlier)");
  if (mode != Mode::Mapping) return;

  // Mở rộng bản đồ cũ: tạo keyframe tại khung này, nối với keyframe cũ bằng cạnh tái định vị
  Keyframe kf;
  kf.id = int(kfs_.size());
  kf.stamp = j.t;
  kf.T_odom = j.odom;
  kf.T_map = meas;
  kf.pitch = j.pitch;
  kf.session = session_;
  kf.pts = ff.pts;
  kf.desc = ff.desc;
  VisualEdge e;
  e.a = best_kf;
  e.b = kf.id;
  e.alpha = best.alpha, e.tx = best.tx, e.ty = best.ty, e.f = f;
  e.inliers = best.inliers;
  e.kind = EdgeKind::Reloc;
  std::vector<size_t> idx;
  {
    std::lock_guard<std::mutex> lk(m_);
    kfs_.push_back(kf);
    idx.push_back(vis_edges_.size());
    vis_edges_.push_back(e);
    has_last_kf_ = true;
    last_kf_odom_ = kf.T_odom;
    last_kf_id_ = kf.id;
    auto it = std::find_if(path_pending_.begin(), path_pending_.end(),
                           [&](const std::pair<double, Pose2> &p) { return p.first > kf.stamp; });
    path_pending_.erase(path_pending_.begin(), it);
  }
  rel_reset_ = true;
  optimizeGraph(idx, j.snap());
  rebuildGrid();
  publishGraph();
  publishGrid();
}

void SlamSystem::doCalib(Job &j) {
  // Ảnh mới nhất: luồng chính không giữ ảnh, nên lấy từ ảnh đang chờ gần nhất qua job.gray (có thể rỗng)
  const char *name = j.type == Job::CALIB_REF ? "calib_ref" : "calib_measure";
  if (j.gray.empty()) {
    reply(name, false, "Chưa có ảnh: thử lại khi camera chạy");
    return;
  }
  FrameFeatures ff;
  fe_.process(j.gray, j.t, j.pitch, ff);
  if (j.type == Job::CALIB_REF) {
    calib_ff_ = ff;
    calib_odom_ = j.odom;
    calib_has_ = true;
    reply(name, int(ff.size()) >= cfg_.frontend.min_features, "Đã lưu khung mốc (" + std::to_string(ff.size()) +
                                                                  " ORB). Ra lệnh xe tiến thẳng ~0,5 m rồi calib_measure");
    return;
  }
  if (!calib_has_) {
    reply(name, false, "Chưa có khung mốc: chạy calib_ref trước");
    return;
  }
  const MatchResult r = matchFrames(calib_ff_, ff, cfg_.loop);
  if (!r.ok) {
    reply(name, false, "Không khớp được với khung mốc: " + r.why);
    return;
  }
  const double f = cam_.f(j.gray.cols), H = heightUsed();
  const Pose2 vis = baseMotion(cam_, r.alpha, r.tx, r.ty, f, H, calib_ff_.pitch, ff.pitch);
  const Pose2 od = Pose2::between(calib_odom_, j.odom);
  const double h = estimateCeilingHeight(cam_, od, r.tx, r.ty, f, calib_ff_.pitch, ff.pitch);
  std::string advice;
  if (std::hypot(od.x, od.y) > 0.15 && std::hypot(vis.x, vis.y) > 0.05) {
    const double dyaw = rad2deg(wrapAngle(std::atan2(od.y, od.x) - std::atan2(vis.y, vis.x)));
    if (std::fabs(dyaw) > 10)
      advice = "Hướng tịnh tiến lệch " + jnum(dyaw, 0) + "°: sửa camera.yaw_deg thành " +
               jnum(cfg_.camera.yaw_deg + dyaw, 0);
    else
      advice = "Dấu trục camera ĐÚNG (lệch hướng " + jnum(dyaw, 1) + "°)";
  }
  if (std::fabs(od.th) > deg2rad(10) && vis.th * od.th < 0)
    advice += "; chiều quay ngược dấu: ảnh bị lật (kiểm tra cách gắn camera)";
  if (h > 0) advice += "; chiều cao trần ước lượng " + jnum(h, 2) + " m";
  reply(name, true, advice.empty() ? "Đi xa hơn (≥ 0,3 m) để kiểm tra" : advice,
        "\"visual\":" + poseJson(vis) + ",\"odom\":" + poseJson(od) + ",\"inliers\":" + std::to_string(r.inliers) +
            ",\"height\":" + jnum(h, 3));
}

void SlamSystem::doNewMap(Job &j) {
  {
    std::lock_guard<std::mutex> lk(m_);
    kfs_.clear();
    odom_edges_.clear();
    odom_slip_.clear();
    vis_edges_.clear();
    places_.clear();
    path_pending_.clear();
    grid_ = TraversedGrid();
    static_.clear();
    last_kf_id_ = -1;
    mode_ = Mode::Mapping;
    map_id_.clear();
    map_name_ = "(chưa lưu)";
    session_ = 0;
    T_map_odom_ = j.odom.inverse();   // xe đang ở gốc bản đồ mới
    P_mo_ = Eigen::Matrix3d::Identity() * 1e-6;
    mo_snap_ = j.snap();
    localized_ = true;
    has_last_kf_ = false;
    kf_reject_streak_ = 0;
    h_samples_.clear();
    loaded_height_ = 0;
    hyp_valid_ = false;
    slips_ = rejected_loops_ = 0;
    ++grid_version_;
    ++graph_version_;
  }
  temp_reset_ = true;
  publishGraph();
  publishGrid();
  publishPlaces();
  reply("new_map", true, "Bản đồ mới: lái xe chậm quanh khu vực, đi giữa lối, quay lại chỗ cũ vài lần");
}

void SlamSystem::doLoadMap(Job &j) {
  const std::string dir = (fs::path(opt_.maps_dir) / j.a).string();
  if (j.a.empty() || j.a.find('/') != std::string::npos || j.a.find("..") != std::string::npos) {
    reply("load_map", false, "Tên bản đồ không hợp lệ");
    return;
  }
  MapData m;
  std::string err;
  if (!loadMap(dir, m, err)) {
    reply("load_map", false, err);
    return;
  }
  std::string name = j.a;
  {
    std::ifstream f(dir + "/meta.json");
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string n = jsonField(ss.str(), "name");
    if (!n.empty()) name = n;
  }
  const bool mapping = j.b == "mapping";
  {
    std::lock_guard<std::mutex> lk(m_);
    kfs_ = std::move(m.kfs);
    odom_edges_ = std::move(m.odom);
    odom_slip_.assign(odom_edges_.size(), false);
    vis_edges_ = std::move(m.vis);
    places_ = std::move(m.places);
    static_.setData(m.obstacles.obs(), m.obstacles.erasures());
    static_.rebuild(kfs_, lop_);
    last_kf_id_ = -1;
    loaded_height_ = m.height_est;
    h_samples_.clear();
    int maxs = 0;
    for (const auto &k : kfs_) maxs = std::max(maxs, k.session);
    session_ = maxs + 1;
    mode_ = mapping ? Mode::Mapping : Mode::Localization;
    map_id_ = j.a;
    map_name_ = name;
    localized_ = false;
    hyp_valid_ = false;
    T_map_odom_ = Pose2();
    P_mo_ = Eigen::Matrix3d::Identity() * 100;
    has_last_kf_ = false;
    path_pending_.clear();
    slips_ = rejected_loops_ = 0;
    ++graph_version_;
  }
  temp_reset_ = true;
  rebuildGrid();
  publishGraph();
  publishGrid();
  publishPlaces();
  reply("load_map", true, "Đã tải \"" + name + "\" (" + std::to_string(kfs_.size()) + " keyframe), đang tái định vị",
        "\"id\":" + jstr(j.a) + ",\"mode\":" + jstr(mapping ? "mapping" : "localization"));
}

void SlamSystem::doSaveMap(Job &j) {
  std::string id = j.a, name = j.b.empty() ? j.a : j.b;
  bool okid = !id.empty() && id.size() <= 64;
  for (char c : id) okid = okid && (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-');
  if (!okid) {
    reply("save_map", false, "Mã bản đồ chỉ gồm chữ, số, _ và -");
    return;
  }
  MapData m;
  TraversedGrid g;
  std::string meta;
  {
    std::lock_guard<std::mutex> lk(m_);
    if (kfs_.empty()) {
      reply("save_map", false, "Bản đồ trống");
      return;
    }
    m.kfs = kfs_;
    m.odom = odom_edges_;
    m.vis = vis_edges_;
    m.places = places_;
    m.height_est = heightUsedLocked();
    g = grid_;
    m.obstacles = static_;
    if (!g.empty()) {
      const cv::Mat occ = obstacleMask(g, static_.occupied(lop_.locc), 0);
      m.obstacle_png = cv::Mat(g.ny, g.nx, CV_8U, cv::Scalar(255));
      m.obstacle_png.setTo(0, occ);
    }
    double len = 0;
    for (const auto &k : kfs_)
      for (size_t i = 1; i < k.path_rel.size(); i++)
        len += std::hypot(k.path_rel[i].x - k.path_rel[i - 1].x, k.path_rel[i].y - k.path_rel[i - 1].y);
    int lp = 0;
    for (const auto &e : vis_edges_) lp += e.kind == EdgeKind::Loop;
    char tbuf[32];
    std::time_t now = std::time(nullptr);
    std::strftime(tbuf, sizeof tbuf, "%Y-%m-%d %H:%M:%S", std::localtime(&now));
    meta = "{\"format\":\"iot-ck-slam2d/1\",\"id\":" + jstr(id) + ",\"name\":" + jstr(name) + ",\"created\":" +
           jstr(tbuf) + ",\"keyframes\":" + std::to_string(kfs_.size()) + ",\"loops\":" + std::to_string(lp) +
           ",\"length_m\":" + jnum(len, 2) + ",\"area_m2\":" +
           jnum(double(g.freeCells()) * g.res * g.res, 2) + ",\"places\":" + std::to_string(places_.size()) +
           ",\"ceiling_height\":" + jnum(m.height_est, 3) + ",\"sonar_obs\":" + std::to_string(static_.obs().size()) +
           ",\"quality\":" + qualityJson() + "}";
  }
  std::string cfgtxt = cfg_.source_text + "\n# ceiling_height dùng khi lưu: " + jnum(m.height_est, 3) + "\n";
  std::string err;
  if (!saveMap(opt_.maps_dir, id, m, g, cfgtxt, meta, err)) {
    reply("save_map", false, err);
    return;
  }
  {
    std::lock_guard<std::mutex> lk(m_);
    map_id_ = id;
    map_name_ = name;
  }
  reply("save_map", true, "Đã lưu \"" + name + "\"", "\"meta\":" + meta);
}

void SlamSystem::doSetMode(Job &j) {
  std::lock_guard<std::mutex> lk(m_);
  if (j.a == "localization") {
    if (kfs_.empty()) {
      reply("set_mode", false, "Chưa có bản đồ để định vị");
      return;
    }
    mode_ = Mode::Localization;
  } else if (j.a == "mapping") {
    if (mode_ != Mode::Mapping) {
      mode_ = Mode::Mapping;
      ++session_;          // keyframe tiếp theo nối vào bản đồ bằng cạnh theo T_map_odom
      has_last_kf_ = false;
    }
  } else {
    reply("set_mode", false, "Chế độ lạ: " + j.a);
    return;
  }
  reply("set_mode", true, j.a == "mapping" ? "Chế độ tạo / mở rộng bản đồ" : "Chế độ định vị (không sửa bản đồ)",
        "\"mode\":" + jstr(j.a));
}

void SlamSystem::persistPlacesLocked() {
  if (map_id_.empty()) return;
  const fs::path dir = fs::path(opt_.maps_dir) / map_id_;
  if (fs::is_directory(dir)) savePlaces(dir.string(), places_);
}

void SlamSystem::doPlace(Job &j) {
  const bool save = j.type == Job::SAVE_PLACE;
  const char *cmd = save ? "save_place" : "delete_place";
  if (j.a.empty() || j.a.size() > 60) {
    reply(cmd, false, "Tên địa điểm không hợp lệ");
    return;
  }
  {
    std::lock_guard<std::mutex> lk(m_);
    auto it = std::find_if(places_.begin(), places_.end(), [&](const Place &p) { return p.name == j.a; });
    if (save) {
      if (!localized_) {
        reply(cmd, false, "Chưa định vị trên bản đồ");
        return;
      }
      const Pose2 p = T_map_odom_ * j.odom;
      Place pl{j.a, p.x, p.y, p.th, true};
      double bd = 1e18;
      for (const auto &k : kfs_) {
        const double d = (k.T_map.t() - p.t()).norm();
        if (d < bd) bd = d, pl.kf = k.id;
      }
      if (pl.kf >= 0) pl.rel = Pose2::between(kfs_[pl.kf].T_map, p);
      if (it != places_.end())
        *it = pl;
      else
        places_.push_back(pl);
    } else {
      if (it == places_.end()) {
        reply(cmd, false, "Không có địa điểm \"" + j.a + "\"");
        return;
      }
      places_.erase(it);
    }
    persistPlacesLocked();
  }
  publishPlaces();
  reply(cmd, true, save ? "Đã lưu địa điểm \"" + j.a + "\"" : "Đã xoá \"" + j.a + "\"");
}

// ------------------------------------------------------------------------------------- grid / phát
void SlamSystem::rebuildGrid() {
  std::vector<std::vector<Eigen::Vector2d>> segs;
  {
    std::lock_guard<std::mutex> lk(m_);
    std::vector<Eigen::Vector2d> cur;
    int cur_session = -1;
    for (const auto &k : kfs_) {
      if (k.session != cur_session && !cur.empty()) segs.push_back(std::move(cur)), cur.clear();
      cur_session = k.session;
      auto add = [&](const Eigen::Vector2d &p) {
        if (!cur.empty() && (cur.back() - p).norm() > 0.4) segs.push_back(std::move(cur)), cur.clear();
        cur.push_back(p);
      };
      add(k.T_map.t());
      for (const auto &q : k.path_rel) add((k.T_map * q).t());
    }
    if (!cur.empty()) segs.push_back(std::move(cur));
  }
  TraversedGrid g;
  g.build(segs, cfg_.grid.resolution, cfg_.robot.body_radius, cfg_.grid.center_tolerance, cfg_.grid.margin);
  std::lock_guard<std::mutex> lk(m_);
  grid_ = g;
  ++grid_version_;
}

void SlamSystem::publishGraph() {
  std::string s;
  {
    std::lock_guard<std::mutex> lk(m_);
    s = "{\"type\":\"graph\",\"version\":" + std::to_string(graph_version_) + ",\"kfs\":[";
    for (size_t i = 0; i < kfs_.size(); i++) {
      const auto &k = kfs_[i];
      s += (i ? ",[" : "[") + jnum(k.T_map.x) + "," + jnum(k.T_map.y) + "," + jnum(k.T_map.th, 3) + "," +
           (k.blind ? "1" : "0") + "," + std::to_string(k.session) + "]";
    }
    s += "],\"edges\":[";
    for (size_t i = 0; i < vis_edges_.size(); i++)
      s += (i ? ",[" : "[") + std::to_string(vis_edges_[i].a) + "," + std::to_string(vis_edges_[i].b) + "," +
           std::to_string(int(vis_edges_[i].kind)) + "]";
    s += "],\"slip\":[";
    bool first = true;
    for (size_t i = 0; i < odom_edges_.size(); i++)
      if (i < odom_slip_.size() && odom_slip_[i]) {
        s += (first ? "" : ",") + std::to_string(odom_edges_[i].b);
        first = false;
      }
    s += "],\"traj\":[";
    first = true;
    Eigen::Vector2d last(1e9, 1e9);
    for (const auto &k : kfs_) {
      auto add = [&](const Eigen::Vector2d &p, bool brk) {
        if (!brk && (p - last).norm() < 0.05) return;
        s += (first ? "[" : ",[") + jnum(p.x(), 3) + "," + jnum(p.y(), 3) + (brk ? ",1]" : "]");
        first = false;
        last = p;
      };
      const bool brk = k.id > 0 && kfs_[k.id - 1].session != k.session;
      add(k.T_map.t(), brk);
      for (const auto &q : k.path_rel) add((k.T_map * q).t(), false);
    }
    s += "]}";
  }
  emit(s);
}

void SlamSystem::publishGrid(bool write_file) {
  TraversedGrid g;
  int v;
  {
    std::lock_guard<std::mutex> lk(m_);
    g = grid_;
    v = grid_version_;
  }
  std::string file;
  if (!opt_.shm_dir.empty() && !g.empty()) {
    if (!write_file) {
      file = "grid.png";
    } else {
    const std::string tmp = opt_.shm_dir + "/grid.tmp.png";
    if (cv::imwrite(tmp, g.renderWeb())) {
      std::error_code ec;
      fs::rename(tmp, opt_.shm_dir + "/grid.png", ec);
      if (!ec) file = "grid.png";
    }
    }
  }
  emit("{\"type\":\"grid\",\"version\":" + std::to_string(v) + ",\"res\":" + jnum(g.res, 3) + ",\"x0\":" + jnum(g.ox) +
       ",\"y0\":" + jnum(g.oy) + ",\"nx\":" + std::to_string(g.nx) + ",\"ny\":" + std::to_string(g.ny) +
       ",\"free\":" + std::to_string(g.freeCells()) + ",\"file\":" + jstr(file) + "}");
}

void SlamSystem::publishPlaces() {
  std::string s = "{\"type\":\"places\",\"items\":[";
  {
    std::lock_guard<std::mutex> lk(m_);
    for (size_t i = 0; i < places_.size(); i++) {
      const auto &p = places_[i];
      s += std::string(i ? "," : "") + "{\"name\":" + jstr(p.name) + ",\"x\":" + jnum(p.x) + ",\"y\":" + jnum(p.y) +
           ",\"theta\":" + (p.has_th ? jnum(p.th, 4) : "null") + "}";
    }
  }
  emit(s + "]}");
}

}  // namespace slam
