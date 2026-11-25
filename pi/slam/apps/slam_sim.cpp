// slam_sim: kịch bản đầu-cuối trên thế giới giả lập (không cần xe).
//
//   Pha A (mapping): lái hình chữ nhật 3 m × 2 m hai vòng (vòng 2 có 1,5 s bánh kẹt), lưu địa điểm ở
//                    các góc, lưu bản đồ "sim".
//   Pha B (localization): khởi động lại ở chỗ khác trên đường đã đi, tải bản đồ, xoay tại chỗ tới khi
//                    tái định vị, goto các địa điểm. So vị trí thật lúc tới với vị trí thật lúc lưu.
//
//   slam_sim --config pi/config/slam.yaml --maps /tmp/maps [--log pha_a.slamlog] [--verbose]
// Mã thoát 0 = đạt tiêu chí (khép vòng, tái định vị, goto lệch < 15 cm).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "slam/matcher.h"
#include <iostream>
#include <map>

#include "sim_world.h"
#include "slam/system.h"

using namespace slam;

int main(int argc, char **argv) {
  std::string config, maps = "/tmp/slam_sim_maps", logfile;
  bool verbose = false;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--config" && i + 1 < argc) config = argv[++i];
    else if (a == "--maps" && i + 1 < argc) maps = argv[++i];
    else if (a == "--log" && i + 1 < argc) logfile = argv[++i];
    else if (a == "--verbose") verbose = true;
    else {
      std::fprintf(stderr, "slam_sim --config slam.yaml [--maps DIR] [--log FILE] [--verbose]\n");
      return 2;
    }
  }
  Config cfg = config.empty() ? Config::defaults() : Config::load(config);
  auto printer = [&](const std::string &s) {
    if (s.rfind("{\"type\":\"event\"", 0) == 0 || s.rfind("{\"type\":\"reply\"", 0) == 0 ||
        (verbose && s.rfind("{\"type\":\"state\"", 0) != 0 && s.rfind("{\"type\":\"cmd\"", 0) != 0 &&
         s.rfind("{\"type\":\"graph\"", 0) != 0))
      std::printf("  %s\n", s.substr(0, 300).c_str());
  };

  sim::SimParams sp;
  sp.stuck = {{100.0 + 95.0, 100.0 + 96.5}};   // trong vòng 2
  SlamSystem::Options opt;
  opt.maps_dir = maps;
  opt.synchronous = true;
  std::map<std::string, Pose2> truth_of;
  bool pass = true;

  // ------------------------------------------------------------------ Pha A
  {
    std::printf("== Pha A: tạo bản đồ ==\n");
    FILE *log = nullptr;
    if (!logfile.empty()) {
      log = std::fopen(logfile.c_str(), "wb");
      std::fwrite("SLAMLOG1", 1, 8, log);
    }
    SlamSystem sys(cfg, opt, printer);
    sim::SimWorld w(cfg, sp);
    sim::SimRunner r(sys, w, log);
    r.run(0, 0, 2.0);
    r.command("new_map");
    r.run(0, 0, 1.0);
    for (int lap = 0; lap < 2; lap++) {
      const double L[4] = {3.0, 2.0, 3.0, 2.0};
      for (int k = 0; k < 4; k++) {
        r.forward(L[k]);
        if (lap == 0 && k < 3) {
          const std::string name = "goc" + std::to_string(k + 1);
          r.command("save_place " + name);
          truth_of[name] = w.truth();
        }
        r.rotate(M_PI / 2);
      }
      if (lap == 0) {
        r.command("save_place xuat_phat");
        truth_of["xuat_phat"] = w.truth();
      }
    }
    r.command("save_map sim Bản đồ giả lập");
    if (log) std::fclose(log);

    // Sai số keyframe (frame map gốc = keyframe đầu ≈ vị trí thật ban đầu)
    const auto kfs = sys.keyframes();
    const Pose2 odom_end = sys.odomPose(), map_end = sys.mapPose(), tr = w.truth();
    std::printf("  keyframe %zu, lân cận %d, loop %d, trượt bánh phát hiện %d, H dùng %.3f m\n", kfs.size(),
                sys.edgeCount(EdgeKind::Neighbor), sys.edgeCount(EdgeKind::Loop), sys.slipCount(), sys.heightUsed());
    std::printf("  cuối: thật (%.3f, %.3f, %.1f°)  map (%.3f, %.3f, %.1f°)  odom (%.3f, %.3f, %.1f°)\n", tr.x, tr.y,
                rad2deg(tr.th), map_end.x, map_end.y, rad2deg(map_end.th), odom_end.x, odom_end.y, rad2deg(odom_end.th));
    double worst = 0, sum = 0;
    for (const auto &k : kfs) {
      const Pose2 t = r.truthAt(k.stamp);
      const double e = std::hypot(k.T_map.x - t.x, k.T_map.y - t.y);
      sum += e;
      worst = std::max(worst, e);
      if (verbose && k.id % 5 == 0)
        std::printf("    kf %3d s%d %s map (%.3f, %.3f, %6.1f°) thật (%.3f, %.3f, %6.1f°) lệch %.3f\n", k.id, k.session,
                    k.blind ? "mù" : "  ", k.T_map.x, k.T_map.y, rad2deg(k.T_map.th), t.x, t.y, rad2deg(t.th), e);
    }
    if (std::getenv("SIM_DEBUG_KF")) {
      int lo = 0, hi = 0;
      std::sscanf(std::getenv("SIM_DEBUG_KF"), "%d-%d", &lo, &hi);
      for (const auto &e : sys.odomEdges())
        if (e.b >= lo && e.b <= hi) {
          const Pose2 tr = Pose2::between(r.truthAt(kfs[e.a].stamp), r.truthAt(kfs[e.b].stamp));
          std::printf("    ODOM %d->%d meas (%.3f,%.3f,%.1f°) thật (%.3f,%.3f,%.1f°) σ %.3f\n", e.a, e.b, e.meas.x, e.meas.y,
                      rad2deg(e.meas.th), tr.x, tr.y, rad2deg(tr.th), std::sqrt(e.cov(0, 0)));
        }
      for (const auto &e : sys.visualEdges())
        if (e.b >= lo && e.b <= hi) {
          const Pose2 m = baseMotion(sys.camera(), e.alpha, e.tx, e.ty, e.f, sys.heightUsed(), kfs[e.a].pitch, kfs[e.b].pitch);
          const Pose2 tr = Pose2::between(r.truthAt(kfs[e.a].stamp), r.truthAt(kfs[e.b].stamp));
          std::printf("    VIS%d %d->%d meas (%.3f,%.3f,%.1f°) thật (%.3f,%.3f,%.1f°) inl %d\n", int(e.kind), e.a, e.b, m.x, m.y,
                      rad2deg(m.th), tr.x, tr.y, rad2deg(tr.th), e.inliers);
        }
      for (const auto &k : kfs)
        if (k.id >= lo && k.id <= hi) {
          const Pose2 t = r.truthAt(k.stamp);
          std::printf("    kf %d t=%.2f map (%.3f,%.3f,%.1f°) thật (%.3f,%.3f,%.1f°) odom (%.3f,%.3f,%.1f°)\n", k.id, k.stamp,
                      k.T_map.x, k.T_map.y, rad2deg(k.T_map.th), t.x, t.y, rad2deg(t.th), k.T_odom.x, k.T_odom.y,
                      rad2deg(k.T_odom.th));
        }
    }
    std::printf("  lệch keyframe so với thật: TB %.3f m, lớn nhất %.3f m\n", kfs.empty() ? 0 : sum / kfs.size(), worst);
    const double e_map = std::hypot(map_end.x - tr.x, map_end.y - tr.y);
    const double e_odom = std::hypot(odom_end.x - tr.x, odom_end.y - tr.y);
    std::printf("  lệch vị trí cuối: SLAM %.3f m, chỉ odometry %.3f m\n", e_map, e_odom);
    if (sys.edgeCount(EdgeKind::Loop) < 1) std::printf("  FAIL: không có loop closure\n"), pass = false;
    if (e_map > 0.10) std::printf("  FAIL: lệch SLAM > 10 cm\n"), pass = false;
  }

  // ------------------------------------------------------------------ Pha B
  {
    std::printf("== Pha B: tái định vị + goto ==\n");
    sp.stuck.clear();
    sp.seed = 2;   // nhiễu khác, cùng trần (tex_seed)
    SlamSystem sys(cfg, opt, printer);
    sim::SimWorld w(cfg, sp, Pose2(1.6, 0.05, deg2rad(20)), 1000.0);
    sim::SimRunner r(sys, w);
    r.run(0, 0, 1.0);
    r.command("load_map sim localization");
    const bool loc = r.waitLocalized(30.0);
    const Pose2 mp = sys.mapPose(), tr = w.truth();
    std::printf("  tái định vị: %s; thật (%.3f, %.3f, %.1f°) map (%.3f, %.3f, %.1f°)\n", loc ? "OK" : "THẤT BẠI", tr.x,
                tr.y, rad2deg(tr.th), mp.x, mp.y, rad2deg(mp.th));
    if (!loc) {
      std::printf("FAIL\n");
      return 1;
    }
    for (const char *name : {"goc2", "xuat_phat", "goc3"}) {
      r.command(std::string("goto_place ") + name);
      const bool ok = r.navigate(120.0);
      const Pose2 t = w.truth(), want = truth_of[name];
      const double e = std::hypot(t.x - want.x, t.y - want.y);
      std::printf("  goto %-10s: %s, lệch thật %.3f m, hướng %.1f°  (%s)\n", name, ok ? "tới" : "KHÔNG tới", e,
                  rad2deg(wrapAngle(t.th - want.th)), sys.navReason().c_str());
      if (!ok || e > 0.15) pass = false;
      r.command("nav_clear");
    }
    // Nhiệm vụ: chuỗi điểm, có quay đầu 180° ở goc2 rồi đi lại đoạn cũ
    {
      std::map<std::string, Place> pl;
      for (const auto &p : sys.places()) pl[p.name] = p;
      const std::vector<std::string> seq = {"goc1", "goc2", "goc1", "xuat_phat"};
      std::string cmd = "mission";
      for (const auto &n : seq) cmd += " " + std::to_string(pl[n].x) + " " + std::to_string(pl[n].y);
      const double t0 = w.t();
      r.command(cmd);
      const bool ok = r.navigate(240.0);
      const double t1 = w.t();
      std::printf("  nhiệm vụ %zu điểm: %s (%s)\n", seq.size(), ok ? "xong" : "KHÔNG xong", sys.navReason().c_str());
      if (!ok) pass = false;
      for (size_t k = 0; k < seq.size(); k++) {
        const double d = r.minDist(truth_of[seq[k]].t(), t0, t1);
        std::printf("    điểm %zu %-10s: xe đi qua cách %.3f m\n", k + 1, seq[k].c_str(), d);
        if (d > 0.15) pass = false;
      }
      const Pose2 t = w.truth();
      const double e = std::hypot(t.x - truth_of["xuat_phat"].x, t.y - truth_of["xuat_phat"].y);
      std::printf("    dừng cách điểm cuối %.3f m\n", e);
      if (e > 0.15) pass = false;
      r.command("nav_clear");
    }
  }
  std::printf(pass ? "PASS\n" : "FAIL\n");
  return pass ? 0 : 1;
}
