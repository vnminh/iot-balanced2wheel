// slam_node: lõi SLAM 2D chạy như tiến trình con của robotd.
//
//   Chạy thật:  slam_node --config slam.yaml --maps ~/iot-ck/maps --shm /dev/shm/slam [--log file.slamlog]
//               đọc bản ghi nhị phân từ stdin (protocol.h), ghi JSON từng dòng ra stdout.
//   Replay:     slam_node --config slam.yaml --replay file.slamlog [--maps dir] [--end "save_map x"]
//               chạy đồng bộ (xác định), nhanh hơn thời gian thực.
#include <poll.h>
#include <unistd.h>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <thread>

#include "slam/config.h"
#include "slam/protocol.h"
#include "slam/system.h"

using namespace slam;

namespace {

void usage() {
  std::fprintf(stderr,
               "slam_node --config slam.yaml [--maps DIR] [--shm DIR] [--log FILE] [--replay FILE]\n"
               "          [--load ID] [--mode localization|mapping] [--cmd LỆNH]... [--end LỆNH]... [--quiet]\n");
}

double monoNow() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Trả về false nếu là bản ghi thoát
bool dispatch(SlamSystem &sys, const Record &r, FILE *log) {
  switch (r.type) {
    case 'M':
      if (r.data.size() >= sizeof(McuRecord)) {
        McuRecord m;
        std::memcpy(&m, r.data.data(), sizeof m);
        sys.onMcu(m);
      }
      if (log) writeRecord(log, 'M', r.data.data(), uint32_t(r.data.size()));
      break;
    case 'I': {
      if (r.data.size() < 12) break;
      double t;
      uint16_t w, h;
      std::memcpy(&t, r.data.data(), 8);
      std::memcpy(&w, r.data.data() + 8, 2);
      std::memcpy(&h, r.data.data() + 10, 2);
      if (r.data.size() < 12 + size_t(w) * h) break;
      cv::Mat g = cv::Mat(h, w, CV_8U, const_cast<uint8_t *>(r.data.data() + 12)).clone();
      sys.onImage(t, g);
      if (log) {
        std::vector<uint8_t> jpg;
        cv::imencode(".jpg", g, jpg, {cv::IMWRITE_JPEG_QUALITY, 90});
        std::vector<uint8_t> rec(8 + jpg.size());
        std::memcpy(rec.data(), &t, 8);
        std::memcpy(rec.data() + 8, jpg.data(), jpg.size());
        writeRecord(log, 'J', rec.data(), uint32_t(rec.size()));
      }
      break;
    }
    case 'J': {
      if (r.data.size() < 9) break;
      double t;
      std::memcpy(&t, r.data.data(), 8);
      std::vector<uint8_t> jpg(r.data.begin() + 8, r.data.end());
      cv::Mat g = cv::imdecode(jpg, cv::IMREAD_GRAYSCALE);
      if (!g.empty()) sys.onImage(t, g);
      if (log) writeRecord(log, 'J', r.data.data(), uint32_t(r.data.size()));
      break;
    }
    case 'C': {
      std::string line(r.data.begin(), r.data.end());
      sys.command(line);
      if (log) writeRecord(log, 'C', r.data.data(), uint32_t(r.data.size()));
      break;
    }
    case 'Q':
      return false;
    default:
      break;
  }
  return true;
}

}  // namespace

int main(int argc, char **argv) {
  std::string config, maps = "maps", shm, logfile, replay, load, load_mode = "localization";
  std::vector<std::string> cmds, ends;
  bool quiet = false;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--config") config = next();
    else if (a == "--maps") maps = next();
    else if (a == "--shm") shm = next();
    else if (a == "--log") logfile = next();
    else if (a == "--replay") replay = next();
    else if (a == "--load") load = next();
    else if (a == "--mode") load_mode = next();
    else if (a == "--cmd") cmds.push_back(next());
    else if (a == "--end") ends.push_back(next());
    else if (a == "--quiet") quiet = true;
    else {
      usage();
      return 2;
    }
  }

  Config cfg;
  try {
    cfg = config.empty() ? Config::defaults() : Config::load(config);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "Lỗi cấu hình: %s\n", e.what());
    return 1;
  }

  std::setvbuf(stdout, nullptr, _IOFBF, 1 << 16);
  SlamSystem::Options opt;
  opt.maps_dir = maps;
  opt.shm_dir = shm;
  opt.synchronous = !replay.empty();
  size_t n_out = 0;
  SlamSystem sys(cfg, opt, [&](const std::string &s) {
    ++n_out;
    if (quiet) return;
    std::fwrite(s.data(), 1, s.size(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
  });
  if (!load.empty()) sys.command("load_map " + load + " " + load_mode);
  for (const auto &c : cmds) sys.command(c);

  // ------------------------------------------------------------------ replay
  if (!replay.empty()) {
    FILE *f = std::fopen(replay.c_str(), "rb");
    if (!f) {
      std::fprintf(stderr, "Không mở được %s\n", replay.c_str());
      return 1;
    }
    char magic[8];
    if (std::fread(magic, 1, 8, f) != 8 || std::memcmp(magic, "SLAMLOG1", 8) != 0) {
      std::fprintf(stderr, "%s không phải log SLAM (thiếu SLAMLOG1)\n", replay.c_str());
      return 1;
    }
    Record r;
    size_t n = 0;
    const auto t0 = std::chrono::steady_clock::now();
    while (readRecord(f, r)) {
      ++n;
      if (!dispatch(sys, r, nullptr)) break;
    }
    std::fclose(f);
    for (const auto &c : ends) sys.command(c);
    sys.command("status");
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    const Pose2 p = sys.mapPose();
    std::fprintf(stderr, "replay: %zu bản ghi, %.1f s; %zu keyframe, loop %d, lân cận %d, trượt %d; pose (%.3f, %.3f, %.1f°)\n",
                 n, secs, sys.keyframes().size(), sys.edgeCount(EdgeKind::Loop), sys.edgeCount(EdgeKind::Neighbor),
                 sys.slipCount(), p.x, p.y, rad2deg(p.th));
    return 0;
  }

  // ---------------------------------------------------------------- chạy thật
  FILE *log = nullptr;
  if (!logfile.empty()) {
    log = std::fopen(logfile.c_str(), "wb");
    if (log) std::fwrite("SLAMLOG1", 1, 8, log);
    else std::fprintf(stderr, "Không ghi được log %s\n", logfile.c_str());
  }
  // Luồng đọc stdin -> hàng đợi; luồng chính xử lý (và kiểm tra mất dữ liệu khi stdin im lặng)
  std::deque<Record> q;
  std::mutex qm;
  std::condition_variable qcv;
  bool eof = false;
  std::thread reader([&] {
    Record r;
    while (readRecord(stdin, r)) {
      {
        std::lock_guard<std::mutex> lk(qm);
        // Ảnh chưa kịp xử lý thì bỏ (luôn giữ thời gian thực); MCU / lệnh không bao giờ bỏ
        if (r.type == 'I')
          for (auto it = q.begin(); it != q.end();)
            it = (it->type == 'I') ? q.erase(it) : it + 1;
        q.push_back(std::move(r));
      }
      qcv.notify_one();
      r = Record();
    }
    {
      std::lock_guard<std::mutex> lk(qm);
      eof = true;
    }
    qcv.notify_one();
  });
  bool run = true;
  size_t since_flush = 0;
  while (run) {
    Record r;
    {
      std::unique_lock<std::mutex> lk(qm);
      qcv.wait_for(lk, std::chrono::milliseconds(50), [&] { return eof || !q.empty(); });
      if (q.empty()) {
        if (eof) break;
        lk.unlock();
        sys.wallTick(monoNow());
        continue;
      }
      r = std::move(q.front());
      q.pop_front();
    }
    run = dispatch(sys, r, log);
    if (log && ++since_flush > 400) {
      std::fflush(log);
      since_flush = 0;
    }
  }
  if (log) std::fclose(log);
  std::_Exit(0);  // luồng đọc có thể đang chặn ở fread(stdin)
}
