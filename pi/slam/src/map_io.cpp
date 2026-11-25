#include "slam/map_io.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace slam {

namespace {

const char KF_MAGIC[8] = {'K', 'F', 'D', 'B', '0', '0', '0', '1'};

template <typename T>
void put(std::ofstream &f, const T &v) {
  f.write(reinterpret_cast<const char *>(&v), sizeof(T));
}
template <typename T>
bool take(std::ifstream &f, T &v) {
  return bool(f.read(reinterpret_cast<char *>(&v), sizeof(T)));
}
void putPose(std::ofstream &f, const Pose2 &p) {
  put(f, p.x), put(f, p.y), put(f, p.th);
}
bool takePose(std::ifstream &f, Pose2 &p) {
  double x, y, th;
  if (!take(f, x) || !take(f, y) || !take(f, th)) return false;
  p = Pose2(x, y, th);
  return true;
}

// Ghi xong thì fsync: thẻ nhớ + mất điện đã làm file rỗng 0 byte trên xe này
void syncFile(const fs::path &p) {
  int fd = ::open(p.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
}

bool writeKeyframes(const fs::path &p, const std::vector<Keyframe> &kfs) {
  std::ofstream f(p, std::ios::binary);
  f.write(KF_MAGIC, 8);
  put(f, uint32_t(kfs.size()));
  for (const auto &k : kfs) {
    put(f, int32_t(k.id));
    put(f, k.stamp);
    putPose(f, k.T_odom);
    putPose(f, k.T_map);
    put(f, k.pitch);
    put(f, uint8_t(k.blind));
    put(f, int32_t(k.session));
    put(f, uint32_t(k.pts.size()));
    for (const auto &q : k.pts) put(f, q.x), put(f, q.y);
    const cv::Mat d = k.desc.isContinuous() ? k.desc : k.desc.clone();
    put(f, uint32_t(d.rows));
    put(f, uint32_t(d.cols));
    if (d.rows) f.write(reinterpret_cast<const char *>(d.data), std::streamsize(d.total()));
    put(f, uint32_t(k.path_rel.size()));
    for (const auto &q : k.path_rel) putPose(f, q);
  }
  return bool(f);
}

bool readKeyframes(const fs::path &p, std::vector<Keyframe> &kfs) {
  std::ifstream f(p, std::ios::binary);
  char magic[8];
  if (!f.read(magic, 8) || std::memcmp(magic, KF_MAGIC, 8) != 0) return false;
  uint32_t n;
  if (!take(f, n)) return false;
  kfs.clear();
  kfs.reserve(n);
  for (uint32_t i = 0; i < n; i++) {
    Keyframe k;
    int32_t id, session;
    uint8_t blind;
    uint32_t np, rows, cols, npath;
    if (!take(f, id) || !take(f, k.stamp) || !takePose(f, k.T_odom) || !takePose(f, k.T_map) || !take(f, k.pitch) ||
        !take(f, blind) || !take(f, session) || !take(f, np))
      return false;
    k.id = id, k.session = session, k.blind = blind;
    k.pts.resize(np);
    for (auto &q : k.pts)
      if (!take(f, q.x) || !take(f, q.y)) return false;
    if (!take(f, rows) || !take(f, cols)) return false;
    k.desc = cv::Mat(int(rows), int(cols), CV_8U);
    if (rows && !f.read(reinterpret_cast<char *>(k.desc.data), std::streamsize(size_t(rows) * cols))) return false;
    if (!take(f, npath)) return false;
    k.path_rel.resize(npath);
    for (auto &q : k.path_rel)
      if (!takePose(f, q)) return false;
    if (k.id != int(i)) return false;  // id phải liên tục 0..n-1
    kfs.push_back(std::move(k));
  }
  return true;
}

}  // namespace

bool savePlaces(const std::string &dir, const std::vector<Place> &places) {
  YAML::Emitter y;
  y << YAML::BeginMap;
  for (const auto &p : places) {
    y << YAML::Key << p.name << YAML::Value << YAML::Flow << YAML::BeginMap << YAML::Key << "x" << YAML::Value << p.x
      << YAML::Key << "y" << YAML::Value << p.y;
    if (p.has_th) y << YAML::Key << "theta" << YAML::Value << p.th;
    if (p.kf >= 0)
      y << YAML::Key << "kf" << YAML::Value << p.kf << YAML::Key << "rel" << YAML::Value << YAML::Flow << YAML::BeginSeq
        << p.rel.x << p.rel.y << p.rel.th << YAML::EndSeq;
    y << YAML::EndMap;
  }
  y << YAML::EndMap;
  const fs::path tmp = fs::path(dir) / "places.yaml.tmp", dst = fs::path(dir) / "places.yaml";
  {
    std::ofstream f(tmp);
    f << "# Địa điểm có tên (frame map): x, y (m), theta (rad, tuỳ chọn); kf + rel = neo vào keyframe\n" << (places.empty() ? "{}" : y.c_str())
      << "\n";
    if (!f) return false;
  }
  syncFile(tmp);
  std::error_code ec;
  fs::rename(tmp, dst, ec);
  return !ec;
}

std::vector<Place> loadPlaces(const std::string &file) {
  std::vector<Place> out;
  try {
    YAML::Node y = YAML::LoadFile(file);
    for (auto it = y.begin(); it != y.end(); ++it) {
      Place p;
      p.name = it->first.as<std::string>();
      p.x = it->second["x"].as<double>();
      p.y = it->second["y"].as<double>();
      if (it->second["theta"]) p.th = it->second["theta"].as<double>(), p.has_th = true;
      if (it->second["kf"] && it->second["rel"]) {
        const auto r = it->second["rel"].as<std::vector<double>>();
        if (r.size() == 3) p.kf = it->second["kf"].as<int>(), p.rel = Pose2(r[0], r[1], r[2]);
      }
      out.push_back(p);
    }
  } catch (const std::exception &) {
  }
  return out;
}

bool saveMap(const std::string &maps_dir, const std::string &id, const MapData &m, const TraversedGrid &grid,
             const std::string &config_text, const std::string &meta_json, std::string &err) {
  std::error_code ec;
  const fs::path dst = fs::path(maps_dir) / id, tmp = fs::path(maps_dir) / (id + ".tmp");
  fs::remove_all(tmp, ec);
  fs::create_directories(tmp, ec);
  if (ec) {
    err = "Không tạo được thư mục " + tmp.string() + ": " + ec.message();
    return false;
  }
  if (!writeKeyframes(tmp / "keyframes.bin", m.kfs)) {
    err = "Lỗi ghi keyframes.bin";
    return false;
  }
  {
    std::ofstream g(tmp / "graph.txt");
    g << "# iot-ck slam graph v1\n";
    g << "# ODOM a b x y th c00 c01 c02 c11 c12 c22\n# VIS a b alpha tx ty f inliers kind(0 lân cận,1 loop,2 tái định vị)\n";
    char b[512];
    std::snprintf(b, sizeof b, "HEIGHT %.6f\n", m.height_est);
    g << b;
    for (const auto &e : m.odom) {
      std::snprintf(b, sizeof b, "ODOM %d %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", e.a, e.b, e.meas.x, e.meas.y,
                    e.meas.th, e.cov(0, 0), e.cov(0, 1), e.cov(0, 2), e.cov(1, 1), e.cov(1, 2), e.cov(2, 2));
      g << b;
    }
    for (const auto &e : m.vis) {
      std::snprintf(b, sizeof b, "VIS %d %d %.9g %.9g %.9g %.9g %d %d\n", e.a, e.b, e.alpha, e.tx, e.ty, e.f, e.inliers,
                    int(e.kind));
      g << b;
    }
    if (!g) {
      err = "Lỗi ghi graph.txt";
      return false;
    }
  }
  if (!grid.empty() && !grid.saveMapServer(tmp.string())) {
    err = "Lỗi ghi grid.png";
    return false;
  }
  if (!savePlaces(tmp.string(), m.places)) {
    err = "Lỗi ghi places.yaml";
    return false;
  }
  {
    std::ofstream c(tmp / "config_snapshot.yaml");
    c << config_text;
    std::ofstream j(tmp / "meta.json");
    j << meta_json << "\n";
  }
  for (const auto &e : fs::directory_iterator(tmp)) syncFile(e.path());
  // Thay bản cũ (nếu có): đổi tên bản cũ trước, xong mới xoá
  const fs::path old = fs::path(maps_dir) / (id + ".old");
  fs::remove_all(old, ec);
  if (fs::exists(dst)) fs::rename(dst, old, ec);
  fs::rename(tmp, dst, ec);
  if (ec) {
    err = "Không đổi tên thư mục bản đồ: " + ec.message();
    return false;
  }
  fs::remove_all(old, ec);
  ::sync();
  return true;
}

bool loadMap(const std::string &dir, MapData &m, std::string &err) {
  m = MapData();
  if (!readKeyframes(fs::path(dir) / "keyframes.bin", m.kfs)) {
    err = "keyframes.bin hỏng hoặc không có";
    return false;
  }
  std::ifstream g(fs::path(dir) / "graph.txt");
  if (!g) {
    err = "Không có graph.txt";
    return false;
  }
  std::string line;
  const int n = int(m.kfs.size());
  while (std::getline(g, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag == "HEIGHT") {
      ss >> m.height_est;
    } else if (tag == "ODOM") {
      OdomEdge e;
      double x, y, th, c00, c01, c02, c11, c12, c22;
      ss >> e.a >> e.b >> x >> y >> th >> c00 >> c01 >> c02 >> c11 >> c12 >> c22;
      if (!ss || e.a < 0 || e.b < 0 || e.a >= n || e.b >= n) continue;
      e.meas = Pose2(x, y, th);
      e.cov << c00, c01, c02, c01, c11, c12, c02, c12, c22;
      m.odom.push_back(e);
    } else if (tag == "VIS") {
      VisualEdge e;
      int kind;
      ss >> e.a >> e.b >> e.alpha >> e.tx >> e.ty >> e.f >> e.inliers >> kind;
      if (!ss || e.a < 0 || e.b < 0 || e.a >= n || e.b >= n) continue;
      e.kind = EdgeKind(kind);
      m.vis.push_back(e);
    }
  }
  m.places = loadPlaces((fs::path(dir) / "places.yaml").string());
  return true;
}

}  // namespace slam
