#include "slam/traversed_grid.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <opencv2/imgcodecs.hpp>

namespace slam {

namespace {

// Tô mọi ô có tâm cách đoạn [a, b] không quá r
void stamp(cv::Mat &m, const TraversedGrid &g, const Eigen::Vector2d &a, const Eigen::Vector2d &b, double r) {
  const double eps = 1e-6;
  const int i0 = std::max(0, int(std::floor((std::min(a.x(), b.x()) - r - g.ox) / g.res)));
  const int i1 = std::min(g.nx - 1, int(std::floor((std::max(a.x(), b.x()) + r - g.ox) / g.res)));
  const int j0 = std::max(0, int(std::floor((std::min(a.y(), b.y()) - r - g.oy) / g.res)));
  const int j1 = std::min(g.ny - 1, int(std::floor((std::max(a.y(), b.y()) + r - g.oy) / g.res)));
  const Eigen::Vector2d ab = b - a;
  const double L2 = ab.squaredNorm();
  for (int j = j0; j <= j1; j++) {
    uchar *row = m.ptr<uchar>(j);
    for (int i = i0; i <= i1; i++) {
      const Eigen::Vector2d p = g.cellCenter(i, j);
      const double u = L2 > 0 ? std::clamp((p - a).dot(ab) / L2, 0.0, 1.0) : 0.0;
      if ((p - (a + u * ab)).norm() <= r + eps) row[i] = 255;
    }
  }
}

}  // namespace

void TraversedGrid::build(const std::vector<std::vector<Eigen::Vector2d>> &segments, double resolution,
                          double body_radius, double center_tol, double margin) {
  res = resolution;
  double x0 = 1e18, y0 = 1e18, x1 = -1e18, y1 = -1e18;
  for (const auto &s : segments)
    for (const auto &p : s) {
      x0 = std::min(x0, p.x()), x1 = std::max(x1, p.x());
      y0 = std::min(y0, p.y()), y1 = std::max(y1, p.y());
    }
  if (x0 > x1) {
    nx = ny = 0;
    center.release();
    footprint.release();
    return;
  }
  const double m = margin + body_radius;
  ox = std::floor((x0 - m) / res) * res;
  oy = std::floor((y0 - m) / res) * res;
  nx = int(std::ceil((x1 + m - ox) / res)) + 1;
  ny = int(std::ceil((y1 + m - oy) / res)) + 1;
  center = cv::Mat::zeros(ny, nx, CV_8U);
  footprint = cv::Mat::zeros(ny, nx, CV_8U);
  for (const auto &s : segments) {
    if (s.size() == 1) {
      stamp(center, *this, s[0], s[0], center_tol);
      stamp(footprint, *this, s[0], s[0], body_radius);
    }
    for (size_t k = 1; k < s.size(); k++) {
      stamp(center, *this, s[k - 1], s[k], center_tol);
      stamp(footprint, *this, s[k - 1], s[k], body_radius);
    }
  }
}

bool TraversedGrid::toCell(const Eigen::Vector2d &p, int &i, int &j) const {
  i = int(std::floor((p.x() - ox) / res));
  j = int(std::floor((p.y() - oy) / res));
  return i >= 0 && j >= 0 && i < nx && j < ny;
}

size_t TraversedGrid::freeCells() const { return empty() ? 0 : size_t(cv::countNonZero(center)); }

bool TraversedGrid::saveMapServer(const std::string &dir) const {
  if (empty()) return false;
  auto out = [&](const cv::Mat &m, const std::string &name) {
    cv::Mat img;
    cv::flip(m, img, 0);  // hàng 0 = y lớn nhất
    img.setTo(254, img == 255);
    return cv::imwrite(dir + "/" + name, img);
  };
  if (!out(center, "grid.png") || !out(footprint, "grid_footprint.png")) return false;
  YAML::Emitter y;
  y << YAML::BeginMap << YAML::Key << "image" << YAML::Value << "grid.png" << YAML::Key << "resolution"
    << YAML::Value << res << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq << ox << oy << 0.0
    << YAML::EndSeq << YAML::Key << "negate" << YAML::Value << 0 << YAML::Key << "occupied_thresh" << YAML::Value
    << 0.65 << YAML::Key << "free_thresh" << YAML::Value << 0.196 << YAML::Key << "mode" << YAML::Value << "trinary"
    << YAML::EndMap;
  std::ofstream f(dir + "/grid.yaml");
  f << "# Bản đồ vùng đã đi (center_layer) cho planner. Trắng = đi được, đen = chặn.\n" << y.c_str() << "\n";
  return bool(f);
}

bool TraversedGrid::loadMapServer(const std::string &dir) {
  try {
    YAML::Node y = YAML::LoadFile(dir + "/grid.yaml");
    res = y["resolution"].as<double>();
    auto o = y["origin"].as<std::vector<double>>();
    ox = o.at(0), oy = o.at(1);
    cv::Mat c = cv::imread(dir + "/grid.png", cv::IMREAD_GRAYSCALE), fp = cv::imread(dir + "/grid_footprint.png",
                                                                                     cv::IMREAD_GRAYSCALE);
    if (c.empty()) return false;
    if (fp.empty() || fp.size() != c.size()) fp = c.clone();
    cv::flip(c, center, 0);
    cv::flip(fp, footprint, 0);
    center = (center > 127);
    footprint = (footprint > 127);
    nx = center.cols, ny = center.rows;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

cv::Mat TraversedGrid::renderWeb() const {
  if (empty()) return {};
  // ảnh web: cao = nx (x giảm dần xuống dưới), rộng = ny (y giảm dần sang phải)
  cv::Mat img(nx, ny, CV_8UC4, cv::Scalar(0, 0, 0, 0));
  for (int j = 0; j < ny; j++)
    for (int i = 0; i < nx; i++) {
      cv::Vec4b &px = img.at<cv::Vec4b>(nx - 1 - i, ny - 1 - j);
      if (center.at<uchar>(j, i))
        px = cv::Vec4b(150, 225, 120, 170);   // BGRA: xanh lá nhạt = đường tâm đi được
      else if (footprint.at<uchar>(j, i))
        px = cv::Vec4b(150, 120, 90, 110);    // xanh xám = vùng thân xe đã quét
    }
  return img;
}

}  // namespace slam
