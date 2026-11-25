#pragma once
// M8: bản đồ vùng đã đi. MỌI Ô MẶC ĐỊNH BỊ CHẶN, chỉ ô xe đã thật sự đi qua mới trống.
//   footprint: mọi ô dưới thân xe (bán kính body_radius) -> hiển thị
//   center   : ô tâm xe đi qua, nong center_tolerance   -> planner (KHÔNG inflation thêm)
// Lưới lưu theo hàng j tăng theo y (toán học); xuất PNG theo chuẩn map_server (hàng 0 = y lớn nhất).
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "slam/pose2.h"

namespace slam {

class TraversedGrid {
public:
  double res = 0.05, ox = 0, oy = 0;   // ô (0,0) có góc dưới-trái ở (ox, oy)
  int nx = 0, ny = 0;                  // số cột (theo x), số hàng (theo y)
  cv::Mat center, footprint;           // CV_8U ny×nx, 255 = trống, 0 = chặn

  // segments: các đoạn quỹ đạo liên tục trong frame map
  void build(const std::vector<std::vector<Eigen::Vector2d>> &segments, double resolution, double body_radius,
             double center_tol, double margin);
  bool empty() const { return nx == 0 || ny == 0; }
  bool toCell(const Eigen::Vector2d &p, int &i, int &j) const;
  Eigen::Vector2d cellCenter(int i, int j) const { return {ox + (i + 0.5) * res, oy + (j + 0.5) * res}; }
  bool free(int i, int j) const { return i >= 0 && j >= 0 && i < nx && j < ny && center.at<uchar>(j, i); }
  size_t freeCells() const;

  // Ghi grid.png + grid.yaml (center) và grid_footprint.png theo định dạng ROS map_server
  bool saveMapServer(const std::string &dir) const;
  bool loadMapServer(const std::string &dir);
  // Ảnh RGBA cho web: hàng 0 = x lớn nhất, cột 0 = y lớn nhất (khớp hướng vẽ của web: x lên, y sang trái)
  cv::Mat renderWeb() const;
};

}  // namespace slam
