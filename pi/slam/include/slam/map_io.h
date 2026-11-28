#pragma once
// M11: lưu / tải bản đồ, thư mục maps/<id>/
//   keyframes.bin        id, T_odom, T_map, pitch, keypoint đã chỉnh, descriptor, path_rel
//   graph.txt            cạnh odometry + cạnh thị giác (dạng pixel) + chiều cao trần
//   grid.png/.yaml       center_layer (định dạng ROS map_server), grid_footprint.png
//   places.yaml          địa điểm có tên
//   sonar.bin            số đo siêu âm neo keyframe + vùng đã tẩy (lớp vật cản tĩnh, dựng lại khi tải)
//   obstacles.png        ảnh lớp vật cản tĩnh lúc lưu (cùng lưới với grid.png, 0 = vật cản), để xem
//   config_snapshot.yaml cấu hình lúc tạo bản đồ
//   meta.json            tóm tắt cho web
#include <string>
#include <vector>

#include "slam/keyframe.h"
#include "slam/obstacle_grid.h"
#include "slam/traversed_grid.h"

namespace slam {

struct Place {
  std::string name;
  double x = 0, y = 0, th = 0;
  bool has_th = false;
  int kf = -1;      // neo vào keyframe gần nhất: tối ưu đồ thị làm keyframe dịch thì địa điểm dịch theo
  Pose2 rel;        // tư thế địa điểm trong khung keyframe đó
};

struct MapData {
  std::vector<Keyframe> kfs;
  std::vector<OdomEdge> odom;
  std::vector<VisualEdge> vis;
  std::vector<Place> places;
  double height_est = 0;        // chiều cao trần tự ước lượng lúc lưu (0 = chưa có)
  StaticObstacles obstacles;    // lớp vật cản tĩnh (số đo gốc)
  cv::Mat obstacle_png;         // CV_8U cùng kích thước grid, 0 = vật cản, 255 = còn lại (chỉ để xem)
};

// Ghi vào <maps_dir>/<id>.tmp rồi đổi tên (mất điện giữa chừng không làm hỏng bản cũ)
bool saveMap(const std::string &maps_dir, const std::string &id, const MapData &m, const TraversedGrid &grid,
             const std::string &config_text, const std::string &meta_json, std::string &err);
bool loadMap(const std::string &dir, MapData &m, std::string &err);
bool savePlaces(const std::string &dir, const std::vector<Place> &places);
std::vector<Place> loadPlaces(const std::string &file);

}  // namespace slam
