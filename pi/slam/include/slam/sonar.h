#pragma once
// Siêu âm: bảng vị trí cảm biến DO ESP32 GỬI (bản ghi 'G' = nguyên khung SONAR_GEOM của firmware 2.2) và số đo
// (bản ghi 'S'). slam_node không có cấu hình vị trí cảm biến nào.
//   SONAR_GEOM: u8 version(=1) · u8 count · u16 max_range_mm · count × {i16 x_mm, i16 y_mm, i16 yaw_cdeg, u8 half_deg, u16 stop_mm}
#include <cstdint>
#include <vector>

#include "slam/pose2.h"

namespace slam {

#pragma pack(push, 1)
struct SonarRecord {
  double t;        // s, đồng hồ monotonic của Pi (lúc ECHO kết thúc)
  uint8_t idx;     // chỉ số cảm biến
  uint16_t mm;     // 0 = không có vang trong tầm, 0xFFFF = lỗi cảm biến
};
#pragma pack(pop)
static_assert(sizeof(SonarRecord) == 11, "SonarRecord phải 11 byte");

constexpr uint16_t SONAR_NO_ECHO = 0, SONAR_FAULT = 0xFFFF;

struct SonarSensor {
  Pose2 mount;        // vị trí + hướng trong khung base_link (tâm trục bánh)
  double half = 0;    // rad
  int stop_mm = 0;
};

struct SonarGeometry {
  std::vector<SonarSensor> sensors;
  double max_range = 2.0;
  bool valid() const { return !sensors.empty(); }
  static bool parse(const uint8_t *d, size_t n, SonarGeometry &out);
};

// Khoảng trống phía trước (m): điểm trên cung tại d nằm trong hành lang |y| ≤ corridor, x > 0; trừ mép trước
// (x lớn nhất của các cảm biến). dists[k] ≤ 0 = bỏ qua. Không thấy gì -> +inf. Cùng công thức robot/sonar.py.
double frontClearance(const SonarGeometry &g, const std::vector<double> &dists, double corridor);

}  // namespace slam
