#pragma once
// Giao thức stdin của slam_node, cũng là định dạng file log (M12).
//
//   bản ghi = <type u8> <len u32 LE> <payload len byte>
//   'M' mẫu MCU   : McuRecord (52 byte, little-endian, packed)
//   'S' siêu âm   : SonarRecord (11 byte: f64 t, u8 idx, u16 mm), xem slam/sonar.h
//   'G' vị trí cảm biến siêu âm: nguyên khung SONAR_GEOM của ESP32 (firmware 2.2)
//   'I' ảnh xám   : f64 t, u16 w, u16 h, w*h byte
//   'J' ảnh JPEG  : f64 t, dữ liệu JPEG (log ghi ảnh dạng này cho nhỏ)
//   'C' lệnh text : UTF-8, một dòng (xem SlamSystem::command)
//   'Q' thoát
// File log bắt đầu bằng 8 byte "SLAMLOG1" rồi tới chuỗi bản ghi như trên.
// Đọc bằng Python: pi/tools/slamlog.py
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace slam {

#pragma pack(push, 1)
struct McuRecord {
  double t;          // s, đồng hồ monotonic của Pi (đã quy đổi từ micros() của ESP32)
  uint32_t t_us;     // micros() ESP32 (chỉ để kiểm tra)
  uint16_t seq;
  int32_t steps[2];  // tổng tích luỹ trái, phải (+ = lăn tới)
  float gyro[3];     // rad/s, khung MPU (X tới, Y trái, Z lên)
  float acc[3];      // m/s^2
  float pitch;       // rad, + = chúi tới; NaN nếu firmware cũ không gửi
  uint8_t state;     // 0 hiệu chỉnh, 1 chờ dựng, 2 đang cân bằng
  uint8_t flags;     // bit0 lệnh Pi còn sống, bit1 động cơ bật, bit2 chạm giới hạn
};
#pragma pack(pop)
static_assert(sizeof(McuRecord) == 52, "McuRecord phải 52 byte");

enum : uint8_t { MCU_BALANCING = 2 };
enum : uint8_t { FLAG_CMD_ALIVE = 1, FLAG_MOTORS = 2, FLAG_SATURATED = 4 };

struct Record {
  char type = 0;
  std::vector<uint8_t> data;
};

// Đọc một bản ghi; false khi hết dữ liệu / lỗi.
bool readRecord(FILE *f, Record &r);
void writeRecord(FILE *f, char type, const void *data, uint32_t len);

}  // namespace slam
