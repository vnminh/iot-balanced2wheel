#pragma once
#include <Arduino.h>

// Cảm biến siêu âm HC-SR04 (bảng SONAR_SENSORS trong config.h, số lượng tuỳ ý tới 8).
// Phát TRIG lần lượt từng cảm biến, đo độ rộng xung ECHO bằng ngắt -> không chặn vòng cân bằng 400 Hz.
// Mỗi số đo xong gửi Pi một khung SONAR; bảng vị trí gửi bằng khung SONAR_GEOM.
namespace Sonar {

constexpr uint16_t NO_ECHO = 0;        // không có vang trong tầm (phía trước trống)
constexpr uint16_t FAULT = 0xFFFF;     // ECHO không lên: lỏng dây / cảm biến hỏng

void begin();
void poll();                 // gọi mỗi vòng loop(); không bao giờ chặn quá ~12 µs
int count();
uint16_t lastMm(int k);
// Dừng khẩn: có cảm biến (dừng_khẩn_mm > 0) vừa đo gần hơn ngưỡng (trung vị 3 lần đo gần nhất, còn mới).
// Trả chỉ số cảm biến gây chặn, -1 nếu không chặn.
int blockingSensor();
void sendGeometry();         // gửi bảng vị trí cảm biến cho Pi

}  // namespace Sonar
