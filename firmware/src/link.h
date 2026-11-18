#pragma once
#include <Arduino.h>

// Liên kết UART2 với Raspberry Pi.
//
// Pi -> ESP32: dòng text kết thúc '\n'
//     VEL:<v>,<t>   v = tốc độ tiến (m/s, + = tới), t = chênh lệch mỗi bánh
//                   (m/s, + = quay phải). Web gửi lệnh này 10 lần/giây.
//     CMD:<TÊN>     FORWARD BACKWARD LEFT RIGHT ROTATE ROTATE_LEFT STOP
//                   (tương thích uart_link.py / điều khiển giọng nói cũ)
//     PING          ESP32 đáp "PONG"
//   Lệnh nào cũng là nhịp tim: quá CMD_TIMEOUT_MS không nghe gì -> dừng.
//
// ESP32 -> Pi: khung nhị phân  A5 5A <type> <len> <payload...> <crc8>
//   crc8 (đa thức 0x07, khởi tạo 0) tính trên type, len, payload.
//   type 0x01 IMU (28 byte, little-endian):
//       uint32 t_us        micros() lúc đọc xong MPU
//       int16  acc[3]      thô, 8192 LSB/g  (khung cảm biến)
//       int16  gyro[3]     thô đã trừ offset, 32.8 LSB/(deg/s)
//       int32  steps[2]    tổng số bước trái, phải (+ = bánh lăn tới)
//       uint8  state       0 hiệu chỉnh, 1 chờ dựng, 2 đang cân bằng
//       uint8  flags       bit0 có lệnh từ Pi, bit1 động cơ đang bật
//       uint16 seq
//   type 0x02 TEXT: ACK:..., ST:READY|BALANCING|FALLEN|IMU_ERROR, INFO:..., PONG
namespace Link {

enum : uint8_t { T_IMU = 0x01, T_TEXT = 0x02 };

struct __attribute__((packed)) ImuPacket {
  uint32_t t_us;
  int16_t acc[3];
  int16_t gyro[3];
  int32_t steps[2];
  uint8_t state;
  uint8_t flags;
  uint16_t seq;
};
static_assert(sizeof(ImuPacket) == 28, "ImuPacket phai 28 byte");

void begin();
void poll();  // gọi mỗi vòng loop(); không bao giờ chặn

// Lệnh đã được giới hạn và làm mượt (gọi mỗi chu kỳ điều khiển với dt)
void update(float dt);
float speed();     // m/s, + = tới
float turn();      // m/s chênh lệch mỗi bánh, + = quay phải
bool alive();      // còn nhận lệnh trong CMD_TIMEOUT_MS
bool consumeTargetChanged();  // true một lần mỗi khi đích lệnh đổi
void clear();      // dừng ngay (khi ngã / lỗi cảm biến)

void sendImu(const ImuPacket &p);
void sendText(const char *fmt, ...);
// Log debug: gửi khung TEXT "LOG:..." (và in ra USB nếu USB không dùng cho Pi)
void log(const char *fmt, ...);

}  // namespace Link
