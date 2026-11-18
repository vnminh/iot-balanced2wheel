#pragma once
#include <Arduino.h>

// Phát xung STEP/DIR cho 2 driver A4988 bằng ngắt timer phần cứng.
// Tốc độ được đặt liên tục (bước/giây, có dấu) từ vòng PID.
namespace Steppers {

enum { LEFT = 0, RIGHT = 1 };

void begin();
void enable(bool on);                    // bật/tắt dòng qua động cơ (chân EN)
bool isEnabled();
void setSpeed(int motor, float stepsPerSec);   // dương = xe chạy tới
int32_t position(int motor);             // tổng số bước đã chạy (để mở rộng sau)

}  // namespace Steppers
