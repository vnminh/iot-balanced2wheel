#pragma once
#include <Arduino.h>
#include <Wire.h>

// Driver tối giản cho MPU-6050 (đọc thanh ghi trực tiếp, không cần thư viện ngoài)
class MPU6050 {
public:
  // Khởi tạo: gyro +-1000 dps, accel +-4g, lọc thông thấp ~42 Hz
  bool begin(TwoWire &wire, uint8_t addr);
  uint8_t whoAmI();

  // acc: đơn vị g, gyro: độ/giây (đã trừ offset).
  // Trả về false nếu lỗi I2C hoặc dữ liệu toàn 0 (chip đang ngủ / vừa bị reset).
  bool read(float acc[3], float gyro[3]);

  enum Error : uint8_t { OK = 0, I2C_FAIL = 1, ALL_ZERO = 2 };
  Error lastError() const { return _lastError; }

  bool readReg(uint8_t reg, uint8_t &val);
  bool readRaw(int16_t raw[7]);  // AX AY AZ TEMP GX GY GZ (giá trị thô)

  // Đo offset gyro: XE PHẢI ĐỨNG YÊN trong lúc gọi hàm này.
  // Trả về false nếu phát hiện xe bị rung/lắc trong lúc đo.
  bool calibrateGyro(uint16_t samples = 1000);

  // Mẫu thô của lần read() thành công gần nhất: AX AY AZ TEMP GX GY GZ
  const int16_t *lastRaw() const { return _raw; }
  // Offset gyro đo được lúc hiệu chỉnh, đơn vị LSB (để trừ khi gửi sang Pi)
  float gyroBiasLsb(int axis) const;

private:
  bool writeReg(uint8_t reg, uint8_t val);

  Error _lastError = OK;
  TwoWire *_wire = nullptr;
  uint8_t _addr = 0x68;
  float _gyroBias[3] = {0, 0, 0};
  int16_t _raw[7] = {0};
};
