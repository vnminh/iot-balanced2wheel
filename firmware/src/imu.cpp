#include "imu.h"

namespace {
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;

constexpr float ACC_LSB_PER_G     = 8192.0f;  // +-4g
constexpr float GYRO_LSB_PER_DPS  = 32.8f;    // +-1000 dps (cú đẩy mạnh có thể vượt 500 dps)
}

bool MPU6050::writeReg(uint8_t reg, uint8_t val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(val);
  return _wire->endTransmission() == 0;
}

bool MPU6050::readReg(uint8_t reg, uint8_t &val) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom(_addr, (uint8_t)1) != 1) return false;
  val = _wire->read();
  return true;
}

uint8_t MPU6050::whoAmI() {
  uint8_t v = 0;
  return readReg(REG_WHO_AM_I, v) ? v : 0;
}

bool MPU6050::begin(TwoWire &wire, uint8_t addr) {
  _wire = &wire;
  _addr = addr;

  if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;  // reset chip
  delay(100);
  bool ok = true;
  ok &= writeReg(REG_PWR_MGMT_1, 0x01);    // thoát sleep, clock theo PLL gyro X
  delay(10);
  ok &= writeReg(REG_SMPLRT_DIV, 0x00);    // 1 kHz
  ok &= writeReg(REG_CONFIG, 0x03);        // DLPF: accel 44 Hz, gyro 42 Hz
  ok &= writeReg(REG_GYRO_CONFIG, 0x10);   // +-1000 dps
  ok &= writeReg(REG_ACCEL_CONFIG, 0x08);  // +-4 g
  delay(50);

  // Xác nhận chip đã thật sự thức (bit SLEEP = bit 6 của PWR_MGMT_1 phải = 0)
  uint8_t pwr = 0xFF;
  if (!readReg(REG_PWR_MGMT_1, pwr) || (pwr & 0x40)) return false;
  return ok;
}

bool MPU6050::readRaw(int16_t raw[7]) {
  _wire->beginTransmission(_addr);
  _wire->write(REG_ACCEL_XOUT_H);
  if (_wire->endTransmission(false) != 0) return false;
  if (_wire->requestFrom(_addr, (uint8_t)14) != 14) return false;
  for (int i = 0; i < 7; i++) {
    uint8_t hi = _wire->read();
    uint8_t lo = _wire->read();
    raw[i] = (int16_t)((hi << 8) | lo);
  }
  return true;  // raw: AX AY AZ TEMP GX GY GZ
}

bool MPU6050::read(float acc[3], float gyro[3]) {
  int16_t raw[7];
  if (!readRaw(raw)) {
    _lastError = I2C_FAIL;
    return false;
  }
  // Gia tốc kế luôn đo được trọng trường -> 3 trục cùng = 0 là vô lý:
  // chip đang ngủ (vừa bị mất nguồn chập chờn) hoặc hỏng.
  if (raw[0] == 0 && raw[1] == 0 && raw[2] == 0) {
    _lastError = ALL_ZERO;
    return false;
  }
  _lastError = OK;
  memcpy(_raw, raw, sizeof(_raw));
  for (int i = 0; i < 3; i++) {
    acc[i]  = raw[i] / ACC_LSB_PER_G;
    gyro[i] = raw[i + 4] / GYRO_LSB_PER_DPS - _gyroBias[i];
  }
  return true;
}

bool MPU6050::calibrateGyro(uint16_t samples) {
  double sum[3] = {0, 0, 0};
  double sumSq[3] = {0, 0, 0};
  uint16_t n = 0;
  int16_t raw[7];

  for (uint16_t i = 0; i < samples; i++) {
    if (readRaw(raw) && (raw[0] | raw[1] | raw[2])) {  // bỏ mẫu toàn 0 (chip ngủ)
      for (int k = 0; k < 3; k++) {
        double g = raw[k + 4] / GYRO_LSB_PER_DPS;
        sum[k] += g;
        sumSq[k] += g * g;
      }
      n++;
    }
    delay(2);
  }
  if (n < samples / 2) return false;

  bool still = true;
  for (int k = 0; k < 3; k++) {
    double mean = sum[k] / n;
    double var = sumSq[k] / n - mean * mean;
    if (var > 1.0) still = false;  // độ lệch chuẩn > 1 dps -> xe đang bị động
    _gyroBias[k] = (float)mean;
  }
  return still;
}

float MPU6050::gyroBiasLsb(int axis) const { return _gyroBias[axis] * GYRO_LSB_PER_DPS; }
