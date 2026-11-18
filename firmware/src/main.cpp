// =============================================================================
//  XE CÂN BẰNG 2 BÁNH CHO VI-SLAM - ESP32 + MPU-6050 + 2x A4988 + 2x Step 42
//
//   lệnh Pi (VEL) ─► [làm mượt] ─► v_đặt ─► [PI tốc độ] ─► góc đặt ─► [PID góc] ─► gia tốc ─► ∫ ─► tốc độ bánh
//                       └─► t (quay) ─────────────────────────────────────────── cộng vi sai ─┘
//
//  Mỗi 2 chu kỳ (200 Hz) gửi sang Pi: mẫu IMU thô + tổng số bước 2 bánh.
//  Động cơ bước không trượt => số bước chính là quãng đường bánh đã lăn
//  (so với thân xe), Pi dùng nó làm odometry bánh xe cho OpenVINS.
//
//  Vận hành:
//   - Bật nguồn: ĐỂ XE NẰM YÊN ~2 s để hiệu chỉnh gyro (LED sáng).
//   - Dựng xe thẳng (±5°) giữ 0.5 s -> tự cân bằng (LED sáng liên tục).
//   - Ngã > 45° -> tắt động cơ, chờ dựng lại (LED chớp).
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "link.h"
#include "steppers.h"

enum State : uint8_t { CALIBRATING = 0, WAITING = 1, BALANCING = 2 };
State state = CALIBRATING;

MPU6050 imu;

float angle = 0;          // góc nghiêng đã lọc (độ), + = chúi tới
bool angleValid = false;
float gyroRate = 0;       // độ/s
float targetAngle = 0;
float wheelSpeed = 0;     // m/s đang ra lệnh (thành phần tiến)
float speedFilt = 0;
float angleInt = 0;
float speedInt = 0;
float wheelAccel = 0;
uint32_t uprightSince = 0;
uint16_t i2cErrors = 0;
uint16_t telemSeq = 0;
uint8_t telemDiv = 0;

constexpr float STEP_SPEED_LIMIT = MAX_STEP_RATE / STEPS_PER_METER;
constexpr float SPEED_LIMIT = MAX_SPEED_MPS < STEP_SPEED_LIMIT ? MAX_SPEED_MPS : STEP_SPEED_LIMIT;
constexpr float G = 9.81f;

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void resetController() {
  targetAngle = 0;
  wheelSpeed = 0;
  speedFilt = 0;
  angleInt = 0;
  speedInt = 0;
  wheelAccel = 0;
}

void stopMotors(const char *why) {
  Steppers::enable(false);
  resetController();
  // Xoá lệnh ở ESP32 VÀ báo Pi: Pi giữ lệnh hiện hành và gửi lại 10 lần/giây,
  // không báo thì xe vừa dựng lên sẽ chạy tiếp theo lệnh cũ.
  Link::clear();
  Link::sendText("ST:%s", why);
}

void sendTelemetry(uint32_t tUs) {
  if (++telemDiv < TELEM_DIV) return;
  telemDiv = 0;
  const int16_t *r = imu.lastRaw();
  Link::ImuPacket p;
  p.t_us = tUs;
  for (int i = 0; i < 3; i++) {
    p.acc[i] = r[i];
    p.gyro[i] = (int16_t)clampf(lroundf(r[i + 4] - imu.gyroBiasLsb(i)), -32768, 32767);
  }
  p.steps[0] = Steppers::position(Steppers::LEFT);
  p.steps[1] = Steppers::position(Steppers::RIGHT);
  p.state = state;
  p.flags = (Link::alive() ? 1 : 0) | (Steppers::isEnabled() ? 2 : 0);
  p.seq = telemSeq++;
  Link::sendImu(p);
}

// ----------------------------- VÒNG ĐIỀU KHIỂN ------------------------------
void controlStep(float dt) {
  float acc[3], gyr[3];
  if (!imu.read(acc, gyr)) {
    ++i2cErrors;
    if (i2cErrors == LOOP_HZ / 50 && state == BALANCING) {  // mất 20 ms dữ liệu
      stopMotors("IMU_ERROR");
      state = WAITING;
    }
    if (i2cErrors % (LOOP_HZ / 2) == 0) {
      bool ok = imu.begin(Wire, MPU_ADDR);
      Link::log("Khoi tao lai MPU: %s", ok ? "OK" : "that bai");
      if (ok) angleValid = false, uprightSince = 0;
    }
    return;
  }
  i2cErrors = 0;
  const uint32_t tSample = micros();

  // --- Góc nghiêng: bộ lọc bù ---
  float accAngle = IMU_ACC_SIGN * atan2f(-acc[IMU_FWD_AXIS], acc[IMU_UP_AXIS]) * RAD_TO_DEG;
#if ACC_COMPENSATION
  accAngle += atanf(wheelAccel / G) * RAD_TO_DEG;
#endif
  gyroRate = IMU_GYRO_SIGN * gyr[IMU_GYRO_AXIS];
  if (!angleValid) {
    angle = accAngle;
    angleValid = true;
  }
  angle = COMP_ALPHA * (angle + gyroRate * dt) + (1.0f - COMP_ALPHA) * accAngle;
  float tilt = angle - ANGLE_OFFSET_DEG;

  Link::update(dt);

  if (state == WAITING) {
    if (fabsf(tilt) < START_ANGLE_DEG) {
      if (uprightSince == 0) uprightSince = millis();
      if (millis() - uprightSince > START_HOLD_MS) {
        resetController();
        Link::clear();
        Steppers::enable(true);
        state = BALANCING;
        Link::sendText("ST:BALANCING");
      }
    } else {
      uprightSince = 0;
    }
    sendTelemetry(tSample);
    return;
  }

  if (fabsf(tilt) > FALL_ANGLE_DEG) {
    stopMotors("FALLEN");
    state = WAITING;
    uprightSince = 0;
    sendTelemetry(tSample);
    return;
  }

  // --- Lệnh từ Pi ---
  // speedInt = quãng đường đã trôi so với điểm giữ. Đổi lệnh thì xoá, nếu không
  // dừng lại xe sẽ chạy ngược về điểm xuất phát.
  if (Link::consumeTargetChanged()) speedInt = 0;
  const float cmdSpeed = Link::speed();
  const float cmdTurn = Link::turn();

  // --- Vòng ngoài: PI tốc độ -> góc đặt ---
  speedFilt += (wheelSpeed - speedFilt) * (dt / (SPEED_FILTER_TAU + dt));
  float speedErr = cmdSpeed - speedFilt;
  if (SPEED_KI > 0) {
    speedInt += speedErr * dt;
    const float lim = MAX_TILT_TARGET / SPEED_KI;
    speedInt = clampf(speedInt, -lim, lim);
  }
  targetAngle = clampf(SPEED_KP * speedErr + SPEED_KI * speedInt, -MAX_TILT_TARGET, MAX_TILT_TARGET);

  // --- Vòng trong: PID góc -> gia tốc bánh ---
  float err = (tilt - targetAngle) * DEG_TO_RAD;
  if (ANGLE_KI > 0) {
    angleInt += err * dt;
    const float lim = MAX_ACCEL_MPS2 / ANGLE_KI;
    angleInt = clampf(angleInt, -lim, lim);
  }
  float accel = ANGLE_KP * err + ANGLE_KI * angleInt + ANGLE_KD * gyroRate * DEG_TO_RAD;
  accel = clampf(accel, -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2);

  float prevSpeed = wheelSpeed;
  wheelSpeed = clampf(wheelSpeed + accel * dt, -SPEED_LIMIT, SPEED_LIMIT);
  wheelAccel = (wheelSpeed - prevSpeed) / dt;

  // Quay = vi sai cộng SAU vòng cân bằng (quay quanh trục đứng không ảnh
  // hưởng cân bằng chúi). Thu nhỏ cùng tỉ lệ nếu vượt để giữ đúng hướng.
  float sps = wheelSpeed * STEPS_PER_METER;
  float dps = cmdTurn * STEPS_PER_METER;
  const float peak = fabsf(sps) + fabsf(dps);
  if (peak > MAX_STEP_RATE) {
    const float k = MAX_STEP_RATE / peak;
    sps *= k;
    dps *= k;
  }
  Steppers::setSpeed(Steppers::LEFT, sps + dps);
  Steppers::setSpeed(Steppers::RIGHT, sps - dps);

  sendTelemetry(tSample);
}

void updateLed() {
  bool on = (state == WAITING) ? (millis() % 1000) < 100 : true;
  digitalWrite(PIN_LED, on);
}

#if DEBUG_LOG
void printLog() {
  static uint32_t last = 0;
  if (millis() - last < 50) return;
  last = millis();
  Serial.printf("goc:%.2f dat:%.2f v_cm:%.1f lenh_cm:%.1f\n", angle - ANGLE_OFFSET_DEG, targetAngle,
                wheelSpeed * 100, Link::speed() * 100);
}
#endif

void setup() {
#if LINK_UART != 0
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
#endif
  pinMode(PIN_LED, OUTPUT);
  Steppers::begin();  // EN = HIGH -> động cơ tắt
  Link::begin();
  delay(300);
  Link::log("%s: xe can bang + VI-SLAM | %.2f m/s, %.1f m/s2 | UART%d RX=%d TX=%d @%d | IMU %d Hz", FW_VERSION,
            SPEED_LIMIT, MAX_ACCEL_MPS2, LINK_UART, PIN_LINK_RX, PIN_LINK_TX, LINK_BAUD, LOOP_HZ / TELEM_DIV);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  while (!imu.begin(Wire, MPU_ADDR)) {
    // Quét bus I2C để biết là mất dây hay sai địa chỉ
    char found[64] = "";
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0 && strlen(found) < 58) sprintf(found + strlen(found), " %02X", a);
    }
    Link::sendText("ST:IMU_ERROR");
    Link::log("Khong thay MPU-6050 (0x%02X). I2C SDA=%d SCL=%d thay:%s", MPU_ADDR, PIN_I2C_SDA, PIN_I2C_SCL,
              found[0] ? found : " (khong co thiet bi nao)");
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    Link::poll();
    delay(1000);
  }

  Link::log("Hieu chinh gyro - DE XE NAM YEN...");
  digitalWrite(PIN_LED, HIGH);
  for (int tries = 0; tries < 5 && !imu.calibrateGyro(1000); tries++) Link::log("Xe bi rung, do lai...");

  state = WAITING;
  Link::sendText("ST:READY");
}

void loop() {
  Link::poll();

  constexpr uint32_t PERIOD_US = 1000000UL / LOOP_HZ;
  static uint32_t lastUs = micros();
  static uint32_t nextUs = lastUs + PERIOD_US;
  uint32_t now = micros();
  if ((int32_t)(now - nextUs) >= 0) {
    float dt = (now - lastUs) * 1e-6f;
    if (dt > 0.05f) dt = 0.05f;
    lastUs = now;
    nextUs += PERIOD_US;
    if ((int32_t)(now - nextUs) >= 0) nextUs = now + PERIOD_US;
    controlStep(dt);
#if DEBUG_LOG
    printLog();
#endif
  }

  static uint32_t lastInfo = 0;  // nhắc lại trạng thái mỗi 2 s (Pi mới mở cổng cũng biết)
  if (millis() - lastInfo > 2000) {
    lastInfo = millis();
    static const char *NAMES[] = {"CALIBRATING", "WAITING", "BALANCING"};
    Link::sendText("HB:%s", NAMES[state]);
  }
  updateLed();
}
