// =============================================================================
//  TEST RIÊNG MPU-6050
//  Nạp:  pio run -e test_mpu -t upload   rồi   pio device monitor
//
//  Chương trình sẽ:
//   1. Quét bus I2C (SDA=21, SCL=22) xem có thiết bị nào
//   2. Đọc WHO_AM_I, nhận dạng chip
//   3. Tự chẩn đoán: gia tốc tổng ~1g? gyro có nhiễu/rung? trục nào hướng lên?
//   4. In dữ liệu liên tục
//
//  Lệnh (gõ rồi Enter):
//   1 = in dữ liệu thô (accel g, gyro dps)
//   2 = in góc nghiêng (dùng để kiểm tra dấu, xem được trên Serial Plotter)
//   0 = tạm dừng in
//   c = hiệu chỉnh lại gyro (để cảm biến nằm yên)
//   d = chạy lại chẩn đoán
//   s = quét lại I2C
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "../imu.h"

MPU6050 imu;
uint8_t printMode = 1;
float angle = 0;
uint32_t readErrors = 0;

const char *AXIS_NAME[3] = {"X", "Y", "Z"};

// ------------------------------- QUÉT I2C -----------------------------------
int scanI2C() {
  Serial.println("\n--- Quet I2C (SDA=GPIO" + String(PIN_I2C_SDA) +
                 ", SCL=GPIO" + String(PIN_I2C_SCL) + ") ---");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Tim thay thiet bi tai 0x%02X", addr);
      if (addr == 0x68 || addr == 0x69) Serial.print("  <-- MPU-6050");
      Serial.println();
      found++;
    }
  }
  if (!found) {
    Serial.println("  KHONG tim thay thiet bi nao! Kiem tra:");
    Serial.println("   - VCC MPU -> 3V3, GND MPU -> GND");
    Serial.println("   - SDA -> GPIO21, SCL -> GPIO22 (co bi dao nguoc khong?)");
    Serial.println("   - Han chan header cua module GY-521 da chac chua");
    Serial.println("   - Thu doi day cam (day Dupont hay bi dut ngam)");
  }
  return found;
}

// ------------------------------ NHẬN DẠNG -----------------------------------
void identifyChip() {
  uint8_t who = imu.whoAmI();
  Serial.printf("WHO_AM_I = 0x%02X -> ", who);
  switch (who) {
    case 0x68: Serial.println("MPU-6050 (chuan)"); break;
    case 0x70: Serial.println("MPU-6500 (dung duoc, cung thanh ghi)"); break;
    case 0x71: Serial.println("MPU-9250 (dung duoc)"); break;
    case 0x73: Serial.println("MPU-9255 (dung duoc)"); break;
    case 0x72: case 0x98: case 0x19:
      Serial.println("chip clone (thuong van dung duoc)"); break;
    case 0x00: Serial.println("LOI doc - khong phan hoi"); break;
    default:   Serial.println("khong ro - co the la clone, thu xem du lieu co hop ly khong"); break;
  }
}

// --------------------------- GIẢI THÍCH LỖI ---------------------------------
void explainError() {
  if (imu.lastError() == MPU6050::I2C_FAIL) {
    Serial.println("  -> LOI I2C: ESP32 khong doc duoc tu MPU.");
    Serial.println("     Day SDA/SCL long, day Dupont dut ngam, chan header han chua chac.");
  } else if (imu.lastError() == MPU6050::ALL_ZERO) {
    Serial.println("  -> MPU tra loi I2C nhung DU LIEU TOAN 0 = chip dang NGU.");
    Serial.println("     Thuong do day VCC/GND long: khi xoay cam bien, MPU mat dien chop nhoang,");
    Serial.println("     tu reset va quay ve che do ngu. Han/cam chac lai VCC va GND.");
    Serial.println("     (Neu dien ap VCC < 3.0V cung bi. Do VCC-GND tren module phai ~3.3V)");
  }
}

// Đọc các thanh ghi quan trọng để chẩn đoán
void dumpRegisters() {
  struct { uint8_t reg; const char *name; const char *expect; } regs[] = {
    {0x75, "WHO_AM_I    ", "0x68 (clone: 0x70/0x72/0x98...)"},
    {0x6B, "PWR_MGMT_1  ", "0x01 (0x40 = DANG NGU!)"},
    {0x1A, "CONFIG      ", "0x03"},
    {0x1B, "GYRO_CONFIG ", "0x10"},
    {0x1C, "ACCEL_CONFIG", "0x08"},
  };
  Serial.println("\n--- Thanh ghi MPU ---");
  for (auto &r : regs) {
    uint8_t v;
    if (imu.readReg(r.reg, v))
      Serial.printf("  0x%02X %s = 0x%02X   (mong doi %s)\n", r.reg, r.name, v, r.expect);
    else
      Serial.printf("  0x%02X %s = LOI DOC I2C\n", r.reg, r.name);
  }
  int16_t raw[7];
  if (imu.readRaw(raw))
    Serial.printf("  Tho: AX=%d AY=%d AZ=%d TEMP=%d GX=%d GY=%d GZ=%d\n",
                  raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]);
  else
    Serial.println("  Tho: LOI DOC I2C");
}

// ------------------------------ CHẨN ĐOÁN -----------------------------------
// Đọc dữ liệu khi cảm biến nằm yên và đánh giá.
void diagnose() {
  Serial.println("\n--- Chan doan (DE CAM BIEN NAM YEN 2 giay) ---");
  const int N = 400;
  double aSum[3] = {0}, gSum[3] = {0}, gSq[3] = {0}, magSum = 0;
  int n = 0;
  float a[3], g[3];
  for (int i = 0; i < N; i++) {
    if (imu.read(a, g)) {
      for (int k = 0; k < 3; k++) {
        aSum[k] += a[k];
        gSum[k] += g[k];
        gSq[k] += g[k] * g[k];
      }
      magSum += sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      n++;
    }
    delay(5);
  }
  if (n < N / 2) {
    Serial.printf("  LOI: chi doc duoc %d/%d mau hop le\n", n, N);
    explainError();
    dumpRegisters();
    return;
  }

  float aMean[3], gMean[3], gStd[3];
  for (int k = 0; k < 3; k++) {
    aMean[k] = aSum[k] / n;
    gMean[k] = gSum[k] / n;
    gStd[k] = sqrt(fmax(0.0, gSq[k] / n - (double)gMean[k] * gMean[k]));
  }
  float mag = magSum / n;

  Serial.printf("  Doc thanh cong %d/%d mau\n", n, N);
  Serial.printf("  Accel TB (g)   : X=%+.3f  Y=%+.3f  Z=%+.3f\n", aMean[0], aMean[1], aMean[2]);
  Serial.printf("  |Accel| (g)    : %.3f  %s\n", mag,
                (mag > 0.9f && mag < 1.1f) ? "[OK]" : "[SAI - phai gan 1.000g]");
  Serial.printf("  Gyro offset    : X=%+.2f  Y=%+.2f  Z=%+.2f dps  (sau 'c' se ve ~0)\n",
                gMean[0], gMean[1], gMean[2]);
  bool quiet = gStd[0] < 0.5f && gStd[1] < 0.5f && gStd[2] < 0.5f;
  Serial.printf("  Gyro nhieu     : X=%.2f  Y=%.2f  Z=%.2f dps  %s\n", gStd[0], gStd[1], gStd[2],
                quiet ? "[OK]" : "[CAO - cam bien bi rung/lac hoac loi]");

  // Trục nào đang hướng lên?
  int up = 0;
  for (int k = 1; k < 3; k++)
    if (fabsf(aMean[k]) > fabsf(aMean[up])) up = k;
  Serial.printf("  Truc huong LEN : %s (%s)\n", AXIS_NAME[up],
                aMean[up] > 0 ? "chieu duong" : "chieu AM - cam bien dang bi up nguoc!");
  Serial.printf("  config.h hien tai: IMU_UP_AXIS=%s, IMU_FWD_AXIS=%s, IMU_GYRO_AXIS=%s\n",
                AXIS_NAME[IMU_UP_AXIS], AXIS_NAME[IMU_FWD_AXIS], AXIS_NAME[IMU_GYRO_AXIS]);
  if (up != IMU_UP_AXIS) {
    Serial.printf("  !! Khi xe dung thang, truc %s huong len nhung config dang la %s.\n",
                  AXIS_NAME[up], AXIS_NAME[IMU_UP_AXIS]);
    Serial.println("     (Neu luc test ban dang dat cam bien dung tu the tren xe thi hay sua config.h)");
  } else if (aMean[up] < 0) {
    Serial.println("  !! Truc huong len bi am -> lat cam bien lai (mat linh kien huong len)");
  } else {
    Serial.println("  Huong lap khop voi config.h [OK]");
  }
}

void calibrate() {
  Serial.println("\nHieu chinh gyro - DE CAM BIEN NAM YEN...");
  for (int t = 0; t < 5; t++) {
    if (imu.calibrateGyro(1000)) {
      Serial.println("Xong [OK]");
      float a[3], g[3];
      if (imu.read(a, g))
        angle = IMU_ACC_SIGN * atan2f(-a[IMU_FWD_AXIS], a[IMU_UP_AXIS]) * RAD_TO_DEG;
      return;
    }
    Serial.println("Bi rung, do lai...");
  }
  Serial.println("Canh bao: gyro van nhieu sau 5 lan do");
}

void printHelp() {
  Serial.println("\nLenh: 1=du lieu tho  2=goc nghieng  0=dung in  c=hieu chinh gyro  d=chan doan");
  Serial.println("      s=quet I2C  r=doc thanh ghi  w=danh thuc/khoi tao lai MPU");
  Serial.println("Kiem tra dau (che do 2): nghieng xe/cam bien ve phia TRUOC ->");
  Serial.println("  acc_goc va goc phai TANG (duong), gyro phai DUONG trong luc dang nghieng.");
  Serial.println("  Sai acc_goc -> doi IMU_ACC_SIGN, sai gyro -> doi IMU_GYRO_SIGN trong config.h\n");
}

// --------------------------------- SETUP ------------------------------------
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n\n=========== TEST MPU-6050 ===========");

  // Tắt driver động cơ (nếu đã lắp) để an toàn
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, HIGH);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  while (scanI2C() == 0) delay(2000);

  while (!imu.begin(Wire, MPU_ADDR)) {
    Serial.printf("Khong khoi tao duoc MPU tai 0x%02X (dia chi trong config.h: MPU_ADDR)\n", MPU_ADDR);
    delay(2000);
  }
  identifyChip();
  diagnose();
  calibrate();
  printHelp();
}

// ---------------------------------- LOOP ------------------------------------
void loop() {
  // Lệnh
  if (Serial.available()) {
    char c = Serial.read();
    while (Serial.available()) Serial.read();
    switch (c) {
      case '0': printMode = 0; Serial.println("Tam dung in"); break;
      case '1': printMode = 1; break;
      case '2': printMode = 2; break;
      case 'c': calibrate(); break;
      case 'd': diagnose(); break;
      case 's': scanI2C(); break;
      case 'r': dumpRegisters(); break;
      case 'w':
        Serial.println(imu.begin(Wire, MPU_ADDR) ? "Khoi tao lai MPU: OK" : "Khoi tao lai MPU: THAT BAI");
        break;
      case '\r': case '\n': break;
      default: printHelp(); break;
    }
  }

  // Đọc cảm biến 200 Hz, chạy bộ lọc bù giống code chính
  static uint32_t lastRead = micros();
  static float a[3], g[3], accAngle = 0, rate = 0;
  static uint16_t failStreak = 0;
  static uint32_t zeroEvents = 0;
  uint32_t now = micros();
  constexpr uint16_t FAIL_REPORT = LOOP_HZ / 20;  // lỗi liên tục 50 ms
  if (now - lastRead >= 1000000UL / LOOP_HZ) {      // đọc cùng tần số với code chính
    float dt = (now - lastRead) * 1e-6f;
    lastRead = now;
    if (imu.read(a, g)) {
      if (failStreak >= FAIL_REPORT) Serial.println(">> MPU doc lai binh thuong");
      failStreak = 0;
      accAngle = IMU_ACC_SIGN * atan2f(-a[IMU_FWD_AXIS], a[IMU_UP_AXIS]) * RAD_TO_DEG;
      rate = IMU_GYRO_SIGN * g[IMU_GYRO_AXIS];
      angle = COMP_ALPHA * (angle + rate * dt) + (1 - COMP_ALPHA) * accAngle;
    } else {
      readErrors++;
      failStreak++;
      if (failStreak == FAIL_REPORT) {  // lỗi liên tục 50 ms -> báo nguyên nhân và thử đánh thức
        Serial.printf("\n!! MPU loi lien tuc (%s)\n",
                      imu.lastError() == MPU6050::ALL_ZERO ? "du lieu toan 0" : "loi I2C");
        explainError();
        if (imu.lastError() == MPU6050::ALL_ZERO) zeroEvents++;
        Serial.printf("   Thu danh thuc lai MPU... %s  (so lan bi reset: %lu)\n",
                      imu.begin(Wire, MPU_ADDR) ? "OK" : "THAT BAI", (unsigned long)zeroEvents);
      } else if (failStreak > FAIL_REPORT && failStreak % LOOP_HZ == 0) {
        imu.begin(Wire, MPU_ADDR);  // tiếp tục thử mỗi ~1 s
      }
    }
  }

  // In dữ liệu
  static uint32_t lastPrint = 0;
  if (failStreak >= FAIL_REPORT) {
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.printf("   [MPU LOI - %s] khong co du lieu hop le, loi:%lu\n",
                    imu.lastError() == MPU6050::ALL_ZERO ? "chip ngu" : "I2C",
                    (unsigned long)readErrors);
    }
  } else if (printMode == 1 && millis() - lastPrint >= 100) {
    lastPrint = millis();
    float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    Serial.printf("ACC(g) X:%+.2f Y:%+.2f Z:%+.2f |%.2f|   GYRO(dps) X:%+7.1f Y:%+7.1f Z:%+7.1f   loi:%lu\n",
                  a[0], a[1], a[2], mag, g[0], g[1], g[2], (unsigned long)readErrors);
  } else if (printMode == 2 && millis() - lastPrint >= 50) {
    lastPrint = millis();
    Serial.printf("acc_goc:%.2f goc:%.2f gyro:%.1f\n", accAngle, angle, rate);
  }
}
