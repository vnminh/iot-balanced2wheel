// =============================================================================
//  TEST RIÊNG DRIVER A4988 + ĐỘNG CƠ BƯỚC 42
//  Nạp:  pio run -e test_driver -t upload   rồi   pio device monitor
//
//  !! NHẤC XE LÊN (bánh không chạm đất) hoặc tháo bánh trước khi test !!
//  !! Cần cấp nguồn 12V cho VMOT, nếu không động cơ sẽ không quay      !!
//
//  Lệnh (gõ rồi Enter):
//   on / off      : bật / tắt dòng qua động cơ (chân EN)
//                   -> 'on' thì trục phải CỨNG, xoay tay rất nặng
//                   -> 'off' thì xoay tay nhẹ
//   m l|r|b       : chọn động cơ trái / phải / cả hai (mặc định cả hai)
//   rev <n>       : quay n vòng (âm = lùi), vd: rev 1, rev -2
//   rpm <n>       : quay liên tục n vòng/phút (âm = lùi), vd: rpm 60
//   stop          : dừng (vẫn giữ lực)
//   auto          : chạy tự động: trái tới 1 vòng -> phải tới 1 vòng -> cả 2 lùi 1 vòng
//   sweep         : tăng tốc dần đến tối đa để tìm tốc độ động cơ bị trượt bước
//   pins          : in cấu hình chân
// =============================================================================
#include <Arduino.h>
#include "config.h"
#include "../steppers.h"

constexpr float RAMP_RPM_PER_S = 300.0f;   // gia tốc khi đổi tốc độ
constexpr float REV_RPM = 60.0f;           // tốc độ khi chạy lệnh rev
constexpr float MAX_RPM = MAX_STEP_RATE * 60.0f / STEPS_PER_REV;

enum Sel { SEL_LEFT = 1, SEL_RIGHT = 2, SEL_BOTH = 3 };
uint8_t sel = SEL_BOTH;

float targetRpm[2] = {0, 0};
float currentRpm[2] = {0, 0};

// Chạy theo số vòng
bool revActive[2] = {false, false};
int32_t revStart[2], revSteps[2];

// Chuỗi tự động / sweep
enum Mode { MANUAL, AUTO_SEQ, SWEEP };
Mode mode = MANUAL;
int autoStep = 0;
uint32_t modeTimer = 0;

const char *selName() {
  return sel == SEL_LEFT ? "TRAI" : sel == SEL_RIGHT ? "PHAI" : "CA HAI";
}

void ensureEnabled() {
  if (!Steppers::isEnabled()) {
    Steppers::enable(true);
    Serial.println("(Tu dong bat EN)");
  }
}

void setTarget(uint8_t which, float rpm) {
  rpm = constrain(rpm, -MAX_RPM, MAX_RPM);
  if (which & SEL_LEFT) { targetRpm[0] = rpm; revActive[0] = false; }
  if (which & SEL_RIGHT) { targetRpm[1] = rpm; revActive[1] = false; }
}

void startRev(uint8_t which, float revs) {
  ensureEnabled();
  for (int i = 0; i < 2; i++) {
    if (!(which & (1 << i))) continue;
    revStart[i] = Steppers::position(i);
    revSteps[i] = (int32_t)(fabsf(revs) * STEPS_PER_REV);
    revActive[i] = true;
    targetRpm[i] = revs >= 0 ? REV_RPM : -REV_RPM;
    currentRpm[i] = targetRpm[i];  // 60 rpm khởi động trực tiếp được, không cần ramp
  }
}

void stopAll() {
  mode = MANUAL;
  for (int i = 0; i < 2; i++) {
    targetRpm[i] = currentRpm[i] = 0;
    revActive[i] = false;
    Steppers::setSpeed(i, 0);
  }
}

bool anyMoving() {
  return currentRpm[0] != 0 || currentRpm[1] != 0 || targetRpm[0] != 0 || targetRpm[1] != 0;
}

void printPins() {
  Serial.println("\n--- Cau hinh chan ---");
  Serial.printf("Trai : STEP=GPIO%d  DIR=GPIO%d  invert=%d\n", PIN_L_STEP, PIN_L_DIR, LEFT_MOTOR_INVERT);
  Serial.printf("Phai : STEP=GPIO%d  DIR=GPIO%d  invert=%d\n", PIN_R_STEP, PIN_R_DIR, RIGHT_MOTOR_INVERT);
  Serial.printf("EN chung = GPIO%d (LOW = bat)\n", PIN_MOTOR_EN);
  Serial.printf("Vi buoc 1/%d -> %d buoc/vong, toc do toi da %.0f rpm (%.2f m/s voi banh %.0f mm)\n",
                MICROSTEP, STEPS_PER_REV, MAX_RPM,
                MAX_RPM / 60.0f * 3.14159f * WHEEL_DIAMETER_M, WHEEL_DIAMETER_M * 1000);
}

void printHelp() {
  Serial.println(F(
    "\n===== TEST DRIVER - lenh =====\n"
    "on / off    : bat / tat dong co (on -> truc phai cung)\n"
    "m l|r|b     : chon dong co trai / phai / ca hai\n"
    "rev <n>     : quay n vong (am = lui)\n"
    "rpm <n>     : quay lien tuc n vong/phut (am = lui)\n"
    "stop        : dung\n"
    "auto        : trai toi 1 vong -> phai toi 1 vong -> ca 2 lui 1 vong\n"
    "sweep       : tang toc dan de tim toc do bi truot buoc\n"
    "pins        : xem cau hinh chan\n"
    "'TOI' = chieu lam xe chay ve phia truoc. Banh nao quay nguoc -> doi *_MOTOR_INVERT\n"));
}

void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *arg = strtok(nullptr, " \t");

  if (!strcmp(cmd, "on")) {
    Steppers::enable(true);
    Serial.println("EN = BAT. Thu xoay truc bang tay: phai rat nang. Neu van nhe -> kiem tra 12V VMOT, day EN, Vref");
  } else if (!strcmp(cmd, "off")) {
    stopAll();
    Steppers::enable(false);
    Serial.println("EN = TAT. Truc xoay tay nhe nhang.");
  } else if (!strcmp(cmd, "m") && arg) {
    sel = arg[0] == 'l' ? SEL_LEFT : arg[0] == 'r' ? SEL_RIGHT : SEL_BOTH;
    Serial.printf("Chon dong co: %s\n", selName());
  } else if (!strcmp(cmd, "rev") && arg) {
    mode = MANUAL;
    float n = atof(arg);
    startRev(sel, n);
    Serial.printf("Dong co %s quay %.2f vong %s\n", selName(), fabsf(n), n >= 0 ? "TOI" : "LUI");
  } else if (!strcmp(cmd, "rpm") && arg) {
    mode = MANUAL;
    ensureEnabled();
    float r = atof(arg);
    setTarget(sel, r);
    Serial.printf("Dong co %s -> %.0f rpm\n", selName(), constrain(r, -MAX_RPM, MAX_RPM));
  } else if (!strcmp(cmd, "stop")) {
    stopAll();
    Serial.println("Dung");
  } else if (!strcmp(cmd, "auto")) {
    stopAll();
    ensureEnabled();
    mode = AUTO_SEQ;
    autoStep = 0;
    modeTimer = 0;
  } else if (!strcmp(cmd, "sweep")) {
    stopAll();
    ensureEnabled();
    mode = SWEEP;
    modeTimer = millis();
    Serial.println("SWEEP: tang toc tu 0 den toi da. Khi dong co keu rit/dung lai thi go 'stop'");
    Serial.println("       va ghi nho rpm do. Dat MAX_SPEED_MPS trong config.h thap hon ~30%.");
  } else if (!strcmp(cmd, "pins")) {
    printPins();
  } else {
    printHelp();
  }
}

void pollSerial() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (len) { buf[len] = 0; handleCommand(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// Chuỗi tự động kiểm tra chiều quay
void runAutoSeq() {
  if (anyMoving()) return;  // đợi bước trước chạy xong
  if (modeTimer && millis() - modeTimer < 700) return;  // nghỉ giữa các bước
  switch (autoStep++) {
    case 0: Serial.println("[1/3] Banh TRAI quay TOI 1 vong"); startRev(SEL_LEFT, 1); break;
    case 1: Serial.println("[2/3] Banh PHAI quay TOI 1 vong"); startRev(SEL_RIGHT, 1); break;
    case 2: Serial.println("[3/3] CA HAI banh quay LUI 1 vong"); startRev(SEL_BOTH, -1); break;
    default:
      Serial.println("Xong. Ca 2 banh co cung lam xe chay TOI o buoc 1, 2 khong?");
      mode = MANUAL;
      return;
  }
  modeTimer = 0;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n\n=========== TEST DRIVER A4988 ===========");
  Serial.println("!! NHAC XE LEN / THAO BANH truoc khi test. Can cap 12V cho VMOT !!");
  Steppers::begin();
  printPins();
  printHelp();
}

void loop() {
  pollSerial();

  static uint32_t lastUpdate = 0;
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastUpdate < 5) return;
  float dt = (now - lastUpdate) * 1e-3f;
  lastUpdate = now;

  if (mode == SWEEP) {
    float r = (now - modeTimer) * 1e-3f * 20.0f;  // tăng 20 rpm mỗi giây
    if (r > MAX_RPM) {
      Serial.printf("Da den gioi han phan mem %.0f rpm ma khong truot buoc. Dung.\n", MAX_RPM);
      stopAll();
    } else {
      targetRpm[0] = targetRpm[1] = currentRpm[0] = currentRpm[1] = r;
      if (now - lastPrint >= 500) {
        lastPrint = now;
        Serial.printf("  %.0f rpm (%.2f m/s)\n", r, r / 60.0f * 3.14159f * WHEEL_DIAMETER_M);
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    // Kết thúc lệnh rev
    if (revActive[i] && abs(Steppers::position(i) - revStart[i]) >= revSteps[i]) {
      revActive[i] = false;
      targetRpm[i] = currentRpm[i] = 0;
      if (mode == AUTO_SEQ && !anyMoving()) modeTimer = millis();
    }
    // Ramp tốc độ
    float diff = targetRpm[i] - currentRpm[i];
    float stepMax = RAMP_RPM_PER_S * dt;
    currentRpm[i] += constrain(diff, -stepMax, stepMax);
    Steppers::setSpeed(i, currentRpm[i] * STEPS_PER_REV / 60.0f);
  }

  if (mode == AUTO_SEQ) runAutoSeq();
}
