#include "steppers.h"
#include "config.h"
#include <soc/gpio_reg.h>

static_assert(PIN_L_STEP < 32 && PIN_L_DIR < 32 && PIN_R_STEP < 32 && PIN_R_DIR < 32,
              "Chan STEP/DIR phai < GPIO32 (dung thanh ghi GPIO_OUT_W1TS)");
static_assert(MAX_STEP_RATE * 2 < STEP_ISR_HZ, "MAX_STEP_RATE phai < STEP_ISR_HZ/2");

namespace {

struct Motor {
  uint32_t stepMask;
  uint32_t dirMask;
  bool invert;
  volatile uint32_t inc;     // bước cộng dồn mỗi lần ngắt (pha 32 bit)
  volatile bool dirReq;      // chiều mong muốn (true = tới)
  uint32_t acc;              // bộ cộng dồn pha
  bool dirCur;               // chiều hiện đang xuất ra chân DIR
  bool stepHigh;             // chân STEP đang ở mức cao
  volatile int32_t pos;
};

Motor motors[2];
hw_timer_t *timer = nullptr;
bool enabled = false;

inline void IRAM_ATTR pinHigh(uint32_t mask) { *(volatile uint32_t *)GPIO_OUT_W1TS_REG = mask; }
inline void IRAM_ATTR pinLow(uint32_t mask)  { *(volatile uint32_t *)GPIO_OUT_W1TC_REG = mask; }

// Mỗi lần bộ cộng dồn tràn -> đảo chân STEP. A4988 bước ở sườn lên,
// nên 2 lần tràn = 1 bước. Độ rộng xung tối thiểu = 1 chu kỳ ngắt (40 us).
void IRAM_ATTR onStepTimer() {
  for (int i = 0; i < 2; i++) {
    Motor &m = motors[i];
    bool want = m.dirReq;
    if (want != m.dirCur) {
      // Đổi chiều: hạ STEP trước, chu kỳ sau mới đổi DIR (đảm bảo thời gian setup)
      if (m.stepHigh) {
        pinLow(m.stepMask);
        m.stepHigh = false;
      } else {
        if (want ^ m.invert) pinHigh(m.dirMask); else pinLow(m.dirMask);
        m.dirCur = want;
      }
      continue;
    }
    uint32_t prev = m.acc;
    m.acc += m.inc;
    if (m.acc < prev) {  // tràn
      if (m.stepHigh) {
        pinLow(m.stepMask);
        m.stepHigh = false;
      } else {
        pinHigh(m.stepMask);
        m.stepHigh = true;
        m.pos += m.dirCur ? 1 : -1;
      }
    }
  }
}

void initMotor(Motor &m, int stepPin, int dirPin, bool invert) {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(stepPin, LOW);
  m.stepMask = 1UL << stepPin;
  m.dirMask = 1UL << dirPin;
  m.invert = invert;
  m.inc = 0;
  m.acc = 0;
  m.dirReq = true;
  m.dirCur = true;
  m.stepHigh = false;
  m.pos = 0;
  digitalWrite(dirPin, (true ^ invert) ? HIGH : LOW);
}

}  // namespace

namespace Steppers {

void begin() {
  pinMode(PIN_MOTOR_EN, OUTPUT);
  digitalWrite(PIN_MOTOR_EN, HIGH);  // tắt động cơ khi khởi động
  enabled = false;

  initMotor(motors[LEFT], PIN_L_STEP, PIN_L_DIR, LEFT_MOTOR_INVERT);
  initMotor(motors[RIGHT], PIN_R_STEP, PIN_R_DIR, RIGHT_MOTOR_INVERT);

  timer = timerBegin(1000000);                       // timer 1 MHz
  timerAttachInterrupt(timer, &onStepTimer);
  timerAlarm(timer, 1000000 / STEP_ISR_HZ, true, 0); // ngắt mỗi 40 us
}

void enable(bool on) {
  if (!on) {
    setSpeed(LEFT, 0);
    setSpeed(RIGHT, 0);
  }
  digitalWrite(PIN_MOTOR_EN, on ? LOW : HIGH);
  enabled = on;
}

bool isEnabled() { return enabled; }

void setSpeed(int motor, float sps) {
  Motor &m = motors[motor];
  float a = fabsf(sps);
  if (a > MAX_STEP_RATE) a = MAX_STEP_RATE;
  // 2 lần tràn / bước:  inc = 2 * f * 2^32 / F_isr
  constexpr float K = 2.0f * 4294967296.0f / STEP_ISR_HZ;
  if (sps > 0) m.dirReq = true;
  else if (sps < 0) m.dirReq = false;
  m.inc = (uint32_t)(a * K);
}

int32_t position(int motor) { return motors[motor].pos; }

}  // namespace Steppers
