#include "sonar.h"

#include "config.h"
#include "link.h"

namespace {

struct SonarDef {
  uint8_t trig, echo;
  int16_t x_mm, y_mm, yaw_deg;
  uint8_t half_deg;
  uint16_t stop_mm;
};

#ifdef SONAR_SENSORS
const SonarDef DEFS[] = {SONAR_SENSORS};
constexpr int N = sizeof(DEFS) / sizeof(DEFS[0]);
#else
const SonarDef *DEFS = nullptr;
constexpr int N = 0;
#endif
static_assert(N <= 8, "Toi da 8 cam bien sieu am");

// ECHO HIGH tối đa cho tầm SONAR_MAX_RANGE_MM (âm thanh 343 m/s, đi + về) + 2 ms dự phòng
constexpr uint32_t TIMEOUT_US = uint32_t(SONAR_MAX_RANGE_MM) * 2000UL / 343UL + 2000UL;
constexpr uint32_t RISE_TIMEOUT_US = 3000;   // HC-SR04 nâng ECHO ~0,5 ms sau TRIG; quá 3 ms = không có cảm biến

volatile uint32_t riseUs[N > 0 ? N : 1], fallUs[N > 0 ? N : 1];
volatile bool rose[N > 0 ? N : 1], fell[N > 0 ? N : 1];

uint16_t last[N > 0 ? N : 1];
uint16_t hist[N > 0 ? N : 1][3];
uint32_t lastMs[N > 0 ? N : 1];
uint8_t faults[N > 0 ? N : 1];

enum { IDLE, WAIT } state = IDLE;
int cur = -1;
uint32_t trigUs = 0, endUs = 0, lastGeomMs = 0;

void IRAM_ATTR echoIsr(void *arg) {
  const int k = (int)(intptr_t)arg;
  const uint32_t now = micros();
  if (digitalRead(DEFS[k].echo)) {
    riseUs[k] = now;
    rose[k] = true;
  } else if (rose[k]) {
    fallUs[k] = now;
    fell[k] = true;
  }
}

uint16_t median3(const uint16_t *h) {
  uint16_t a = h[0], b = h[1], c = h[2];
  if (a > b) std::swap(a, b);
  if (b > c) std::swap(b, c);
  if (a > b) std::swap(a, b);
  return b;
}

void finish(int k, uint16_t mm, uint32_t tUs) {
  last[k] = mm;
  lastMs[k] = millis();
  hist[k][0] = hist[k][1];
  hist[k][1] = hist[k][2];
  hist[k][2] = mm;
  Link::SonarReading r{tUs, (uint8_t)k, mm};
  Link::sendFrame(Link::T_SONAR, &r, sizeof r);
}

}  // namespace

namespace Sonar {

int count() { return N; }
uint16_t lastMm(int k) { return (k >= 0 && k < N) ? last[k] : FAULT; }

void begin() {
  for (int k = 0; k < N; k++) {
    pinMode(DEFS[k].trig, OUTPUT);
    digitalWrite(DEFS[k].trig, LOW);
    pinMode(DEFS[k].echo, INPUT);
    attachInterruptArg(digitalPinToInterrupt(DEFS[k].echo), echoIsr, (void *)(intptr_t)k, CHANGE);
    last[k] = NO_ECHO;
    hist[k][0] = hist[k][1] = hist[k][2] = NO_ECHO;
    lastMs[k] = 0;
    faults[k] = 0;
  }
  endUs = micros();
  sendGeometry();
}

void sendGeometry() {
  uint8_t buf[4 + 9 * 8];
  buf[0] = 1;   // version
  buf[1] = (uint8_t)N;
  const uint16_t mr = SONAR_MAX_RANGE_MM;
  memcpy(buf + 2, &mr, 2);
  for (int k = 0; k < N; k++) {
    Link::SonarGeomItem it{DEFS[k].x_mm, DEFS[k].y_mm, (int16_t)(DEFS[k].yaw_deg * 100), DEFS[k].half_deg,
                           DEFS[k].stop_mm};
    memcpy(buf + 4 + 9 * k, &it, 9);
  }
  Link::sendFrame(Link::T_SONAR_GEOM, buf, (uint8_t)(4 + 9 * N));
  lastGeomMs = millis();
}

void poll() {
  if (millis() - lastGeomMs >= SONAR_GEOM_PERIOD_MS) sendGeometry();
  if (N == 0) return;
  const uint32_t now = micros();
  if (state == IDLE) {
    if (now - endUs < SONAR_GAP_US) return;
    // cảm biến kế tiếp; bỏ qua cái còn đang giữ ECHO HIGH từ lần trước (HC-SR04 không vang giữ ~38 ms)
    for (int tries = 0; tries < N; tries++) {
      cur = (cur + 1) % N;
      if (!digitalRead(DEFS[cur].echo)) break;
    }
    if (digitalRead(DEFS[cur].echo)) return;
    rose[cur] = fell[cur] = false;
    digitalWrite(DEFS[cur].trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(DEFS[cur].trig, LOW);
    trigUs = micros();
    state = WAIT;
    return;
  }
  // WAIT
  const int k = cur;
  if (fell[k]) {
    const uint32_t w = fallUs[k] - riseUs[k];
    uint32_t mm = w * 343UL / 2000UL;   // µs -> mm (đi + về)
    faults[k] = 0;
    finish(k, (mm > SONAR_MAX_RANGE_MM) ? NO_ECHO : (uint16_t)max<uint32_t>(mm, 20), fallUs[k]);
    state = IDLE;
    endUs = micros();
  } else if (!rose[k] && now - trigUs > RISE_TIMEOUT_US) {
    if (faults[k] < 255) faults[k]++;
    finish(k, faults[k] >= 3 ? FAULT : NO_ECHO, now);   // 3 lần liền không có ECHO mới coi là hỏng
    state = IDLE;
    endUs = now;
  } else if (now - trigUs > TIMEOUT_US) {
    faults[k] = 0;
    finish(k, NO_ECHO, now);   // ECHO vẫn HIGH: vật xa hơn tầm hoặc không có vang
    state = IDLE;
    endUs = now;
  }
}

int blockingSensor() {
  const uint32_t ms = millis();
  for (int k = 0; k < N; k++) {
    if (!DEFS[k].stop_mm || ms - lastMs[k] > SONAR_FRESH_MS) continue;
    const uint16_t m = median3(hist[k]);
    if (m != NO_ECHO && m != FAULT && m < DEFS[k].stop_mm) return k;
  }
  return -1;
}

}  // namespace Sonar
