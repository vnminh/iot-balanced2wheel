#include "link.h"
#include "config.h"
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#if LINK_UART == 0
#define LINK Serial
#else
#define LINK Serial2
#endif

namespace {

float tgtSpeed = 0, tgtTurn = 0;  // đích nhận từ Pi
float curSpeed = 0, curTurn = 0;  // đã làm mượt
bool targetChanged = false;
bool infoRequested = false;
bool everRx = false;
uint32_t lastRx = 0;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

uint8_t crc8(const uint8_t *d, size_t n, uint8_t crc = 0) {
  while (n--) {
    crc ^= *d++;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

void sendFrameImpl(uint8_t type, const void *payload, uint8_t len) {
  uint8_t buf[4 + 255 + 1];
  buf[0] = 0xA5;
  buf[1] = 0x5A;
  buf[2] = type;
  buf[3] = len;
  memcpy(buf + 4, payload, len);
  buf[4 + len] = crc8(buf + 2, len + 2);
  // Bỏ khung nếu bộ đệm TX đầy thay vì chặn vòng cân bằng.
  if (LINK.availableForWrite() >= len + 5) LINK.write(buf, len + 5);
}

void setTarget(float v, float t) {
  v = clampf(v, -CMD_MAX_SPEED, CMD_MAX_SPEED);
  t = clampf(t, -CMD_MAX_TURN, CMD_MAX_TURN);
  // Chỉ báo đổi khi lệnh TIẾN/LÙI đổi: xoay tại chỗ (A/D) không được xoá điểm giữ
  // vị trí, nếu không xe sẽ trôi mỗi lần bấm/thả phím xoay.
  if (v != tgtSpeed) targetChanged = true;
  tgtSpeed = v;
  tgtTurn = t;
}

struct Named {
  const char *name;
  float v, t;
};
const Named NAMED[] = {
    {"FORWARD", CMD_DRIVE_SPEED, 0},
    {"BACKWARD", -CMD_DRIVE_SPEED, 0},
    {"LEFT", CMD_DRIVE_SPEED, -CMD_TURN_SPEED},
    {"RIGHT", CMD_DRIVE_SPEED, CMD_TURN_SPEED},
    {"ROTATE", 0, CMD_ROTATE_SPEED},
    {"ROTATE_RIGHT", 0, CMD_ROTATE_SPEED},
    {"ROTATE_LEFT", 0, -CMD_ROTATE_SPEED},
    {"STOP", 0, 0},
};

void handleLine(char *line) {
  bool ok = false;
  if (strncmp(line, "VEL:", 4) == 0) {
    char *end = nullptr;
    float v = strtof(line + 4, &end);
    if (end && *end == ',') {
      char *end2 = nullptr;
      float t = strtof(end + 1, &end2);
      if (end2 != end + 1 && isfinite(v) && isfinite(t)) {
        setTarget(v, t);
        ok = true;
      }
    }
  } else if (strncmp(line, "CMD:", 4) == 0) {
    for (const Named &n : NAMED) {
      if (strcmp(line + 4, n.name) == 0) {
        bool changed = (n.v != tgtSpeed || n.t != tgtTurn);
        setTarget(n.v, n.t);
        if (changed) Link::sendText("ACK:%s", n.name);
        ok = true;
        break;
      }
    }
    if (!ok) Link::sendText("ERR:UNKNOWN %s", line + 4);
  } else if (strcmp(line, "PING") == 0) {
    Link::sendText("PONG");
    ok = true;
  } else if (strcmp(line, "INFO") == 0) {
    Link::sendText("INFO:fw=%s steps_per_m=%.3f wheel_d=%.4f acc_lsb_g=8192 gyro_lsb_dps=32.8 rate=%d",
                   FW_VERSION, (double)STEPS_PER_METER, (double)WHEEL_DIAMETER_M, LOOP_HZ / TELEM_DIV);
    infoRequested = true;
    ok = true;
  }
  if (ok) {
    lastRx = millis();
    everRx = true;
  }
}

}  // namespace

namespace Link {

void begin() {
  LINK.setRxBufferSize(512);
  LINK.setTxBufferSize(2048);
  LINK.begin(LINK_BAUD, SERIAL_8N1, PIN_LINK_RX, PIN_LINK_TX);
}

void poll() {
  static char buf[48];
  static uint8_t len = 0;
  static bool overflow = false;
  while (LINK.available()) {
    char c = LINK.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (len && !overflow) {
        buf[len] = 0;
        handleLine(buf);
      }
      len = 0;
      overflow = false;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      overflow = true;  // dòng dài bất thường = nhiễu, bỏ cả dòng
    }
  }

  // Watchdog: đang chạy mà mất lệnh -> dừng
  if (everRx && (tgtSpeed != 0 || tgtTurn != 0) && millis() - lastRx >= CMD_TIMEOUT_MS) {
    setTarget(0, 0);
    sendText("ERR:TIMEOUT");
  }
}

void update(float dt) {
  float ds = CMD_SPEED_SLEW * dt, dtn = CMD_TURN_SLEW * dt;
  curSpeed += clampf(tgtSpeed - curSpeed, -ds, ds);
  curTurn += clampf(tgtTurn - curTurn, -dtn, dtn);
}

float speed() { return curSpeed; }
float turn() { return curTurn; }
bool alive() { return everRx && millis() - lastRx < CMD_TIMEOUT_MS; }

bool consumeTargetChanged() {
  bool c = targetChanged;
  targetChanged = false;
  return c;
}

void clear() {
  setTarget(0, 0);
  curSpeed = 0;
  curTurn = 0;
}

void sendFrame(uint8_t type, const void *payload, uint8_t len) { sendFrameImpl(type, payload, len); }
void sendImu(const ImuPacket &p) { sendFrameImpl(T_IMU, &p, sizeof(p)); }

bool consumeInfoRequest() {
  bool r = infoRequested;
  infoRequested = false;
  return r;
}

void sendText(const char *fmt, ...) {
  char s[160];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(s, sizeof(s), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(s) - 1) n = sizeof(s) - 1;
  sendFrameImpl(T_TEXT, s, (uint8_t)n);
#if LINK_UART != 0
  Serial.println(s);  // cổng USB rảnh -> in thêm để debug
#endif
}

void log(const char *fmt, ...) {
  char s[150];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(s, sizeof(s), fmt, ap);
  va_end(ap);
  sendText("LOG:%s", s);
}

}  // namespace Link
