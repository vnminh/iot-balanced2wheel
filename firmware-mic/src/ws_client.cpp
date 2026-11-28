#include "ws_client.h"

#include <esp_random.h>
#include <mbedtls/base64.h>

bool WsClient::connect(const IPAddress &ip, uint16_t port, const char *host, const char *path) {
  close();
  if (!cli_.connect(ip, port, 3000)) return false;
  cli_.setNoDelay(true);
  uint8_t key[16];
  for (auto &b : key) b = uint8_t(esp_random());
  unsigned char key64[32];
  size_t olen = 0;
  mbedtls_base64_encode(key64, sizeof key64, &olen, key, sizeof key);
  key64[olen] = 0;
  char req[256];
  snprintf(req, sizeof req,
           "GET %s HTTP/1.1\r\nHost: %s:%u\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
           "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n",
           path, host, port, key64);
  cli_.print(req);
  // Đọc dòng trạng thái + header tới dòng trống
  String status;
  const uint32_t t0 = millis();
  bool first = true, gotEnd = false;
  String line;
  while (millis() - t0 < 3000 && cli_.connected()) {
    while (cli_.available()) {
      const char c = cli_.read();
      if (c == '\n') {
        if (first) status = line, first = false;
        if (line.length() <= 1) {   // "\r" hoặc rỗng: hết header
          gotEnd = true;
          break;
        }
        line = "";
      } else {
        line += c;
      }
    }
    if (gotEnd) break;
    delay(2);
  }
  if (!gotEnd || status.indexOf(" 101") < 0) {
    Serial.printf("WS: bat tay that bai: %s\n", status.c_str());
    cli_.stop();
    return false;
  }
  ok_ = true;
  return true;
}

void WsClient::close() {
  if (ok_ && cli_.connected()) {
    const uint8_t code[2] = {0x03, 0xE8};   // 1000 đóng bình thường
    sendFrame(0x8, code, 2);
  }
  cli_.stop();
  ok_ = false;
}

bool WsClient::sendFrame(uint8_t opcode, const uint8_t *d, size_t n) {
  if (!ok_ || n > 1024) return false;
  size_t h = 0;
  tx_[h++] = 0x80 | opcode;   // FIN + opcode
  if (n < 126) {
    tx_[h++] = 0x80 | uint8_t(n);   // client -> server luôn có mask
  } else {
    tx_[h++] = 0x80 | 126;
    tx_[h++] = uint8_t(n >> 8);
    tx_[h++] = uint8_t(n);
  }
  const uint32_t m = esp_random();
  const uint8_t mask[4] = {uint8_t(m), uint8_t(m >> 8), uint8_t(m >> 16), uint8_t(m >> 24)};
  memcpy(tx_ + h, mask, 4);
  h += 4;
  for (size_t i = 0; i < n; i++) tx_[h + i] = d[i] ^ mask[i & 3];
  const size_t total = h + n;
  if (cli_.write(tx_, total) != total) {   // mạng nghẽn / mất: đóng, vòng chính sẽ nối lại
    cli_.stop();
    ok_ = false;
    return false;
  }
  return true;
}

bool WsClient::readExact(uint8_t *buf, size_t n, uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  size_t got = 0;
  while (got < n && millis() - t0 < timeout_ms && cli_.connected()) {
    const int a = cli_.available();
    if (a > 0) got += cli_.read(buf + got, n - got);
    else delay(1);
  }
  return got == n;
}

void WsClient::poll() {
  while (ok_ && cli_.available() >= 2) {
    uint8_t hdr[2];
    if (!readExact(hdr, 2, 200)) break;
    const uint8_t op = hdr[0] & 0x0F;
    uint64_t len = hdr[1] & 0x7F;
    const bool masked = hdr[1] & 0x80;
    if (len == 126) {
      uint8_t b[2];
      if (!readExact(b, 2, 200)) break;
      len = (uint16_t(b[0]) << 8) | b[1];
    } else if (len == 127) {
      uint8_t b[8];
      if (!readExact(b, 8, 200)) break;
      len = 0;
      for (int i = 0; i < 8; i++) len = (len << 8) | b[i];
    }
    uint8_t mk[4] = {0, 0, 0, 0};
    if (masked && !readExact(mk, 4, 200)) break;
    uint8_t payload[125];
    const size_t keep = len <= sizeof payload ? size_t(len) : 0;
    if (keep) {
      if (!readExact(payload, keep, 500)) break;
      for (size_t i = 0; i < keep; i++) payload[i] ^= mk[i & 3];
    } else {   // khung lớn (server không gửi gì lớn): bỏ qua
      for (uint64_t i = 0; i < len && cli_.connected(); i++) {
        uint8_t x;
        if (!readExact(&x, 1, 200)) break;
      }
    }
    if (op == 0x9) {
      sendFrame(0xA, payload, keep);   // ping -> pong cùng nội dung
    } else if (op == 0x8) {
      Serial.println("WS: server dong ket noi");
      cli_.stop();
      ok_ = false;
    } else if (op == 0x1 && keep) {
      payload[keep < sizeof payload ? keep : sizeof payload - 1] = 0;
      Serial.printf("WS <- %s\n", reinterpret_cast<char *>(payload));
    }
  }
  if (ok_ && !cli_.connected()) ok_ = false;
}
