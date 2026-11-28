#pragma once
// WebSocket client tối giản (RFC 6455) trên WiFiClient: bắt tay HTTP, gửi khung có mask, trả lời ping,
// nhận close. Đủ cho một luồng âm thanh một chiều, không phụ thuộc thư viện ngoài.
#include <Arduino.h>
#include <WiFi.h>

class WsClient {
public:
  bool connect(const IPAddress &ip, uint16_t port, const char *host, const char *path);
  bool connected() { return ok_ && cli_.connected(); }
  void close();
  bool sendText(const char *s) { return sendFrame(0x1, reinterpret_cast<const uint8_t *>(s), strlen(s)); }
  bool sendBinary(const uint8_t *d, size_t n) { return sendFrame(0x2, d, n); }
  void poll();   // đọc khung từ server: ping -> pong, close -> đóng

private:
  bool sendFrame(uint8_t opcode, const uint8_t *d, size_t n);
  bool readExact(uint8_t *buf, size_t n, uint32_t timeout_ms);
  WiFiClient cli_;
  bool ok_ = false;
  uint8_t tx_[8 + 1024];
};
