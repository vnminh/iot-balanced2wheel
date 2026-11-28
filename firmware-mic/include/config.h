#pragma once
// =============================================================================
//  ESP32 #2: THU ÂM CHO ĐIỀU KHIỂN GIỌNG NÓI
//  INMP441 --I2S--> ESP32 --WiFi, WebSocket (nhị phân PCM16)--> Pi: Speech-control (server :8765/audio)
//
//  ESP32 chỉ thu và gửi âm thanh thô. Nhận dạng (VAD + Zipformer + từ điển) chạy trên Pi.
// =============================================================================
#include "secrets.h"   // WIFI_SSID, WIFI_PASS (chép từ secrets.h.example)

// ----------------------------- INMP441 -> ESP32 -----------------------------
//   VDD -> 3V3, GND -> GND, L/R -> GND (mic ở kênh TRÁI)
//   SCK -> PIN_I2S_BCLK, WS -> PIN_I2S_WS, SD -> PIN_I2S_SD
#define PIN_I2S_BCLK      26
#define PIN_I2S_WS        25
#define PIN_I2S_SD        33
#define MIC_RIGHT_CHANNEL 0       // 1 nếu L/R nối 3V3
#define PIN_LED           2       // sáng = đã nối Pi, chớp = đang nối WiFi / Pi

// ----------------------------- ÂM THANH -------------------------------------
#define SAMPLE_RATE       16000   // Hz, đúng tần số mô hình Zipformer (Pi không phải đổi tần số)
#define FRAME_MS          20      // mỗi khung WebSocket 20 ms = 320 mẫu = 640 byte
// INMP441 cho 24 bit trong khung 32 bit, tiếng nói bình thường khá nhỏ. Mẫu 16 bit = mẫu32 >> MIC_SHIFT.
// 16 = không khuếch đại, 14 = ×4, 12 = ×16. Chỉnh theo mức dBFS in ra cổng USB (nói thường nên -30..-12 dBFS).
#define MIC_SHIFT         13
#define HPF_ALPHA         0.995f  // lọc thông cao bỏ lệch DC (~13 Hz ở 16 kHz)

// ----------------------------- MẠNG ------------------------------------------
#define PI_HOST           "raspberrypi4.local"   // tên mDNS của Pi, hoặc địa chỉ IP "192.168.1.50"
#define PI_PORT           8765
#define PI_PATH           "/audio"
#define DEVICE_NAME       "iotck-mic"            // tên mDNS của ESP32 này
#define RECONNECT_MS      2000
#define STAT_PERIOD_MS    5000                   // gửi thống kê (RSSI, mức âm, khung bỏ) cho Pi
#define FW_VERSION        "iot-ck-mic-1.0"
