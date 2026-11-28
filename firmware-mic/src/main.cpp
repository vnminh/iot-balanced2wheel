// =============================================================================
//  ESP32 #2: INMP441 -> PCM16 16 kHz -> WebSocket -> Raspberry Pi (Speech-control :8765/audio)
//
//   I2S DMA (32 bit, kênh trái) -> lọc DC -> >> MIC_SHIFT -> khung 20 ms (640 byte) -> khung nhị phân WebSocket
//   Mở kết nối: gửi text {"type":"hello",...}. Mỗi 5 s gửi {"type":"stat",...} (RSSI, mức âm, khung bỏ).
//   Mất WiFi / mất Pi: tự nối lại mỗi 2 s, âm thanh trong lúc đó bị bỏ (không đệm, giữ độ trễ thấp).
//   LED: chớp = đang nối, sáng = đang gửi cho Pi.
// =============================================================================
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <driver/i2s_std.h>
#include <math.h>

#include "config.h"
#include "ws_client.h"

namespace {

constexpr int FRAME = SAMPLE_RATE * FRAME_MS / 1000;   // 320 mẫu
i2s_chan_handle_t rx = nullptr;
int32_t raw[FRAME];
int16_t pcm[FRAME];
WsClient ws;
IPAddress piIp;
uint32_t lastTry = 0, lastStat = 0, sent = 0, dropped = 0;
float hpX = 0, hpY = 0;          // lọc thông cao
double sumSq = 0;                // năng lượng để báo mức âm
uint32_t sumN = 0;
float peakDb = -120;

void initI2S() {
  i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  cc.dma_desc_num = 6;
  cc.dma_frame_num = FRAME;
  ESP_ERROR_CHECK(i2s_new_channel(&cc, nullptr, &rx));
  i2s_std_config_t sc = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = (gpio_num_t)PIN_I2S_BCLK,
              .ws = (gpio_num_t)PIN_I2S_WS,
              .dout = I2S_GPIO_UNUSED,
              .din = (gpio_num_t)PIN_I2S_SD,
              .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
          },
  };
  sc.slot_cfg.slot_mask = MIC_RIGHT_CHANNEL ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT;
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx, &sc));
  ESP_ERROR_CHECK(i2s_channel_enable(rx));
}

bool resolvePi() {
  String host = PI_HOST;
  if (piIp.fromString(host)) return true;
  if (host.endsWith(".local")) {
    host.remove(host.length() - 6);
    const IPAddress ip = MDNS.queryHost(host, 2000);
    if (ip != IPAddress(0, 0, 0, 0)) {
      piIp = ip;
      return true;
    }
    return false;
  }
  return WiFi.hostByName(PI_HOST, piIp) == 1;
}

void ensureConnection() {
  if (ws.connected()) return;
  if (millis() - lastTry < RECONNECT_MS) return;
  lastTry = millis();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("WiFi: dang noi \"%s\"...\n", WIFI_SSID);
    return;   // WiFi tự nối lại (setAutoReconnect)
  }
  if (!resolvePi()) {
    Serial.printf("Khong tim thay %s (mDNS). Dat PI_HOST = dia chi IP trong config.h neu can\n", PI_HOST);
    return;
  }
  Serial.printf("Noi ws://%s:%d%s ...\n", piIp.toString().c_str(), PI_PORT, PI_PATH);
  if (!ws.connect(piIp, PI_PORT, PI_HOST, PI_PATH)) return;
  char hello[200];
  snprintf(hello, sizeof hello,
           "{\"type\":\"hello\",\"device\":\"%s\",\"fw\":\"%s\",\"rate\":%d,\"format\":\"s16le\",\"channels\":1,"
           "\"frame_ms\":%d,\"mic_shift\":%d}",
           DEVICE_NAME, FW_VERSION, SAMPLE_RATE, FRAME_MS, MIC_SHIFT);
  ws.sendText(hello);
  Serial.println("Da noi Pi, dang gui am thanh");
}

void sendStat() {
  const float db = sumN ? 10.0f * log10f(float(sumSq / sumN) + 1e-12f) : -120.0f;
  char s[200];
  snprintf(s, sizeof s,
           "{\"type\":\"stat\",\"rssi\":%d,\"sent\":%u,\"dropped\":%u,\"level_dbfs\":%.1f,\"peak_dbfs\":%.1f,"
           "\"uptime_s\":%u}",
           WiFi.RSSI(), (unsigned)sent, (unsigned)dropped, db, peakDb, (unsigned)(millis() / 1000));
  Serial.printf("muc am TB %.1f dBFS, dinh %.1f dBFS, RSSI %d, gui %u, bo %u\n", db, peakDb, WiFi.RSSI(),
                (unsigned)sent, (unsigned)dropped);
  if (ws.connected()) ws.sendText(s);
  sumSq = 0;
  sumN = 0;
  peakDb = -120;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  delay(200);
  Serial.printf("\n%s: INMP441 BCLK=%d WS=%d SD=%d %s, %d Hz, khung %d ms -> ws://%s:%d%s\n", FW_VERSION,
                PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_SD, MIC_RIGHT_CHANNEL ? "kenh phai" : "kenh trai", SAMPLE_RATE,
                FRAME_MS, PI_HOST, PI_PORT, PI_PATH);
  initI2S();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // tắt tiết kiệm điện: giảm trễ và giật âm thanh
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  MDNS.begin(DEVICE_NAME);
}

void loop() {
  // 1. Đọc một khung 20 ms (chặn tối đa ~20 ms: DMA tự đệm, nhịp vòng lặp theo tốc độ mẫu)
  size_t bytes = 0;
  if (i2s_channel_read(rx, raw, sizeof raw, &bytes, 100) != ESP_OK || bytes != sizeof raw) return;
  float peak = 0;
  for (int i = 0; i < FRAME; i++) {
    const float x = float(raw[i] >> MIC_SHIFT);
    hpY = HPF_ALPHA * (hpY + x - hpX);   // lọc thông cao bậc 1
    hpX = x;
    const float y = hpY > 32767.f ? 32767.f : (hpY < -32768.f ? -32768.f : hpY);
    pcm[i] = int16_t(y);
    const float n = y / 32768.f;
    sumSq += n * n;
    if (fabsf(n) > peak) peak = fabsf(n);
  }
  sumN += FRAME;
  const float pdb = 20.0f * log10f(peak + 1e-9f);
  if (pdb > peakDb) peakDb = pdb;

  // 2. Gửi
  ensureConnection();
  if (ws.connected()) {
    if (ws.sendBinary(reinterpret_cast<uint8_t *>(pcm), sizeof pcm)) ++sent;
    else ++dropped;
    ws.poll();
  } else {
    ++dropped;
  }
  if (millis() - lastStat >= STAT_PERIOD_MS) {
    lastStat = millis();
    sendStat();
  }
  digitalWrite(PIN_LED, ws.connected() ? HIGH : ((millis() / 250) % 2));
}
