# firmware-mic: ESP32 #2 thu âm cho điều khiển giọng nói

```
INMP441 --I2S 16 kHz--> ESP32 #2 --WiFi, WebSocket (PCM16, khung 20 ms)--> Pi: Speech-control ws://pi:8765/audio
                                                                            (VAD + Zipformer + từ điển -> robotd)
```

ESP32 này chỉ thu và gửi âm thanh thô. Nhận dạng chạy trên Pi. Pi là **server**, ESP32 tự tìm Pi bằng mDNS
(`raspberrypi4.local`) và tự nối lại khi mất WiFi hoặc Pi khởi động lại.

## Đấu dây INMP441 → ESP32 #2

| INMP441 | ESP32 | |
|---|---|---|
| VDD | 3V3 | **không** dùng 5 V |
| GND | GND | |
| SCK | GPIO26 | `PIN_I2S_BCLK` |
| WS | GPIO25 | `PIN_I2S_WS` |
| SD | GPIO33 | `PIN_I2S_SD` |
| L/R | GND | kênh trái (`MIC_RIGHT_CHANNEL 0`) |

Đổi chân trong `include/config.h`.

## Nạp

```bash
cp firmware-mic/include/secrets.h.example firmware-mic/include/secrets.h   # điền WiFi Pi đang dùng
./scripts/flash_esp32_mic.sh              # build + nạp + mở log (cắm USB ESP32 #2 vào laptop)
```

Log mong đợi: `Da noi Pi, dang gui am thanh`, rồi mỗi 5 s một dòng mức âm (`muc am TB -45 dBFS, dinh -30 dBFS, RSSI -55`).
Nói bình thường cách 0,5–1 m nên thấy đỉnh khoảng −30…−12 dBFS. Nhỏ quá: giảm `MIC_SHIFT` (13 → 12 = ×2); vỡ tiếng
(đỉnh ≈ 0 dBFS): tăng `MIC_SHIFT`.

## Giao thức

- Mở kết nối: text `{"type":"hello","device":"iotck-mic","fw":"iot-ck-mic-1.0","rate":16000,"format":"s16le","channels":1,"frame_ms":20,"mic_shift":13}`
- Âm thanh: mỗi message nhị phân = 320 mẫu int16 little-endian (640 byte, 20 ms), ≈ 32 kB/s.
- Mỗi 5 s: text `{"type":"stat","rssi":-55,"sent":…,"dropped":…,"level_dbfs":…,"peak_dbfs":…}`.
- Mất kết nối: âm thanh trong lúc đó bị bỏ (không đệm, để giữ độ trễ thấp), thử nối lại mỗi 2 s.

WebSocket client tự viết (`src/ws_client.cpp`, RFC 6455 tối giản), không phụ thuộc thư viện ngoài.

## Kiểm tra trên Pi

```bash
journalctl --user -u voice -f        # "ESP32 mic đã nối từ 192.168.x.x", "ESP32 mic: iotck-mic ..., 16000 Hz"
cd ~/iot-ck/Speech-control && systemctl --user stop voice && ../.venv/bin/python speech_control.py --level
```

Web: chip **Mic** hiện tên ESP32 #2, RSSI và mức âm.
