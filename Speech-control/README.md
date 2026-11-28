# Speech-control: điều khiển xe cân bằng bằng giọng nói tiếng Việt

Gốc: https://github.com/vnminh/Speech-control (bản điều khiển đèn/quạt), đã sửa lại để điều khiển xe iot-ck.

```
INMP441 ─I2S─► ESP32 #2 ─WiFi ws://pi:8765/audio─► speech_control.py ─ws://127.0.0.1:8080/ws/voice─► robotd ─► ESP32 #1
 (kênh trái)    (firmware-mic/, PCM16 16 kHz)      (server) VAD + Zipformer       {"type":"move","distance_m":0.1}
                                                   + từ điển (vi_dictionary)       {"type":"turn","angle_deg":-90}
```

Nguồn âm thanh `--source ws` (mặc định): Speech-control mở WebSocket server cổng 8765, ESP32 #2 tự nối tới và gửi
PCM. `--source mic`: đọc INMP441 nối thẳng vào Pi (cần `sudo bash ~/iot-ck/pi/scripts/setup_mic.sh`).

## Lệnh

| Nói | Xe làm |
|---|---|
| **tiến 10** | tiến 10 cm (mặc định cm; "tiến một mét rưỡi", "tiến 2 tấc", "tiến năm mươi phân") |
| **lùi 20** | lùi 20 cm |
| **xoay trái 180** | xoay tại chỗ sang trái 180° ("xoay trái nửa vòng", "quay đầu") |
| **xoay phải 90** | xoay phải 90° ("quay phải chín mươi độ", "rẽ phải 45") |
| **dừng** / dừng lại | huỷ lệnh giọng nói đang chạy + hàng đợi, và cả goto/nhiệm vụ của web (an toàn) |

- Nói liền nhiều lệnh được: "tiến 50 rồi xoay trái 90 rồi lùi 10" -> chạy lần lượt (tối đa 5 lệnh chờ).
- Số đọc bằng chữ hay chữ số đều được: "một trăm tám mươi", "hai lăm", "trăm rưỡi", "180".
- Câu không đủ (thiếu số, thiếu trái/phải, sai đơn vị như "tiến 90 độ") thì **không chạy**, chỉ báo lỗi.
- Giới hạn: 3 m / lệnh, 720° / lệnh, tốc độ 0,10 m/s, quay 0,8 rad/s (`voice:` trong `pi/config/robot.yaml`).

## Ưu tiên: web hơn giọng nói

robotd nhận lệnh giọng nói ở endpoint riêng `/ws/voice` (chỉ từ 127.0.0.1) nên phân biệt được với web (`/ws`):

- Đang lái tay trên web, vừa thả phím < 2 s, hoặc đang goto / nhiệm vụ: lệnh giọng nói **bị từ chối**.
- Bấm phím / DỪNG / goto / nhiệm vụ trên web: lệnh giọng nói đang chạy **bị huỷ ngay**.
- Đóng web không dừng lệnh giọng nói. Speech-control mất kết nối thì robotd dừng lệnh giọng nói.
- Xe không cân bằng, mất dữ liệu ESP32 > 0,3 s, hay 3 s không tiến triển (kẹt): dừng lệnh.

Web hiện chip **Mic** (tím nhấp nháy = đang nghe), khung "Giọng nói" dưới bàn phím (câu nghe được, câu đã sửa,
lệnh đang chạy + tiến độ, hàng đợi) và chế độ **VOICE**.

## Từ điển sửa lỗi ASR (`vi_dictionary.py`)

Zipformer hay nhầm thanh điệu và phương ngữ, ví dụ "tiếng/tiền/kiến" → tiến, "lui/nùi/lụi" → lùi,
"soay/xoài/xoáy" → xoay, "chái/trai" → trái, "phái/phai/phẩy" → phải, "đồ/đổ" → độ, "chăm" → trăm,
"sau/xáu" → sáu, "nam/nằm" → năm, "xăng ti mét/xen ti/phân" → cm. Thứ tự tra: cụm từ → từ chuẩn → từ đệm (bỏ "lên",
"lại", "khoảng"...) → lỗi đã biết → bỏ dấu → sai 1 ký tự với từ khoá. Thêm lỗi mới: xem log dòng `🧠 ASR`, thêm vào
`CONFUSIONS`, chạy test:

```bash
python3 test_robot_commands.py                       # ~70 câu mẫu (đúng / sai chính tả / không phải lệnh)
python3 robot_commands.py "soay chái chín mươi đồ"   # xem câu được chuẩn hoá và tách lệnh thế nào
```

## Phần cứng (chỉ khi `--source mic`): INMP441 → Pi 4. Mặc định mic nằm trên ESP32 #2, xem `firmware-mic/README.md`

| INMP441 | Pi 4 |
|---|---|
| VDD | 3V3 (chân 1), **không** dùng 5 V |
| GND | GND (chân 6) |
| SCK | GPIO18 (chân 12) |
| WS | GPIO19 (chân 35) |
| SD | GPIO20 (chân 38) |
| L/R | GND → **kênh trái** (`--channel 0`) |

Không trùng UART ESP32 (GPIO14/15). Overlay `googlevoicehat-soundcard` còn chiếm GPIO16.

## Cài đặt

Cài cùng toàn bộ hệ thống: `bash ~/iot-ck/pi/scripts/install.sh` (đã gồm bước giọng nói). Hoặc riêng (không cần sudo):

```bash
bash ~/iot-ck/pi/scripts/setup_voice.sh        # sherpa-onnx, tải mô hình (~50 MB), dịch vụ voice (systemd --user)
```

Rồi nạp ESP32 #2: `./scripts/flash_esp32_mic.sh` trên laptop (điền WiFi trong `firmware-mic/include/secrets.h`).

Dịch vụ `voice` chạy cùng `robotd` khi Pi bật. Tham số ở `voice.env` (`--source`, `--listen`, `--gain`, `--chunk-size`...).

```bash
journalctl --user -u voice -f                  # xem câu nghe được và lệnh
systemctl --user restart voice                 # sau khi sửa voice.env
systemctl --user disable --now voice           # tắt hẳn giọng nói
```

## Kiểm tra mic và thử

```bash
cd ~/iot-ck/Speech-control
systemctl --user stop voice                    # cổng 8765 / mic chỉ một chương trình dùng được
../.venv/bin/python speech_control.py --level  # đợi ESP32 #2 nối; im lặng ~ -55 dBFS, nói ~ -25..-10 dBFS (chỉnh MIC_SHIFT trên ESP32)
../.venv/bin/python speech_control.py --dry-run   # nghe mic, in lệnh, KHÔNG gửi xe
../.venv/bin/python speech_control.py --keyboard  # gõ lệnh thay cho nói (gửi xe thật)
../.venv/bin/python speech_control.py --text "tiến 10"
systemctl --user start voice
```

## Mô hình

- Zipformer: https://huggingface.co/hynt/Zipformer-30M-RNNT-Streaming-6000h (chunk 32 mặc định, `CHUNK_SIZE=16
  bash setup_voice.sh` để trễ ít hơn). Giấy phép **cc-by-nc-nd-4.0**: chỉ dùng phi thương mại.
- Silero VAD: https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx
- Trên Pi 4 chạy 2 luồng CPU, beam 4, `Nice=5` để không giành CPU của robotd / slam_node.
