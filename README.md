# iot-ck: Xe cân bằng 2 bánh + SLAM 2D nhìn trần + goto + tránh vật cản + web + giọng nói

```
 MPU-6050 ──I2C──► ESP32 #1 ──UART 460800──► Raspberry Pi 4: robotd (Python) ──stdin──► slam_node (C++)
 HC-SR04 ×3 ─ECHO─┘ │  IMU + bước + pitch 200 Hz   ▲   UART, camera, web :8080 ◄─JSON─┘ EKF, keyframe, loop,
 2x A4988 ◄STEP/DIR─┘  SONAR + SONAR_GEOM (vị trí) ┤   giảm tốc theo siêu âm            pose graph, grid, A*,
                    ◄── VEL:v,t (10–20 Hz) ────────┤                                    vật cản log-odds, replan,
                              camera OV5647 ───────┘ hướng LÊN TRẦN, 640x480 15 fps     pure pursuit
 INMP441 ─I2S─► ESP32 #2 ──WiFi ws://pi:8765/audio──► Speech-control (Zipformer) ──/ws/voice──► robotd (web ưu tiên)
```

Thiết kế và lý do từng lựa chọn: **[SLAM-DESIGN.md](SLAM-DESIGN.md)** (mục 13 là bản rà soát so với xe thật).

| Thư mục | Nội dung |
|---|---|
| `firmware/` | PlatformIO cho ESP32 #1: cân bằng (PID tầng 400 Hz), nhận lệnh, gửi IMU + bước bánh + pitch, siêu âm + dừng khẩn |
| `firmware-mic/` | PlatformIO cho ESP32 #2: mic INMP441 → WiFi WebSocket tới Pi ([README](firmware-mic/README.md)) |
| `pi/slam/` | Lõi SLAM 2D C++17 (`slam_node`), simulator `slam_sim`, GoogleTest |
| `pi/robot/` | `robotd` (Python): UART, camera, chạy `slam_node`, web server (aiohttp), hiệu chỉnh camera |
| `pi/web/` | Giao diện web (1 file HTML, không cần internet) |
| `pi/config/` | `robot.yaml` (robotd), `slam.yaml` (mọi thông số SLAM) |
| `pi/tools/slamlog.py` | Đọc / xuất / vẽ file log để replay |
| `Speech-control/` | Giọng nói: mic INMP441, ASR tiếng Việt, từ điển sửa lỗi, gửi lệnh cho robotd ([README](Speech-control/README.md)) |
| `scripts/` | Chạy trên laptop: `flash_esp32.sh`, `flash_esp32_mic.sh`, `deploy_pi.sh`, `flash_pi_sd.sh`, `sd_scan.py` |

## 1. Đấu dây

| Pi 4 | | ESP32 |
|---|---|---|
| GPIO14 TXD (chân 8) | → | **RX0 (GPIO3)** |
| GPIO15 RXD (chân 10) | ← | **TX0 (GPIO1)** |
| GND (chân 6) | — | GND |

Dây đang nối vào **UART0** của ESP32 (dùng chung cổng USB, `LINK_UART 0`), nên khi nạp code phải thả chân TX của Pi.
`scripts/flash_esp32.sh` tự làm việc này. Khuyên chuyển sang GPIO16/17 và đặt `LINK_UART 2`.

MPU-6050: VCC 3V3, GND, SDA GPIO21, SCL GPIO22, mũi tên X chỉ về trước xe, mặt linh kiện quay lên.

**Siêu âm HC-SR04** → ESP32 #1 (mặc định 3 cái phủ ~110° phía trước, sửa trong `firmware/include/config.h`):

| Cảm biến | TRIG | ECHO (qua cầu chia áp 1k/2k) | Vị trí (so với tâm trục bánh) | Hướng | Dừng khẩn |
|---|---|---|---|---|---|
| #0 trái | GPIO32 | GPIO34 | x 8 cm, y +5 cm | +40° | < 12 cm |
| #1 giữa | GPIO33 | GPIO35 | x 9 cm, y 0 | 0° | < 15 cm |
| #2 phải | GPIO4 | GPIO23 | x 8 cm, y −5 cm | −40° | < 12 cm |

VCC 5 V, GND chung. **Thêm / bớt / dời / đổi góc cảm biến: chỉ sửa bảng `SONAR_SENSORS` trong `firmware/include/config.h`
rồi nạp lại ESP32.** Pi không có cấu hình vị trí cảm biến: ESP32 gửi bảng này lên (khung `SONAR_GEOM`), robotd và
slam_node dùng đúng như vậy.

**Mic INMP441 → ESP32 #2** (riêng, không nối Pi): VDD 3V3, GND, SCK GPIO26, WS GPIO25, SD GPIO33, L/R → GND.
Xem [firmware-mic/README.md](firmware-mic/README.md). (Nối thẳng vào Pi vẫn được: `pi/scripts/setup_mic.sh` + `--source mic`.)

**Camera OV5647 phải gắn nhìn THẲNG LÊN TRẦN** (không nhìn tới trước như hệ OpenVINS cũ). Mặc định mép trên ảnh
hướng về mũi xe (`camera.yaw_deg: 90`). Gắn kiểu khác thì chạy kiểm tra ở mục 5.

## 2. Firmware ESP32 #1 (2.2)

```bash
./scripts/flash_esp32.sh              # build -> SSH dừng robotd + thả TX Pi -> nạp -> trả TX, kiểm tra IMU, bật robotd
NO_PI=1 ./scripts/flash_esp32.sh      # không SSH (đã rút dây TX của Pi)
```

- Bật nguồn để xe nằm yên ~2 s (hiệu chỉnh gyro), dựng thẳng giữ 0,5 s là xe tự đứng. Ngã > 45° thì tắt động cơ.
- Gói IMU 32 byte: thêm `pitch` (góc vòng cân bằng đang dùng) để SLAM bù nghiêng ảnh trần. Pi vẫn nhận gói 28 byte
  của firmware cũ (tự tính pitch, web nhắc nạp firmware mới).
- `flags` bit2: bộ điều khiển chạm giới hạn (dễ trượt bước), SLAM tăng nhiễu odometry lúc đó.
- 2.2: siêu âm đo lần lượt từng cảm biến bằng ngắt (~15 Hz mỗi cái, không chặn vòng cân bằng). Mỗi số đo một khung
  `SONAR` (7 byte); bảng vị trí khung `SONAR_GEOM` lúc khởi động, mỗi 2 s và khi Pi gửi `INFO`. Gói IMU giữ 32 byte.
- Dừng khẩn tại ESP32: lệnh TIẾN bị chặn khi cảm biến đo gần hơn ngưỡng của nó (vẫn cho lùi / xoay), báo `SONAR:STOP`.

ESP32 #2 (mic): `./scripts/flash_esp32_mic.sh` (cắm USB vào laptop, điền WiFi trong `firmware-mic/include/secrets.h`).

## 3. Raspberry Pi: cài đặt

```bash
./scripts/deploy_pi.sh --install    # lần đầu: apt + UART + I2S mic + linger + venv + slam_node + robotd + giọng nói
                                    # xong thì sudo reboot (UART, I2S chỉ có sau khi khởi động lại)
./scripts/deploy_pi.sh --build      # copy code + build lại slam_node + restart robotd (sau khi sửa C++)
./scripts/deploy_pi.sh              # copy code (Python/web/config) + restart robotd
```

Trên Pi: `bash ~/iot-ck/pi/scripts/install.sh [--skip-apt] [--skip-slam] [--skip-voice]`. Gói apt: `libceres-dev libyaml-cpp-dev
libopencv-dev libeigen3-dev` (không cần build GTSAM / OpenVINS). Web: **http://raspberrypi4.local:8080**.
Chạy tay: `cd ~/iot-ck/pi && ../.venv/bin/python -m robot.main`.

## 4. Dùng

1. **Tạo bản đồ**: khởi động là ở chế độ "Tạo bản đồ". Dựng xe, lái chậm (W/S tiến lùi, A/D xoay, Space dừng)
   **giữa các lối đi**, quay lại chỗ cũ vài lần (để có loop closure). Xe chỉ tự đi được trên vùng đã đi qua.
2. **Địa điểm**: lái tới chỗ cần nhớ (cả hướng xe), nút 📍 → nhập tên → "Lưu vị trí hiện tại".
3. **Lưu bản đồ**: nút 💾 hiện bảng kiểm tra (đủ keyframe, camera bám liên tục, có loop closure…), đạt thì lưu.
   Lưu ở `~/iot-ck/maps/<id>/` (keyframe, đồ thị, `grid.png`+`grid.yaml` chuẩn ROS map_server, `places.yaml`).
4. **Lần sau**: nút bản đồ → chọn bản đồ → **Định vị** (chỉ dùng) hoặc **Mở rộng** (vẽ thêm). Xoay xe chậm tại chỗ
   tới khi web báo "Đã tái định vị" (cần 2 lần khớp nhất quán). Muốn tự tải khi bật: `slam.autoload` trong `robot.yaml`.
5. **Goto**: bấm vào vùng xanh trên bản đồ rồi "Đi tới", hoặc "Đi tới" ở một địa điểm. Xe lập đường A* trong
   vùng đã đi, xoay tại chỗ khi lệch > 50°, bám đường bằng pure pursuit (≤ 0,15 m/s). Tự dừng khi: bấm phím bất kỳ,
   Esc, xe ngã, mất định vị, thân giật bất thường (va chạm?), 8 s không tiến triển (vướng / kẹt bánh), mất dữ liệu ESP32
   > 0,2 s, đóng web. **Không né vật cản mới** (xe không có cảm biến khoảng cách).
6. **Nhiệm vụ (mission)**: nút nhiệm vụ ở cột trái (hoặc phím **M**) → bấm lần lượt các điểm trên bản đồ (đánh số 1, 2, 3…;
   Backspace xoá điểm cuối) → **Bắt đầu**. Xe đi một mạch qua từng điểm theo thứ tự, **phải chạm từng điểm** (trong 10 cm)
   rồi mới sang chặng sau, quay đầu được, chỉ dừng ở điểm cuối. Vận tốc / tốc độ quay cố định theo `controller.v_max`,
   `w_max` trong `slam.yaml` (mặc định 0,15 m/s, 1 rad/s), không chỉnh theo từng điểm. Thẻ dưới bản đồ hiện "Tới điểm k/n";
   huỷ, an toàn và "Thử lại" như goto.
7. **Giọng nói** (mic INMP441, dịch vụ `voice`): "**tiến 10**" (cm), "**lùi 20**", "**xoay trái 180**" (độ),
   "**xoay phải 90**", "**dừng**". Nói nhiều lệnh liền thì chạy lần lượt. Đo bằng bước bánh + gyro, không cần bản đồ.
   **Web luôn ưu tiên**: đang lái / goto trên web thì lệnh giọng nói bị bỏ qua, bấm phím trên web huỷ lệnh giọng nói.
   Từ điển sửa lỗi nhận dạng ("tiếng mười" → tiến 10, "soay chái" → xoay trái...) ở `Speech-control/vi_dictionary.py`.
   Âm thanh đến từ ESP32 #2 qua WiFi (Speech-control là WebSocket server cổng 8765).
8. **Tránh vật cản (siêu âm)**, ba lớp:
   - ESP32 dừng khẩn khi vật quá sát (không phụ thuộc Pi).
   - robotd giảm tốc khi TIẾN theo khoảng trống phía trước (từ 50 cm, dừng ở 15 cm) cho web, goto và giọng nói;
     lệnh "tiến" bằng giọng nói gặp vật thì dừng và báo.
   - slam_node ghi vật cản vào lưới log-odds (cách cập nhật của EGO-Planner, rút gọn cho siêu âm): lớp **tĩnh** khi tạo /
     mở rộng bản đồ (neo keyframe, lưu cùng bản đồ) và lớp **tạm** lúc chạy (tự quên ~25 s). Goto: đường phía trước đè lên
     ô vật cản (hoặc siêu âm giữ xe đứng > 1 s) → dừng, quét ±25°, chờ 3 s, còn chặn thì chạy lại A* tránh ô vật cản.
   - Web: ô đỏ = vật cản tĩnh, cam = tạm; nón siêu âm quanh xe; nút **tẩy** (phím E) xoá vật cản ghi nhầm.

| Trên web | Nội dung |
|---|---|
| Nón quanh xe, ô đỏ / cam | Siêu âm (cung đậm = khoảng cách đo được), vật cản tĩnh (đỏ, lưu cùng bản đồ) / tạm (cam, tự quên) |
| Chip Mic / khung Giọng nói | Speech-control đã nối chưa, đang nghe (tím nhấp nháy), câu nghe được → câu đã sửa, lệnh đang chạy + tiến độ |
| Bản đồ | Vùng đã đi (xanh lá = đường tâm planner dùng, xám xanh = vùng thân xe quét), quỹ đạo đã tối ưu, keyframe (xanh = có đặc trưng trần, xám = "mù", đỏ = nghi trượt bánh), loop (vàng), tái định vị (tím), địa điểm, đường A* |
| Góc trên phải | Camera trần (đặc trưng ORB / ảnh gốc), bấm để đổi chỗ với bản đồ |
| Thanh trên | Kết nối, MANUAL/GOTO, trạng thái xe, chế độ SLAM, độ bất định định vị, IMU, chất lượng bản đồ |
| Telemetry | σ định vị, keyframe/loop, ORB, chiều cao trần, bias gyro, thời gian xử lý, ảnh bỏ… |

## 5. Hiệu chỉnh cho xe thật (`pi/config/slam.yaml`)

| Thông số | Cách làm |
|---|---|
| `robot.wheel_base` | Thước: tâm bánh trái tới tâm bánh phải |
| Nội tham số camera | `systemctl --user stop robotd; cd ~/iot-ck/pi && ../.venv/bin/python -m robot.calibrate` (bàn cờ 9x6 ô 25 mm, cầm phía trên camera), dán kết quả vào mục `camera` |
| Hướng gắn camera `camera.yaw_deg` | Web → nút 📷 → "1. Lưu khung mốc", lái thẳng ~0,5 m, "2. Đo". Đúng thì báo "Dấu trục camera ĐÚNG", sai thì in giá trị `yaw_deg` cần sửa |
| `camera.offset_x/y`, `height_above_axle` | Thước: camera so với tâm trục bánh |
| `camera.ceiling_height` | `0` = tự ước lượng từ odometry (web hiện số mẫu), hoặc đo thước (trần − camera) |

Đo xong đặt các cờ `calibrated:` thành `true` để web hết cảnh báo. Trần trơn ít hoa văn: giảm `frontend.orb_fast_threshold`,
bật đèn, hoặc dán vài hình lên trần.

## 6. Thử không cần xe

```bash
# Build + test trên laptop (cần libceres-dev libyaml-cpp-dev libopencv-dev libgtest-dev)
cmake -S pi/slam -B pi/slam/build && cmake --build pi/slam/build -j && (cd pi/slam/build && ctest)

# Kịch bản đầu-cuối: tạo bản đồ (2 vòng 3x2 m, có bánh kẹt) -> lưu -> khởi động lại chỗ khác -> tái định vị -> goto 3 địa điểm
pi/slam/build/slam_sim --config pi/config/slam.yaml --maps /tmp/maps --log /tmp/a.slamlog

# Replay xác định (cùng file -> cùng kết quả từng byte), nhanh hơn thời gian thực
pi/slam/build/slam_node --config pi/config/slam.yaml --replay /tmp/a.slamlog --maps /tmp/m > /tmp/out.jsonl
python3 pi/tools/slamlog.py plot /tmp/out.jsonl

# Web đầy đủ trên laptop, dữ liệu phát lại từ log
cd pi && python3 -m robot.main --replay /tmp/a.slamlog
```

Kết quả `slam_sim` (odometry giả lập: bánh lệch tỉ lệ 1,5%/−1%, gyro bias 0,46°/s, 1,5 s bánh kẹt):

| | Kết quả |
|---|---|
| Lệch keyframe so với thật sau 2 vòng (19 m) | TB 1,8 cm, lớn nhất 5,8 cm |
| Lệch vị trí cuối: SLAM / chỉ odometry | 0,5 cm / 18 cm |
| Tái định vị trên bản đồ đã lưu | đúng, lệch < 1 cm |
| Goto 3 địa điểm (sai số vị trí thật) | 6–8 cm |

Ghi log trên xe để replay: `slam.log_dir` trong `robot.yaml` (khoảng 1 MB/s, thẻ nhớ đầy nhanh).

## 7. Tài nguyên trên Pi 4

`slam_node` chỉ chạy ORB trên khung cần thiết (keyframe mỗi 0,3 m / 15°, định vị 2 lần/giây), không xử lý mọi ảnh.
Trên laptop mỗi keyframe (ORB + khớp + tối ưu) mất ~12 ms, tối ưu đồ thị 100 node ~2 ms. Pi 4 chậm hơn khoảng 5 lần
nên vẫn dư thời gian thực. RAM: mỗi keyframe ~20 kB (500 descriptor), 1000 keyframe ~20 MB. Tái định vị so với mọi
keyframe (brute force), nên với bản đồ rất lớn (> 500 keyframe) mỗi lần thử mất cỡ 1 s.
