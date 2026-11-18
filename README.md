# iot-ck: Xe cân bằng 2 bánh + VI-SLAM (OpenVINS + odometry bánh xe) + web điều khiển

```
 MPU-6050 ──I2C──► ESP32 ──UART 460800──► Raspberry Pi 4 ──► OpenVINS (ovw_node) ──► web :8080
 2x A4988 ◄─STEP/DIR─┘   (IMU 200 Hz + số bước 2 bánh)     ▲                       bản đồ, quỹ đạo,
                         ◄── lệnh VEL:v,t (10 Hz) ─────────┤                       nút tiến/lùi/quay/dừng
                                         camera OV5647 ────┘ (15 fps, 640x480)
```

| Thư mục | Nội dung |
|---|---|
| `firmware/` | PlatformIO cho ESP32: cân bằng (PID tầng 400 Hz) + nhận lệnh + gửi IMU/bước bánh |
| `pi/vio/` | `ovw_node` (C++): OpenVINS không ROS + cập nhật EKF từ odometry bánh xe |
| `pi/robot/` | `robotd` (Python): UART, camera, chạy ovw_node, web server (aiohttp) |
| `pi/web/` | Giao diện web (1 file HTML, không cần internet) |
| `pi/config/` | `robot.yaml`, cấu hình OpenVINS (`estimator_config.yaml`, `kalibr_*.yaml`), `wheel.yaml` |
| `pi/tests/sim_feed.py` | Dữ liệu mô phỏng để thử ovw_node không cần xe |

## 1. Đấu dây (hiện tại)

| Pi 4 | | ESP32 |
|---|---|---|
| GPIO14 TXD (chân 8) | → | **RX0 (GPIO3)** |
| GPIO15 RXD (chân 10) | ← | **TX0 (GPIO1)** |
| GND (chân 6) | — | GND |

Dây đang nối vào **UART0** của ESP32, dùng chung với cổng USB. Firmware đặt sẵn `LINK_UART 0` cho khớp.
Hệ quả: khi nạp code từ máy tính phải thả chân TX của Pi:

```bash
ssh iot09@raspberrypi4.local 'bash ~/iot-ck/pi/scripts/esp32_flash_mode.sh release'
cd firmware && pio run -e main -t upload
ssh iot09@raspberrypi4.local 'bash ~/iot-ck/pi/scripts/esp32_flash_mode.sh restore'
```

Khuyên dùng: chuyển 2 dây sang ESP32 **GPIO16 (RX2) / GPIO17 (TX2)** và đặt `LINK_UART 2` trong
`firmware/include/config.h`. Khi đó cổng USB rảnh để nạp và debug.

MPU-6050: VCC 3V3, GND, SDA GPIO21, SCL GPIO22. Nếu IMU không có điện, ESP32 sẽ báo
`LOG:Khong thay MPU-6050 ...` (xem ở web, mục "Nhật ký ESP32") và tự chạy tiếp khi có điện.

## 2. Firmware ESP32

```bash
cd firmware
pio run -e main -t upload     # nhớ thả chân TX của Pi trước (mục 1)
```

- Vẫn là xe **tự cân bằng**: bật nguồn để xe nằm yên khoảng 2 s (hiệu chỉnh gyro), dựng thẳng giữ 0,5 s là xe tự đứng.
- Vref đã vặn tối đa và hết rung, nên giới hạn được trả lại mức thiết kế: `MAX_SPEED_MPS 0.9`, `MAX_ACCEL_MPS2 8`.
  A4988 ở Vref tối đa rất nóng, **bắt buộc có tản nhiệt**.
- Lệnh từ Pi: `VEL:<v>,<t>` (m/s), hoặc `CMD:FORWARD|BACKWARD|LEFT|RIGHT|ROTATE|STOP` (tương thích `uart_link.py` cũ).
  Quá 500 ms không nhận lệnh nào thì xe tự dừng.
- Gửi sang Pi khung nhị phân 200 Hz gồm thời gian, accel, gyro và tổng số bước 2 bánh. Định dạng xem `firmware/src/link.h`.

## 3. Raspberry Pi: cài một lần

```bash
ssh iot09@raspberrypi4.local
sudo bash ~/iot-ck/pi/scripts/install_deps.sh   # apt: cmake, eigen, opencv, ceres, boost (CẦN sudo)
bash ~/iot-ck/pi/scripts/build_vio.sh            # tải OpenVINS + biên dịch ovw_node (~15-25 phút)
bash ~/iot-ck/pi/scripts/setup_python.sh         # venv + aiohttp
bash ~/iot-ck/pi/scripts/install_service.sh      # chạy robotd tự động khi bật Pi
```

Chạy tay (thay vì dịch vụ): `cd ~/iot-ck/pi && ../.venv/bin/python -m robot.main`.
Web: **http://raspberrypi4.local:8080**.

## 4. Dùng

1. Bật xe, dựng cho cân bằng. Hướng camera vào cảnh **nhiều chi tiết** (tránh tường trơn).
2. Đứng yên 2–3 s: OpenVINS tự khởi tạo (ZUPT lúc đầu), web báo "Đang bám".
3. Giữ nút ▲ ▼ ⟲ ⟳ (hoặc W A S D) để chạy, thả ra là dừng. Chạy chậm (≤ 0,15 m/s, quay ≤ 30°/s), vì camera rolling shutter dễ mất bám.
4. Bản đồ: điểm 3D (màu theo độ cao) chiếu xuống mặt đất, kèm đường đỏ là quỹ đạo. Nút "Tải bản đồ .ply" xuất ra file mở được bằng MeshLab hoặc CloudCompare.

## 5. VI-SLAM: odometry bánh xe được đưa vào thế nào

OpenVINS là bộ lọc MSCKF (camera + IMU). `ovw_node` thêm một bước cập nhật EKF sau mỗi ảnh:

- Bánh không trượt nên số bước = quãng đường bánh lăn **so với thân xe**. Xe cân bằng thì thân chúi qua lại,
  nên tốc độ tâm trục so với đất là `v = (bước/s)/STEPS_PER_METER + r·ω_chúi`.
- Đo lường gồm 3 thành phần, với u là vận tốc tâm trục trong khung IMU:
  - tiến `d·u = v`;
  - không trượt ngang `a·u = 0`;
  - không đi lên/xuống `v_z = 0`.

  Jacobian tính theo hướng, vận tốc và bias gyro. Có kiểm định χ² để loại lúc va chạm hoặc bị nhấc lên.
- Chỉ cập nhật khi động cơ đang bật (`flags` bit1). Khi xe nằm, bánh quay tự do nên bỏ qua.

Mô phỏng (`pi/tests/sim_feed.py`, xe cân bằng chạy 1,05 m) cho kết quả: khởi tạo sau 2 s, quãng đường ước lượng
1,052 m, 261/261 lần cập nhật bánh xe được nhận, điểm tường nằm đúng ở 2,5 m.

## 6. Cần đo / hiệu chỉnh cho xe thật

| Thông số | File | Ghi chú |
|---|---|---|
| Nội tham số camera | `pi/config/kalibr_imucam_chain.yaml` | Đang là ước lượng cho ống kính 3,6 mm. Chạy `../.venv/bin/python -m robot.calibrate` với bàn cờ 9x6 |
| Vị trí camera so với MPU `T_imu_cam` (cột cuối) | như trên | Đo bằng thước (m). Hướng mặc định: camera nhìn thẳng về trước |
| Tâm trục bánh so với MPU `p_axle_in_imu` | `pi/config/wheel.yaml` | MPU cao hơn trục bao nhiêu thì `z = -khoảng cách` |
| Phơi sáng / gain | `pi/config/robot.yaml` | Mặc định cố định 12 ms, gain 8. Phòng sáng thì giảm gain |

## 7. RAM trên Pi 4 4 GB

Đo thực tế khi chạy robotd (UART + camera + web): khoảng 400 MB đã dùng, còn trống 3,3 GB.
OpenVINS với cấu hình này (11 clone, 30 điểm SLAM, 150 đặc trưng) ước tính dùng khoảng 150–300 MB (chưa đo trên Pi vì ovw_node chưa build xong).
Bản đồ gộp theo voxel 5 cm, tối đa 200k điểm (khoảng 30 MB). Web phục vụ 1 người xem thì không đáng kể. **Thừa RAM.**
Giới hạn thật là CPU: trên máy x86 mỗi ảnh mất 3–20 ms, Pi 4 ước tính chậm hơn khoảng 5 lần, nên camera để 15 fps. Ảnh nào node xử lý không kịp sẽ bị bỏ, IMU thì không bao giờ bỏ.

## 8. Thử ovw_node không cần xe

```bash
cd ~/iot-ck/pi
python3 tests/sim_feed.py | vio/build/ovw_node config/estimator_config.yaml config/wheel.yaml > /tmp/out.jsonl
```

Script này cần `python3-opencv`.
