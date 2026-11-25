# Thiết kế SLAM 2D cho xe cân bằng hai bánh

Tài liệu này mô tả ý tưởng và yêu cầu để các coding agent triển khai hệ định vị, tạo bản đồ và goto location cho một xe cân bằng hai bánh chạy trên Raspberry Pi 4. Đọc hết mục "Nguyên tắc" và "Kiến trúc" trước khi viết code cho bất kỳ module nào.

---

## 1. Mục tiêu và phạm vi

**Mục tiêu**

- Xe tự biết vị trí của mình trên một bản đồ đã lưu, sai số chấp nhận được 10–30 cm.
- Tạo bản đồ bằng cách lái xe bằng tay quanh nhà một lần (hoặc vài lần).
- Ra lệnh `goto <tên địa điểm>`, xe tự đi tới, **chỉ đi trên những vùng đã từng đi qua**.
- Bật máy ở chỗ bất kỳ đã có trong bản đồ, xe tự tái định vị.

**Không thuộc phạm vi**

- Tránh vật cản động. Xe không có cảm biến khoảng cách (xem mục 11).
- Vòng điều khiển cân bằng. Phần này chạy trên vi điều khiển và đã có sẵn; Pi chỉ gửi lệnh vận tốc.
- SLAM 3D, bundle adjustment, học sâu.

---

## 2. Phần cứng và thông số

| Thành phần | Giá trị | Ghi chú |
|---|---|---|
| Máy tính chính | Raspberry Pi 4, 4 GB | **Raspberry Pi OS 64-bit (Trixie, Debian 13)** đang chạy trên xe, xem mục 2.1 |
| Vi điều khiển | **ESP32 DevKit** | Vòng cân bằng PID 400 Hz, phát xung A4988, đếm bước (`firmware/`) |
| Camera | **OV5647 (Pi Camera v1, CSI)**, mono (dùng kênh Y), **hướng lên trần** | Picamera2, 640x480 @ 15 FPS, rolling shutter, phơi sáng cố định 12 ms. **Phải tháo camera đang nhìn tới trước, gắn lại nhìn lên trần.** |
| IMU | **MPU-6050** trên ESP32 | X tới, Y trái, Z lên. Dùng gyro (chiếu lên trục đứng) và pitch |
| Động cơ | 2 stepper 42 + A4988 | **Không có encoder**: số bước là số bước RA LỆNH. Trượt bước/kẹt bánh thì odometry sai mà không ai biết (xem M3, M6) |
| Đường kính bánh | **65 mm** | Chu vi = π × 0,065 ≈ **0,2042 m** |
| Bề rộng xe | **≈ 20 cm** | Bán kính bao ≈ 0,10 m |
| Khoảng cách hai bánh (wheel base) | TBD (mặc định 0,17 m) | Đo tâm bánh đến tâm bánh |
| Bước / vòng | **200 × microstep 16 = 3200** | `firmware/include/config.h` |
| Liên kết MCU ↔ Pi | UART0 ESP32 ↔ `/dev/serial0`, **460800 baud** | Dùng chung với cổng USB của ESP32 (khi nạp phải thả chân TX của Pi) |
| Chiều cao camera → trần | TBD (mặc định 2,30 m) | Đo bằng thước, **hoặc để hệ tự ước lượng** từ odometry (mục M6, `ceiling_height: 0`) |

**Hằng số dẫn xuất (tính trong code từ config, không hard-code):**

```
meters_per_step = PI * wheel_diameter / (steps_per_rev * microsteps)
# Ví dụ: PI * 0.065 / 3200 ≈ 6.38e-5 m/bước
```

Mọi giá trị TBD phải nằm trong file config, có giá trị mặc định hợp lý và log cảnh báo khi đang dùng mặc định.

### 2.1. Môi trường Raspberry Pi OS

Hệ chạy trên **Raspberry Pi OS 64-bit (Trixie)** (thẻ nhớ hiện tại). Những điểm agent cần biết:

- **Không dùng ROS 2 trên Pi.** ROS 2 không có gói chính thức cho Raspberry Pi OS (chỉ Ubuntu). ROS 2 / RViz2, nếu muốn, chỉ chạy trên laptop để xem dữ liệu (xem M12).
- **Thư viện cài bằng apt** (tất cả có sẵn trong Trixie, không phải build gì từ source):
  ```bash
  sudo apt install build-essential cmake git libeigen3-dev libopencv-dev \
       libceres-dev libyaml-cpp-dev libgtest-dev
  ```
- **Tối ưu đồ thị dùng Ceres Solver** thay cho GTSAM (mục M7). *Lý do (rà soát):* GTSAM không có trong apt, build trên Pi 4 mất 1–2 giờ với swap 2 GB, mà thẻ nhớ của xe là thẻ giả dung lượng và nguồn Pi đang sụt áp, build dài dễ hỏng. Ceres có sẵn trong apt, có robust loss Huber và manifold góc, đủ cho pose graph 2D vài trăm node.
- **Camera:** Picamera2 (Python, stack libcamera) đã chạy ổn định trên xe, có watchdog tự mở lại khi camera treo do sụt áp. Không viết lại phần đọc camera bằng C++ libcamera (xem nguyên tắc 3).
- **Serial:** `/dev/serial0` (UART0 của ESP32), đã bật UART và tắt serial console.
- **Hiệu năng:** build Release `-O2` (đủ nhanh, ít RAM khi biên dịch trên Pi 4 GB); theo dõi nhiệt độ và `vcgencmd get_throttled` (sụt áp làm camera treo).
- **Chạy tự động:** service `systemd` `robotd` (đã có).

---

## 3. Nguyên tắc

1. **Đơn vị SI** ở mọi nơi: mét, radian, giây. Góc luôn được wrap về (−π, π].
2. **Timestamp** dùng đồng hồ monotonic của Pi, gắn ngay khi nhận dữ liệu. Không trộn đồng hồ của vi điều khiển với đồng hồ Pi mà không đồng bộ.
3. **Lõi SLAM (M2–M11) là C++17 thuần** trong một tiến trình `slam_node`, không phụ thuộc ROS. Phần vào/ra (UART với ESP32, camera Picamera2, web) do `robotd` (Python, đã có và chạy ổn) đảm nhận: nó chỉ chuyển dữ liệu thô vào stdin của `slam_node` và chuyển lệnh vận tốc của `slam_node` xuống ESP32. *Lý do (rà soát):* viết lại camera libcamera + web server bằng C++ tốn công mà không nhẹ hơn; ranh giới stdin/stdout còn cho phép ghi log và replay chính xác từng byte. Cùng một code build được trên Pi và trên laptop để test offline, chạy lại dữ liệu ghi sẵn. Tên frame (`map`, `odom`, `base_link`) vẫn theo quy ước ROS REP-105.
4. **Mọi module có unit test** và chạy được trên dữ liệu ghi sẵn (replay). Cùng một file dữ liệu phải cho cùng một kết quả.
5. **Không chặn luồng điều khiển.** Tối ưu đồ thị và loop closure chạy ở luồng riêng; EKF và controller không bao giờ phải chờ chúng.
6. **Bộ điều khiển chuyển động chỉ dùng frame `odom`.** Chỉ planner và goto dùng frame `map`.
7. **Không ghép sai còn hơn ghép nhiều.** Khi nghi ngờ một loop closure, bỏ nó.

---

## 4. Kiến trúc tổng thể

### 4.0. Đây là SLAM 2D, với ba thành phần có vai trò tách bạch

**Hệ là SLAM 2D.** Giả định sàn phẳng; mọi pose là `Pose2 (x, y, θ)`. Không ước lượng z, roll, pitch. Góc pitch từ IMU chỉ dùng để bù ảnh nghiêng và phát hiện va chạm, không nằm trong trạng thái.

| Thành phần | Vai trò | Không làm |
|---|---|---|
| **EKF 2D** (M3) | Vị trí mượt, tần số cao → `odom → base_link` | Không sửa drift dài hạn |
| **Pose graph** (M7) | Nơi tiếp nhận loop closure: giữ keyframe + cạnh odometry + cạnh loop, tối ưu để **xóa drift**, cho ra `map → odom`, và là "bộ khung" để tái định vị và dựng grid | Không tự phát hiện loop (M6 làm), không lập đường đi |
| **Occupancy grid vùng đã đi** (M8) | **Bản đồ dẫn đường**: mọi ô mặc định bị chặn, chỉ ô xe đã đi qua là trống; planner (M9) tìm đường trên đó | Không dùng để định vị |

Quan hệ giữa chúng: pose graph tối ưu xong → vị trí keyframe thay đổi → **grid được dựng lại** từ quỹ đạo đã tối ưu. Grid luôn là sản phẩm phụ của pose graph, không bao giờ ngược lại.

### 4.1. Ý tưởng cốt lõi: EKF cho `odom`, pose graph cho `map`

Hệ dùng hai bộ ước lượng chạy song song, mỗi bộ làm một việc:

**EKF chạy liên tục ở tần số cao, cho frame `odom`.**
EKF trộn odometry stepper và gyro ở 30–100 Hz, cho vị trí mượt và liên tục. Bộ điều khiển cần đúng thứ này: nếu vị trí nhảy giật, xe sẽ giật theo, và với xe cân bằng có thể ngã. Đổi lại, frame `odom` trôi dần theo thời gian.

**Pose graph chỉ chứa keyframe, tối ưu khi có loop, sửa frame `map`.**
Pose graph chạy thưa hơn nhiều: thêm node khi có keyframe, tối ưu khi có loop closure. Kết quả của nó **không ghi đè lên EKF**, mà được dùng để tính phép biến đổi `map → odom`, tức là "frame `odom` đã trôi lệch bao nhiêu so với bản đồ thật".

### 4.2. Cây frame (TF)

```
map ──(pose graph)──► odom ──(EKF)──► base_link ──(cố định)──► camera_link
                                                └──(cố định)──► imu_link
```

| Phép biến đổi | Ai phát | Tần số | Đặc điểm |
|---|---|---|---|
| `odom → base_link` | EKF | 30–100 Hz | Liên tục, không bao giờ nhảy, trôi dần |
| `map → odom` | Pose graph | Khi có keyframe/loop | Có thể nhảy nhẹ khi sửa sai |
| `base_link → camera_link` | Config tĩnh | — | Extrinsic camera |

Vị trí xe trên bản đồ:

```
T_map_base = T_map_odom ⊕ T_odom_base
```

Sau mỗi lần tối ưu pose graph, với keyframe mới nhất `k`:

```
T_map_odom = T_map_kf(k)_optimized ⊕ inverse(T_odom_kf(k))
```

trong đó `T_odom_kf(k)` là pose EKF đã lưu lúc tạo keyframe `k`. `⊕` là phép ghép SE(2).

### 4.3. Luồng dữ liệu

```
┌──────────── ESP32 (firmware/) ────────┐
│ IMU → pitch → PID/LQR cân bằng        │
│ Phát xung stepper, đếm bước có dấu    │
└───────────┬───────────────▲───────────┘
   số bước, gyro_z, pitch   │ (v, ω)
            │ UART 50–100 Hz│
┌───────────▼───────────────┴──────────── Raspberry Pi 4 ───────────────────┐
│ [M1] serial_bridge (robotd: esp_link.py) ──stdin──► slam_node (C++):     │
│   └► [M2] wheel_odometry ─► [M3] ekf2d ─► odom → base_link                │
│                                   │                                       │
│ [M4] ceiling_frontend (camera) ───┼─► [M5] keyframe_db                    │
│                                   ▼            │                          │
│                     [M7] pose_graph ◄── [M6] loop_closure                 │
│                           │                                               │
│                           ├─► map → odom                                  │
│                           └─► [M8] traversed_grid ─► [M9] planner         │
│                                                          │                │
│ [M10] controller (dùng frame odom) ◄─────── waypoints ───┘                │
│ [M11] map_io (lưu/tải, tái định vị)   [M12] logger / replay / viz         │
└───────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Các module

Mỗi module mô tả: trách nhiệm, đầu vào/đầu ra, thuật toán, tiêu chí hoàn thành.

### M1. `serial_bridge`

Giao tiếp nhị phân với vi điều khiển qua UART.

*Rà soát:* **giữ giao thức đang chạy** của firmware `iot-ck` thay vì gói `0xAA55`/CRC-16 ban đầu. Nó đã thỏa mọi yêu cầu bên dưới (bước tích lũy, seq, t_mcu, tự dừng khi mất lệnh) và đã chạy ổn; đổi giao thức chỉ thêm rủi ro. Chỉ bổ sung trường `pitch`.

**Khung MCU → Pi (200 Hz):** `A5 5A <type> <len> <payload> <crc8>` (crc8 đa thức 0x07 trên type, len, payload). Gói IMU `type 0x01`, little-endian:

| Trường | Kiểu | Ý nghĩa |
|---|---|---|
| t_us | u32 | `micros()` của ESP32 (Pi quy về đồng hồ monotonic bằng bộ lọc biên dưới) |
| acc[3] | i16 | thô, 8192 LSB/g, khung MPU |
| gyro[3] | i16 | thô đã trừ offset, 32,8 LSB/(°/s), khung MPU |
| steps[2] | i32 | **Tổng tích lũy** số bước trái, phải (+ = lăn tới) |
| state | u8 | 0 hiệu chỉnh, 1 chờ dựng, 2 đang cân bằng |
| flags | u8 | bit0 có lệnh từ Pi, bit1 động cơ bật, bit2 bộ điều khiển chạm giới hạn (dễ trượt bước) |
| seq | u16 | Tăng dần, phát hiện mất gói |
| pitch | f32 | **Mới (firmware 2.1, gói 32 byte):** rad, + = chúi tới, đúng góc vòng cân bằng đang dùng. Pi vẫn nhận gói 28 byte cũ và tự tính pitch từ acc+gyro |

`gyro_z` quanh trục **thẳng đứng của thế giới** do `slam_node` tính: `ω = −sin(pitch)·gyro_x + cos(pitch)·gyro_z` (thân xe chúi tới lui liên tục nên không lấy thẳng gyro_z của MPU).

**Pi → MCU:** dòng text `VEL:<v>,<t>\n`, `v` m/s (+ tới), `t` = chênh lệch tốc độ mỗi bánh m/s (**+ = quay phải**). Đổi từ `ω` (rad/s, + ngược chiều kim đồng hồ): `t = −ω · wheel_base / 2`. Gửi lại 10 lần/giây làm nhịp tim.

**Yêu cầu:**
- Gửi số bước dạng **tích lũy**, không gửi delta, để mất gói không làm mất quãng đường.
- Gắn timestamp Pi monotonic ngay khi nhận.
- MCU tự đưa `v = ω = 0` nếu không nhận lệnh trong 0,5 s.

**Hoàn thành khi:** chạy 10 phút không lỗi CRC ngoài dự kiến, phát hiện và đếm được gói mất. (Giao thức đã có, `robot.esp_link` in số lỗi CRC và gói mất.)

### M2. `wheel_odometry`

Động học vi sai từ số bước:

```
dL = (stepsL - prevL) * meters_per_step
dR = (stepsR - prevR) * meters_per_step
d   = (dL + dR) / 2
dth = (dR - dL) / wheel_base
```

**Lưu ý xe cân bằng:** bánh quay tới lui liên tục để giữ thăng bằng. Tích phân theo bước có dấu sẽ tự triệt tiêu, không cần lọc riêng. Không dùng `dth` từ bánh làm nguồn góc chính (trượt bánh khi tăng tốc); gyro tốt hơn.

**Lưu ý stepper không encoder (rà soát):** số bước là số bước *ra lệnh*. Khi firmware báo cờ `bit2` (chạm giới hạn gia tốc/tốc độ), nhiễu quãng đường trong EKF được nhân `sat_noise_mult` lần. Trượt bước kéo dài thì chỉ cạnh thị giác (M6) sửa được.

**Đầu ra:** `(t, d, dth_wheel, v, omega_wheel)`.

**Hoàn thành khi:** ra lệnh cho xe chạy thẳng, so `d` tích lũy với thước dây trên 1,00 m, lệch dưới 1 cm. (*Rà soát:* không kiểm bằng cách đẩy tay được: không có encoder, đẩy tay thì số bước không đổi.)

### M3. `ekf2d`

**Trạng thái:** `x = [x, y, θ, b_g]` (b_g = bias gyro z).

**Dự đoán** (mỗi gói từ MCU):
```
ω   = gyro_z - b_g
θ'  = θ + ω·dt
x'  = x + d·cos(θ + ω·dt/2)
y'  = y + d·sin(θ + ω·dt/2)
b_g' = b_g  (random walk nhỏ)
```
Q phụ thuộc `|d|` (nhiễu tỉ lệ quãng đường) và `dt` (nhiễu gyro). Tự tính Jacobian F bằng Eigen.

**Cập nhật:**
- **Zero-velocity update:** khi xe *không dời chỗ* trong `zupt_window` (0,5 s) và gyro nhỏ, đo `gyro_z = b_g` để ước lượng bias. *Rà soát:* điều kiện ban đầu "hai bánh không đổi bước > 0,3 s" không bao giờ xảy ra khi xe đang cân bằng (bánh luôn đưa tới lui). Điều kiện đúng: **dịch chuyển ròng** của mỗi bánh trong cửa sổ < `zupt_disp` (5 mm), |gyro_z| trung bình < `zupt_gyro`, |pitch rate| < `zupt_pitch_rate`.
- (Tùy chọn) Tốc độ quay từ bánh `omega_wheel` làm phép đo của bias gyro (`ω_wheel − gyro_z = −b_g`), lấy trung bình 0,1 s, covariance lớn, có cổng χ² để bỏ lúc bánh trượt.
- Ngoài covariance toàn cục, EKF tích luỹ thêm **covariance tương đối kể từ keyframe gần nhất** (cùng mô hình nhiễu, đặt lại 0 ở mỗi keyframe). Đó là nhiễu của cạnh odometry trong pose graph (M7). Hiệu hai covariance toàn cục *không* phải covariance của chuyển động tương đối.

**Đầu ra:** `odom → base_link` + covariance, 30–100 Hz.

**Hoàn thành khi:** lái một vòng 5 m quay về điểm đầu, lệch dưới 10–20 cm; đứng yên 5 phút, góc trôi dưới 2°.

### M4. `ceiling_frontend`

Xử lý ảnh từ camera hướng lên trần.

1. Trích ORB, 300–500 điểm **trên ảnh gốc** (không warp cả ảnh).
2. Undistort **toạ độ keypoint** (`cv::undistortPoints`) ra tia chuẩn hoá `r = (x_n, y_n, 1)`.
3. **Bù góc pitch** bằng cách xoay tia: `r_level = R_z(φ)ᵀ · R_y(pitch) · R_b_c · r`, với `R_b_c = R_z(φ)·(độ nghiêng gắn camera)` là hướng camera trong khung thân xe (`camera.yaw`), pitch nội suy theo timestamp ảnh. Chiếu lại và nhân `f`: `q = f · (r_x/r_z, r_y/r_z)`, toạ độ **đã lấy tâm ảnh làm gốc**. Nếu |pitch| > ngưỡng (8°) hoặc |pitch rate| quá lớn (ảnh nhoè), bỏ khung.
   *Rà soát:* về hình học tương đương warp `H = K·R·K⁻¹` nhưng rẻ hơn nhiều (chỉ vài trăm điểm thay vì 300 nghìn pixel), và công thức ban đầu thiếu phép đổi khung `R(pitch)` từ khung thân xe sang khung camera.
4. **Chính sách keyframe:** tạo keyframe khi EKF báo xe đã đi ≥ 0,30 m hoặc quay ≥ 15° so với keyframe trước. Đồng thời yêu cầu đủ điểm ORB (≥ 100), nếu không thì đợi khung sau.

**Đầu ra:** `Keyframe` (xem M5).

### M5. `keyframe_db`

```cpp
struct Keyframe {
  int id;
  double stamp;
  Pose2 T_odom_kf;              // pose EKF lúc tạo keyframe (bất biến)
  Pose2 T_map_kf;               // pose trong map (thay đổi sau mỗi lần tối ưu)
  std::vector<cv::KeyPoint> kps;
  cv::Mat desc;                 // ORB
  std::vector<Pose2> path_rel;  // quỹ đạo từ keyframe này đến keyframe sau,
                                // lưu TƯƠNG ĐỐI với keyframe này (xem M8)
};
```

`path_rel`: lấy mẫu pose EKF mỗi ~2 cm giữa hai keyframe, lưu dưới dạng `inverse(T_odom_kf) ⊕ T_odom_base`. Nhờ vậy khi `T_map_kf` thay đổi sau tối ưu, cả đoạn quỹ đạo dịch theo.

### M6. `loop_closure`

**Tìm ứng viên:**
- Đang chạy bình thường: keyframe cách vị trí hiện tại (theo `map`) dưới 1,0 m và cách về thời gian ít nhất 20 keyframe.
- **Cạnh thị giác lân cận (rà soát, bổ sung):** mỗi keyframe mới cũng được so với `neighbor_edges` (3) keyframe ngay trước. Ảnh trần chồng lấn rất nhiều (0,3 m so với vùng nhìn ~2 m), nên đây là odometry thị giác gần như miễn phí. Nó là thứ duy nhất phát hiện và sửa được **bánh kẹt / trượt bước** (stepper không encoder). Cổng kiểm tra giống cạnh loop, cùng bọc Huber.
- Tái định vị khi khởi động: so với toàn bộ keyframe (brute force nếu < 500; DBoW3 nếu lớn hơn). *Rà soát:* một khung khớp chưa đủ, phải có `reloc_confirm` (2) lần khớp liên tiếp cho cùng `T_map_odom` (lệch < 0,2 m / 10°) mới nhận.

**Kiểm tra hình học:**
1. `BFMatcher(NORM_HAMMING)` + ratio test 0,7.
2. `cv::estimateAffinePartial2D(..., cv::RANSAC, 3.0 px)`.
3. Chấp nhận khi **tất cả** đúng:
   - số inlier ≥ 30;
   - hệ số scale trong [0,95; 1,05] (trần phẳng, camera cùng chiều cao);
   - T đo được không lệch quá xa dự đoán theo pose graph hiện tại (ví dụ < 0,5 m và < 20°).
4. Đổi sang chuyển động camera (*rà soát: công thức chính xác, tránh sai dấu*). Với toạ độ `q` đã lấy tâm ảnh làm gốc (bước 3 của M4), mọi điểm trên trần thoả `q = (f/H)·R(−ψ)·(P − C)`, ψ là hướng camera, C là vị trí camera, H là chiều cao trần so với camera. `estimateAffinePartial2D` trả về `q_B = s·R(α)·q_A + t`. Từ đó:
   - `Δψ = ψ_B − ψ_A = −α`;
   - vị trí camera B trong khung camera A: `d = −(H/f) · R(−α) · t`.
   Phải lấy **tâm ảnh làm gốc** trước khi ước lượng. Nếu dùng toạ độ pixel gốc (gốc ở góc trái trên), phép quay quanh góc ảnh lẫn vào tịnh tiến và cho kết quả sai.
5. Đổi sang thân xe: `T_bA_bB = T_b_c(pitch_A) ⊕ T_cA_cB ⊕ T_b_c(pitch_B)⁻¹`, với `T_b_c(p) = (off_x + h_cam·sin p, off_y, φ)`: camera lệch tâm trục bánh, và chúi theo thân xe. Vì trục quang hướng lên và hệ trục camera theo quy tắc tay phải, ảnh là phép đồng dạng **không lật** so với mặt phẳng sàn. "Lật" chỉ là cảm giác khi người nhìn ảnh trần. Vẫn **phải kiểm tra dấu bằng thực nghiệm**: ra lệnh xe tiến 0,5 m (lệnh web `calib_ref` rồi `calib_measure`), T phải ra ≈ +0,5 m theo x của robot.
6. **Chiều cao trần tự ước lượng (bổ sung):** với các cặp keyframe liền nhau khớp được, `H ≈ f · |dịch chuyển camera theo odometry| / |t|`. Lấy trung vị. Đặt `ceiling_height: 0` để dùng giá trị ước lượng. Cạnh thị giác được lưu ở dạng pixel `(α, t, …)` và đổi ra mét lúc tối ưu, nên H thay đổi thì mọi cạnh tự cập nhật.

**Đầu ra:** `LoopConstraint {kf_a, kf_b, α, t_px, inliers, loại (lân cận/loop/tái định vị)}`, đổi ra `T_a_b` + covariance (`loop_sigma_xy`, `loop_sigma_th`, nhân `sqrt(min_inliers/inliers)`) khi dựng đồ thị.

### M7. `pose_graph`

Dùng **Ceres Solver** (rà soát: thay GTSAM, xem 2.1). Cách làm theo ví dụ `pose_graph_2d` của Ceres.

- **Node:** `(x, y, θ)` cho mỗi keyframe, θ dùng manifold góc (luôn wrap về (−π, π]).
- **Cạnh odometry:** sai số between `[R(θa)ᵀ(p_b − p_a) − p_ab ; wrap(θb − θa − θab)]`, giá trị = `inverse(T_odom_kf(i)) ⊕ T_odom_kf(i+1)`, nhiễu = covariance tương đối của EKF (M3).
- **Cạnh thị giác (lân cận + loop):** cùng sai số, **bọc `ceres::HuberLoss`** (k ≈ 1,345 trên sai số đã chuẩn hoá).
- **Prior:** keyframe đầu tiên cố định ở gốc map (khi tạo bản đồ mới). Mở rộng bản đồ cũ: keyframe mới đầu tiên nối với bản đồ cũ bằng cạnh tái định vị.
- **Tối ưu:** Levenberg-Marquardt (sparse Cholesky / `SPARSE_NORMAL_CHOLESKY`) sau mỗi keyframe có cạnh thị giác mới. Vài trăm node 2D chỉ mất vài ms đến vài chục ms.
- **Sau tối ưu:** cập nhật `T_map_kf` mọi keyframe, tính lại `T_map_odom` (mục 4.2), báo M8 dựng lại grid.
- Chạy ở luồng riêng.

**Hoàn thành khi:** dữ liệu replay một vòng có loop, quỹ đạo sau tối ưu khép kín, lệch điểm đầu–cuối < 5 cm.

### M8. `traversed_grid` (bản đồ vùng đã đi)

**Giả định quan trọng: mọi ô mặc định là bị chặn.** Ô chỉ được coi là đi được khi xe đã thực sự đi qua.

**Thông số:** độ phân giải 0,05 m; bề rộng xe 0,20 m.

**Hai lớp, phải tách bạch:**

| Lớp | Đánh dấu "trống" | Dùng cho |
|---|---|---|
| `footprint_layer` | Mọi ô nằm dưới thân xe (hình tròn bán kính 0,10 m) tại mỗi pose trên quỹ đạo | Hiển thị, debug |
| `center_layer` | Ô mà **tâm xe** đã đi qua, nong thêm dung sai ngang `center_tolerance` (mặc định 0,05 m) | **Planner** |

**Vì sao cần `center_layer`:** dải `footprint` chỉ rộng đúng bằng thân xe (20 cm). Nếu đưa nó vào planner kiểu Nav2 có inflation theo bán kính robot (0,10 m), hành lang co lại còn 0 cm và không tìm được đường. Planner phải lập đường cho **tâm xe** trên `center_layer`, **không inflation thêm**: thân xe tự động nằm trong vùng đã từng đi qua.

**Dựng grid:**
1. Với mỗi keyframe, lấy `T_map_kf` (đã tối ưu) ghép với từng pose trong `path_rel` để ra quỹ đạo trong `map`.
2. Vẽ các đoạn nối liên tiếp lên hai lớp.
3. Dựng lại toàn bộ sau mỗi lần pose graph tối ưu (vài trăm keyframe: vài chục ms, chấp nhận được).

**Chỉ ghép cạnh "gần nhau" khi thật sự gần:** hai đoạn quỹ đạo cách nhau dưới `center_tolerance` thì ô giữa chúng tự nhiên được nối; không tự nối các đoạn xa hơn.

**Xuất:** PNG + YAML theo định dạng của ROS `map_server` (trắng = trống, đen = chặn, YAML chứa resolution và origin), để xem được bằng bất kỳ trình xem ảnh nào và nạp được vào ROS sau này nếu cần.

### M9. `planner`

- A* 8 hướng trên `center_layer`, chi phí = độ dài + phạt nhỏ khi gần mép (dùng distance transform) để xe bám đường tâm.
- Làm thưa đường đi thành waypoint cách nhau ~0,15 m.
- **Địa điểm có tên:** `places.yaml`
  ```yaml
  bep:  {x: 2.10, y: 3.40, theta: 1.57}
  sac:  {x: 0.00, y: 0.00, theta: 0.00}
  ```
  Lệnh lưu địa điểm hiện tại: `save_place <tên>`. Đích phải nằm trong `center_layer`; nếu không, chọn ô trống gần nhất và báo.
- Waypoint được tính trong `map`, rồi **đổi sang `odom`** bằng `inverse(T_map_odom)` trước khi gửi cho controller. Khi `T_map_odom` thay đổi, đổi lại các waypoint còn lại.
- Điểm xuất phát (vị trí xe) cũng có thể nằm ngoài `center_layer` sau một lần tối ưu: chọn ô trống gần nhất (≤ `max_snap`).
- **Nhiệm vụ (bổ sung):** lệnh `mission x1 y1 x2 y2 …` lập A* lần lượt từng chặng (chặng sau bắt đầu từ đích chặng trước), nối thành một chuỗi waypoint. Điểm cuối mỗi chặng là điểm **bắt buộc chạm** (trong `goal_tol_xy`): controller không được nhìn trước vượt qua nó, chạm rồi mới đi tiếp. Vận tốc và tốc độ quay lấy cố định từ cấu hình controller, chỉ chậm lại ở điểm cuối.

### M10. `controller`

- Pure pursuit trong frame `odom`, lookahead 0,25–0,35 m. Điểm lookahead đo **theo chiều dài đường** từ waypoint hiện tại, chỉ số waypoint chỉ tăng lần lượt (đã tới gần hoặc đã vượt qua theo hướng đoạn kế), để bám được đường có quay đầu / đi lại đoạn cũ mà không cắt tắt.
- **Xoay tại chỗ trước (rà soát):** lệch hướng tới điểm lookahead > `rotate_in_place` (50°) thì `v = 0`, chỉ xoay. Xe cân bằng vừa chạy vừa bẻ lái gắt dễ mất thăng bằng, còn ảnh trần nhoè khi vừa xoay vừa chạy.
- Giới hạn: `v_max` 0,15 m/s (mặc định, xe cân bằng nên chậm), `ω_max` 1,0 rad/s, giới hạn gia tốc để không làm xe mất thăng bằng. Firmware còn chặn thêm (`CMD_MAX_SPEED` 0,35 m/s, `CMD_MAX_TURN` 0,20 m/s, slew 0,4 m/s²).
- Đầu ra `(v, ω)` đổi sang `VEL:v,t` với `t = −ω·wheel_base/2` (M1).
- Không tiến triển dọc đường đi trong `stuck_time` (8 s) thì dừng và báo (vướng vật / bánh kẹt).
- Tới đích: dung sai vị trí 0,10 m, rồi xoay tại chỗ về `theta` đích (dung sai 10°).
- **An toàn (bắt buộc):**
  - Dừng ngay nếu |pitch| hoặc tốc độ thay đổi pitch vượt ngưỡng bất thường (dấu hiệu va chạm).
  - Dừng nếu mất `T_map_odom` hoặc EKF báo covariance quá lớn.
  - Dừng nếu không nhận dữ liệu MCU > 0,2 s.

### M11. `map_io` và tái định vị

**Lưu bản đồ** (thư mục `maps/<tên>/`):
- `graph.g2o` hoặc định dạng riêng: node + cạnh.
- `keyframes.bin`: id, `T_map_kf`, keypoints, descriptor, `path_rel`.
- `grid.png` + `grid.yaml`.
- `places.yaml`, `config_snapshot.yaml` (thông số camera, wheel base, chiều cao trần lúc tạo bản đồ).

**Khi khởi động với bản đồ có sẵn:**
1. Chụp vài khung, so với mọi keyframe (M6, chế độ tái định vị).
2. Ứng viên tốt nhất qua kiểm tra hình học → `T_map_base` ban đầu.
3. Đặt `T_map_odom = T_map_base ⊕ inverse(T_odom_base)`.
4. Chưa tái định vị được thì **không cho goto**; báo người dùng xoay xe hoặc đẩy tới chỗ quen.

**Hai chế độ:**
- `mapping`: thêm keyframe, thêm cạnh, tối ưu, mở rộng grid. Trên bản đồ đã tải: tái định vị trước. Lúc tái định vị thành công, tạo một keyframe tại khung đó, nối với keyframe cũ bằng cạnh thị giác, rồi chạy tiếp như bản đồ mới.
- `localization`: chỉ đọc bản đồ; vẫn dùng loop closure với keyframe cũ để sửa `T_map_odom`, nhưng không thêm keyframe mới. *Rà soát (cụ thể hoá):* `T_map_odom` được giữ bằng một bộ lọc Kalman 3 trạng thái. Nhiễu quá trình tăng theo quãng đường đi (odometry trôi). Mỗi lần khớp khung hiện tại với keyframe cũ `j` cho một phép đo `T_map_odom = T_map_kf(j) ⊕ T_kf(j)_base ⊕ T_odom_base⁻¹`. Phép đo có cổng Mahalanobis χ² (3 bậc tự do) và cổng tuyệt đối 0,5 m / 20°. Độ bất định vượt `max_map_sigma` thì coi là mất định vị: không cho goto, đang goto thì dừng.

### M12. Công cụ

- **Logger:** ghi ảnh (PNG hoặc nén), gói MCU, timestamp vào một thư mục; định dạng đọc được bằng Python.
- **Replay:** chạy toàn bộ pipeline trên dữ liệu ghi, nhanh hơn thời gian thực, kết quả xác định.
- **Viz:** Pi phát trạng thái (pose, keyframe, cạnh loop, grid) qua UDP hoặc WebSocket dạng JSON với tần số thấp (~5 Hz); script Python trên laptop (matplotlib hoặc một trang web nhỏ) vẽ trực tiếp. Không chạy giao diện đồ họa trên Pi. Script cũng đọc được thư mục log để vẽ offline.
- **Calib:** script hiệu chuẩn camera bằng bàn cờ; script kiểm tra dấu trục camera → robot.

---

## 6. File cấu hình mẫu

Bản đầy đủ đang dùng: `pi/config/slam.yaml`.

```yaml
robot:
  wheel_diameter: 0.065        # m
  wheel_base: 0.170            # m, TBD - đo thực tế
  steps_per_rev: 200
  microsteps: 16
  body_width: 0.20             # m
serial:
  port: /dev/serial0           # hoặc /dev/ttyUSB0 nếu dùng USB-UART
  baud: 921600
  cmd_timeout: 0.5             # s
ekf:
  rate_hz: 50
  sigma_d_per_m: 0.02          # nhiễu quãng đường / mét
  sigma_gyro: 0.01             # rad/s
  zupt_still_time: 0.3         # s
camera:
  type: libcamera              # libcamera (Pi Camera CSI) | v4l2 (USB)
  width: 640
  height: 480
  fps: 15
  calib_file: config/camera.yaml
  ceiling_height_from_cam: 2.30 # m, TBD
  max_pitch_for_frame: 0.14    # rad (~8°)
keyframe:
  min_dist: 0.30               # m
  min_angle: 0.26              # rad (~15°)
  orb_features: 500
loop:
  search_radius: 1.0           # m
  min_kf_gap: 20
  ratio_test: 0.7
  ransac_px: 3.0
  min_inliers: 30
  scale_range: [0.95, 1.05]
  huber_k: 1.345
grid:
  resolution: 0.05             # m
  center_tolerance: 0.05       # m
controller:
  v_max: 0.15                  # m/s
  w_max: 1.0                   # rad/s
  lookahead: 0.30              # m
  goal_tol_xy: 0.10            # m
  goal_tol_theta: 0.17         # rad
```

---

## 7. Cấu trúc thư mục (đã triển khai trong repo này)

```
pi/slam/                     # lõi SLAM C++17, build trên Pi OS và laptop
├── CMakeLists.txt
├── include/slam/            # pose2, config, wheel_odometry, ekf2d, frontend, keyframe,
│                            # matcher, pose_graph, traversed_grid, planner, controller,
│                            # map_io, system, protocol
├── src/
├── apps/slam_node.cpp       # chạy thật (stdin/stdout với robotd) + replay log
├── apps/slam_sim.cpp        # sinh log tổng hợp: ảnh trần + odometry trôi (test tích hợp)
└── test/                    # GoogleTest
pi/robot/slam_bridge.py      # robotd <-> slam_node
pi/tools/                    # đọc log, vẽ quỹ đạo (Python, chạy trên laptop)
pi/config/slam.yaml          # cấu hình (mục 6)
pi/web/index.html            # giao diện: grid, keyframe, loop, địa điểm, goto
firmware/                    # ESP32 (giao thức M1)
maps/<tên>/                  # bản đồ đã lưu
```

**Thư viện:** Eigen3, OpenCV 4, Ceres 2.x, yaml-cpp, GoogleTest. Build bằng CMake (`pi/scripts/build_slam.sh`).

---

## 8. Lộ trình và tiêu chí nghiệm thu

| Giai đoạn | Việc | Nghiệm thu |
|---|---|---|
| 1 | M1 + giao thức firmware | Xe đứng vững, điều khiển (v, ω) từ Pi, không lỗi CRC 10 phút |
| 2 | M2 + M3 + logger | Vòng 5 m quay về lệch < 20 cm; đứng yên 5 phút trôi < 2° |
| 3 | M4 + M5 + M12 replay | Có dataset ảnh trần + odometry; replay xác định |
| 4 | M7 chỉ với cạnh odometry | Pose graph cho kết quả giống EKF; xuất `map → odom` |
| 5 | M6 + cạnh loop + robust | Replay có loop: khép vòng < 5 cm; không loop sai trên dataset thử |
| 6 | M8 + M11 lưu/tải + tái định vị | Bật máy ở 5 vị trí khác nhau, tái định vị đúng < 15 cm |
| 7 | M9 + M10 | `goto` tới 3 địa điểm, sai số < 15 cm, xe không ra khỏi vùng đã đi |
| 8 | Tối ưu trên Pi | Toàn hệ chạy thời gian thực, CPU < 70% |

Làm tuần tự. Không sang giai đoạn sau khi giai đoạn trước chưa đạt.

---

## 9. Kiểm thử bắt buộc

- **Unit test:** phép ghép SE(2) và nghịch đảo; wrap góc; động học vi sai; Jacobian EKF (so với sai phân hữu hạn); đổi pixel → mét; vẽ đoạn lên grid; A* trên grid nhỏ.
- **Test tích hợp trên dữ liệu tổng hợp:** sinh quỹ đạo vòng kín + odometry có drift + loop giả → pose graph phải khép vòng.
- **Test chống loop sai:** đưa một loop sai 1 m vào giữa → kết quả không bị kéo lệch quá 5 cm (nhờ Huber).
- **Test grid:** quỹ đạo thẳng 1 m → `center_layer` là dải rộng ≈ 2 × `center_tolerance` + 1 ô; mọi ô khác bị chặn.

---

## 10. Lỗi thường gặp cần tránh

- Lệch timestamp giữa ảnh và odometry → ghép khung sai.
- Sai dấu trục khi đổi từ ảnh trần sang robot → loop closure kéo sai hướng.
- Dùng `dth` từ bánh xe làm góc chính → trượt bánh làm sai hướng.
- Inflation thêm bán kính robot lên `center_layer` → không tìm được đường (xem M8).
- Gửi waypoint trong `map` thẳng cho controller → xe giật khi loop closure. Controller chỉ nhận waypoint trong `odom`.
- Chạy tối ưu đồ thị trong luồng EKF/controller → trễ điều khiển.

---

## 11. Giới hạn đã biết

- **Không phát hiện vật cản mới** (ghế bị kéo ra, người, thú cưng). Giảm rủi ro bằng tốc độ thấp, bám đường tâm, và dừng khi pitch bất thường. Khuyến nghị sau này thêm công tắc va chạm hoặc cảm biến ToF.
- Trần trắng trơn ít đặc trưng → ít loop closure. Giải pháp: dán vài hình hoặc AprilTag lên trần (có thể thêm làm `PriorFactor`).
- Chiều cao trần không đều (đèn thả, dầm) gây sai số đổi pixel → mét vài phần trăm; chấp nhận được với mục tiêu 10–30 cm.
- Xe chỉ đến được nơi đã đi qua khi tạo bản đồ. Khi tạo bản đồ, hãy đi chính giữa lối đi và đi qua lại mỗi khu vực vài lần.

---

## 12. Câu hỏi mở (cần người dùng xác nhận)

- [ ] Khoảng cách hai bánh (wheel base) chính xác? (mặc định 0,17 m, `pi/config/slam.yaml`)
- [x] Số bước/vòng và microstepping: 200 × 16 = 3200.
- [x] Vi điều khiển và IMU: ESP32 + MPU-6050.
- [x] Camera: OV5647 Pi Camera CSI qua Picamera2, ~54°×41°, rolling shutter. **Cần gắn lại nhìn lên trần**, ghi hướng gắn vào `camera.yaw`.
- [ ] Chiều cao từ camera đến trần? (hoặc để tự ước lượng: `ceiling_height: 0`)
- [x] MCU nối Pi qua UART GPIO `/dev/serial0` 460800 baud.
- [ ] Nội tham số camera thật (chạy `python3 -m robot.calibrate` với bàn cờ). Hiện là ước lượng fx = fy = 635 px.

---

## 13. Tóm tắt rà soát (đối chiếu với xe thật)

| # | Vấn đề trong bản thiết kế gốc | Điều chỉnh |
|---|---|---|
| 1 | Ghi Bookworm, xe chạy Trixie | Sửa mục 2, 2.1 |
| 2 | GTSAM build 1–2 giờ trên thẻ nhớ giả + nguồn sụt áp | Ceres (có trong apt) |
| 3 | "Toàn hệ C++" bắt viết lại camera libcamera + web | Lõi SLAM C++ (`slam_node`), I/O giữ ở `robotd` Python |
| 4 | Gói UART mới `0xAA55` trong khi giao thức hiện có đã đủ | Giữ giao thức, thêm `pitch` (gói 32 byte, tương thích ngược) |
| 5 | Kiểm odometry bằng cách đẩy tay: stepper không encoder nên không đếm | Kiểm bằng lệnh chạy + thước |
| 6 | ZUPT "bánh không đổi bước 0,3 s" không bao giờ xảy ra khi cân bằng | Dùng dịch chuyển ròng trong cửa sổ |
| 7 | gyro_z của MPU ≠ tốc độ quay quanh trục đứng khi thân chúi | Chiếu theo pitch |
| 8 | Warp cả ảnh theo pitch: tốn CPU, thiếu đổi khung | Xoay tia của keypoint |
| 9 | Đổi pixel→mét không lấy tâm ảnh làm gốc, dấu trục chưa rõ | Công thức chính xác (M6 bước 4–5), có cánh tay đòn camera |
| 10 | Bánh kẹt / trượt bước (đã gặp thật) chỉ sửa được nhờ loop | Thêm cạnh thị giác lân cận |
| 11 | Chiều cao trần phải đo tay | Tự ước lượng từ odometry |
| 12 | "Nhiễu cạnh odometry lấy từ EKF" chưa nói cách | Covariance tương đối từ keyframe |
| 13 | Tái định vị 1 khung dễ nhận sai | Cần 2 lần khớp nhất quán |
| 14 | Chế độ localization sửa `T_map_odom` thế nào chưa rõ | Bộ lọc Kalman 3 trạng thái có cổng |
| 15 | Pure pursuit bẻ lái gắt với xe cân bằng | Xoay tại chỗ khi lệch > 50° |
| 16 | Không nói đổi `(v, ω)` sang lệnh firmware | `t = −ω·b/2` |
| 17 | Không có keyframe khi trần trơn → không có grid | Keyframe "mù" (chỉ odometry) sau `blind_kf_dist` để grid vẫn liền |