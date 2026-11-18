#pragma once
// =============================================================================
//  XE CÂN BẰNG 2 BÁNH CHO VI-SLAM
//  ESP32 DevKit 38 chân + MPU-6050 + 2x A4988 + 2x Step 42 (bánh 65 mm)
//
//  ESP32 lo 3 việc:
//   1. Giữ thăng bằng (PID tầng 400 Hz, giống bản cũ).
//   2. Nhận lệnh vận tốc từ Raspberry Pi qua UART2.
//   3. Gửi sang Pi luồng IMU + số bước 2 bánh 200 Hz (cho OpenVINS + odometry).
//
//  Sửa thông số trong file này cho khớp với xe.
// =============================================================================

// ----------------------------- CHÂN KẾT NỐI ---------------------------------
#define PIN_I2C_SDA        21
#define PIN_I2C_SCL        22

#define PIN_L_STEP         26      // A4988 bánh TRÁI
#define PIN_L_DIR          25
#define PIN_R_STEP         19      // A4988 bánh PHẢI
#define PIN_R_DIR          18
#define PIN_MOTOR_EN       27      // ENABLE chung, có trở kéo lên 10k (LOW = chạy)
#define PIN_LED            2

// UART nối Raspberry Pi. Chọn theo cách đã đấu dây:
//   LINK_UART 0 : Pi GPIO14 (TXD, chân 8) -> ESP32 RX0 (GPIO3)
//                 Pi GPIO15 (RXD, chân 10) <- ESP32 TX0 (GPIO1), GND chung
//                 (dây hiện tại). Dùng chung với cổng USB nên KHI NẠP CODE phải
//                 thả chân TX của Pi: chạy pi/scripts/esp32_flash_mode.sh release
//                 trên Pi (hoặc rút dây chân 8), nạp xong chạy ... restore.
//   LINK_UART 2 : Pi chân 8 -> ESP32 GPIO16, Pi chân 10 <- ESP32 GPIO17.
//                 Khuyên dùng: cổng USB rảnh để nạp/debug.
#define LINK_UART          0
#if LINK_UART == 0
#define PIN_LINK_RX        3
#define PIN_LINK_TX        1
#else
#define PIN_LINK_RX        16
#define PIN_LINK_TX        17
#endif
#define LINK_BAUD          460800  // phải khớp pi/config/robot.yaml (serial.baud)

// ------------------------------- CƠ KHÍ -------------------------------------
#define MOTOR_FULL_STEPS   200      // 1.8 độ/bước
#define MICROSTEP          16       // MS1 = MS2 = MS3 = 3V3
#define WHEEL_DIAMETER_M   0.065f   // bánh 65 mm

// Kiểm tra bằng env test_driver (lệnh "auto"): cả 2 bánh phải làm xe chạy TỚI.
#define LEFT_MOTOR_INVERT  true
#define RIGHT_MOTOR_INVERT false

// ------------------------------- MPU-6050 -----------------------------------
#define MPU_ADDR           0x68
// MPU nằm ngang, mũi tên X chỉ về TRƯỚC xe, mặt linh kiện quay LÊN (Z lên)
// => Y chỉ sang TRÁI. Khung này cũng là khung IMU mà OpenVINS dùng.
#define IMU_FWD_AXIS       0
#define IMU_UP_AXIS        2
#define IMU_GYRO_AXIS      1
#define IMU_ACC_SIGN       1
#define IMU_GYRO_SIGN      1

// ---------------------------- VÒNG ĐIỀU KHIỂN -------------------------------
#define LOOP_HZ            400
#define COMP_TAU           0.5f
#define COMP_ALPHA         (COMP_TAU / (COMP_TAU + 1.0f / LOOP_HZ))

#define FALL_ANGLE_DEG     45.0f
#define START_ANGLE_DEG    5.0f
#define START_HOLD_MS      500

// Vref đã vặn tối đa, động cơ hết rung -> trả về giới hạn thiết kế ban đầu.
// Nếu thấy trượt bước khi bị đẩy mạnh: giảm MAX_ACCEL_MPS2 mỗi lần 0.5.
// Vref tối đa làm A4988 rất nóng: BẮT BUỘC có tản nhiệt (tốt nhất thêm quạt).
#define MAX_SPEED_MPS      0.90f
#define MAX_ACCEL_MPS2     8.0f
#define MAX_TILT_TARGET    10.0f
#define SPEED_FILTER_TAU   0.10f
#define ACC_COMPENSATION   1

#define ANGLE_KP           30.0f
#define ANGLE_KI           0.0f
#define ANGLE_KD           2.0f
#define SPEED_KP           6.0f
#define SPEED_KI           3.0f
#define ANGLE_OFFSET_DEG   0.0f

// ------------------------- LỆNH CHẠY TỪ PI ----------------------------------
// Giới hạn trên của lệnh VEL:<v>,<t> (Pi/web có thể gửi giá trị nhỏ hơn).
// SLAM bằng camera cần chạy CHẬM: ảnh nhòe và rolling shutter làm mất bám.
#define CMD_MAX_SPEED      0.35f    // m/s tiến/lùi
#define CMD_MAX_TURN       0.20f    // m/s chênh lệch tốc độ mỗi bánh khi quay
// Lệnh chữ (tương thích uart_link.py cũ / điều khiển giọng nói)
#define CMD_DRIVE_SPEED    0.15f
#define CMD_TURN_SPEED     0.05f
#define CMD_ROTATE_SPEED   0.06f
// Đổi lệnh mượt để camera không bị giật (m/s^2)
#define CMD_SPEED_SLEW     0.4f
#define CMD_TURN_SLEW      0.4f
// Pi gửi lệnh 10 lần/giây. Quá thời gian này không nghe gì -> tự dừng.
#define CMD_TIMEOUT_MS     500

// ------------------------- LUỒNG DỮ LIỆU SANG PI ----------------------------
#define TELEM_DIV          2        // gửi 1 mẫu / 2 vòng  ->  400/2 = 200 Hz

// ------------------------- PHÁT XUNG STEP (timer) ---------------------------
#define STEP_ISR_HZ        40000
#define MAX_STEP_RATE      16000

#define DEBUG_LOG          0

// ------------------------------- SUY RA -------------------------------------
#define STEPS_PER_REV      (MOTOR_FULL_STEPS * MICROSTEP)
#define STEPS_PER_METER    (STEPS_PER_REV / (3.14159265f * WHEEL_DIAMETER_M))
#define FW_VERSION         "iot-ck-2.0"
