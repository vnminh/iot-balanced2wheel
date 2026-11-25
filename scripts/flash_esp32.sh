#!/bin/bash
# Nạp firmware ESP32 từ LAPTOP qua cổng USB (CP2102).
#
#   ./scripts/flash_esp32.sh                 nạp chương trình chính (env main)
#   ./scripts/flash_esp32.sh test_mpu        nạp env khác (test_driver, test_mpu)
#   PORT=/dev/ttyUSB1 ./scripts/flash_esp32.sh
#   PI=iot09@192.168.1.50 ./scripts/flash_esp32.sh
#   NO_PI=1 ./scripts/flash_esp32.sh         bỏ qua bước SSH (đã rút dây TX của Pi)
#
# Dây Pi đang nối vào UART0 của ESP32 (chung với USB). Chân TX của Pi giữ đường RX0
# nên máy tính không nạp được. Script tự làm:
#   1. build trước (Pi chỉ phải nhả dây trong lúc nạp)
#   2. SSH vào Pi: dừng robotd, thả GPIO14 (TXD) về input
#   3. pio upload
#   4. trả GPIO14 về TXD, kiểm tra ESP32 gửi IMU, bật lại robotd
# Bước 4 luôn chạy kể cả khi nạp lỗi / Ctrl+C.
set -euo pipefail

ENV="${1:-main}"
PI="${PI:-iot09@raspberrypi4.local}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
FW="$ROOT/firmware"

PIO="$(command -v pio || true)"
[ -z "$PIO" ] && [ -x "$HOME/.platformio/penv/bin/pio" ] && PIO="$HOME/.platformio/penv/bin/pio"
[ -z "$PIO" ] && { echo "Không tìm thấy PlatformIO (pio). Cài: pip install platformio" >&2; exit 1; }

if [ -z "${PORT:-}" ]; then
  PORT="$(ls /dev/serial/by-id/*CP210* /dev/serial/by-id/*CH34* 2>/dev/null | head -1 || true)"
  [ -z "$PORT" ] && PORT="$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1 || true)"
fi
[ -z "$PORT" ] && { echo "Không thấy ESP32. Cắm USB rồi thử lại (hoặc đặt PORT=/dev/ttyUSBx)." >&2; exit 1; }
[ -w "$PORT" ] || { echo "Không có quyền ghi $PORT. Chạy: sudo usermod -aG dialout $USER  (rồi đăng nhập lại)" >&2; exit 1; }

echo "==> [1/4] Build env '$ENV'"
"$PIO" run -d "$FW" -e "$ENV"

SSH=(ssh -o ConnectTimeout=8 -o BatchMode=yes "$PI")
PI_OK=0
WAS_ENABLED=0
restore_pi() {
  [ "$PI_OK" = 1 ] || return 0
  PI_OK=0
  echo "==> [4/4] Trả GPIO14 về TXD trên Pi"
  "${SSH[@]}" 'pinctrl set 14 a0 pn && pinctrl get 14' || { echo "!! Không trả được GPIO14. Trên Pi chạy: pinctrl set 14 a0 pn"; return 0; }
  if [ "$ENV" = main ]; then
    echo "    Kiểm tra luồng IMU từ ESP32 (chờ ESP32 khởi động + hiệu chỉnh gyro)..."
    sleep 3
    "${SSH[@]}" 'set -o pipefail; cd ~/iot-ck/pi && timeout 10 ../.venv/bin/python -m robot.esp_link --secs 4 | tail -4' \
      || echo "!! Chưa thấy dữ liệu IMU. Xem lại dây UART / nguồn MPU-6050."
  fi
  if [ "$ENV" != main ]; then
    # Firmware test dùng chung UART0 với Pi: robotd gửi VEL sẽ chen vào lệnh test
    echo "    Firmware test: KHÔNG bật lại robotd. Nạp lại env main xong robotd sẽ tự bật."
  elif [ "$WAS_ENABLED" = 1 ]; then
    echo "    Bật lại robotd"
    "${SSH[@]}" 'systemctl --user start robotd' || true
  else
    echo "    robotd chưa cài service, chạy tay: cd ~/iot-ck/pi && ../.venv/bin/python -m robot.main"
  fi
}
trap restore_pi EXIT
trap 'exit 130' INT TERM

if [ -z "${NO_PI:-}" ]; then
  echo "==> [2/4] Pi ($PI): dừng robotd, thả chân TX"
  if "${SSH[@]}" true 2>/dev/null; then
    PI_OK=1
    if "${SSH[@]}" 'systemctl --user is-enabled --quiet robotd 2>/dev/null'; then WAS_ENABLED=1; fi
    # [r]obot.main: để pkill không khớp chính lệnh bash này
    "${SSH[@]}" 'systemctl --user stop robotd 2>/dev/null; pkill -f "[r]obot\.main" || true; sleep 0.5; pinctrl set 14 ip pn && pinctrl get 14'
  else
    echo "!! Không SSH được vào $PI. Nếu dây TX của Pi còn nối, nạp sẽ lỗi."
    echo "   Rút dây Pi chân 8 -> ESP32 RX0 hoặc tắt Pi, rồi chạy lại với NO_PI=1."
  fi
else
  echo "==> [2/4] Bỏ qua Pi (NO_PI=1)"
fi

echo "==> [3/4] Nạp qua $PORT"
"$PIO" run -d "$FW" -e "$ENV" -t upload --upload-port "$PORT"
echo "==> Nạp xong."
