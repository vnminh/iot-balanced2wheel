#!/bin/bash
# Cài thư viện hệ thống cho robotd + slam_node (cần sudo, chạy 1 lần):
#   sudo bash ~/iot-ck/pi/scripts/install_deps.sh
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then echo "Chạy bằng: sudo bash $0" >&2; exit 1; fi
apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config \
  libeigen3-dev libopencv-dev libceres-dev libyaml-cpp-dev libgtest-dev \
  python3-venv python3-yaml python3-picamera2 python3-serial python3-opencv python3-numpy
# UART GPIO14/15 cho ESP32: dùng PL011 (ổn định ở 460800 baud), tắt Bluetooth (chiếm PL011) và serial console
CFG=/boot/firmware/config.txt
CMD=/boot/firmware/cmdline.txt
REBOOT=0
grep -q '^enable_uart=1' "$CFG" || { printf '\n[all]\nenable_uart=1\n' >> "$CFG"; REBOOT=1; }
grep -q '^dtoverlay=disable-bt' "$CFG" || { echo 'dtoverlay=disable-bt' >> "$CFG"; REBOOT=1; }
if grep -q 'console=serial0' "$CMD"; then
  sed -i 's/console=serial0,[0-9]* *//' "$CMD"; REBOOT=1
fi
systemctl disable --now hciuart 2>/dev/null || true
systemctl disable --now serial-getty@ttyAMA0.service serial-getty@serial0.service 2>/dev/null || true
[ "$REBOOT" = 1 ] && echo "!! Đã bật UART: cần KHỞI ĐỘNG LẠI Pi (sudo reboot) để có /dev/serial0"

# Cho phép dịch vụ robotd (systemd --user) chạy ngay khi Pi khởi động, không cần đăng nhập
loginctl enable-linger "${SUDO_USER:-iot09}" || true
echo "XONG. Tiếp theo (không cần sudo):  bash ~/iot-ck/pi/scripts/build_slam.sh"
