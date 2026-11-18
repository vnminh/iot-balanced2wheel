#!/bin/bash
# Cài thư viện hệ thống cần cho OpenVINS (cần sudo, chạy 1 lần):
#   sudo bash ~/iot-ck/pi/scripts/install_deps.sh
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then echo "Chạy bằng: sudo bash $0" >&2; exit 1; fi
apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config \
  libeigen3-dev libopencv-dev libceres-dev \
  libboost-system-dev libboost-filesystem-dev libboost-thread-dev libboost-date-time-dev \
  python3-venv python3-yaml python3-picamera2 python3-serial python3-opencv
# Cho phép dịch vụ robotd (systemd --user) chạy ngay khi Pi khởi động, không cần đăng nhập
loginctl enable-linger "${SUDO_USER:-iot09}" || true
echo "XONG. Tiếp theo (không cần sudo):  bash ~/iot-ck/pi/scripts/build_vio.sh"
