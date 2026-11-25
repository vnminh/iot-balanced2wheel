#!/bin/bash
# Cài TẤT CẢ trên Raspberry Pi (chạy được nhiều lần, bước nào xong rồi sẽ nhanh):
#   bash ~/iot-ck/pi/scripts/install.sh              đầy đủ
#   bash ~/iot-ck/pi/scripts/install.sh --skip-apt   bỏ qua apt (đã cài gói hệ thống)
#   bash ~/iot-ck/pi/scripts/install.sh --skip-slam  không build slam_node (web + điều khiển tay vẫn chạy)
#
#   1. apt: cmake, eigen, opencv, ceres, yaml-cpp, picamera2... (hỏi mật khẩu sudo)
#   2. bật linger: robotd tự chạy khi Pi bật, không cần đăng nhập
#   3. venv Python + aiohttp, pyserial, PyYAML
#   4. build slam_node (pi/slam, ~3-5 phút)
#   5. cài + khởi động dịch vụ robotd (systemd --user)
set -euo pipefail
cd "$(dirname "$0")"
SKIP_APT=0
SKIP_SLAM=0
for a in "$@"; do
  case "$a" in
    --skip-apt) SKIP_APT=1 ;;
    --skip-slam) SKIP_SLAM=1 ;;
    *) echo "Tham số lạ: $a"; exit 1 ;;
  esac
done
[ "$(id -u)" -eq 0 ] && { echo "Chạy bằng user thường (iot09), KHÔNG dùng sudo. Script tự hỏi sudo khi cần." >&2; exit 1; }

step() { echo; echo "==================== $* ===================="; }

if [ "$SKIP_APT" = 0 ]; then
  step "1/5 Gói hệ thống (sudo)"
  sudo bash ./install_deps.sh
else
  step "1/5 Bỏ qua apt"
fi

step "2/5 Linger (robotd chạy khi bật Pi)"
if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != yes ]; then
  sudo loginctl enable-linger "$USER"
fi
loginctl show-user "$USER" -p Linger

step "3/5 Python venv"
bash ./setup_python.sh

if [ "$SKIP_SLAM" = 0 ]; then
  step "4/5 Build slam_node"
  bash ./build_slam.sh
else
  step "4/5 Bỏ qua build slam_node (web vẫn chạy, chưa có SLAM)"
fi

step "5/5 Dịch vụ robotd"
bash ./install_service.sh

echo
echo "XONG. Web: http://$(hostname).local:8080   Log: journalctl --user -u robotd -f"
