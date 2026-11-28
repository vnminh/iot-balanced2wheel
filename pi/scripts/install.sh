#!/bin/bash
# Cài TẤT CẢ trên Raspberry Pi (chạy được nhiều lần, bước nào xong rồi sẽ nhanh):
#   bash ~/iot-ck/pi/scripts/install.sh              đầy đủ
#   bash ~/iot-ck/pi/scripts/install.sh --skip-apt   bỏ qua apt (đã cài gói hệ thống)
#   bash ~/iot-ck/pi/scripts/install.sh --skip-slam  không build slam_node (web + điều khiển tay vẫn chạy)
#   bash ~/iot-ck/pi/scripts/install.sh --skip-voice không cài điều khiển giọng nói
#
#   1. apt: cmake, eigen, opencv, ceres, yaml-cpp, picamera2... (hỏi mật khẩu sudo)
#   2. bật linger: robotd tự chạy khi Pi bật, không cần đăng nhập
#   3. venv Python + aiohttp, pyserial, PyYAML
#   4. build slam_node (pi/slam, ~3-5 phút)
#   5. cài + khởi động dịch vụ robotd (systemd --user)
#   6. giọng nói: sherpa-onnx, tải mô hình Zipformer (~50 MB), dịch vụ voice (Speech-control, WebSocket
#      server :8765 nhận âm thanh từ ESP32 #2)
#   Lần đầu: sudo reboot sau khi xong (bật UART cho ESP32)
set -euo pipefail
cd "$(dirname "$0")"
SKIP_APT=0
SKIP_SLAM=0
SKIP_VOICE=0
for a in "$@"; do
  case "$a" in
    --skip-apt) SKIP_APT=1 ;;
    --skip-slam) SKIP_SLAM=1 ;;
    --skip-voice) SKIP_VOICE=1 ;;
    *) echo "Tham số lạ: $a"; exit 1 ;;
  esac
done
[ "$(id -u)" -eq 0 ] && { echo "Chạy bằng user thường (iot09), KHÔNG dùng sudo. Script tự hỏi sudo khi cần." >&2; exit 1; }

step() { echo; echo "==================== $* ===================="; }

if [ "$SKIP_APT" = 0 ]; then
  step "1/6 Gói hệ thống (sudo)"
  sudo bash ./install_deps.sh
else
  step "1/6 Bỏ qua apt"
fi

step "2/6 Linger (robotd chạy khi bật Pi)"
if [ "$(loginctl show-user "$USER" -p Linger --value 2>/dev/null)" != yes ]; then
  sudo loginctl enable-linger "$USER"
fi
loginctl show-user "$USER" -p Linger

step "3/6 Python venv"
bash ./setup_python.sh

if [ "$SKIP_SLAM" = 0 ]; then
  step "4/6 Build slam_node"
  bash ./build_slam.sh
else
  step "4/6 Bỏ qua build slam_node (web vẫn chạy, chưa có SLAM)"
fi

step "5/6 Dịch vụ robotd"
bash ./install_service.sh

if [ "$SKIP_VOICE" = 0 ]; then
  step "6/6 Điều khiển giọng nói (Speech-control)"
  bash ./setup_voice.sh || echo "!! Cài giọng nói lỗi (robotd vẫn chạy). Chạy lại: bash ~/iot-ck/pi/scripts/setup_voice.sh"
else
  step "6/6 Bỏ qua giọng nói"
fi

echo
echo "XONG. Web: http://$(hostname).local:8080   Log: journalctl --user -u robotd -f  /  -u voice -f"
echo "Lần đầu cài: sudo reboot (UART cho ESP32 chỉ có hiệu lực sau khi khởi động lại)"
