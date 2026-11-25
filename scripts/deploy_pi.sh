#!/bin/bash
# Đồng bộ code từ LAPTOP lên Pi (~/iot-ck), rồi cài đặt hoặc khởi động lại robotd.
#
#   ./scripts/deploy_pi.sh             chỉ copy code + restart robotd (nếu đã cài service)
#   ./scripts/deploy_pi.sh --install   copy code + chạy pi/scripts/install.sh trên Pi
#                                      (hỏi mật khẩu sudo của Pi, lần đầu mất ~10 phút)
#   ./scripts/deploy_pi.sh --build     copy code + build lại slam_node + restart robotd
#   ./scripts/deploy_pi.sh --install --skip-slam  cài nhưng không build slam_node
#   PI=iot09@192.168.1.50 ./scripts/deploy_pi.sh
#
# Không dùng --delete: file chỉ có trên Pi (ảnh hiệu chỉnh, log...) được giữ nguyên.
set -euo pipefail

PI="${PI:-iot09@raspberrypi4.local}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INSTALL=0
BUILD=0
INSTALL_ARGS=()
for a in "$@"; do
  case "$a" in
    --install) INSTALL=1 ;;
    --build) BUILD=1 ;;
    -h|--help) sed -n 2,12p "$0"; exit 0 ;;
    *) INSTALL_ARGS+=("$a") ;;
  esac
done

echo "==> Copy code lên $PI:~/iot-ck"
rsync -az --info=stats1 \
  --exclude .git/ --exclude .venv/ --exclude .vscode/ --exclude __pycache__/ --exclude '*.pyc' \
  --exclude firmware/.pio/ --exclude pi/slam/build/ --exclude .build-slam/ --exclude maps/ --exclude logs/ \
  --exclude '*.slamlog' \
  "$ROOT/" "$PI:iot-ck/"

# File của hệ OpenVINS cũ đã xoá khỏi repo: xoá luôn trên Pi (rsync không --delete nên không tự xoá)
ssh "$PI" 'cd ~/iot-ck && rm -rf pi/vio pi/tests pi/robot/vio_bridge.py pi/robot/nav.py pi/scripts/build_vio.sh \
  pi/config/estimator_config.yaml pi/config/kalibr_imu_chain.yaml pi/config/kalibr_imucam_chain.yaml pi/config/wheel.yaml'

if [ "$INSTALL" = 1 ]; then
  echo "==> Cài đặt trên Pi"
  ssh -t "$PI" "bash ~/iot-ck/pi/scripts/install.sh ${INSTALL_ARGS[*]:-}"
else
  if [ "$BUILD" = 1 ]; then
    echo "==> Build slam_node trên Pi"
    ssh "$PI" 'bash ~/iot-ck/pi/scripts/build_slam.sh'
  fi
  ssh "$PI" 'if systemctl --user is-enabled --quiet robotd 2>/dev/null; then
               systemctl --user restart robotd && echo "==> Đã restart robotd";
             else
               echo "==> robotd chưa cài service. Chạy: ./scripts/deploy_pi.sh --install";
             fi'
fi
echo "Web: http://${PI#*@}:8080"
