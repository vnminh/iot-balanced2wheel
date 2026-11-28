#!/usr/bin/env bash
# Cài riêng phần giọng nói (khi robotd đã cài). Toàn bộ hệ thống: bash ~/iot-ck/pi/scripts/install.sh
#   pip + tải mô hình Zipformer/Silero + dịch vụ voice (WebSocket server :8765 nhận âm thanh từ ESP32 #2).
#   Mic nối thẳng vào Pi (--source mic): chạy thêm sudo bash ~/iot-ck/pi/scripts/setup_mic.sh
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "$HERE/../pi/scripts/setup_voice.sh" "$@"
