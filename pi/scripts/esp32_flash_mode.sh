#!/bin/bash
# Dây UART của Pi đang nối vào UART0 (RX0/TX0) của ESP32 - chung với cổng USB nạp code.
# Chân TX của Pi giữ đường RX0 nên máy tính KHÔNG nạp được. Trước khi nạp:
#   bash esp32_flash_mode.sh release   (GPIO14 -> input, thả đường dây; dừng robotd trước)
# Nạp xong:
#   bash esp32_flash_mode.sh restore   (GPIO14 -> TXD0)
set -euo pipefail
case "${1:-}" in
  release) systemctl --user stop robotd 2>/dev/null || true; pkill -f "[r]obot\.main" || true; pinctrl set 14 ip pn ;;
  restore) pinctrl set 14 a0 pn ;;
  *) echo "dùng: $0 release|restore"; exit 1 ;;
esac
pinctrl get 14
