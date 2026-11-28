#!/bin/bash
# Nạp firmware cho ESP32 #2 (mic INMP441 -> WiFi WebSocket -> Pi). Cắm USB ESP32 #2 vào LAPTOP rồi chạy:
#   ./scripts/flash_esp32_mic.sh            build + nạp + mở cổng xem log (Ctrl+C thoát)
#   PORT=/dev/ttyUSB1 ./scripts/flash_esp32_mic.sh
# Lần đầu: chép firmware-mic/include/secrets.h.example thành secrets.h, điền tên + mật khẩu WiFi mà Pi đang dùng.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PIO="$(command -v pio || true)"
[ -z "$PIO" ] && [ -x "$HOME/.platformio/penv/bin/pio" ] && PIO="$HOME/.platformio/penv/bin/pio"
[ -z "$PIO" ] && { echo "Không tìm thấy PlatformIO (pio). Cài: pip install platformio" >&2; exit 1; }
SEC="$ROOT/firmware-mic/include/secrets.h"
if [ ! -f "$SEC" ]; then
  cp "$ROOT/firmware-mic/include/secrets.h.example" "$SEC"
  echo "!! Đã tạo $SEC: điền WIFI_SSID / WIFI_PASS rồi chạy lại." >&2
  exit 1
fi
if grep -q '"TEN_WIFI"' "$SEC"; then
  echo "!! Chưa điền WiFi trong $SEC" >&2
  exit 1
fi
ARGS=()
[ -n "${PORT:-}" ] && ARGS=(--upload-port "$PORT")
"$PIO" run -d "$ROOT/firmware-mic" -t upload "${ARGS[@]}"
echo "==> Log (thấy 'Da noi Pi, dang gui am thanh' là được; Ctrl+C để thoát)"
"$PIO" device monitor -d "$ROOT/firmware-mic" ${PORT:+--port "$PORT"}
