#!/bin/bash
# TUỲ CHỌN. Mặc định mic nằm trên ESP32 #2 (firmware-mic/) và gửi qua WiFi, KHÔNG cần script này.
# Chỉ dùng khi nối INMP441 thẳng vào Pi (rồi đặt --source mic trong Speech-control/voice.env).
# Bật mic I2S INMP441 trên Raspberry Pi 4 (cần sudo, chạy 1 lần, xong phải khởi động lại):
#   sudo bash ~/iot-ck/pi/scripts/setup_mic.sh
#
# Đấu dây INMP441 -> Pi 4 (KHÔNG cấp 5 V):
#   VDD -> 3V3 (chân 1)      GND -> GND (chân 6/9)
#   SCK -> GPIO18 (chân 12)  WS  -> GPIO19 (chân 35)   SD -> GPIO20 (chân 38)
#   L/R -> GND  => mic ở kênh TRÁI (Speech-control mặc định --channel 0)
# Không đụng UART ESP32 (GPIO14/15) và camera CSI. Overlay googlevoicehat còn giữ GPIO16 (chân bật loa): đừng dùng.
set -euo pipefail
if [ "$(id -u)" -ne 0 ]; then echo "Chạy bằng: sudo bash $0" >&2; exit 1; fi

apt-get install -y --no-install-recommends alsa-utils libportaudio2 sox

CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt
REBOOT=0
add_line() {   # thêm dòng vào cuối config.txt (sau [all]) nếu chưa có
  if ! grep -Eq "^[[:space:]]*$1[[:space:]]*$" "$CFG"; then
    tail -n 5 "$CFG" | grep -q '^\[all\]' || printf '\n[all]\n' >> "$CFG"
    echo "$1" >> "$CFG"
    echo "  + $1"
    REBOOT=1
  fi
}
echo "==> I2S + overlay mic ($CFG)"
add_line 'dtparam=i2s=on'
add_line 'dtoverlay=googlevoicehat-soundcard'

if [ "$REBOOT" = 1 ]; then
  echo "!! Đã bật I2S: cần KHỞI ĐỘNG LẠI Pi (sudo reboot). Sau đó kiểm tra: arecord -l (thấy googlevoicehat)"
else
  echo "I2S đã bật sẵn."
  arecord -l 2>/dev/null | grep -i -E 'voicehat|googlevoi' || echo "!! Chưa thấy card googlevoicehat: đã khởi động lại sau khi bật chưa?"
fi
echo "Thử thu 5 s kênh trái:  arecord -D plughw:CARD=sndrpigooglevoi -c 2 -r 48000 -f S32_LE -d 5 /tmp/t.wav && sox /tmp/t.wav -n remix 1 stat"
