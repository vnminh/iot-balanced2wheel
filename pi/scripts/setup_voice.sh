#!/bin/bash
# Cài điều khiển giọng nói (KHÔNG cần sudo). Âm thanh mặc định đến từ ESP32 #2 qua WebSocket (firmware-mic/),
# Pi không cần mic / I2S; chỉ khi nối mic thẳng vào Pi mới cần sudo bash setup_mic.sh.
#   bash ~/iot-ck/pi/scripts/setup_voice.sh            pip + tải mô hình + cài dịch vụ voice
#   CHUNK_SIZE=16 bash setup_voice.sh                   mô hình chunk 16 (trễ ít hơn, kém chính xác hơn)
#   bash setup_voice.sh --no-service                    không cài dịch vụ
#
# Dùng chung venv ~/iot-ck/.venv với robotd. Mô hình: hynt/Zipformer-30M-RNNT-Streaming-6000h (giấy phép
# cc-by-nc-nd-4.0: chỉ dùng phi thương mại) + Silero VAD, lưu ở ~/iot-ck/Speech-control/models/.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SC="$ROOT/Speech-control"
VENV="$ROOT/.venv"
CHUNK_SIZE="${CHUNK_SIZE:-32}"
SERVICE=1
for a in "$@"; do
  case "$a" in
    --no-service) SERVICE=0 ;;
    *) echo "Tham số lạ: $a"; exit 1 ;;
  esac
done
case "$CHUNK_SIZE" in 16|32|64) ;; *) echo "CHUNK_SIZE phải là 16, 32 hoặc 64"; exit 1 ;; esac
[ -f "$SC/speech_control.py" ] || { echo "Không thấy $SC/speech_control.py (deploy code trước)"; exit 1; }

echo "==> Python: sounddevice, sherpa-onnx"
[ -x "$VENV/bin/python" ] || python3 -m venv --system-site-packages "$VENV"
"$VENV/bin/pip" install -q -r "$SC/requirements.txt"
"$VENV/bin/python" -c "import sherpa_onnx, numpy; print('sherpa-onnx', sherpa_onnx.__version__)"
"$VENV/bin/python" -c "import sounddevice" 2>/dev/null || echo "   (sounddevice chưa dùng được: chỉ cần khi nối mic thẳng vào Pi, --source mic, xem setup_mic.sh)"

echo "==> Mô hình (chunk $CHUNK_SIZE)"
MODEL="$SC/models/Zipformer-30M-RNNT-Streaming-6000h"
mkdir -p "$MODEL"
HF="https://huggingface.co/hynt/Zipformer-30M-RNNT-Streaming-6000h/resolve/main"
SUF="epoch-31-avg-11-chunk-${CHUNK_SIZE}-left-128.fp16.onnx"
get() {   # tải nếu chưa có (qua file .part: mất mạng giữa chừng không để lại file hỏng)
  if [ -s "$2" ] && ! head -c 40 "$2" | grep -q 'git-lfs'; then echo "    có rồi: ${2##*/}"; return; fi
  echo "    tải: ${2##*/}"
  curl -fL --retry 3 --retry-delay 2 -o "$2.part" "$1" && mv "$2.part" "$2"
}
get "$HF/config.json?download=true" "$MODEL/config.json"
for p in encoder decoder joiner; do get "$HF/$p-$SUF?download=true" "$MODEL/$p-$SUF"; done
get "https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/silero_vad.onnx" "$SC/models/silero_vad.onnx"
sync
(cd "$SC" && "$VENV/bin/python" speech_control.py --check --chunk-size "$CHUNK_SIZE") || echo "!! Nạp mô hình lỗi"

if [ "$SERVICE" = 1 ]; then
  echo "==> Dịch vụ voice (systemd --user, chạy cùng robotd)"
  mkdir -p ~/.config/systemd/user
  cp "$SC/voice.service" ~/.config/systemd/user/
  sync
  systemctl --user daemon-reload
  systemctl --user enable voice
  systemctl --user restart voice
  sleep 2
  systemctl --user --no-pager status voice | head -5 || true
  echo "Log: journalctl --user -u voice -f     Tắt: systemctl --user disable --now voice"
fi
