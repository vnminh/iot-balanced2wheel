#!/bin/bash
# Cài robotd thành dịch vụ systemd --user (tự chạy khi Pi bật nếu đã enable-linger).
set -euo pipefail
mkdir -p ~/.config/systemd/user
cp "$(dirname "$0")/robotd.service" ~/.config/systemd/user/
sync   # mất điện ngay sau khi chép -> file rỗng 0 byte (đã gặp)
systemctl --user daemon-reload
systemctl --user enable robotd
systemctl --user restart robotd   # restart để nạp code mới nếu đã chạy
sleep 2
systemctl --user --no-pager status robotd | head -5
echo "Log: journalctl --user -u robotd -f"
