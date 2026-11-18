#!/bin/bash
# Cài robotd thành dịch vụ systemd --user (tự chạy khi Pi bật nếu đã enable-linger).
set -euo pipefail
mkdir -p ~/.config/systemd/user
cp "$(dirname "$0")/robotd.service" ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now robotd
systemctl --user --no-pager status robotd | head -5
echo "Log: journalctl --user -u robotd -f"
