#!/bin/bash
# Flash Raspberry Pi OS lên thẻ SD GIẢ DUNG LƯỢNG (vd. báo 64 GB, thật 32 GB theo f3probe)
# mà không bao giờ dùng tới vùng không có thật.
#
#   sudo LIMIT_GIB=<n> ./scripts/flash_pi_sd.sh /dev/sdb [image.img.xz] [thư_mục_cấu_hình]
#   LIMIT_GIB (bắt buộc): rootfs kết thúc ở n GiB. Lấy từ "Vùng an toàn" của
#   scripts/sd_scan.py, trừ đi vài GiB dự phòng (f3probe KHÔNG đủ: nó chỉ thử vài điểm).
#
# Raspberry Pi OS lần đầu khởi động sẽ nới rootfs ra HẾT thẻ (cờ "resize" trong
# cmdline.txt -> initramfs resize_early chạy parted, rồi systemd-growfs-root).
# Với thẻ giả, việc đó làm ghi vòng đè lên dữ liệu cũ -> hỏng hệ thống. Script này:
#   1. ghi image
#   2. sửa bảng phân vùng: p2 (rootfs) kết thúc ở LIMIT_GIB, p3 = phân vùng "chặn"
#      kiểu 0xda (Non-FS data), không có filesystem, phủ phần còn lại -> không công
#      cụ nào nới p2 được nữa
#   3. nới filesystem p2 ngay trên laptop (resize2fs)
#   4. bỏ " resize" và console serial khỏi cmdline.txt; chép user-data / network-config
#      / config.txt cũ (user iot09, wifi, UART, voicehat) vào bootfs
# KHÔNG BAO GIỜ format / mount p3 (/dev/sdX3) và không chạy "Expand filesystem" trong raspi-config.
set -euo pipefail

DEV="${1:?dùng: sudo $0 /dev/sdX [image.img.xz] [thư_mục_cấu_hình]}"
U_HOME="$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)"
IMG="${2:-$(ls -1 "$U_HOME"/Downloads/raspios/*raspios*arm64*.img.xz 2>/dev/null | sort | tail -1)}"
CFG="${3:-$U_HOME/Downloads/raspios/boot-config}"
LIMIT_GIB="${LIMIT_GIB:?đặt LIMIT_GIB=<số GiB an toàn>, vd: sudo LIMIT_GIB=12 $0 /dev/sdX}"

[ "$(id -u)" -eq 0 ] || { echo "Cần chạy bằng sudo." >&2; exit 1; }
[ -b "$DEV" ] || { echo "$DEV không phải block device." >&2; exit 1; }
[ -f "$IMG" ] || { echo "Không thấy image: $IMG" >&2; exit 1; }
for f in user-data network-config config.txt; do
  [ -f "$CFG/$f" ] || { echo "Thiếu $CFG/$f" >&2; exit 1; }
done
NAME="$(basename "$DEV")"
[ "$(cat /sys/block/"$NAME"/removable)" = 1 ] || { echo "$DEV không phải thẻ tháo rời được, dừng để an toàn." >&2; exit 1; }
if lsblk -no MOUNTPOINTS "$DEV" | grep -qE '^/($|boot|home|usr)'; then
  echo "$DEV đang chứa hệ thống của laptop! Dừng." >&2; exit 1
fi

SECTORS=$(cat /sys/block/"$NAME"/size)
LIMIT_SEC=$((LIMIT_GIB * 1024 * 1024 * 2))   # sector 512 B
[ "$LIMIT_SEC" -lt "$SECTORS" ] || { echo "LIMIT_GIB lớn hơn thẻ." >&2; exit 1; }

echo "Thẻ:    $DEV  $(lsblk -dno SIZE,MODEL "$DEV")  ($SECTORS sector)"
echo "Image:  $IMG"
echo "Cấu hình: $CFG"
echo "rootfs giới hạn ở $LIMIT_GIB GiB, phần sau làm phân vùng chặn."
echo "TOÀN BỘ dữ liệu trên $DEV sẽ bị xoá."
read -rp "Gõ YES để tiếp tục: " ans
[ "$ans" = YES ] || { echo "Huỷ."; exit 1; }

echo "==> Unmount"
for p in $(lsblk -lno PATH "$DEV" | tail -n +2); do umount "$p" 2>/dev/null || true; done

echo "==> [1/4] Ghi image (vài phút)"
xz -dc "$IMG" | dd of="$DEV" bs=4M conv=fsync status=progress
sync
partprobe "$DEV" 2>/dev/null || blockdev --rereadpt "$DEV"; udevadm settle

echo "==> [2/4] Sửa bảng phân vùng"
DUMP="$(sfdisk -d "$DEV")"
LABEL_ID=$(awk '/^label-id:/{print $2}' <<<"$DUMP")
P1_START=$(sed -nE 's/^\S+1 : start= *([0-9]+), size= *([0-9]+).*/\1/p' <<<"$DUMP")
P1_SIZE=$(sed -nE 's/^\S+1 : start= *([0-9]+), size= *([0-9]+).*/\2/p' <<<"$DUMP")
P2_START=$(sed -nE 's/^\S+2 : start= *([0-9]+), size= *([0-9]+).*/\1/p' <<<"$DUMP")
[ -n "$LABEL_ID" ] && [ -n "$P1_START" ] && [ -n "$P2_START" ] || { echo "Không đọc được bảng phân vùng:"; echo "$DUMP"; exit 1; }
P2_SIZE=$((LIMIT_SEC - P2_START))
# label-id giữ nguyên: cmdline.txt/fstab dùng PARTUUID=<label-id>-02
sfdisk --no-reread --wipe never "$DEV" <<EOF
label: dos
label-id: $LABEL_ID
unit: sectors

start=$P1_START, size=$P1_SIZE, type=c
start=$P2_START, size=$P2_SIZE, type=83
start=$LIMIT_SEC, type=da
EOF
partprobe "$DEV" 2>/dev/null || blockdev --rereadpt "$DEV"; udevadm settle
P1="$(lsblk -lno PATH "$DEV" | sed -n 2p)"
P2="$(lsblk -lno PATH "$DEV" | sed -n 3p)"

echo "==> [3/4] Nới filesystem rootfs tới $LIMIT_GIB GiB"
e2fsck -fy "$P2" || [ $? -le 1 ]
resize2fs "$P2"

echo "==> [4/4] Cấu hình bootfs"
MNT="$(mktemp -d)"
mount "$P1" "$MNT"
trap 'umount "$MNT" 2>/dev/null || true; rmdir "$MNT" 2>/dev/null || true' EXIT
cp "$CFG/user-data" "$CFG/network-config" "$CFG/config.txt" "$MNT/"
# bỏ cờ resize (tự nới phân vùng) + console serial (UART dành cho ESP32)
sed -i -e 's/ resize\b//g' -e 's/console=serial0,[0-9]* *//g' "$MNT/cmdline.txt"
grep -q 'cfg80211.ieee80211_regdom' "$MNT/cmdline.txt" || sed -i 's/$/ cfg80211.ieee80211_regdom=VN/' "$MNT/cmdline.txt"
echo "cmdline.txt: $(cat "$MNT/cmdline.txt")"
if grep -q ' resize' "$MNT/cmdline.txt"; then echo "!! cmdline.txt vẫn còn resize" >&2; exit 1; fi
umount "$MNT"

# Tắt fstrim: lệnh TRIM trên thẻ giả có thể xoá nhầm vùng khác
mount "$P2" "$MNT"
for u in fstrim.timer fstrim.service; do ln -sf /dev/null "$MNT/etc/systemd/system/$u"; done
rm -f "$MNT/etc/systemd/system/timers.target.wants/fstrim.timer"
echo "fstrim: đã tắt (mask)"
umount "$MNT"
sync

echo
sfdisk -l "$DEV" | tail -4
echo
echo "XONG. Rút thẻ, cắm vào Pi, chờ ~2-3 phút (lần đầu tự khởi động lại 1 lần), rồi:"
echo "  ssh iot09@raspberrypi4.local 'df -h /; lsblk; dmesg | grep -i ext4-fs'   # / ~${LIMIT_GIB}G, KHÔNG có dòng \"EXT4-fs error\""
echo "  ./scripts/deploy_pi.sh --install"
