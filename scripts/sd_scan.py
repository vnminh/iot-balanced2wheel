#!/usr/bin/env python3
"""Quét toàn bộ thẻ nhớ để tìm vùng ghi/đọc đúng (phát hiện thẻ giả, ghi vòng, vùng hỏng).

  sudo python3 scripts/sd_scan.py /dev/sdb --gib 32          (XOÁ SẠCH thẻ!)
  sudo python3 scripts/sd_scan.py /dev/sdb --gib 32 --read-only   (chỉ đọc lại lần quét trước)

Pha 1 ghi toàn bộ [0, gib) với mỗi sector 512 B chứa chính số thứ tự của nó.
Pha 2 đọc lại TẤT CẢ (sau khi đã ghi hết, nên lộ ra cả việc ghi vòng đè lên nhau).
Sector đọc ra mang số khác -> biết bị đè TỪ địa chỉ nào (thẻ giả ghi vòng);
sector rác -> chip hỏng. Dùng O_DIRECT nên không bị cache của Linux đánh lừa.
"""
import argparse
import mmap
import os
import sys
import time

import struct

SECTOR = 512
CHUNK = 4 * 1024 * 1024
SPC = CHUNK // SECTOR           # sector / chunk
MAGIC = 0x5344534341_4E0000     # "SDSCAN"
HDR = struct.Struct("<QQ")      # magic, số sector; lặp 32 lần = 512 B


def pattern(first_sector: int) -> bytes:
    pk = HDR.pack
    return b"".join(pk(MAGIC, i) * 32 for i in range(first_sector, first_sector + SPC))


def fmt_gib(sectors: int) -> str:
    return f"{sectors * SECTOR / 2**30:.2f} GiB"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("dev")
    ap.add_argument("--gib", type=float, required=True, help="quét [0, gib) GiB")
    ap.add_argument("--read-only", action="store_true")
    a = ap.parse_args()

    total = int(a.gib * 2**30) // CHUNK * SPC
    nchunks = total // SPC
    buf = mmap.mmap(-1, CHUNK)                       # căn trang -> dùng được O_DIRECT
    flags = os.O_RDWR | getattr(os, "O_DIRECT", 0)
    fd = os.open(a.dev, flags)
    dev_sectors = os.lseek(fd, 0, os.SEEK_END) // SECTOR
    if total > dev_sectors:
        print(f"Thiết bị chỉ có {fmt_gib(dev_sectors)}"); return 1

    if not a.read_only:
        if input(f"XOÁ SẠCH {a.dev} vùng 0..{fmt_gib(total)}? Gõ YES: ") != "YES":
            return 1
        t0 = time.time()
        os.lseek(fd, 0, os.SEEK_SET)
        for c in range(nchunks):
            buf[:] = pattern(c * SPC)
            os.write(fd, buf)
            if c % 64 == 0 or c == nchunks - 1:
                done = (c + 1) * CHUNK
                el = time.time() - t0
                print(f"\rGhi {done / 2**30:6.2f}/{a.gib:.2f} GiB  {done / el / 1e6:5.1f} MB/s  "
                      f"còn ~{(nchunks - c - 1) * CHUNK / (done / el) / 60:4.0f} phút", end="", flush=True)
        os.fsync(fd)
        print()

    t0 = time.time()
    os.lseek(fd, 0, os.SEEK_SET)
    GIB_SEC = 2**30 // SECTOR
    per_gib = {}            # gib -> [tốt, đè, rác]
    alias = {}              # (gib, độ lệch GiB) -> số sector
    first_bad = None
    for c in range(nchunks):
        n = os.readv(fd, [buf])
        if n != CHUNK:
            print(f"\nĐọc thiếu tại {fmt_gib(c * SPC)}"); break
        exp = pattern(c * SPC)
        g = (c * SPC) // GIB_SEC
        st = per_gib.setdefault(g, [0, 0, 0])
        got = bytes(buf)
        if got == exp:
            st[0] += SPC
        else:
            for k in range(SPC):
                o = k * SECTOR
                sec = got[o:o + SECTOR]
                if sec == exp[o:o + SECTOR]:
                    st[0] += 1
                    continue
                if first_bad is None:
                    first_bad = c * SPC + k
                m, idx = HDR.unpack_from(sec)
                if m == MAGIC and sec == HDR.pack(m, idx) * 32:
                    st[1] += 1
                    key = (g, round((idx - (c * SPC + k)) * SECTOR / 2**30, 1))
                    alias[key] = alias.get(key, 0) + 1
                else:
                    st[2] += 1
        if c % 64 == 0 or c == nchunks - 1:
            done = (c + 1) * CHUNK
            print(f"\rĐọc {done / 2**30:6.2f}/{a.gib:.2f} GiB  {done / (time.time() - t0) / 1e6:5.1f} MB/s", end="", flush=True)
    print("\n")
    print(" GiB   tốt%   bị-đè   rác")
    for g in sorted(per_gib):
        t, o, r = per_gib[g]
        tot = t + o + r
        print(f"{g:4d}  {100 * t / tot:6.2f}  {o:7d}  {r:6d}")
    if alias:
        print("\nBị đè bởi dữ liệu của địa chỉ khác (GiB vùng, lệch GiB: số sector), 15 dòng nhiều nhất:")
        for (g, off), n in sorted(alias.items(), key=lambda x: -x[1])[:15]:
            print(f"  vùng {g} GiB  <- dữ liệu của địa chỉ {off:+.1f} GiB : {n}")
    if first_bad is None:
        print(f"\nTOÀN BỘ {fmt_gib(total)} ĐỀU ĐÚNG.")
    else:
        print(f"\nSector sai đầu tiên: {first_bad} (= {fmt_gib(first_bad)}). "
              f"Vùng an toàn liên tục từ đầu thẻ: {fmt_gib(first_bad)}")
    os.close(fd)
    return 0


if __name__ == "__main__":
    sys.exit(main())
