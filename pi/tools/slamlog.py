#!/usr/bin/env python3
"""Đọc file log SLAM (.slamlog, M12) trên laptop: tóm tắt, xuất CSV, xuất ảnh, vẽ quỹ đạo.

Định dạng: "SLAMLOG1" rồi các bản ghi <type u8><len u32 LE><payload> (pi/slam/include/slam/protocol.h)
  M = mẫu MCU 52 byte, I = ảnh xám (f64 t, u16 w, u16 h, pixel), J = ảnh JPEG (f64 t, jpeg), C = lệnh text.

  python3 pi/tools/slamlog.py info  file.slamlog
  python3 pi/tools/slamlog.py csv   file.slamlog mcu.csv          # t, bước trái/phải, gyro, acc, pitch, state, flags
  python3 pi/tools/slamlog.py images file.slamlog thu_muc/ [--every 5]
  python3 pi/tools/slamlog.py plot  out.jsonl                      # vẽ quỹ đạo từ đầu ra slam_node (cần matplotlib)

Replay (xác định, nhanh hơn thời gian thực):
  pi/slam/build/slam_node --config pi/config/slam.yaml --replay file.slamlog --maps /tmp/maps > out.jsonl
Chạy web không cần xe:
  cd pi && python3 -m robot.main --replay ../file.slamlog
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

MCU = struct.Struct("<dIHii3f3ffBB")


def records(path):
    with open(path, "rb") as f:
        if f.read(8) != b"SLAMLOG1":
            raise SystemExit(f"{path}: không phải log SLAM")
        while True:
            hdr = f.read(5)
            if len(hdr) < 5:
                return
            n = struct.unpack("<I", hdr[1:])[0]
            data = f.read(n)
            if len(data) < n:
                return
            yield chr(hdr[0]), data


def cmd_info(path):
    cnt, t0, t1, cmds = {}, None, None, []
    for k, d in records(path):
        cnt[k] = cnt.get(k, 0) + 1
        if k in "MIJS" and len(d) >= 8:
            t = struct.unpack("<d", d[:8])[0]
            t0 = t if t0 is None else t0
            t1 = t
        elif k == "C":
            cmds.append(d.decode("utf-8", "replace"))
    dur = (t1 - t0) if t0 is not None else 0
    print(f"{path}: {dur:.1f} s, MCU {cnt.get('M', 0)} ({cnt.get('M', 0) / max(dur, 1e-9):.0f} Hz), "
          f"ảnh {cnt.get('I', 0) + cnt.get('J', 0)}, siêu âm {cnt.get('S', 0)} số đo / {cnt.get('G', 0)} bảng vị trí, "
          f"lệnh {len(cmds)}")
    for c in cmds[:30]:
        print("  lệnh:", c)


def cmd_csv(path, out):
    with open(out, "w") as f:
        f.write("t,t_us,seq,steps_l,steps_r,gx,gy,gz,ax,ay,az,pitch,state,flags\n")
        for k, d in records(path):
            if k == "M":
                f.write(",".join(f"{v:.6f}" if isinstance(v, float) else str(v) for v in MCU.unpack(d[:52])) + "\n")
    print("đã ghi", out)


def cmd_images(path, folder, every=1):
    import cv2
    import numpy as np
    Path(folder).mkdir(parents=True, exist_ok=True)
    i = 0
    for k, d in records(path):
        if k not in "IJ":
            continue
        i += 1
        if (i - 1) % every:
            continue
        t = struct.unpack("<d", d[:8])[0]
        if k == "I":
            w, h = struct.unpack("<HH", d[8:12])
            img = np.frombuffer(d[12:12 + w * h], np.uint8).reshape(h, w)
        else:
            img = cv2.imdecode(np.frombuffer(d[8:], np.uint8), cv2.IMREAD_GRAYSCALE)
        cv2.imwrite(str(Path(folder) / f"{t:.3f}.png"), img)
    print(f"đã xuất ảnh vào {folder}")


def cmd_plot(jsonl):
    import matplotlib.pyplot as plt
    graph, poses = None, []
    for line in open(jsonl):
        try:
            m = json.loads(line)
        except ValueError:
            continue
        if m.get("type") == "graph":
            graph = m
        elif m.get("type") == "state" and m.get("localized"):
            poses.append(m["pose"][:2])
    fig, ax = plt.subplots(figsize=(8, 8))
    if graph:
        tr = graph["traj"]
        ax.plot([p[1] for p in tr], [p[0] for p in tr], "-", color="#e55", lw=1, label="quỹ đạo tối ưu")
        k = graph["kfs"]
        ax.scatter([p[1] for p in k], [p[0] for p in k], s=8, c=["#888" if p[3] else "#09c" for p in k], label="keyframe")
        for a, b, kind in graph["edges"]:
            if kind:
                ax.plot([k[a][1], k[b][1]], [k[a][0], k[b][0]], "-", color="#fc0" if kind == 1 else "#c6f", lw=0.6)
    if poses:
        ax.plot([p[1] for p in poses], [p[0] for p in poses], ":", color="#3a3", lw=1, label="tư thế 10 Hz")
    ax.invert_xaxis()   # y sang trái, như web
    ax.set_aspect("equal")
    ax.set_xlabel("y (m)")
    ax.set_ylabel("x (m)")
    ax.legend()
    ax.grid(alpha=0.3)
    plt.show()


def main():
    a = sys.argv[1:]
    if len(a) >= 2 and a[0] == "info":
        cmd_info(a[1])
    elif len(a) >= 3 and a[0] == "csv":
        cmd_csv(a[1], a[2])
    elif len(a) >= 3 and a[0] == "images":
        every = int(a[a.index("--every") + 1]) if "--every" in a else 1
        cmd_images(a[1], a[2], every)
    elif len(a) >= 2 and a[0] == "plot":
        cmd_plot(a[1])
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
