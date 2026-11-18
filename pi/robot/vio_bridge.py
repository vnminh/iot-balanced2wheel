"""Chạy ovw_node (OpenVINS + odometry bánh xe) như tiến trình con và gom bản đồ.

- Gửi IMU/bánh xe (200 Hz) và ảnh xám vào stdin của ovw_node (giao thức nhị phân).
- Ảnh bị bỏ nếu node đang bận (luôn giữ thời gian thực); IMU không bao giờ bỏ.
- Đọc JSON từ stdout: tư thế + điểm 3D mới -> quỹ đạo và đám mây điểm.
- Bản đồ được lưu theo lưới voxel (mặc định 5 cm) để RAM không tăng mãi.

Khung hiển thị: gốc = vị trí lúc khởi tạo, trục X = hướng mũi xe lúc khởi tạo,
Z hướng lên (OpenVINS chọn hướng X thế giới tuỳ ý, ta xoay lại cho dễ nhìn).
"""
from __future__ import annotations

import json
import math
import os
import queue
import struct
import subprocess
import threading
import time
from typing import Optional

import numpy as np

IMU_STRUCT = struct.Struct("<d3f3f2iB")
CAM_STRUCT = struct.Struct("<dHH")


class VioBridge:
    def __init__(self, node_path: str, estimator_cfg: str, wheel_cfg: str,
                 voxel: float = 0.05, max_points: int = 200_000):
        self.node_path = node_path
        self.args = [node_path, estimator_cfg, wheel_cfg]
        self.voxel = voxel
        self.max_points = max_points
        self.proc: Optional[subprocess.Popen] = None
        self._q: "queue.Queue[bytes]" = queue.Queue(maxsize=4000)
        self._img_pending = 0
        self._img_lock = threading.Lock()
        self._lock = threading.Lock()
        self.status = "stopped"
        self.last_pose: Optional[dict] = None
        self._origin = None  # (p0, yaw0)
        self._reset_map()
        self.stats = {"img_sent": 0, "img_dropped": 0, "imu_sent": 0}
        self.stderr_tail: list[str] = []

    # ------------------------------------------------------------ vòng đời
    def available(self) -> bool:
        return os.path.isfile(self.node_path) and os.access(self.node_path, os.X_OK)

    def start(self) -> bool:
        if not self.available():
            self.status = f"chưa build ovw_node ({self.node_path})"
            return False
        self.proc = subprocess.Popen(self.args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0)
        self.status = "running"
        threading.Thread(target=self._writer, name="vio-w", daemon=True).start()
        threading.Thread(target=self._reader, name="vio-r", daemon=True).start()
        threading.Thread(target=self._err_reader, name="vio-e", daemon=True).start()
        return True

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            try:
                self._q.put_nowait(b"Q")
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()
        self.status = "stopped"

    def reset(self) -> None:
        """Khởi động lại bộ lọc và xoá bản đồ."""
        with self._lock:
            self._reset_map()
        self._put(b"R")

    def _reset_map(self) -> None:
        self.voxels: dict = {}          # (ix,iy,iz) -> [x,y,z,count]
        self.new_points: list = []      # điểm mới chưa gửi cho web
        self.traj: list = []            # [[x,y,z],...] khung hiển thị
        self.new_traj: list = []
        self._origin = None
        self.map_version = time.time()

    # ------------------------------------------------------------- đầu vào
    def _put(self, msg: bytes) -> None:
        try:
            self._q.put_nowait(msg)
        except queue.Full:
            pass

    def push_imu(self, t: float, acc, gyro, steps, flags: int) -> None:
        if self.status != "running":
            return
        self._put(b"I" + IMU_STRUCT.pack(t, *acc, *gyro, steps[0], steps[1], flags))
        self.stats["imu_sent"] += 1

    def push_image(self, t: float, gray: np.ndarray) -> None:
        if self.status != "running":
            return
        with self._img_lock:
            if self._img_pending >= 1:  # node chưa kịp nhận ảnh trước -> bỏ ảnh này
                self.stats["img_dropped"] += 1
                return
            self._img_pending += 1
        h, w = gray.shape
        self._put(b"C" + CAM_STRUCT.pack(t, w, h) + gray.tobytes())
        self.stats["img_sent"] += 1

    def _writer(self) -> None:
        assert self.proc and self.proc.stdin
        while self.proc.poll() is None:
            try:
                msg = self._q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.proc.stdin.write(msg)
            except (BrokenPipeError, OSError):
                break
            if msg[:1] == b"C":
                with self._img_lock:
                    self._img_pending -= 1
        self.status = f"ovw_node đã thoát (mã {self.proc.poll()})"

    def _err_reader(self) -> None:
        assert self.proc and self.proc.stderr
        for raw in self.proc.stderr:
            line = raw.decode("utf-8", "replace").rstrip()
            if line:
                self.stderr_tail = (self.stderr_tail + [line])[-30:]

    # -------------------------------------------------------------- đầu ra
    def _to_display(self, p):
        p0, yaw0 = self._origin
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        dx, dy = p[0] - p0[0], p[1] - p0[1]
        return [c * dx - s * dy, s * dx + c * dy, p[2] - p0[2]]

    def _reader(self) -> None:
        assert self.proc and self.proc.stdout
        for raw in self.proc.stdout:
            try:
                m = json.loads(raw)
            except ValueError:
                continue
            if m.get("type") == "status":
                if m.get("msg") == "reset":
                    with self._lock:
                        self._reset_map()
                continue
            if m.get("type") != "pose":
                continue
            with self._lock:
                if not m.get("init"):
                    self.last_pose = m
                    continue
                if self._origin is None:
                    self._origin = (m["p"], m["yaw"])
                pd = self._to_display(m["p"])
                m["p_disp"] = pd
                m["yaw_disp"] = math.atan2(math.sin(m["yaw"] - self._origin[1]), math.cos(m["yaw"] - self._origin[1]))
                self.last_pose = m
                if not self.traj or math.dist(self.traj[-1], pd) > 0.02:
                    pt = [round(v, 3) for v in pd]
                    self.traj.append(pt)
                    self.new_traj.append(pt)
                for f in m.get("pts", []) + m.get("slam_pts", []):
                    self._add_point(self._to_display(f))

    def _add_point(self, q) -> None:
        if len(self.voxels) >= self.max_points:
            return
        k = (int(math.floor(q[0] / self.voxel)), int(math.floor(q[1] / self.voxel)), int(math.floor(q[2] / self.voxel)))
        v = self.voxels.get(k)
        if v is None:
            self.voxels[k] = [q[0], q[1], q[2], 1]
            self.new_points.append([round(q[0], 3), round(q[1], 3), round(q[2], 3)])
        else:  # trung bình dần vị trí trong voxel
            n = v[3] + 1
            for i in range(3):
                v[i] += (q[i] - v[i]) / n
            v[3] = n

    # ------------------------------------------------------- cho web server
    def snapshot(self) -> dict:
        with self._lock:
            return {"traj": list(self.traj),
                    "pts": [[round(v[0], 3), round(v[1], 3), round(v[2], 3)] for v in self.voxels.values()],
                    "version": self.map_version}

    def take_updates(self) -> tuple[list, list]:
        with self._lock:
            t, p = self.new_traj, self.new_points
            self.new_traj, self.new_points = [], []
            return t, p

    def info(self) -> dict:
        m = self.last_pose or {}
        out = {"status": self.status, "init": bool(m.get("init")), "points": len(self.voxels),
               "img_sent": self.stats["img_sent"], "img_dropped": self.stats["img_dropped"],
               "version": self.map_version}
        if m.get("init"):
            out.update({"p": m["p_disp"], "yaw": round(m["yaw_disp"], 4), "pitch": round(m.get("pitch", 0), 4),
                        "v": m.get("v"), "ms": m.get("ms"), "slam": m.get("slam"), "td": m.get("td"),
                        "vw": m.get("vw"), "wheel_ok": m.get("wheel_ok"), "wheel_rej": m.get("wheel_rej")})
        elif m:
            out["ms"] = m.get("ms")
        if self.status != "running" and self.stderr_tail:
            out["log"] = self.stderr_tail[-5:]
        return out

    def export_ply(self) -> bytes:
        with self._lock:
            pts = [v[:3] for v in self.voxels.values()]
            traj = list(self.traj)
        lines = ["ply", "format ascii 1.0", f"element vertex {len(pts) + len(traj)}",
                 "property float x", "property float y", "property float z",
                 "property uchar red", "property uchar green", "property uchar blue", "end_header"]
        lines += [f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} 200 200 200" for p in pts]
        lines += [f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} 255 60 60" for p in traj]
        return ("\n".join(lines) + "\n").encode()
