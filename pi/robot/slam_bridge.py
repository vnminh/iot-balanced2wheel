"""Chạy slam_node (lõi SLAM 2D C++, pi/slam) như tiến trình con.

- stdin của slam_node: bản ghi nhị phân <type u8><len u32><payload> (pi/slam/include/slam/protocol.h)
    'M' mẫu MCU 200 Hz (không bao giờ bỏ), 'I' ảnh xám (bỏ nếu node chưa nhận ảnh trước), 'C' lệnh text.
- stdout: JSON mỗi dòng: state (10 Hz), cmd (20 Hz khi đang goto), graph, grid, places, path, event, reply.
- Lệnh vận tốc của controller (cmd) chuyển qua callback on_cmd -> robotd gửi xuống ESP32.
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
from collections import deque
from typing import Callable, Optional

import numpy as np

MCU_STRUCT = struct.Struct("<dIHii3f3ffBB")   # 52 byte, khớp McuRecord
IMG_HDR = struct.Struct("<dHH")
SONAR_STRUCT = struct.Struct("<dBH")              # 11 byte, khớp SonarRecord (bản ghi 'S')
assert MCU_STRUCT.size == 52


def _rec(kind: bytes, payload: bytes) -> bytes:
    return kind + struct.pack("<I", len(payload)) + payload


class SlamBridge:
    def __init__(self, node: str, config: str, maps_dir: str, shm_dir: str, log_file: str = "",
                 autoload: str = "", autoload_mode: str = "localization",
                 on_cmd: Optional[Callable[[dict], None]] = None,
                 on_message: Optional[Callable[[dict], None]] = None):
        self.node = node
        self.args = [node, "--config", config, "--maps", maps_dir, "--shm", shm_dir]
        if log_file:
            self.args += ["--log", log_file]
        if autoload:
            self.args += ["--load", autoload, "--mode", autoload_mode]
        self.maps_dir = maps_dir
        self.shm_dir = shm_dir
        self.on_cmd = on_cmd
        self.on_message = on_message
        self.proc: Optional[subprocess.Popen] = None
        self._q: "queue.Queue[bytes]" = queue.Queue(maxsize=4000)
        self._img_pending = 0
        self._img_lock = threading.Lock()
        self._lock = threading.Lock()
        self.status = "stopped"
        self.state: Optional[dict] = None
        self.state_time = 0.0
        self.graph: Optional[dict] = None
        self.grid: Optional[dict] = None
        self.places: dict = {"type": "places", "items": []}
        self.path: dict = {"type": "path", "pts": [], "label": ""}
        self.events: deque = deque(maxlen=40)
        self.stats = {"img_sent": 0, "img_dropped": 0, "mcu_sent": 0}
        self.stderr_tail: list[str] = []

    # ------------------------------------------------------------ vòng đời
    def available(self) -> bool:
        return os.path.isfile(self.node) and os.access(self.node, os.X_OK)

    def start(self) -> bool:
        if not self.available():
            self.status = f"chưa build slam_node ({self.node}): chạy pi/scripts/build_slam.sh"
            return False
        os.makedirs(self.shm_dir, exist_ok=True)
        os.makedirs(self.maps_dir, exist_ok=True)
        self.proc = subprocess.Popen(self.args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.PIPE, bufsize=0)
        self.status = "running"
        threading.Thread(target=self._writer, name="slam-w", daemon=True).start()
        threading.Thread(target=self._reader, name="slam-r", daemon=True).start()
        threading.Thread(target=self._err_reader, name="slam-e", daemon=True).start()
        return True

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            try:
                self._q.put_nowait(_rec(b"Q", b""))
                self.proc.wait(timeout=3)
            except Exception:
                self.proc.kill()
        self.status = "stopped"

    # ------------------------------------------------------------- đầu vào
    def _put(self, msg: bytes) -> None:
        try:
            self._q.put_nowait(msg)
        except queue.Full:
            pass

    def push_mcu(self, s) -> None:
        """s = esp_link.ImuSample"""
        if self.status != "running":
            return
        pitch = s.pitch if s.pitch is not None else float("nan")
        self._put(_rec(b"M", MCU_STRUCT.pack(s.t, int(s.t_esp * 1e6) & 0xFFFFFFFF, s.seq & 0xFFFF,
                                             s.steps[0], s.steps[1], *s.gyro, *s.acc, pitch, s.state, s.flags)))
        self.stats["mcu_sent"] += 1

    def push_image(self, t: float, gray: np.ndarray) -> None:
        if self.status != "running":
            return
        with self._img_lock:
            if self._img_pending >= 1:  # node chưa nhận ảnh trước -> bỏ ảnh này (giữ thời gian thực)
                self.stats["img_dropped"] += 1
                return
            self._img_pending += 1
        h, w = gray.shape
        self._put(_rec(b"I", IMG_HDR.pack(t, w, h) + np.ascontiguousarray(gray).tobytes()))
        self.stats["img_sent"] += 1

    def push_sonar(self, s) -> None:
        """s = esp_link.SonarSample"""
        if self.status == "running":
            self._put(_rec(b"S", SONAR_STRUCT.pack(s.t, s.idx & 0xFF, s.mm & 0xFFFF)))

    def push_sonar_geom(self, raw: bytes) -> None:
        """Nguyên khung SONAR_GEOM do ESP32 gửi (bản ghi 'G')."""
        if self.status == "running":
            self._put(_rec(b"G", bytes(raw)))

    def push_raw(self, kind: bytes, payload: bytes) -> None:
        """Bản ghi đọc từ file log (robotd --replay)."""
        if self.status != "running":
            return
        if kind in (b"I", b"J"):
            with self._img_lock:
                if self._img_pending >= 1:
                    self.stats["img_dropped"] += 1
                    return
                self._img_pending += 1
            self.stats["img_sent"] += 1
        self._put(_rec(kind, payload))

    def command(self, line: str) -> None:
        line = " ".join(line.replace("\n", " ").split())
        if line:
            self._put(_rec(b"C", line.encode("utf-8")))

    # ------------------------------------------------------------ luồng
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
            if msg[:1] in (b"I", b"J"):
                with self._img_lock:
                    self._img_pending -= 1
        self.status = f"slam_node đã thoát (mã {self.proc.poll()})"

    def _err_reader(self) -> None:
        assert self.proc and self.proc.stderr
        for raw in self.proc.stderr:
            line = raw.decode("utf-8", "replace").rstrip()
            if line:
                self.stderr_tail = (self.stderr_tail + [line])[-30:]

    def _reader(self) -> None:
        assert self.proc and self.proc.stdout
        for raw in self.proc.stdout:
            try:
                m = json.loads(raw)
            except ValueError:
                continue
            kind = m.get("type")
            if kind == "cmd":
                if self.on_cmd:
                    self.on_cmd(m)
                continue
            with self._lock:
                if kind == "state":
                    self.state, self.state_time = m, time.monotonic()
                    continue
                if kind == "graph":
                    self.graph = m
                elif kind == "grid":
                    self.grid = m
                elif kind == "places":
                    self.places = m
                elif kind == "path":
                    self.path = m
                elif kind == "event":
                    self.events.append({"t": time.strftime("%H:%M:%S"), "level": m.get("level"), "msg": m.get("msg")})
            if self.on_message:
                self.on_message(m)
        self.status = f"slam_node đã thoát (mã {self.proc.poll()})"

    # ------------------------------------------------------- cho web server
    def info(self) -> dict:
        with self._lock:
            st = dict(self.state) if self.state else None
        out = {"status": self.status, "img_sent": self.stats["img_sent"], "img_dropped": self.stats["img_dropped"]}
        if st:
            st["age"] = round(time.monotonic() - self.state_time, 2)
            out.update(st)
            out["type"] = "slam"
        if self.status != "running" and self.stderr_tail:
            out["log"] = self.stderr_tail[-5:]
        return out

    def snapshot(self) -> list:
        """Các bản tin cần gửi cho client mới kết nối."""
        with self._lock:
            msgs = [self.graph, self.grid, self.places, self.path]
            ev = list(self.events)
        return [m for m in msgs if m] + [{"type": "events", "items": ev}]

    def quality(self) -> Optional[dict]:
        with self._lock:
            return (self.state or {}).get("quality")

    def localized(self) -> bool:
        with self._lock:
            return bool(self.state and self.state.get("localized") and time.monotonic() - self.state_time < 1.0)

    def nav_active(self) -> bool:
        with self._lock:
            st = (self.state or {}).get("nav", {}).get("state", "idle")
        return st in ("following", "rotating", "final_rotate")

    def grid_png(self) -> Optional[bytes]:
        p = os.path.join(self.shm_dir, "grid.png")
        try:
            with open(p, "rb") as f:
                return f.read()
        except OSError:
            return None

    def frame_jpg(self) -> Optional[bytes]:
        p = os.path.join(self.shm_dir, "frame.jpg")
        try:
            with open(p, "rb") as f:
                return f.read()
        except OSError:
            return None


def finite(x) -> bool:
    try:
        return math.isfinite(float(x))
    except (TypeError, ValueError):
        return False
