"""Siêu âm phía Pi: bảng vị trí do ESP32 gửi (khung SONAR_GEOM) + số đo mới nhất + khoảng trống phía trước.

Pi không có cấu hình vị trí cảm biến. Thêm / dời cảm biến: sửa SONAR_SENSORS trong firmware/include/config.h
rồi nạp lại ESP32. Khung SONAR_GEOM (little-endian):
    u8 version (=1) · u8 count · u16 max_range_mm
    count lần: i16 x_mm · i16 y_mm · i16 yaw_cdeg (0,01°, + = trái) · u8 half_deg · u16 stop_mm

Khoảng trống phía trước (dùng chung công thức với slam_node, pi/slam/src/obstacle_grid.cpp):
    vật có thể nằm ở bất kỳ đâu trên cung tại khoảng cách d trong chùm ±half -> lấy các điểm trên cung nằm
    trong hành lang thân xe (|y| ≤ corridor, x > 0); khoảng trống = x nhỏ nhất − mép trước (x lớn nhất của các
    cảm biến). An toàn trước: chưa biết vật ở đâu trên cung thì coi là ở chỗ gần nhất.
"""
from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass, field

NO_ECHO = 0
FAULT = 0xFFFF
_HDR = struct.Struct("<BBH")
_ITEM = struct.Struct("<hhhBH")


@dataclass
class SonarSensor:
    x: float          # m, so với tâm trục bánh, + = tới
    y: float          # m, + = trái
    yaw: float        # rad, + = trái
    half: float       # rad, nửa góc chùm
    stop_mm: int      # ngưỡng dừng khẩn trong firmware (0 = không dùng)


@dataclass
class SonarGeometry:
    sensors: list = field(default_factory=list)
    max_range: float = 2.0

    @classmethod
    def parse(cls, raw: bytes) -> "SonarGeometry":
        ver, n, mr = _HDR.unpack_from(raw, 0)
        if ver != 1 or len(raw) < 4 + n * _ITEM.size:
            raise ValueError("khung SONAR_GEOM không hợp lệ")
        out = []
        for k in range(n):
            x, y, yaw, half, stop = _ITEM.unpack_from(raw, 4 + k * _ITEM.size)
            out.append(SonarSensor(x / 1000.0, y / 1000.0, math.radians(yaw / 100.0), math.radians(half), stop))
        return cls(out, mr / 1000.0)

    def describe(self) -> str:
        return f"{len(self.sensors)} cảm biến, tầm {self.max_range:.1f} m: " + "; ".join(
            f"#{k} ({s.x * 100:.0f}, {s.y * 100:.0f}) cm {math.degrees(s.yaw):+.0f}° ±{math.degrees(s.half):.0f}°"
            for k, s in enumerate(self.sensors))

    def to_json(self) -> list:
        return [[round(s.x, 3), round(s.y, 3), round(math.degrees(s.yaw), 1), round(math.degrees(s.half), 1), s.stop_mm]
                for s in self.sensors]


def front_clearance(geom: SonarGeometry, dists: dict, corridor: float) -> float:
    """dists: {idx: d (m)} chỉ gồm số đo hợp lệ có vang. Trả khoảng trống phía trước (m), inf nếu không thấy gì."""
    if not geom.sensors:
        return math.inf
    front = max(s.x for s in geom.sensors)
    best = math.inf
    for k, d in dists.items():
        if k >= len(geom.sensors) or d is None or d <= 0:
            continue
        s = geom.sensors[k]
        for i in range(9):   # 9 điểm trên cung
            a = s.yaw - s.half + 2 * s.half * i / 8
            px, py = s.x + d * math.cos(a), s.y + d * math.sin(a)
            if px > 0 and abs(py) <= corridor:
                best = min(best, px - front)
    return max(0.0, best) if math.isfinite(best) else math.inf


class SonarState:
    """Số đo mới nhất của từng cảm biến (luồng UART ghi, luồng web / điều khiển đọc)."""

    def __init__(self, fresh_s: float = 0.5):
        self.geom: SonarGeometry | None = None
        self.fresh_s = fresh_s
        self._lock = threading.Lock()
        self._last: dict = {}            # idx -> (t, mm)
        self.blocked_fw: str = ""        # firmware báo SONAR:STOP

    def set_geometry(self, raw: bytes) -> None:
        try:
            g = SonarGeometry.parse(raw)
        except (ValueError, struct.error) as e:
            print("sonar:", e)
            return
        with self._lock:
            self.geom = g
            self._last = {k: v for k, v in self._last.items() if k < len(g.sensors)}
        print("sonar: nhận bảng vị trí từ ESP32:", g.describe())

    def update(self, t: float, idx: int, mm: int) -> None:
        with self._lock:
            self._last[idx] = (t, mm)

    def readings(self, now: float | None = None) -> dict:
        """{idx: (tuổi s, mm)}"""
        now = time.monotonic() if now is None else now
        with self._lock:
            return {k: (now - t, mm) for k, (t, mm) in self._last.items()}

    def clearance(self, corridor: float, now: float | None = None) -> tuple[float, bool]:
        """(khoảng trống m, đủ dữ liệu). Thiếu dữ liệu (chưa có bảng / số đo cũ / cảm biến lỗi) -> (inf, False)."""
        g = self.geom
        if g is None or not g.sensors:
            return math.inf, False
        dists, ok = {}, True
        for k, (age, mm) in self.readings(now).items():
            if age > self.fresh_s or mm == FAULT:
                ok = False
                continue
            if mm != NO_ECHO:
                dists[k] = mm / 1000.0
        if len(self.readings(now)) < len(g.sensors):
            ok = False
        return front_clearance(g, dists, corridor), ok

    def faults(self) -> list:
        return [k for k, (_, mm) in self.readings().items() if mm == FAULT]
