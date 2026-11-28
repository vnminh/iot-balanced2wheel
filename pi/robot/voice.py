"""Chạy lệnh giọng nói trong robotd: tiến/lùi N cm, xoay trái/phải N độ, dừng.

Lệnh tới từ Speech-control qua WebSocket /ws/voice (chỉ nhận từ 127.0.0.1). Đo quãng đường bằng số bước
trung bình hai bánh, góc bằng gyro (đã bù nghiêng thân xe và bias ước lượng lúc đứng yên), vòng kín ở 20 Hz.
Không cần bản đồ / định vị SLAM.

Ưu tiên (WEB > giọng nói):
  - Web đang lái tay (có lệnh trong `web_hold_s` gần đây) hoặc goto/mission đang chạy -> từ chối lệnh giọng nói.
  - Web gửi lái tay / dừng / goto / mission -> huỷ ngay lệnh giọng nói đang chạy và hàng đợi.
  - "dừng" bằng giọng nói: huỷ lệnh giọng nói và cả goto/mission (an toàn), không chặn được phím web đang giữ.
Vật cản: lệnh TIẾN gặp vật trong khoảng stop_dist phía trước (siêu âm) thì dừng ngay và báo quãng đã đi.
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional


@dataclass
class VoiceCfg:
    v_max: float = 0.10          # m/s
    v_min: float = 0.03
    acc: float = 0.25            # m/s² (tăng tốc và hãm)
    w_max: float = 0.8           # rad/s
    w_min: float = 0.25
    acc_w: float = 2.0           # rad/s²
    tol_m: float = 0.005
    tol_deg: float = 2.0
    heading_kp: float = 2.0      # giữ hướng khi tiến/lùi
    max_distance_m: float = 3.0
    max_angle_deg: float = 720.0
    queue_max: int = 5
    web_hold_s: float = 2.0      # web lái tay trong khoảng này -> giọng nói bị từ chối
    settle_s: float = 0.4        # nghỉ giữa hai lệnh
    stall_s: float = 3.0         # không tiến triển trong khoảng này -> huỷ (kẹt?)
    lag_s: float = 0.15          # trễ đáp ứng của xe: dừng sớm theo vị trí dự đoán (đỡ vượt đích)
    turn_source: str = "gyro"    # gyro | wheels

    @classmethod
    def from_dict(cls, d: Optional[dict]) -> "VoiceCfg":
        c = cls()
        for k, v in (d or {}).items():
            if hasattr(c, k) and k != "enable":
                setattr(c, k, type(getattr(c, k))(v))
        return c


@dataclass
class Motion:
    op: str                      # move | turn
    target: float                # m (+ tiến) | rad (+ trái)
    text: str = ""
    start_dist: float = 0.0
    start_yaw: float = 0.0
    t0: float = 0.0
    best: float = 1e9
    best_t: float = 0.0
    started: bool = False

    def describe(self) -> str:
        if self.op == "move":
            return f"{'Tiến' if self.target >= 0 else 'Lùi'} {abs(self.target) * 100:.0f} cm"
        return f"Xoay {'trái' if self.target >= 0 else 'phải'} {abs(math.degrees(self.target)):.0f}°"


@dataclass
class VoiceState:
    clients: int = 0
    mic: str = ""
    listening_t: float = 0.0
    partial: str = ""
    heard: dict = field(default_factory=dict)


class VoiceControl:
    """drive(v, t): lệnh vận tốc cho firmware (t = m/s chênh lệch mỗi bánh, + = quay phải).
    emit(level, msg): thông báo cho web + Speech-control. robot_ok() -> lý do không chạy được (hoặc None)."""

    def __init__(self, cfg: VoiceCfg, wheel_base: float, m_per_step: float,
                 drive: Callable[[float, float], None], emit: Callable[[str, str], None],
                 robot_ok: Callable[[], Optional[str]], has_imu: bool,
                 front_clear: Optional[Callable[[], tuple]] = None, stop_dist: float = 0.15):
        self.cfg = cfg
        self.b = wheel_base
        self.mps = m_per_step
        self._drive = drive
        self.emit = emit
        self.robot_ok = robot_ok
        self.has_imu = has_imu
        self.front_clear = front_clear
        self.stop_dist = stop_dist
        if not has_imu:
            self.cfg.lag_s = 0.0               # odometry giả lập = lệnh, không có trễ
        self.st = VoiceState()
        self.queue: deque[Motion] = deque()
        self.cur: Optional[Motion] = None
        self.done_t = 0.0
        self.v = self.w = 0.0                  # lệnh hiện tại (sau giới hạn gia tốc), w + = trái
        self.last_result = ""
        self._report = None                    # (motion, dist0, yaw0): báo kết quả đo khi xe đã dừng hẳn
        self._prev_odom = None
        self.vel_d = self.vel_y = 0.0          # tốc độ đo được (m/s, rad/s)
        # odometry (luồng UART ghi, vòng điều khiển đọc)
        self._lock = threading.Lock()
        self._dist = 0.0
        self._yaw_gyro = 0.0
        self._yaw_wheel = 0.0
        self._bias = 0.0
        self._last_steps = None
        self._last_t = None
        self._still_t = 0.0
        self._sim_t = None

    # ---------------------------------------------------------------- odometry
    def on_imu(self, s) -> None:
        """Mẫu IMU 200 Hz (esp_link.ImuSample)."""
        pitch = s.pitch if s.pitch is not None else math.atan2(-s.acc[0], s.acc[2])
        rate = -math.sin(pitch) * s.gyro[0] + math.cos(pitch) * s.gyro[2]   # tốc độ quay quanh trục đứng, + = trái
        with self._lock:
            if self._last_t is None:
                self._last_t, self._last_steps = s.t, s.steps
                return
            dt = min(max(s.t - self._last_t, 0.0), 0.05)
            dl = (s.steps[0] - self._last_steps[0]) * self.mps
            dr = (s.steps[1] - self._last_steps[1]) * self.mps
            self._last_t, self._last_steps = s.t, s.steps
            if abs(dl) > 0.05 or abs(dr) > 0.05:      # ESP32 khởi động lại -> bỏ bước nhảy
                return
            self._dist += 0.5 * (dl + dr)
            self._yaw_wheel += (dr - dl) / self.b
            # bias gyro: chỉ học khi xe không có lệnh và bánh gần như đứng yên
            idle = self.cur is None and abs(dl) + abs(dr) < 4e-4 and abs(rate) < 0.05
            self._still_t = self._still_t + dt if idle else 0.0
            if self._still_t > 0.5:
                self._bias += min(dt / 2.0, 1.0) * (rate - self._bias)
            self._yaw_gyro += (rate - self._bias) * dt

    def _sim_step(self, now: float) -> None:
        """Không có ESP32 (--no-serial / --replay): giả lập odometry từ lệnh để thử web."""
        if self._sim_t is not None:
            dt = min(now - self._sim_t, 0.2)
            with self._lock:
                self._dist += self.v * dt
                self._yaw_gyro += self.w * dt
                self._yaw_wheel += self.w * dt
        self._sim_t = now

    def odom(self) -> tuple[float, float]:
        with self._lock:
            return self._dist, (self._yaw_gyro if self.cfg.turn_source != "wheels" else self._yaw_wheel)

    # ---------------------------------------------------------------- lệnh
    def submit(self, msg: dict, web_busy: Optional[str]) -> tuple[bool, str]:
        kind = msg.get("type")
        text = str(msg.get("text", ""))[:120]
        if kind == "stop":
            n = self.cancel("Lệnh giọng nói: dừng", quiet=True)
            return True, "Đã dừng" + (f" ({n} lệnh giọng nói bị huỷ)" if n else "")
        if web_busy:
            return False, f"Bỏ qua lệnh giọng nói: {web_busy} (web được ưu tiên)"
        why = self.robot_ok()
        if why:
            return False, f"Bỏ qua lệnh giọng nói: {why}"
        try:
            if kind == "move":
                val = float(msg["distance_m"])
                if not (math.isfinite(val) and 0.005 <= abs(val) <= self.cfg.max_distance_m):
                    return False, f"Quãng đường phải 0,5–{self.cfg.max_distance_m * 100:.0f} cm"
                m = Motion("move", val, text)
            elif kind == "turn":
                val = float(msg["angle_deg"])
                if not (math.isfinite(val) and 1.0 <= abs(val) <= self.cfg.max_angle_deg):
                    return False, f"Góc phải 1–{self.cfg.max_angle_deg:.0f}°"
                m = Motion("turn", math.radians(val), text)
            else:
                return False, f"Lệnh lạ: {kind}"
        except (KeyError, TypeError, ValueError):
            return False, "Lệnh sai định dạng"
        if len(self.queue) + (self.cur is not None) >= self.cfg.queue_max:
            return False, f"Hàng đợi đầy ({self.cfg.queue_max} lệnh), nói 'dừng' để xoá"
        self.queue.append(m)
        busy = self.cur is not None or len(self.queue) > 1
        return True, m.describe() + (" (xếp hàng)" if busy else "")

    def cancel(self, reason: str, quiet: bool = False) -> int:
        """Huỷ lệnh đang chạy + hàng đợi. Trả số lệnh bị huỷ."""
        n = len(self.queue) + (self.cur is not None)
        was_moving = self.cur is not None
        self.queue.clear()
        if self.cur is not None:
            self.last_result = f"Huỷ: {self.cur.describe()} ({reason})"
            self.cur = None
        self.v = self.w = 0.0
        if was_moving:
            self._drive(0.0, 0.0)
        if n and not quiet:
            self.emit("warn", f"🎤 Huỷ {n} lệnh giọng nói: {reason}")
        return n

    @property
    def active(self) -> bool:
        return self.cur is not None or bool(self.queue)

    # ---------------------------------------------------------------- vòng điều khiển 20 Hz
    def tick(self, now: float, dt: float) -> None:
        if not self.has_imu:
            self._sim_step(now)
        dist, yaw = self.odom()
        if self._prev_odom is not None and dt > 0:
            self.vel_d += 0.5 * ((dist - self._prev_odom[0]) / dt - self.vel_d)
            self.vel_y += 0.5 * ((yaw - self._prev_odom[1]) / dt - self.vel_y)
        self._prev_odom = (dist, yaw)
        if self.cur is None:
            if now - self.done_t < self.cfg.settle_s:
                return
            if self._report:
                m, d0, y0 = self._report
                self._report = None
                meas = f"{(dist - d0) * 100:.1f} cm" if m.op == "move" else f"{math.degrees(yaw - y0):.1f}°"
                self.last_result = f"Xong: {m.describe()} (đo được {meas})"
                self.emit("info", "🎤 " + self.last_result)
            if not self.queue:
                return
            self.cur = self.queue.popleft()
        m, c = self.cur, self.cfg
        why = self.robot_ok()
        if why:
            self._finish(f"Dừng lệnh giọng nói: {why}", "warn", now)
            return
        if not m.started:
            m.started, m.start_dist, m.start_yaw, m.t0, m.best_t = True, dist, yaw, now, now
            self.emit("info", f"🎤 {m.describe()}")
        # còn lại tính theo vị trí DỰ ĐOÁN sau lag_s (xe còn trôi tiếp sau khi lệnh về 0)
        if m.op == "move":
            rem = m.target - (dist - m.start_dist + self.vel_d * c.lag_s)
            ok = abs(rem) < 0.3 * c.tol_m or rem * m.target < 0      # sát đích hoặc (dự đoán) đã qua đích
            unit_tol = 0.005
        else:
            rem = m.target - (yaw - m.start_yaw + self.vel_y * c.lag_s)
            ok = abs(rem) < math.radians(0.3 * c.tol_deg) or rem * m.target < 0
            unit_tol = math.radians(2.0)
        prog = abs(rem)
        if ok:
            self._report = (m, m.start_dist, m.start_yaw)
            self._finish("", "info", now)
            return
        # không tiến triển -> kẹt / trượt
        if m.op == "move" and m.target > 0 and self.front_clear is not None:
            clear, _ = self.front_clear()
            if clear < self.stop_dist + 0.01:
                self._finish(f"Dừng lệnh giọng nói: vật cản phía trước {clear * 100:.0f} cm "
                             f"(đã đi {(dist - m.start_dist) * 100:.0f}/{m.target * 100:.0f} cm)", "warn", now)
                return
        if prog < m.best - unit_tol:
            m.best, m.best_t = prog, now
        elif now - m.best_t > c.stall_s:
            self._finish(f"Dừng lệnh giọng nói: {m.describe()} không tiến triển {c.stall_s:.0f} s (kẹt?)", "warn", now)
            return
        # profile vận tốc: hãm kịp trước đích (v = √(2·a·còn lại)), giới hạn gia tốc
        if m.op == "move":
            v = math.copysign(max(c.v_min, min(c.v_max, math.sqrt(2 * c.acc * abs(rem)))), rem)
            yaw_err = (yaw - m.start_yaw)
            w = max(-0.5, min(0.5, -c.heading_kp * yaw_err))          # giữ thẳng
        else:
            v = 0.0
            w = math.copysign(max(c.w_min, min(c.w_max, math.sqrt(2 * c.acc_w * abs(rem)))), rem)
        self.v += max(-c.acc * dt, min(c.acc * dt, v - self.v))
        self.w += max(-c.acc_w * dt, min(c.acc_w * dt, w - self.w))
        # đổi chiều giữa chừng (vượt đích) thì không đợi giảm tốc qua 0 chậm chạp
        if m.op == "move" and self.v * rem < 0 and abs(rem) < 0.02:
            self.v = math.copysign(c.v_min, rem)
        self._drive(self.v, -self.w * self.b / 2.0)

    def _finish(self, msg: str, level: str, now: float) -> None:
        self.last_result = msg
        self.cur = None
        self.v = self.w = 0.0
        self._drive(0.0, 0.0)
        self.done_t = now
        if level != "info":
            self.queue.clear()          # lỗi -> bỏ các lệnh còn lại (không chạy tiếp khi đã lệch)
        if msg:
            self.emit(level, msg)

    # ---------------------------------------------------------------- web
    def info(self, now: float) -> dict:
        st = self.st
        cur = None
        if self.cur is not None and self.cur.started:
            dist, yaw = self.odom()
            done = (dist - self.cur.start_dist) * 100 if self.cur.op == "move" \
                else math.degrees(yaw - self.cur.start_yaw)
            cur = {"desc": self.cur.describe(), "done": round(done, 1),
                   "target": round(self.cur.target * 100 if self.cur.op == "move" else math.degrees(self.cur.target), 1),
                   "unit": "cm" if self.cur.op == "move" else "°"}
        return {"connected": st.clients > 0, "mic": st.mic,
                "listening": now - st.listening_t < 4.0 and bool(st.listening_t),
                "partial": st.partial if now - st.listening_t < 4.0 else "",
                "heard": st.heard, "current": cur, "queue": [m.describe() for m in self.queue],
                "last": self.last_result, "bias_dps": round(math.degrees(self._bias), 2)}
