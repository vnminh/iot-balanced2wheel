"""Liên kết UART Raspberry Pi <-> ESP32 (firmware iot-ck-2.x).

ESP32 -> Pi: khung nhị phân  A5 5A <type> <len> <payload> <crc8>
  type 0x01: mẫu IMU + số bước 2 bánh (200 Hz), xem firmware/src/link.h
             28 byte (firmware 2.0) hoặc 32 byte (2.1: thêm pitch float, rad, + = chúi tới)
  type 0x02: text (ST:..., ACK:..., LOG:..., HB:..., SONAR:STOP|CLEAR, PONG)
  type 0x03: một số đo siêu âm (firmware 2.2): u32 t_us, u8 idx, u16 mm (0 = không vang, 0xFFFF = lỗi)
  type 0x04: bảng vị trí cảm biến siêu âm (firmware 2.2), xem robot/sonar.py. Pi KHÔNG có cấu hình vị trí
             cảm biến: dùng đúng bảng ESP32 gửi.
Pi -> ESP32: dòng text  VEL:<v>,<t>\\n  hoặc  CMD:<TÊN>\\n

Đồng bộ đồng hồ: mỗi mẫu IMU mang micros() của ESP32. Ta quy về
time.monotonic() của Pi (cùng đồng hồ với SensorTimestamp của libcamera) bằng
bộ lọc "biên dưới": offset = min(t_nhận - t_esp) có cho phép trôi chậm lên trên.
Độ trễ tối thiểu (truyền một khung) được cộng bù.

Chạy thử:  python3 -m robot.esp_link --port /dev/serial0
"""
from __future__ import annotations

import argparse
import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

import serial

ACC_LSB_PER_G = 8192.0
GYRO_LSB_PER_DPS = 32.8
G = 9.80665
IMU_FMT = struct.Struct("<I3h3h2iBBH")     # 28 byte, firmware 2.0
IMU_FMT_P = struct.Struct("<I3h3h2iBBHf")  # 32 byte, firmware 2.1 (+ pitch)
SONAR_FMT = struct.Struct("<IBH")          # 7 byte, firmware 2.2
STATE_NAMES = {0: "CALIBRATING", 1: "WAITING", 2: "BALANCING"}


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


_CRC_TABLE = [crc8(bytes([i])) for i in range(256)]


def crc8_fast(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[crc ^ b]
    return crc


@dataclass
class ImuSample:
    t: float               # giây, đồng hồ time.monotonic() của Pi
    t_esp: float           # giây, đồng hồ ESP32 (đã gỡ tràn 32 bit)
    acc: tuple             # m/s^2, khung cảm biến
    gyro: tuple            # rad/s, khung cảm biến
    steps: tuple           # (trái, phải) tổng số bước, + = lăn tới
    state: int
    flags: int
    seq: int
    pitch: Optional[float] = None   # rad, + = chúi tới (firmware 2.1); None = firmware cũ


@dataclass
class SonarSample:
    t: float               # giây, đồng hồ Pi (lúc ECHO kết thúc)
    t_esp: float
    idx: int               # chỉ số cảm biến (theo bảng của ESP32)
    mm: int                # 0 = không có vang trong tầm, 0xFFFF = cảm biến lỗi


class ClockSync:
    """Quy đổi đồng hồ ESP32 sang đồng hồ Pi bằng biên dưới của độ trễ."""

    def __init__(self, creep_ppm: float = 100.0, fixed_latency: float = 0.0):
        self.offset: Optional[float] = None
        self.creep = creep_ppm * 1e-6
        self.fixed_latency = fixed_latency
        self._last_esp = None

    def update(self, t_esp: float, t_rx: float) -> float:
        d = t_rx - t_esp
        if self.offset is None or self._last_esp is None:
            self.offset = d
        else:
            dt = max(0.0, t_esp - self._last_esp)
            # Offset chỉ được tăng chậm (bù lệch tần số thạch anh), giảm thì ngay
            self.offset = min(d, self.offset + self.creep * dt + 2e-6)
            if d - self.offset > 0.5:  # ESP32 reset / lệch lớn -> đặt lại
                self.offset = d
        self._last_esp = t_esp
        return t_esp + self.offset - self.fixed_latency


class EspLink:
    def __init__(self, port: str = "/dev/serial0", baud: int = 460800,
                 on_imu: Optional[Callable[[ImuSample], None]] = None,
                 on_text: Optional[Callable[[str], None]] = None,
                 on_sonar: Optional[Callable[[SonarSample], None]] = None,
                 on_sonar_geom: Optional[Callable[[bytes], None]] = None,
                 repeat_hz: float = 10.0):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.baud = baud
        self.on_imu = on_imu
        self.on_text = on_text
        self.on_sonar = on_sonar
        self.on_sonar_geom = on_sonar_geom
        self.sonar_geom_raw: Optional[bytes] = None   # khung SONAR_GEOM gần nhất (robot/sonar.py đọc)
        self.period = 1.0 / repeat_hz
        # 37 byte * 10 bit / baud: thời gian tối thiểu để nhận xong một khung IMU (32 byte + 5)
        self.sync = ClockSync(fixed_latency=37 * 10 / baud)
        self._lock = threading.Lock()
        self._cmd = "VEL:0.000,0.000"
        self._stop = threading.Event()
        self._wrap = 0
        self._last_us = None
        self.last_imu: Optional[ImuSample] = None
        self.robot_state = "UNKNOWN"
        self.stats = {"imu": 0, "crc_err": 0, "text": 0, "dropped_seq": 0, "sonar": 0}
        self._last_seq = None
        self._threads = [threading.Thread(target=self._rx_loop, name="esp-rx", daemon=True),
                         threading.Thread(target=self._tx_loop, name="esp-tx", daemon=True)]

    # ------------------------------------------------------------------ API
    def start(self) -> "EspLink":
        for t in self._threads:
            t.start()
        return self

    def close(self) -> None:
        self.set_velocity(0.0, 0.0)
        time.sleep(self.period * 2)
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        self.ser.close()

    def set_velocity(self, v: float, turn: float) -> None:
        """v: m/s tiến (+), turn: m/s chênh lệch mỗi bánh (+ = quay phải)."""
        if not (math.isfinite(v) and math.isfinite(turn)):
            v, turn = 0.0, 0.0
        with self._lock:
            self._cmd = f"VEL:{v:.3f},{turn:.3f}"
        self._send_now()

    def send_line(self, line: str) -> None:
        with self._lock:
            self.ser.write((line.strip() + "\n").encode("ascii"))

    # ------------------------------------------------------------ internals
    def _send_now(self) -> None:
        with self._lock:
            self.ser.write((self._cmd + "\n").encode("ascii"))

    def _tx_loop(self) -> None:
        # Gửi lại lệnh hiện hành đều đặn: vừa là nhịp tim vừa tự sửa khi mất gói
        while not self._stop.wait(self.period):
            try:
                self._send_now()
            except Exception as e:  # cổng bị rút
                print("esp_link: lỗi gửi:", e)
                return

    def _unwrap(self, t_us: int) -> float:
        if self._last_us is not None and t_us < self._last_us and self._last_us - t_us > 2**31:
            self._wrap += 1
        self._last_us = t_us
        return (self._wrap * 2**32 + t_us) * 1e-6

    def _unwrap_peek(self, t_us: int) -> float:
        """Như _unwrap nhưng không cập nhật trạng thái (số đo siêu âm có thể cũ hơn mẫu IMU vừa nhận)."""
        wrap = self._wrap
        if self._last_us is not None and t_us > self._last_us and t_us - self._last_us > 2**31:
            wrap -= 1   # mốc trước lần tràn
        elif self._last_us is not None and t_us < self._last_us and self._last_us - t_us > 2**31:
            wrap += 1
        return (wrap * 2**32 + t_us) * 1e-6

    def _handle_frame(self, ftype: int, payload: bytes, t_rx: float) -> None:
        if ftype == 0x01 and len(payload) in (IMU_FMT.size, IMU_FMT_P.size):
            pitch = None
            if len(payload) == IMU_FMT_P.size:
                t_us, ax, ay, az, gx, gy, gz, sl, sr, state, flags, seq, pitch = IMU_FMT_P.unpack(payload)
            else:
                t_us, ax, ay, az, gx, gy, gz, sl, sr, state, flags, seq = IMU_FMT.unpack(payload)
            t_esp = self._unwrap(t_us)
            t = self.sync.update(t_esp, t_rx)
            ka = G / ACC_LSB_PER_G
            kg = math.radians(1.0 / GYRO_LSB_PER_DPS)
            s = ImuSample(t, t_esp, (ax * ka, ay * ka, az * ka), (gx * kg, gy * kg, gz * kg),
                          (sl, sr), state, flags, seq, pitch)
            if self._last_seq is not None:
                lost = (seq - self._last_seq - 1) & 0xFFFF
                if lost < 1000:
                    self.stats["dropped_seq"] += lost
            self._last_seq = seq
            self.stats["imu"] += 1
            self.last_imu = s
            self.robot_state = STATE_NAMES.get(state, str(state))
            if self.on_imu:
                self.on_imu(s)
        elif ftype == 0x03 and len(payload) == SONAR_FMT.size:
            t_us, idx, mm = SONAR_FMT.unpack(payload)
            t_esp = self._unwrap_peek(t_us)
            off = self.sync.offset if self.sync.offset is not None else t_rx - t_esp
            ss = SonarSample(t_esp + off - self.sync.fixed_latency, t_esp, idx, mm)
            self.stats["sonar"] += 1
            if self.on_sonar:
                self.on_sonar(ss)
        elif ftype == 0x04 and len(payload) >= 4:
            self.sonar_geom_raw = payload
            if self.on_sonar_geom:   # mỗi lần nhận (2 s/lần): slam_node khởi động lại cũng có bảng
                self.on_sonar_geom(payload)
        elif ftype == 0x02:
            text = payload.decode("utf-8", "replace")
            self.stats["text"] += 1
            if text.startswith("ST:"):
                # Xe ngã / ESP32 khởi động lại: về đứng yên, KHÔNG gửi lại lệnh cũ
                with self._lock:
                    self._cmd = "VEL:0.000,0.000"
            if self.on_text:
                self.on_text(text)

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting))
            except Exception as e:
                print("esp_link: lỗi đọc:", e)
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            t_rx = time.monotonic()
            buf += chunk
            while True:
                i = buf.find(b"\xa5\x5a")
                if i < 0:
                    del buf[:-1]
                    break
                if i:
                    del buf[:i]
                if len(buf) < 5:
                    break
                n = buf[3]
                if len(buf) < 5 + n:
                    break
                body = bytes(buf[2:4 + n])
                if crc8_fast(body) == buf[4 + n]:
                    # nhiều khung tới cùng lúc: khung cũ hơn tới sớm hơn ~ (khung còn lại)*thời gian 1 khung
                    remaining = len(buf) - (5 + n)
                    self._handle_frame(buf[2], bytes(buf[4:4 + n]), t_rx - remaining * 10 / self.baud)
                    del buf[:5 + n]
                else:
                    self.stats["crc_err"] += 1
                    del buf[:2]


def main() -> int:
    ap = argparse.ArgumentParser(description="Kiểm tra luồng dữ liệu ESP32")
    ap.add_argument("--port", default="/dev/serial0")
    ap.add_argument("--baud", type=int, default=460800)
    ap.add_argument("--secs", type=float, default=5.0)
    args = ap.parse_args()

    texts = []
    sonar_last = {}
    link = EspLink(args.port, args.baud, on_text=lambda s: (texts.append(s), print("  <-", s)),
                   on_sonar=lambda s: sonar_last.__setitem__(s.idx, s.mm))
    link.start()
    link.send_line("INFO")
    t0 = time.monotonic()
    first = None
    while time.monotonic() - t0 < args.secs:
        time.sleep(1.0)
        s = link.last_imu
        if s:
            first = first or (s.t_esp, link.stats["imu"])
            print(f"imu={link.stats['imu']:5d} crc_err={link.stats['crc_err']} mất={link.stats['dropped_seq']} "
                  f"state={link.robot_state} acc=({s.acc[0]:+.2f},{s.acc[1]:+.2f},{s.acc[2]:+.2f}) "
                  f"gyro=({s.gyro[0]:+.3f},{s.gyro[1]:+.3f},{s.gyro[2]:+.3f}) steps={s.steps} "
                  f"pitch={'-' if s.pitch is None else f'{math.degrees(s.pitch):+.1f}°'} "
                  f"lat={1000 * (time.monotonic() - s.t):.1f}ms")
        else:
            print("chưa nhận được mẫu IMU nào; stats:", link.stats)
        if link.sonar_geom_raw:
            from .sonar import SonarGeometry
            g = SonarGeometry.parse(link.sonar_geom_raw)
            print(f"  siêu âm: {g.describe()} | đo (mm): "
                  + ", ".join(f"#{k}={'lỗi' if v == 0xFFFF else ('trống' if v == 0 else v)}" for k, v in sorted(sonar_last.items())))
        else:
            print("  siêu âm: chưa nhận bảng vị trí (firmware < 2.2?)")
    if first and link.last_imu:
        rate = (link.stats["imu"] - first[1]) / max(1e-6, link.last_imu.t_esp - first[0])
        print(f"Tốc độ mẫu IMU: {rate:.1f} Hz")
    link.close()
    return 0 if link.stats["imu"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
