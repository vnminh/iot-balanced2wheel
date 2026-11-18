"""Liên kết UART Raspberry Pi <-> ESP32 (firmware iot-ck-2.0).

ESP32 -> Pi: khung nhị phân  A5 5A <type> <len> <payload> <crc8>
  type 0x01: mẫu IMU + số bước 2 bánh (200 Hz), xem firmware/src/link.h
  type 0x02: text (ST:..., ACK:..., LOG:..., HB:..., PONG)
Pi -> ESP32: dòng text  VEL:<v>,<t>\\n  hoặc  CMD:<TÊN>\\n

Đồng bộ đồng hồ: mỗi mẫu IMU mang micros() của ESP32. Ta quy về
time.monotonic() của Pi (cùng đồng hồ với SensorTimestamp của libcamera) bằng
bộ lọc "biên dưới": offset = min(t_nhận - t_esp) có cho phép trôi chậm lên trên.
Độ trễ tối thiểu (truyền 33 byte) được cộng bù; phần còn lại OpenVINS tự ước
lượng (calib_cam_timeoffset).

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
IMU_FMT = struct.Struct("<I3h3h2iBBH")  # 28 byte
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
                 repeat_hz: float = 10.0):
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.baud = baud
        self.on_imu = on_imu
        self.on_text = on_text
        self.period = 1.0 / repeat_hz
        # 33 byte * 10 bit / baud: thời gian tối thiểu để nhận xong một khung
        self.sync = ClockSync(fixed_latency=33 * 10 / baud)
        self._lock = threading.Lock()
        self._cmd = "VEL:0.000,0.000"
        self._stop = threading.Event()
        self._wrap = 0
        self._last_us = None
        self.last_imu: Optional[ImuSample] = None
        self.robot_state = "UNKNOWN"
        self.stats = {"imu": 0, "crc_err": 0, "text": 0, "dropped_seq": 0}
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

    def _handle_frame(self, ftype: int, payload: bytes, t_rx: float) -> None:
        if ftype == 0x01 and len(payload) == IMU_FMT.size:
            t_us, ax, ay, az, gx, gy, gz, sl, sr, state, flags, seq = IMU_FMT.unpack(payload)
            t_esp = self._unwrap(t_us)
            t = self.sync.update(t_esp, t_rx)
            ka = G / ACC_LSB_PER_G
            kg = math.radians(1.0 / GYRO_LSB_PER_DPS)
            s = ImuSample(t, t_esp, (ax * ka, ay * ka, az * ka), (gx * kg, gy * kg, gz * kg),
                          (sl, sr), state, flags, seq)
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
    link = EspLink(args.port, args.baud, on_text=lambda s: (texts.append(s), print("  <-", s)))
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
                  f"lat={1000 * (time.monotonic() - s.t):.1f}ms")
        else:
            print("chưa nhận được mẫu IMU nào; stats:", link.stats)
    if first and link.last_imu:
        rate = (link.stats["imu"] - first[1]) / max(1e-6, link.last_imu.t_esp - first[0])
        print(f"Tốc độ mẫu IMU: {rate:.1f} Hz")
    link.close()
    return 0 if link.stats["imu"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
