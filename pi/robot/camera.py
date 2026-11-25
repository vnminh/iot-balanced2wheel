"""Chụp ảnh trần từ camera Pi (OV5647) bằng Picamera2, gắn mốc thời gian cho SLAM 2D.

Mốc thời gian: SensorTimestamp của libcamera (đồng hồ CLOCK_MONOTONIC, ns) là lúc
bắt đầu đọc dòng đầu tiên. Ta lấy thời điểm giữa phơi sáng của dòng GIỮA ảnh:
    t = ts - exposure/2 + readout/2
slam_node nội suy pose odometry và góc chúi đúng thời điểm này.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np


class Camera:
    def __init__(self, width: int = 640, height: int = 480, fps: float = 15.0,
                 sensor_size=(1296, 972), exposure_us: int = 0, gain: float = 4.0,
                 readout_s: float = 0.020, extra_shift_s: float = 0.0,
                 on_frame: Optional[Callable[[float, np.ndarray], None]] = None):
        self.w, self.h, self.fps = width, height, fps
        self.sensor_size = tuple(sensor_size)
        self.exposure_us = exposure_us  # 0 = tự động (chế độ phơi sáng ngắn)
        self.gain = gain
        self.readout_s = readout_s
        self.extra_shift_s = extra_shift_s
        self.on_frame = on_frame
        self.picam2 = None
        self.restarts = 0
        self.latest_gray: Optional[np.ndarray] = None
        self.latest_t = 0.0
        self.frames = 0
        self.error: Optional[str] = None
        self.meta = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="camera", daemon=True)

    def start(self) -> "Camera":
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _open(self):
        from picamera2 import Picamera2
        cam = Picamera2()
        from libcamera import controls as lc
        frame_us = int(1e6 / self.fps)
        ctrl = {"FrameDurationLimits": (frame_us, frame_us)}
        if self.exposure_us > 0:  # phơi sáng cố định: ảnh ổn định nhất cho SLAM
            ctrl.update({"AeEnable": False, "ExposureTime": int(self.exposure_us), "AnalogueGain": float(self.gain)})
        else:  # tự động nhưng ưu tiên thời gian phơi sáng ngắn (ít nhoè khi xe lắc)
            ctrl.update({"AeEnable": True, "AeExposureMode": lc.AeExposureModeEnum.Short})
        cfg = cam.create_video_configuration(
            main={"size": (self.w, self.h), "format": "YUV420"},
            sensor={"output_size": self.sensor_size},
            buffer_count=4,
            controls=ctrl,
        )
        cam.configure(cfg)
        cam.start()
        return cam

    FRAME_TIMEOUT_S = 2.0   # quá lâu không có ảnh = camera treo (vd. sụt áp) -> mở lại

    def _close(self) -> None:
        try:
            self.picam2.stop()
        except Exception:
            pass
        try:
            self.picam2.close()
        except Exception:
            pass
        self.picam2 = None

    def _run(self) -> None:
        self.restarts = 0
        while not self._stop.is_set():
            try:
                self.picam2 = self._open()
            except Exception as e:  # không có camera / bị chương trình khác giữ / vừa treo
                self.error = f"Không mở được camera: {e}"
                print(self.error)
                self.picam2 = None
                self._stop.wait(2.0)
                continue
            self.error = None
            print(f"Camera: {self.w}x{self.h} @ {self.fps:g} fps (sensor {self.sensor_size})")
            self._capture_loop()
            self._close()
            if not self._stop.is_set():
                self.restarts += 1
                self._stop.wait(1.0)

    def _capture_loop(self) -> None:
        """Lấy ảnh tới khi dừng hoặc camera treo (trả về để mở lại)."""
        while not self._stop.is_set():
            try:
                # Không chờ vô hạn: cảm biến treo do sụt nguồn thì capture_request() kẹt mãi
                job = self.picam2.capture_request(wait=False)
                req = self.picam2.wait(job, timeout=self.FRAME_TIMEOUT_S)
            except TimeoutError:
                self.error = f"Camera không ra ảnh {self.FRAME_TIMEOUT_S:g} s (sụt nguồn?), đang mở lại"
                print(self.error)
                return
            except Exception as e:
                self.error = f"Lỗi camera: {e}, đang mở lại"
                print(self.error)
                return
            try:
                yuv = req.make_array("main")
                md = req.get_metadata()
            finally:
                req.release()
            gray = np.ascontiguousarray(yuv[: self.h, : self.w])
            ts = md.get("SensorTimestamp")
            exp_us = md.get("ExposureTime", 0)
            if ts is None:
                t = time.monotonic()
            else:
                t = ts * 1e-9 - exp_us * 0.5e-6 + 0.5 * self.readout_s + self.extra_shift_s
            self.latest_gray, self.latest_t = gray, t
            self.frames += 1
            self.meta = {"exposure_us": exp_us, "gain": round(md.get("AnalogueGain", 0.0), 2),
                         "lux": round(md.get("Lux", 0.0), 1), "restarts": self.restarts}
            if self.on_frame:
                self.on_frame(t, gray)
