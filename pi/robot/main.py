"""robotd: chạy toàn bộ phía Raspberry Pi.

  ESP32 --UART--> EspLink --IMU+bánh--> VioBridge (ovw_node: OpenVINS + odometry)
  Camera (Picamera2) ---------ảnh----->     |
                                            v
                         Web (aiohttp): bản đồ + quỹ đạo + điều khiển

Chạy:  cd ~/iot-ck/pi && ../.venv/bin/python -m robot.main
Mở:    http://raspberrypi4.local:8080
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import signal
import time
from pathlib import Path

import yaml
from aiohttp import WSMsgType, web

from .esp_link import EspLink, ImuSample

HERE = Path(__file__).resolve().parent
WEB_DIR = HERE.parent / "web"


def resolve(base: Path, p: str) -> str:
    return str((base / p).resolve()) if not os.path.isabs(p) else p


class Robot:
    def __init__(self, cfg: dict, cfg_dir: Path):
        self.cfg = cfg
        self.texts: list[str] = []
        self.drive_cfg = cfg["drive"]
        self.cmd = (0.0, 0.0)
        self.last_drive = 0.0
        self.imu_rate = 0.0
        self._imu_count = 0
        self._rate_t = time.monotonic()

        vc = cfg["vio"]
        self.vio = None
        if vc.get("enable", True):
            from .vio_bridge import VioBridge
            self.vio = VioBridge(resolve(cfg_dir, vc["node"]), resolve(cfg_dir, vc["estimator_config"]),
                                 resolve(cfg_dir, vc["wheel_config"]), vc.get("voxel", 0.05), vc.get("max_points", 200000))

        sc = cfg["serial"]
        self.link = None
        self.link_error = None
        try:
            self.link = EspLink(sc["port"], sc["baud"], on_imu=self._on_imu, on_text=self._on_text)
        except Exception as e:
            self.link_error = f"Không mở được {sc['port']}: {e}"
            print(self.link_error)

        cc = cfg["camera"]
        self.camera = None
        if cc.get("enable", True):
            from .camera import Camera
            self.camera = Camera(cc["width"], cc["height"], cc["fps"], cc.get("sensor_size", (1296, 972)),
                                 cc.get("exposure_us", 0), cc.get("gain", 4.0), cc.get("readout_s", 0.02),
                                 cc.get("extra_shift_s", 0.0), on_frame=self._on_frame)

    # ----------------------------------------------------------- callbacks
    def _on_imu(self, s: ImuSample) -> None:
        self._imu_count += 1
        now = time.monotonic()
        if now - self._rate_t >= 1.0:
            self.imu_rate = self._imu_count / (now - self._rate_t)
            self._imu_count, self._rate_t = 0, now
        if self.vio:
            self.vio.push_imu(s.t, s.acc, s.gyro, s.steps, s.flags)

    def _on_frame(self, t: float, gray) -> None:
        if self.vio:
            self.vio.push_image(t, gray)

    def _on_text(self, text: str) -> None:
        if text.startswith("HB:"):
            return
        self.texts = (self.texts + [f"{time.strftime('%H:%M:%S')} {text}"])[-20:]
        if text.startswith("ST:"):
            self.cmd = (0.0, 0.0)

    # ----------------------------------------------------------- điều khiển
    def drive(self, v: float, t: float) -> None:
        mv, mt = self.drive_cfg["max_speed"], self.drive_cfg["max_turn"]
        v = max(-mv, min(mv, float(v))) if math.isfinite(v) else 0.0
        t = max(-mt, min(mt, float(t))) if math.isfinite(t) else 0.0
        self.cmd = (v, t)
        self.last_drive = time.monotonic()
        if self.link:
            self.link.set_velocity(v, t)

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def watchdog(self) -> None:
        if self.cmd != (0.0, 0.0) and time.monotonic() - self.last_drive > self.drive_cfg["timeout_s"]:
            self.stop()

    def start(self) -> None:
        if self.vio:
            self.vio.start()
        if self.link:
            self.link.start()
            self.link.send_line("INFO")
        if self.camera:
            self.camera.start()

    def shutdown(self) -> None:
        if self.link:
            self.link.close()
        if self.camera:
            self.camera.stop()
        if self.vio:
            self.vio.stop()

    def state(self) -> dict:
        s = self.link.last_imu if self.link else None
        robot = {"link": bool(self.link), "error": self.link_error,
                 "state": self.link.robot_state if self.link else "NO_LINK",
                 "imu_rate": round(self.imu_rate, 1), "cmd": self.cmd, "texts": self.texts[-6:]}
        if s:
            robot.update({"steps": s.steps, "motors": bool(s.flags & 2), "pi_cmd_ok": bool(s.flags & 1),
                          "acc": [round(a, 2) for a in s.acc], "gyro": [round(g, 3) for g in s.gyro],
                          "imu_age": round(time.monotonic() - s.t, 3)})
            if self.link:
                robot["crc_err"] = self.link.stats["crc_err"]
                robot["lost"] = self.link.stats["dropped_seq"]
        cam = None
        if self.camera:
            cam = {"frames": self.camera.frames, "error": self.camera.error, **self.camera.meta}
        return {"type": "state", "robot": robot, "camera": cam, "vio": self.vio.info() if self.vio else None}


# =============================================================== web server
def make_app(robot: Robot) -> web.Application:
    app = web.Application()
    clients: set = set()

    async def index(_):
        return web.FileResponse(WEB_DIR / "index.html")

    async def track_jpg(_):
        p = Path("/dev/shm/ovw_track.jpg")
        if not p.exists():
            raise web.HTTPNotFound()
        return web.Response(body=p.read_bytes(), content_type="image/jpeg",
                            headers={"Cache-Control": "no-store"})

    async def camera_jpg(_):
        cam = robot.camera
        if not cam or cam.latest_gray is None:
            raise web.HTTPNotFound()
        import simplejpeg
        g = cam.latest_gray
        jpg = await asyncio.get_running_loop().run_in_executor(
            None, lambda: simplejpeg.encode_jpeg(g[:, :, None], quality=60, colorspace="GRAY"))
        return web.Response(body=jpg, content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def map_ply(_):
        if not robot.vio:
            raise web.HTTPNotFound()
        return web.Response(body=robot.vio.export_ply(), content_type="application/octet-stream",
                            headers={"Content-Disposition": f"attachment; filename=map_{time.strftime('%Y%m%d_%H%M%S')}.ply"})

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        clients.add(ws)
        if robot.vio:
            await ws.send_json({"type": "snapshot", **robot.vio.snapshot()})
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    m = json.loads(msg.data)
                except ValueError:
                    continue
                kind = m.get("type")
                if kind == "drive":
                    robot.drive(m.get("v", 0.0), m.get("t", 0.0))
                elif kind == "stop":
                    robot.stop()
                elif kind == "reset_vio" and robot.vio:
                    robot.vio.reset()
                    for c in list(clients):
                        await c.send_json({"type": "snapshot", "traj": [], "pts": [], "version": robot.vio.map_version})
        finally:
            clients.discard(ws)
            if not clients:  # người điều khiển rời trang -> dừng xe
                robot.stop()
        return ws

    async def broadcaster(app_):
        while True:
            await asyncio.sleep(0.1)
            robot.watchdog()
            if not clients:
                if robot.vio:
                    robot.vio.take_updates()  # bỏ cập nhật dồn, client mới sẽ lấy snapshot
                continue
            msg = robot.state()
            if robot.vio:
                traj, pts = robot.vio.take_updates()
                msg["traj_new"], msg["pts_new"] = traj, pts
            data = json.dumps(msg, separators=(",", ":"))
            for c in list(clients):
                try:
                    await c.send_str(data)
                except Exception:
                    clients.discard(c)

    async def on_startup(app_):
        app_["bcast"] = asyncio.create_task(broadcaster(app_))

    async def on_cleanup(app_):
        app_["bcast"].cancel()

    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/track.jpg", track_jpg)
    app.router.add_get("/camera.jpg", camera_jpg)
    app.router.add_get("/map.ply", map_ply)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def main() -> int:
    ap = argparse.ArgumentParser(description="robotd: VI-SLAM + web điều khiển")
    ap.add_argument("--config", default=str(HERE.parent / "config" / "robot.yaml"))
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-vio", action="store_true")
    args = ap.parse_args()
    cfg_path = Path(args.config).resolve()
    cfg = yaml.safe_load(cfg_path.read_text())
    if args.no_camera:
        cfg["camera"]["enable"] = False
    if args.no_vio:
        cfg["vio"]["enable"] = False

    robot = Robot(cfg, cfg_path.parent)
    robot.start()
    if robot.vio and robot.vio.status != "running":
        print("VIO:", robot.vio.status)
    app = make_app(robot)

    def _sig(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _sig)
    try:
        web.run_app(app, host=cfg["web"]["host"], port=cfg["web"]["port"], print=lambda s: print(s), handle_signals=False)
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
