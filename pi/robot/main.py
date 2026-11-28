"""robotd: chạy toàn bộ phía Raspberry Pi (vào/ra cho lõi SLAM 2D C++).

  ESP32 --UART--> EspLink --mẫu MCU 200 Hz--> SlamBridge --stdin--> slam_node (pi/slam, C++)
  Camera trần (Picamera2) --ảnh 15 Hz-------->     |                    |
                                                   |<--- JSON stdout ---+  (state, graph, grid, cmd...)
  Web (aiohttp) <---- bản đồ vùng đã đi, keyframe, địa điểm, goto ----+
  Lệnh vận tốc của controller (cmd) -> EspLink -> ESP32
  Speech-control (âm thanh từ ESP32 #2) --WebSocket /ws/voice (chỉ loopback)--> VoiceControl (voice.py), web ưu tiên hơn
  Siêu âm (khung SONAR / SONAR_GEOM từ ESP32) -> SonarState (sonar.py): giảm tốc mọi nguồn lệnh khi có vật phía trước;
                                              -> slam_node (bản ghi S / G): lưới vật cản log-odds, lập lại đường

Chạy:  cd ~/iot-ck/pi && ../.venv/bin/python -m robot.main
Mở:    http://raspberrypi4.local:8080
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import re
import shutil
import signal
import time
from pathlib import Path
from typing import Optional

import yaml
from aiohttp import WSMsgType, web

from .esp_link import EspLink, ImuSample, SonarSample
from .sonar import SonarState

HERE = Path(__file__).resolve().parent
WEB_DIR = HERE.parent / "web"
MAP_FILES = ("meta.json", "grid.png", "grid.yaml", "grid_footprint.png", "places.yaml", "graph.txt",
             "config_snapshot.yaml", "obstacles.png")


def resolve(base: Path, p: str) -> str:
    return str((base / p).resolve()) if not os.path.isabs(p) else p


class Robot:
    def __init__(self, cfg: dict, cfg_dir: Path):
        self.cfg = cfg
        self.texts: list[str] = []
        self.drive_cfg = cfg["drive"]
        self.maps_dir = Path(resolve(cfg_dir, cfg["maps"]["dir"]))
        self.cmd = (0.0, 0.0)
        self.cmd_source = "manual"
        self.last_drive = 0.0
        self.imu_rate = 0.0
        self._imu_count = 0
        self._rate_t = time.monotonic()
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.outbox: Optional[asyncio.Queue] = None   # bản tin từ slam_node -> mọi client web

        sc = cfg["slam"]
        self.slam = None
        if sc.get("enable", True):
            from .slam_bridge import SlamBridge
            log_file = ""
            if sc.get("log_dir"):
                d = Path(resolve(cfg_dir, sc["log_dir"]))
                d.mkdir(parents=True, exist_ok=True)
                log_file = str(d / time.strftime("slam_%Y%m%d_%H%M%S.slamlog"))
            self.slam = SlamBridge(resolve(cfg_dir, sc["node"]), resolve(cfg_dir, sc["config"]), str(self.maps_dir),
                                   sc.get("shm", "/dev/shm/slam"), log_file, sc.get("autoload", ""),
                                   sc.get("autoload_mode", "localization"), on_cmd=self._on_slam_cmd,
                                   on_message=self._on_slam_message)

        self.sonar = SonarState(fresh_s=self.drive_cfg.get("sonar_fresh_s", 0.5))
        self.sonar_cap = None        # giới hạn tốc độ tiến đang áp vì siêu âm (m/s), None = không
        ec = cfg["serial"]
        self.link = None
        self.link_error = None
        if not ec.get("port"):
            self.link_error = "UART đang tắt (--no-serial)"
        else:
            try:
                self.link = EspLink(ec["port"], ec["baud"], on_imu=self._on_imu, on_text=self._on_text,
                                    on_sonar=self._on_sonar, on_sonar_geom=self._on_sonar_geom)
            except Exception as e:
                self.link_error = f"Không mở được {ec['port']}: {e}"
                print(self.link_error)

        self.web_drive_t = 0.0   # lần cuối web gửi lệnh lái tay khác 0 (ưu tiên hơn giọng nói)
        self.voice = None
        vc = cfg.get("voice") or {}
        if vc.get("enable", True):
            from .voice import VoiceCfg, VoiceControl
            geo = {}
            try:
                geo = (yaml.safe_load(Path(resolve(cfg_dir, sc["config"])).read_text()) or {}).get("robot", {})
            except (OSError, yaml.YAMLError) as e:
                print("voice: không đọc được slam.yaml, dùng thông số mặc định:", e)
            steps_per_m = geo.get("steps_per_rev", 200) * geo.get("microsteps", 16) / (math.pi * geo.get("wheel_diameter", 0.065))
            self.voice = VoiceControl(VoiceCfg.from_dict(vc), geo.get("wheel_base", 0.17), 1.0 / steps_per_m,
                                      drive=lambda v, t: self.drive(v, t, source="voice"), emit=self.emit,
                                      robot_ok=self._voice_robot_ok, has_imu=self.link is not None,
                                      front_clear=self.front_clear,
                                      stop_dist=self.drive_cfg.get("sonar_stop", 0.15))

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
        if self.slam:
            self.slam.push_mcu(s)
        if self.voice:
            self.voice.on_imu(s)

    def _on_sonar(self, s: SonarSample) -> None:
        self.sonar.update(s.t, s.idx, s.mm)
        if self.slam:
            self.slam.push_sonar(s)

    def _on_sonar_geom(self, raw: bytes) -> None:
        if self.sonar.geom is None or raw != getattr(self, "_geom_raw", None):
            self._geom_raw = raw
            self.sonar.set_geometry(raw)
        if self.slam:
            self.slam.push_sonar_geom(raw)

    def front_clear(self) -> tuple:
        """(khoảng trống phía trước m, đủ dữ liệu)"""
        return self.sonar.clearance(self.drive_cfg.get("sonar_corridor", 0.15))

    def _on_frame(self, t: float, gray) -> None:
        if self.slam:
            self.slam.push_image(t, gray)

    def _on_text(self, text: str) -> None:
        if text.startswith("HB:"):
            return
        self.texts = (self.texts + [f"{time.strftime('%H:%M:%S')} {text}"])[-20:]
        if text.startswith("SONAR:"):
            self.sonar.blocked_fw = "" if text.startswith("SONAR:CLEAR") else text[6:]
        if text.startswith("ST:"):
            self.cmd = (0.0, 0.0)   # slam_node tự huỷ goto khi thấy xe không còn cân bằng

    def _on_slam_cmd(self, m: dict) -> None:
        """Lệnh vận tốc từ controller của slam_node (luồng đọc stdout)."""
        if m.get("active"):
            self.drive(m.get("v", 0.0), m.get("t", 0.0), source="nav")
        elif self.cmd_source == "nav":
            self.drive(0.0, 0.0, source="nav")

    def _on_slam_message(self, m: dict) -> None:
        if self.loop and self.outbox is not None and m.get("type") in ("graph", "grid", "places", "path", "event", "reply",
                                                                         "obstacles"):
            self.loop.call_soon_threadsafe(self.outbox.put_nowait, m)

    # ----------------------------------------------------------- điều khiển
    def emit(self, level: str, msg: str, voice: bool = True) -> None:
        """Thông báo cho mọi client web (và Speech-control nếu voice=True). Gọi được từ mọi luồng."""
        if self.loop and self.outbox is not None:
            self.loop.call_soon_threadsafe(self.outbox.put_nowait,
                                           {"type": "event", "level": level, "msg": msg, "voice": voice})

    def web_busy(self) -> Optional[str]:
        """Web đang điều khiển -> lý do (lệnh giọng nói bị từ chối)."""
        hold = self.voice.cfg.web_hold_s if self.voice else 2.0
        if self.cmd_source == "manual" and self.cmd != (0.0, 0.0):
            return "web đang lái tay"
        if time.monotonic() - self.web_drive_t < hold:
            return "web vừa lái tay"
        if self.slam and self.slam.nav_active():
            return "xe đang goto / nhiệm vụ từ web"
        return None

    def _voice_robot_ok(self) -> Optional[str]:
        if not self.link:
            return None   # --no-serial / --replay: odometry giả lập từ lệnh (thử web không cần xe)
        s = self.link.last_imu
        if s is None or time.monotonic() - s.t > 0.3:
            return "mất dữ liệu ESP32"
        if self.link.robot_state != "BALANCING":
            return f"xe chưa cân bằng ({self.link.robot_state})"
        return None

    def drive(self, v: float, t: float, source: str = "manual") -> None:
        if source == "manual":   # bấm phím trên web = giành lại quyền lái (từ goto và từ giọng nói)
            if self.slam and self.slam.nav_active():
                self.slam.command("cancel Điều khiển tay")
            if self.voice and self.voice.active:
                self.voice.cancel("web điều khiển tay")
        mv, mt = self.drive_cfg["max_speed"], self.drive_cfg["max_turn"]
        try:
            v, t = float(v), float(t)
        except (TypeError, ValueError):
            v, t = 0.0, 0.0
        v = max(-mv, min(mv, v)) if math.isfinite(v) else 0.0
        t = max(-mt, min(mt, t)) if math.isfinite(t) else 0.0
        # Siêu âm: giảm tốc khi TIẾN theo khoảng trống phía trước (mọi nguồn lệnh: web, goto, giọng nói)
        self.sonar_cap = None
        if v > 0 and self.sonar.geom is not None and self.sonar.geom.sensors:
            stop, slow = self.drive_cfg.get("sonar_stop", 0.15), self.drive_cfg.get("sonar_slow", 0.50)
            clear, ok = self.front_clear()
            cap = mv * max(0.0, min(1.0, (clear - stop) / max(1e-3, slow - stop))) if math.isfinite(clear) else mv
            if not ok:
                cap = min(cap, 0.5 * mv)   # thiếu số đo / cảm biến lỗi: chạy chậm
            if v > cap:
                v, self.sonar_cap = cap, cap
        self.cmd = (v, t)
        self.cmd_source = source
        self.last_drive = time.monotonic()
        if self.link:
            self.link.set_velocity(v, t)

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def watchdog(self) -> None:
        if self.cmd != (0.0, 0.0) and time.monotonic() - self.last_drive > self.drive_cfg["timeout_s"]:
            self.drive(0.0, 0.0, source=self.cmd_source)

    # -------------------------------------------------------- bản đồ
    def save_map(self, name: str, force: bool) -> Optional[str]:
        """Gửi lệnh lưu cho slam_node. Trả về lý do nếu từ chối ngay."""
        if not self.slam:
            return "SLAM đang tắt"
        q = self.slam.quality()
        if not q:
            return "Chưa có trạng thái SLAM"
        if not q.get("ready") and not force:
            return "Bản đồ chưa đạt yêu cầu"
        name = (name or "").strip()[:60] or "ban-do"
        slug = re.sub(r"[^A-Za-z0-9_-]+", "-", name).strip("-")[:30] or "map"
        st = self.slam.state or {}
        mid = st.get("map") or f"{time.strftime('%Y%m%d_%H%M%S')}_{slug}"   # đang mở bản đồ cũ -> lưu đè (mở rộng)
        self.slam.command(f"save_map {mid} {name}")
        return None

    def list_maps(self) -> list:
        out = []
        if self.maps_dir.is_dir():
            for d in sorted(self.maps_dir.iterdir(), reverse=True):
                if d.name.endswith((".tmp", ".old")):
                    continue
                try:
                    m = json.loads((d / "meta.json").read_text())
                except (OSError, ValueError):
                    continue
                if m.get("format", "").startswith("iot-ck-slam2d"):
                    out.append(m)
        return out

    def map_path(self, mid: str, fname: str) -> Optional[Path]:
        if not re.fullmatch(r"[\w-]+", mid) or fname not in MAP_FILES:
            return None
        p = self.maps_dir / mid / fname
        return p if p.is_file() else None

    def delete_map(self, mid: str) -> bool:
        if not re.fullmatch(r"[\w-]+", mid) or not (self.maps_dir / mid).is_dir():
            return False
        shutil.rmtree(self.maps_dir / mid)
        os.sync()
        return True

    def replay(self, path: str, speed: float = 1.0) -> None:
        """Phát lại file .slamlog (MCU + ảnh) vào slam_node theo thời gian thật: chạy web không cần xe.
        Lệnh lái chỉ được ghi nhận, không có xe nào chạy theo."""
        import struct
        import threading

        def run():
            with open(path, "rb") as f:
                if f.read(8) != b"SLAMLOG1":
                    print("replay: không phải file SLAMLOG1")
                    return
                t0_wall = t0_log = None
                while True:
                    hdr = f.read(5)
                    if len(hdr) < 5:
                        break
                    kind, n = hdr[:1], struct.unpack("<I", hdr[1:])[0]
                    data = f.read(n)
                    if kind == b"G":
                        self._on_sonar_geom(data)
                        continue
                    if kind in (b"M", b"I", b"J", b"S") and len(data) >= 8:
                        t = struct.unpack("<d", data[:8])[0]
                        if t0_wall is None:
                            t0_wall, t0_log = time.monotonic(), t
                        wait = (t - t0_log) / speed - (time.monotonic() - t0_wall)
                        if wait > 0:
                            time.sleep(wait)
                        if kind == b"S" and len(data) >= 11:
                            st, idx, mm = struct.unpack("<dBH", data[:11])
                            self.sonar.update(time.monotonic(), idx, mm)
                        if kind == b"M":
                            self.replay_mcu = data
                            self._imu_count += 1
                            now = time.monotonic()
                            if now - self._rate_t >= 1.0:
                                self.imu_rate = self._imu_count / (now - self._rate_t)
                                self._imu_count, self._rate_t = 0, now
                    if kind != b"C" and self.slam:
                        self.slam.push_raw(kind, data)
            print("replay: hết file")
        self.replay_mcu = None
        threading.Thread(target=run, name="replay", daemon=True).start()

    def start(self) -> None:
        if self.slam:
            self.slam.start()
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
        if self.slam:
            self.slam.stop()

    def state(self) -> dict:
        s = self.link.last_imu if self.link else None
        replay = getattr(self, "replay_mcu", None)
        robot = {"link": bool(self.link), "error": None if replay else self.link_error,
                 "state": self.link.robot_state if self.link else ("BALANCING" if replay else "NO_LINK"),
                 "imu_rate": round(self.imu_rate, 1), "cmd": self.cmd, "cmd_source": self.cmd_source,
                 "texts": self.texts[-6:]}
        if s:
            tilt = math.degrees(s.pitch) if s.pitch is not None else math.degrees(math.atan2(-s.acc[0], s.acc[2]))
            robot.update({"steps": s.steps, "motors": bool(s.flags & 2), "pi_cmd_ok": bool(s.flags & 1),
                          "sat": bool(s.flags & 4), "tilt": round(tilt, 1), "fw_pitch": s.pitch is not None,
                          "imu_age": round(time.monotonic() - s.t, 3)})
            if self.link:
                robot["crc_err"] = self.link.stats["crc_err"]
                robot["lost"] = self.link.stats["dropped_seq"]
        cam = None
        if self.camera:
            cam = {"frames": self.camera.frames, "error": self.camera.error, **self.camera.meta}
        sonar = None
        g = self.sonar.geom
        if g is not None:
            rd = self.sonar.readings()
            clear, ok = self.front_clear()
            sonar = {"geom": g.to_json(), "max_range": g.max_range,
                     "mm": [rd[k][1] if k in rd else None for k in range(len(g.sensors))],
                     "age": [round(rd[k][0], 2) if k in rd else None for k in range(len(g.sensors))],
                     "clear": round(clear, 3) if math.isfinite(clear) else None, "ok": ok,
                     "faults": self.sonar.faults(), "fw_stop": self.sonar.blocked_fw,
                     "cap": None if self.sonar_cap is None else round(self.sonar_cap, 3)}
        return {"type": "state", "robot": robot, "camera": cam, "slam": self.slam.info() if self.slam else None,
                "sonar": sonar,
                "voice": self.voice.info(time.monotonic()) if self.voice else None}


# =============================================================== web server
def make_app(robot: Robot) -> web.Application:
    app = web.Application()
    clients: set = set()          # trình duyệt (web)
    voice_clients: set = set()    # Speech-control (/ws/voice)

    async def index(_):
        # no-cache: deploy giao diện mới thì trình duyệt tải lại ngay, không dùng bản cũ
        return web.FileResponse(WEB_DIR / "index.html", headers={"Cache-Control": "no-cache"})

    def _bytes(data: Optional[bytes], ctype: str):
        if data is None:
            raise web.HTTPNotFound()
        return web.Response(body=data, content_type=ctype, headers={"Cache-Control": "no-store"})

    async def grid_png(_):
        return _bytes(robot.slam.grid_png() if robot.slam else None, "image/png")

    async def track_jpg(_):
        return _bytes(robot.slam.frame_jpg() if robot.slam else None, "image/jpeg")

    async def camera_jpg(_):
        cam = robot.camera
        if not cam or cam.latest_gray is None:
            raise web.HTTPNotFound()
        import simplejpeg
        g = cam.latest_gray
        jpg = await asyncio.get_running_loop().run_in_executor(
            None, lambda: simplejpeg.encode_jpeg(g[:, :, None], quality=60, colorspace="GRAY"))
        return web.Response(body=jpg, content_type="image/jpeg", headers={"Cache-Control": "no-store"})

    async def api_maps(_):
        return web.json_response(robot.list_maps())

    async def map_file(request):
        p = robot.map_path(request.match_info["mid"], request.match_info["fname"])
        if not p:
            raise web.HTTPNotFound()
        return web.FileResponse(p, headers={"Cache-Control": "no-cache"})

    async def broadcast(msg: dict, to: Optional[set] = None) -> None:
        to = clients if to is None else to
        data = json.dumps(msg, separators=(",", ":"), ensure_ascii=False)
        for c in list(to):
            try:
                await c.send_str(data)
            except Exception:
                to.discard(c)

    def slam_cmd(line: str) -> Optional[str]:
        if not robot.slam or robot.slam.status != "running":
            return "SLAM chưa chạy"
        robot.slam.command(line)
        return None

    async def ws_handler(request):
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        clients.add(ws)
        if robot.slam:
            for m in robot.slam.snapshot():
                await ws.send_json(m)
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    m = json.loads(msg.data)
                except ValueError:
                    continue
                kind = m.get("type")
                err = None
                name = str(m.get("name", "")).replace("\n", " ").strip()[:60]
                if kind in ("goto", "mission", "goto_place") and robot.voice:
                    robot.voice.cancel("web gửi " + kind)          # web ưu tiên hơn giọng nói
                if kind == "drive":
                    try:
                        if float(m.get("v", 0.0)) or float(m.get("t", 0.0)):
                            robot.web_drive_t = time.monotonic()
                    except (TypeError, ValueError):
                        pass
                    robot.drive(m.get("v", 0.0), m.get("t", 0.0))
                elif kind == "stop":
                    robot.stop()
                elif kind == "goto":
                    try:
                        x, y = float(m["x"]), float(m["y"])
                        th = m.get("theta_deg")
                        line = f"goto {x:.3f} {y:.3f}" + (f" {float(th):.1f}" if th is not None else "")
                        err = slam_cmd(line) if math.isfinite(x) and math.isfinite(y) else "Toạ độ không hợp lệ"
                    except (KeyError, TypeError, ValueError):
                        err = "Toạ độ không hợp lệ"
                elif kind == "mission":
                    try:
                        pts = [(float(q[0]), float(q[1])) for q in (m.get("points") or [])]
                    except (TypeError, ValueError, IndexError):
                        pts = []
                    if not pts or len(pts) > 100 or not all(math.isfinite(a) and math.isfinite(b) for a, b in pts):
                        err = "Nhiệm vụ cần 1–100 điểm hợp lệ"
                    else:
                        err = slam_cmd("mission " + " ".join(f"{a:.3f} {b:.3f}" for a, b in pts))
                elif kind == "goto_place":
                    err = slam_cmd(f"goto_place {name}") if name else "Thiếu tên địa điểm"
                elif kind == "goto_cancel":
                    err = slam_cmd("cancel Đã huỷ")
                elif kind == "nav_clear":
                    err = slam_cmd("nav_clear")
                elif kind == "save_map":
                    err = robot.save_map(name, bool(m.get("force")))
                elif kind == "load_map":
                    mid = str(m.get("id", ""))
                    mode = "mapping" if m.get("mode") == "mapping" else "localization"
                    err = slam_cmd(f"load_map {mid} {mode}") if re.fullmatch(r"[\w-]+", mid) else "Mã bản đồ sai"
                elif kind in ("new_map", "reloc", "calib_ref", "calib_measure"):
                    err = slam_cmd(kind)
                elif kind == "set_mode":
                    err = slam_cmd("set_mode " + ("mapping" if m.get("mode") == "mapping" else "localization"))
                elif kind == "obst_erase":
                    try:
                        x, y, r = float(m["x"]), float(m["y"]), float(m.get("r", 0.2))
                        ok = all(math.isfinite(q) for q in (x, y, r)) and 0 < r < 2
                    except (KeyError, TypeError, ValueError):
                        ok = False
                    err = slam_cmd(f"obst_erase {x:.3f} {y:.3f} {r:.3f}") if ok else "Toạ độ tẩy không hợp lệ"
                elif kind == "obst_clear_temp":
                    err = slam_cmd("obst_clear_temp")
                elif kind in ("save_place", "delete_place"):
                    err = slam_cmd(f"{kind} {name}") if name else "Thiếu tên địa điểm"
                elif kind == "list_maps":
                    await ws.send_json({"type": "maps", "items": robot.list_maps()})
                elif kind == "delete_map":
                    robot.delete_map(str(m.get("id", "")))
                    await ws.send_json({"type": "maps", "items": robot.list_maps()})
                if err:
                    await ws.send_json({"type": "reply", "cmd": kind, "ok": False, "msg": err})
        finally:
            clients.discard(ws)
            if not clients:  # người điều khiển rời trang -> dừng xe (kể cả đang goto), trừ khi đang chạy lệnh giọng nói
                if robot.slam and robot.slam.nav_active():
                    robot.slam.command("cancel Không còn ai mở web")
                if robot.cmd_source != "voice":
                    robot.drive(0.0, 0.0, source="web_gone")
        return ws

    async def forwarder(app_):
        while True:
            m = await robot.outbox.get()
            if m.get("type") == "reply" and m.get("cmd") in ("save_map",) and m.get("ok"):
                await broadcast({"type": "maps", "items": robot.list_maps()})
            if m.pop("voice", False) and voice_clients:
                await broadcast(m, voice_clients)
            if clients:
                await broadcast(m)

    # --------------------------------------------------------- giọng nói (Speech-control)
    async def ws_voice(request):
        """Speech-control gửi lệnh giống web nhưng ở endpoint riêng: robotd biết là giọng nói để xếp ưu tiên.
        Chỉ nhận từ máy này (loopback). Không tính là client web (đóng web không ảnh hưởng)."""
        if request.remote not in ("127.0.0.1", "::1") or not robot.voice:
            raise web.HTTPForbidden(text="voice: chỉ nhận từ 127.0.0.1" if robot.voice else "voice: đang tắt")
        ws = web.WebSocketResponse(heartbeat=5.0)
        await ws.prepare(request)
        vo = robot.voice
        voice_clients.add(ws)
        vo.st.clients = len(voice_clients)
        robot.emit("info", "🎤 Speech-control đã kết nối", voice=False)
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    m = json.loads(msg.data)
                except ValueError:
                    continue
                kind = m.get("type")
                now = time.monotonic()
                if kind == "hello":
                    vo.st.mic = str(m.get("mic", ""))[:40]
                elif kind == "mic":   # trạng thái ESP32 #2 (nguồn âm thanh WebSocket)
                    if m.get("connected"):
                        parts = [str(m.get("device") or "ESP32 #2")[:30]]
                        if m.get("rssi") is not None:
                            parts.append(f"{m['rssi']} dBm")
                        if m.get("level_dbfs") is not None:
                            parts.append(f"{m['level_dbfs']} dBFS")
                        vo.st.mic = " · ".join(parts)
                    else:
                        vo.st.mic = "ESP32 #2 chưa nối"
                elif kind == "listening":
                    vo.st.listening_t, vo.st.partial = now, ""
                elif kind == "partial":
                    vo.st.listening_t, vo.st.partial = now, str(m.get("text", ""))[:120]
                elif kind == "heard":
                    cmds = [str(c)[:40] for c in (m.get("commands") or [])][:8]
                    vo.st.heard = {"text": str(m.get("text", ""))[:120], "normalized": str(m.get("normalized", ""))[:120],
                                   "commands": cmds, "errors": [str(e)[:80] for e in (m.get("errors") or [])][:4],
                                   "time": time.strftime("%H:%M:%S")}
                    vo.st.listening_t, vo.st.partial = 0.0, ""
                    robot.emit("info", f"🎤 Nghe: “{vo.st.heard['text']}”" + (f" → {', '.join(cmds)}" if cmds else ""),
                               voice=False)
                elif kind in ("move", "turn", "stop"):
                    ok, text = vo.submit(m, robot.web_busy())
                    if kind == "stop" and robot.slam and robot.slam.nav_active():
                        robot.slam.command("cancel Lệnh giọng nói: dừng")   # an toàn: dừng cả goto/nhiệm vụ
                    if robot.cmd != (0.0, 0.0) and kind == "stop" and robot.cmd_source != "manual":
                        robot.drive(0.0, 0.0, source="voice")
                    await ws.send_json({"type": "reply", "cmd": kind, "ok": ok, "msg": text})
                    if not ok:
                        robot.emit("warn", "🎤 " + text, voice=False)
                    elif kind == "stop":
                        robot.emit("info", "🎤 " + text, voice=False)
        finally:
            voice_clients.discard(ws)
            vo.st.clients = len(voice_clients)
            if not voice_clients:
                vo.cancel("mất kết nối Speech-control")
        return ws

    async def voice_loop(app_):
        dt = 0.05
        while True:
            await asyncio.sleep(dt)
            try:
                robot.voice.tick(time.monotonic(), dt)
            except Exception as e:   # không để lỗi giết vòng điều khiển; dừng xe cho an toàn
                print("voice tick:", repr(e))
                robot.voice.cancel(f"lỗi {e!r}")

    async def broadcaster(app_):
        while True:
            await asyncio.sleep(0.1)
            robot.watchdog()
            if clients:
                await broadcast(robot.state())

    async def on_startup(app_):
        robot.loop = asyncio.get_running_loop()
        robot.outbox = asyncio.Queue()
        app_["tasks"] = [asyncio.create_task(broadcaster(app_)), asyncio.create_task(forwarder(app_))]
        if robot.voice:
            app_["tasks"].append(asyncio.create_task(voice_loop(app_)))

    async def on_cleanup(app_):
        for t in app_["tasks"]:
            t.cancel()

    app.router.add_get("/", index)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/ws/voice", ws_voice)
    app.router.add_get("/grid.png", grid_png)
    app.router.add_get("/track.jpg", track_jpg)
    app.router.add_get("/camera.jpg", camera_jpg)
    app.router.add_get("/api/maps", api_maps)
    app.router.add_get("/maps/{mid}/{fname}", map_file)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def main() -> int:
    ap = argparse.ArgumentParser(description="robotd: SLAM 2D trần nhà + web điều khiển")
    ap.add_argument("--config", default=str(HERE.parent / "config" / "robot.yaml"))
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-slam", action="store_true")
    ap.add_argument("--no-serial", action="store_true", help="không mở UART (máy không có ESP32)")
    ap.add_argument("--no-voice", action="store_true", help="tắt nhận lệnh giọng nói (/ws/voice)")
    ap.add_argument("--replay", help="phát lại file .slamlog thay cho ESP32 + camera (thử web trên laptop)")
    ap.add_argument("--replay-speed", type=float, default=1.0)
    args = ap.parse_args()
    cfg_path = Path(args.config).resolve()
    cfg = yaml.safe_load(cfg_path.read_text())
    if args.no_camera:
        cfg["camera"]["enable"] = False
    if args.no_slam:
        cfg["slam"]["enable"] = False
    if args.no_voice:
        cfg.setdefault("voice", {})["enable"] = False
    if args.replay:
        cfg["camera"]["enable"] = False
        args.no_serial = True
    if args.no_serial:
        cfg["serial"]["port"] = ""

    robot = Robot(cfg, cfg_path.parent)
    robot.start()
    if args.replay:
        robot.replay(args.replay, args.replay_speed)
    if robot.slam and robot.slam.status != "running":
        print("SLAM:", robot.slam.status)
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
