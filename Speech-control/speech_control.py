#!/usr/bin/env python3
"""Điều khiển xe cân bằng bằng giọng nói tiếng Việt (ESP32 #2 + INMP441 -> Zipformer -> robotd).

  ESP32 #2 (INMP441) --WiFi WebSocket ws://pi:8765/audio (Pi là server)--> realtime_asr_streaming (VAD + Zipformer)
          --> robot_commands (từ điển sửa lỗi + tách lệnh)
          --> WebSocket ws://127.0.0.1:8080/ws/voice --> robotd (pi/robot/voice.py chạy lệnh bằng odometry)
  --source mic: đọc mic nối thẳng vào Pi thay cho ESP32 #2.

Lệnh:  "tiến 10" (cm)   "lùi 20"   "xoay trái 90" (độ)   "xoay phải 180"   "dừng"
robotd ưu tiên WEB: đang lái / goto trên web thì lệnh giọng nói bị từ chối; bấm phím trên web huỷ lệnh giọng nói.

  python speech_control.py                 # nghe mic (dịch vụ voice.service chạy lệnh này)
  python speech_control.py --keyboard      # gõ lệnh thay cho nói (thử không cần mic)
  python speech_control.py --text "tiến 10"
  python speech_control.py --dry-run       # nghe mic, chỉ in lệnh, KHÔNG gửi cho xe
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
import threading
import time

import realtime_asr_streaming as asr
from robot_commands import parse

USE_COLOR = True


class C:
    RESET, BOLD, DIM = "\033[0m", "\033[1m", "\033[2m"
    RED, GREEN, YELLOW, MAGENTA, CYAN, WHITE = "\033[91m", "\033[92m", "\033[93m", "\033[95m", "\033[96m", "\033[37m"


def color(text: str, *codes: str) -> str:
    return "".join(codes) + text + C.RESET if USE_COLOR else text


def log(tag: str, msg: str, *codes: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {color(tag, C.BOLD, *codes)}  {msg}", flush=True)


# ------------------------------------------------------------------ kết nối robotd

class RobotLink:
    """WebSocket tới robotd chạy trong luồng riêng, tự nối lại. send() gọi được từ mọi luồng."""

    def __init__(self, url: str, hello: dict):
        self.url = url
        self.hello = hello
        self.connected = threading.Event()
        self.loop = asyncio.new_event_loop()
        self.queue: asyncio.Queue | None = None
        self.thread = threading.Thread(target=self._run, name="robotd-ws", daemon=True)
        self._warned = False
        self.accepted = 0          # lệnh move/turn robotd đã nhận
        self.finished = 0          # lệnh đã xong / bị huỷ (dùng cho --text: chờ xe chạy xong mới thoát)

    def start(self) -> "RobotLink":
        self.thread.start()
        return self

    def send(self, msg: dict) -> None:
        if self.queue is not None:
            self.loop.call_soon_threadsafe(self._put, msg)

    def _put(self, msg: dict) -> None:
        if self.queue.qsize() < 50:
            self.queue.put_nowait(msg)

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.queue = asyncio.Queue()
        self.loop.run_until_complete(self._main())

    async def _main(self) -> None:
        import aiohttp
        async with aiohttp.ClientSession() as session:
            while True:
                try:
                    async with session.ws_connect(self.url, heartbeat=5.0) as ws:
                        self.connected.set()
                        self._warned = False
                        log("🔗 ROBOTD", f"đã nối {self.url}", C.GREEN)
                        await ws.send_json(self.hello)
                        sender = asyncio.create_task(self._sender(ws))
                        try:
                            async for m in ws:
                                if m.type == aiohttp.WSMsgType.TEXT:
                                    self._on_message(json.loads(m.data))
                        finally:
                            sender.cancel()
                except (aiohttp.ClientError, OSError, asyncio.TimeoutError) as e:
                    if not self._warned:
                        log("🔗 ROBOTD", f"chưa nối được {self.url} ({e.__class__.__name__}), thử lại mỗi 2 s",
                            C.YELLOW)
                        self._warned = True
                if self.connected.is_set():
                    log("🔗 ROBOTD", "mất kết nối", C.YELLOW)
                self.connected.clear()
                while not self.queue.empty():          # không chạy lệnh cũ khi nối lại
                    self.queue.get_nowait()
                await asyncio.sleep(2.0)

    async def _sender(self, ws) -> None:
        while True:
            msg = await self.queue.get()
            await ws.send_str(json.dumps(msg, ensure_ascii=False))

    def _on_message(self, m: dict) -> None:
        kind = m.get("type")
        msg = m.get("msg", "")
        if kind == "reply":
            if m.get("ok"):
                if m.get("cmd") in ("move", "turn"):
                    self.accepted += 1
                elif m.get("cmd") == "stop":
                    self.finished = self.accepted
                log("✓ XE", msg, C.GREEN)
            else:
                log("✗ XE", msg, C.RED)
        elif kind == "event":
            if "Xong:" in msg or "Dừng lệnh giọng nói" in msg:
                self.finished += 1
            elif "Huỷ" in msg:
                self.finished = self.accepted
            log("• XE", msg, C.CYAN if m.get("level") == "info" else C.YELLOW)

    def wait_idle(self, timeout: float) -> None:
        """Chờ các lệnh đã gửi chạy xong (robotd huỷ lệnh khi mất kết nối, nên --text không được thoát sớm)."""
        t_end = time.monotonic() + timeout
        time.sleep(0.5)
        while time.monotonic() < t_end and self.connected.is_set() and self.finished < self.accepted:
            time.sleep(0.1)


# ------------------------------------------------------------------ xử lý câu

class VoiceController:
    def __init__(self, link: RobotLink | None):
        self.link = link
        self.last_partial = 0.0

    def on_start(self) -> None:
        if self.link:
            self.link.send({"type": "listening"})

    def on_mic_status(self, st: dict) -> None:
        if self.link:
            self.link.send({"type": "mic", **{k: st.get(k) for k in ("connected", "device", "rssi", "level_dbfs", "peer")}})

    def on_partial(self, raw: str) -> None:
        if self.link:
            self.link.send({"type": "partial", "text": raw})

    def handle_text(self, raw: str) -> None:
        r = parse(raw)
        log("🧠 ASR", f"“{raw}”" + (f"  →  “{r.normalized}”" if r.normalized != raw.lower().strip() else ""),
            C.MAGENTA)
        if self.link:
            self.link.send({"type": "heard", "text": raw, "normalized": r.normalized,
                            "commands": [c.describe() for c in r.commands], "errors": r.errors})
        for e in r.errors:
            log("⚠ LỆNH", e, C.YELLOW)
        if not r.commands:
            if not r.errors:
                log("⚠ LỆNH", "không có lệnh (tiến/lùi <cm>, xoay trái/phải <độ>, dừng)", C.DIM)
            return
        for c in r.commands:
            log("⚡ LỆNH", c.describe(), C.CYAN)
            if self.link:
                if not self.link.connected.is_set():
                    log("✗ XE", "chưa nối robotd: bỏ lệnh", C.RED)
                    continue
                self.link.send(c.to_message(raw))


class ColorDisplay(asr.TranscriptDisplay):
    def speech_started(self) -> None:
        if self.is_terminal:
            print("\r\033[2K" + color("● ĐANG NGHE", C.BOLD, C.GREEN), end="", flush=True)

    def partial(self, text: str) -> None:
        if text and text != self.last_partial and self.is_terminal:
            self.last_partial = text
            print("\r\033[2K" + color("≈ ", C.YELLOW) + text, end="", flush=True)

    def final(self, text: str) -> None:
        if self.is_terminal:
            print("\r\033[2K", end="")
        self.last_partial = ""


# ------------------------------------------------------------------ main

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Điều khiển xe bằng giọng nói tiếng Việt",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--robotd", default="ws://127.0.0.1:8080/ws/voice", help="WebSocket giọng nói của robotd")
    p.add_argument("--text", help="gửi một câu, chờ xe chạy xong rồi thoát")
    p.add_argument("--wait", type=float, default=60.0, help="--text: chờ tối đa (s)")
    p.add_argument("--keyboard", action="store_true", help="gõ câu lệnh thay cho mic")
    p.add_argument("--dry-run", action="store_true", help="không nối robotd, chỉ in lệnh")
    p.add_argument("--no-color", action="store_true")
    asr.add_microphone_arguments(p)
    asr.add_model_arguments(p)
    return p.parse_args()


def main() -> int:
    global USE_COLOR
    args = parse_args()
    asr.check_args(args)
    USE_COLOR = not args.no_color and sys.stdout.isatty()

    if args.list_devices:
        print(asr.import_sounddevice().query_devices())
        return 0
    if args.level:
        import numpy as np
        asr.run_level_meter(np, asr.import_sounddevice() if args.source == "mic" else None, args)
        return 0

    use_mic = not (args.text or args.keyboard)
    link = None
    if not args.dry_run and not args.check:
        mic_name = "bàn phím" if not use_mic else ("ESP32 #2 (WiFi)" if args.source == "ws" else "mic trên Pi")
        link = RobotLink(args.robotd, {"type": "hello", "client": "speech-control", "mic": mic_name}).start()
        link.connected.wait(3.0)
    ctl = VoiceController(link)

    if args.text:
        ctl.handle_text(args.text)
        if link:
            link.wait_idle(args.wait)
        return 0
    if args.keyboard:
        print("Gõ lệnh (vd: tiến 10, xoay trái 90, dừng). q = thoát.")
        while True:
            try:
                line = input("giọng nói > ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0
            if line.lower() in ("q", "quit", "exit"):
                return 0
            if line:
                ctl.handle_text(line)

    log("🎤 ASR", "đang nạp Zipformer + Silero VAD...", C.CYAN)
    np, recognizer, vad, window_size, files = asr.load_models(args)
    if args.check:
        print(f"OK: đã nạp mô hình ({files.encoder.name})")
        return 0
    asr.run_microphone(np, asr.import_sounddevice() if args.source == "mic" else None, recognizer, vad, window_size,
                       args, on_final=ctl.handle_text, on_partial=ctl.on_partial, on_start=ctl.on_start,
                       display=ColorDisplay(), on_status=ctl.on_mic_status)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Lỗi: {error}", file=sys.stderr)
        raise SystemExit(1)
