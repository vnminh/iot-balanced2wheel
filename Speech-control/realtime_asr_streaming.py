#!/usr/bin/env python3
"""Nhận dạng tiếng Việt streaming bằng hynt Zipformer + sherpa-onnx.

Nguồn âm thanh (--source):
  ws  (mặc định) ESP32 #2 đọc INMP441 và gửi PCM16 16 kHz qua WiFi WebSocket; Pi làm server
      ws://0.0.0.0:8765/audio (firmware-mic/). Không cần mic / overlay I2S trên Pi.
  mic đọc mic ngay trên Pi (INMP441 nối I2S của Pi, overlay googlevoicehat, xem pi/scripts/setup_mic.sh).

Mô hình: hynt/Zipformer-30M-RNNT-Streaming-6000h (OnlineRecognizer, streaming thật).
Silero VAD chỉ quyết định lúc bắt đầu / kết thúc câu. Có bộ đệm âm thanh trước khi VAD bật (pre-roll)
để không mất đầu câu ngắn ("tiến", "lùi").

--source mic: overlay googlevoicehat-soundcard chỉ chạy 48 kHz, 2 kênh, S32. Mở đúng tần số gốc của thiết bị,
lấy một kênh (L/R nối GND = kênh TRÁI = 0), lọc DC, khuếch đại, rồi hạ mẫu về 16 kHz.

  python realtime_asr_streaming.py --list-devices
  python realtime_asr_streaming.py --level        # đo mức âm (chỉnh --gain)
  python realtime_asr_streaming.py --check        # chỉ nạp mô hình
  python realtime_asr_streaming.py                # in câu nhận được (đã chuẩn hoá theo lệnh xe)
"""

from __future__ import annotations

import argparse
import math
import queue
import sys
import time
from collections import deque
from collections.abc import Callable
from pathlib import Path
from typing import Any, NamedTuple

from robot_commands import normalize_text

SAMPLE_RATE = 16_000
PROJECT_DIR = Path(__file__).resolve().parent
DEFAULT_MODEL_DIR = PROJECT_DIR / "models" / "Zipformer-30M-RNNT-Streaming-6000h"
DEFAULT_VAD_MODEL = PROJECT_DIR / "models" / "silero_vad.onnx"
# Tên thiết bị ALSA của INMP441 (googlevoicehat) để tự chọn mic
DEVICE_HINTS = ("googlevoicehat", "voicehat", "sndrpigooglevoi", "inmp441", "i2s")


def postprocess_command_text(text: str) -> str:
    """Chuẩn hoá câu ASR theo từ vựng lệnh xe (vi_dictionary.py)."""
    return normalize_text(text)


class ModelFiles(NamedTuple):
    tokens: Path
    encoder: Path
    decoder: Path
    joiner: Path


def parse_device(value: str) -> int | str:
    try:
        return int(value)
    except ValueError:
        return value


# ------------------------------------------------------------------ tham số dòng lệnh

def add_microphone_arguments(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("nguồn âm thanh")
    g.add_argument("--source", choices=("ws", "mic"), default="ws",
                   help="ws = nhận PCM từ ESP32 #2 qua WebSocket (Pi là server); mic = mic nối thẳng vào Pi")
    g.add_argument("--listen", default="0.0.0.0:8765", help="--source ws: địa chỉ:cổng của WebSocket server")
    g.add_argument("--device", type=parse_device, default=None,
                   help="số hoặc một phần tên thiết bị vào (mặc định: tự tìm googlevoicehat/INMP441)")
    g.add_argument("--channel", type=int, default=0,
                   help="kênh của mic: 0 = TRÁI (INMP441 L/R nối GND), 1 = phải")
    g.add_argument("--gain", type=float, default=None,
                   help="khuếch đại thêm trên Pi (mặc định: ws 1 vì ESP32 đã khuếch đại, mic 8). "
                        "Đo bằng --level: nói bình thường nên ~ -25..-10 dBFS")
    g.add_argument("--list-devices", action="store_true", help="liệt kê thiết bị âm thanh rồi thoát")
    g.add_argument("--level", action="store_true", help="in mức âm (dBFS) liên tục để chỉnh --gain")


def add_model_arguments(parser: argparse.ArgumentParser) -> None:
    g = parser.add_argument_group("ASR / VAD")
    g.add_argument("--model-dir", type=Path, default=DEFAULT_MODEL_DIR)
    g.add_argument("--vad-model", type=Path, default=DEFAULT_VAD_MODEL)
    g.add_argument("--chunk-size", type=int, default=32, choices=(16, 32, 64),
                   help="chunk encoder Zipformer: lớn hơn = chính xác hơn nhưng trễ hơn")
    g.add_argument("--check", action="store_true", help="nạp mô hình ASR + VAD rồi thoát (không mở mic)")
    g.add_argument("--num-threads", type=int, default=2, help="luồng CPU cho ASR (Pi 4: 2, để dành cho SLAM)")
    g.add_argument("--provider", default="cpu", choices=("cpu", "cuda", "coreml"))
    g.add_argument("--decoding-method", default="modified_beam_search",
                   choices=("greedy_search", "modified_beam_search"))
    g.add_argument("--max-active-paths", type=int, default=4, help="beam (4 đủ cho câu lệnh ngắn, nhẹ CPU)")
    g.add_argument("--blank-penalty", type=float, default=0.25)
    g.add_argument("--partial-interval", type=float, default=0.3,
                   help="giây giữa hai lần hiện kết quả tạm; 0 = tắt")
    g.add_argument("--vad-threshold", type=float, default=0.4)
    g.add_argument("--min-silence", type=float, default=0.6, help="im lặng bao lâu thì chốt câu (s)")
    g.add_argument("--min-speech", type=float, default=0.15)
    g.add_argument("--max-speech", type=float, default=8.0, help="câu lệnh dài hơn bị cắt (s)")
    g.add_argument("--pre-roll", type=float, default=0.5, help="âm thanh giữ lại trước khi VAD bật (s)")
    g.add_argument("--tail-padding", type=float, default=0.30, help="số 0 thêm vào cuối câu trước khi chốt (s)")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Nhận dạng tiếng Việt streaming (hynt/Zipformer-30M-RNNT-Streaming-6000h) từ INMP441",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    add_microphone_arguments(parser)
    add_model_arguments(parser)
    return parser.parse_args()


def check_args(args: argparse.Namespace) -> None:
    if args.num_threads < 1:
        raise ValueError("--num-threads phải >= 1")
    for name in ("partial_interval", "pre_roll", "tail_padding"):
        if getattr(args, name) < 0:
            raise ValueError(f"--{name.replace('_', '-')} không được âm")
    if args.channel < 0:
        raise ValueError("--channel phải >= 0")
    if args.gain is None:
        args.gain = 1.0 if args.source == "ws" else 8.0


# ------------------------------------------------------------------ nạp thư viện / mô hình

def import_sounddevice() -> Any:
    try:
        import sounddevice as sd
    except (ImportError, OSError) as exc:
        raise RuntimeError("Không dùng được mic. Cài: sudo apt install libportaudio2 và pip install sounddevice. "
                           f"Lỗi gốc: {exc}") from exc
    return sd


def import_runtime() -> tuple[Any, Any]:
    try:
        import numpy as np
        import sherpa_onnx
    except ImportError as exc:
        raise RuntimeError("Thiếu numpy / sherpa-onnx: chạy bash ~/iot-ck/pi/scripts/setup_voice.sh") from exc
    return np, sherpa_onnx


def is_lfs_pointer(path: Path) -> bool:
    try:
        with path.open("rb") as file:
            return file.read(80).startswith(b"version https://git-lfs.github.com/spec/v1")
    except OSError:
        return False


def require_real_file(path: Path, label: str) -> None:
    if not path.is_file():
        raise FileNotFoundError(f"Thiếu {label}: {path}  (chạy bash ~/iot-ck/pi/scripts/setup_voice.sh)")
    if is_lfs_pointer(path):
        raise RuntimeError(f"{label} chỉ là con trỏ Git LFS, chưa tải mô hình thật: {path}")


def find_model_files(model_dir: Path, chunk_size: int = 32) -> ModelFiles:
    tokens = model_dir / "tokens.txt"
    if not tokens.is_file():
        tokens = model_dir / "config.json"     # repo của hynt để bảng token trong config.json
    suffix = f"epoch-31-avg-11-chunk-{chunk_size}-left-128.fp16.onnx"
    files = ModelFiles(tokens=tokens, encoder=model_dir / f"encoder-{suffix}",
                       decoder=model_dir / f"decoder-{suffix}", joiner=model_dir / f"joiner-{suffix}")
    for label, path in zip(ModelFiles._fields, files):
        require_real_file(path, label)
    return files


def create_recognizer(sherpa_onnx: Any, files: ModelFiles, args: argparse.Namespace) -> Any:
    return sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=str(files.tokens), encoder=str(files.encoder), decoder=str(files.decoder),
        joiner=str(files.joiner), num_threads=args.num_threads, sample_rate=SAMPLE_RATE, feature_dim=80,
        decoding_method=args.decoding_method, max_active_paths=args.max_active_paths,
        blank_penalty=args.blank_penalty, provider=args.provider, enable_endpoint_detection=False)


def create_vad(sherpa_onnx: Any, args: argparse.Namespace) -> tuple[Any, int]:
    require_real_file(args.vad_model, "mô hình VAD")
    config = sherpa_onnx.VadModelConfig()
    config.silero_vad.model = str(args.vad_model)
    config.silero_vad.threshold = args.vad_threshold
    config.silero_vad.min_silence_duration = args.min_silence
    config.silero_vad.min_speech_duration = args.min_speech
    config.silero_vad.max_speech_duration = args.max_speech
    config.sample_rate = SAMPLE_RATE
    window_size = int(config.silero_vad.window_size)
    detector = sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=max(30, int(args.max_speech) + 5))
    return detector, window_size


def load_models(args: argparse.Namespace) -> tuple[Any, Any, Any, int, ModelFiles]:
    np, sherpa_onnx = import_runtime()
    files = find_model_files(args.model_dir, args.chunk_size)
    recognizer = create_recognizer(sherpa_onnx, files, args)
    vad, window_size = create_vad(sherpa_onnx, args)
    return np, recognizer, vad, window_size, files


def decode_ready(recognizer: Any, stream: Any) -> None:
    while recognizer.is_ready(stream):
        recognizer.decode_stream(stream)


def get_result_text(recognizer: Any, stream: Any) -> str:
    result = recognizer.get_result(stream)
    return (result.text if hasattr(result, "text") else str(result)).strip()


def finalize_stream(np: Any, recognizer: Any, stream: Any, tail_padding: float) -> str:
    if tail_padding > 0:
        stream.accept_waveform(SAMPLE_RATE, np.zeros(int(tail_padding * SAMPLE_RATE), dtype=np.float32))
        decode_ready(recognizer, stream)
    stream.input_finished()
    decode_ready(recognizer, stream)
    return get_result_text(recognizer, stream)


# ------------------------------------------------------------------ mic

def find_input_device(sd: Any, wanted: int | str | None) -> int:
    """Chọn thiết bị vào: theo --device, hoặc tự tìm INMP441 (googlevoicehat), hoặc mặc định."""
    devices = sd.query_devices()
    inputs = [(i, d) for i, d in enumerate(devices) if d["max_input_channels"] > 0]
    if not inputs:
        raise RuntimeError("Không có thiết bị thu âm nào. Kiểm tra: arecord -l (cần dtoverlay=googlevoicehat-soundcard "
                           "và khởi động lại Pi)")
    if isinstance(wanted, int):
        return wanted
    if isinstance(wanted, str):
        for i, d in inputs:
            if wanted.lower() in d["name"].lower():
                return i
        raise RuntimeError(f"Không thấy thiết bị vào có tên chứa '{wanted}' (xem --list-devices)")
    for hint in DEVICE_HINTS:
        for i, d in inputs:
            if hint in d["name"].lower():
                return i
    default = sd.default.device[0]
    return default if default is not None and default >= 0 else inputs[0][0]


class MicConditioner:
    """Tần số gốc + nhiều kênh -> 16 kHz một kênh: chọn kênh, lọc DC, khuếch đại, lọc thông thấp + hạ mẫu."""

    def __init__(self, np: Any, in_rate: int, gain: float):
        self.np = np
        self.in_rate = int(in_rate)
        self.gain = gain
        self.dc = 0.0
        self.factor = self.in_rate // SAMPLE_RATE if self.in_rate % SAMPLE_RATE == 0 else 0
        self.h = None
        self.hist = np.zeros(0, dtype=np.float32)
        self.phase = 0
        self.frac_pos = 0.0
        if self.factor > 1:
            taps = 16 * self.factor + 1          # FIR sinc cửa sổ Hamming, cắt ở 0,45 x 16 kHz
            n = np.arange(taps) - (taps - 1) / 2
            fc = 0.45 * SAMPLE_RATE / self.in_rate
            h = 2 * fc * np.sinc(2 * fc * n) * np.hamming(taps)
            self.h = (h / h.sum()).astype(np.float32)
            self.hist = np.zeros(taps - 1, dtype=np.float32)

    def process(self, x: Any) -> Any:
        np = self.np
        x = np.asarray(x, dtype=np.float32)
        # lọc DC (INMP441 có lệch DC): trung bình trượt rất chậm
        mean = float(x.mean()) if x.size else 0.0
        self.dc += 0.05 * (mean - self.dc)
        x = (x - self.dc) * self.gain
        np.clip(x, -1.0, 1.0, out=x)
        if self.in_rate == SAMPLE_RATE:
            return x
        if self.factor > 1:
            buf = np.concatenate([self.hist, x])
            y = np.convolve(buf, self.h, mode="valid")          # len = len(x)
            self.hist = buf[-(len(self.h) - 1):]
            out = y[self.phase::self.factor]
            self.phase = (self.phase - len(y)) % self.factor
            return out.astype(np.float32)
        # tỉ lệ không nguyên: nội suy tuyến tính (đủ cho tiếng nói)
        step = self.in_rate / SAMPLE_RATE
        buf = np.concatenate([self.hist[-1:] if self.hist.size else x[:1], x])
        pos = np.arange(self.frac_pos + 1.0, len(buf) - 1, step)
        out = np.interp(pos, np.arange(len(buf)), buf).astype(np.float32)
        self.frac_pos = (pos[-1] + step - (len(buf) - 1)) - 1.0 if pos.size else self.frac_pos - len(x)
        self.hist = buf[-1:]
        return out


class Rechunker:
    """Gom mẫu 16 kHz thành từng khối đúng window_size cho Silero VAD."""

    def __init__(self, np: Any, size: int):
        self.np, self.size = np, size
        self.buf = np.zeros(0, dtype=np.float32)

    def push(self, x: Any) -> list:
        self.buf = self.np.concatenate([self.buf, x])
        out = []
        while len(self.buf) >= self.size:
            out.append(self.buf[:self.size])
            self.buf = self.buf[self.size:]
        return out


def open_microphone(np: Any, sd: Any, args: argparse.Namespace, block_16k: int,
                    callback: Callable[[Any], None]) -> tuple[Any, str]:
    """Mở mic ở tần số gốc; callback nhận mẫu float32 16 kHz một kênh (đã xử lý)."""
    dev = find_input_device(sd, args.device)
    info = sd.query_devices(dev, "input")
    rate = int(info["default_samplerate"]) or 48000
    channels = max(1, min(int(info["max_input_channels"]), 2))
    if args.channel >= channels:
        raise RuntimeError(f"Thiết bị chỉ có {channels} kênh, không có --channel {args.channel}")
    cond = MicConditioner(np, rate, args.gain)

    def cb(indata: Any, frames: int, time_info: Any, status: Any) -> None:
        del frames, time_info
        if status:
            print(f"Cảnh báo âm thanh: {status}", file=sys.stderr)
        callback(cond.process(indata[:, args.channel]))

    blocksize = int(block_16k * rate / SAMPLE_RATE)
    stream = sd.InputStream(device=dev, channels=channels, samplerate=rate, dtype="float32",
                            blocksize=blocksize, latency="low", callback=cb)
    desc = f"{info['name']} (#{dev}, {rate} Hz, {channels} kênh, dùng kênh {'trái' if args.channel == 0 else args.channel}, gain {args.gain:g})"
    return stream, desc


class WsAudioSource:
    """WebSocket server nhận âm thanh từ ESP32 #2 (firmware-mic): text {"type":"hello",...} rồi các khung nhị phân
    PCM16 little-endian. Chỉ một ESP32 tại một thời điểm (kết nối mới thay kết nối cũ). Dùng như context manager."""

    def __init__(self, np: Any, args: argparse.Namespace, callback: Callable[[Any], None],
                 on_status: Callable[[dict], None] | None = None):
        self.np, self.args, self.callback, self.on_status = np, args, callback, on_status
        host, _, port = args.listen.rpartition(":")
        self.host, self.port = host or "0.0.0.0", int(port)
        self.cond = MicConditioner(np, SAMPLE_RATE, args.gain)
        self.loop = None
        self.thread = None
        self.ready = __import__("threading").Event()
        self.current = None
        self.info: dict = {"connected": False}
        self._last_stat_print = 0.0

    def __enter__(self) -> "WsAudioSource":
        import asyncio
        import threading

        def run() -> None:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._start())
            self.ready.set()
            self.loop.run_forever()

        self.thread = threading.Thread(target=run, name="ws-audio", daemon=True)
        self.thread.start()
        if not self.ready.wait(5.0):
            raise RuntimeError(f"Không mở được WebSocket server {self.args.listen}")
        return self

    def __exit__(self, *exc: Any) -> None:
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)

    def _status(self, **kw: Any) -> None:
        self.info.update(kw)
        if self.on_status:
            try:
                self.on_status(dict(self.info))
            except Exception as e:   # không để lỗi hiển thị làm rơi luồng âm thanh
                print("ws-audio status:", e, file=sys.stderr)

    async def _start(self) -> None:
        from aiohttp import web
        app = web.Application()
        app.router.add_get("/audio", self._handler)
        app.router.add_get("/", lambda r: web.Response(text=f"Speech-control: {self.info}\n"))
        self.runner = web.AppRunner(app, access_log=None)
        await self.runner.setup()
        await web.TCPSite(self.runner, self.host, self.port).start()

    async def _handler(self, request: Any) -> Any:
        import json as _json
        from aiohttp import WSMsgType, web
        ws = web.WebSocketResponse(max_msg_size=1 << 20)
        await ws.prepare(request)
        if self.current is not None and not self.current.closed:
            await self.current.close()   # chỉ giữ ESP32 mới nhất
        self.current = ws
        peer = request.remote
        print(f"ESP32 mic đã nối từ {peer}", flush=True)
        self._status(connected=True, peer=peer, device="", rssi=None, level_dbfs=None)
        rate = SAMPLE_RATE
        np = self.np
        try:
            async for msg in ws:
                if msg.type == WSMsgType.BINARY:
                    pcm = np.frombuffer(msg.data, dtype="<i2").astype(np.float32) / 32768.0
                    self.callback(self.cond.process(pcm))
                elif msg.type == WSMsgType.TEXT:
                    try:
                        m = _json.loads(msg.data)
                    except ValueError:
                        continue
                    if m.get("type") == "hello":
                        rate = int(m.get("rate", SAMPLE_RATE))
                        if rate != self.cond.in_rate:
                            self.cond = MicConditioner(np, rate, self.args.gain)
                        print(f"ESP32 mic: {m.get('device')} {m.get('fw')}, {rate} Hz, khung {m.get('frame_ms')} ms",
                              flush=True)
                        self._status(device=f"{m.get('device', 'esp32')} ({m.get('fw', '')})")
                    elif m.get("type") == "stat":
                        self._status(rssi=m.get("rssi"), level_dbfs=m.get("level_dbfs"), dropped=m.get("dropped"))
                        now = time.monotonic()
                        if now - self._last_stat_print > 60:
                            self._last_stat_print = now
                            print(f"ESP32 mic: RSSI {m.get('rssi')} dBm, mức âm {m.get('level_dbfs')} dBFS, "
                                  f"đỉnh {m.get('peak_dbfs')} dBFS, khung bỏ {m.get('dropped')}", flush=True)
        finally:
            if self.current is ws:
                self.current = None
                print("ESP32 mic mất kết nối", flush=True)
                self._status(connected=False)
        return ws


def open_source(np: Any, sd: Any, args: argparse.Namespace, block_16k: int, callback: Callable[[Any], None],
                on_status: Callable[[dict], None] | None = None) -> tuple[Any, str]:
    """Nguồn âm thanh theo --source: trả (context manager, mô tả). callback nhận float32 16 kHz một kênh."""
    if args.source == "ws":
        src = WsAudioSource(np, args, callback, on_status)
        return src, f"WebSocket server ws://{args.listen}/audio (ESP32 #2, gain {args.gain:g})"
    return open_microphone(np, sd, args, block_16k, callback)


def run_level_meter(np: Any, sd: Any, args: argparse.Namespace) -> None:
    """In mức âm để chỉnh --gain: im lặng ~ -60..-45 dBFS, nói bình thường ~ -25..-10 dBFS."""
    q: queue.Queue = queue.Queue(maxsize=100)
    chunker = Rechunker(np, 1600)

    def push(x: Any) -> None:
        for blk in chunker.push(x):
            if not q.full():
                q.put_nowait(blk)
    stream, desc = open_source(np, sd, args, 1600, push)
    print(f"Mic: {desc}\nNói thử (Ctrl+C để thoát)...")
    with stream:
        try:
            while True:
                try:
                    x = q.get(timeout=1.0)
                except queue.Empty:
                    print("\r(chưa có âm thanh)" + " " * 40, end="", flush=True)
                    continue
                rms = float(np.sqrt(np.mean(x * x))) if x.size else 0.0
                peak = float(np.max(np.abs(x))) if x.size else 0.0
                db = 20 * math.log10(max(rms, 1e-6))
                bar = "#" * max(0, int((db + 70) / 2))
                print(f"\r{db:6.1f} dBFS  đỉnh {peak:4.2f} {'CLIP ' if peak >= 0.99 else '     '}{bar:<35}", end="", flush=True)
        except KeyboardInterrupt:
            print()


class TranscriptDisplay:
    def __init__(self) -> None:
        self.is_terminal = sys.stdout.isatty()
        self.last_partial = ""
        self.sentence_number = 0

    def speech_started(self) -> None:
        print("\r\033[2KĐang nghe...", end="", flush=True) if self.is_terminal else None

    def partial(self, text: str) -> None:
        if not text or text == self.last_partial:
            return
        self.last_partial = text
        if self.is_terminal:
            print(f"\r\033[2K[tạm] {text}", end="", flush=True)

    def final(self, text: str) -> None:
        self.sentence_number += 1
        prefix = "\r\033[2K" if self.is_terminal else ""
        print(f"{prefix}[câu {self.sentence_number}] {text or '(không nhận ra chữ nào)'}", flush=True)
        self.last_partial = ""


def run_microphone(np: Any, sd: Any, recognizer: Any, vad: Any, window_size: int, args: argparse.Namespace,
                   on_final: Callable[[str], None] | None = None,
                   on_partial: Callable[[str], None] | None = None,
                   on_start: Callable[[], None] | None = None,
                   display: Any = None, stop_event: Any = None,
                   on_status: Callable[[dict], None] | None = None) -> None:
    """Vòng nghe mic. on_final nhận câu GỐC của ASR (chưa chuẩn hoá); on_partial nhận câu tạm GỐC."""
    audio_queue: queue.Queue[Any] = queue.Queue(maxsize=250)
    dropped = [0]
    chunker = Rechunker(np, window_size)

    def push(samples: Any) -> None:
        for blk in chunker.push(samples):
            try:
                audio_queue.put_nowait(blk)
            except queue.Full:
                dropped[0] += 1

    block_seconds = window_size / SAMPLE_RATE
    pre_roll: deque[Any] = deque(maxlen=max(1, math.ceil(args.pre_roll / block_seconds)))
    display = display or TranscriptDisplay()
    active, stream, last_partial_time = False, None, 0.0

    mic, desc = open_source(np, sd, args, window_size, push, on_status)
    print(f"Nguồn âm thanh: {desc}")
    print(f"Zipformer chunk={args.chunk_size}, beam={args.max_active_paths}, VAD={args.vad_threshold:.2f}, "
          f"im lặng chốt câu={args.min_silence:.2f}s")
    print("Sẵn sàng. Nói lệnh rồi ngừng một chút để chốt. Ctrl+C để thoát.", flush=True)

    def finish() -> None:
        nonlocal active, stream, last_partial_time
        raw = finalize_stream(np, recognizer, stream, args.tail_padding)
        display.final(postprocess_command_text(raw))
        if raw and on_final is not None:
            on_final(raw)
        active, stream, last_partial_time = False, None, 0.0
        pre_roll.clear()

    try:
        with mic:
            while stop_event is None or not stop_event.is_set():
                try:
                    samples = audio_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                pre_roll.append(samples)
                vad.accept_waveform(samples)           # VAD chỉ quyết định bắt đầu / kết thúc
                speech_now = vad.is_speech_detected()
                if not active and speech_now:
                    active = True
                    stream = recognizer.create_stream()
                    stream.accept_waveform(SAMPLE_RATE, np.concatenate(list(pre_roll)))   # cả đoạn trước VAD
                    decode_ready(recognizer, stream)
                    last_partial_time = 0.0
                    display.speech_started()
                    if on_start:
                        on_start()
                elif active and stream is not None:
                    stream.accept_waveform(SAMPLE_RATE, samples)
                    decode_ready(recognizer, stream)

                if active and stream is not None and args.partial_interval > 0:
                    now = time.monotonic()
                    if now - last_partial_time >= args.partial_interval:
                        raw = get_result_text(recognizer, stream)
                        display.partial(postprocess_command_text(raw))
                        if raw and on_partial:
                            on_partial(raw)
                        last_partial_time = now

                done = False
                while not vad.empty():                 # không giải mã vad.front.samples (có thể bị cắt đầu)
                    vad.pop()
                    done = True
                if done and active and stream is not None:
                    finish()
    except KeyboardInterrupt:
        if active and stream is not None:
            finish()
        print("Đã dừng.")
    finally:
        if dropped[0]:
            print(f"Cảnh báo: bỏ {dropped[0]} khối âm thanh vì xử lý không kịp (giảm --max-active-paths?)",
                  file=sys.stderr)


def main() -> int:
    args = parse_args()
    check_args(args)
    if args.list_devices:
        print(import_sounddevice().query_devices())
        return 0
    if args.level:
        import numpy as np
        run_level_meter(np, import_sounddevice() if args.source == "mic" else None, args)
        return 0
    print("Đang nạp Zipformer + Silero VAD...")
    np, recognizer, vad, window_size, files = load_models(args)
    if args.check:
        print(f"OK: đã nạp mô hình ASR streaming + VAD ({files.encoder.name})")
        return 0
    run_microphone(np, import_sounddevice() if args.source == "mic" else None, recognizer, vad, window_size, args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, RuntimeError, ValueError) as error:
        print(f"Lỗi: {error}", file=sys.stderr)
        raise SystemExit(1)
