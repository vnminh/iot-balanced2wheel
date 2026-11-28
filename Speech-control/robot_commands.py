"""Chuẩn hoá câu ASR tiếng Việt và tách lệnh điều khiển xe.

  "tiến 10"                 -> forward 10 cm
  "lùi năm mươi xăng ti mét" -> backward 50 cm
  "xoay trái 180"           -> turn +180°   (+ = sang trái, ngược chiều kim đồng hồ)
  "quay phải chín mươi độ"  -> turn -90°
  "tiến một mét rưỡi rồi xoay trái 90"  -> 2 lệnh nối tiếp
  "dừng" / "dừng lại"        -> stop

Đơn vị mặc định: cm cho tiến/lùi, độ cho xoay. Nhận cả mét, tấc, mm, vòng, "nửa vòng", "quay đầu".
Chỉ thư viện chuẩn (không cần numpy), dùng được ở cả máy không có mic:  python3 robot_commands.py "tiếng mười"
"""
from __future__ import annotations

import re
import sys
import unicodedata
from dataclasses import dataclass, field

import vi_dictionary as D

# ------------------------------------------------------------------ tiện ích


def strip_marks(text: str) -> str:
    text = text.replace("đ", "d").replace("Đ", "D")
    return "".join(c for c in unicodedata.normalize("NFD", text) if unicodedata.category(c) != "Mn").lower()


def edit_distance(a: str, b: str) -> int:
    prev = list(range(len(b) + 1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            cur.append(min(cur[-1] + 1, prev[j] + 1, prev[j - 1] + (ca != cb)))
        prev = cur
    return prev[-1]


def _build_stripped() -> dict:
    """bỏ dấu -> từ chuẩn, chỉ giữ khi không nhập nhằng (hoặc có trong STRIPPED_PREFERRED)."""
    cand: dict[str, set] = {}
    for w in D.CANONICAL:
        cand.setdefault(strip_marks(w), set()).add(w)
    for w, c in D.CONFUSIONS.items():
        cand.setdefault(strip_marks(w), set()).add(c)
    out = {k: next(iter(v)) for k, v in cand.items() if len(v) == 1}
    out.update(D.STRIPPED_PREFERRED)
    return out


_STRIPPED = _build_stripped()
_FUZZY = [(strip_marks(w), D.CONFUSIONS.get(w, w)) for w in D.FUZZY_TARGETS]
_PHRASES = sorted(D.PHRASES.items(), key=lambda kv: -len(kv[0]))
_MAXPH = max(len(k) for k in D.PHRASES)
_NUMERIC = re.compile(r"^\d+(?:[.,]\d+)?$")
_NUM_TOKENS = set(D.DIGITS) | {"mười", "mươi", "trăm", "nghìn", "linh", "lẻ"}


def correct_token(tok: str) -> str:
    """Một từ -> từ chuẩn ('' = bỏ). Thứ tự: chuẩn, đệm, lỗi đã biết, bỏ dấu, gần đúng."""
    if _NUMERIC.match(tok) or tok in D.CANONICAL:
        return tok
    if tok in D.FILLERS:
        return ""
    if tok in D.CONFUSIONS:
        return D.CONFUSIONS[tok]
    plain = strip_marks(tok)
    if plain in _STRIPPED:
        return _STRIPPED[plain]
    if len(plain) >= 3:
        best = sorted((edit_distance(plain, t), abs(len(t) - len(plain)), w) for t, w in _FUZZY
                      if t[0] == plain[0] and abs(len(t) - len(plain)) <= 1)
        if best and best[0][0] <= 1 and (len(best) == 1 or best[1][:2] != best[0][:2]):
            return best[0][2]
    return tok


def tokenize(text: str) -> list[str]:
    text = unicodedata.normalize("NFC", text).lower()
    text = re.sub(r"(\d)[,.](\d)", r"\1#\2", text)               # giữ dấu thập phân 1,5 / 1.5
    text = re.sub(r"(\d)\s*°", r"\1 độ ", text)
    text = re.sub(r"[^\w#]+", " ", text, flags=re.UNICODE)
    text = re.sub(r"(\d)([^\d\s#])", r"\1 \2", text)              # "10cm" -> "10 cm"
    text = re.sub(r"([^\d\s#])(\d)", r"\1 \2", text)
    return [t.replace("#", ".") for t in text.split()]


def normalize_tokens(text: str) -> list[str]:
    raw = tokenize(text)
    plain = [strip_marks(t) for t in raw]
    out: list[str] = []
    i = 0
    while i < len(raw):
        for n in range(min(_MAXPH, len(raw) - i), 1, -1):          # cụm dài nhất trước
            rep = D.PHRASES.get(tuple(plain[i:i + n]))
            if rep is not None:
                out.extend([rep] if isinstance(rep, str) else list(rep))
                i += n
                break
        else:
            tok = raw[i]
            # "sau" là từ đệm, trừ khi đứng cạnh từ số / sau lệnh: khi đó là "sáu" bị mất dấu
            if plain[i] == "sau":
                nxt = correct_token(raw[i + 1]) if i + 1 < len(raw) else ""
                prev = out[-1] if out else ""
                if nxt in ("mươi", "mười", "trăm", "độ", "cm", "m", "tấc", "vòng") or prev in _NUM_TOKENS \
                        or prev in D.ACTIONS or prev in D.DIRECTIONS:
                    tok = "sáu"
            out.append(correct_token(tok))
            i += 1
    out = [t for t in out if t]
    # "mười" đứng ngay sau một chữ số = "mươi" (hai mười -> hai mươi)
    for k in range(1, len(out)):
        if out[k] == "mười" and out[k - 1] in D.DIGITS:
            out[k] = "mươi"
    return out


def normalize_text(text: str) -> str:
    return " ".join(normalize_tokens(text))


# ------------------------------------------------------------------ số tiếng Việt


def words_to_number(words: list[str]) -> float | None:
    """['một','trăm','tám','mươi'] -> 180. Hỗ trợ: mốt/lăm/tư, linh/lẻ, 'hai lăm' (=25),
    'hai trăm năm' (=250), 'trăm tám' (=180), 'trăm rưỡi' (=150), 'một phẩy năm' (=1.5)."""
    if not words:
        return None
    if "phẩy" in words:
        k = words.index("phẩy")
        ip, fp = words_to_number(words[:k]), words[k + 1:]
        if ip is None or not fp or not all(w in D.DIGITS for w in fp):
            return None
        return float(f"{int(ip)}." + "".join(str(D.DIGITS[w]) for w in fp))
    total, group = 0, 0
    last, last_d = None, None       # từ trước, chữ số trước (đang ở hàng đơn vị của group)
    n = len(words)
    for k, w in enumerate(words):
        if w in D.DIGITS:
            d = D.DIGITS[w]
            if last in ("mươi", "mười", "linh", "lẻ"):
                group += d
                last_d = None
            elif last == "trăm" and k == n - 1:          # "hai trăm năm" = 250
                group += d * 10
                last_d = None
            elif last == "nghìn" and k == n - 1:         # "một nghìn hai" = 1200
                group += d * 100
                last_d = None
            elif last in D.DIGITS and last_d is not None and d > 0:   # "hai lăm" = 25
                group += last_d * 9 + d
                last_d = None
            else:
                group += d
                last_d = d
        elif w == "mười":
            group += 10
            last_d = None
        elif w == "mươi":
            if last_d is not None:
                group += last_d * 9
            else:
                group += 10
            last_d = None
        elif w == "trăm":
            if last_d is not None:
                group += last_d * 99
            elif last != "trăm":
                group += 100
            last_d = None
        elif w == "nghìn":
            total += (group or 1) * 1000
            group, last_d = 0, None
        elif w in ("linh", "lẻ"):
            pass
        elif w == "rưỡi":
            if last == "trăm":
                group += 50
            elif last == "nghìn":
                total += 500
            else:
                return None
        else:
            return None
        last = w
    return float(total + group)


# ------------------------------------------------------------------ tách lệnh


@dataclass
class Command:
    op: str                   # move | turn | stop
    value: float = 0.0        # move: cm (+ tiến, - lùi); turn: độ (+ trái, - phải)
    words: str = ""           # đoạn câu tạo ra lệnh (để hiện log)

    def describe(self) -> str:
        if self.op == "stop":
            return "Dừng"
        if self.op == "move":
            return f"{'Tiến' if self.value >= 0 else 'Lùi'} {abs(self.value):g} cm"
        return f"Xoay {'trái' if self.value >= 0 else 'phải'} {abs(self.value):g}°"

    def to_message(self, text: str) -> dict:
        """Bản tin gửi robotd qua WebSocket /ws/voice (giống web gửi {"type":"drive",...})."""
        if self.op == "move":
            return {"type": "move", "distance_m": round(self.value / 100.0, 4), "text": text}
        if self.op == "turn":
            return {"type": "turn", "angle_deg": round(self.value, 2), "text": text}
        return {"type": "stop", "text": text}


@dataclass
class ParseResult:
    text: str
    normalized: str
    commands: list = field(default_factory=list)
    errors: list = field(default_factory=list)


def _number_at(toks: list[str], i: int) -> tuple[float | None, int]:
    """Đọc một số bắt đầu ở toks[i]: chữ số '10', '1.5' hoặc chuỗi từ số. Trả (giá trị, vị trí kế)."""
    if i >= len(toks):
        return None, i
    if _NUMERIC.match(toks[i]):
        return float(toks[i].replace(",", ".")), i + 1
    j = i
    while j < len(toks) and (toks[j] in _NUM_TOKENS or toks[j] == "phẩy"
                             or (toks[j] == "rưỡi" and j > i and toks[j - 1] in ("trăm", "nghìn"))):
        j += 1
    while j > i and toks[j - 1] == "phẩy":       # "phẩy" ở cuối không phải dấu thập phân
        j -= 1
    if j == i:
        return None, i
    v = words_to_number(toks[i:j])
    return (v, j) if v is not None else (None, i)


def _quantity(toks: list[str], i: int, angular: bool) -> tuple[float | None, int, str]:
    """Số + đơn vị (+ 'rưỡi'). Trả (cm hoặc độ, vị trí kế, lỗi)."""
    if i < len(toks) and toks[i] == "nửa" and i + 1 < len(toks) and toks[i + 1] == "vòng":
        return (180.0, i + 2, "") if angular else (None, i + 2, "Tiến/lùi không đo bằng vòng")
    val, j = _number_at(toks, i)
    if val is None:
        return None, i, ""
    units = D.ANGLE_UNITS if angular else D.LINEAR_UNITS
    other = D.LINEAR_UNITS if angular else D.ANGLE_UNITS
    scale = 1.0
    if j < len(toks) and toks[j] in units:
        scale = units[toks[j]]
        unit = toks[j]
        j += 1
        if j < len(toks) and toks[j] == "rưỡi":          # "một mét rưỡi", "một vòng rưỡi"
            val += 0.5
            j += 1
        elif unit == "m" and val == int(val):            # "một mét hai" = 1,2 m; "một mét hai mươi" = 1,2 m
            frac, k = _number_at(toks, j)
            if frac is not None and frac < 100 and not (k < len(toks) and (toks[k] in units or toks[k] in other)):
                val += frac / 10.0 if frac < 10 else frac / 100.0
                j = k
    elif j < len(toks) and toks[j] in other:
        return None, j + 1, f"Sai đơn vị: {'xoay dùng độ' if angular else 'tiến/lùi dùng cm/mét'}"
    return val * scale, j, ""


def parse(text: str) -> ParseResult:
    toks = normalize_tokens(text)
    res = ParseResult(text=text, normalized=" ".join(toks))
    i, n = 0, len(toks)
    while i < n:
        t = toks[i]
        act = D.ACTIONS.get(t)
        start = i
        if act == "stop":
            res.commands.append(Command("stop", 0.0, t))
            i += 1
        elif act in ("forward", "backward"):
            j = i + 1
            val, k, err = _quantity(toks, j, angular=False)
            if val is None and not err and j < n and toks[j] not in D.ACTIONS:   # bỏ qua 1 từ lạ
                val, k, err = _quantity(toks, j + 1, angular=False)
            if err:
                res.errors.append(err)
                i = max(k, i + 1)
            elif val is None:
                res.errors.append(f"'{t}' thiếu số cm (vd: {t} 20)")
                i += 1
            else:
                res.commands.append(Command("move", val if act == "forward" else -val, " ".join(toks[start:k])))
                i = k
        elif act in ("turn", "uturn") or (t in D.DIRECTIONS and i + 1 < n and
                                           (_number_at(toks, i + 1)[0] is not None or toks[i + 1] == "nửa")):
            # xoay [trái|phải] <số> [độ]  |  xoay <số> [độ] [trái|phải]  |  trái <số>  |  quay đầu [<số>]
            j = i + 1 if act else i
            sign = 0
            if j < n and toks[j] in D.DIRECTIONS:
                sign = D.DIRECTIONS[toks[j]]
                j += 1
            elif j < n and toks[j] == "phẩy":           # "xoay phẩy 90": phải bị nghe thành phẩy
                sign = -1
                j += 1
            val, k, err = _quantity(toks, j, angular=True)
            if val is not None and not sign and k < n and toks[k] in D.DIRECTIONS:
                sign = D.DIRECTIONS[toks[k]]
                k += 1
            if act == "uturn":
                sign = sign or 1
                if val is None and not err:
                    val, k = 180.0, j
            if err:
                res.errors.append(err)
                i = max(k, i + 1)
            elif val is None:
                res.errors.append("Xoay thiếu số độ (vd: xoay trái 90)")
                i = j if j > i else i + 1
            elif not sign:
                res.errors.append("Xoay thiếu hướng trái / phải")
                i = k
            else:
                res.commands.append(Command("turn", sign * val, " ".join(toks[start:k])))
                i = k
        else:
            i += 1
    return res


if __name__ == "__main__":
    for line in (sys.argv[1:] or [l.strip() for l in sys.stdin if l.strip()]):
        r = parse(line)
        print(f"{line!r}\n  chuẩn hoá: {r.normalized!r}")
        for c in r.commands:
            print("  ->", c.describe(), c.to_message(line))
        for e in r.errors:
            print("  !!", e)
