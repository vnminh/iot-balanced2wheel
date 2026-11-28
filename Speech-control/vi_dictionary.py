"""Từ điển sửa lỗi nhận dạng (ASR) cho lệnh điều khiển xe bằng tiếng Việt.

Mô hình Zipformer hay nhầm do: thanh điệu (tiến/tiếng/tiền), phương ngữ (tr/ch, s/x, l/n, r/d/gi, n/ng cuối),
từ đồng âm ("năm" / "nam" / "nằm"), và đôi khi bỏ dấu hẳn. Thứ tự tra một từ (robot_commands.py):

  1. PHRASES       cụm nhiều từ (so khớp sau khi BỎ DẤU), vd "xen ti mét" -> cm, "đi thẳng" -> tiến
  2. CANONICAL     từ chuẩn: giữ nguyên
  3. FILLERS       từ đệm: bỏ đi ("lên", "lại", "khoảng"...)
  4. CONFUSIONS    lỗi đã biết: thay bằng từ chuẩn (khớp đúng cả dấu)
  5. bỏ dấu        "tien" -> tiến, "lui" -> lùi (chỉ khi không nhập nhằng)
  6. gần đúng      sai 1 ký tự (sau khi bỏ dấu) với từ khoá dài >= 3 chữ, cùng chữ cái đầu: "tieng" -> tiến

Muốn thêm lỗi mới: xem log (journalctl --user -u voice -f) dòng "ASR: ..." rồi thêm vào CONFUSIONS.
Kiểm tra: python3 test_robot_commands.py
"""

# ------------------------------------------------------------------ từ chuẩn
# Lệnh
ACTIONS = {
    "tiến": "forward",
    "lùi": "backward",
    "xoay": "turn",
    "quay": "turn",
    "rẽ": "turn",
    "quayđầu": "uturn",     # "quay đầu" = xoay trái 180°
    "dừng": "stop",
}
DIRECTIONS = {"trái": +1, "phải": -1}   # + = ngược chiều kim đồng hồ (sang trái)

# Đơn vị (về cm hoặc độ)
LINEAR_UNITS = {"cm": 1.0, "mm": 0.1, "m": 100.0, "tấc": 10.0}
ANGLE_UNITS = {"độ": 1.0, "vòng": 360.0}

# Số
DIGITS = {
    "không": 0, "một": 1, "mốt": 1, "hai": 2, "ba": 3, "bốn": 4, "tư": 4,
    "năm": 5, "lăm": 5, "nhăm": 5, "sáu": 6, "bảy": 7, "tám": 8, "chín": 9,
}
NUMBER_WORDS = {"mười", "mươi", "trăm", "nghìn", "linh", "lẻ", "rưỡi", "phẩy", "nửa"}

CANONICAL = (set(ACTIONS) | set(DIRECTIONS) | set(LINEAR_UNITS) | set(ANGLE_UNITS) | set(DIGITS) | NUMBER_WORDS)

# Từ khoá được phép sửa "gần đúng" (sai 1 ký tự). KHÔNG đưa chữ số ngắn vào (ba/bà/bá dễ sửa nhầm)
FUZZY_TARGETS = ("tiến", "lùi", "xoay", "quay", "trái", "phải", "dừng", "mét", "vòng", "trăm", "mười")

# ------------------------------------------------------------------ từ đệm (bỏ)
FILLERS = {
    "đi", "lên", "tới", "tiếp", "lại", "qua", "sang", "bên", "về", "phía", "hướng", "góc", "chút", "xíu",
    "khoảng", "chừng", "tầm", "cỡ", "độ_chừng", "thêm", "nữa", "đúng",
    "xe", "robot", "rô", "bốt", "ơi", "nhé", "nha", "nhá", "đấy", "đó", "này", "cho", "tôi", "mình", "em",
    "anh", "chị", "hãy", "vui", "lòng", "giúp", "cái", "ra", "vào", "và", "rồi", "xong", "sau", "đó",
    "với", "thì", "là", "ạ", "à", "ừ", "ờ", "ừm", "hả", "nào", "the", "a", "uh", "um",
}
# "sau" là từ đệm ("rồi sau đó"), nhưng "sáu" (6) bị bỏ dấu thành "sau": xử lý ở robot_commands
# (nằm giữa các từ số -> 6).

# ------------------------------------------------------------------ cụm từ (so khớp KHÔNG DẤU)
PHRASES = {
    # cm: "xăng ti mét", "xen ti mét", "cen ti mét", "phân"
    ("xen", "ti", "met"): "cm", ("xang", "ti", "met"): "cm", ("sen", "ti", "met"): "cm",
    ("cen", "ti", "met"): "cm", ("xen", "ti", "me"): "cm", ("xang", "ti", "me"): "cm",
    ("xen", "ti", "mec"): "cm", ("sang", "ti", "met"): "cm", ("xan", "ti", "met"): "cm",
    ("xen", "ty", "met"): "cm", ("xang", "ty", "met"): "cm", ("xen", "ti"): "cm", ("xang", "ti"): "cm",
    ("sen", "ti"): "cm", ("cen", "ti"): "cm", ("xen", "timet"): "cm", ("xen", "ti", "mét"): "cm",
    ("centi", "met"): "cm", ("xenti", "met"): "cm", ("c", "m"): "cm", ("xe", "em"): "cm",
    ("xi", "em"): "cm", ("xê", "em"): "cm",
    # mm
    ("mi", "li", "met"): "mm", ("mi", "li", "me"): "mm", ("mi", "li"): "mm", ("mili", "met"): "mm",
    ("mi", "ni", "met"): "mm", ("em", "em"): "mm",
    # tiến
    ("di", "thang"): "tiến", ("di", "toi"): "tiến", ("chay", "toi"): "tiến", ("tien", "len"): "tiến",
    ("tien", "toi"): "tiến", ("di", "len"): "tiến", ("chay", "thang"): "tiến", ("tien", "thang"): "tiến",
    ("di", "ve", "phia", "truoc"): "tiến", ("ve", "phia", "truoc"): "tiến", ("ra", "truoc"): "tiến",
    # lùi
    ("di", "lui"): "lùi", ("lui", "lai"): "lùi", ("chay", "lui"): "lùi", ("de", "lui"): "lùi",
    ("di", "ve", "phia", "sau"): "lùi", ("ve", "phia", "sau"): "lùi", ("ra", "sau"): "lùi",
    ("lui", "ve"): "lùi", ("nui", "lai"): "lùi",
    # quay đầu
    ("quay", "dau"): "quayđầu", ("quay", "dau", "lai"): "quayđầu", ("xoay", "dau"): "quayđầu",
    ("quai", "dau"): "quayđầu", ("quay", "lau"): "quayđầu", ("quay", "nau"): "quayđầu",
    # dừng
    ("dung", "lai"): "dừng", ("dung", "yen"): "dừng", ("ngung", "lai"): "dừng", ("thoi", "dung"): "dừng",
    ("dung", "xe"): "dừng", ("phanh", "lai"): "dừng", ("thang", "lai"): "dừng", ("dung", "ngay"): "dừng",
    ("ngung", "ngay"): "dừng", ("dung", "het"): "dừng", ("huy", "lenh"): "dừng", ("dung", "lai", "ngay"): "dừng",
    # một vòng / nửa vòng
    ("mot", "vong"): ("một", "vòng"), ("nua", "vong"): ("nửa", "vòng"),
    # bỏ hẳn (chuỗi rỗng)
    ("chinh", "xac"): "", ("mot", "chut"): "", ("mot", "xiu"): "", ("sau", "do"): "",
    # "rưỡi" hay bị tách
    ("ruoi", "i"): "rưỡi",
}

# ------------------------------------------------------------------ lỗi đã biết (khớp đúng dấu)
CONFUSIONS = {
    # --- tiến
    "tiếng": "tiến", "tiền": "tiến", "tiên": "tiến", "tiễn": "tiến", "tiện": "tiến", "tiển": "tiến",
    "tến": "tiến", "tín": "tiến", "tiếm": "tiến", "tiệm": "tiến", "tiềm": "tiến", "kiến": "tiến",
    "chiến": "tiến", "thiến": "tiến", "tiếc": "tiến", "tiết": "tiến", "tiêng": "tiến", "tiềng": "tiến",
    "tién": "tiến", "tíên": "tiến", "tíen": "tiến", "chiếng": "tiến",
    "điến": "tiến", "tiến.": "tiến", "tuyến": "tiến",
    # --- lùi
    "lui": "lùi", "lũi": "lùi", "lụi": "lùi", "lủi": "lùi", "lúi": "lùi", "lùy": "lùi", "lũy": "lùi",
    "lụy": "lùi", "lúy": "lùi", "lủy": "lùi", "nùi": "lùi", "nui": "lùi", "nụi": "lùi", "núi": "lùi",
    "dùi": "lùi", "rùi": "lùi", "lồi": "lùi", "lùn": "lùi", "lùng": "lùi", "luồi": "lùi", "lùi.": "lùi",
    "nùy": "lùi", "lui.": "lùi", "rui": "lùi", "lòi": "lùi",
    # --- xoay
    "soay": "xoay", "xoai": "xoay", "xoài": "xoay", "xoái": "xoay", "xoáy": "xoay", "xoạy": "xoay",
    "xoãy": "xoay", "xoảy": "xoay", "xòay": "xoay", "xóay": "xoay", "soài": "xoay", "soái": "xoay",
    "soai": "xoay", "xuay": "xoay", "xoe": "xoay", "khoay": "xoay", "choay": "xoay", "zoay": "xoay",
    "xoan": "xoay", "xoăn": "xoay", "hoay": "xoay", "xoày": "xoay", "xoạ": "xoay",
    # --- quay
    "qoay": "quay", "quai": "quay", "quây": "quay", "quày": "quay", "quầy": "quay", "quạy": "quay",
    "quẩy": "quay", "quảy": "quay", "quáy": "quay", "goay": "quay", "guay": "quay", "uay": "quay",
    "quoay": "quay", "quậy": "quay", "quai.": "quay", "wuay": "quay", "quăy": "quay",
    # --- rẽ
    "rẻ": "rẽ", "rể": "rẽ", "rễ": "rẽ", "rệ": "rẽ", "rè": "rẽ", "dẽ": "rẽ", "dẻ": "rẽ", "giẽ": "rẽ",
    "giẻ": "rẽ", "rẹ": "rẽ", "rế": "rẽ", "dễ": "rẽ",
    # --- trái
    "chái": "trái", "trai": "trái", "trải": "trái", "trại": "trái", "trãi": "trái", "trài": "trái",
    "chải": "trái", "chài": "trái", "chai": "trái", "chại": "trái", "trá": "trái", "tái": "trái",
    "trấy": "trái", "chấy": "trái", "trây": "trái", "chày": "trái", "trái.": "trái",
    "thái": "trái", "trao": "trái",
    # --- phải
    "phái": "phải", "phai": "phải", "phại": "phải", "phãi": "phải", "phả": "phải", "phày": "phải",
    "fải": "phải", "fai": "phải", "pải": "phải", "phải.": "phải", "phảy": "phải", "phay": "phải",
    "phài": "phải", "phãy": "phải", "phẩi": "phải", "phái.": "phải",
    "phá": "phải",
    # --- dừng
    "dừn": "dừng", "dưng": "dừng", "dửng": "dừng", "dựng": "dừng", "giừng": "dừng", "rừng": "dừng",
    "dứng": "dừng", "đừng": "dừng", "ngừng": "dừng", "ngưng": "dừng", "stop": "dừng", "thôi": "dừng",
    "dừng.": "dừng", "dững": "dừng", "giựng": "dừng", "rựng": "dừng", "đứng": "dừng", "dừm": "dừng",
    "xtốp": "dừng", "tốp": "dừng", "phanh": "dừng",
    # --- độ
    "đồ": "độ", "đổ": "độ", "đỗ": "độ", "đọ": "độ", "đô": "độ", "đố": "độ", "độc": "độ", "đột": "độ",
    "đội": "độ", "đậu": "độ", "đợ": "độ", "đò": "độ", "độ.": "độ", "dộ": "độ", "đôộ": "độ", "đ": "độ",
    "°": "độ",
    # --- mét / cm / tấc / vòng
    "mét": "m", "mẹt": "m", "méc": "m", "mếch": "m", "mết": "m", "mệt": "m", "mét.": "m", "mếc": "m",
    "mẹc": "m", "mê": "m", "met": "m", "mít": "m", "mắt": "m", "mấc": "m", "mết.": "m", "mè": "m",
    "phân": "cm", "phấn": "cm", "phần": "cm", "phận": "cm", "phăn": "cm", "xăng": "cm", "xen": "cm",
    "xentimet": "cm", "centimet": "cm", "centimét": "cm", "xentimét": "cm", "xăngtimét": "cm",
    "xăngtimet": "cm", "cm.": "cm", "xenti": "cm", "centi": "cm",
    "milimet": "mm", "milimét": "mm", "mi-li-mét": "mm",
    "tất": "tấc", "tắc": "tấc", "tậc": "tấc", "tấc.": "tấc", "tắt": "tấc",
    "vồng": "vòng", "vong": "vòng", "vọng": "vòng", "võng": "vòng", "vỏng": "vòng", "vóng": "vòng",
    "vòng.": "vòng", "dòng": "vòng", "giòng": "vòng", "vonng": "vòng", "vùng": "vòng",
    "nưả": "nửa", "nủa": "nửa", "nựa": "nửa", "nứa": "nửa",
    # --- số
    "mộ": "một", "mọt": "một", "mốc": "một", "mục": "một", "mồ": "một", "mổ": "một",
    "hài": "hai", "hái": "hai", "hại": "hai", "hay": "hai", "hây": "hai", "hãi": "hai",
    "bà": "ba", "bá": "ba", "bả": "ba", "bạ": "ba", "bã": "ba", "pa": "ba",
    "bón": "bốn", "bổn": "bốn", "bộn": "bốn", "bôn": "bốn", "bống": "bốn", "bốm": "bốn", "bốnn": "bốn",
    "nam": "năm", "nằm": "năm", "nắm": "năm", "nặm": "năm", "năn": "năm", "lam": "lăm", "lắm": "lăm",
    "nhâm": "nhăm", "nhằm": "nhăm",
    "xáu": "sáu", "sấu": "sáu", "sáo": "sáu", "xấu": "sáu", "sạu": "sáu", "sắu": "sáu",
    "bẩy": "bảy", "bẫy": "bảy", "bày": "bảy", "bãy": "bảy", "bái": "bảy", "bậy": "bảy", "bạy": "bảy",
    "tam": "tám", "tắm": "tám", "tạm": "tám", "tấm": "tám", "thám": "tám", "tảm": "tám", "táp": "tám",
    "chính": "chín", "chinh": "chín", "chìn": "chín", "trín": "chín", "chịn": "chín", "chím": "chín",
    "chính.": "chín",
    "mừi": "mười", "mưới": "mười", "mượi": "mười", "muời": "mười", "mườì": "mười", "mười.": "mười",
    "mừơi": "mười", "mừời": "mười", "muoi": "mười", "mươì": "mươi", "mướt": "mươi", "mươn": "mươi",
    "chăm": "trăm", "tram": "trăm", "trâm": "trăm", "trầm": "trăm", "trắm": "trăm", "chằm": "trăm",
    "trạm": "trăm", "tràm": "trăm", "trẵm": "trăm", "chẳm": "trăm", "trăng": "trăm", "trằm": "trăm",
    "ngàn": "nghìn", "nghìn.": "nghìn", "nghin": "nghìn", "ngìn": "nghìn",
    "lẽ": "lẻ", "lẹ": "lẻ", "lẹt": "lẻ", "lình": "linh",
    "rưởi": "rưỡi", "dưỡi": "rưỡi", "rưới": "rưỡi", "rượi": "rưỡi", "ruỗi": "rưỡi", "rưỡi.": "rưỡi",
    "rửi": "rưỡi", "giưỡi": "rưỡi",
    "hông": "không", "khôn": "không", "khong": "không", "khổng": "không",
    "phẩy": "phẩy", "phẫy": "phẩy", "chấm": "phẩy",
}

# Bỏ dấu ra trùng nhiều từ chuẩn: chọn theo ý nghĩa thường gặp trong lệnh xe
STRIPPED_PREFERRED = {
    "muoi": "mười",     # mười / mươi: robot_commands tự đổi thành "mươi" khi đứng sau một chữ số
    "nam": "năm",
    "sau": "sáu",       # chỉ dùng khi đứng giữa các từ số (xem robot_commands)
    "do": "độ",
    "tam": "tám",
    "bay": "bảy",
    "lam": "lăm",
    "tu": "tư",
    "mot": "một",
    "dung": "dừng",
    "le": "lẻ",
    "nua": "nửa",
    "met": "m",
    "phay": "phải",
}
