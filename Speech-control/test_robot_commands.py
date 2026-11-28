"""Kiểm tra từ điển + bộ tách lệnh:  python3 test_robot_commands.py   (hoặc pytest)"""
import sys
import unittest

from robot_commands import parse, words_to_number

M, T, S = "move", "turn", "stop"

CASES = [
    # --- 4 lệnh chuẩn
    ("tiến 10", [(M, 10)]),
    ("lùi 25", [(M, -25)]),
    ("xoay trái 180", [(T, 180)]),
    ("xoay phải 90", [(T, -90)]),
    # --- số bằng chữ
    ("tiến mười", [(M, 10)]),
    ("tiến hai mươi lăm", [(M, 25)]),
    ("lùi năm mươi xăng ti mét", [(M, -50)]),
    ("xoay trái một trăm tám mươi độ", [(T, 180)]),
    ("xoay phải chín mươi độ", [(T, -90)]),
    ("quay phải ba trăm sáu mươi độ", [(T, -360)]),
    ("xoay trái bốn lăm độ", [(T, 45)]),
    ("tiến mười lăm phân", [(M, 15)]),
    ("tiến một trăm linh năm", [(M, 105)]),
    ("tiến hai mươi mốt", [(M, 21)]),
    ("xoay phải trăm tám", [(T, -180)]),
    ("tiến 10cm", [(M, 10)]),
    ("tiến 1,5 mét", [(M, 150)]),
    ("tiến một mét rưỡi", [(M, 150)]),
    ("tiến một mét hai", [(M, 120)]),
    ("lùi 2 tấc", [(M, -20)]),
    ("tiến một phẩy năm mét", [(M, 150)]),
    ("xoay trái nửa vòng", [(T, 180)]),
    ("xoay phải một vòng", [(T, -360)]),
    ("quay đầu", [(T, 180)]),
    # --- lỗi chính tả ASR
    ("tiếng mười", [(M, 10)]),
    ("tiền hai mươi", [(M, 20)]),
    ("tiên 30", [(M, 30)]),
    ("kiến năm mươi", [(M, 50)]),
    ("tien muoi", [(M, 10)]),
    ("lui hai muoi", [(M, -20)]),
    ("lụi mười", [(M, -10)]),
    ("nùi ba mươi", [(M, -30)]),
    ("soay chái chín mươi", [(T, 90)]),
    ("xoài trái 180", [(T, 180)]),
    ("xoáy phải 90", [(T, -90)]),
    ("quai phái chín mươi đồ", [(T, -90)]),
    ("xoay trai 90 do", [(T, 90)]),
    ("xoay phai 45", [(T, -45)]),
    ("quay phẩy chín mươi", [(T, -90)]),
    ("rẻ trái 90", [(T, 90)]),
    ("tiến hai mười", [(M, 20)]),
    ("tiến mừi lăm", [(M, 15)]),
    ("xoay trái một chăm tám mươi", [(T, 180)]),
    ("tiến nam mươi", [(M, 50)]),
    ("tiến sau mươi", [(M, 60)]),
    ("lùi bẩy mươi", [(M, -70)]),
    ("tiến tắm mươi", [(M, 80)]),
    ("xoay trái chính mươi độ", [(T, 90)]),
    ("tieng 20", [(M, 20)]),
    ("xoay phải 90°", [(T, -90)]),
    ("Tiến 10.", [(M, 10)]),
    ("TIẾN MƯỜI XĂNG TI MÉT", [(M, 10)]),
    ("tiến 10 xen ti mét", [(M, 10)]),
    # --- câu tự nhiên, từ đệm
    ("xe ơi tiến lên mười phân nhé", [(M, 10)]),
    ("đi thẳng hai mươi", [(M, 20)]),
    ("lùi lại khoảng ba mươi", [(M, -30)]),
    ("quay sang trái chín mươi độ", [(T, 90)]),
    ("xoay qua bên phải 45 độ", [(T, -45)]),
    ("xoay 90 độ sang trái", [(T, 90)]),
    ("tiến tầm năm mươi", [(M, 50)]),
    ("xoay phải đúng 90 độ", [(T, -90)]),
    # --- nhiều lệnh, dừng
    ("tiến 50 rồi xoay trái 90", [(M, 50), (T, 90)]),
    ("tiến năm mươi xoay phải chín mươi lùi mười", [(M, 50), (T, -90), (M, -10)]),
    ("dừng lại", [(S, 0)]),
    ("dừng", [(S, 0)]),
    ("đứng lại", [(S, 0)]),
    ("rừng lại", [(S, 0)]),
    # --- không phải lệnh / thiếu thông tin: không được chạy
    ("hôm nay trời đẹp", []),
    ("tiến", []),
    ("xoay 90", []),
    ("bật đèn", []),
    ("năm mươi", []),
    ("tiến 90 độ", []),
]


class ParseTest(unittest.TestCase):
    def test_cases(self):
        bad = []
        for text, want in CASES:
            r = parse(text)
            got = [(c.op, round(c.value, 3)) for c in r.commands]
            if got != [(o, float(v)) for o, v in want]:
                bad.append(f"{text!r}: muốn {want}, được {got}  (chuẩn hoá {r.normalized!r}, lỗi {r.errors})")
        self.assertFalse(bad, "\n" + "\n".join(bad))

    def test_numbers(self):
        for words, v in [("một trăm tám mươi", 180), ("hai trăm năm", 250), ("trăm rưỡi", 150),
                         ("ba mươi", 30), ("mười", 10), ("chín", 9), ("một nghìn", 1000), ("hai lăm", 25)]:
            self.assertEqual(words_to_number(words.split()), v, words)

    def test_messages(self):
        r = parse("tiến 10 xoay phải 90 dừng")
        self.assertEqual([c.to_message("x")["type"] for c in r.commands], ["move", "turn", "stop"])
        self.assertAlmostEqual(r.commands[0].to_message("x")["distance_m"], 0.10)
        self.assertEqual(r.commands[1].to_message("x")["angle_deg"], -90)


if __name__ == "__main__":
    sys.exit(unittest.main(verbosity=1))
