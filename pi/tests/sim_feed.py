"""Sinh dữ liệu giả (IMU + bánh xe + camera nhìn bức tường) để thử ovw_node.

  python3 tests/sim_feed.py | ovw_node config/estimator_config.yaml config/wheel.yaml > out.jsonl

Xe cân bằng (thân chúi qua lại ±0.3°), đứng yên 4 s, tăng tốc lên 0.15 m/s,
chạy 6 s rồi dừng. Số bước bánh mô phỏng đúng như firmware: động cơ quay bánh
SO VỚI THÂN, nên steps = (x - r*θ) * STEPS_PER_METER.
"""
import argparse
import math
import struct
import sys

import cv2
import numpy as np

FX = FY = 635.0
CX, CY = 320.0, 240.0
W, H = 640, 480
R_CtoI = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)
P_CinI = np.array([0.03, 0.0, 0.10])
P_AXLE_IN_I = np.array([0.0, 0.0, -0.05])  # trục bánh thấp hơn IMU 5 cm
WALL_X, RES = 2.5, 0.004
SPM, RW, G = 15670.64, 0.0325, 9.78
rng = np.random.default_rng(1)
tex = (rng.random((90, 150)) * 255).astype(np.uint8)
tex = cv2.GaussianBlur(cv2.resize(tex, (1500, 900), interpolation=cv2.INTER_NEAREST), (0, 0), 1.2)
uu, vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
rays_c = np.stack([(uu - CX) / FX, (vv - CY) / FY, np.ones_like(uu)], -1)  # H W 3


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def pitch(t):  # rad, + = chúi tới (quay dương quanh trục Y chỉ sang trái)
    return math.radians(0.3) * math.sin(2 * math.pi * 1.3 * t)


def axle_x(t):
    # tích phân tốc độ: 0 (t<4), dốc lên 4..5, 0.15 tới 11, dốc xuống 11..12
    if t < 4: return 0.0
    if t < 5: return 0.075 * (t - 4) ** 2
    if t < 11: return 0.075 + 0.15 * (t - 5)
    if t < 12: return 0.975 + 0.15 * (t - 11) - 0.075 * (t - 11) ** 2
    return 1.05


def imu_pose(t):
    R = Ry(pitch(t))  # R_ItoG (xe không quay đầu)
    p_axle = np.array([axle_x(t), 0.0, RW])
    p_I = p_axle - R @ P_AXLE_IN_I
    return R, p_I


def render(t):
    R, p_I = imu_pose(t)
    Rc = R @ R_CtoI
    cam = p_I + R @ P_CinI
    d = rays_c @ Rc.T
    s = (WALL_X - cam[0]) / d[..., 0]
    yw = cam[1] + s * d[..., 1]
    zw = cam[2] + s * d[..., 2]
    img = cv2.remap(tex, ((yw + 3.0) / RES).astype(np.float32), ((1.8 - zw) / RES).astype(np.float32),
                    cv2.INTER_LINEAR)
    return np.clip(img + rng.normal(0, 1.5, img.shape), 0, 255).astype(np.uint8)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=15.0)
    args = ap.parse_args()
    out = sys.stdout.buffer
    dt, t0, h = 0.005, 1000.0, 1e-3
    for k in range(int(args.secs / dt)):
        t = k * dt
        R, p = imu_pose(t)
        a_G = (imu_pose(t + h)[1] - 2 * p + imu_pose(t - h)[1]) / h ** 2
        acc = R.T @ (a_G + np.array([0, 0, G])) + rng.normal(0, 0.05, 3)
        th_dot = (pitch(t + h) - pitch(t - h)) / (2 * h)
        gyr = np.array([0.0, th_dot, 0.0]) + rng.normal(0, 0.003, 3)
        steps = int(round((axle_x(t) - RW * pitch(t)) * SPM))
        out.write(b"I" + struct.pack("<d3f3f2iB", t0 + t, *acc, *gyr, steps, steps, 2))
        if k % 10 == 0:
            out.write(b"C" + struct.pack("<dHH", t0 + t, W, H) + render(t).tobytes())
    print(f"quang duong that = {axle_x(args.secs):.3f} m", file=sys.stderr)
    out.write(b"Q")


if __name__ == "__main__":
    main()
