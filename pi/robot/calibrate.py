"""Hiệu chỉnh nội tham số camera bằng bàn cờ (in tấm 9x6 ô vuông trong, ô 25 mm).

  sudo apt install python3-opencv   (đã có trong install_deps.sh)
  systemctl --user stop robotd      (giải phóng camera)
  cd ~/iot-ck/pi && ../.venv/bin/python -m robot.calibrate --square 0.025

Camera hướng lên trần: cầm bàn cờ phía trên camera, nghiêng/xoay nhiều góc, phủ khắp khung hình
(cả góc ảnh). Chương trình tự chụp khi thấy bàn cờ ở tư thế mới, đủ 25 ảnh thì in kết quả để
dán vào config/slam.yaml (mục camera) rồi đặt calibrated.camera_intrinsics: true.
"""
import argparse
import time

import numpy as np
import yaml


def main() -> int:
    import cv2
    from .camera import Camera
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=9, help="số góc trong theo chiều ngang")
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.025, help="cạnh ô (m)")
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--config", default="config/robot.yaml")
    args = ap.parse_args()
    cc = yaml.safe_load(open(args.config))["camera"]
    cam = Camera(cc["width"], cc["height"], cc["fps"], cc["sensor_size"], cc["exposure_us"], cc["gain"]).start()
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    objs, imgs, centers = [], [], []
    last = 0.0
    print("Đưa bàn cờ vào khung hình...")
    while len(imgs) < args.n:
        time.sleep(0.1)
        g = cam.latest_gray
        if g is None or time.monotonic() - last < 0.7:
            continue
        ok, corners = cv2.findChessboardCorners(g, pattern, cv2.CALIB_CB_FAST_CHECK)
        if not ok:
            continue
        c = corners.reshape(-1, 2).mean(0)
        size = np.ptp(corners.reshape(-1, 2), axis=0).prod()
        if any(np.linalg.norm(c - c0) < 40 and abs(size - s0) / s0 < 0.2 for c0, s0 in centers):
            continue  # giống ảnh đã có
        corners = cv2.cornerSubPix(g, corners, (5, 5), (-1, -1),
                                   (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        objs.append(objp); imgs.append(corners); centers.append((c, size)); last = time.monotonic()
        print(f"  chụp {len(imgs)}/{args.n}")
    cam.stop()
    h, w = g.shape
    rms, K, D, _, _ = cv2.calibrateCamera(objs, imgs, (w, h), None, None, flags=cv2.CALIB_FIX_K3)
    D = D.ravel()
    print(f"\nRMS lỗi chiếu lại: {rms:.3f} px (tốt nếu < 0.5)")
    print("Dán vào config/slam.yaml, mục camera:")
    print(f"  width: {w}")
    print(f"  height: {h}")
    print(f"  fx: {K[0,0]:.3f}")
    print(f"  fy: {K[1,1]:.3f}")
    print(f"  cx: {K[0,2]:.3f}")
    print(f"  cy: {K[1,2]:.3f}")
    print(f"  dist: [{D[0]:.6f}, {D[1]:.6f}, {D[2]:.6f}, {D[3]:.6f}, {D[4] if len(D) > 4 else 0.0:.6f}]")
    print("rồi đặt calibrated.camera_intrinsics: true và khởi động lại robotd")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
