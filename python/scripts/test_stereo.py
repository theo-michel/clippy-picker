"""
MMlove Stereo Camera — test script

Plug in the camera, then run:
    python test_stereo.py

Keys:
    s  — save a stereo snapshot (left + right) to disk
    c  — capture a calibration frame (saves to calibration_frames/)
    d  — toggle disparity map preview (StereoSGBM)
    q  — quit
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def find_stereo_camera(preferred_index: int | None = None) -> int:
    """Try to find the stereo camera by index or brute-force scan."""
    if preferred_index is not None:
        cap = cv2.VideoCapture(preferred_index)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                return preferred_index
        print(f"[!] Index {preferred_index} didn't work, scanning...")

    print("Scanning video devices 0-9 ...")
    stereo_idx = -1
    for idx in range(10):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            continue
        ret, frame = cap.read()
        w_prop = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h_prop = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()
        if not ret or frame is None:
            print(
                f"  index {idx}: opened but read() failed ({w_prop}x{h_prop}) "
                "— permissions issue or device busy"
            )
            continue
        h, w = frame.shape[:2]
        aspect = w / h
        tag = " <-- likely stereo" if aspect > 2.0 else ""
        print(f"  index {idx}: {w}x{h}  aspect={aspect:.2f}{tag}")
        if aspect > 2.0 and stereo_idx < 0:
            stereo_idx = idx
    print()
    return stereo_idx


def compute_disparity(left_gray: np.ndarray, right_gray: np.ndarray) -> np.ndarray:
    """Compute a normalized disparity map using StereoSGBM."""
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,
        blockSize=9,
        P1=8 * 9 * 9,
        P2=32 * 9 * 9,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
    )
    disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0
    disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
    return np.uint8(disp_norm)


def main() -> None:
    parser = argparse.ArgumentParser(description="MMlove stereo camera test")
    parser.add_argument(
        "-i", "--index", type=int, default=0, help="Video device index (default 0)"
    )
    parser.add_argument(
        "-W",
        "--width",
        type=int,
        default=2560,
        help="Capture width (both eyes combined, default 2560)",
    )
    parser.add_argument(
        "-H", "--height", type=int, default=720, help="Capture height (default 720)"
    )
    args = parser.parse_args()

    idx = args.index

    cap = cv2.VideoCapture(idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Opened device {idx}: {actual_w}x{actual_h}")

    if actual_w / actual_h < 2.0:
        print(
            "[warn] Aspect ratio < 2:1 — this might not be a stereo side-by-side feed."
        )
        print("       Try a different --index or check the camera connection.\n")

    snap_dir = Path("snapshots")
    cal_dir = Path("calibration_frames")
    show_disparity = False
    snap_count = 0
    cal_count = 0

    print("\nKeys:  s=snapshot  c=calibration-frame  d=disparity-toggle  q=quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[!] Failed to read frame")
            time.sleep(0.1)
            continue

        h, w = frame.shape[:2]
        mid = w // 2
        left = frame[:, :mid]
        right = frame[:, mid:]

        # Display side-by-side with divider
        display = frame.copy()
        cv2.line(display, (mid, 0), (mid, h), (0, 255, 0), 1)
        cv2.putText(
            display, "LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.putText(
            display,
            "RIGHT",
            (mid + 10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        scale = min(1280 / w, 720 / h, 1.0)
        if scale < 1.0:
            display = cv2.resize(display, None, fx=scale, fy=scale)
        cv2.imshow("MMlove Stereo", display)

        if show_disparity:
            left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            disp = compute_disparity(left_gray, right_gray)
            disp_color = cv2.applyColorMap(disp, cv2.COLORMAP_JET)
            if scale < 1.0:
                disp_color = cv2.resize(disp_color, None, fx=scale, fy=scale)
            cv2.imshow("Disparity (uncalibrated)", disp_color)
        else:
            cv2.destroyWindow("Disparity (uncalibrated)")

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            snap_dir.mkdir(exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            cv2.imwrite(str(snap_dir / f"stereo_{ts}.png"), frame)
            cv2.imwrite(str(snap_dir / f"left_{ts}.png"), left)
            cv2.imwrite(str(snap_dir / f"right_{ts}.png"), right)
            snap_count += 1
            print(f"[snapshot {snap_count}] saved to {snap_dir}/")
        elif key == ord("c"):
            cal_dir.mkdir(exist_ok=True)
            cv2.imwrite(str(cal_dir / f"left_{cal_count:03d}.png"), left)
            cv2.imwrite(str(cal_dir / f"right_{cal_count:03d}.png"), right)
            cal_count += 1
            print(f"[calibration frame {cal_count}] saved to {cal_dir}/")
        elif key == ord("d"):
            show_disparity = not show_disparity
            print(f"Disparity {'ON' if show_disparity else 'OFF'}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
