"""
Live depth preview — shows the left camera view with a crosshair
at the center and the estimated depth of that pixel.

Usage:
    python depth_preview.py              # default: half-res, min ~148mm
    python depth_preview.py --scale 1.0  # full res, min ~222mm
    python depth_preview.py --scale 0.5 --num-disp 256  # min ~111mm

Keys:
    d  — toggle disparity color map overlay
    q  — quit
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_FILE = Path(__file__).resolve().parent.parent / "calibration" / "intrinsic" / "stereo_calibration.npz"


def main() -> None:
    parser = argparse.ArgumentParser(description="Live stereo depth preview")
    parser.add_argument("-i", "--index", type=int, default=0)
    parser.add_argument("--cal", type=str, default=str(CALIBRATION_FILE))
    parser.add_argument("--scale", type=float, default=0.5,
                        help="Downscale factor for stereo matching (0.5 = half res)")
    parser.add_argument("--num-disp", type=int, default=192,
                        help="numDisparities for SGBM (must be multiple of 16)")
    args = parser.parse_args()

    cal = np.load(args.cal)
    map_l1, map_l2 = cal["map_left_1"], cal["map_left_2"]
    map_r1, map_r2 = cal["map_right_1"], cal["map_right_2"]
    Q = cal["Q"]

    focal_full = abs(Q[2, 3])
    baseline = abs(1.0 / Q[3, 2])
    focal = focal_full * args.scale
    num_disp = args.num_disp
    min_depth_mm = focal * baseline / num_disp * 1000
    print(f"scale={args.scale}  focal={focal:.0f}px  baseline={baseline*1000:.1f}mm  "
          f"numDisp={num_disp}  -> min depth ≈ {min_depth_mm:.0f} mm")

    block = 5
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block,
        P1=8 * 3 * block * block,
        P2=32 * 3 * block * block,
        disp12MaxDiff=1,
        uniquenessRatio=5,
        speckleWindowSize=200,
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )

    cap = cv2.VideoCapture(args.index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    show_disparity_overlay = False

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        h, w = frame.shape[:2]
        mid = w // 2
        left_raw = frame[:, :mid]
        right_raw = frame[:, mid:]

        left_rect = cv2.remap(left_raw, map_l1, map_l2, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right_raw, map_r1, map_r2, cv2.INTER_LINEAR)

        if args.scale != 1.0:
            left_sm = cv2.resize(left_rect, None, fx=args.scale, fy=args.scale)
            right_sm = cv2.resize(right_rect, None, fx=args.scale, fy=args.scale)
        else:
            left_sm, right_sm = left_rect, right_rect

        left_gray = cv2.cvtColor(left_sm, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_sm, cv2.COLOR_BGR2GRAY)
        disparity = stereo.compute(left_gray, right_gray).astype(np.float32) / 16.0

        display = left_rect.copy()

        if show_disparity_overlay:
            disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disp_color = cv2.applyColorMap(np.uint8(disp_norm), cv2.COLORMAP_JET)
            disp_color = cv2.resize(disp_color, (display.shape[1], display.shape[0]))
            display = cv2.addWeighted(display, 0.5, disp_color, 0.5, 0)

        sh, sw = left_sm.shape[:2]
        cy_sm, cx_sm = sh // 2, sw // 2
        cy_disp = int(cy_sm / args.scale)
        cx_disp = int(cx_sm / args.scale)
        size = 15
        cv2.line(display, (cx_disp - size, cy_disp), (cx_disp + size, cy_disp), (0, 255, 0), 1)
        cv2.line(display, (cx_disp, cy_disp - size), (cx_disp, cy_disp + size), (0, 255, 0), 1)

        patch = disparity[cy_sm - 3 : cy_sm + 4, cx_sm - 3 : cx_sm + 4]
        valid = patch[patch > 0]
        if len(valid) > 0:
            d = float(np.median(valid))
            depth_mm = (focal * baseline) / d * 1000
            label = f"{depth_mm:.0f} mm"
            color = (0, 255, 0)
        else:
            label = "---"
            color = (0, 0, 255)

        cv2.putText(display, label, (cx_disp + 20, cy_disp - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3)
        cv2.putText(display, label, (cx_disp + 20, cy_disp - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        cv2.imshow("Depth Preview", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("d"):
            show_disparity_overlay = not show_disparity_overlay

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
