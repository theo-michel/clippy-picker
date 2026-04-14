"""
Stereo calibration from ChArUco board captures.

Board spec: 9x17 squares, checker=16mm, marker=12mm, DICT_4X4_50

Usage:
    1. Capture frames with test_stereo.py (press 'c')
    2. Run:  python calibrate_stereo.py
    3. Output: calibration_data/stereo_calibration.npz
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

SQUARES_X = 17
SQUARES_Y = 9
CHECKER_SIZE_MM = 16.0
MARKER_SIZE_MM = 12.0
ARUCO_DICT = cv2.aruco.DICT_4X4_50


def build_board() -> cv2.aruco.CharucoBoard:
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    return cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        CHECKER_SIZE_MM / 1000.0,
        MARKER_SIZE_MM / 1000.0,
        dictionary,
    )


def detect_charuco(
    img: np.ndarray,
    charuco_detector: cv2.aruco.CharucoDetector,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Detect ChArUco corners in a single image."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
    if charuco_ids is None or len(charuco_ids) < 6:
        return None, None
    return charuco_corners, charuco_ids


def calibrate_single(
    images: list[np.ndarray],
    board: cv2.aruco.CharucoBoard,
    charuco_detector: cv2.aruco.CharucoDetector,
    label: str,
) -> tuple[np.ndarray, np.ndarray, list, list]:
    """Calibrate a single camera from ChArUco detections."""
    all_corners, all_ids = [], []
    h, w = images[0].shape[:2]

    for i, img in enumerate(images):
        corners, ids = detect_charuco(img, charuco_detector)
        if corners is None:
            print(f"  [{label}] frame {i}: skipped (too few corners)")
            continue
        all_corners.append(corners)
        all_ids.append(ids)
        print(f"  [{label}] frame {i}: {len(ids)} corners")

    if len(all_corners) < 3:
        print(f"  [{label}] not enough valid frames ({len(all_corners)})")
        sys.exit(1)

    obj_pts, img_pts = [], []
    board_pts = board.getChessboardCorners()
    for corners, ids in zip(all_corners, all_ids):
        obj_pts.append(board_pts[ids.flatten()])
        img_pts.append(corners)

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, (w, h), None, None
    )
    print(f"  [{label}] RMS reprojection error: {ret:.4f}")
    return K, dist, obj_pts, img_pts


def main() -> None:
    parser = argparse.ArgumentParser(description="Stereo calibration from ChArUco frames")
    parser.add_argument("--frames", type=str, default="calibration_frames",
                        help="Directory with left_NNN.png / right_NNN.png")
    parser.add_argument("--out", type=str, default="calibration_data",
                        help="Output directory for calibration result")
    args = parser.parse_args()

    frames_dir = Path(args.frames)
    out_dir = Path(args.out)

    left_paths = sorted(frames_dir.glob("left_*.png"))
    right_paths = sorted(frames_dir.glob("right_*.png"))

    if len(left_paths) != len(right_paths) or len(left_paths) == 0:
        print(f"Found {len(left_paths)} left, {len(right_paths)} right frames in {frames_dir}/")
        print("Capture pairs with test_stereo.py first (press 'c').")
        sys.exit(1)

    print(f"Found {len(left_paths)} stereo pairs in {frames_dir}/\n")

    left_imgs = [cv2.imread(str(p)) for p in left_paths]
    right_imgs = [cv2.imread(str(p)) for p in right_paths]

    board = build_board()
    charuco_detector = cv2.aruco.CharucoDetector(board)

    # --- Individual calibration ---
    print("Calibrating left camera...")
    K_left, dist_left, obj_pts_l, img_pts_l = calibrate_single(
        left_imgs, board, charuco_detector, "L"
    )
    print("\nCalibrating right camera...")
    K_right, dist_right, obj_pts_r, img_pts_r = calibrate_single(
        right_imgs, board, charuco_detector, "R"
    )

    # --- Keep only pairs where both cameras had detections ---
    # Re-detect to build matched pairs
    paired_obj, paired_img_l, paired_img_r = [], [], []
    board_pts = board.getChessboardCorners()

    for i, (l_img, r_img) in enumerate(zip(left_imgs, right_imgs)):
        lc, li = detect_charuco(l_img, charuco_detector)
        rc, ri = detect_charuco(r_img, charuco_detector)
        if lc is None or rc is None:
            continue
        common_ids = np.intersect1d(li.flatten(), ri.flatten())
        if len(common_ids) < 6:
            continue
        l_mask = np.isin(li.flatten(), common_ids)
        r_mask = np.isin(ri.flatten(), common_ids)
        l_order = np.argsort(li.flatten()[l_mask])
        r_order = np.argsort(ri.flatten()[r_mask])

        paired_obj.append(board_pts[common_ids[np.argsort(common_ids)]])
        paired_img_l.append(lc[l_mask][l_order])
        paired_img_r.append(rc[r_mask][r_order])

    print(f"\n{len(paired_obj)} stereo pairs with matched corners")
    if len(paired_obj) < 3:
        print("Not enough matched pairs for stereo calibration.")
        sys.exit(1)

    h, w = left_imgs[0].shape[:2]
    ret, K_l, d_l, K_r, d_r, R, T, E, F = cv2.stereoCalibrate(
        paired_obj, paired_img_l, paired_img_r,
        K_left, dist_left, K_right, dist_right,
        (w, h),
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    print(f"\nStereo RMS reprojection error: {ret:.4f}")

    baseline_mm = np.linalg.norm(T) * 1000
    print(f"Baseline: {baseline_mm:.1f} mm")

    # --- Rectification ---
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K_l, d_l, K_r, d_r, (w, h), R, T, alpha=0
    )

    map_l1, map_l2 = cv2.initUndistortRectifyMap(K_l, d_l, R1, P1, (w, h), cv2.CV_16SC2)
    map_r1, map_r2 = cv2.initUndistortRectifyMap(K_r, d_r, R2, P2, (w, h), cv2.CV_16SC2)

    # --- Save ---
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / "stereo_calibration.npz"
    np.savez(
        str(out_file),
        K_left=K_l, dist_left=d_l,
        K_right=K_r, dist_right=d_r,
        R=R, T=T, E=E, F=F,
        R1=R1, R2=R2, P1=P1, P2=P2, Q=Q,
        roi_left=roi1, roi_right=roi2,
        map_left_1=map_l1, map_left_2=map_l2,
        map_right_1=map_r1, map_right_2=map_r2,
        image_size=np.array([w, h]),
    )
    print(f"\nSaved to {out_file}")

    # --- Show one rectified pair ---
    sample_l = cv2.remap(left_imgs[0], map_l1, map_l2, cv2.INTER_LINEAR)
    sample_r = cv2.remap(right_imgs[0], map_r1, map_r2, cv2.INTER_LINEAR)
    combo = np.hstack([sample_l, sample_r])
    for y in range(0, combo.shape[0], 40):
        cv2.line(combo, (0, y), (combo.shape[1], y), (0, 255, 0), 1)
    scale = min(1280 / combo.shape[1], 720 / combo.shape[0], 1.0)
    if scale < 1.0:
        combo = cv2.resize(combo, None, fx=scale, fy=scale)
    cv2.imshow("Rectified (horizontal lines should align)", combo)
    print("\nShowing rectified sample — press any key to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
