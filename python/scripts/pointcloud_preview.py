"""Live stereo point-cloud preview.

Grabs a frame from the MMlove stereo camera, computes a full-resolution
disparity map with SGBM, reprojects it to 3D using the calibrated ``Q``
matrix, and visualises the colored point cloud with Open3D.

Two modes:

* Default (snapshot): capture one averaged frame, open a blocking Open3D
  window. Close it to exit.
* ``--live``: keep grabbing frames and refreshing the point cloud in a
  non-blocking Open3D window. Press ``q`` in the window to quit.

Install Open3D (not a core project dep — too heavy for the Pi)::

    uv pip install open3d

Usage::

    uv run python scripts/pointcloud_preview.py              # snapshot
    uv run python scripts/pointcloud_preview.py --live       # live refresh
    uv run python scripts/pointcloud_preview.py --save out.ply
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_FILE = (
    Path(__file__).resolve().parent.parent
    / "calibration"
    / "intrinsic"
    / "stereo_calibration.npz"
)

# Trim points farther than this — stereo falls apart past ~2 m and the
# far points just clutter the view.
Z_MIN_MM = 100.0
Z_MAX_MM = 1500.0


def _import_open3d():
    try:
        import open3d as o3d  # noqa: F401

        return o3d
    except ImportError:
        sys.exit(
            "open3d is not installed. Install it with:\n"
            "    uv pip install open3d"
        )


def _build_matcher(num_disp: int) -> cv2.StereoSGBM:
    block = 5
    return cv2.StereoSGBM_create(
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


def _rectify_pair(frame, maps):
    mid = frame.shape[1] // 2
    left_raw, right_raw = frame[:, :mid], frame[:, mid:]
    left = cv2.remap(left_raw, maps[0], maps[1], cv2.INTER_LINEAR)
    right = cv2.remap(right_raw, maps[2], maps[3], cv2.INTER_LINEAR)
    return left, right


def _compute_cloud(
    left_rect: np.ndarray,
    right_rect: np.ndarray,
    matcher: cv2.StereoSGBM,
    Q: np.ndarray,
    *,
    z_min_mm: float = Z_MIN_MM,
    z_max_mm: float = Z_MAX_MM,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (points_mm, colors_rgb_float01) masked to a sane depth range."""
    left_gray = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
    disparity = matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0

    # reprojectImageTo3D returns Z in the same units as baseline (metres here);
    # we want millimetres to match the rest of the pipeline.
    points_m = cv2.reprojectImageTo3D(disparity, Q)
    points_mm = points_m * 1000.0

    z = points_mm[..., 2]
    mask = (disparity > 0) & np.isfinite(z) & (z > z_min_mm) & (z < z_max_mm)

    pts = points_mm[mask]
    colors = cv2.cvtColor(left_rect, cv2.COLOR_BGR2RGB)[mask].astype(np.float32) / 255.0
    return pts, colors


def _open_camera(index: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    if not cap.isOpened():
        sys.exit(f"Cannot open camera at index {index}")
    return cap


def snapshot_mode(
    cap: cv2.VideoCapture,
    maps,
    matcher: cv2.StereoSGBM,
    Q: np.ndarray,
    *,
    z_min_mm: float,
    z_max_mm: float,
    warmup: int = 5,
    save_path: Path | None = None,
) -> None:
    o3d = _import_open3d()

    # Discard a handful of frames so auto-exposure settles.
    for _ in range(warmup):
        cap.read()
        time.sleep(0.03)

    ok, frame = cap.read()
    if not ok:
        sys.exit("Failed to grab frame from camera")

    left, right = _rectify_pair(frame, maps)
    pts, colors = _compute_cloud(left, right, matcher, Q, z_min_mm=z_min_mm, z_max_mm=z_max_mm)
    print(f"Captured {len(pts):,} points")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_point_cloud(str(save_path), pcd)
        print(f"Saved -> {save_path}")

    # Add a small coordinate frame (100 mm axes) so orientation is obvious.
    frame_geom = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100.0)
    o3d.visualization.draw_geometries(
        [pcd, frame_geom], window_name="Stereo point cloud (close to exit)"
    )


def live_mode(
    cap: cv2.VideoCapture,
    maps,
    matcher: cv2.StereoSGBM,
    Q: np.ndarray,
    *,
    z_min_mm: float,
    z_max_mm: float,
) -> None:
    o3d = _import_open3d()

    pcd = o3d.geometry.PointCloud()
    # Seed with one frame so the viewer has something to render.
    ok, frame = cap.read()
    if not ok:
        sys.exit("Failed to grab initial frame")
    left, right = _rectify_pair(frame, maps)
    pts, colors = _compute_cloud(left, right, matcher, Q, z_min_mm=z_min_mm, z_max_mm=z_max_mm)
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Stereo point cloud (q to quit)")
    vis.add_geometry(pcd)
    vis.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=100.0))

    print("Streaming point cloud — press 'q' in the viewer to quit.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            left, right = _rectify_pair(frame, maps)
            pts, colors = _compute_cloud(
                left, right, matcher, Q, z_min_mm=z_min_mm, z_max_mm=z_max_mm
            )
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd.colors = o3d.utility.Vector3dVector(colors)
            vis.update_geometry(pcd)
            if not vis.poll_events():
                break
            vis.update_renderer()
    finally:
        vis.destroy_window()


def main() -> None:
    parser = argparse.ArgumentParser(description="Live stereo point-cloud preview")
    parser.add_argument("-i", "--index", type=int, default=0, help="Camera device index")
    parser.add_argument("--cal", type=str, default=str(CALIBRATION_FILE))
    parser.add_argument("--num-disp", type=int, default=192)
    parser.add_argument("--live", action="store_true", help="Continuously refresh")
    parser.add_argument("--save", type=str, default=None,
                        help="Save snapshot to this .ply path (snapshot mode only)")
    parser.add_argument("--z-min", type=float, default=Z_MIN_MM)
    parser.add_argument("--z-max", type=float, default=Z_MAX_MM)
    args = parser.parse_args()

    cal = np.load(args.cal)
    maps = (cal["map_left_1"], cal["map_left_2"], cal["map_right_1"], cal["map_right_2"])
    Q = cal["Q"]
    matcher = _build_matcher(args.num_disp)

    cap = _open_camera(args.index)
    try:
        if args.live:
            live_mode(cap, maps, matcher, Q, z_min_mm=args.z_min, z_max_mm=args.z_max)
        else:
            snapshot_mode(
                cap,
                maps,
                matcher,
                Q,
                z_min_mm=args.z_min,
                z_max_mm=args.z_max,
                save_path=Path(args.save) if args.save else None,
            )
    finally:
        cap.release()


if __name__ == "__main__":
    main()
