"""RealSense D456 live viewer with depth diagnostics.

Shows RGB feed and checks whether depth frames arrive alongside color.
If depth is available, displays center-pixel 3D coordinates and depth overlay.

Controls (click the window first):
  q / ESC  — quit
  s        — save snapshot
  d        — toggle depth colormap overlay
Or Ctrl+C in the terminal.
"""

import signal
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs  # type: ignore[import-untyped]


def log(msg: str) -> None:
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def find_device() -> rs.device:
    ctx = rs.context()
    try:
        devices = ctx.query_devices()
        if len(devices) == 0:
            log("ERROR: No RealSense device found.")
            sys.exit(1)
        dev = devices[0]
        for attr in ["name", "serial_number", "firmware_version", "usb_type_descriptor"]:
            try:
                log(f"  {attr}: {dev.get_info(getattr(rs.camera_info, attr))}")
            except Exception:
                pass
        return dev
    except RuntimeError as e:
        if "power state" in str(e).lower():
            log(f"ERROR: {e}")
            log("Run with sudo:  sudo uv run python scripts/realsense_camera.py")
            sys.exit(1)
        raise


def reset_device(device: rs.device) -> rs.device:
    log("Hardware reset...")
    device.hardware_reset()
    time.sleep(5)
    ctx = rs.context()
    for attempt in range(15):
        devices = ctx.query_devices()
        if len(devices) > 0:
            log(f"Device back online (attempt {attempt + 1})")
            return devices[0]
        time.sleep(1)
    log("ERROR: Device did not come back after reset")
    sys.exit(1)


STREAM_CONFIGS = [
    ("depth+color 640x480 @ 30fps", [
        (rs.stream.depth, 640, 480, rs.format.z16, 30),
        (rs.stream.color, 640, 480, rs.format.bgr8, 30),
    ]),
    ("color-only 1280x720 @ 30fps", [
        (rs.stream.color, 1280, 720, rs.format.bgr8, 30),
    ]),
]


def start_pipeline(max_attempts: int = 3):
    for cfg_name, streams in STREAM_CONFIGS:
        for attempt in range(1, max_attempts + 1):
            pipeline = rs.pipeline()
            config = rs.config()
            for s in streams:
                config.enable_stream(*s)
            try:
                log(f"[{cfg_name}] attempt {attempt}/{max_attempts}...")
                profile = pipeline.start(config)
                frames = pipeline.wait_for_frames(timeout_ms=3000)
                if frames.get_color_frame() or frames.get_depth_frame():
                    log(f"Streaming: {cfg_name}")
                    return pipeline, profile
                log("  Started but no frames.")
                pipeline.stop()
            except RuntimeError as e:
                log(f"  Failed: {e}")
            if attempt < max_attempts:
                time.sleep(2)
        log(f"  {cfg_name} exhausted — trying next config")

    log("ERROR: Could not start pipeline with any config")
    sys.exit(1)


def diagnose_depth(profile, frames) -> None:
    """Log depth stream availability and intrinsics."""
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        log("Depth: NOT available (color-only stream)")
        log("  -> pixel-to-3D deprojection will NOT work")
        return

    log(f"Depth: available! {depth_frame.get_width()}x{depth_frame.get_height()}")

    try:
        depth_profile = profile.get_stream(rs.stream.depth)
        intrin = depth_profile.as_video_stream_profile().get_intrinsics()
        log(f"  Intrinsics: {intrin.width}x{intrin.height}, "
            f"fx={intrin.fx:.1f}, fy={intrin.fy:.1f}, "
            f"cx={intrin.ppx:.1f}, cy={intrin.ppy:.1f}")
        log(f"  Model: {intrin.model}, coeffs: {intrin.coeffs}")
    except RuntimeError:
        log("  Intrinsics: not available from profile (implicit stream)")
        log("  Will use per-frame intrinsics instead")

    try:
        depth_sensor = profile.get_device().first_depth_sensor()
        log(f"  Depth scale: {depth_sensor.get_depth_scale():.6f} m/unit")
    except RuntimeError:
        pass

    cx, cy = depth_frame.get_width() // 2, depth_frame.get_height() // 2
    dist = depth_frame.get_distance(cx, cy)
    log(f"  Center pixel ({cx},{cy}) distance: {dist:.3f} m")

    try:
        intrin = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()
        point_3d = rs.rs2_deproject_pixel_to_point(intrin, [float(cx), float(cy)], dist)
        log(f"  Center 3D point: X={point_3d[0]:.3f}, Y={point_3d[1]:.3f}, Z={point_3d[2]:.3f} m")
        log("  -> pixel-to-3D deprojection WORKS")
    except Exception as e:
        log(f"  Deprojection failed: {e}")


def live_view() -> None:
    pipeline, profile = start_pipeline()

    # Check depth on the first few frames
    log("\nChecking depth availability...")
    frames = pipeline.wait_for_frames(timeout_ms=3000)
    diagnose_depth(profile, frames)

    has_depth = frames.get_depth_frame() is not None
    show_depth = False
    log("")

    window_name = "RealSense D456"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    frame_count = 0
    fps_start = time.monotonic()
    running = True

    depth_intrin = None
    if has_depth:
        try:
            depth_intrin = (frames.get_depth_frame()
                           .get_profile().as_video_stream_profile().get_intrinsics())
        except Exception:
            pass

    def on_sigint(_sig, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, on_sigint)
    controls = "q=quit, s=snap"
    if has_depth:
        controls += ", d=depth overlay"
    log(f"Live — {controls}")

    try:
        while running:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=500)
            except RuntimeError:
                continue

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame_count += 1
            color_image = np.asanyarray(color_frame.get_data())
            depth_frame = frames.get_depth_frame() if has_depth else None

            if show_depth and depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                )
                if depth_colormap.shape[:2] != color_image.shape[:2]:
                    depth_colormap = cv2.resize(
                        depth_colormap, (color_image.shape[1], color_image.shape[0]))
                display = np.hstack((color_image, depth_colormap))
            else:
                display = color_image

            # HUD
            elapsed = time.monotonic() - fps_start
            fps = frame_count / elapsed if elapsed > 0 else 0
            info = f"{fps:.0f} fps"

            if depth_frame and depth_intrin:
                cx = color_frame.get_width() // 2
                cy = color_frame.get_height() // 2
                dist = depth_frame.get_distance(
                    cx * depth_frame.get_width() // color_frame.get_width(),
                    cy * depth_frame.get_height() // color_frame.get_height(),
                )
                if dist > 0:
                    point = rs.rs2_deproject_pixel_to_point(
                        depth_intrin,
                        [float(depth_frame.get_width() // 2),
                         float(depth_frame.get_height() // 2)],
                        dist,
                    )
                    info += (f" | Z={dist:.2f}m"
                             f"  XYZ=({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})")

            cv2.putText(display, info, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Crosshair at center
            h, w = display.shape[:2]
            cv2.drawMarker(display, (w // 2, h // 2), (0, 255, 0),
                           cv2.MARKER_CROSS, 20, 1)

            cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("s"):
                cv2.imwrite("realsense_snapshot.png", display)
                log("Saved realsense_snapshot.png")
            elif key == ord("d") and has_depth:
                show_depth = not show_depth
                log(f"Depth overlay: {'on' if show_depth else 'off'}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        log(f"Stopped. {frame_count} frames displayed.")


if __name__ == "__main__":
    print("=" * 50, flush=True)
    print("RealSense D456 Live Viewer", flush=True)
    print("=" * 50, flush=True)
    print(flush=True)

    dev = find_device()
    dev = reset_device(dev)
    live_view()
