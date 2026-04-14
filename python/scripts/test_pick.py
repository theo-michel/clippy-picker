#!/usr/bin/env python3
"""Quick pick-and-return test.

Sequence:
  1. Home everything (gantry → 0, delta → park, gripper → open)
  2. Move gantry to the scanning position (~450 mm)
  3. Detect the Raspberry Pi case with YOLO + RealSense depth
  4. Convert camera detection → delta-frame TCP target
  5. Move above the object, descend, close gripper
  6. Return home and release

Run:  uv run python scripts/test_pick.py --port COM5
"""

from __future__ import annotations

import argparse
import logging
import sys
import time

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))

from camera import RealsenseCamera
from coordinates import (
    camera_to_robot, get_default_home,
    GANTRY_X_MIN, GANTRY_X_MAX,
)
from delta_robot import DeltaRobot
from homing import run_homing_sequence
from object_detection import ObjectDetector

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

SCAN_GANTRY_X = 450.0      # mm — gantry position for scanning
APPROACH_OFFSET = 50.0      # mm — hover this far above detected object
SETTLE_TIME = 1.0           # seconds — wait for camera after gantry stops
DETECT_CONF = 0.4           # YOLO confidence threshold


def detect_object(camera: RealsenseCamera, detector: ObjectDetector) -> dict | None:
    """Run detection, deproject the best hit, return delta-frame coords or None."""
    frame = camera.get_color_frame()
    if frame is None:
        log.warning("No camera frame available")
        return None

    dets = detector.detect(frame, conf=DETECT_CONF)
    if not dets:
        log.warning("No objects detected")
        return None

    best = max(dets, key=lambda d: d.confidence)
    u, v = int(round(best.center_uv[0])), int(round(best.center_uv[1]))
    c_point = camera.deproject(u, v)
    if c_point is None:
        log.warning("No depth at detection center (%d, %d)", u, v)
        return None

    d_point = camera_to_robot(np.array(c_point))
    log.info(
        "Detected %s (%.0f%%) at pixel (%d,%d) → delta (%.1f, %.1f, %.1f)",
        best.label, best.confidence * 100, u, v, *d_point,
    )
    return {"delta": d_point, "label": best.label, "confidence": best.confidence}


def run(port: str) -> None:
    camera = RealsenseCamera()
    camera.start()
    time.sleep(0.5)

    detector = ObjectDetector()

    with DeltaRobot(port) as robot:
        home = get_default_home()

        # 1 — Home
        log.info("=== Homing ===")
        run_homing_sequence(robot, home)
        robot.wait_until_done()

        # 2 — Move gantry to scan position
        log.info("=== Moving gantry to %.0f mm ===", SCAN_GANTRY_X)
        robot.move_gantry_and_wait(SCAN_GANTRY_X)
        time.sleep(SETTLE_TIME)

        # 3 — Detect
        log.info("=== Scanning for objects ===")
        hit = detect_object(camera, detector)
        if hit is None:
            log.error("Nothing found — aborting")
            camera.stop()
            return

        dx, dy, dz = hit["delta"]

        # 4 — Reposition gantry so delta only needs a small XY offset
        world_x = SCAN_GANTRY_X + dx
        gantry_pick = max(GANTRY_X_MIN, min(GANTRY_X_MAX, world_x))
        pick_dx = world_x - gantry_pick
        hover_z = dz - APPROACH_OFFSET
        log.info(
            "=== World X=%.0f → gantry %.0f + delta dx=%.1f ===",
            world_x, gantry_pick, pick_dx,
        )
        robot.move_gantry_and_wait(gantry_pick)

        # 5 — Approach from above (hover)
        log.info("=== Hovering above object ===")
        robot.move_tcp_and_wait(pick_dx, dy, hover_z)

        # 6 — Descend (straight down)
        log.info("=== Descending to pick ===")
        robot.move_tcp_and_wait(pick_dx, dy, dz)
        time.sleep(0.3)

        # 7 — Grab
        log.info("=== Closing gripper ===")
        robot.grip_close()
        time.sleep(0.5)

        # 8 — Lift (straight up)
        log.info("=== Lifting ===")
        robot.move_tcp_and_wait(pick_dx, dy, hover_z)

        # 9 — Return home
        log.info("=== Returning home ===")
        run_homing_sequence(robot, home, home_gripper=False)
        robot.wait_until_done()

        # 10 — Release
        log.info("=== Releasing ===")
        robot.grip_open()
        time.sleep(0.5)

        log.info("=== Done ===")

    camera.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM5)")
    run(parser.parse_args().port)
