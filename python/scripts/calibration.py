"""
Delta Robot Calibration — trace an 8×8 cm square in the XY plane.

Drives the physical robot through a square path using inverse kinematics,
with no web UI required. Useful for verifying mechanical accuracy and
tuning IK parameters.

Usage:
    uv run python scripts/calibration.py /dev/tty.usbmodem14101
    uv run python scripts/calibration.py /dev/ttyUSB0 --z -250 --speed 20
    uv run python scripts/calibration.py --dry-run   # IK only, no hardware
"""

from __future__ import annotations

import argparse
import math
import sys
import time

sys.path.insert(0, ".")

from delta_kinematics import DeltaKinematics

# Default robot geometry (mm) — must match your physical robot
DEFAULT_UPPER_ARM = 150.0
DEFAULT_LOWER_ARM = 271.0
DEFAULT_FD = 36.7
DEFAULT_ED = 80.0

# Default working height (mm, Z points downward so workspace is negative)
DEFAULT_Z = -250.0

# Square side length (mm)
SQUARE_SIDE = 80.0  # 8 cm

# How many interpolation points per edge
POINTS_PER_EDGE = 20


def build_square_path(
    side: float,
    z: float,
    points_per_edge: int,
    center_x: float = 0.0,
    center_y: float = 0.0,
) -> list[tuple[float, float, float]]:
    """
    Generate waypoints for a square centered at (center_x, center_y) in the
    XY plane at height z.
    """
    half = side / 2.0
    corners = [
        (center_x - half, center_y - half),
        (center_x + half, center_y - half),
        (center_x + half, center_y + half),
        (center_x - half, center_y + half),
    ]

    path: list[tuple[float, float, float]] = []
    for i in range(4):
        x0, y0 = corners[i]
        x1, y1 = corners[(i + 1) % 4]
        for step in range(points_per_edge):
            t = step / points_per_edge
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            path.append((x, y, z))

    path.append((corners[0][0], corners[0][1], z))
    return path


def run_calibration(
    port: str | None,
    z: float,
    speed_rpm: float,
    accel: float,
    dry_run: bool,
    upper_arm: float,
    lower_arm: float,
    fd: float,
    ed: float,
    dwell: float,
) -> None:
    dk = DeltaKinematics(upper_arm=upper_arm, lower_arm=lower_arm, Fd=fd, Ed=ed)
    path = build_square_path(SQUARE_SIDE, z, POINTS_PER_EDGE)

    # Pre-check: make sure every waypoint is reachable
    print(f"Square: {SQUARE_SIDE:.0f}×{SQUARE_SIDE:.0f} mm at Z={z:.1f} mm")
    print(f"Waypoints: {len(path)}")
    angle_path: list[tuple[float, float, float]] = []
    for i, (x, y, zz) in enumerate(path):
        try:
            angles = dk.inverse(x, y, zz)
            angle_path.append(angles)
        except ValueError as e:
            print(f"  UNREACHABLE waypoint #{i}: ({x:.1f}, {y:.1f}, {zz:.1f}) — {e}")
            print("  Try a different --z height or smaller square.")
            sys.exit(1)

    # Show corner angles for sanity check
    edges = POINTS_PER_EDGE
    corner_indices = [0, edges, 2 * edges, 3 * edges]
    print("\nCorner angles (θ1, θ2, θ3):")
    for ci in corner_indices:
        x, y, _ = path[ci]
        a = angle_path[ci]
        print(f"  ({x:7.1f}, {y:7.1f}) → ({a[0]:6.2f}°, {a[1]:6.2f}°, {a[2]:6.2f}°)")

    if dry_run:
        print("\n[DRY RUN] No hardware commands sent.")
        return

    if port is None:
        print("\nError: serial port required (or use --dry-run)")
        sys.exit(1)

    from delta_robot import DeltaRobot

    with DeltaRobot(port) as robot:
        print(f"\nConfiguring: speed={speed_rpm} RPM, accel={accel} RPM/s")
        robot.set_speed(speed_rpm)
        robot.set_acceleration(accel)

        # Move to the start position first
        print("Moving to start position …")
        t1, t2, t3 = angle_path[0]
        robot.move_to(t1, t2, t3)
        robot.wait_until_done()
        time.sleep(dwell)

        print("Tracing square …")
        for i, (t1, t2, t3) in enumerate(angle_path[1:], start=1):
            x, y, _ = path[i]
            robot.move_to(t1, t2, t3)
            robot.wait_until_done()
            if i % POINTS_PER_EDGE == 0:
                corner_num = i // POINTS_PER_EDGE
                print(f"  Corner {corner_num}/4 reached  ({x:.1f}, {y:.1f})")
                time.sleep(dwell)

        print("Square complete — homing …")
        robot.home()
        robot.wait_until_done()
        print("Done.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Trace an 8×8 cm calibration square with the delta robot.",
    )
    parser.add_argument(
        "port", nargs="?", default=None,
        help="Serial port (e.g. /dev/tty.usbmodem14101, COM3)",
    )
    parser.add_argument(
        "--z", type=float, default=DEFAULT_Z,
        help=f"Working height in mm (default: {DEFAULT_Z})",
    )
    parser.add_argument(
        "--speed", type=float, default=20.0,
        help="Motor speed in RPM (default: 20 — slow for calibration)",
    )
    parser.add_argument(
        "--accel", type=float, default=40.0,
        help="Motor acceleration in RPM/s (default: 40)",
    )
    parser.add_argument(
        "--dwell", type=float, default=0.5,
        help="Pause at each corner in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Compute IK and print angles without connecting to hardware",
    )
    parser.add_argument("--upper-arm", type=float, default=DEFAULT_UPPER_ARM)
    parser.add_argument("--lower-arm", type=float, default=DEFAULT_LOWER_ARM)
    parser.add_argument("--fd", type=float, default=DEFAULT_FD)
    parser.add_argument("--ed", type=float, default=DEFAULT_ED)

    args = parser.parse_args()

    run_calibration(
        port=args.port,
        z=args.z,
        speed_rpm=args.speed,
        accel=args.accel,
        dry_run=args.dry_run,
        upper_arm=args.upper_arm,
        lower_arm=args.lower_arm,
        fd=args.fd,
        ed=args.ed,
        dwell=args.dwell,
    )


if __name__ == "__main__":
    main()
