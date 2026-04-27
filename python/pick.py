"""Pick task

Pure pick-and-return sequence
Takes hardware objects as parameters, yields progress events as plain dicts.

Usage (standalone)::

    for event in pick_sequence(robot, camera, pointer, "red screwdriver", scan_gx=450):
        print(event)

Usage (from web controller)::

    def generate():
        for event in pick_sequence(robot, camera, pointer, description, ...):
            yield f"data: {json.dumps(event)}\\n\\n"
"""

from __future__ import annotations

import base64
import logging
import threading
import time
from typing import TYPE_CHECKING, Generator

import cv2
import numpy as np

from coordinates import (
    DELTA_HOME_ANGLE_1,
    DELTA_HOME_ANGLE_2,
    DELTA_HOME_ANGLE_3,
    GANTRY_X_MAX,
    GANTRY_X_MIN,
    TCP_OFFSET_FROM_EE,
    camera_to_robot,
)

if TYPE_CHECKING:
    from camera import StereoCamera
    from delta_robot import DeltaRobot
    from object_detection import ObjectPointer

log = logging.getLogger(__name__)

HOME_ANGLES = (DELTA_HOME_ANGLE_1, DELTA_HOME_ANGLE_2, DELTA_HOME_ANGLE_3)

Event = dict


class PickError(Exception):
    """A step failed — abort the sequence."""


class PickCancelled(Exception):
    """Cooperative cancellation requested."""


class PickSequence:
    """Stateful pick-and-return sequence.

    Each step is a method that yields progress events and raises
    :class:`PickError` or :class:`PickCancelled` to abort.
    """

    STEPS = [
        "_prepare",
        "_scan",
        "_approach",
        "_descend",
        "_grab",
        "_lift",
        "_return_home",
        "_release",
    ]

    def __init__(
        self,
        robot: "DeltaRobot",
        camera: "StereoCamera",
        pointer: "ObjectPointer",
        target_description: str,
        *,
        scan_gx: float = 450.0,
        approach_offset: float = 50.0,
        cancel: threading.Event | None = None,
    ) -> None:
        if not target_description or not target_description.strip():
            raise ValueError("target_description is required")
        self.robot = robot
        self.camera = camera
        self.pointer = pointer
        self.target_description = target_description.strip()
        self.scan_gx = scan_gx
        self.approach_offset = approach_offset
        self._cancel = cancel or threading.Event()

        # Populated by _scan, consumed by _approach through _lift
        self.label: str = ""
        self.gantry_pick: float = 0.0
        self.pick_dx: float = 0.0
        self.pick_dy: float = 0.0
        self.pick_dz: float = 0.0
        self.hover_z: float = 0.0

    @property
    def n_steps(self) -> int:
        return len(self.STEPS)

    # ── Public API ──────────────────────────────────────────────────

    def run(self) -> Generator[Event, None, None]:
        """Execute all steps, yielding events. Safe to iterate once."""
        try:
            for step_fn_name in self.STEPS:
                yield from getattr(self, step_fn_name)()
            yield {"type": "done", "message": "Pick complete."}
        except PickCancelled:
            yield {"type": "cancelled"}
        except PickError as e:
            yield {"type": "error", "message": str(e)}

    # ── Helpers ─────────────────────────────────────────────────────

    def _event(self, index: int, name: str, detail: str = "") -> Event:
        return {
            "type": "step",
            "step": name,
            "detail": detail,
            "index": index,
            "total": self.n_steps,
        }

    def _cmd(self, fn) -> None:
        """Run a robot command; raise PickError on failure."""
        try:
            fn()
        except Exception as e:
            raise PickError(str(e)) from e

    def _wait(self, timeout: float = 30.0) -> None:
        time.sleep(0.3)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.robot.is_moving():
                return
            time.sleep(0.2)

    def _check_cancel(self) -> None:
        if self._cancel.is_set():
            raise PickCancelled

    # ── Steps ──────────────────────────────────────────────────────

    def _prepare(self) -> Generator[Event, None, None]:
        yield self._event(1, "Prepare", "Gripper open, delta → home")
        self._cmd(self.robot.grip_open)
        self._cmd(lambda: self.robot.move_delta(*HOME_ANGLES))
        self._wait()
        self._check_cancel()

    def _scan(self) -> Generator[Event, None, None]:
        yield self._event(
            2, "Scan", f"Gantry → {self.scan_gx:.0f} mm, target: {self.target_description!r}"
        )
        self._cmd(lambda: self.robot.move_gantry(self.scan_gx))
        self._wait()
        time.sleep(1.0)

        frame = self.camera.get_color_frame()
        if frame is None:
            raise PickError("No camera frame")

        point = self.pointer.point(frame, self.target_description)
        if point is None:
            raise PickError(f"VLM could not locate {self.target_description!r}")

        self.label = point.label
        u, v = int(round(point.u)), int(round(point.v))

        yield {
            "type": "point",
            "description": self.target_description,
            "label": self.label,
            "pixel": [u, v],
            "frame": _encode_annotated_frame(frame, u, v, self.label),
        }

        c_point = self.camera.deproject(u, v, patch=5)
        if c_point is None:
            raise PickError(f"No depth at pixel ({u}, {v})")

        d_pt = camera_to_robot(np.array(c_point))
        dx, dy, dz = float(d_pt[0]), float(d_pt[1]), float(d_pt[2])

        world_x = self.scan_gx + dx
        self.gantry_pick = max(GANTRY_X_MIN, min(GANTRY_X_MAX, world_x))
        self.pick_dx = world_x - self.gantry_pick
        self.pick_dy = dy
        self.pick_dz = dz
        self.hover_z = self.pick_dz - self.approach_offset

        yield self._event(
            2,
            "Scan",
            f"{self.label} @ ({u}, {v}) — "
            f"world X={world_x:.0f}, gantry→{self.gantry_pick:.0f}, "
            f"delta ({self.pick_dx:.1f}, {self.pick_dy:.1f}, {self.pick_dz:.1f})",
        )

        self._verify_reachability()
        time.sleep(0.5)
        self._check_cancel()

    def _verify_reachability(self) -> None:
        for label, z in [("hover", self.hover_z), ("pick", self.pick_dz)]:
            ee = (
                self.pick_dx - TCP_OFFSET_FROM_EE[0],
                self.pick_dy - TCP_OFFSET_FROM_EE[1],
                z - TCP_OFFSET_FROM_EE[2],
            )
            try:
                a1, a2, a3 = self.robot.ik.inverse(*ee)
                if not all(-21.8 <= a <= 80.0 for a in (a1, a2, a3)):
                    raise ValueError(
                        f"angles ({a1:.1f}, {a2:.1f}, {a3:.1f}) outside limits"
                    )
            except (ValueError, Exception) as e:
                raise PickError(f"Target unreachable at {label} Z={z:.1f}: {e}") from e

    def _approach(self) -> Generator[Event, None, None]:
        yield self._event(
            3,
            "Approach",
            f"Gantry → {self.gantry_pick:.0f}, "
            f"TCP → ({self.pick_dx:.1f}, {self.pick_dy:.1f}, {self.hover_z:.1f})",
        )
        self._cmd(lambda: self.robot.move_gantry(self.gantry_pick))
        self._wait()
        self._cmd(lambda: self.robot.move_tcp(self.pick_dx, self.pick_dy, self.hover_z))
        self._wait()
        self._check_cancel()

    def _descend(self) -> Generator[Event, None, None]:
        yield self._event(
            4,
            "Descend",
            f"TCP → ({self.pick_dx:.1f}, {self.pick_dy:.1f}, {self.pick_dz:.1f})",
        )
        self._cmd(lambda: self.robot.move_tcp(self.pick_dx, self.pick_dy, self.pick_dz))
        self._wait()
        time.sleep(0.3)

    def _grab(self) -> Generator[Event, None, None]:
        yield self._event(5, "Grab", "Closing gripper")
        self._cmd(self.robot.grip_close)
        # Block until the servo stops advancing — either it reached the closed
        # position or stalled against the object. Either way it's safe to lift.
        self._cmd(lambda: self.robot.wait_for_gripper(timeout=3.0))
        self._check_cancel()

    def _lift(self) -> Generator[Event, None, None]:
        yield self._event(
            6,
            "Lift",
            f"TCP → ({self.pick_dx:.1f}, {self.pick_dy:.1f}, {self.hover_z:.1f})",
        )
        self._cmd(lambda: self.robot.move_tcp(self.pick_dx, self.pick_dy, self.hover_z))
        self._wait()
        self._check_cancel()

    def _return_home(self) -> Generator[Event, None, None]:
        yield self._event(7, "Return home", "Delta → home, gantry → 0")
        self._cmd(lambda: self.robot.move_delta(*HOME_ANGLES))
        self._wait()
        self._cmd(lambda: self.robot.move_gantry(0.0))
        self._wait()

    def _release(self) -> Generator[Event, None, None]:
        yield self._event(8, "Release", "Opening gripper")
        self._cmd(self.robot.grip_open)
        time.sleep(0.5)


# ── Helpers ──────────────────────────────────────────────────────────


def _encode_annotated_frame(
    frame: np.ndarray, u: int, v: int, label: str
) -> str | None:
    """Draw a crosshair + label on ``frame`` at ``(u, v)`` and return a data URL.

    Returns ``None`` if encoding fails so the caller can degrade gracefully.
    """
    try:
        annotated = frame.copy()
        h, w = annotated.shape[:2]
        u = int(np.clip(u, 0, w - 1))
        v = int(np.clip(v, 0, h - 1))

        color = (0, 255, 0)
        shadow = (0, 0, 0)

        cv2.line(annotated, (u - 20, v), (u + 20, v), shadow, 4)
        cv2.line(annotated, (u, v - 20), (u, v + 20), shadow, 4)
        cv2.line(annotated, (u - 20, v), (u + 20, v), color, 2)
        cv2.line(annotated, (u, v - 20), (u, v + 20), color, 2)
        cv2.circle(annotated, (u, v), 10, shadow, 4)
        cv2.circle(annotated, (u, v), 10, color, 2)
        cv2.circle(annotated, (u, v), 2, color, -1)

        text = f"{label} ({u},{v})"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        tx = max(5, min(u + 14, w - tw - 5))
        ty = max(th + 5, min(v - 14, h - 5))
        cv2.rectangle(annotated, (tx - 3, ty - th - 3), (tx + tw + 3, ty + 3), shadow, -1)
        cv2.putText(annotated, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return None
        return "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("ascii")
    except Exception:
        log.exception("Failed to annotate VLM point frame")
        return None


# ── Convenience wrapper ──────────────────────────────────────────────


def pick_sequence(
    robot: "DeltaRobot",
    camera: "StereoCamera",
    pointer: "ObjectPointer",
    target_description: str,
    **kwargs,
) -> Generator[Event, None, None]:
    """Functional interface — same as ``PickSequence(...).run()``."""
    return PickSequence(robot, camera, pointer, target_description, **kwargs).run()
