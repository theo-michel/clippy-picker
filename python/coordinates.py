"""
Unified coordinate system and homing definitions for the picker + gantry robot.

This module documents the world frame and defines home positions for:
  - Gantry (linear axis with endstop)
  - Delta arms (manual home at mechanical limit)
  - Gripper (open position)

Coordinate system
----------------

  World / machine frame (right-handed, mm):

    - X_world = gantry position along the linear rail.
      • 0 = endstop pressed (home).
      • Positive = travel away from endstop (up to GANTRY_X_MAX mm).
      • On the fourth (gantry) motor, "forward" = anticlockwise rotation
        when viewed from the motor shaft, which increases gantry position.

    - Y, Z = delta robot frame (fixed to the delta base).
      • Origin at the centre of the delta base plate.
      • Z points downward (workspace has negative z).
      • Y axis direction is fixed by the delta geometry (see delta_kinematics).

  Full end-effector pose in world:

    (gantry_x, x, y, z)
    - gantry_x: position along the rail (mm), 0 at endstop, max 500 mm.
    - (x, y, z): delta end-effector position in the delta frame (mm).
      Same convention as DeltaKinematics: z negative below the base.

  Gripper:

    - Raw servo position; "home" = open (configurable).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

# ── Gantry ─────────────────────────────────────────────────────────────────

GANTRY_X_MIN = 0.0   # mm — endstop at home
GANTRY_X_MAX = 625.0 # mm — max travel away from endstop

# Gantry motor: "forward" = anticlockwise spin → increases gantry position (away from endstop).

# ── Delta home (manual) ────────────────────────────────────────────────────
#
# Best practice: define home as 0° in joint space.
#   - Mechanical home = arms at highest position (~20° above horizontal).
#   - After homing (manual push + "Set Zero Here" or drive to limit + "Set delta home"),
#     store (0, 0, 0) as delta home so that 0° = that pose.
#   - Positive angle = arm swings down from home. Stay away from the 90° (horizontal)
#     singularity: e.g. limit travel to 95° from home (home + 95° ≈ -75° from horizontal).
#
# Range from home: 0° (home) to DELTA_ANGLE_RANGE_FROM_HOME (max safe travel down).
DELTA_ANGLE_RANGE_FROM_HOME = 95.0   # degrees — home to max down (avoids ~90° singularity)

# Kinematic vs firmware angle: in delta_kinematics, θ=0° = upper arm horizontal.
# If physical home is ~20° above horizontal, that pose is θ = -20° in kinematic space.
# When you ZERO at that pose, the firmware reports 0. So: kinematic_θ = firmware_θ + this constant.
DELTA_KINEMATIC_AT_FIRMWARE_ZERO = -20.0   # degrees (kinematic angle when firmware reports 0)

# Delta home = arms at highest position (mechanical limit).
# Set via the web UI "Set delta home" (captures current angles) or by editing
# delta_home.json. Prefer "Set delta home (0,0,0)" after ZERO so home = 0°.

# Default = kinematic angle at physical home (20° above horizontal = -20° in kinematics).
DELTA_HOME_ANGLE_1 = -20.0   # degrees — default if no delta_home.json
DELTA_HOME_ANGLE_2 = -20.0
DELTA_HOME_ANGLE_3 = -20.0

# Persisted delta home (same directory as this module). Created by "Set delta home" in UI.
_DELTA_HOME_FILE = Path(__file__).resolve().parent / "delta_home.json"


def load_delta_home() -> tuple[float, float, float]:
    """Load delta home angles from delta_home.json, or return defaults."""
    if not _DELTA_HOME_FILE.exists():
        return (DELTA_HOME_ANGLE_1, DELTA_HOME_ANGLE_2, DELTA_HOME_ANGLE_3)
    try:
        data = json.loads(_DELTA_HOME_FILE.read_text())
        return (
            float(data.get("delta_angle_1", DELTA_HOME_ANGLE_1)),
            float(data.get("delta_angle_2", DELTA_HOME_ANGLE_2)),
            float(data.get("delta_angle_3", DELTA_HOME_ANGLE_3)),
        )
    except (OSError, json.JSONDecodeError, TypeError):
        return (DELTA_HOME_ANGLE_1, DELTA_HOME_ANGLE_2, DELTA_HOME_ANGLE_3)


def save_delta_home(a1: float, a2: float, a3: float) -> None:
    """Save delta home angles to delta_home.json."""
    _DELTA_HOME_FILE.write_text(
        json.dumps(
            {"delta_angle_1": a1, "delta_angle_2": a2, "delta_angle_3": a3},
            indent=2,
        )
    )

# ── Gripper home ───────────────────────────────────────────────────────────

# Feetech position for "gripper home" (fully open). Should match firmware GRIP_OPEN_POS.
GRIPPER_HOME_POSITION = 2900

# ── Home position container ─────────────────────────────────────────────────

@dataclass
class HomePosition:
    """Defines what 'home' means for each subsystem."""
    gantry_x: float = 0.0
    delta_angle_1: float = DELTA_HOME_ANGLE_1
    delta_angle_2: float = DELTA_HOME_ANGLE_2
    delta_angle_3: float = DELTA_HOME_ANGLE_3
    gripper_position: int = GRIPPER_HOME_POSITION

    @property
    def delta_angles(self) -> tuple[float, float, float]:
        return (self.delta_angle_1, self.delta_angle_2, self.delta_angle_3)


def get_default_home() -> HomePosition:
    """Return the current home definition (delta angles from file if present)."""
    a1, a2, a3 = load_delta_home()
    return HomePosition(
        gantry_x=0.0,
        delta_angle_1=a1,
        delta_angle_2=a2,
        delta_angle_3=a3,
        gripper_position=GRIPPER_HOME_POSITION,
    )


# Default home — use get_default_home() to get angles from file.
DEFAULT_HOME = get_default_home()
