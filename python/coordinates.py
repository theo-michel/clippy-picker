"""
Unified coordinate system and homing definitions for the picker + gantry robot.

This module documents the world frame and defines home positions for:
  - Gantry (linear axis with endstop)
  - Delta arms (manual home at mechanical limit)
  - Gripper (open position)

Coordinate system
-----------------

  World / machine frame (right-handed, mm):

    - Gantry axis = linear rail.
      • 0 = endstop pressed (home).
      • Positive = travel away from endstop (up to GANTRY_X_MAX mm).
      • Same direction as +X in the delta frame.

    - Delta robot frame (fixed to the delta base):
      • Origin at the centre of the delta base plate.
      • +X toward arm 1 (motor 1), same direction as gantry travel.
      • +Z downward (workspace at positive z).
      • +Y completes the right-hand rule.

  Full end-effector pose in world:

    (gantry_x, x, y, z)
    - gantry_x: position along the rail (mm), 0 at endstop.
    - (x, y, z): delta end-effector position in the delta frame (mm).
      z > 0 = below the base plate.

  Gripper:

    - Raw servo position; "home" = open (configurable).
"""

from __future__ import annotations

from dataclasses import dataclass

# ── Gantry ─────────────────────────────────────────────────────────────────

GANTRY_X_MIN = 0.0  # mm — endstop at home
GANTRY_X_MAX = 625.0  # mm — max travel away from endstop

# Gantry motor: "forward" = anticlockwise spin → increases gantry position (away from endstop).

# ── Delta angles ──────────────────────────────────────────────────────────
#
# Mechanical limit = arms at highest position (21.8° above horizontal).
# After homing (manual push to limit + ZERO), firmware reports 0° at that pose.
# Positive firmware angle = arm swings down.
#
# "Home" (safe park) = 15° above horizontal = 6.8° firmware.
# Usable range is 95° from home → max = 6.8 + 95 = 101.8° firmware (80° below horizontal).
#
DELTA_FIRMWARE_MAX_ANGLE = 101.8  # degrees — home (6.8°) + 95° travel

# Kinematic vs firmware angle: in delta_kinematics, θ=0° = upper arm horizontal.
# Mechanical stop is 21.8° above horizontal → θ = -21.8° kinematic.
# When you ZERO there, firmware reports 0. So: kinematic_θ = firmware_θ + this offset.
DELTA_KINEMATIC_AT_FIRMWARE_ZERO = (
    -21.8
)  # degrees (kinematic angle when firmware reports 0)

# Home (park) position — 15° above horizontal = -15° kinematic.
# After zeroing at the mechanical limit, the homing wizard moves here.
DELTA_HOME_ANGLE_1 = -15.0  # degrees (kinematic)
DELTA_HOME_ANGLE_2 = -15.0
DELTA_HOME_ANGLE_3 = -15.0


# ── Gripper home ───────────────────────────────────────────────────────────

# Feetech position for "gripper home" (fully open). Should match firmware GRIP_OPEN_POS.
GRIPPER_HOME_POSITION = 2900

# ── Calibration marker offset ──────────────────────────────────────────────
#
# Offset from the end-effector frame origin to the ArUco marker center,
# expressed in the delta frame (mm).  The EE platform is ~10 mm thick;
# its frame origin is at the centre (5 mm from top surface).  A marker
# taped on top sits 5 mm above the origin, i.e. at z = -5 in the
# z-down convention.  Adjust X/Y if the marker is off-centre.
MARKER_OFFSET_FROM_EE = (0.0, 0.0, -5.0)  # (dx, dy, dz) in mm

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
    """Return the default home definition."""
    return HomePosition()
