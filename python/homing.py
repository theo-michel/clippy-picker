"""
Homing sequence for the picker + gantry robot.

Provides a consistent way to define and run homing at program start:

  1. Gantry: run to endstop, set position to 0, then can move 0–500 mm away.
  2. Delta: move to stored home angles (manual home = arms at mechanical limit).
  3. Gripper: move to open (home) position.

Usage:

    from delta_robot import DeltaRobot
    from homing import run_homing_sequence, DEFAULT_HOME

    with DeltaRobot(port) as robot:
        run_homing_sequence(robot, DEFAULT_HOME, home_gantry=True, home_delta=True, home_gripper=True)
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from coordinates import HomePosition

if TYPE_CHECKING:
    from delta_robot import DeltaRobot

logger = logging.getLogger(__name__)


def home_gantry_axis(robot: "DeltaRobot") -> None:
    """
    Home the gantry using the endstop.

    Requires firmware support: send GANTRY_HOME (or equivalent). The firmware
    moves the gantry toward the endstop until it is pressed, sets position to 0,
    then backs off slightly. After this, valid gantry range is [0, GANTRY_X_MAX] mm.

    If the firmware does not support GANTRY_HOME, this sends HOME and the gantry
    will move to the current software zero (run gantry endstop homing once via
    firmware, then ZERO, so that zero = endstop).
    """
    try:
        robot._command_ok("GANTRY_HOME")
        logger.info("Gantry homing started (endstop); position will be 0 when done.")
    except RuntimeError as e:
        if "UNKNOWN_CMD" in str(e) or "Unknown" in str(e):
            logger.warning(
                "GANTRY_HOME not supported; moving gantry to 0. "
                "Ensure endstop homing has been done and ZERO set, or add GANTRY_HOME to firmware."
            )
            robot.move_gantry(0.0)
        else:
            raise


def home_delta_arms(robot: "DeltaRobot", home: HomePosition) -> None:
    """
    Move the delta arms to the configured home angles (e.g. mechanical limit = highest position).
    """
    a1, a2, a3 = home.delta_angles
    robot.move_delta(a1, a2, a3)
    logger.info("Delta homing: moving to (%.2f, %.2f, %.2f) deg", a1, a2, a3)


def home_gripper_axis(robot: "DeltaRobot", home: HomePosition) -> None:
    """Move the gripper to the home (open) position."""
    robot.grip_position(home.gripper_position)
    logger.info("Gripper homing: position %d", home.gripper_position)


def run_homing_sequence(
    robot: "DeltaRobot",
    home: HomePosition,
    *,
    home_gantry: bool = True,
    home_delta: bool = True,
    home_gripper: bool = True,
) -> None:
    """
    Run the full homing sequence.

    Order: gantry first (so the delta is at a known X), then delta, then gripper.
    Each step is started; call robot.wait_until_done() after if you want to block.
    """
    if home_gantry:
        home_gantry_axis(robot)
        robot.wait_until_done()
    if home_delta:
        home_delta_arms(robot, home)
        robot.wait_until_done()
    if home_gripper:
        home_gripper_axis(robot, home)
        robot.wait_until_done()
