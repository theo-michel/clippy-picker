"""
Delta Robot Controller — serial interface to the ESP32 firmware.

Communicates over USB serial at 1 Mbaud using a text-based protocol.
Manages 3 delta-arm stepper motors, 1 gantry axis, and 1 Feetech
STS3215 gripper — all driven by the ESP32 firmware.

Supports both joint-space and Cartesian-space motion.  Cartesian
commands run inverse kinematics on the host and send joint angles
to the firmware.

Coordinate frame (see delta_kinematics.py):
  +X toward arm 1, +Z downward, +Y right-hand rule.
  Workspace is at z > 0 (below the base plate).

Usage::

    from delta_robot import DeltaRobot

    with DeltaRobot("/dev/tty.usbserial-0001") as robot:
        # Joint-space
        robot.move_delta(10.0, 10.0, 10.0)
        robot.wait_until_done()

        # Cartesian — delta only
        robot.move_to_xyz(0, 0, 200)
        robot.wait_until_done()

        # Cartesian — gantry + delta combined
        robot.move_to_position(gantry_x=400, x=0, y=0, z=200)
        robot.wait_until_done()

        robot.grip_close()
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

import serial

from delta_kinematics import DeltaKinematics

from coordinates import (
    GANTRY_X_MAX as _GANTRY_X_MAX,
    GANTRY_X_MIN as _GANTRY_X_MIN,
    DELTA_KINEMATIC_AT_FIRMWARE_ZERO as _DELTA_OFFSET,
    TCP_OFFSET_FROM_EE as _TCP_OFFSET,
)

logger = logging.getLogger(__name__)

BAUD = 115_200
READ_TIMEOUT = 0.5  # seconds — how long readline blocks
CONNECT_TIMEOUT = 5.0  # seconds — wait for READY after reset
DONE_TIMEOUT = 30.0  # seconds — max wait for motion to finish

# Delta joint limits in **kinematic** space (θ=0 = horizontal in delta_kinematics).
# Mechanical stop = -21.8° kinematic.  Home = -15° kinematic.
# Max = home + 95° = 80° kinematic (firmware 101.8°).
MOTOR_ANGLE_MIN = -21.8  # degrees — kinematic angle at mechanical stop
MOTOR_ANGLE_MAX = 80.0  # degrees — home (-15°) + 95° travel
GANTRY_X_MIN = _GANTRY_X_MIN  # mm — 0 at endstop (see coordinates.py)
GANTRY_X_MAX = _GANTRY_X_MAX  # mm — max travel from endstop (500)

DEFAULT_IK = DeltaKinematics(
    upper_arm=150.0,
    lower_arm=268.0,
    Fd=82.5,
    Ed=27.3,
)


class DeltaRobotError(RuntimeError):
    """Any error originating from the delta robot controller."""


@dataclass
class Telemetry:
    """Parsed TELEM response from the firmware."""

    d1: float = 0.0
    d2: float = 0.0
    d3: float = 0.0
    gx: float = 0.0
    moving: bool = False
    dxl_pos: int = 0
    dxl_temp: int = 0
    dxl_load: int = 0
    dxl_ok: bool = False


class DeltaRobot:
    """Serial interface to the FR8 delta robot ESP32 controller.

    Accepts an optional ``ik`` parameter to override the default
    inverse-kinematics geometry.  When omitted the factory default
    (L=150, l=268, Fd=82.5, Ed=27.3) is used.
    """

    def __init__(
        self,
        port: str,
        baud: int = BAUD,
        timeout: float = READ_TIMEOUT,
        ik: DeltaKinematics | None = None,
    ) -> None:
        self._port_name = port
        self._baud = baud
        self._timeout = timeout
        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()
        self.ik = ik or DEFAULT_IK

    # ── Context manager ──────────────────────────────────────────────────

    def __enter__(self) -> DeltaRobot:
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.disconnect()

    # ── Connection ───────────────────────────────────────────────────────

    def connect(self) -> str:
        """Open serial port, wait for firmware to respond, return startup banner.

        Strategy: the ESP32 resets on DTR when the port opens. Instead of a
        long fixed sleep we immediately start polling with PING at short
        intervals so we connect as soon as the firmware is up (typically
        under 1 second).
        """
        self._ser = serial.Serial(
            self._port_name,
            self._baud,
            timeout=self._timeout,
        )

        deadline = time.time() + CONNECT_TIMEOUT
        attempt = 0
        while time.time() < deadline:
            wait = 0.15 if attempt == 0 else 0.25
            time.sleep(wait)
            self._ser.reset_input_buffer()
            attempt += 1

            # Check for READY line in the buffer
            line = self._readline()
            if line == "READY":
                logger.info("Connected (READY) after %d attempts", attempt)
                return "READY"

            # Actively probe with PING
            try:
                resp = self._command("PING")
                if resp == "PONG":
                    logger.info("Connected (PING) after %d attempts", attempt)
                    return "READY (via PING)"
            except (TimeoutError, RuntimeError):
                logger.debug("PING attempt %d — no response yet", attempt)

        raise TimeoutError(
            f"Firmware did not respond within {CONNECT_TIMEOUT}s "
            f"after {attempt} attempts."
        )

    def disconnect(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    @property
    def is_connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    # ── Low-level serial helpers ─────────────────────────────────────────

    def _send(self, cmd: str) -> None:
        assert self._ser, "Not connected"
        with self._lock:
            self._ser.write(f"{cmd}\n".encode())
            self._ser.flush()

    def _readline(self) -> str | None:
        """Read one line (blocking up to self._timeout). Returns None on timeout."""
        try:
            raw = self._ser.readline()
            line = raw.decode(errors="replace").strip()
            return line if line else None
        except serial.SerialException:
            return None

    def _command(self, cmd: str) -> str:
        """Send a command and return the first response line.

        The entire flush→send→read cycle is held under lock so concurrent
        callers (e.g. the status poller) cannot interleave commands.
        """
        assert self._ser, "Not connected"
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(f"{cmd}\n".encode())
            self._ser.flush()
            try:
                raw = self._ser.readline()
                line = raw.decode(errors="replace").strip() or None
            except serial.SerialException:
                line = None
        if line is None:
            raise TimeoutError(f"No response to: {cmd}")
        if line.startswith("ERR:"):
            raise RuntimeError(f"Firmware error: {line[4:]}")
        return line

    def _command_ok(self, cmd: str) -> None:
        """Send a command, expect OK."""
        resp = self._command(cmd)
        if resp != "OK":
            raise RuntimeError(f"Expected OK for '{cmd}', got: {resp!r}")

    # ── Delta motion (angles in degrees) ─────────────────────────────────

    def _kinematic_to_firmware(self, a: float) -> float:
        """Convert kinematic angle (θ=0 = horizontal) to firmware angle (0 = physical home)."""
        return a - _DELTA_OFFSET

    def _firmware_to_kinematic(self, a: float) -> float:
        """Convert firmware angle to kinematic angle."""
        return a + _DELTA_OFFSET

    def move_delta(self, a1: float, a2: float, a3: float) -> None:
        """Move the three delta arms to absolute joint angles (degrees, kinematic space)."""
        f1 = self._kinematic_to_firmware(a1)
        f2 = self._kinematic_to_firmware(a2)
        f3 = self._kinematic_to_firmware(a3)
        self._command_ok(f"M {f1:.4f} {f2:.4f} {f3:.4f}")

    def move_gantry(self, x_mm: float) -> None:
        """Move the gantry axis to an absolute X position (mm)."""
        self._command_ok(f"G {x_mm:.4f}")

    def move_all(
        self,
        a1: float,
        a2: float,
        a3: float,
        x_mm: float,
    ) -> None:
        """Move delta + gantry simultaneously (delta angles in kinematic space)."""
        f1 = self._kinematic_to_firmware(a1)
        f2 = self._kinematic_to_firmware(a2)
        f3 = self._kinematic_to_firmware(a3)
        self._command_ok(f"MG {f1:.4f} {f2:.4f} {f3:.4f} {x_mm:.4f}")

    def home(self) -> None:
        """Move all axes to the current software zero (delta + gantry to 0)."""
        self._command_ok("HOME")

    # ── Gripper ──────────────────────────────────────────────────────────

    def grip_open(self) -> None:
        self._command_ok("GRIP OPEN")

    def grip_close(self) -> None:
        self._command_ok("GRIP CLOSE")

    def grip_position(self, pos: int) -> None:
        """Set gripper to a raw servo position."""
        self._command_ok(f"GRIP {pos}")

    # ── Speed / acceleration ─────────────────────────────────────────────

    def set_delta_speed(self, steps_per_sec: float) -> None:
        self._command_ok(f"SPD {steps_per_sec:.1f}")

    def set_delta_accel(self, steps_per_sec_sq: float) -> None:
        self._command_ok(f"ACC {steps_per_sec_sq:.1f}")

    def set_gantry_speed(self, steps_per_sec: float) -> None:
        self._command_ok(f"GSPD {steps_per_sec:.1f}")

    def set_gantry_accel(self, steps_per_sec_sq: float) -> None:
        self._command_ok(f"GACC {steps_per_sec_sq:.1f}")

    # ── Control ──────────────────────────────────────────────────────────

    def stop(self) -> None:
        """Decelerate all axes to a stop (controlled)."""
        self._command_ok("STOP")

    def emergency_stop(self) -> None:
        """Immediate hard stop — no deceleration."""
        self._command_ok("ESTOP")

    def zero(self) -> None:
        """Declare the current position as the origin for all axes."""
        self._command_ok("ZERO")

    # ── Queries ──────────────────────────────────────────────────────────

    def get_position(self) -> tuple[float, float, float, float]:
        """
        Query current positions.

        Returns:
            (d1_deg, d2_deg, d3_deg, gantry_mm) in **kinematic** space for delta
            (θ=0 = horizontal; matches delta_kinematics and IK/FK).
        """
        resp = self._command("POS")
        if not resp.startswith("POS:"):
            raise RuntimeError(f"Unexpected POS response: {resp!r}")
        parts = resp[4:].split(",")
        f1, f2, f3 = float(parts[0]), float(parts[1]), float(parts[2])
        gx = float(parts[3])
        k1 = self._firmware_to_kinematic(f1)
        k2 = self._firmware_to_kinematic(f2)
        k3 = self._firmware_to_kinematic(f3)
        return (k1, k2, k3, gx)

    def is_moving(self) -> bool:
        resp = self._command("STATUS")
        return "MOVING" in resp

    def get_telemetry(self) -> Telemetry:
        """Read full telemetry from the firmware (positions in kinematic space, Dynamixel state)."""
        resp = self._command("TELEM")
        if not resp.startswith("TELEM:"):
            raise RuntimeError(f"Unexpected TELEM response: {resp!r}")
        kv = dict(pair.split("=", 1) for pair in resp[6:].split(","))
        f1 = float(kv.get("d1", 0))
        f2 = float(kv.get("d2", 0))
        f3 = float(kv.get("d3", 0))
        return Telemetry(
            d1=self._firmware_to_kinematic(f1),
            d2=self._firmware_to_kinematic(f2),
            d3=self._firmware_to_kinematic(f3),
            gx=float(kv.get("gx", 0)),
            moving=kv.get("moving", "0") == "1",
            dxl_pos=int(float(kv.get("dxl_pos", 0))),
            dxl_temp=int(float(kv.get("dxl_temp", 0))),
            dxl_load=int(float(kv.get("dxl_load", 0))),
            dxl_ok=kv.get("dxl_ok", "0") == "1",
        )

    def ping(self) -> bool:
        """Heartbeat check. Returns True if firmware responds."""
        try:
            return self._command("PING") == "PONG"
        except (TimeoutError, RuntimeError):
            return False

    # ── Wait helpers ─────────────────────────────────────────────────────

    def wait_until_done(self, timeout: float = DONE_TIMEOUT) -> bool:
        """
        Block until the firmware sends DONE (all motors reached target).

        Returns True on success, False on timeout.
        Raises RuntimeError if the firmware reports an error.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._readline()
            if line == "DONE":
                return True
            if line and line.startswith("ERR:"):
                raise RuntimeError(f"Error during motion: {line[4:]}")
        return False

    def move_delta_and_wait(
        self,
        a1: float,
        a2: float,
        a3: float,
        timeout: float = DONE_TIMEOUT,
    ) -> bool:
        """Convenience: move delta, then block until done."""
        self.move_delta(a1, a2, a3)
        return self.wait_until_done(timeout)

    def move_gantry_and_wait(
        self,
        x_mm: float,
        timeout: float = DONE_TIMEOUT,
    ) -> bool:
        """Convenience: move gantry, then block until done."""
        self.move_gantry(x_mm)
        return self.wait_until_done(timeout)

    # ── Cartesian motion (IK on the host) ────────────────────────────────

    def _validate_angles(
        self,
        a1: float,
        a2: float,
        a3: float,
    ) -> None:
        for i, a in enumerate((a1, a2, a3), 1):
            if not (MOTOR_ANGLE_MIN <= a <= MOTOR_ANGLE_MAX):
                raise DeltaRobotError(
                    f"Motor {i} angle {a:.2f}° is outside "
                    f"[{MOTOR_ANGLE_MIN}, {MOTOR_ANGLE_MAX}]°"
                )

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
    ) -> tuple[float, float, float]:
        """
        Move the delta end-effector to a Cartesian position (mm).

        Runs inverse kinematics, validates motor limits, and sends
        the resulting joint angles to the firmware.

        Args:
            x, y, z:   Target position in the delta robot frame (mm).
                        Z is positive below the base plate.
            validate:   If True (default), raise on out-of-range angles.

        Returns:
            (a1, a2, a3) — the joint angles actually commanded (degrees).

        Raises:
            DeltaRobotError: If the position is unreachable or angles
                             exceed motor limits.
        """
        try:
            a1, a2, a3 = self.ik.inverse(x, y, z)
        except ValueError as exc:
            raise DeltaRobotError(
                f"Target ({x:.1f}, {y:.1f}, {z:.1f}) is unreachable: {exc}"
            ) from exc

        if validate:
            self._validate_angles(a1, a2, a3)

        self.move_delta(a1, a2, a3)
        logger.info(
            "move_to_xyz(%.1f, %.1f, %.1f) → (%.2f°, %.2f°, %.2f°)",
            x,
            y,
            z,
            a1,
            a2,
            a3,
        )
        return (a1, a2, a3)

    def move_to_xyz_and_wait(
        self,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
        timeout: float = DONE_TIMEOUT,
    ) -> tuple[float, float, float]:
        """Convenience: ``move_to_xyz`` then block until done."""
        angles = self.move_to_xyz(x, y, z, validate=validate)
        self.wait_until_done(timeout)
        return angles

    # ── TCP (gripper tip) motion ──────────────────────────────────────────

    @staticmethod
    def _tcp_to_ee(x: float, y: float, z: float) -> tuple[float, float, float]:
        """Back-calculate the EE position required to place the TCP at (x, y, z)."""
        return (x - _TCP_OFFSET[0], y - _TCP_OFFSET[1], z - _TCP_OFFSET[2])

    def move_tcp(
        self,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
    ) -> tuple[float, float, float]:
        """Move the gripper TCP to a Cartesian position (mm) in the delta frame.

        Internally subtracts the TCP offset and commands the end-effector.

        Returns:
            (a1, a2, a3) — joint angles commanded (degrees).
        """
        return self.move_to_xyz(*self._tcp_to_ee(x, y, z), validate=validate)

    def move_tcp_and_wait(
        self,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
        timeout: float = DONE_TIMEOUT,
    ) -> tuple[float, float, float]:
        """Convenience: ``move_tcp`` then block until done."""
        angles = self.move_tcp(x, y, z, validate=validate)
        self.wait_until_done(timeout)
        return angles

    def move_to_position_tcp(
        self,
        gantry_x: float,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
    ) -> tuple[float, float, float]:
        """Move gantry + delta so the gripper TCP reaches (x, y, z).

        Returns:
            (a1, a2, a3) — joint angles commanded (degrees).
        """
        ex, ey, ez = self._tcp_to_ee(x, y, z)
        return self.move_to_position(gantry_x, ex, ey, ez, validate=validate)

    def move_to_position_tcp_and_wait(
        self,
        gantry_x: float,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
        timeout: float = DONE_TIMEOUT,
    ) -> tuple[float, float, float]:
        """Convenience: ``move_to_position_tcp`` then block until done."""
        angles = self.move_to_position_tcp(
            gantry_x, x, y, z, validate=validate,
        )
        self.wait_until_done(timeout)
        return angles

    def move_to_position(
        self,
        gantry_x: float,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
    ) -> tuple[float, float, float]:
        """
        Move gantry + delta simultaneously to a desired end-effector pose.

        Args:
            gantry_x:  Target gantry position along the linear rail (mm).
            x, y, z:   Target delta end-effector position (mm).
            validate:   If True (default), check all limits.

        Returns:
            (a1, a2, a3) — the joint angles actually commanded (degrees).

        Raises:
            DeltaRobotError: If any target is out of range.
        """
        if validate and not (GANTRY_X_MIN <= gantry_x <= GANTRY_X_MAX):
            raise DeltaRobotError(
                f"Gantry position {gantry_x:.1f} mm is outside "
                f"[{GANTRY_X_MIN}, {GANTRY_X_MAX}] mm"
            )

        try:
            a1, a2, a3 = self.ik.inverse(x, y, z)
        except ValueError as exc:
            raise DeltaRobotError(
                f"Delta target ({x:.1f}, {y:.1f}, {z:.1f}) is unreachable: {exc}"
            ) from exc

        if validate:
            self._validate_angles(a1, a2, a3)

        self.move_all(a1, a2, a3, gantry_x)
        logger.info(
            "move_to_position(gantry=%.1f, %.1f, %.1f, %.1f) "
            "→ (%.2f°, %.2f°, %.2f°)",
            gantry_x,
            x,
            y,
            z,
            a1,
            a2,
            a3,
        )
        return (a1, a2, a3)

    def move_to_position_and_wait(
        self,
        gantry_x: float,
        x: float,
        y: float,
        z: float,
        validate: bool = True,
        timeout: float = DONE_TIMEOUT,
    ) -> tuple[float, float, float]:
        """Convenience: ``move_to_position`` then block until done."""
        angles = self.move_to_position(
            gantry_x,
            x,
            y,
            z,
            validate=validate,
        )
        self.wait_until_done(timeout)
        return angles
