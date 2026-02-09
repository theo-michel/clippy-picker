"""
Delta Robot Controller — Python side
Communicates with the Arduino over serial (115200 baud).

Requirements:
    pip install pyserial

Usage:
    from delta_robot import DeltaRobot

    robot = DeltaRobot("/dev/tty.usbmodem14101")  # macOS example
    robot.move_to(45, 45, 45)
    robot.wait_until_done()
    robot.home()
    robot.wait_until_done()
    robot.close()
"""

from __future__ import annotations

import logging
import sys
import time
from typing import Optional

import serial

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("delta_robot")


class DeltaRobotError(Exception):
    """Raised when the Arduino returns an ERR response."""


class DeltaRobot:
    """Serial interface to the Delta Robot Arduino controller."""

    def __init__(
        self,
        port: str,
        baud: int = 115200,
        timeout: float = 2.0,
        settle_time: float = 2.0,
    ) -> None:
        """
        Open the serial connection and wait for the Arduino to reset.

        Args:
            port: Serial port (e.g. '/dev/tty.usbmodem14101', 'COM3').
            baud: Baud rate — must match the Arduino sketch (default 115200).
            timeout: Read timeout in seconds.
            settle_time: Seconds to wait after opening for the Arduino to boot.
        """
        self.ser = None  # set early so __del__ / close() never hits AttributeError
        log.info("Connecting to %s @ %d baud …", port, baud)
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(settle_time)  # Arduino resets on serial open

        # Drain the boot messages
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="replace").strip()
            if line:
                log.info("Arduino: %s", line)

        log.info("Connected.")

    # ------------------------------------------------------------------ #
    #  Low-level communication
    # ------------------------------------------------------------------ #

    def send(self, command: str) -> str:
        """
        Send a command string and return the first response line.

        Raises DeltaRobotError if the Arduino replies with 'ERR:'.
        """
        cmd = command.strip()
        log.debug("→ %s", cmd)
        self.ser.reset_input_buffer()
        self.ser.write(f"{cmd}\n".encode())

        response = self._read_line()
        if response.startswith("ERR"):
            raise DeltaRobotError(response)
        return response

    def _read_line(self, timeout: Optional[float] = None) -> str:
        """Read one line from serial, optionally with a custom timeout."""
        old_timeout = self.ser.timeout
        if timeout is not None:
            self.ser.timeout = timeout
        try:
            raw = self.ser.readline()
            line = raw.decode(errors="replace").strip()
            log.debug("← %s", line)
            return line
        finally:
            if timeout is not None:
                self.ser.timeout = old_timeout

    # ------------------------------------------------------------------ #
    #  Motion commands
    # ------------------------------------------------------------------ #

    def move_to(self, deg1: float, deg2: float, deg3: float) -> str:
        """Move all three motors to absolute degree positions."""
        return self.send(f"M {deg1} {deg2} {deg3}")

    def move_motor(self, motor: int, degrees: float) -> str:
        """
        Move a single motor to an absolute degree position.

        Args:
            motor: Motor number (1, 2, or 3).
            degrees: Target angle in degrees.
        """
        if motor not in (1, 2, 3):
            raise ValueError("motor must be 1, 2, or 3")
        return self.send(f"M{motor} {degrees}")

    def move_relative(self, deg1: float, deg2: float, deg3: float) -> str:
        """Move all three motors by a relative amount (degrees)."""
        return self.send(f"R {deg1} {deg2} {deg3}")

    def home(self) -> str:
        """Return all motors to the zero position."""
        return self.send("HOME")

    def stop(self) -> str:
        """Decelerate all motors to a stop."""
        return self.send("STOP")

    def emergency_stop(self) -> str:
        """Immediately halt all motors (no deceleration)."""
        return self.send("ESTOP")

    # ------------------------------------------------------------------ #
    #  Configuration
    # ------------------------------------------------------------------ #

    def set_speed(self, rpm: float) -> str:
        """Set the maximum speed in RPM for all motors."""
        return self.send(f"SPD {rpm}")

    def set_acceleration(self, rpm_per_sec: float) -> str:
        """Set acceleration in RPM/s for all motors."""
        return self.send(f"ACC {rpm_per_sec}")

    def enable(self) -> str:
        """Enable the DRV8825 drivers (energize coils, hold position)."""
        return self.send("ENABLE")

    def disable(self) -> str:
        """Disable the DRV8825 drivers (coils free, no holding torque)."""
        return self.send("DISABLE")

    def zero(self) -> str:
        """Set the current position as the new zero reference."""
        return self.send("ZERO")

    # ------------------------------------------------------------------ #
    #  Status
    # ------------------------------------------------------------------ #

    def get_position(self) -> tuple[int, int, int]:
        """
        Get current motor positions in steps.

        Returns:
            (steps1, steps2, steps3)
        """
        resp = self.send("POS")
        # Expected format: "POS: s1 s2 s3"
        parts = resp.replace("POS:", "").strip().split()
        return (int(parts[0]), int(parts[1]), int(parts[2]))

    def is_moving(self) -> bool:
        """Return True if any motor is still in motion."""
        resp = self.send("STATUS")
        return "MOVING" in resp

    def wait_until_done(
        self, poll_interval: float = 0.05, timeout: float = 60.0
    ) -> None:
        """
        Block until all motors report IDLE or a DONE message arrives.

        Uses a combination of listening for the asynchronous 'DONE' line
        and polling STATUS as a fallback.

        Args:
            poll_interval: Seconds between STATUS polls.
            timeout: Maximum seconds to wait before raising TimeoutError.
        """
        start = time.time()
        while True:
            # Check for any queued lines (the Arduino sends 'DONE' automatically)
            while self.ser.in_waiting:
                line = self._read_line(timeout=0.1)
                if line == "DONE":
                    return

            # Fallback: explicit poll
            if not self.is_moving():
                return

            if time.time() - start > timeout:
                raise TimeoutError(f"Motors did not finish within {timeout} s")
            time.sleep(poll_interval)

    # ------------------------------------------------------------------ #
    #  Context manager & cleanup
    # ------------------------------------------------------------------ #

    def close(self) -> None:
        """Cleanly close the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            log.info("Serial connection closed.")

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        self.close()


# ====================================================================== #
#  Standalone demo
# ====================================================================== #


def demo(port: str) -> None:
    """Run a simple movement demo."""
    with DeltaRobot(port) as robot:
        print("\n--- Delta Robot Demo ---\n")

        print("Moving all motors to 90°…")
        robot.move_to(90, 90, 90)
        robot.wait_until_done()
        print("Position:", robot.get_position())

        print("Moving to staggered angles (45°, 90°, 135°)…")
        robot.move_to(45, 90, 135)
        robot.wait_until_done()
        print("Position:", robot.get_position())

        print("Relative move: +30° on each…")
        robot.move_relative(30, 30, 30)
        robot.wait_until_done()
        print("Position:", robot.get_position())

        print("Homing…")
        robot.home()
        robot.wait_until_done()
        print("Position:", robot.get_position())

        print("\nDone!")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python delta_robot.py <serial_port>")
        print("  macOS example:   python delta_robot.py /dev/tty.usbmodem14101")
        print("  Linux example:   python delta_robot.py /dev/ttyUSB0")
        print("  Windows example: python delta_robot.py COM3")
        sys.exit(1)

    demo(sys.argv[1])
