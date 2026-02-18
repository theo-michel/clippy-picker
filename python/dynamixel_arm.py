"""
Dynamixel XL330 Arm Controller.
Thin wrapper around dynamixel_sdk for servo IDs 4, 5, 6.
Protocol 2.0, 1 Mbps.

NOT internally locked — the caller (web_controller.py) holds the lock,
mirroring how DeltaRobot is used with robot_lock.

Usage:
    from dynamixel_arm import DynamixelArm
    arm = DynamixelArm("/dev/cu.usbmodem58FA0961181")
    arm.set_torque(4, True)
    arm.set_position(4, 2048)
    arm.close()
"""

from __future__ import annotations

import logging
import os
from glob import glob
from typing import Optional

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

log = logging.getLogger("dynamixel_arm")


class DynamixelArmError(Exception):
    """Raised on communication failures with the Dynamixel bus."""


class DynamixelArm:
    """Interface to the Dynamixel XL330 arm servos (joints 4, 5 and claw 6)."""

    SERVO_IDS = [4, 5, 6]
    BAUDRATE = 1_000_000
    PROTOCOL_VERSION = 2.0

    # Control table addresses (XL330, Protocol 2.0)
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    def __init__(self, port: str) -> None:
        log.info("Opening Dynamixel port %s @ %d bps", port, self.BAUDRATE)
        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        if not self._port_handler.openPort():
            raise DynamixelArmError(f"Cannot open port {port}")
        if not self._port_handler.setBaudRate(self.BAUDRATE):
            self._port_handler.closePort()
            raise DynamixelArmError("Cannot set baudrate")

        # Read initial torque states
        self._torques: dict[int, bool] = {}
        for sid in self.SERVO_IDS:
            val, res, _ = self._packet_handler.read1ByteTxRx(
                self._port_handler, sid, self.ADDR_TORQUE_ENABLE
            )
            self._torques[sid] = bool(val) if res == COMM_SUCCESS else False

        log.info("Dynamixel arm connected on %s", port)

    def close(self) -> None:
        if self._port_handler is not None:
            self._port_handler.closePort()
            log.info("Dynamixel port closed.")

    # ── Position ──────────────────────────────────────────────────────────

    def set_position(self, servo_id: int, value: int) -> None:
        """Set goal position (0-4095) for a servo."""
        value = max(0, min(4095, value))
        res, err = self._packet_handler.write4ByteTxRx(
            self._port_handler, servo_id, self.ADDR_GOAL_POSITION, value
        )
        if res != COMM_SUCCESS:
            raise DynamixelArmError(
                f"Set position failed for servo {servo_id}: "
                f"{self._packet_handler.getTxRxResult(res)}"
            )

    def get_position(self, servo_id: int) -> int:
        """Read present position (0-4095) of a servo."""
        val, res, _ = self._packet_handler.read4ByteTxRx(
            self._port_handler, servo_id, self.ADDR_PRESENT_POSITION
        )
        if res != COMM_SUCCESS:
            raise DynamixelArmError(
                f"Get position failed for servo {servo_id}: "
                f"{self._packet_handler.getTxRxResult(res)}"
            )
        return val

    def get_all_positions(self) -> dict[int, int]:
        """Read present positions of all servos. Returns {id: position}."""
        positions = {}
        for sid in self.SERVO_IDS:
            val, res, _ = self._packet_handler.read4ByteTxRx(
                self._port_handler, sid, self.ADDR_PRESENT_POSITION
            )
            if res == COMM_SUCCESS:
                positions[sid] = val
        return positions

    # ── Torque ────────────────────────────────────────────────────────────

    def set_torque(self, servo_id: int, enabled: bool) -> None:
        res, err = self._packet_handler.write1ByteTxRx(
            self._port_handler, servo_id, self.ADDR_TORQUE_ENABLE, int(enabled)
        )
        if res != COMM_SUCCESS:
            raise DynamixelArmError(
                f"Set torque failed for servo {servo_id}: "
                f"{self._packet_handler.getTxRxResult(res)}"
            )
        self._torques[servo_id] = enabled

    def get_torque(self, servo_id: int) -> bool:
        val, res, _ = self._packet_handler.read1ByteTxRx(
            self._port_handler, servo_id, self.ADDR_TORQUE_ENABLE
        )
        if res == COMM_SUCCESS:
            self._torques[servo_id] = bool(val)
        return self._torques.get(servo_id, False)

    # ── Convenience ───────────────────────────────────────────────────────

    def center_all(self) -> None:
        """Enable torque and move all servos to center (2048)."""
        for sid in self.SERVO_IDS:
            self.set_torque(sid, True)
            self.set_position(sid, 2048)

    # ── Port detection (class methods) ────────────────────────────────────

    @staticmethod
    def find_default_port() -> Optional[str]:
        for p in ["/dev/ttyUSB0", "/dev/ttyACM0"]:
            if os.path.exists(p):
                return p
        for pattern in ["/dev/cu.usbserial*", "/dev/cu.usbmodem*"]:
            for p in sorted(glob(pattern)):
                if os.path.exists(p):
                    return p
        return None

    @staticmethod
    def list_ports() -> list[str]:
        ports: list[str] = []
        for p in ["/dev/ttyUSB0", "/dev/ttyACM0"]:
            if os.path.exists(p):
                ports.append(p)
        for pattern in ["/dev/cu.usbserial*", "/dev/cu.usbmodem*"]:
            ports.extend(sorted(glob(pattern)))
        return ports
