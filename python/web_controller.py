"""
Delta Robot — Web Controller
Serves a browser UI on http://localhost:5000 that talks to the Arduino.

Usage:
    python web_controller.py                        # auto-detect port
    python web_controller.py --port /dev/ttyUSB0    # explicit port
    python web_controller.py --sim                   # simulation mode (no Arduino)

Requirements:
    pip install -r requirements.txt
"""

from __future__ import annotations

import argparse
import json
import logging
import threading
import time
from typing import Optional

from flask import Flask, jsonify, render_template, request
import serial.tools.list_ports

from delta_robot import DeltaRobot, DeltaRobotError
from delta_kinematics import DeltaKinematics

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("web_controller")

app = Flask(__name__)

# ====================================================================== #
#  Global robot state
# ====================================================================== #

robot: Optional[DeltaRobot] = None
robot_lock = threading.Lock()

# Cached state (updated by background thread)
state = {
    "connected": False,
    "port": None,
    "positions": [0, 0, 0],
    "moving": False,
    "speed_rpm": 60.0,
    "accel_rpm_s": 120.0,
    "enabled": True,
}

# Command log (ring buffer, newest first)
command_log: list[dict] = []
LOG_MAX = 50

# Inverse kinematics (all dimensions in mm)
ik_config = {
    "upper_arm": 150.0,
    "lower_arm": 271.0,
    "Fd": 36.7,
    "Ed": 80.0,
}
dk = DeltaKinematics(**ik_config)


def log_command(cmd: str, response: str, ok: bool = True) -> None:
    entry = {
        "time": time.strftime("%H:%M:%S"),
        "cmd": cmd,
        "response": response,
        "ok": ok,
    }
    command_log.insert(0, entry)
    if len(command_log) > LOG_MAX:
        command_log.pop()


# ====================================================================== #
#  Simulation mode (no Arduino needed)
# ====================================================================== #


class SimulatedRobot:
    """Drop-in fake that mimics DeltaRobot for UI testing."""

    def __init__(self):
        self._positions = [0.0, 0.0, 0.0]
        self._targets = [0.0, 0.0, 0.0]
        self._speed = 60.0
        self._accel = 120.0
        self._enabled = True
        self._moving = False
        self._move_thread: Optional[threading.Thread] = None

    def _animate(self):
        """Slowly interpolate positions toward targets."""
        self._moving = True
        while True:
            done = True
            for i in range(3):
                diff = self._targets[i] - self._positions[i]
                if abs(diff) > 0.5:
                    step = min(abs(diff), self._speed / 10.0)
                    self._positions[i] += step if diff > 0 else -step
                    done = False
            if done:
                for i in range(3):
                    self._positions[i] = self._targets[i]
                break
            time.sleep(0.05)
        self._moving = False

    def _start_move(self):
        if self._move_thread and self._move_thread.is_alive():
            pass  # let it finish
        self._move_thread = threading.Thread(target=self._animate, daemon=True)
        self._move_thread.start()

    def send(self, command):
        return f"OK: SIM {command}"

    def move_to(self, d1, d2, d3):
        self._targets = [d1, d2, d3]
        self._start_move()
        return "OK: SIM Moving"

    def move_motor(self, motor, degrees):
        self._targets[motor - 1] = degrees
        self._start_move()
        return f"OK: SIM Motor {motor}"

    def move_relative(self, d1, d2, d3):
        self._targets[0] = self._positions[0] + d1
        self._targets[1] = self._positions[1] + d2
        self._targets[2] = self._positions[2] + d3
        self._start_move()
        return "OK: SIM Relative"

    def home(self):
        self._targets = [0.0, 0.0, 0.0]
        self._start_move()
        return "OK: SIM Homing"

    def stop(self):
        self._targets = list(self._positions)
        self._moving = False
        return "OK: SIM Stop"

    def emergency_stop(self):
        self._targets = list(self._positions)
        self._moving = False
        return "OK: SIM E-Stop"

    def set_speed(self, rpm):
        self._speed = rpm
        return f"OK: SIM Speed {rpm}"

    def set_acceleration(self, val):
        self._accel = val
        return f"OK: SIM Accel {val}"

    def enable(self):
        self._enabled = True
        return "OK: SIM Enabled"

    def disable(self):
        self._enabled = False
        return "OK: SIM Disabled"

    def zero(self):
        self._positions = [0.0, 0.0, 0.0]
        self._targets = [0.0, 0.0, 0.0]
        return "OK: SIM Zeroed"

    def get_position(self):
        return tuple(int(p) for p in self._positions)

    def is_moving(self):
        return self._moving

    def close(self):
        pass


# ====================================================================== #
#  Background status poller
# ====================================================================== #


def status_poller():
    """Periodically update cached state from the Arduino."""
    while True:
        time.sleep(0.25)
        with robot_lock:
            if robot is None:
                state["connected"] = False
                continue
            try:
                pos = robot.get_position()
                state["positions"] = list(pos)
                state["moving"] = robot.is_moving()
                state["connected"] = True
            except Exception:
                state["connected"] = False


poller_thread = threading.Thread(target=status_poller, daemon=True)
poller_thread.start()


# ====================================================================== #
#  Helper: execute a robot command safely
# ====================================================================== #


def robot_exec(label: str, fn, *args, **kwargs):
    """Call a robot method under lock, return (response, ok) tuple."""
    with robot_lock:
        if robot is None:
            return "Not connected", False
        try:
            result = fn(*args, **kwargs)
            resp = str(result) if result is not None else "OK"
            log_command(label, resp, ok=True)
            return resp, True
        except DeltaRobotError as e:
            log_command(label, str(e), ok=False)
            return str(e), False
        except Exception as e:
            log_command(label, str(e), ok=False)
            return str(e), False


# ====================================================================== #
#  Routes — Pages
# ====================================================================== #


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/cartesian")
def cartesian():
    return render_template("cartesian.html")


# ====================================================================== #
#  Routes — API
# ====================================================================== #


@app.route("/api/ports")
def api_ports():
    """List available serial ports."""
    ports = [
        {"device": p.device, "description": p.description}
        for p in serial.tools.list_ports.comports()
    ]
    return jsonify(ports)


@app.route("/api/connect", methods=["POST"])
def api_connect():
    global robot
    data = request.json or {}
    port = data.get("port", "")
    sim = data.get("sim", False)

    with robot_lock:
        if robot is not None:
            try:
                robot.close()
            except Exception:
                pass
            robot = None

        try:
            if sim:
                robot = SimulatedRobot()
                state["port"] = "SIMULATION"
                state["connected"] = True
                log_command("CONNECT", "Simulation mode", ok=True)
                return jsonify({"ok": True, "message": "Simulation mode active"})
            else:
                robot = DeltaRobot(port)
                state["port"] = port
                state["connected"] = True
                state["speed_rpm"] = 60.0
                state["accel_rpm_s"] = 120.0
                log_command("CONNECT", f"Connected to {port}", ok=True)
                return jsonify({"ok": True, "message": f"Connected to {port}"})
        except Exception as e:
            robot = None
            state["connected"] = False
            return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    global robot
    with robot_lock:
        if robot:
            try:
                robot.close()
            except Exception:
                pass
            robot = None
        state["connected"] = False
        state["port"] = None
        log_command("DISCONNECT", "Disconnected", ok=True)
    return jsonify({"ok": True})


@app.route("/api/state")
def api_state():
    """Return the current cached state (polled by the UI)."""
    return jsonify(state)


@app.route("/api/log")
def api_log():
    return jsonify(command_log)


# ---------- Motion commands ----------


@app.route("/api/move", methods=["POST"])
def api_move():
    data = request.json or {}
    d1, d2, d3 = float(data["d1"]), float(data["d2"]), float(data["d3"])
    resp, ok = robot_exec(f"M {d1} {d2} {d3}", robot.move_to, d1, d2, d3)
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/move_motor", methods=["POST"])
def api_move_motor():
    data = request.json or {}
    motor = int(data["motor"])
    deg = float(data["degrees"])
    resp, ok = robot_exec(f"M{motor} {deg}", robot.move_motor, motor, deg)
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/move_relative", methods=["POST"])
def api_move_relative():
    data = request.json or {}
    d1, d2, d3 = float(data["d1"]), float(data["d2"]), float(data["d3"])
    resp, ok = robot_exec(f"R {d1} {d2} {d3}", robot.move_relative, d1, d2, d3)
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/home", methods=["POST"])
def api_home():
    resp, ok = robot_exec("HOME", robot.home)
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    resp, ok = robot_exec("STOP", robot.stop)
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    resp, ok = robot_exec("ESTOP", robot.emergency_stop)
    return jsonify({"ok": ok, "response": resp})


# ---------- Configuration ----------


@app.route("/api/speed", methods=["POST"])
def api_speed():
    data = request.json or {}
    rpm = float(data["rpm"])
    resp, ok = robot_exec(f"SPD {rpm}", robot.set_speed, rpm)
    if ok:
        state["speed_rpm"] = rpm
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/acceleration", methods=["POST"])
def api_acceleration():
    data = request.json or {}
    val = float(data["value"])
    resp, ok = robot_exec(f"ACC {val}", robot.set_acceleration, val)
    if ok:
        state["accel_rpm_s"] = val
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/enable", methods=["POST"])
def api_enable():
    resp, ok = robot_exec("ENABLE", robot.enable)
    if ok:
        state["enabled"] = True
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/disable", methods=["POST"])
def api_disable():
    resp, ok = robot_exec("DISABLE", robot.disable)
    if ok:
        state["enabled"] = False
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/zero", methods=["POST"])
def api_zero():
    resp, ok = robot_exec("ZERO", robot.zero)
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/raw", methods=["POST"])
def api_raw():
    """Send a raw command string."""
    data = request.json or {}
    cmd = data.get("command", "").strip()
    if not cmd:
        return jsonify({"ok": False, "response": "Empty command"}), 400
    resp, ok = robot_exec(cmd, robot.send, cmd)
    return jsonify({"ok": ok, "response": resp})


# ---------- Inverse Kinematics ----------


@app.route("/api/move_xyz", methods=["POST"])
def api_move_xyz():
    """Compute IK for (x, y, z) and send the resulting angles to the robot."""
    data = request.json or {}
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    z = float(data.get("z", -5))
    try:
        t1, t2, t3 = dk.inverse(x, y, z)
        resp, ok = robot_exec(
            f"IK ({x:.2f},{y:.2f},{z:.2f}) \u2192 ({t1:.1f}\u00b0,{t2:.1f}\u00b0,{t3:.1f}\u00b0)",
            robot.move_to, t1, t2, t3,
        )
        return jsonify({"ok": ok, "response": resp, "angles": [t1, t2, t3]})
    except ValueError as e:
        return jsonify({"ok": False, "response": str(e), "angles": None}), 400


@app.route("/api/ik_config", methods=["GET", "POST"])
def api_ik_config():
    """Read or update the robot geometry used for inverse kinematics."""
    global dk
    if request.method == "POST":
        data = request.json or {}
        for key in ("upper_arm", "lower_arm", "Fd", "Ed"):
            if key in data:
                ik_config[key] = float(data[key])
        dk = DeltaKinematics(**ik_config)
        return jsonify({"ok": True, **ik_config})
    return jsonify(ik_config)


# ====================================================================== #
#  Main
# ====================================================================== #


def main():
    parser = argparse.ArgumentParser(description="Delta Robot Web Controller")
    parser.add_argument("--port", type=str, default=None, help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Web server host")
    parser.add_argument("--web-port", type=int, default=8080, help="Web server port")
    parser.add_argument(
        "--sim", action="store_true", help="Simulation mode (no Arduino)"
    )
    args = parser.parse_args()

    global robot

    # Auto-connect if port given or sim mode
    if args.sim:
        robot = SimulatedRobot()
        state["connected"] = True
        state["port"] = "SIMULATION"
        log.info("Running in SIMULATION mode")
    elif args.port:
        try:
            robot = DeltaRobot(args.port, args.baud)
            state["connected"] = True
            state["port"] = args.port
        except Exception as e:
            log.error("Could not connect to %s: %s", args.port, e)
            log.info("Start without connection — use the web UI to connect.")

    log.info("Starting web server at http://%s:%d", args.host, args.web_port)
    app.run(host=args.host, port=args.web_port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
