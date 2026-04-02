"""
Picker — Web Controller
Serves a browser UI on http://localhost:8080 that talks to the ESP32.

Usage:
    python web_controller.py                        # start disconnected
    python web_controller.py --port /dev/ttyUSB0    # pre-connect to port

Requirements:
    pip install -r requirements.txt
"""

from __future__ import annotations

import argparse
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
    "gantry_mm": 0.0,
    "moving": False,
    "speed_rpm": 60.0,
    "accel_rpm_s": 120.0,
    "enabled": True,
    "last_command": None,
}

# Command log (ring buffer, newest first)
command_log: list[dict] = []
LOG_MAX = 50

# Inverse kinematics (all dimensions in mm)
ik_config = {
    "upper_arm": 150.0,
    "lower_arm": 268.0,
    "Fd": 82.5,
    "Ed": 27.3,
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
#  Background status poller
# ====================================================================== #


def status_poller():
    """Periodically update cached state from the robot."""
    while True:
        time.sleep(0.25)
        with robot_lock:
            if robot is None:
                state["connected"] = False
                continue
            try:
                pos = robot.get_position()
                state["positions"] = list(pos[:3])
                state["gantry_mm"] = pos[3] if len(pos) > 3 else 0.0
                state["moving"] = robot.is_moving()
                state["connected"] = True
            except Exception as e:
                state["connected"] = False
                log.debug("Poller error: %s", e)


poller_thread = threading.Thread(target=status_poller, daemon=True)
poller_thread.start()


# ====================================================================== #
#  Helper: execute a robot command safely
# ====================================================================== #


def robot_exec(label: str, fn, *args, **kwargs):
    """Call a robot method under lock, return (response, ok) tuple."""
    log.info("CMD  %s  args=%s", label, args or "")
    with robot_lock:
        if robot is None:
            log.warning("CMD  %s  => not connected", label)
            state["last_command"] = {
                "cmd": label,
                "ok": False,
                "response": "Not connected",
                "time": time.strftime("%H:%M:%S"),
            }
            return "Not connected", False
        try:
            result = fn(*args, **kwargs)
            resp = str(result) if result is not None else "OK"
            log.info("CMD  %s  => OK  %s", label, resp[:120])
            log_command(label, resp, ok=True)
            state["last_command"] = {
                "cmd": label,
                "ok": True,
                "response": resp,
                "time": time.strftime("%H:%M:%S"),
            }
            return resp, True
        except DeltaRobotError as e:
            log.error("CMD  %s  => ERR  %s", label, e)
            log_command(label, str(e), ok=False)
            state["last_command"] = {
                "cmd": label,
                "ok": False,
                "response": str(e),
                "time": time.strftime("%H:%M:%S"),
            }
            return str(e), False
        except Exception as e:
            log.error("CMD  %s  => ERR  %s", label, e)
            log_command(label, str(e), ok=False)
            state["last_command"] = {
                "cmd": label,
                "ok": False,
                "response": str(e),
                "time": time.strftime("%H:%M:%S"),
            }
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


@app.route("/homing")
def homing():
    return render_template("homing.html")


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

    if not port:
        return jsonify({"ok": False, "message": "No port specified"}), 400

    log.info("Connecting to %s …", port)

    with robot_lock:
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass
            robot = None

        try:
            r = DeltaRobot(port)
            r.connect()
            robot = r
            state["port"] = port
            state["connected"] = True
            log_command("CONNECT", f"Connected to {port}", ok=True)
            log.info("Connected to %s", port)
            return jsonify({"ok": True, "message": f"Connected to {port}"})
        except Exception as e:
            robot = None
            state["connected"] = False
            log.error("Connection failed: %s", e)
            return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    global robot
    with robot_lock:
        if robot:
            try:
                robot.disconnect()
            except Exception:
                pass
            robot = None
        state["connected"] = False
        log_command("DISCONNECT", "Disconnected", ok=True)
    return jsonify({"ok": True})


@app.route("/api/state")
def api_state():
    """Return the current cached state (polled by the UI)."""
    from coordinates import DELTA_KINEMATIC_AT_FIRMWARE_ZERO
    return jsonify({**state, "delta_offset": DELTA_KINEMATIC_AT_FIRMWARE_ZERO})


@app.route("/api/log")
def api_log():
    return jsonify(command_log)


# ---------- Motion commands ----------


@app.route("/api/move", methods=["POST"])
def api_move():
    """Move delta motors to absolute joint angles (degrees)."""
    data = request.json or {}
    d1, d2, d3 = float(data["d1"]), float(data["d2"]), float(data["d3"])
    # Use a lambda so robot is looked up inside robot_exec (after the None check)
    resp, ok = robot_exec(
        f"M {d1} {d2} {d3}",
        lambda a1, a2, a3: robot.move_delta(a1, a2, a3),
        d1,
        d2,
        d3,
    )
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/move_gantry", methods=["POST"])
def api_move_gantry():
    """Move the gantry to an absolute X position (mm)."""
    data = request.json or {}
    x_mm = float(data["x"])
    resp, ok = robot_exec(
        f"G {x_mm}",
        lambda x: robot.move_gantry(x),
        x_mm,
    )
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/home", methods=["POST"])
def api_home():
    resp, ok = robot_exec("HOME", lambda: robot.home())
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/delta_home", methods=["GET"])
def api_delta_home():
    """Return the saved delta home angles (from delta_home.json or defaults)."""
    from coordinates import get_default_home

    home = get_default_home()
    return jsonify(
        {
            "delta_angle_1": home.delta_angle_1,
            "delta_angle_2": home.delta_angle_2,
            "delta_angle_3": home.delta_angle_3,
        }
    )


@app.route("/api/set_delta_home", methods=["POST"])
def api_set_delta_home():
    """
    Save delta home angles.
    Body: {} or { "use_zero": true }.
    - use_zero false/omit: capture current robot position and save (use after driving to limit).
    - use_zero true: save (0, 0, 0) — use after manual push then "Set Zero Here".
    """
    from coordinates import save_delta_home

    data = request.json or {}
    use_zero = data.get("use_zero", False)
    if use_zero:
        # Firmware 0 = physical home = kinematic DELTA_KINEMATIC_AT_FIRMWARE_ZERO (e.g. -20°)
        from coordinates import DELTA_KINEMATIC_AT_FIRMWARE_ZERO as k0

        save_delta_home(k0, k0, k0)
        log_command(
            "SET_DELTA_HOME", f"Saved ({k0}, {k0}, {k0})° kinematic (use_zero)", ok=True
        )
        return jsonify(
            {"ok": True, "delta_angle_1": k0, "delta_angle_2": k0, "delta_angle_3": k0}
        )
    with robot_lock:
        if robot is None:
            return jsonify({"ok": False, "message": "Not connected"}), 400
        try:
            d1, d2, d3, _ = robot.get_position()
            save_delta_home(d1, d2, d3)
            log_command(
                "SET_DELTA_HOME", f"Saved ({d1:.2f}, {d2:.2f}, {d3:.2f})°", ok=True
            )
            return jsonify(
                {
                    "ok": True,
                    "delta_angle_1": d1,
                    "delta_angle_2": d2,
                    "delta_angle_3": d3,
                }
            )
        except Exception as e:
            return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/full_home", methods=["POST"])
def api_full_home():
    """Run full homing: gantry to endstop, delta to saved home angles, gripper open."""
    from coordinates import get_default_home
    from homing import run_homing_sequence

    data = request.json or {}
    home_gantry = data.get("home_gantry", True)
    home_delta = data.get("home_delta", True)
    home_gripper = data.get("home_gripper", True)
    resp, ok = robot_exec(
        "FULL_HOME",
        lambda: run_homing_sequence(
            robot,
            get_default_home(),
            home_gantry=home_gantry,
            home_delta=home_delta,
            home_gripper=home_gripper,
        ),
    )
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/rectangle", methods=["POST"])
def api_rectangle():
    """Run a 2D rectangle: gantry to max, delta to max depth, gantry back, delta up.

    All delta angles are in **kinematic** space (move_delta expects kinematic).
    Home = DELTA_KINEMATIC_AT_FIRMWARE_ZERO (-20°), max = home + 95° = 75°.
    """
    from coordinates import (
        GANTRY_X_MAX,
        DELTA_KINEMATIC_AT_FIRMWARE_ZERO,
        DELTA_ANGLE_RANGE_FROM_HOME,
    )

    delta_home = DELTA_KINEMATIC_AT_FIRMWARE_ZERO                   # -20° kinematic
    delta_max = DELTA_KINEMATIC_AT_FIRMWARE_ZERO + DELTA_ANGLE_RANGE_FROM_HOME  # 75° kinematic
    gantry_max = GANTRY_X_MAX                                       # 625 mm

    def _run_rectangle():
        def _wait_idle(timeout: float = 60.0):
            deadline = time.time() + timeout
            time.sleep(0.3)
            while time.time() < deadline:
                if not robot.is_moving():
                    return
                time.sleep(0.2)
            raise TimeoutError("Timed out waiting for motion to complete")

        robot.move_gantry(gantry_max)
        _wait_idle()

        robot.move_delta(delta_max, delta_max, delta_max)
        _wait_idle()

        robot.move_gantry(0.0)
        _wait_idle()

        robot.move_delta(delta_home, delta_home, delta_home)
        _wait_idle()

        return "Rectangle complete"

    resp, ok = robot_exec("RECTANGLE", lambda: _run_rectangle())
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    resp, ok = robot_exec("STOP", lambda: robot.stop())
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    resp, ok = robot_exec("ESTOP", lambda: robot.emergency_stop())
    return jsonify({"ok": ok, "response": resp})


# ---------- Configuration ----------


@app.route("/api/speed", methods=["POST"])
def api_speed():
    data = request.json or {}
    val = float(data.get("steps_per_sec", data.get("rpm", 2000)))
    resp, ok = robot_exec(
        f"SPD {val}",
        lambda v: robot.set_delta_speed(v),
        val,
    )
    if ok:
        state["speed_rpm"] = val
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/acceleration", methods=["POST"])
def api_acceleration():
    data = request.json or {}
    val = float(data["value"])
    resp, ok = robot_exec(
        f"ACC {val}",
        lambda v: robot.set_delta_accel(v),
        val,
    )
    if ok:
        state["accel_rpm_s"] = val
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/zero", methods=["POST"])
def api_zero():
    resp, ok = robot_exec("ZERO", lambda: robot.zero())
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/raw", methods=["POST"])
def api_raw():
    """Send a raw serial command string."""
    data = request.json or {}
    cmd = data.get("command", "").strip()
    if not cmd:
        return jsonify({"ok": False, "response": "Empty command"}), 400
    resp, ok = robot_exec(
        cmd,
        lambda c: robot._command(c),
        cmd,
    )
    return jsonify({"ok": ok, "response": resp})


# ---------- Inverse Kinematics ----------


@app.route("/api/move_xyz", methods=["POST"])
def api_move_xyz():
    """Move delta end-effector to Cartesian (x, y, z) in mm."""
    data = request.json or {}
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    z = float(data.get("z", -200))
    try:
        resp, ok = robot_exec(
            f"XYZ ({x:.1f}, {y:.1f}, {z:.1f})",
            lambda x_, y_, z_: robot.move_to_xyz(x_, y_, z_),
            x,
            y,
            z,
        )
        if ok and isinstance(resp, str) and resp.startswith("("):
            angles = None
        elif ok:
            angles = list(resp) if not isinstance(resp, str) else None
        else:
            angles = None
        return jsonify({"ok": ok, "response": str(resp), "angles": angles})
    except (DeltaRobotError, ValueError) as e:
        return jsonify({"ok": False, "response": str(e), "angles": None}), 400


@app.route("/api/move_position", methods=["POST"])
def api_move_position():
    """Move gantry + delta to a desired end-effector pose.

    Body: ``{"gantry_x": 400, "x": 50, "y": 0, "z": -200}``
    """
    data = request.json or {}
    gantry_x = float(data.get("gantry_x", 0))
    x = float(data.get("x", 0))
    y = float(data.get("y", 0))
    z = float(data.get("z", -200))
    try:
        resp, ok = robot_exec(
            f"POS G={gantry_x:.1f} ({x:.1f}, {y:.1f}, {z:.1f})",
            lambda gx, x_, y_, z_: robot.move_to_position(gx, x_, y_, z_),
            gantry_x,
            x,
            y,
            z,
        )
        if ok and not isinstance(resp, str):
            angles = list(resp)
        else:
            angles = None
        return jsonify(
            {
                "ok": ok,
                "response": str(resp),
                "angles": angles,
                "gantry_x": gantry_x,
            }
        )
    except (DeltaRobotError, ValueError) as e:
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
        if robot is not None:
            robot.ik = dk
        return jsonify({"ok": True, **ik_config})
    return jsonify(ik_config)


# ====================================================================== #
#  Main
# ====================================================================== #


def main():
    parser = argparse.ArgumentParser(description="Picker — Web Controller")
    parser.add_argument(
        "--port", type=str, default=None, help="Serial port to pre-connect"
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Web server host")
    parser.add_argument("--web-port", type=int, default=8080, help="Web server port")
    args = parser.parse_args()

    global robot

    if args.port:
        try:
            r = DeltaRobot(args.port, args.baud)
            r.connect()
            robot = r
            state["connected"] = True
            state["port"] = args.port
            log.info("Pre-connected to %s from --port", args.port)
        except Exception as e:
            log.error("Could not connect to %s: %s", args.port, e)
            log.info("Start without connection — use the web UI to connect.")

    log.info("Starting web server at http://%s:%d", args.host, args.web_port)
    app.run(host=args.host, port=args.web_port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
