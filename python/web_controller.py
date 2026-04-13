"""
Picker — Web Controller
A Flask web controller that serves a browser UI on http://localhost:8080 that talks to the ESP32.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np
from flask import (
    Flask,
    Response,
    jsonify,
    redirect,
    render_template,
    request,
    stream_with_context,
)
import serial.tools.list_ports

from delta_robot import DeltaRobot, DeltaRobotError
from delta_kinematics import DeltaKinematics
from calibration.kabsch import kabsch
from object_detection import ObjectDetector

try:
    from camera import RealsenseCamera, realsense_available
except ImportError:
    RealsenseCamera = None  # type: ignore[assignment,misc]

    def realsense_available() -> bool:  # type: ignore[misc]
        return False


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

# Camera
camera: Optional[RealsenseCamera] = None  # type: ignore[type-arg]
camera_lock = threading.Lock()

# Command log (ring buffer, newest first)
command_log: list[dict] = []
LOG_MAX = 50

# Calibration state
CALIBRATION_DIR = Path(__file__).parent / "calibration"
CALIBRATION_FILE = CALIBRATION_DIR / "camera_transform.json"
calibration_points: list[dict] = []  # [{"d_point": [x,y,z], "c_point": [x,y,z]}]
calibration_result: dict | None = None  # {"R": ..., "t": ..., "rmsd": ..., ...}
auto_cal_cancel = threading.Event()

# Object detection
detector = ObjectDetector()

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


@app.route("/homing")
def homing():
    return redirect("/", code=302)


@app.route("/calibration")
def calibration():
    return redirect("/", code=302)


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
    """Return the delta home angles."""
    from coordinates import DELTA_HOME_ANGLE_1, DELTA_HOME_ANGLE_2, DELTA_HOME_ANGLE_3

    return jsonify(
        {
            "delta_angle_1": DELTA_HOME_ANGLE_1,
            "delta_angle_2": DELTA_HOME_ANGLE_2,
            "delta_angle_3": DELTA_HOME_ANGLE_3,
        }
    )


@app.route("/api/home_xyz", methods=["GET"])
def api_home_xyz():
    """Return the XYZ position corresponding to the delta home angles."""
    from coordinates import DELTA_HOME_ANGLE_1, DELTA_HOME_ANGLE_2, DELTA_HOME_ANGLE_3

    x, y, z = dk.forward(DELTA_HOME_ANGLE_1, DELTA_HOME_ANGLE_2, DELTA_HOME_ANGLE_3)
    return jsonify({"x": round(x, 2), "y": round(y, 2), "z": round(z, 2)})


@app.route("/api/full_home", methods=["POST"])
def api_full_home():
    """Run full homing: gantry to endstop, delta to home angles, gripper open."""
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


@app.route("/api/grip", methods=["POST"])
def api_grip():
    """Control the gripper: OPEN, CLOSE, or a raw position."""
    data = request.json or {}
    action = data.get("action", "").upper()
    if action == "OPEN":
        resp, ok = robot_exec("GRIP OPEN", lambda: robot.grip_open())
    elif action == "CLOSE":
        resp, ok = robot_exec("GRIP CLOSE", lambda: robot.grip_close())
    else:
        pos = int(data.get("position", 0))
        resp, ok = robot_exec(f"GRIP {pos}", lambda p: robot.grip_position(p), pos)
    return jsonify({"ok": ok, "response": resp})


@app.route("/api/rectangle", methods=["POST"])
def api_rectangle():
    """Run a 2D rectangle: gantry to max, delta to max depth, gantry back, delta up.

    All delta angles are in **kinematic** space (move_delta expects kinematic).
    Home = -15° kinematic (15° above horizontal), max = 80° kinematic.
    """
    from coordinates import (
        GANTRY_X_MAX,
        DELTA_HOME_ANGLE_1,
        DELTA_KINEMATIC_AT_FIRMWARE_ZERO,
        DELTA_FIRMWARE_MAX_ANGLE,
    )

    delta_home = DELTA_HOME_ANGLE_1  # -15° kinematic (park position)
    delta_max = (
        DELTA_KINEMATIC_AT_FIRMWARE_ZERO + DELTA_FIRMWARE_MAX_ANGLE
    )  # 80° kinematic
    gantry_max = GANTRY_X_MAX  # 625 mm

    def _run_rectangle():
        def _wait_idle(timeout: float = 60.0):
            deadline = time.time() + timeout
            time.sleep(0.3)
            while time.time() < deadline:
                if not robot.is_moving():
                    return
                time.sleep(0.2)
            raise TimeoutError("Timed out waiting for motion to complete")

        robot.grip_open()
        robot.move_gantry(gantry_max)
        _wait_idle()

        robot.move_delta(delta_max, delta_max, delta_max)
        _wait_idle()

        robot.grip_close()
        time.sleep(0.5)

        robot.move_delta(delta_home, delta_home, delta_home)
        _wait_idle()

        robot.move_gantry(0.0)
        _wait_idle()

        robot.move_delta(delta_max, delta_max, delta_max)
        _wait_idle()

        robot.grip_open()
        time.sleep(0.5)

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
    z = float(data.get("z", 200))
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
    z = float(data.get("z", 200))
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


# ---------- Camera ----------


@app.route("/api/camera/status")
def api_camera_status():
    return jsonify(
        {
            "available": realsense_available(),
            "running": camera is not None and camera.running,
        }
    )


@app.route("/api/camera/start", methods=["POST"])
def api_camera_start():
    global camera
    if not realsense_available():
        return jsonify({"ok": False, "message": "pyrealsense2 not available"}), 503
    with camera_lock:
        if camera and camera.running:
            return jsonify({"ok": True, "message": "Already running"})
        try:
            camera = RealsenseCamera()
            camera.start()
            return jsonify({"ok": True})
        except Exception as e:
            return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/camera/stop", methods=["POST"])
def api_camera_stop():
    global camera
    with camera_lock:
        if camera:
            camera.stop()
            camera = None
    return jsonify({"ok": True})


@app.route("/api/camera/feed")
def api_camera_feed():
    if not camera or not camera.running:
        return "Camera not running", 503
    return Response(
        camera.generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/api/camera/overlay", methods=["POST"])
def api_camera_overlay():
    data = request.json or {}
    with camera_lock:
        if not camera or not camera.running:
            return jsonify({"ok": False, "message": "Camera not running"}), 400
        camera.set_overlay(
            aruco=bool(data.get("aruco", False)),
            detections=bool(data.get("detections", False)),
        )
    return jsonify({"ok": True})


# ---------- Calibration ----------


@app.route("/api/calibration/state")
def api_calibration_state():
    """Return current calibration session: collected points and solved result."""
    saved = None
    if CALIBRATION_FILE.exists():
        try:
            saved = json.loads(CALIBRATION_FILE.read_text())
        except Exception:
            pass
    return jsonify(
        {
            "points": calibration_points,
            "num_points": len(calibration_points),
            "result": calibration_result,
            "saved": saved,
        }
    )


@app.route("/api/calibration/capture", methods=["POST"])
def api_calibration_capture():
    """Capture one calibration point: detect ArUco, deproject, get FK position."""
    data = request.json or {}
    aruco_dict = data.get("aruco_dict", "DICT_4X4_50")
    marker_id_filter = data.get("marker_id")  # None = accept any
    offset = data.get("marker_offset", None)

    if offset is None:
        from coordinates import MARKER_OFFSET_FROM_EE

        offset = list(MARKER_OFFSET_FROM_EE)

    with camera_lock:
        if not camera or not camera.running:
            return jsonify({"ok": False, "message": "Camera not running"}), 400
        markers = camera.detect_aruco(aruco_dict)
        if not markers:
            return jsonify({"ok": False, "message": "No ArUco marker detected"}), 400

        marker = markers[0]
        if marker_id_filter is not None:
            matching = [m for m in markers if m["id"] == marker_id_filter]
            if not matching:
                return (
                    jsonify(
                        {
                            "ok": False,
                            "message": f"Marker ID {marker_id_filter} not found (saw {[m['id'] for m in markers]})",
                        }
                    ),
                    400,
                )
            marker = matching[0]

        cu, cv = marker["center"]
        c_point = camera.deproject(int(round(cu)), int(round(cv)))
        if c_point is None:
            return jsonify({"ok": False, "message": "No depth at marker center"}), 400

    if not state["connected"]:
        return jsonify({"ok": False, "message": "Robot not connected"}), 400

    positions = state["positions"]
    fk_pos = dk.forward(positions[0], positions[1], positions[2])
    d_point = [
        fk_pos[0] + offset[0],
        fk_pos[1] + offset[1],
        fk_pos[2] + offset[2],
    ]

    entry = {
        "d_point": [round(v, 2) for v in d_point],
        "c_point": [round(v, 2) for v in c_point],
        "angles": [round(v, 2) for v in positions],
        "marker_id": marker["id"],
        "pixel": [round(cu, 1), round(cv, 1)],
    }
    calibration_points.append(entry)

    return jsonify({"ok": True, "point": entry, "index": len(calibration_points) - 1})


@app.route("/api/calibration/remove", methods=["POST"])
def api_calibration_remove():
    """Remove a calibration point by index."""
    global calibration_result
    data = request.json or {}
    idx = int(data.get("index", -1))
    if 0 <= idx < len(calibration_points):
        calibration_points.pop(idx)
        calibration_result = None
        return jsonify({"ok": True})
    return jsonify({"ok": False, "message": "Invalid index"}), 400


@app.route("/api/calibration/solve", methods=["POST"])
def api_calibration_solve():
    """Run Kabsch on collected points to compute D_T_c."""
    global calibration_result
    if len(calibration_points) < 4:
        return jsonify({"ok": False, "message": "Need at least 4 points"}), 400

    C = np.array([p["c_point"] for p in calibration_points])
    D = np.array([p["d_point"] for p in calibration_points])

    R, t, rmsd, residuals = kabsch(C, D)

    calibration_result = {
        "R": R.tolist(),
        "t": t.tolist(),
        "rmsd": round(rmsd, 3),
        "residuals": [round(float(r), 3) for r in residuals],
        "num_points": len(calibration_points),
    }
    return jsonify({"ok": True, **calibration_result})


@app.route("/api/calibration/save", methods=["POST"])
def api_calibration_save():
    """Persist the solved D_T_c to disk."""
    if calibration_result is None:
        return jsonify({"ok": False, "message": "No calibration result to save"}), 400

    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        **calibration_result,
        "timestamp": datetime.now(timezone.utc).isoformat(),
    }
    CALIBRATION_FILE.write_text(json.dumps(payload, indent=2))
    log.info(
        "Calibration saved to %s (RMSD=%.2f mm)",
        CALIBRATION_FILE,
        calibration_result["rmsd"],
    )
    return jsonify({"ok": True, "path": str(CALIBRATION_FILE)})


@app.route("/api/calibration/load")
def api_calibration_load():
    """Load a previously saved D_T_c from disk."""
    if not CALIBRATION_FILE.exists():
        return jsonify({"ok": False, "message": "No saved calibration found"}), 404
    try:
        data = json.loads(CALIBRATION_FILE.read_text())
        return jsonify({"ok": True, **data})
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500


@app.route("/api/calibration/auto")
def api_calibration_auto():
    """Auto-calibrate: move to ~20 spiral waypoints, capture at each. SSE stream."""
    import math

    global calibration_result

    num_points = int(request.args.get("n", 20))

    z_min = 200.0  # avoid high-up positions where depth is unreliable
    z_max = 350.0
    r_max = 90.0  # conservative XY radius to stay well within workspace

    # Generate a spiral that sweeps XY while varying Z.
    # Two full turns, radius grows linearly, Z oscillates between levels.
    waypoints = []
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        angle = t * 4 * math.pi  # 2 full turns
        r = 20.0 + t * (r_max - 20.0)
        x = round(r * math.cos(angle), 1)
        y = round(r * math.sin(angle), 1)
        z = round(z_min + (z_max - z_min) * (0.5 - 0.5 * math.cos(t * 3 * math.pi)), 1)
        waypoints.append((x, y, z))

    def _wait_idle(timeout: float = 15.0):
        deadline = time.time() + timeout
        time.sleep(0.4)
        while time.time() < deadline:
            if not robot.is_moving():
                return True
            time.sleep(0.2)
        return False

    def generate():
        global calibration_result
        auto_cal_cancel.clear()

        for i, (x, y, z) in enumerate(waypoints):
            if auto_cal_cancel.is_set():
                yield f"data: {json.dumps({'type': 'cancelled', 'index': i})}\n\n"
                return

            yield f"data: {json.dumps({'type': 'moving', 'index': i, 'total': len(waypoints), 'target': [x, y, z]})}\n\n"

            with robot_lock:
                if robot is None:
                    yield f"data: {json.dumps({'type': 'error', 'message': 'Robot disconnected'})}\n\n"
                    return
                try:
                    robot.move_to_xyz(x, y, z)
                except (DeltaRobotError, ValueError) as e:
                    yield f"data: {json.dumps({'type': 'skip', 'index': i, 'message': str(e)})}\n\n"
                    continue

            _wait_idle()
            time.sleep(0.5)  # settle time for camera

            if auto_cal_cancel.is_set():
                yield f"data: {json.dumps({'type': 'cancelled', 'index': i})}\n\n"
                return

            # Capture (inline version of api_calibration_capture logic)
            from coordinates import MARKER_OFFSET_FROM_EE

            offset = list(MARKER_OFFSET_FROM_EE)

            with camera_lock:
                if not camera or not camera.running:
                    yield f"data: {json.dumps({'type': 'skip', 'index': i, 'message': 'Camera not running'})}\n\n"
                    continue
                markers = camera.detect_aruco("DICT_4X4_50")
                if not markers:
                    yield f"data: {json.dumps({'type': 'skip', 'index': i, 'message': 'No ArUco marker detected'})}\n\n"
                    continue
                marker = markers[0]
                cu, cv = marker["center"]
                c_point = camera.deproject(int(round(cu)), int(round(cv)))
                if c_point is None:
                    yield f"data: {json.dumps({'type': 'skip', 'index': i, 'message': 'No depth at marker'})}\n\n"
                    continue

            positions = state["positions"]
            fk_pos = dk.forward(positions[0], positions[1], positions[2])
            d_point = [
                fk_pos[0] + offset[0],
                fk_pos[1] + offset[1],
                fk_pos[2] + offset[2],
            ]

            entry = {
                "d_point": [round(v, 2) for v in d_point],
                "c_point": [round(v, 2) for v in c_point],
                "angles": [round(v, 2) for v in positions],
                "marker_id": marker["id"],
                "pixel": [round(cu, 1), round(cv, 1)],
            }
            calibration_points.append(entry)
            calibration_result = None

            yield f"data: {json.dumps({'type': 'captured', 'index': i, 'total': len(waypoints), 'point': entry, 'num_points': len(calibration_points)})}\n\n"

        yield f"data: {json.dumps({'type': 'done', 'num_points': len(calibration_points)})}\n\n"

    return Response(stream_with_context(generate()), content_type="text/event-stream")


@app.route("/api/calibration/auto/cancel", methods=["POST"])
def api_calibration_auto_cancel():
    """Cancel a running auto-calibration."""
    auto_cal_cancel.set()
    return jsonify({"ok": True})


@app.route("/api/calibration/clear", methods=["POST"])
def api_calibration_clear():
    """Discard all collected calibration points and results."""
    global calibration_result
    calibration_points.clear()
    calibration_result = None
    return jsonify({"ok": True})


# ====================================================================== #
#  Object Detection
# ====================================================================== #


def _detections_to_world(detections_raw, cam):
    """Convert raw detections to world-frame dicts.

    Returns a list of dicts with pixel, delta-frame, and world-frame info.
    Drops any detection that has no depth.
    """
    from coordinates import camera_to_robot

    gantry_x = state.get("gantry_mm", 0.0)
    results = []
    for det in detections_raw:
        u, v = int(round(det.center_uv[0])), int(round(det.center_uv[1]))
        c_point = cam.deproject(u, v)
        if c_point is None:
            continue
        d_point = camera_to_robot(np.array(c_point))
        world_x = gantry_x + d_point[0]
        world_y = float(d_point[1])
        world_z = float(d_point[2])
        results.append({
            "pixel": [det.center_uv[0], det.center_uv[1]],
            "bbox": list(det.bbox),
            "label": det.label,
            "confidence": round(det.confidence, 3),
            "delta": [round(float(d_point[0]), 2), round(float(d_point[1]), 2), round(float(d_point[2]), 2)],
            "world": [round(world_x, 2), round(world_y, 2), round(world_z, 2)],
        })
    return results


@app.route("/api/detect/scan", methods=["POST"])
def api_detect_scan():
    """Run object detection on the current camera frame.

    Body (optional): ``{"tray": 3}`` — filter to a specific tray.
    """
    from coordinates import TRAY_REGIONS

    data = request.json or {}
    tray = data.get("tray")

    with camera_lock:
        if not camera or not camera.running:
            return jsonify({"ok": False, "message": "Camera not running"}), 400
        frame = camera.get_color_frame()

    if frame is None:
        return jsonify({"ok": False, "message": "No frame available"}), 400

    raw = detector.detect(frame)
    world_dets = _detections_to_world(raw, camera)

    if tray is not None:
        tray = int(tray)
        region = TRAY_REGIONS.get(tray)
        if region is None:
            return jsonify({"ok": False, "message": f"Unknown tray {tray}"}), 400
        world_dets = [d for d in world_dets if region.contains(d["world"][0], d["world"][1])]

    return jsonify({"ok": True, "tray": tray, "detections": world_dets, "count": len(world_dets)})


@app.route("/api/detect/pick", methods=["POST"])
def api_detect_pick():
    """Detect objects in a tray, pick one at random, and move the TCP there.

    Body: ``{"tray": 3, "class_name": "stepper_motor"}``
    ``tray`` is required.  ``class_name`` is an optional filter.
    """
    import random

    from coordinates import GANTRY_X_MAX, GANTRY_X_MIN, TRAY_REGIONS

    data = request.json or {}
    tray = data.get("tray")
    class_name = data.get("class_name")

    if tray is None:
        return jsonify({"ok": False, "message": "tray is required"}), 400
    tray = int(tray)
    region = TRAY_REGIONS.get(tray)
    if region is None:
        return jsonify({"ok": False, "message": f"Unknown tray {tray}"}), 400

    with camera_lock:
        if not camera or not camera.running:
            return jsonify({"ok": False, "message": "Camera not running"}), 400
        frame = camera.get_color_frame()

    if frame is None:
        return jsonify({"ok": False, "message": "No frame available"}), 400

    raw = detector.detect(frame)
    world_dets = _detections_to_world(raw, camera)

    # Filter to tray
    world_dets = [d for d in world_dets if region.contains(d["world"][0], d["world"][1])]

    # Optional class filter
    if class_name:
        world_dets = [d for d in world_dets if d["label"] == class_name]

    if not world_dets:
        return jsonify({"ok": False, "message": "No detections in tray", "tray": tray, "count": 0}), 404

    chosen = random.choice(world_dets)
    world_x, world_y, world_z = chosen["world"]

    # Split into gantry + delta
    gantry_target = max(GANTRY_X_MIN, min(GANTRY_X_MAX, world_x))
    delta_x = world_x - gantry_target
    delta_y = world_y
    delta_z = world_z

    with robot_lock:
        if robot is None:
            return jsonify({"ok": False, "message": "Robot not connected"}), 400
        try:
            robot.move_to_position_tcp_and_wait(gantry_target, delta_x, delta_y, delta_z)
        except (DeltaRobotError, ValueError) as e:
            return jsonify({"ok": False, "message": str(e), "detection": chosen}), 400

    return jsonify({
        "ok": True,
        "tray": tray,
        "detection": chosen,
        "motion": {
            "gantry_x": round(gantry_target, 2),
            "delta": [round(delta_x, 2), round(delta_y, 2), round(delta_z, 2)],
        },
    })


# ====================================================================== #
#  Main
# ====================================================================== #


def main():
    log.info("Starting web server at http://0.0.0.0:8080")
    app.run(host="0.0.0.0", port=8080, debug=False, threaded=True)


if __name__ == "__main__":
    main()
