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

from dotenv import load_dotenv

load_dotenv(Path(__file__).parent / ".env")

import numpy as np
from flask import (
    Flask,
    Response,
    jsonify,
    render_template,
    request,
    stream_with_context,
)
import serial.tools.list_ports

from delta_robot import DEFAULT_IK, DeltaRobot, DeltaRobotError
from delta_kinematics import DeltaKinematics
from calibration.extrinsic.kabsch import kabsch
from object_detection import ObjectPointer, VLMError

try:
    from camera import StereoCamera, camera_available
except ImportError:
    StereoCamera = None  # type: ignore[assignment,misc]

    def camera_available() -> bool:  # type: ignore[misc]
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
# `zeroed` is the picker's own record of whether its motors have been zeroed
# this session. Set by /api/zero, cleared on every (re)connect. Consumed by
# preflight via /api/state — no attestation from the factory side.
state = {
    "connected": False,
    "port": None,
    "positions": [0, 0, 0],
    "gantry_mm": 0.0,
    "moving": False,
    "zeroed": False,
    "last_command": None,
}

# Camera
camera: Optional[StereoCamera] = None  # type: ignore[type-arg]
camera_lock = threading.Lock()

# Calibration state
CALIBRATION_DIR = Path(__file__).parent / "calibration" / "extrinsic"
CALIBRATION_FILE = CALIBRATION_DIR / "camera_transform.json"
calibration_points: list[dict] = []  # [{"d_point": [x,y,z], "c_point": [x,y,z]}]
calibration_result: dict | None = None  # {"R": ..., "t": ..., "rmsd": ..., ...}
auto_cal_cancel = threading.Event()

# Object pointing (VLM)
pointer = ObjectPointer()
pick_test_cancel = threading.Event()

# Inverse kinematics (all dimensions in mm)
dk = DEFAULT_IK
IK_KEYS = ("upper_arm", "lower_arm", "Fd", "Ed")


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
            resp, ok = "Not connected", False
        else:
            try:
                result = fn(*args, **kwargs)
                resp = str(result) if result is not None else "OK"
                ok = True
                log.info("CMD  %s  => OK  %s", label, resp[:120])
            except Exception as e:
                resp, ok = str(e), False
                log.error("CMD  %s  => ERR  %s", label, e)
        state["last_command"] = {
            "cmd": label,
            "ok": ok,
            "response": resp,
            "time": time.strftime("%H:%M:%S"),
        }
        return resp, ok


# ====================================================================== #
#  Routes — Pages
# ====================================================================== #


@app.route("/")
def index():
    return render_template("index.html")


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
            state["zeroed"] = False
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
        state["zeroed"] = False
    return jsonify({"ok": True})


@app.route("/api/state")
def api_state():
    """Return the current cached state (polled by the UI)."""
    from coordinates import DELTA_KINEMATIC_AT_FIRMWARE_ZERO

    return jsonify({**state, "delta_offset": DELTA_KINEMATIC_AT_FIRMWARE_ZERO})


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
    from coordinates import HomePosition
    from homing import run_homing_sequence

    data = request.json or {}
    home_gantry = data.get("home_gantry", True)
    home_delta = data.get("home_delta", True)
    home_gripper = data.get("home_gripper", True)
    resp, ok = robot_exec(
        "FULL_HOME",
        lambda: run_homing_sequence(
            robot,
            HomePosition(),
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


# ---------- Configuration ----------


@app.route("/api/zero", methods=["POST"])
def api_zero():
    resp, ok = robot_exec("ZERO", lambda: robot.zero())
    if ok:
        state["zeroed"] = True
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
    resp, ok = robot_exec(
        f"XYZ ({x:.1f}, {y:.1f}, {z:.1f})",
        lambda x_, y_, z_: robot.move_to_xyz(x_, y_, z_),
        x,
        y,
        z,
    )
    angles = list(resp) if ok and not isinstance(resp, str) else None
    return jsonify({"ok": ok, "response": str(resp), "angles": angles})


@app.route("/api/ik_config", methods=["GET", "POST"])
def api_ik_config():
    """Read or update the robot geometry used for inverse kinematics."""
    global dk
    if request.method == "POST":
        data = request.json or {}
        params = {k: float(data.get(k, getattr(dk, k))) for k in IK_KEYS}
        dk = DeltaKinematics(**params)
        if robot is not None:
            robot.ik = dk
    return jsonify({k: getattr(dk, k) for k in IK_KEYS})


# ---------- Camera ----------


@app.route("/api/camera/status")
def api_camera_status():
    return jsonify(
        {
            "available": camera_available(),
            "running": camera is not None and camera.running,
        }
    )


@app.route("/api/camera/start", methods=["POST"])
def api_camera_start():
    global camera
    if not camera_available():
        return jsonify({"ok": False, "message": "Camera module not available"}), 503
    with camera_lock:
        if camera and camera.running:
            return jsonify({"ok": True, "message": "Already running"})
        try:
            camera = StereoCamera()
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
        camera.set_overlay(aruco=bool(data.get("aruco", False)))
    return jsonify({"ok": True})


@app.route("/api/camera/snapshot")
def api_camera_snapshot():
    """Single JPEG of the current color frame. For agents that need to
    visually inspect the scene (e.g. 'is the cardboard transporter under
    the delta robot?'). Returns raw image/jpeg bytes."""
    import cv2

    with camera_lock:
        if not camera or not camera.running:
            return jsonify({"ok": False, "message": "Start the camera first"}), 400
        frame = camera.get_color_frame()
    if frame is None:
        return jsonify({"ok": False, "message": "No camera frame available"}), 503
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        return jsonify({"ok": False, "message": "JPEG encode failed"}), 500
    return Response(buf.tobytes(), mimetype="image/jpeg")


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
        c_point = camera.deproject(int(round(cu)), int(round(cv)), patch=5)
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

    inlier_mask = residuals <= 1.5 * rmsd
    calibration_result = {
        "R": R.tolist(),
        "t": t.tolist(),
        "rmsd": round(rmsd, 3),
        "residuals": [round(float(r), 3) for r in residuals],
        "num_points": len(calibration_points),
        "num_inliers": int(inlier_mask.sum()),
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


@app.route("/api/calibration/auto")
def api_calibration_auto():
    """Auto-calibrate: move to ~100 spiral waypoints, capture at each. SSE stream."""
    import math
    from coordinates import MARKER_OFFSET_FROM_EE

    global calibration_result
    offset = list(MARKER_OFFSET_FROM_EE)

    num_points = int(request.args.get("n", 100))

    z_min = 300.0
    z_max = 400.0
    r_max = 90.0

    waypoints = []
    for i in range(num_points):
        t = i / max(num_points - 1, 1)
        angle = t * 6 * math.pi  # 3 full turns
        r = 20.0 + t * (r_max - 20.0)
        x = round(r * math.cos(angle), 1)
        y = round(r * math.sin(angle), 1)
        z = round(z_min + (z_max - z_min) * (0.5 - 0.5 * math.cos(t * 5 * math.pi)), 1)
        waypoints.append((x, y, z))

    def _wait_idle(timeout: float = 15.0):
        deadline = time.time() + timeout
        time.sleep(0.6)
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
            time.sleep(0.8)  # settle time for camera

            if auto_cal_cancel.is_set():
                yield f"data: {json.dumps({'type': 'cancelled', 'index': i})}\n\n"
                return

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

                img_w = camera.capture_width // 2
                img_h = camera.capture_height
                max_r = 0.45 * min(img_w, img_h)
                dist_from_center = (
                    (cu - img_w / 2) ** 2 + (cv - img_h / 2) ** 2
                ) ** 0.5
                if dist_from_center > max_r:
                    yield f"data: {json.dumps({'type': 'skip', 'index': i, 'message': f'Marker too far from center ({dist_from_center:.0f}px)'})}\n\n"
                    continue

                c_point = camera.deproject(int(round(cu)), int(round(cv)), patch=5)
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
#  Pick Test (SSE)
# ====================================================================== #


def _get_description(request_obj) -> str | None:
    """Pull the target description from query args or JSON body."""
    desc = request_obj.args.get("description")
    if not desc and request_obj.is_json:
        desc = (request_obj.get_json(silent=True) or {}).get("description")
    return (desc or "").strip() or None


@app.route("/api/pick", methods=["GET", "POST"])
def api_pick():
    """Run a pick-and-return sequence.  Streams SSE progress events.

    Requires a ``description`` of the object to pick (query string or JSON body).
    """
    from pick import pick_sequence

    description = _get_description(request)
    if not description:
        return jsonify({"ok": False, "message": "description is required"}), 400

    scan_gx = float(request.args.get("gantry_x", 450))
    approach_offset = float(request.args.get("approach_offset", 50))

    with robot_lock:
        if robot is None:
            return jsonify({"ok": False, "message": "Robot not connected"}), 400
    with camera_lock:
        if not camera or not camera.running:
            return jsonify({"ok": False, "message": "Start the camera first"}), 400

    pick_test_cancel.clear()

    def generate():
        for event in pick_sequence(
            robot,
            camera,
            pointer,
            description,
            scan_gx=scan_gx,
            approach_offset=approach_offset,
            cancel=pick_test_cancel,
        ):
            yield f"data: {json.dumps(event)}\n\n"

    return Response(stream_with_context(generate()), content_type="text/event-stream")


@app.route("/api/vlm/point", methods=["GET", "POST"])
def api_vlm_point():
    """Run the VLM on the current camera frame and return an annotated snapshot.

    Does not require the robot. Useful for previewing what Gemini sees before
    (or instead of) running a full pick sequence.
    """
    from pick import _encode_annotated_frame

    description = _get_description(request)
    if not description:
        return jsonify({"ok": False, "message": "description is required"}), 400

    with camera_lock:
        if not camera or not camera.running:
            return jsonify({"ok": False, "message": "Start the camera first"}), 400
        frame = camera.get_color_frame()

    if frame is None:
        return jsonify({"ok": False, "message": "No camera frame available"}), 503

    try:
        point = pointer.point(frame, description)
    except VLMError as exc:
        return jsonify({"ok": False, "message": f"VLM error: {exc}"}), 502

    if point is None:
        return (
            jsonify({"ok": False, "message": f"VLM could not locate {description!r}"}),
            404,
        )

    u, v = int(round(point.u)), int(round(point.v))
    h, w = frame.shape[:2]
    return jsonify(
        {
            "ok": True,
            "description": description,
            "label": point.label,
            "pixel": [u, v],
            "frame_size": [w, h],
            "frame": _encode_annotated_frame(frame, u, v, point.label),
        }
    )


@app.route("/api/pick/cancel", methods=["POST"])
def api_pick_cancel():
    pick_test_cancel.set()
    return jsonify({"ok": True})


# ====================================================================== #
#  Main
# ====================================================================== #


def main():
    import signal, os

    signal.signal(signal.SIGINT, lambda *_: os._exit(0))
    log.info("Starting web server at http://0.0.0.0:8080")
    app.run(host="0.0.0.0", port=8080, debug=False, threaded=True)


if __name__ == "__main__":
    main()
