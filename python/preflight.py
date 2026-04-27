"""Deterministic picker preflight probes.

Each probe queries `web_controller.py` on localhost and returns a dict shaped
like `PreflightCheck` (name, ok, detail, data). Whether a failure is
agent-fixable or human-fixable is decided by the agent — it's a function of
which tools the agent owns, not metadata on the probe itself.

Probes do not mutate robot state except for `gripper_ok`, which is guarded
on the edge state machine being idle.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

import os

import requests

BASE = os.environ.get(
    "PICKER_BASE", f"http://localhost:{os.environ.get('PICKER_PORT', '8080')}"
)
HTTP_TIMEOUT = 8.0

HOME_ANGLE_TOL_DEG = 1.0
HOME_GANTRY_TOL_MM = 1.0

CALIBRATION_FILE = (
    Path(__file__).parent / "calibration" / "extrinsic" / "camera_transform.json"
)

log = logging.getLogger("picker.preflight")


def _check(
    name: str,
    ok: bool,
    detail: str = "",
    data: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {"name": name, "ok": ok, "detail": detail, "data": data or {}}


def _get(path: str) -> dict[str, Any] | None:
    try:
        r = requests.get(f"{BASE}{path}", timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        log.warning("preflight GET %s failed: %s", path, e)
        return None


def _post(path: str, body: dict[str, Any] | None = None) -> dict[str, Any] | None:
    try:
        r = requests.post(f"{BASE}{path}", json=body or {}, timeout=HTTP_TIMEOUT)
        r.raise_for_status()
        return r.json()
    except Exception as e:
        log.warning("preflight POST %s failed: %s", path, e)
        return None


def picker_connected() -> dict[str, Any]:
    state = _get("/api/state")
    if state is None:
        return _check(
            "picker_connected",
            False,
            detail="Picker web controller is unreachable. Start it on the picker Pi.",
            data={"controller_reachable": False, "connected": False},
        )
    ok = bool(state.get("connected"))
    return _check(
        "picker_connected",
        ok,
        detail="" if ok else "Serial port not open — call reconnect_picker.",
        data={"controller_reachable": True, "connected": ok, "port": state.get("port")},
    )


def picker_zeroed() -> dict[str, Any]:
    state = _get("/api/state")
    ok = bool(state and state.get("zeroed"))
    return _check(
        "picker_zeroed",
        ok,
        detail="" if ok else "Align motors to physical zero and press ZERO on the picker UI.",
        data={"zeroed": ok},
    )


def picker_homed() -> dict[str, Any]:
    state = _get("/api/state")
    home_xyz = _get("/api/home_xyz")
    home_angles = _get("/api/delta_home")

    if not state or state.get("moving"):
        return _check(
            "picker_homed",
            False,
            detail="Run full_home.",
            data={"state_available": state is not None, "moving": bool(state and state.get("moving"))},
        )

    positions = state.get("positions") or []
    gantry_mm = state.get("gantry_mm")
    ok = True
    data: dict[str, Any] = {"gantry_mm": gantry_mm}

    if gantry_mm is None or abs(float(gantry_mm)) > HOME_GANTRY_TOL_MM:
        ok = False

    if home_angles and isinstance(positions, list) and len(positions) >= 3:
        for idx, key in enumerate(("d1", "d2", "d3")):
            home_angle = home_angles.get(f"delta_angle_{idx + 1}")
            actual = positions[idx]
            data[key] = {"actual": actual, "home": home_angle}
            if (
                home_angle is None
                or actual is None
                or abs(float(actual) - float(home_angle)) > HOME_ANGLE_TOL_DEG
            ):
                ok = False
    else:
        ok = False

    if home_xyz:
        data["home_xyz"] = home_xyz

    return _check("picker_homed", ok, detail="" if ok else "Run full_home.", data=data)


def camera_running() -> dict[str, Any]:
    status = _get("/api/camera/status")
    running = bool(status and status.get("running"))
    return _check(
        "camera_running",
        running,
        detail="" if running else "Start the stereo camera.",
        data=status or {},
    )


def calibration_loaded() -> dict[str, Any]:
    exists = CALIBRATION_FILE.exists()
    return _check(
        "calibration_loaded",
        exists,
        detail="" if exists else "Run the extrinsic calibration flow and save the transform.",
        data={"path": str(CALIBRATION_FILE), "exists": exists},
    )


def gripper_ok() -> dict[str, Any]:
    # Cycle OPEN → CLOSE → OPEN; every step must report ok.
    steps = []
    for action in ("OPEN", "CLOSE", "OPEN"):
        r = _post("/api/grip", {"action": action})
        steps.append({"action": action, "ok": bool(r and r.get("ok"))})
        if not (r and r.get("ok")):
            return _check(
                "gripper_ok", False, detail="Cycle the gripper.", data={"steps": steps}
            )
    return _check("gripper_ok", True, detail="", data={"steps": steps})


def run_all() -> list[dict[str, Any]]:
    """Run probes in a safe order. Short-circuit downstream probes when an
    upstream dependency fails (no point running VLM if the camera is off)."""
    checks: list[dict[str, Any]] = []

    conn = picker_connected()
    checks.append(conn)
    checks.append(picker_zeroed())

    if conn["ok"]:
        checks.append(picker_homed())
        checks.append(gripper_ok())
    else:
        checks.append(
            _check(
                "picker_homed",
                False,
                detail="Run full_home.",
                data={"skipped": "picker_not_connected"},
            )
        )
        checks.append(
            _check(
                "gripper_ok",
                False,
                detail="Cycle the gripper.",
                data={"skipped": "picker_not_connected"},
            )
        )

    checks.append(camera_running())
    checks.append(calibration_loaded())

    return checks
