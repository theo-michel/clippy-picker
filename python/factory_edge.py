"""Picker factory edge — MQTT bridge + state machine.

Runs alongside web_controller.py on the picker machine. Publishes state
transitions to the MQTT broker and receives commands from the orchestrator.

Self-contained: only depends on paho-mqtt, pydantic, requests.

Usage:
    uv run python factory_edge.py --broker 192.168.1.100
    uv run python factory_edge.py  # defaults to localhost
"""

from __future__ import annotations

import argparse
import json
import logging
import signal
import threading
import time
from collections import deque
from collections.abc import Callable
from datetime import UTC, datetime
from enum import Enum, auto
from pathlib import Path
from typing import Any

import paho.mqtt.client as mqtt
import requests
from dotenv import load_dotenv
from pydantic import BaseModel, Field

load_dotenv(Path(__file__).parent / ".env")

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)
log = logging.getLogger("picker.edge")


# ── Vendored shared code ────────────────────────────────────────────
# TODO: Replace with `from paperclip_shared import ...` once factory/src/shared
# is published as a pip-installable package. Until then, this is inlined so the
# edge file runs standalone on the Pi without access to the factory repo.


def _now() -> datetime:
    return datetime.now(UTC)


class MachineState(BaseModel):
    machine: str
    state: str
    previous_state: str | None = None
    event: str | None = None
    task_complete: bool = False
    timestamp: datetime = Field(default_factory=_now)


class MachineError(BaseModel):
    machine: str
    error: str
    state_at_fault: str
    context: str = ""
    timestamp: datetime = Field(default_factory=_now)


class Command(BaseModel):
    event: str
    params: dict[str, Any] = Field(default_factory=dict)
    timestamp: datetime = Field(default_factory=_now)


class MachineResult(BaseModel):
    machine: str
    request_id: str
    event: str
    ok: bool
    detail: str = ""
    data: dict[str, Any] = Field(default_factory=dict)
    timestamp: datetime = Field(default_factory=_now)


class PreflightCheck(BaseModel):
    name: str
    ok: bool
    detail: str = ""
    data: dict[str, Any] = Field(default_factory=dict)


class PreflightReport(BaseModel):
    machine: str
    request_id: str
    checks: list[PreflightCheck]
    timestamp: datetime = Field(default_factory=_now)


_P = "factory"


def machine_state_topic(m: str) -> str:
    return f"{_P}/machines/{m}/state"


def machine_error_topic(m: str) -> str:
    return f"{_P}/machines/{m}/error"


def command_topic(m: str) -> str:
    return f"{_P}/commands/{m}"


def machine_result_topic(m: str) -> str:
    return f"{_P}/machines/{m}/results"


MessageCallback = Callable[[str, dict[str, Any]], None]


class MqttClient:
    def __init__(
        self, client_id: str, host: str = "localhost", port: int = 1883
    ) -> None:
        self._host, self._port = host, port
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2, client_id=client_id
        )
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._connected = threading.Event()
        self._subs: dict[str, MessageCallback] = {}

    def connect(self) -> None:
        self._connected.clear()
        self._client.connect(self._host, self._port)

    def wait_for_connection(self, timeout: float = 5.0) -> bool:
        return self._connected.wait(timeout=timeout)

    def loop_start(self) -> None:
        self._client.loop_start()

    def loop_stop(self) -> None:
        self._client.loop_stop()

    def disconnect(self) -> None:
        self._client.disconnect()

    def publish_model(
        self, topic: str, model: BaseModel, *, retain: bool = False
    ) -> None:
        self._client.publish(topic, model.model_dump_json(), retain=retain)

    def subscribe(self, topic: str, callback: MessageCallback) -> None:
        self._subs[topic] = callback
        if self._client.is_connected():
            self._client.subscribe(topic)

    def _on_connect(self, *args: Any, **kw: Any) -> None:
        rc = args[3] if len(args) > 3 else kw.get("rc", 0)
        if hasattr(rc, "value"):
            rc = rc.value
        if rc == 0:
            self._connected.set()
            for t in self._subs:
                self._client.subscribe(t)

    def _on_message(self, _c: Any, _u: Any, msg: mqtt.MQTTMessage) -> None:
        try:
            data = json.loads(msg.payload.decode())
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        for t, cb in self._subs.items():
            if mqtt.topic_matches_sub(t, msg.topic):
                cb(msg.topic, data)

    def _on_disconnect(self, *_a: Any, **_kw: Any) -> None:
        self._connected.clear()


class InvalidTransition(Exception):
    pass


class StateMachine:
    def __init__(
        self,
        states: type[Enum],
        initial: Enum,
        transitions: dict,
        on_transition: Callable[[Enum, str, Enum], Any] | None = None,
    ) -> None:
        self._state = initial
        self._transitions = transitions
        self.on_transition = on_transition

    @property
    def state(self) -> Enum:
        return self._state

    def send(self, event: str) -> Enum:
        allowed = self._transitions.get(self._state, {})
        if event not in allowed:
            raise InvalidTransition(
                f"No '{event}' from {self._state.name}. Allowed: {list(allowed)}"
            )
        old = self._state
        self._state = allowed[event]
        log.info("%s --%s--> %s", old.name, event, self._state.name)
        if self.on_transition:
            self.on_transition(old, event, self._state)
        return self._state


# ── Picker states & transitions ─────────────────────────────────────

MACHINE_NAME = "picker"
import os as _os

PICKER_BASE = _os.environ.get(
    "PICKER_BASE", f"http://localhost:{_os.environ.get('PICKER_PORT', '8080')}"
)


class S(Enum):
    IDLE = auto()
    SCANNING = auto()
    HOLDING = auto()
    ERROR = auto()


TRANSITIONS = {
    S.IDLE: {"task_received": S.SCANNING},
    S.SCANNING: {
        "object_found": S.HOLDING,
        "pick_failed": S.SCANNING,
        "no_object": S.IDLE,
        "fault": S.ERROR,
    },
    S.HOLDING: {
        "drop_complete": S.IDLE,
        "fault": S.ERROR,
    },
    S.ERROR: {"recover": S.IDLE},
}


# ── REST calls to web_controller.py ─────────────────────────────────
# The edge is the *only* thing that talks to web_controller from outside
# the picker Pi. All factory-issued actions flow through here.


def _post(path: str, body: dict | None = None, timeout: float = 60.0) -> dict:
    try:
        r = requests.post(f"{PICKER_BASE}{path}", json=body or {}, timeout=timeout)
        r.raise_for_status()
        return r.json() if r.content else {}
    except Exception as e:
        log.warning("POST %s failed: %s", path, e)
        return {"ok": False, "error": str(e)}


def _get(path: str, timeout: float = 5.0) -> dict:
    try:
        r = requests.get(f"{PICKER_BASE}{path}", timeout=timeout)
        r.raise_for_status()
        return r.json() if r.content else {}
    except Exception as e:
        log.warning("GET %s failed: %s", path, e)
        return {"ok": False, "error": str(e)}


def do_home() -> bool:
    """Full homing; may take 60-90 s. Poll /api/state.moving afterwards as a
    guard against returning before motion actually stops."""
    result = _post("/api/full_home", timeout=120.0)
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        state = _get("/api/state", timeout=2.0)
        if state and not state.get("moving"):
            break
        time.sleep(0.5)
    return bool(result.get("ok"))


def do_scan_and_pick(scan_x: float, description: str) -> str:
    """Run the pick sequence via the SSE /api/pick endpoint.

    ``description`` is the natural-language target handed to the VLM.
    Returns 'object_found', 'no_object', 'cancelled', or 'fault'.
    """
    try:
        with requests.get(
            f"{PICKER_BASE}/api/pick",
            params={"gantry_x": scan_x, "description": description},
            stream=True,
            timeout=120,
        ) as resp:
            last_step = ""
            for line in resp.iter_lines(decode_unicode=True):
                if not line or not line.startswith("data: "):
                    continue
                data = json.loads(line[6:])
                msg_type = data.get("type", "")
                if msg_type == "step":
                    last_step = data.get("step", "")
                    log.info("Pick step: %s — %s", last_step, data.get("detail", ""))
                elif msg_type == "error":
                    log.error("Pick error: %s", data.get("message", ""))
                    return "no_object" if last_step == "Scan" else "fault"
                elif msg_type == "done":
                    return "object_found"
                elif msg_type == "cancelled":
                    return "cancelled"
    except Exception as e:
        log.error("Pick request failed: %s", e)
    return "fault"


def do_drop() -> bool:
    return bool(_post("/api/grip", {"action": "OPEN"}).get("ok"))


def do_gripper(action: str) -> dict:
    return _post("/api/grip", {"action": action}, timeout=8.0)


def do_cycle_gripper() -> tuple[bool, list[dict]]:
    steps: list[dict] = []
    for action in ("OPEN", "CLOSE", "OPEN"):
        r = do_gripper(action)
        steps.append({"action": action, "ok": bool(r.get("ok"))})
        if not r.get("ok"):
            return False, steps
    return True, steps


def do_start_camera() -> dict:
    return _post("/api/camera/start", timeout=10.0)


def do_reconnect() -> dict:
    """Auto-discover a USB serial port and open it.

    Prefers devices whose name contains 'usbserial' or 'ttyUSB'. Returns
    a dict with ok + detail; a missing port is flagged so the factory
    knows it's a human-fixable condition, not a transient failure.
    """
    listing = _get("/api/ports", timeout=5.0)
    ports = listing.get("response") or listing.get("ports") or []
    # /api/ports returns a list of dicts with 'device' keys (or a list of strings).
    port = None
    for p in ports:
        dev = p.get("device") if isinstance(p, dict) else str(p)
        if dev and ("usbserial" in dev or "ttyUSB" in dev):
            port = dev
            break
    if port is None:
        return {
            "ok": False,
            "error": "No USB serial port detected — plug the picker in.",
            "ports": [p.get("device") if isinstance(p, dict) else p for p in ports],
        }
    return _post("/api/connect", {"port": port}, timeout=10.0)


def do_look_camera() -> dict:
    """Fetch a single JPEG snapshot from the picker's stereo camera and
    return it base64-encoded. The agent receives this as an actual image
    in its tool-result, so Claude can visually inspect the scene."""
    import base64

    try:
        r = requests.get(f"{PICKER_BASE}/api/camera/snapshot", timeout=10.0)
    except Exception as e:
        return {"ok": False, "error": f"camera snapshot failed: {e}"}
    if r.status_code != 200 or not r.headers.get("Content-Type", "").startswith("image/"):
        msg = r.json().get("message") if r.content else f"HTTP {r.status_code}"
        return {"ok": False, "error": msg or "camera snapshot unavailable"}
    return {"ok": True, "_image_b64": base64.b64encode(r.content).decode("ascii")}


def do_cancel_pick() -> bool:
    return bool(_post("/api/pick/cancel", timeout=5.0).get("ok"))


# ── Main ────────────────────────────────────────────────────────────


HandlerResult = tuple[bool, str, dict[str, Any]]
Handler = Callable[[dict[str, Any]], HandlerResult]

MAX_PICK_RETRIES = 3
DEFAULT_SCAN_X = 500.0
RECENT_REQUEST_IDS_MAX = 64


def main():
    parser = argparse.ArgumentParser(description="Picker factory edge (MQTT bridge)")
    parser.add_argument("--broker", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    args = parser.parse_args()

    mqtt_client = MqttClient(client_id="picker-edge", host=args.broker, port=args.port)

    def publish_state(old: Enum, event: str, new: Enum):
        mqtt_client.publish_model(
            machine_state_topic(MACHINE_NAME),
            MachineState(
                machine=MACHINE_NAME,
                state=new.name,
                previous_state=old.name,
                event=event,
                task_complete=(event == "drop_complete"),
            ),
            retain=True,
        )

    sm = StateMachine(S, S.IDLE, TRANSITIONS, on_transition=publish_state)

    def publish_error(error: str, context: str = ""):
        mqtt_client.publish_model(
            machine_error_topic(MACHINE_NAME),
            MachineError(
                machine=MACHINE_NAME,
                error=error,
                state_at_fault=sm.state.name,
                context=context,
            ),
        )

    # ── Handlers ─────────────────────────────────────────────────────
    # Each handler takes cmd params and returns (ok, detail, data). The
    # dispatcher wraps the return value in a MachineResult if the command
    # carried a request_id.

    def _require_idle(action: str) -> HandlerResult | None:
        if sm.state != S.IDLE:
            return (
                False,
                f"picker is {sm.state.name}, not IDLE — refusing {action}",
                {"state": sm.state.name},
            )
        return None

    def h_task_received(params: dict) -> HandlerResult:
        description = params.get("description")
        if not description:
            return False, "task_received requires a 'description' param", {}
        if sm.state != S.IDLE:
            return (
                False,
                f"picker is {sm.state.name}, cannot accept task",
                {"state": sm.state.name},
            )
        scan_x = float(params.get("scan_x", DEFAULT_SCAN_X))
        threading.Thread(
            target=run_pick_cycle, args=(scan_x, str(description)), daemon=True
        ).start()
        return True, "task accepted", {"scan_x": scan_x, "description": description}

    def h_recover(params: dict) -> HandlerResult:
        if sm.state != S.ERROR:
            return (
                False,
                f"picker is {sm.state.name}, not ERROR — ignoring recover",
                {"state": sm.state.name},
            )
        mode = str(params.get("mode", "re_home"))
        threading.Thread(target=run_recovery, args=(mode,), daemon=True).start()
        return True, f"recovery started (mode={mode})", {"mode": mode}

    def h_preflight(params: dict) -> HandlerResult:
        report = _run_preflight_sync()
        log.info(
            "Preflight: %s",
            ", ".join(f"{c.name}={'ok' if c.ok else 'fail'}" for c in report.checks),
        )
        for c in report.checks:
            if not c.ok:
                log.info("  %s FAIL: %s", c.name, c.detail or "(no detail)")
        return (
            all(c.ok for c in report.checks),
            "preflight complete",
            report.model_dump(mode="json"),
        )

    def h_home(_params: dict) -> HandlerResult:
        gate = _require_idle("home")
        if gate is not None:
            return gate
        ok = do_home()
        return ok, "home complete" if ok else "home failed", {}

    def h_cycle_gripper(_params: dict) -> HandlerResult:
        gate = _require_idle("cycle_gripper")
        if gate is not None:
            return gate
        ok, steps = do_cycle_gripper()
        return ok, "gripper cycled" if ok else "gripper cycle failed", {"steps": steps}

    def h_open_gripper(_params: dict) -> HandlerResult:
        # Safety release: must work from any state, including HOLDING / ERROR.
        r = do_gripper("OPEN")
        ok = bool(r.get("ok"))
        return ok, "gripper opened" if ok else r.get("error", "open failed"), r

    def h_close_gripper(_params: dict) -> HandlerResult:
        gate = _require_idle("close_gripper")
        if gate is not None:
            return gate
        r = do_gripper("CLOSE")
        ok = bool(r.get("ok"))
        return ok, "gripper closed" if ok else r.get("error", "close failed"), r

    def h_start_camera(_params: dict) -> HandlerResult:
        r = do_start_camera()
        ok = bool(r.get("ok"))
        return ok, "camera started" if ok else r.get("error", "start failed"), r

    def h_reconnect(_params: dict) -> HandlerResult:
        r = do_reconnect()
        ok = bool(r.get("ok"))
        return ok, "serial reconnected" if ok else r.get("error", "reconnect failed"), r

    def h_look_camera(_params: dict) -> HandlerResult:
        r = do_look_camera()
        return (
            bool(r.get("ok")),
            "camera snapshot" if r.get("ok") else r.get("error", "snapshot failed"),
            r,
        )

    def h_cancel_task(_params: dict) -> HandlerResult:
        if sm.state not in (S.SCANNING, S.HOLDING):
            return (
                True,
                f"nothing to cancel (state={sm.state.name})",
                {"state": sm.state.name},
            )
        ok = do_cancel_pick()
        return ok, "cancel signalled" if ok else "cancel failed", {}

    def h_pause(_params: dict) -> HandlerResult:
        """Stop-and-idle for the picker: cancel any in-flight pick, release
        the gripper, and nudge the state machine back to IDLE. Always
        reaches a safe state; ok=False only on hardware faults."""
        if sm.state in (S.SCANNING, S.HOLDING):
            do_cancel_pick()
        release = do_gripper("OPEN")
        try:
            if sm.state == S.ERROR:
                sm.send("recover")
            elif sm.state == S.HOLDING:
                sm.send("drop_complete")
            elif sm.state == S.SCANNING:
                # SCANNING → IDLE goes via no_object. Piggy-back so the state
                # machine stays consistent with the published state.
                sm.send("no_object")
        except Exception as e:
            log.warning("pause transition failed: %s", e)
        return (
            bool(release.get("ok")),
            "paused",
            {"final_state": sm.state.name},
        )

    def h_resume(_params: dict) -> HandlerResult:
        # Picker has no queued-work concept; resume is a declarative no-op
        # that keeps the verb set symmetric across machines.
        return (
            True,
            f"nothing to resume (state={sm.state.name})",
            {"state": sm.state.name},
        )

    HANDLERS: dict[str, Handler] = {
        "task_received": h_task_received,
        "recover": h_recover,
        "preflight": h_preflight,
        "home": h_home,
        "cycle_gripper": h_cycle_gripper,
        "open_gripper": h_open_gripper,
        "close_gripper": h_close_gripper,
        "start_camera": h_start_camera,
        "reconnect": h_reconnect,
        "look_camera": h_look_camera,
        "cancel_task": h_cancel_task,
        "pause": h_pause,
        "resume": h_resume,
    }

    recent_req_ids: deque[str] = deque(maxlen=RECENT_REQUEST_IDS_MAX)

    def publish_result(
        req_id: str, event: str, ok: bool, detail: str, data: dict
    ) -> None:
        if not req_id:
            return
        mqtt_client.publish_model(
            machine_result_topic(MACHINE_NAME),
            MachineResult(
                machine=MACHINE_NAME,
                request_id=req_id,
                event=event,
                ok=ok,
                detail=detail,
                data=data,
            ),
        )

    def on_command(_topic: str, data: dict) -> None:
        try:
            cmd = Command.model_validate(data)
        except Exception:
            log.warning("Invalid command: %s", data)
            return
        handler = HANDLERS.get(cmd.event)
        if handler is None:
            log.warning("Unknown command event: %s", cmd.event)
            return
        req_id = str(cmd.params.get("request_id") or "")
        if req_id and req_id in recent_req_ids:
            log.info("Dropping duplicate request_id=%s", req_id)
            return
        if req_id:
            recent_req_ids.append(req_id)

        def run() -> None:
            try:
                ok, detail, payload = handler(cmd.params)
            except Exception as e:
                log.exception("Handler %s crashed", cmd.event)
                ok, detail, payload = False, f"handler crashed: {e}", {}
            publish_result(req_id, cmd.event, ok, detail, payload)

        threading.Thread(target=run, daemon=True).start()

    # ── Long-running workers ─────────────────────────────────────────

    def run_pick_cycle(scan_x: float, description: str) -> None:
        try:
            sm.send("task_received")
        except Exception as e:
            log.error("State transition failed: %s", e)
            return

        retries = 0
        while sm.state == S.SCANNING:
            outcome = do_scan_and_pick(scan_x, description)
            if outcome == "object_found":
                sm.send("object_found")
                if do_drop():
                    sm.send("drop_complete")
                else:
                    sm.send("fault")
                    publish_error("drop_failed", "Gripper release failed")
                break
            if outcome == "cancelled":
                try:
                    sm.send("no_object")
                except Exception:
                    pass
                break
            if outcome == "no_object":
                retries += 1
                if retries >= MAX_PICK_RETRIES:
                    sm.send("no_object")
                else:
                    sm.send("pick_failed")
                continue
            # outcome == "fault"
            sm.send("fault")
            publish_error(
                "pick_sequence_failed",
                f"Fault during pick at scan_x={scan_x}, target={description!r}",
            )
            break

    def run_recovery(mode: str) -> None:
        if mode in ("open_gripper", "stop_and_idle"):
            do_gripper("OPEN")
        if mode != "stop_and_idle":
            do_home()
        try:
            sm.send("recover")
        except Exception as e:
            log.error("Recovery transition failed: %s", e)

    def _run_preflight_sync() -> PreflightReport:
        # Import lazily so the edge still imports if preflight.py is missing.
        from preflight import run_all

        # Gripper cycle is the only preflight probe with side effects; guard it
        # behind IDLE so we never poke the gripper mid-job.
        if sm.state == S.IDLE:
            raw_checks = run_all()
        else:
            raw_checks = [
                {
                    "name": "preflight_skipped",
                    "ok": False,
                    "detail": f"Picker is {sm.state.name}, not IDLE — cannot run preflight now.",
                    "data": {"state": sm.state.name},
                }
            ]
        return PreflightReport(
            machine=MACHINE_NAME,
            request_id="",  # request_id travels on the MachineResult envelope
            checks=[PreflightCheck.model_validate(c) for c in raw_checks],
        )

    # ── Main loop ────────────────────────────────────────────────────

    mqtt_client.connect()
    mqtt_client.loop_start()
    if not mqtt_client.wait_for_connection(timeout=10):
        log.error("Could not connect to MQTT broker at %s:%d", args.broker, args.port)
        return

    mqtt_client.subscribe(command_topic(MACHINE_NAME), on_command)
    mqtt_client.publish_model(
        machine_state_topic(MACHINE_NAME),
        MachineState(machine=MACHINE_NAME, state=sm.state.name),
        retain=True,
    )
    log.info("Picker edge running. State: %s. Waiting for commands...", sm.state.name)

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stop.set())

    try:
        stop.wait()
        log.info("Shutting down.")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()


if __name__ == "__main__":
    main()
