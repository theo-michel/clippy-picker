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


_P = "factory"


def machine_state_topic(m: str) -> str:
    return f"{_P}/machines/{m}/state"


def machine_error_topic(m: str) -> str:
    return f"{_P}/machines/{m}/error"


def command_topic(m: str) -> str:
    return f"{_P}/commands/{m}"


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

    def publish_model(self, topic: str, model: BaseModel, *, retain: bool = False) -> None:
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
PICKER_BASE = "http://localhost:8080"


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


def _post(path: str, body: dict | None = None) -> dict:
    return requests.post(f"{PICKER_BASE}{path}", json=body or {}, timeout=60).json()


def do_home() -> bool:
    return _post("/api/full_home").get("ok", False)


def do_scan_and_pick(scan_x: float, description: str) -> str:
    """Run the pick sequence via the SSE test/pick endpoint.

    ``description`` is the natural-language target handed to the VLM.
    Returns 'object_found', 'no_object', or 'fault'.
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
                    return "fault"
    except Exception as e:
        log.error("Pick request failed: %s", e)
    return "fault"


def do_drop() -> bool:
    return _post("/api/grip", {"action": "OPEN"}).get("ok", False)


def do_open_gripper() -> bool:
    return _post("/api/grip", {"action": "OPEN"}).get("ok", False)


# ── Main ────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="Picker factory edge (MQTT bridge)")
    parser.add_argument("--broker", default="localhost", help="MQTT broker host")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    args = parser.parse_args()

    mqtt_client = MqttClient(client_id="picker-edge", host=args.broker, port=args.port)
    task_params: dict = {}
    retries = 0
    max_retries = 3

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

    def on_command(_topic: str, data: dict):
        nonlocal task_params, retries
        try:
            cmd = Command.model_validate(data)
        except Exception:
            log.warning("Invalid command: %s", data)
            return
        if cmd.event == "task_received" and sm.state == S.IDLE:
            task_params = cmd.params
            retries = 0
            threading.Thread(target=run_pick_cycle, daemon=True).start()
        elif cmd.event == "recover" and sm.state == S.ERROR:
            mode = cmd.params.get("mode", "re_home")
            threading.Thread(target=run_recovery, args=(mode,), daemon=True).start()

    def run_pick_cycle():
        nonlocal retries
        scan_x = task_params.get("scan_x", 450.0)
        description = task_params.get("description")
        if not description:
            log.error("task_received missing 'description' param")
            sm.send("fault")
            publish_error("missing_description", "task_received requires a 'description' param")
            return

        try:
            sm.send("task_received")
        except Exception as e:
            log.error("State transition failed: %s", e)
            return

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
            elif outcome == "no_object":
                retries += 1
                if retries >= max_retries:
                    sm.send("no_object")
                else:
                    sm.send("pick_failed")
            elif outcome == "fault":
                sm.send("fault")
                publish_error(
                    "pick_sequence_failed",
                    f"Fault during pick at scan_x={scan_x}, target={description!r}",
                )
                break

    def run_recovery(mode: str):
        if mode == "open_gripper":
            do_open_gripper()
        do_home()
        try:
            sm.send("recover")
        except Exception as e:
            log.error("Recovery transition failed: %s", e)

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
