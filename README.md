# FR8 Delta Robot Controller

A delta robot with a linear gantry axis, controlled by an ESP32 running custom firmware and orchestrated from Python over 1 Mbaud serial.

The system picks objects using a Feetech STS3215, positions them with the 3-axis delta mechanism using Nema 17 42mm steppers, and translates along an 800 mm GT2 belt-driven gantry rail powered by another Nema 17.

## Project Structure

```
delta/
├── firmware/                          ← ESP32 PlatformIO project
├── python/                            ← Host-side control (uv project)
│   ├── delta_robot.py                 ← Serial interface to ESP32
│   ├── delta_kinematics.py            ← Inverse kinematics solver
│   ├── web_controller.py              ← Flask web dashboard
│   ├── scripts/
│   └── templates/                     ← Web UI HTML
```

## Architecture

```
Python (host)                          ESP32 (firmware)
┌──────────────────────┐               ┌──────────────────────┐
│  delta_kinematics.py │  IK: XYZ→θ   │                      │
│  delta_robot.py      │──── serial ──→│  protocol.cpp        │
│  web_controller.py   │  1 Mbaud USB  │  motors.cpp          │
│  scripts/            │←── OK/DONE ───│  gripper.cpp         │
└──────────────────────┘               └──────┬───────────────┘
                                              │
                              ┌───────────────┼───────────────┐
                              │               │               │
                         3× DRV8825      1× DRV8825     Feetech
                         (delta arms)    (gantry)       SCS/STS Bus
                              │               │               │
                         3× NEMA 17      1× NEMA 17     Feetech
                                                        STS3215 12V gripper
```

**Python** handles inverse kinematics, trajectory planning, and orchestration.
**ESP32** handles real-time motor stepping, gripper control, joint-limit enforcement, and motion-complete detection.

## Hardware

| Component    | Spec                                                |
| ------------ | --------------------------------------------------- |
| Delta motors | 3× NEMA 17, 200 steps/rev, 2 A                      |
| Gantry motor | 1× NEMA 17, GT2 belt + 20T pulley                   |
| Drivers      | 4× DRV8825 — microstepping, 2.5 A max               |
| Controller   | ESP32 Dev Module (240 MHz, 320 KB RAM)              |
| Gripper      | 1x Feetech STS3215 via Waveshare Servo Adapter v1.1 |
| Power        | 12 V supply, ≥ 8 A (4 motors × 2 A)                 |

## Wiring

See `firmware/include/config.h`

### Current Limit on DRV8825 drivers

```
Vref = I_max × 0.5
```

For the 17HS19-2004S1 at 2 A: **Vref ≈ 1.0 V**. We set it to a bit lower, around 0.75 V though to reduce the max current.

### Microstepping the Nema17 motors

All parameters are derived in `firmware/include/config.h`:

| Parameter          | Value | Derivation                        |
| ------------------ | ----- | --------------------------------- |
| Delta steps/rev    | -     | 200 × microsteps × 3 (3:1 pulley) |
| Delta steps/degree | 53.33 | delta_steps/rev / 360             |
| Gantry steps/rev   | 6 400 | 200 × microsteps                  |
| Gantry steps/mm    | 160   | 6 400 / 40 mm (20T GT2)           |

## Software

The firmware is an ESP32 [PlatformIO](https://platformio.org/) project using the Arduino framework. The host-side controller is a Python 3.11 `uv` project; the main entry points are `python/web_controller.py` for the Flask UI and `python/delta_robot.py` for direct scripting.

### Setup

```bash
cd firmware
pio run -e esp32 --target upload
```

```bash
cd python
uv sync
```

### Run

```bash
cd python
uv run python web_controller.py
```

The UI is served at `http://localhost:8080`.

For direct scripting:

```python
from delta_robot import DeltaRobot

with DeltaRobot("/dev/tty.usbserial-0001") as robot:
    robot.move_to_position(gantry_x=400, x=50, y=0, z=-200)
    robot.wait_until_done()
```

### Reference

- `python/delta_robot.py`: serial protocol and Python control API
- `python/coordinates.py`: coordinate system, home definition, and motion limits
- `python/homing.py`: homing sequence
- `python/scripts/workspace_analysis.py`: workspace calculations
