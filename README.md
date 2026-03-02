# Delta Robot Controller

Control 3 NEMA 17 stepper motors (17HS19-2004S1) via DRV8825 drivers, coordinated from Python over serial.

## Project Structure

```
Detla/
├── arduino/
│   └── delta_robot_controller/
│       └── delta_robot_controller.ino   ← Upload to Arduino
├── python/
│   ├── delta_robot.py                   ← Python control library + demo
│   └── requirements.txt
└── README.md
```

## Hardware


| Component  | Spec                                                    |
| ---------- | ------------------------------------------------------- |
| Motors     | 3x 17HS19-2004S1 — NEMA 17, 200 steps/rev, 2 A          |
| Drivers    | 3x DRV8825 — up to 1/32 microstepping, 2.5 A max        |
| Controller | Arduino Uno / Mega (any board with enough digital pins) |
| Power      | 12–24 V supply rated for at least 6 A (3 motors x 2 A)  |


## Wiring

### DRV8825 → Arduino Pin Map


| Signal | Motor 1 | Motor 2 | Motor 3 | Notes                          |
| ------ | ------- | ------- | ------- | ------------------------------ |
| STEP   | D2      | D4      | D6      |                                |
| DIR    | D3      | D5      | D7      |                                |
| EN     | D8      | D8      | D8      | All tied together (active LOW) |
| RESET  | —       | —       | —       | Tie to SLEEP (pull HIGH)       |
| SLEEP  | —       | —       | —       | Tie to RESET (pull HIGH)       |


### DRV8825 → Power & Motor

```
VMOT ──── +12–24 V (motor supply)
GND  ──── Supply GND  ───  Arduino GND  (common ground!)
B2, B1 ── Motor coil B
A2, A1 ── Motor coil A
```

> **Important**: Place a 100 µF electrolytic capacitor across VMOT and GND on each DRV8825 board to protect against voltage spikes.

### Current Limit

Set the DRV8825 current limit potentiometer so that:

```
Vref = I_max × 0.5
```

For the 17HS19-2004S1 at 2 A: **Vref ≈ 1.0 V** (measure between the pot wiper and GND with a multimeter while the motor is NOT running).

### Microstepping (M0 / M1 / M2)

Leave all three LOW (or unconnected) for full-step mode. Change the `MICROSTEPS` constant in the Arduino sketch to match if you change these.


| M0  | M1  | M2  | Resolution |
| --- | --- | --- | ---------- |
| L   | L   | L   | Full step  |
| H   | L   | L   | 1/2 step   |
| L   | H   | L   | 1/4 step   |
| H   | H   | L   | 1/8 step   |
| L   | L   | H   | 1/16 step  |
| H   | H   | H   | 1/32 step  |


## Software Setup

### Arduino

1. Install the **AccelStepper** library:
  - Arduino IDE → Sketch → Include Library → Manage Libraries → search "AccelStepper" → Install
2. Open `arduino/delta_robot_controller/delta_robot_controller.ino`
3. Verify pin definitions match your wiring
4. Upload to the board

### Python

```bash
cd python
pip install -r requirements.txt
```

## Usage

### Quick Test

```bash
python python/delta_robot.py /dev/tty.usbmodem14101
```

(Replace with your actual serial port — `COM3` on Windows, `/dev/ttyUSB0` on Linux.)

### In Your Own Code

```python
from delta_robot import DeltaRobot

with DeltaRobot("/dev/tty.usbmodem14101") as robot:
    robot.set_speed(100)            # 100 RPM
    robot.move_to(90, 90, 90)       # All motors to 90°
    robot.wait_until_done()

    robot.move_relative(30, -30, 0) # Relative move
    robot.wait_until_done()

    robot.home()
    robot.wait_until_done()
```

## Serial Command Reference


| Command                  | Description                                  |
| ------------------------ | -------------------------------------------- |
| `M d1 d2 d3`             | Move all motors to absolute degree positions |
| `M1 d` / `M2 d` / `M3 d` | Move single motor to absolute degrees        |
| `R d1 d2 d3`             | Relative move (degrees)                      |
| `SPD rpm`                | Set max speed (RPM)                          |
| `ACC val`                | Set acceleration (RPM/s)                     |
| `HOME`                   | Return all to zero                           |
| `STOP`                   | Decelerate to stop                           |
| `ESTOP`                  | Immediate hard stop                          |
| `ENABLE` / `DISABLE`     | Enable/disable drivers                       |
| `POS`                    | Report positions (steps)                     |
| `STATUS`                 | Report MOVING or IDLE                        |
| `ZERO`                   | Set current position as zero                 |


## Next Steps

- Add limit/homing switches for absolute position reference
- Implement inverse kinematics (XYZ → motor angles) on the Python side
- Add a GUI or joystick input

