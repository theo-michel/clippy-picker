# Picker — How It Works

This document explains the four core subsystems of the picker: stereo camera calibration (intrinsic), camera-to-robot registration (extrinsic), homing, and the vision-guided pick sequence.

Every section references the actual source files so you can follow along.

---

## Table of contents

1. [Coordinate frames](#1-coordinate-frames)
2. [Intrinsic calibration (stereo camera)](#2-intrinsic-calibration)
3. [Extrinsic calibration (camera → delta)](#3-extrinsic-calibration)
4. [Homing](#4-homing)
5. [The pick sequence (YOLO + stereo + IK)](#5-the-pick-sequence)

---

## 1. Coordinate frames

Before anything else, you need to understand the three coordinate frames that the picker converts between. Getting any sign or axis wrong here breaks everything downstream.

### Camera frame (OpenCV convention)

```
     ┌──────────────┐
     │   Rectified   │
     │   left image  │
     │               │
     │    (u, v)     │
     └──────────────┘

  +X → right  (u increases)
  +Y → down   (v increases)
  +Z → forward (into the scene, away from the lens)
```

All pixel operations (detection, disparity, deprojection) happen in this frame. The origin is at the _rectified_ left camera's projection centre (the `Q` matrix and `P1` from `stereoRectify` define this frame). We got this value using OpenCV's stereoRectify method.

### Delta frame

Defined in `python/coordinates.py`:

```
  Origin: centre of the delta base plate.
  +X → toward arm 1 (motor 1), same direction as gantry travel.
  +Z → downward (workspace is at positive Z).
  +Y → completes the right-hand rule.
```

The inverse/forward kinematics in `python/delta_kinematics.py` work entirely in this frame. All positions are in millimetres.

### World frame

The world frame combines the gantry's linear axis with the delta frame:

```
  world_x  =  gantry_x  +  delta_x
  world_y  =  delta_y
  world_z  =  delta_z
```

The gantry only moves along X. Its origin (`gantry_x = 0`) is at the endstop. The delta frame rides on the gantry carriage, so the delta origin is always at `(gantry_x, 0, 0)` in world coordinates.

---

## 2. Intrinsic calibration

**Goal:** Correct lens distortion and compute the geometry needed for stereo depth.

**Source files:**

- `python/calibration/intrinsic/calibrate_stereo.py` — calibration script
- `python/calibration/intrinsic/stereo_calibration.npz` — saved result
- `python/scripts/test_stereo.py` — frame capture tool

### 2.1 What we're solving for

A stereo camera is two cameras with a known horizontal separation (the _baseline_). Each camera has internal properties:

- **K** (3×3 intrinsic matrix): focal length `(fx, fy)` and principal point `(cx, cy)` in pixels.
- **dist** (distortion coefficients): radial and tangential lens distortion.

Between the cameras there's an extrinsic relationship:

- **R** (3×3 rotation): relative orientation of the right camera w.r.t. the left.
- **T** (3×1 translation): position of the right camera in the left camera's frame. The magnitude `‖T‖` is the baseline.

### 2.2 The calibration board

The script uses a **ChArUco board** (a checkerboard with ArUco markers embedded in the white squares). The board specification in `calibrate_stereo.py`:

```
Squares:     17 × 9
Checker size: 16 mm  (passed as 0.016 m — this sets the unit for ALL outputs)
Marker size:  12 mm
Dictionary:   DICT_4X4_50
```

> **Critical detail:** because the board size is passed in _metres_ (`CHECKER_SIZE_MM / 1000.0`), every output from OpenCV — `K`, `T`, `Q` — is in metres, not millimetres. This affects the stereo depth formula later.

### 2.3 Calibration procedure

1. **Capture stereo pairs.** Use `test_stereo.py` — press `c` to save `left_NNN.png` / `right_NNN.png` pairs. Capture 15–30 pairs with the board at varying angles and distances.

2. **Detect ChArUco corners** in each image (left and right independently). The detector finds ArUco markers first, then interpolates the checkerboard corners between them.

3. **Single-camera calibration** (`cv2.calibrateCamera`). Each side is calibrated separately to get `K_left`, `dist_left`, `K_right`, `dist_right`.

4. **Stereo calibration** (`cv2.stereoCalibrate`). Using matched corner pairs visible in both cameras simultaneously, OpenCV solves for `R` and `T` with the intrinsics fixed (`CALIB_FIX_INTRINSIC`).

5. **Stereo rectification** (`cv2.stereoRectify`). This computes:
   - `R1, R2` — rotation matrices that make both cameras' image planes coplanar.
   - `P1, P2` — projection matrices for the rectified images.
   - **Q** (4×4 disparity-to-depth matrix) — used to convert pixel + disparity to 3D.
   - Rectification maps (`map_left_1/2`, `map_right_1/2`) for `cv2.remap`.

6. **Save** everything to `stereo_calibration.npz`.

### 2.4 The Q matrix

Q is the key output for 3D reconstruction. Its structure:

```
Q = | 1   0   0   -cx  |
    | 0   1   0   -cy  |
    | 0   0   0    f   |
    | 0   0  -1/Tx  0  |
```

Where:

- `cx, cy` = principal point of the rectified left camera (pixels)
- `f` = focal length of the rectified left camera (pixels)
- `Tx` = baseline (metres, because we calibrated in metres)

Given a pixel `(u, v)` with disparity `d`, the 3D point is:

```
[X, Y, Z, W]^T = Q · [u, v, d, 1]^T

X_3d = (u - cx) / (d · (-1/Tx))  =  (u - cx) · Tx / d
Y_3d = (v - cy) / (d · (-1/Tx))  =  (v - cy) · Tx / d
Z_3d =  f       / (d · (-1/Tx))  =   f · Tx / d
```

These coordinates are in **metres** (same unit as `Tx`).

### 2.5 Stereo matching at runtime

In `python/camera.py` (`StereoCamera._loop`):

1. Grab a 2560×720 side-by-side frame. Split into left (1280×720) and right (1280×720).
2. Rectify both images with the precomputed maps (`cv2.remap`).
3. Downscale by `stereo_scale` (default 0.5 → 640×360) for performance.
4. Convert to greyscale, run `StereoSGBM` to get a disparity map.

The disparity map has the same resolution as the downscaled images. A higher `numDisparities` (default 192) allows detecting closer objects but costs more compute.

### 2.6 Deprojection: pixel → 3D (`StereoCamera.deproject`)

```python
# python/camera.py, deproject() method

Z_mm = focal * baseline / disp * 1000.0  # focal·baseline is in px·m → Z in m → ×1000 = mm
X_mm = (u + Q[0,3]) / abs(Q[2,3]) * Z_mm
Y_mm = (v + Q[1,3]) / abs(Q[2,3]) * Z_mm
```

Breaking this down:

- `focal = abs(Q[2,3]) * stereo_scale` — focal length at the _reduced_ resolution.
- `baseline = abs(1.0 / Q[3,2])` — baseline in metres.
- `disp` — disparity at reduced resolution.
- `Z_mm = (f_scaled · B_metres / d_scaled) × 1000`. Because `f_scaled / d_scaled = f_full / d_full` (both scale the same way), depth is resolution-independent.

For X and Y, note that `Q[0,3] = -cx` and `Q[1,3] = -cy` (the Q matrix stores the _negative_ principal point). So `u + Q[0,3] = u - cx`, which is the standard pinhole formula. The pixel coordinate `u` is at _full_ resolution (the caller passes the original rectified-image coordinate), and `abs(Q[2,3])` is the full-resolution focal length, so the ratio is correct.

The `patch` parameter averages disparity over a small neighbourhood (default 5 pixels at scaled resolution) to reduce noise from stereo matching artefacts.

---

## 3. Extrinsic calibration

**Goal:** Find the rigid-body transform (rotation **R**, translation **t**) that maps a 3D point in the _camera_ frame to the _delta robot_ frame.

**Source files:**

- `python/calibration/extrinsic/kabsch.py` — SVD-based Kabsch solver
- `python/calibration/extrinsic/camera_transform.json` — saved R, t
- `python/coordinates.py` — `camera_to_robot()` function
- `python/web_controller.py` — calibration capture/solve/save endpoints

### 3.1 Collecting matched point pairs

For each calibration capture, we obtain a _camera point_ and a _delta point_ for the same physical location (an ArUco marker taped to the end-effector platform):

**Camera point** `c_i`:

1. Camera detects the ArUco marker in the rectified left image.
2. The marker's centre pixel `(u, v)` is deprojected to 3D via `StereoCamera.deproject()`.
3. This gives `c_i = (X_cam, Y_cam, Z_cam)` in the camera frame (mm).

**Delta point** `d_i`:

1. Read the current joint angles `(θ1, θ2, θ3)` from the firmware (converted to kinematic space).
2. Forward kinematics (`DeltaKinematics.forward()`) gives the end-effector position `(x_ee, y_ee, z_ee)` in the delta frame.
3. Add the marker offset: `d_i = FK_pos + MARKER_OFFSET_FROM_EE`.

```python
# python/coordinates.py
MARKER_OFFSET_FROM_EE = (0.0, 0.0, -5.0)  # mm — marker sits 5mm above EE origin (z-up = negative in z-down frame)
```

> **Why −5 mm in Z?** The EE platform is ~10 mm thick. Its frame origin is at the centre (5 mm from the top surface). A marker taped on top sits 5 mm _above_ the origin. In the z-down convention, "above" = negative Z, hence `dz = −5`.

### 3.2 The Kabsch algorithm

Given N matched pairs `{(c_i, d_i)}`, find R and t that minimise:

```
Σᵢ ‖d_i − (R · c_i + t)‖²
```

This is a classic rigid-body registration problem. The Kabsch algorithm solves it via SVD:

**Step 1 — Centre both point clouds:**

```
c̄ = mean(c_i)       d̄ = mean(d_i)
Cᶜ = c_i − c̄        Dᶜ = d_i − d̄
```

**Step 2 — Cross-covariance matrix:**

```
H = (Cᶜ)ᵀ · Dᶜ     (3×3 matrix)
```

**Step 3 — SVD:**

```
H = U · Σ · Vᵀ
```

**Step 4 — Rotation (ensuring det(R) = +1 for a proper rotation, not a reflection):**

```
d = sign(det(Vᵀ · Uᵀ))
R = Vᵀ · diag(1, 1, d) · Uᵀ
```

**Step 5 — Translation:**

```
t = d̄ − R · c̄
```

### 3.3 Outlier rejection

`kabsch.py` wraps the core solver in an iterative loop (up to 5 iterations):

1. Solve Kabsch on current inlier set.
2. Compute per-point residual `rᵢ = ‖dᵢ − (R·cᵢ + t)‖`.
3. Reject points where `rᵢ > 1.5 × RMSD`.
4. Re-solve on the remaining inliers. Stop when nothing is rejected.
5. Return R, t, RMSD (inliers only), and _full-set_ residuals (so you can see how every original point performed).

### 3.4 Applying the transform at runtime

```python
# python/coordinates.py
def camera_to_robot(c_point: np.ndarray) -> np.ndarray:
    R, t = load_camera_transform()
    return R @ c_point + t
```

This single matrix multiply + addition takes a camera-frame 3D point and returns a delta-frame 3D point, both in mm.

### 3.5 What good calibration looks like

- **RMSD < 5 mm** — the transform is accurate enough for reliable pick-and-place.
- **RMSD 5–15 mm** — usable but may need multiple pick attempts.
- **RMSD > 15 mm** — expect frequent misses. Likely a systematic error (e.g. the sign bug above) or very noisy stereo depth.

The R matrix should reflect the physical camera mounting. For a downward-looking camera rotated ~180° about its optical axis relative to the delta frame, you'd expect roughly:

```
R ≈ | −1   0   0 |     (camera +X ≈ delta −X)
    |  0  −1   0 |     (camera +Y ≈ delta −Y)
    |  0   0  +1 |     (camera +Z ≈ delta +Z, both point down)
```

with small off-diagonal terms accounting for any tilt.

---

## 4. Homing

**Goal:** Establish a known, repeatable reference position for every axis so that all subsequent motion commands are meaningful.

**Source files:**

- `python/homing.py` — homing sequence
- `python/coordinates.py` — home positions, angle conventions
- `python/delta_robot.py` — firmware angle conversion
- `python/templates/_homing_modal.html` — UI wizard

### 4.1 The three subsystems

The picker has three independent motion systems, each homed differently:

| Subsystem  | Home method                             | Home position                   |
| ---------- | --------------------------------------- | ------------------------------- |
| Gantry     | Endstop switch                          | `gantry_x = 0 mm`               |
| Delta arms | Manual push to mechanical stop + `ZERO` | `θ = −15°` kinematic (each arm) |
| Gripper    | Move to open position                   | Servo position 2900             |

### 4.2 Gantry homing

The gantry is a linear rail driven by a stepper motor. The firmware command `GANTRY_HOME` drives the carriage toward the endstop until the switch triggers, then sets position to 0 and backs off slightly. After homing, valid positions are `[0, 625]` mm.

### 4.3 Delta homing — the angle convention

This is the trickiest part. There are **two** angle conventions in play:

**Kinematic angles** (`delta_kinematics.py`):

```
θ = 0°    → upper arm horizontal
θ > 0°    → arm tilted downward (toward the workspace)
θ < 0°    → arm tilted upward
```

**Firmware angles** (ESP32):

```
θ_fw = 0°  → mechanical stop (arms at highest position, 21.8° above horizontal)
θ_fw > 0°  → arm swings down from there
```

The conversion:

```
θ_kinematic = θ_firmware + DELTA_KINEMATIC_AT_FIRMWARE_ZERO
            = θ_firmware + (−21.8°)
            = θ_firmware − 21.8°
```

Or equivalently: `θ_firmware = θ_kinematic + 21.8°`

**The homing wizard steps:**

1. **Manually push** all three arms to the mechanical stop (highest position). This is `θ_kinematic = −21.8°`.
2. Send `ZERO` to the firmware — this declares the current position as `θ_fw = 0°`.
3. Command the arms to the **home (park) position**: `θ_kinematic = −15°`, which is `θ_fw = 6.8°`. This is 15° above horizontal — a safe park position clear of the workspace.

```python
# python/coordinates.py
DELTA_KINEMATIC_AT_FIRMWARE_ZERO = -21.8   # mechanical stop
DELTA_HOME_ANGLE_1 = -15.0                 # park position (kinematic)
DELTA_HOME_ANGLE_2 = -15.0
DELTA_HOME_ANGLE_3 = -15.0
```

The full usable range after homing:

```
Kinematic:  −21.8°  to  +80.0°   (total 101.8° travel)
Firmware:      0°   to  101.8°
```

### 4.4 Homing sequence order

From `homing.py`, the order is deliberate:

1. **Delta first** — retract the arms upward so they don't collide with anything during gantry travel.
2. **Gantry second** — drive to the endstop.
3. **Gripper last** — open the gripper.

---

## 5. The pick sequence

**Goal:** Detect an object with the camera, compute where it is in the robot's frame, move the gripper there, grab it, and return home.

**Source files:**

- `python/pick.py` — pure pick task (no Flask dependencies)
- `python/web_controller.py` — `/api/pick` SSE endpoint (thin wrapper)
- `python/object_detection.py` — YOLO wrapper
- `python/camera.py` — stereo depth + deprojection
- `python/coordinates.py` — `camera_to_robot()`, TCP offset
- `python/delta_robot.py` — `move_tcp()`, IK

### 5.1 The eight steps

```
[1/8] Prepare     Open gripper, delta → home angles
[2/8] Scan        Move gantry to scan position, detect objects, deproject to 3D
[3/8] Approach    Move gantry above object, delta to hover height
[4/8] Descend     Lower TCP straight down to pick depth
[5/8] Grab        Close gripper
[6/8] Lift        Raise TCP back to hover height
[7/8] Return      Delta → home, gantry → 0
[8/8] Release     Open gripper
```

### 5.2 Step 2 in detail: scan → 3D → robot frame → gantry/delta split

This is where all the calibration comes together. Here's the full chain:

**a) YOLO detection**

```python
# python/object_detection.py
dets = detector.detect(frame, conf=0.4)
best = max(dets, key=lambda d: d.confidence)
u, v = int(round(best.center_uv[0])), int(round(best.center_uv[1]))
```

The YOLO model (`runs/rpi_case/weights/best.pt`) runs inference on the rectified left camera frame. It returns bounding boxes with class labels and confidence scores. We take the highest-confidence detection and use the centre of its bounding box `(u, v)` as the pick target.

**b) Stereo deprojection: pixel → camera 3D**

```python
c_point = camera.deproject(u, v, patch=5)
# Returns (X_cam, Y_cam, Z_cam) in mm, camera frame
```

This uses the disparity map from the stereo matcher (see Section 2.6). The `patch=5` parameter takes the median disparity in a 5-pixel neighbourhood to reduce noise.

**c) Extrinsic transform: camera 3D → delta 3D**

```python
d_pt = camera_to_robot(np.array(c_point))
dx, dy, dz = float(d_pt[0]), float(d_pt[1]), float(d_pt[2])
```

Applies `d = R · c + t` (see Section 3.4). The result `(dx, dy, dz)` is the object's position in the **delta frame** (mm), i.e. relative to the delta base plate centre.

**d) World-frame conversion and gantry/delta split**

```python
world_x = scan_gx + dx
gantry_pick = clamp(world_x, GANTRY_X_MIN, GANTRY_X_MAX)
pick_dx = world_x - gantry_pick
pick_dy = dy
pick_dz = dz
```

Because the delta frame rides on the gantry, the object's world X = gantry position + delta X. We then split this into:

- `gantry_pick` — where to move the gantry (clamped to `[0, 625]` mm).
- `pick_dx` — the residual X that the delta must cover (usually ≈0 unless gantry hit a limit).

Y and Z pass through directly because the gantry only moves along X.

### 5.3 The TCP offset — why you don't command the gripper tip directly

The inverse kinematics solves for the **end-effector (EE)** position — the centre of the moving platform. But the gripper tip (the **Tool Centre Point**, or TCP) is 98 mm below the EE:

```python
# python/coordinates.py
TCP_OFFSET_FROM_EE = (0.0, 0.0, 98.0)   # dx, dy, dz in mm
```

When you call `robot.move_tcp(x, y, z)`, the robot internally subtracts this offset to find the required EE position:

```python
# python/delta_robot.py
def _tcp_to_ee(x, y, z):
    return (x - TCP_OFFSET[0], y - TCP_OFFSET[1], z - TCP_OFFSET[2])
```

So commanding `move_tcp(0, 0, 400)` actually moves the EE to `(0, 0, 302)` so the gripper tip ends up at Z = 400. This is important because:

- The camera sees the **top surface** of the object.
- `dz` from deprojection is the Z of that surface in the delta frame.
- You want the gripper tip at that Z (or slightly below to ensure a grip).
- The IK solver needs to know where the _EE_ should be, not the gripper tip.
- `move_tcp` handles this subtraction automatically.

### 5.4 Hover and approach offset

The pick doesn't go straight to the object — it uses a two-stage vertical approach:

```python
hover_z = pick_dz - approach_offset   # default approach_offset = 50 mm
```

1. **Approach (step 3):** Move gantry to `gantry_pick`, then move TCP to `(pick_dx, pick_dy, hover_z)`. This positions the gripper 50 mm _above_ the object (remember Z is positive downward, so `hover_z < pick_dz`).

2. **Descend (step 4):** Lower TCP straight down to `(pick_dx, pick_dy, pick_dz)`. Only Z changes — this gives a clean vertical drop.

This avoids lateral collisions: the robot first gets above the object, then descends.

### 5.5 Reachability pre-check

Before any motion, the pick sequence validates that the target is physically reachable:

```python
for label, z in [("hover", hover_z), ("pick", pick_dz)]:
    ee = (pick_dx - TCP_OFFSET[0],
          pick_dy - TCP_OFFSET[1],
          z       - TCP_OFFSET[2])
    a1, a2, a3 = robot.ik.inverse(*ee)
    if not all(-21.8 <= a <= 80.0 for a in (a1, a2, a3)):
        raise ValueError(...)
```

This checks _both_ the hover and pick positions. The IK solver runs on the EE position (after subtracting the TCP offset), and the resulting joint angles must be within `[−21.8°, 80.0°]` kinematic.

### 5.6 The full coordinate pipeline (summary)

```
Pixel (u, v)                                  ← YOLO bounding box centre
    │
    │  deproject(u, v)  [camera.py]
    │  Z = f·B/d·1000;  X = (u-cx)/f·Z;  Y = (v-cy)/f·Z
    ▼
Camera-frame 3D (X_c, Y_c, Z_c) in mm        ← stereo deprojection
    │
    │  camera_to_robot()  [coordinates.py]
    │  d = R·c + t
    ▼
Delta-frame 3D (dx, dy, dz) in mm             ← rigid-body transform
    │
    │  world_x = gantry_scan + dx
    │  gantry_pick = clamp(world_x)
    │  pick_dx = world_x − gantry_pick
    ▼
TCP target (pick_dx, dy, dz)                   ← gantry absorbs X, delta covers the rest
    │
    │  move_tcp → _tcp_to_ee  [delta_robot.py]
    │  EE = TCP − (0, 0, 98)
    ▼
EE target (pick_dx, dy, dz − 98)              ← gripper offset subtracted
    │
    │  inverse()  [delta_kinematics.py]
    ▼
Joint angles (θ1, θ2, θ3) kinematic           ← IK solution
    │
    │  _kinematic_to_firmware  [delta_robot.py]
    │  θ_fw = θ_kin − (−21.8) = θ_kin + 21.8
    ▼
Firmware angles → M command over serial        ← sent to ESP32
```
