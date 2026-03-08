#pragma once

// ── Firmware identity ───────────────────────────────────────────────────────

#define FW_VERSION "1.0.0"

// ── Serial (USB ↔ host PC) ──────────────────────────────────────────────────

#define SERIAL_BAUD 1000000

// ── Dynamixel gripper (XL330 on Serial2 via Waveshare adapter) ──────────────

#define DXL_RX_PIN    16       // ESP32 RX2
#define DXL_TX_PIN    17       // ESP32 TX2
#define DXL_DIR_PIN   -1       // Waveshare v1.1 has auto-direction
#define DXL_ID        1
#define DXL_BAUD      1000000
#define DXL_PROTOCOL  2.0f

#define GRIP_OPEN_POS   3095   // Dynamixel position units (0–4095)
#define GRIP_CLOSE_POS  2036

// ── Stepper pin map (DRV8825 drivers) ───────────────────────────────────────

// Delta motors
#define M1_STEP_PIN  12
#define M1_DIR_PIN   13
#define M2_STEP_PIN  14
#define M2_DIR_PIN   27
#define M3_STEP_PIN  26
#define M3_DIR_PIN   25

// Gantry motor
#define M4_STEP_PIN  32
#define M4_DIR_PIN   33

// ── Delta motor parameters ──────────────────────────────────────────────────
//
//   NEMA 17: 200 full steps/rev
//   DRV8825: 1/32 microstepping  →  6 400 microsteps / motor rev
//   Pulley:  3 : 1 reduction     →  19 200 microsteps / output rev
//

#define DELTA_MICROSTEPS       32
#define DELTA_FULL_STEPS_REV   200
#define DELTA_PULLEY_RATIO     3

#define DELTA_STEPS_PER_REV    (long)(DELTA_FULL_STEPS_REV * DELTA_MICROSTEPS * DELTA_PULLEY_RATIO)
#define DELTA_STEPS_PER_DEG    (DELTA_STEPS_PER_REV / 360.0f)   // ≈ 53.33

// ── Gantry motor parameters ─────────────────────────────────────────────────
//
//   NEMA 17: 200 full steps/rev
//   DRV8825: 1/32 microstepping  →  6 400 microsteps / motor rev
//   GT2 belt, 20-tooth pulley    →  40 mm travel / motor rev
//

#define GANTRY_MICROSTEPS      32
#define GANTRY_FULL_STEPS_REV  200
#define GANTRY_PULLEY_TEETH    20
#define GANTRY_BELT_PITCH_MM   2.0f

#define GANTRY_STEPS_PER_REV   (long)(GANTRY_FULL_STEPS_REV * GANTRY_MICROSTEPS)
#define GANTRY_MM_PER_REV      (GANTRY_PULLEY_TEETH * GANTRY_BELT_PITCH_MM)
#define GANTRY_STEPS_PER_MM    (GANTRY_STEPS_PER_REV / GANTRY_MM_PER_REV)  // 160

// ── Default speed / acceleration ────────────────────────────────────────────

#define DELTA_DEFAULT_SPEED    8000.0f    // steps / s
#define DELTA_DEFAULT_ACCEL    4000.0f    // steps / s²
#define GANTRY_DEFAULT_SPEED   8000.0f
#define GANTRY_DEFAULT_ACCEL   4000.0f

// ── Soft joint limits ───────────────────────────────────────────────────────

#define DELTA_ANGLE_MIN  -70.0f   // degrees
#define DELTA_ANGLE_MAX   70.0f

#define GANTRY_POS_MIN     0.0f   // mm
#define GANTRY_POS_MAX   800.0f

// ── Counts ──────────────────────────────────────────────────────────────────

#define NUM_DELTA_MOTORS  3
#define NUM_MOTORS        4
