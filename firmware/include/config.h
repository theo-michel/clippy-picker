#pragma once

// ── Firmware identity ───────────────────────────────────────────────────────

#define FW_VERSION "1.0.0"

// ── Serial (USB ↔ host PC) ──────────────────────────────────────────────────

#define SERIAL_BAUD 1000000

// ── Feetech gripper (STS3215 on Serial2) ────────────────────────────────────

#define STS_RX_PIN 16  // ESP32 RX2
#define STS_TX_PIN 17  // ESP32 TX2
#define STS_ID 5
#define STS_BAUD 1000000

#define GRIP_OPEN_POS  2900 // Feetech position units
#define GRIP_CLOSE_POS 1428
#define STS_MOVE_SPEED 2000
#define STS_MOVE_ACC   200

// ── Stepper pin map (DRV8825 drivers) ───────────────────────────────────────

// Delta motors
#define M1_STEP_PIN 12
#define M1_DIR_PIN 13
#define M2_STEP_PIN 14
#define M2_DIR_PIN 27
#define M3_STEP_PIN 26
#define M3_DIR_PIN 25

// Delta motor enable (shared DRV8825 EN pin, active LOW)
#define DELTA_ENABLE_PIN 4

// Gantry motor
#define M4_STEP_PIN 32
#define M4_DIR_PIN 33

// ── Delta motor parameters ──────────────────────────────────────────────────
//
//   NEMA 17: 200 full steps/rev
//   DRV8825: 1/32 microstepping  →  6 400 microsteps / motor rev
//   Pulley:  3 : 1 reduction     →  19 200 microsteps / output rev
//

#define DELTA_MICROSTEPS 32
#define DELTA_FULL_STEPS_REV 200
#define DELTA_PULLEY_RATIO 3

#define DELTA_STEPS_PER_REV (long)(DELTA_FULL_STEPS_REV * DELTA_MICROSTEPS * DELTA_PULLEY_RATIO)
#define DELTA_STEPS_PER_DEG (DELTA_STEPS_PER_REV / 360.0f) // ≈ 53.33

// ── Gantry motor parameters ─────────────────────────────────────────────────
//
//   NEMA 17: 200 full steps/rev
//   DRV8825: 1/32 microstepping  →  6 400 microsteps / motor rev
//   GT2 belt, 20-tooth pulley    →  40 mm travel / motor rev
//

#define GANTRY_MICROSTEPS 32
#define GANTRY_FULL_STEPS_REV 200
#define GANTRY_PULLEY_TEETH 20
#define GANTRY_BELT_PITCH_MM 2.0f

#define GANTRY_STEPS_PER_REV (long)(GANTRY_FULL_STEPS_REV * GANTRY_MICROSTEPS)
#define GANTRY_MM_PER_REV (GANTRY_PULLEY_TEETH * GANTRY_BELT_PITCH_MM)
#define GANTRY_STEPS_PER_MM (GANTRY_STEPS_PER_REV / GANTRY_MM_PER_REV) // 160

// ── Default speed / acceleration ────────────────────────────────────────────

#define DELTA_DEFAULT_SPEED 8000.0f // steps / s
#define DELTA_DEFAULT_ACCEL 4000.0f // steps / s²
#define GANTRY_DEFAULT_SPEED 16000.0f
#define GANTRY_DEFAULT_ACCEL 8000.0f

// ── Soft joint limits ───────────────────────────────────────────────────────
// Home = 0° (arms at mechanical limit, ~20° above horizontal). Max 95° down to avoid 90° singularity.
#define DELTA_ANGLE_MIN 0.0f  // degrees — home
#define DELTA_ANGLE_MAX 95.0f // degrees — max down from home (~-75° from horizontal)

#define GANTRY_POS_MIN 0.0f   // mm — endstop at home
#define GANTRY_POS_MAX 625.0f // mm — max travel from endstop

// Gantry endstop (limit switch) for homing. LOW = pressed (at home).
#define GANTRY_ENDSTOP_PIN 34
#define GANTRY_HOME_SPEED 16000.0f  // steps/s — matches tested endstop speed
#define GANTRY_HOME_BACKOFF_MM 2.0f // mm to back off after hitting endstop

// ── Counts ──────────────────────────────────────────────────────────────────

#define NUM_DELTA_MOTORS 3
#define NUM_MOTORS 4
