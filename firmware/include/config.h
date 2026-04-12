#pragma once

// ── Firmware identity ───────────────────────────────────────────────────────

#define FW_VERSION "1.0.0"

// ── Serial (USB ↔ host PC) ──────────────────────────────────────────────────

#define SERIAL_BAUD 1000000

// ── Feetech gripper (STS3215 on Serial2) ────────────────────────────────────

#define STS_RX_PIN 16 // ESP32 RX2
#define STS_TX_PIN 17 // ESP32 TX2
#define STS_ID 5
#define STS_BAUD 1000000

#define GRIP_OPEN_POS 2900 // Feetech position units
#define GRIP_CLOSE_POS 1428
#define STS_MOVE_SPEED 2000
#define STS_MOVE_ACC 200

// ── Stepper pin map (DRV8825 drivers) ───────────────────────────────────────

// Delta motors
// NOTE: I just changed the order of the pins to be more consistent for the design of the PCB
#define M1_STEP_PIN 13
#define M1_DIR_PIN 12
#define M2_STEP_PIN 14
#define M2_DIR_PIN 27
#define M3_STEP_PIN 26
#define M3_DIR_PIN 25

// Delta motor enable (shared DRV8825 EN pin, active LOW)
#define DELTA_ENABLE_PIN 4

// Gantry motor
// NOTE: I just changed the order of the pins to be more consistent for the design of the PCB
#define M4_STEP_PIN 33
#define M4_DIR_PIN 32

// ── Delta motor parameters ──────────────────────────────────────────────────
//
//   NEMA 17: 200 full steps/rev
//   DRV8825: 1/16 microstepping →  9 600 microsteps / motor rev
//   Pulley:  3 : 1 reduction
//
//   DRV8825 microstepping table (TI datasheet Table 1):
//   MODE0 MODE1 MODE2 | Step mode
//     L     L     L   | Full step
//     H     L     L   | 1/2 step
//     L     H     L   | 1/4 step
//     H     H     L   | 1/8 step
//     L     L     H   | 1/16 step     ← delta motors, gantry motor
//     H     L     H   | 1/32 step
//     L     H     H   | 1/32 step
//     H     H     H   | 1/32 step
//
//   Delta  MODE pins: MODE0 = LOW, MODE1 = LOW, MODE2 = HIGH.
//   Gantry MODE pins: MODE0 = LOW, MODE1 = LOW, MODE2 = HIGH.

#define DELTA_MICROSTEPS 16
#define DELTA_FULL_STEPS_REV 200
#define DELTA_PULLEY_RATIO 3

#define DELTA_STEPS_PER_REV (long)(DELTA_FULL_STEPS_REV * DELTA_MICROSTEPS * DELTA_PULLEY_RATIO)
#define DELTA_STEPS_PER_DEG (DELTA_STEPS_PER_REV / 360.0f) // ≈ 26.67

// ── Gantry motor parameters ─────────────────────────────────────────────────
//
//   NEMA 17: 200 full steps/rev
//   DRV8825: 1/16 microstepping  →  3 200 microsteps / rev
//   GT2 belt, 20-tooth pulley    →  40 mm / rev  →  80 steps/mm
//

#define GANTRY_MICROSTEPS 16
#define GANTRY_FULL_STEPS_REV 200
#define GANTRY_PULLEY_TEETH 20
#define GANTRY_BELT_PITCH_MM 2.0f

#define GANTRY_STEPS_PER_REV (long)(GANTRY_FULL_STEPS_REV * GANTRY_MICROSTEPS)
#define GANTRY_MM_PER_REV (GANTRY_PULLEY_TEETH * GANTRY_BELT_PITCH_MM)
#define GANTRY_STEPS_PER_MM (GANTRY_STEPS_PER_REV / GANTRY_MM_PER_REV) // 80

// ── Default speed / acceleration ────────────────────────────────────────────

#define DELTA_DEFAULT_SPEED 8000.0f // steps / s
#define DELTA_DEFAULT_ACCEL 4000.0f // steps / s²
#define GANTRY_DEFAULT_SPEED 8000.0f
#define GANTRY_DEFAULT_ACCEL 4000.0f

// ── Soft joint limits ───────────────────────────────────────────────────────
// Firmware 0° = mechanical stop (21.8° above horizontal).
// Home = 6.8° firmware (15° above horizontal).
// Max = 101.8° firmware (home + 95° = 80° below horizontal).
#define DELTA_ANGLE_MIN 0.0f   // degrees — mechanical stop
#define DELTA_ANGLE_MAX 101.8f // degrees — home + 95° (80° below horizontal)

#define GANTRY_POS_MIN 0.0f   // mm — endstop at home
#define GANTRY_POS_MAX 625.0f // mm — max travel from endstop

// Gantry endstop (limit switch) for homing. LOW = pressed (at home).
#define GANTRY_ENDSTOP_PIN 34
#define GANTRY_HOME_SPEED 8000.0f   // steps/s — matches tested endstop speed
#define GANTRY_HOME_BACKOFF_MM 2.0f // mm to back off after hitting endstop

// ── Counts ──────────────────────────────────────────────────────────────────

#define NUM_DELTA_MOTORS 3
#define NUM_MOTORS 4
