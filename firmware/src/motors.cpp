#include "motors.h"
#include "config.h"
#include <AccelStepper.h>
#include <Arduino.h>

// Gantry homing state: 0 = idle, 1 = moving toward endstop, 2 = backing off
static int s_gantry_homing = 0;

// File-scope motor instances (avoids C++ aggregate-init headaches)
static AccelStepper s_delta[] = {
    AccelStepper(AccelStepper::DRIVER, M1_STEP_PIN, M1_DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, M2_STEP_PIN, M2_DIR_PIN),
    AccelStepper(AccelStepper::DRIVER, M3_STEP_PIN, M3_DIR_PIN),
};
static AccelStepper s_gantry(AccelStepper::DRIVER, M4_STEP_PIN, M4_DIR_PIN);

Motors motors;

static bool s_delta_enabled = true;

// ── Helpers ─────────────────────────────────────────────────────────────────

static inline long degToSteps(float deg)
{
    return (long)(deg * DELTA_STEPS_PER_DEG);
}

static inline float stepsToDeg(long steps)
{
    return steps / DELTA_STEPS_PER_DEG;
}

static inline long mmToSteps(float mm)
{
    return (long)(mm * GANTRY_STEPS_PER_MM);
}

static inline float stepsToMm(long steps)
{
    return steps / GANTRY_STEPS_PER_MM;
}

// ── Public API ──────────────────────────────────────────────────────────────

void Motors::init()
{
    pinMode(DELTA_ENABLE_PIN, OUTPUT);
    digitalWrite(DELTA_ENABLE_PIN, LOW); // LOW = enabled on DRV8825
    s_delta_enabled = true;

    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
    {
        s_delta[i].setMaxSpeed(DELTA_DEFAULT_SPEED);
        s_delta[i].setAcceleration(DELTA_DEFAULT_ACCEL);
    }
    s_gantry.setMaxSpeed(GANTRY_DEFAULT_SPEED);
    s_gantry.setAcceleration(GANTRY_DEFAULT_ACCEL);
    pinMode(GANTRY_ENDSTOP_PIN, INPUT);
}

bool Motors::setDeltaTarget(float a1, float a2, float a3)
{
    const float angles[] = {a1, a2, a3};
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
    {
        if (angles[i] < DELTA_ANGLE_MIN || angles[i] > DELTA_ANGLE_MAX)
            return false;
    }
    s_delta[0].moveTo(degToSteps(a1));
    s_delta[1].moveTo(degToSteps(a2));
    s_delta[2].moveTo(degToSteps(a3));
    return true;
}

bool Motors::setGantryTarget(float x_mm)
{
    if (x_mm < GANTRY_POS_MIN || x_mm > GANTRY_POS_MAX)
        return false;
    s_gantry.moveTo(mmToSteps(x_mm));
    return true;
}

void Motors::runAll()
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
        s_delta[i].run();

    if (s_gantry_homing == 1)
    {
        // Move toward endstop (negative direction). Endstop LOW = pressed.
        if (digitalRead(GANTRY_ENDSTOP_PIN) == LOW)
        {
            s_gantry.stop();
            s_gantry.setCurrentPosition(0);
            s_gantry.moveTo(mmToSteps(GANTRY_HOME_BACKOFF_MM));
            s_gantry_homing = 2;
        }
        else
        {
            s_gantry.setSpeed(-GANTRY_HOME_SPEED);
            s_gantry.runSpeed();
        }
    }
    else if (s_gantry_homing == 2)
    {
        s_gantry.run();
        if (s_gantry.distanceToGo() == 0)
            s_gantry_homing = 0;
    }
    else
    {
        s_gantry.run();
    }
}

bool Motors::isMoving() const
{
    return deltaMoving() || gantryMoving() || (s_gantry_homing != 0);
}

bool Motors::deltaMoving() const
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
    {
        if (s_delta[i].distanceToGo() != 0)
            return true;
    }
    return false;
}

bool Motors::gantryMoving() const
{
    return s_gantry.distanceToGo() != 0;
}

float Motors::getDeltaAngle(int idx) const
{
    if (idx < 0 || idx >= NUM_DELTA_MOTORS)
        return 0.0f;
    return stepsToDeg(s_delta[idx].currentPosition());
}

float Motors::getGantryMm() const
{
    return stepsToMm(s_gantry.currentPosition());
}

void Motors::setDeltaSpeed(float v)
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
        s_delta[i].setMaxSpeed(v);
}

void Motors::setDeltaAccel(float a)
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
        s_delta[i].setAcceleration(a);
}

void Motors::setGantrySpeed(float v) { s_gantry.setMaxSpeed(v); }
void Motors::setGantryAccel(float a) { s_gantry.setAcceleration(a); }

void Motors::stop()
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
        s_delta[i].stop();
    s_gantry.stop();
}

void Motors::emergencyStop()
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
        s_delta[i].setCurrentPosition(s_delta[i].currentPosition());
    s_gantry.setCurrentPosition(s_gantry.currentPosition());
}

void Motors::setZero()
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
        s_delta[i].setCurrentPosition(0);
    s_gantry.setCurrentPosition(0);
}

void Motors::home()
{
    for (int i = 0; i < NUM_DELTA_MOTORS; i++)
        s_delta[i].moveTo(0);
    s_gantry.moveTo(0);
}

void Motors::enableDelta()
{
    digitalWrite(DELTA_ENABLE_PIN, LOW);
    s_delta_enabled = true;
}

void Motors::disableDelta()
{
    digitalWrite(DELTA_ENABLE_PIN, HIGH);
    s_delta_enabled = false;
}

bool Motors::isDeltaEnabled() const
{
    return s_delta_enabled;
}

void Motors::startGantryHoming()
{
    s_gantry_homing = 1;
}

bool Motors::isGantryHoming() const
{
    return s_gantry_homing != 0;
}
