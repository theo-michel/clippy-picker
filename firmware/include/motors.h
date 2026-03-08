#pragma once

#include "config.h"

/// Stepper motor management for 3 delta arms + 1 gantry axis.
///
/// Converts between physical units (degrees, mm) and motor steps,
/// enforces soft joint limits, and provides a non-blocking `runAll()`
/// that must be called every loop iteration.
class Motors {
public:
    void init();

    // Delta motion — angles in degrees
    bool setDeltaTarget(float a1, float a2, float a3);

    // Gantry motion — position in mm
    bool setGantryTarget(float x_mm);

    // Must be called every loop() iteration
    void runAll();

    // Status
    bool isMoving()      const;
    bool deltaMoving()   const;
    bool gantryMoving()  const;

    // Position readback
    float getDeltaAngle(int idx) const;   // idx 0–2, returns degrees
    float getGantryMm()         const;

    // Speed / acceleration (steps/s, steps/s²)
    void setDeltaSpeed(float v);
    void setDeltaAccel(float a);
    void setGantrySpeed(float v);
    void setGantryAccel(float a);

    // Control
    void stop();            // decelerate to standstill
    void emergencyStop();   // instant halt, position preserved
    void setZero();         // declare current position as origin
    void home();            // move all axes to zero
};

extern Motors motors;
