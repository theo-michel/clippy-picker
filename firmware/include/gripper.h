#pragma once

#include <stdint.h>
#include "config.h"

/// Dynamixel XL330 gripper control over Serial2 (via Waveshare adapter).
///
/// Provides open/close, arbitrary position control, and telemetry
/// readback (position, temperature, load).
class Gripper {
public:
    bool init();

    void open();
    void close();
    void setPosition(int32_t pos);

    int32_t getPosition();
    int32_t getTemperature();   // °C
    int32_t getLoad();          // raw current register value (mA)
    bool    isMoving();

    bool isConnected() const { return _connected; }

private:
    bool _connected = false;
};

extern Gripper gripper;
