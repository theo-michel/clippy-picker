#include <Arduino.h>
#include "config.h"
#include "motors.h"
#include "gripper.h"
#include "protocol.h"

static bool wasMoving = false;

void setup() {
    protocol.init();
    delay(500);

    Serial.printf("FR8 Delta v%s\n", FW_VERSION);

    motors.init();
    Serial.println("Motors OK");

    if (gripper.init()) {
        Serial.println("Gripper OK");
    } else {
        Serial.println("WARN:Gripper not detected");
    }

    Serial.println("READY");
}

void loop() {
    if (protocol.readCommand()) {
        protocol.dispatch();
    }

    motors.runAll();

    bool moving = motors.isMoving();
    if (wasMoving && !moving) {
        Protocol::done();
    }
    wasMoving = moving;
}
