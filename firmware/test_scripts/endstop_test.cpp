#include <Arduino.h>
#include <AccelStepper.h>

// --- PINS ---
#define STEP_PIN 32
#define DIR_PIN 33
#define LIMIT_PIN 34

// 1 = Driver interface
AccelStepper stepper4(1, STEP_PIN, DIR_PIN);

void setup()
{
    Serial.begin(115200);

    // Using INPUT because your SVG board has its own pull-up resistors
    pinMode(LIMIT_PIN, INPUT);

    stepper4.setMaxSpeed(2000);
    stepper4.setAcceleration(1000);

    Serial.println("--- Starting Homing Sequence ---");
}

void loop()
{
    // Read the sensor (1 = Open, 0 = Pressed)
    int limitSwitch = digitalRead(LIMIT_PIN);

    if (limitSwitch == HIGH)
    {
        // Switch is NOT pressed: Keep moving toward home
        // Use a constant speed for homing to ensure accuracy
        stepper4.setSpeed(-1000); // Negative usually moves 'backwards' toward a switch
        stepper4.runSpeed();
    }
    else
    {
        // Switch IS pressed: Stop and Define Home
        stepper4.stop();
        stepper4.setCurrentPosition(0);

        Serial.println("HOME REACHED! Position reset to 0.");

        // Back off slightly so the switch isn't held down
        delay(500);
        stepper4.runToNewPosition(200);

        Serial.println("Ready for commands. System Halted.");
        while (1)
            ; // Stay here until reset
    }
}