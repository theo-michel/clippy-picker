#include <Arduino.h>
#include <AccelStepper.h>

#define STEP_PIN 32
#define DIR_PIN 33
#define LIMIT_PIN 34

#define STEPS_PER_MM 160.0f
#define JOG_MM 10.0f
#define JOG_STEP_SIZE (long)(JOG_MM * STEPS_PER_MM)
#define MAX_POS 100000 // 625mm

AccelStepper stepper(1, STEP_PIN, DIR_PIN);

enum State
{
    HOMING,
    BACKING_OFF,
    READY
};
static State state = HOMING;

float stepsToMm(long steps) { return steps / STEPS_PER_MM; }

void printPosition()
{
    Serial.print("pos=");
    Serial.print(stepsToMm(stepper.currentPosition()), 1);
    Serial.print("mm (");
    Serial.print(stepper.currentPosition());
    Serial.println(" µsteps)");
}

void setup()
{
    Serial.begin(115200);
    pinMode(LIMIT_PIN, INPUT);

    stepper.setMaxSpeed(16000);
    stepper.setAcceleration(8000);

    Serial.println("--- Endstop Test ---");
    Serial.println("Homing...");
}

void loop()
{
    switch (state)
    {
    case HOMING:
        if (digitalRead(LIMIT_PIN) == LOW)
        {
            stepper.stop();
            stepper.setCurrentPosition(0);
            stepper.moveTo(320); // back off ~2mm
            state = BACKING_OFF;
        }
        else
        {
            stepper.setSpeed(-16000);
            stepper.runSpeed();
        }
        return;

    case BACKING_OFF:
        stepper.run();
        if (stepper.distanceToGo() == 0)
        {
            stepper.setCurrentPosition(0);
            Serial.println("HOME OK");
            Serial.println("Controls:");
            Serial.println("  a = jog away from endstop");
            Serial.println("  d = jog toward endstop");
            Serial.println("  s = stop");
            Serial.println("  h = re-home");
            Serial.println("  p = print position");
            printPosition();
            state = READY;
        }
        return;

    case READY:
        break;
    }

    if (digitalRead(LIMIT_PIN) == LOW && stepper.targetPosition() < stepper.currentPosition())
    {
        stepper.stop();
        stepper.setCurrentPosition(0);
        Serial.println("ENDSTOP HIT — stopped, position reset to 0.0mm (0 µsteps)");
    }

    if (Serial.available())
    {
        char c = Serial.read();
        switch (c)
        {
        case 'a':
        case 'A':
        {
            long target = min(stepper.currentPosition() + JOG_STEP_SIZE, (long)MAX_POS);
            if (stepper.currentPosition() >= MAX_POS)
            {
                Serial.println("already at max position");
            }
            else
            {
                stepper.moveTo(target);
                Serial.print("jog + -> ");
                Serial.print(stepsToMm(stepper.targetPosition()), 1);
                Serial.print("mm (");
                Serial.print(stepper.targetPosition());
                Serial.println(" µsteps)");
            }
        }
        break;
        case 'd':
        case 'D':
            if (digitalRead(LIMIT_PIN) == LOW)
            {
                Serial.println("already at endstop");
            }
            else
            {
                stepper.moveTo(stepper.currentPosition() - JOG_STEP_SIZE);
                Serial.print("jog - -> ");
                Serial.print(stepsToMm(stepper.targetPosition()), 1);
                Serial.print("mm (");
                Serial.print(stepper.targetPosition());
                Serial.println(" µsteps)");
            }
            break;
        case 's':
        case 'S':
            stepper.stop();
            Serial.println("STOP");
            break;
        case 'h':
        case 'H':
            Serial.println("Re-homing...");
            state = HOMING;
            return;
        case 'p':
        case 'P':
            printPosition();
            break;
        }
    }

    stepper.run();
}
