#include <Arduino.h>
#include <AccelStepper.h>

// Delta Motors
#define M1_STEP 12
#define M1_DIR  13
#define M2_STEP 14
#define M2_DIR  27
#define M3_STEP 26
#define M3_DIR  25

// Gantry Motor (Motor 4)
#define M4_STEP 32
#define M4_DIR  33

// 1/32 Microstepping = 6400 steps/rev
const int STEPS_REV = 6400;

// Initialize the 4 stepper objects
AccelStepper stepper1(1, M1_STEP, M1_DIR);
AccelStepper stepper2(1, M2_STEP, M2_DIR);
AccelStepper stepper3(1, M3_STEP, M3_DIR);
AccelStepper stepper4(1, M4_STEP, M4_DIR);

// Array for easier batch configuration
AccelStepper* motors[] = {&stepper1, &stepper2, &stepper3, &stepper4};

void setup() {
    Serial.begin(115200);
    delay(2000); // Give you time to open the Serial Monitor

    for(int i = 0; i < 4; i++) {
        motors[i]->setMaxSpeed(8000);     
        motors[i]->setAcceleration(4000); 
    }
    
    Serial.println("--- 4-Motor System Initialized ---");
    Serial.println("Ensure 12V/24V power is ON.");
}

void loop() {
    // 1. Sequential Test (Safe Current Draw: ~1.5A max)
    // This moves each motor one-by-one so you can check wiring/heat
    for(int i = 0; i < 4; i++) {
        Serial.print("Testing Motor ");
        Serial.print(i + 1);
        Serial.println("...");
        
        motors[i]->runToNewPosition(STEPS_REV); // Move 1 full turn
        delay(500);
        motors[i]->runToNewPosition(0);         // Back to start
        delay(500);
    }

    Serial.println("Waiting 2 seconds before Simultaneous Test...");
    delay(2000);

    // 2. Simultaneous Test (High Current Draw: up to 6A - 7A!)
    // This tests if your power supply and capacitors can handle the surge
    Serial.println("MOVING ALL MOTORS AT ONCE (Watch your power supply!)");
    
    stepper1.moveTo(STEPS_REV);
    stepper2.moveTo(STEPS_REV);
    stepper3.moveTo(STEPS_REV);
    stepper4.moveTo(STEPS_REV);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || 
           stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
    }

    delay(1000);

    // Return all to zero
    Serial.println("Returning all to 0...");
    stepper1.moveTo(0);
    stepper2.moveTo(0);
    stepper3.moveTo(0);
    stepper4.moveTo(0);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || 
           stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0) {
        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
    }
    
    Serial.println("Cycle Complete. Cooling down for 5 seconds...");
    delay(5000);
}