#include <Arduino.h>
#include <AccelStepper.h>
#include <Dynamixel2Arduino.h>

// --- CONFIGURATION: DYNAMIXEL (GRIPPER) ---
#define DXL_RX 16        // ESP32 RX2
#define DXL_TX 17        // ESP32 TX2
#define DXL_DIR -1       // Waveshare v1.1 usually has auto-direction
#define DXL_ID 1         // Default XL330 ID
#define DXL_BAUD 1000000 // Default XL330 Baudrate
#define DXL_PROTOCOL 2.0 // XL330 uses Protocol 2.0

const int GRIP_CLOSE = 2036; // Adjust these values based on your
const int GRIP_OPEN = 3095;  // specific gripper hardware

// --- CONFIGURATION: STEPPERS (DELTA + GANTRY) ---
#define M1_STEP 12
#define M1_DIR 13
#define M2_STEP 14
#define M2_DIR 27
#define M3_STEP 26
#define M3_DIR 25
#define M4_STEP 32
#define M4_DIR 33

const int STEPS_REV = 6400; // 1/32 Microstepping

// --- OBJECT INITIALIZATION ---
AccelStepper stepper1(1, M1_STEP, M1_DIR);
AccelStepper stepper2(1, M2_STEP, M2_DIR);
AccelStepper stepper3(1, M3_STEP, M3_DIR);
AccelStepper stepper4(1, M4_STEP, M4_DIR);
AccelStepper *motors[] = {&stepper1, &stepper2, &stepper3, &stepper4};

// Initialize Dynamixel on Serial2
HardwareSerial &DXL_SERIAL = Serial2;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR);

void setup()
{
    Serial.begin(115200); // USB to Laptop
    delay(2000);

    // 1. Initialize Stepper Drivers
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setMaxSpeed(8000);
        motors[i]->setAcceleration(4000);
    }
    Serial.println("Steppers Initialized.");

    // 2. Initialize Serial2 for Waveshare Board
    // Format: .begin(baud, config, rx_pin, tx_pin)
    DXL_SERIAL.begin(DXL_BAUD, SERIAL_8N1, DXL_RX, DXL_TX);

    // 3. Initialize Dynamixel Library
    dxl.begin(DXL_BAUD);
    dxl.setPortProtocolVersion(DXL_PROTOCOL);

    // Check if the servo is actually responding
    if (dxl.ping(DXL_ID))
    {
        Serial.println("Dynamixel XL330 Detected!");
        dxl.torqueOff(DXL_ID);
        dxl.setOperatingMode(DXL_ID, OP_POSITION);
        dxl.torqueOn(DXL_ID);
    }
    else
    {
        Serial.println("ERROR: Dynamixel not found. Check wiring/ground.");
    }

    Serial.println("--- System Ready ---");
}

void loop()
{
    // --- PHASE 1: SEQUENTIAL TEST + GRIPPER OPEN ---
    Serial.println("Opening Gripper...");
    dxl.setGoalPosition(DXL_ID, GRIP_OPEN);
    delay(500);

    for (int i = 0; i < 4; i++)
    {
        Serial.printf("Testing Motor %d...\n", i + 1);
        motors[i]->runToNewPosition(STEPS_REV);
        delay(200);
        motors[i]->runToNewPosition(0);
    }

    // --- PHASE 2: SIMULTANEOUS TEST + GRIPPER CLOSE ---
    Serial.println("Closing Gripper and moving all motors...");
    dxl.setGoalPosition(DXL_ID, GRIP_CLOSE);

    // Set targets
    for (int i = 0; i < 4; i++)
        motors[i]->moveTo(STEPS_REV);

    // Run until all reach target
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 ||
           stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
    {
        for (int i = 0; i < 4; i++)
            motors[i]->run();
    }

    delay(1000);

    // --- PHASE 3: RESET ---
    Serial.println("Resetting Positions...");
    dxl.setGoalPosition(DXL_ID, GRIP_OPEN);

    for (int i = 0; i < 4; i++)
        motors[i]->moveTo(0);

    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 ||
           stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0)
    {
        for (int i = 0; i < 4; i++)
            motors[i]->run();
    }

    Serial.println("Cycle Complete. Cooling for 5s...");
    delay(5000);
}