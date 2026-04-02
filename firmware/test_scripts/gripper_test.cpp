#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "config.h"

using namespace ControlTableItem;

// Use a dummy direction pin to force the library to call flush() after writes.
// Without this (dir_pin = -1), the library skips flush() and write packets
// may not fully transmit before it starts listening for the response.
// Pick any unused GPIO — it doesn't need to be connected to anything.
#define DXL_DIR_PIN_DUMMY 4

static HardwareSerial &dxlSerial = Serial2;
static Dynamixel2Arduino dxl(dxlSerial, DXL_DIR_PIN_DUMMY);

namespace
{
    constexpr unsigned long kMoveDelayMs = 1500;
    constexpr unsigned long kLoopPauseMs = 1000;

    bool writeAndVerify(const char *label, uint8_t item, int32_t value)
    {
        bool writeOk = dxl.writeControlTableItem(item, DXL_ID, value);
        int32_t readback = dxl.readControlTableItem(item, DXL_ID);

        Serial.print(label);
        Serial.print(" write=");
        Serial.print(writeOk ? "OK" : "FAIL");
        Serial.print(" target=");
        Serial.print(value);
        Serial.print(" readback=");
        Serial.println(readback);

        return writeOk;
    }

    void printControlState(const char *label)
    {
        long torqueEnable = dxl.readControlTableItem(TORQUE_ENABLE, DXL_ID);
        long operatingMode = dxl.readControlTableItem(OPERATING_MODE, DXL_ID);
        long goalPosition = dxl.readControlTableItem(GOAL_POSITION, DXL_ID);
        long moving = dxl.readControlTableItem(MOVING, DXL_ID);
        long presentVoltage = dxl.readControlTableItem(PRESENT_VOLTAGE, DXL_ID);

        Serial.print(label);
        Serial.print(" torque=");
        Serial.print(torqueEnable);
        Serial.print(" mode=");
        Serial.print(operatingMode);
        Serial.print(" goal=");
        Serial.print(goalPosition);
        Serial.print(" moving=");
        Serial.print(moving);
        Serial.print(" voltage=");
        Serial.println(presentVoltage);
    }

    bool initGripper()
    {
        dxlSerial.begin(DXL_BAUD, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
        dxl.begin(DXL_BAUD);
        dxl.setPortProtocolVersion(DXL_PROTOCOL);

        bool pingOk = dxl.ping(DXL_ID);
        Serial.print("ping=");
        Serial.println(pingOk ? "OK" : "FAIL");
        if (!pingOk)
        {
            return false;
        }

        bool torqueOffOk = writeAndVerify("torque-off", TORQUE_ENABLE, 0);
        bool modeOk = dxl.setOperatingMode(DXL_ID, OP_POSITION);
        bool torqueOnOk = writeAndVerify("torque-on", TORQUE_ENABLE, 1);

        Serial.print("torqueOff=");
        Serial.println(torqueOffOk ? "OK" : "FAIL");
        Serial.print("setOperatingMode=");
        Serial.println(modeOk ? "OK" : "FAIL");
        Serial.print("torqueOn=");
        Serial.println(torqueOnOk ? "OK" : "FAIL");
        printControlState("after-init");

        return true;
    }

    void printTelemetry(const char *label)
    {
        long position = dxl.getPresentPosition(DXL_ID);
        long temperature = dxl.readControlTableItem(PRESENT_TEMPERATURE, DXL_ID);
        long current = dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID);

        Serial.print(label);
        Serial.print(" pos=");
        Serial.print(position);
        Serial.print(" tempC=");
        Serial.print(temperature);
        Serial.print(" current=");
        Serial.println(current);
        printControlState(label);
    }

    void moveToPosition(const char *label, int32_t position)
    {
        Serial.print("Moving ");
        Serial.print(label);
        Serial.print(" -> ");
        Serial.println(position);

        bool goalOk = writeAndVerify("goal-write", GOAL_POSITION, position);
        Serial.print("setGoalPositionRaw=");
        Serial.println(goalOk ? "OK" : "FAIL");
        printControlState("after-write");
        delay(kMoveDelayMs);
        printTelemetry(label);
    }
} // namespace

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("=== Gripper Test ===");
    Serial.printf("Target ID=%d baud=%d\n", DXL_ID, DXL_BAUD);
    Serial.printf("OPEN=%d CLOSE=%d\n", GRIP_OPEN_POS, GRIP_CLOSE_POS);

    if (!initGripper())
    {
        Serial.println("ERROR: Failed to detect gripper");
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("Gripper detected");
    printTelemetry("startup");
}

void loop()
{
    moveToPosition("open", GRIP_OPEN_POS);
    delay(kLoopPauseMs);

    moveToPosition("close", GRIP_CLOSE_POS);
    delay(kLoopPauseMs);
}
