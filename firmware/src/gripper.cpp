#include "gripper.h"
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

static HardwareSerial& dxlSerial = Serial2;
static Dynamixel2Arduino dxl(dxlSerial, DXL_DIR_PIN);

Gripper gripper;

bool Gripper::init() {
    dxlSerial.begin(DXL_BAUD, SERIAL_8N1, DXL_RX_PIN, DXL_TX_PIN);
    dxl.begin(DXL_BAUD);
    dxl.setPortProtocolVersion(DXL_PROTOCOL);

    _connected = dxl.ping(DXL_ID);
    if (!_connected) return false;

    dxl.torqueOff(DXL_ID);
    dxl.setOperatingMode(DXL_ID, OP_POSITION);
    dxl.torqueOn(DXL_ID);
    return true;
}

void Gripper::open()  { if (_connected) dxl.setGoalPosition(DXL_ID, GRIP_OPEN_POS); }
void Gripper::close() { if (_connected) dxl.setGoalPosition(DXL_ID, GRIP_CLOSE_POS); }

void Gripper::setPosition(int32_t pos) {
    if (_connected) dxl.setGoalPosition(DXL_ID, pos);
}

int32_t Gripper::getPosition() {
    if (!_connected) return -1;
    return dxl.getPresentPosition(DXL_ID);
}

int32_t Gripper::getTemperature() {
    if (!_connected) return -1;
    return dxl.readControlTableItem(PRESENT_TEMPERATURE, DXL_ID);
}

int32_t Gripper::getLoad() {
    if (!_connected) return 0;
    return dxl.readControlTableItem(PRESENT_CURRENT, DXL_ID);
}

bool Gripper::isMoving() {
    if (!_connected) return false;
    return dxl.readControlTableItem(MOVING, DXL_ID) != 0;
}
