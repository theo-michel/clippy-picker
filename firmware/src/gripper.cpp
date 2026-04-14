#include "gripper.h"
#include <SCServo.h>

static SMS_STS st;

Gripper gripper;

bool Gripper::init() {
    Serial2.begin(STS_BAUD, SERIAL_8N1, STS_RX_PIN, STS_TX_PIN);
    st.pSerial = &Serial2;
    delay(100);

    for (int attempt = 0; attempt < 5; attempt++) {
        delay(200);
        if (st.Ping(STS_ID) != -1) {
            _connected = true;
            st.EnableTorque(STS_ID, 1);
            return true;
        }
    }
    _connected = false;
    return false;
}

void Gripper::open()  { if (_connected) st.WritePosEx(STS_ID, GRIP_OPEN_POS, STS_MOVE_SPEED, STS_MOVE_ACC); }
void Gripper::close() { if (_connected) st.WritePosEx(STS_ID, GRIP_CLOSE_POS, STS_MOVE_SPEED, STS_MOVE_ACC); }

void Gripper::setPosition(int32_t pos) {
    if (_connected) st.WritePosEx(STS_ID, (int16_t)pos, STS_MOVE_SPEED, STS_MOVE_ACC);
}

int32_t Gripper::getPosition() {
    if (!_connected) return -1;
    return st.ReadPos(STS_ID);
}

int32_t Gripper::getTemperature() {
    if (!_connected) return -1;
    return st.ReadTemper(STS_ID);
}

int32_t Gripper::getLoad() {
    if (!_connected) return 0;
    return st.ReadLoad(STS_ID);
}

bool Gripper::isMoving() {
    if (!_connected) return false;
    return st.ReadMove(STS_ID) != 0;
}
