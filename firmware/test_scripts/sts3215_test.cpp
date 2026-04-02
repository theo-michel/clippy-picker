#include <Arduino.h>
#include <SCServo.h>

#define STS_RX 16
#define STS_TX 17
#define STS_BAUD 1000000
#define SCAN_MAX_ID 10
#define READ_INTERVAL_MS 200

SMS_STS st;

static uint8_t ids[SCAN_MAX_ID];
static int16_t posMin[SCAN_MAX_ID];
static int16_t posMax[SCAN_MAX_ID];
static uint8_t nFound = 0;
static bool torque = false;

void scan() {
    nFound = 0;
    Serial.println("Scanning IDs 1-10...");
    for (uint8_t id = 1; id <= SCAN_MAX_ID && nFound < SCAN_MAX_ID; id++) {
        int model = st.Ping(id);
        if (model != -1) {
            ids[nFound] = id;
            int16_t pos = st.ReadPos(id);
            int volt = st.ReadVoltage(id);
            int temp = st.ReadTemper(id);
            posMin[nFound] = pos;
            posMax[nFound] = pos;
            Serial.printf("  ID %d  model=%d  pos=%d  %.1fV  %d°C\n",
                          id, model, pos, volt / 10.0f, temp);
            nFound++;
        }
    }
    Serial.printf("Found %d servo(s)\n\n", nFound);
}

void setTorque(bool on) {
    torque = on;
    for (uint8_t i = 0; i < nFound; i++)
        st.EnableTorque(ids[i], on ? 1 : 0);
    Serial.printf("Torque %s\n", on ? "ON" : "OFF");
}

void resetMinMax() {
    for (uint8_t i = 0; i < nFound; i++) {
        int16_t p = st.ReadPos(ids[i]);
        posMin[i] = posMax[i] = p;
    }
    Serial.println("Min/max reset");
}

void moveCenter() {
    setTorque(true);
    for (uint8_t i = 0; i < nFound; i++)
        st.WritePosEx(ids[i], 2048, 200, 50);
    Serial.println("Moving to center (2048)");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== STS3215 Joint Limit Finder ===");

    Serial2.begin(STS_BAUD, SERIAL_8N1, STS_RX, STS_TX);
    st.pSerial = &Serial2;
    delay(100);

    scan();
    if (nFound > 0) setTorque(false);

    Serial.println("Commands: r=reset min/max  t=toggle torque  c=center  s=rescan");
    Serial.println("Move servos by hand to find limits.\n");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'r': resetMinMax(); break;
            case 't': setTorque(!torque); break;
            case 'c': moveCenter(); break;
            case 's': scan(); break;
        }
    }

    for (uint8_t i = 0; i < nFound; i++) {
        int16_t p = st.ReadPos(ids[i]);
        if (p == -1) continue;
        if (p < posMin[i]) posMin[i] = p;
        if (p > posMax[i]) posMax[i] = p;

        Serial.printf("[ID %d] pos=%-5d  min=%-5d  max=%-5d  range=%-5d  (%.1f deg)\n",
                      ids[i], p, posMin[i], posMax[i],
                      posMax[i] - posMin[i],
                      p * 360.0f / 4096.0f);
    }
    if (nFound > 0) Serial.println();

    delay(READ_INTERVAL_MS);
}
