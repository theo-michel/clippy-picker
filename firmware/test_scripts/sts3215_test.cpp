#include <Arduino.h>
#include <SCServo.h>

#define STS_RX 16
#define STS_TX 17
#define STS_BAUD 1000000
#define SCAN_MAX_ID 10
#define MOVE_SPEED 2000
#define MOVE_ACC 200
#define MOVE_DWELL_MS 2000

SMS_STS st;

static uint8_t ids[SCAN_MAX_ID];
static int16_t posMin[SCAN_MAX_ID];
static int16_t posMax[SCAN_MAX_ID];
static uint8_t nFound = 0;
static bool recording = true;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== STS3215 Joint Limit Finder ===");

    Serial2.begin(STS_BAUD, SERIAL_8N1, STS_RX, STS_TX);
    st.pSerial = &Serial2;
    delay(100);

    Serial.println("Scanning IDs 1-10...");
    for (uint8_t id = 1; id <= SCAN_MAX_ID && nFound < SCAN_MAX_ID; id++)
    {
        int model = st.Ping(id);
        if (model != -1)
        {
            int16_t pos = st.ReadPos(id);
            Serial.printf("  ID %d  model=%d  pos=%d\n", id, model, pos);
            ids[nFound] = id;
            posMin[nFound] = posMax[nFound] = pos;
            st.EnableTorque(id, 0);
            nFound++;
        }
    }

    if (nFound == 0)
    {
        Serial.println("No servos found!");
        while (true)
            delay(1000);
    }

    Serial.printf("Found %d servo(s), torque off.\n", nFound);
    Serial.println("Move joints to limits, then press 'd' when done.\n");
}

void loop()
{
    if (recording)
    {
        if (Serial.available() && Serial.read() == 'd')
        {
            recording = false;
            Serial.println("\n--- Recorded limits ---");
            for (uint8_t i = 0; i < nFound; i++)
                Serial.printf("  ID %d  min=%-5d  max=%-5d  range=%d\n",
                              ids[i], posMin[i], posMax[i], posMax[i] - posMin[i]);
            Serial.println("\nMoving in 2 seconds...");
            delay(2000);
            for (uint8_t i = 0; i < nFound; i++)
                st.EnableTorque(ids[i], 1);
            return;
        }

        for (uint8_t i = 0; i < nFound; i++)
        {
            int16_t p = st.ReadPos(ids[i]);
            if (p == -1)
                continue;
            if (p < posMin[i])
                posMin[i] = p;
            if (p > posMax[i])
                posMax[i] = p;
            Serial.printf("[ID %d] pos=%-5d  min=%-5d  max=%-5d\n",
                          ids[i], p, posMin[i], posMax[i]);
        }
        Serial.println();
        delay(200);
    }
    else
    {
        int16_t maxRange = 0;
        for (uint8_t i = 0; i < nFound; i++)
            maxRange = max(maxRange, (int16_t)(posMax[i] - posMin[i]));
        uint32_t dwell = max((uint32_t)MOVE_DWELL_MS,
                             (uint32_t)(maxRange * 1000UL / MOVE_SPEED) + 500);

        for (uint8_t i = 0; i < nFound; i++)
            st.WritePosEx(ids[i], posMin[i], MOVE_SPEED, MOVE_ACC);
        delay(dwell);

        for (uint8_t i = 0; i < nFound; i++)
            st.WritePosEx(ids[i], posMax[i], MOVE_SPEED, MOVE_ACC);
        delay(dwell);
    }
}
