#include <Arduino.h>
#include <WiFi.h>

#ifndef WIFI_SSID
#error "WIFI_SSID not defined. Create firmware/secrets.ini from secrets.ini.example"
#endif
#ifndef WIFI_PASS
#error "WIFI_PASS not defined. Create firmware/secrets.ini from secrets.ini.example"
#endif

const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\nConnected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.println(WiFi.RSSI());
}

void loop()
{
    // Add a periodic ping or keep-alive check here if desired
}