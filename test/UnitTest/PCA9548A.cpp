#include <Wire.h>

#define TCAADDR 0x70

void tcaselect(uint8_t i)
{
    if (i > 7)
        return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(21, 22); // Your SDA, SCL

    for (uint8_t i = 0; i < 4; i++)
    {
        Serial.print("Selecting channel ");
        Serial.println(i);
        tcaselect(i);
        delay(500);
    }

    Serial.println("PCA9548A appears responsive.");
}

void loop() {}
