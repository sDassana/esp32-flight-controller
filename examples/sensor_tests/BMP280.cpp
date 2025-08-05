#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> // BMP280, not BME280

Adafruit_BMP280 bmp; // Use I2C interface

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("BMP280 Test");

    // Initialize sensor (default I2C address is 0x76 or 0x77)
    if (!bmp.begin(0x76))
    { // Try 0x77 if this fails
        Serial.println("❌ BMP280 not found. Check wiring or I2C address!");
        while (1)
            delay(500);
    }

    Serial.println("✅ BMP280 initialized!");

    // Optional: Configure settings
    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,   // Temp oversampling
        Adafruit_BMP280::SAMPLING_X16,  // Pressure oversampling
        Adafruit_BMP280::FILTER_X16,    // Filtering
        Adafruit_BMP280::STANDBY_MS_500 // Standby time
    );
}

void loop()
{
    Serial.println("========== BMP280 Readings ==========");
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure() / 100.0F); // hPa
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bmp.readAltitude(1013.25)); // Sea level pressure in hPa
    Serial.println(" m");

    Serial.println();
    delay(1000);
}
