#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

// Create sensor instances
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp; // I2C interface

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("Initializing sensors...");

    // MPU6050 initialization
    if (!mpu.begin())
    {
        Serial.println("❌ Could not find MPU6050. Check wiring!");
        while (1)
            delay(500);
    }
    Serial.println("✅ MPU6050 found");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(100);

    // BMP280 initialization
    if (!bmp.begin(0x76))
    { // Use 0x77 if your BMP280 is configured that way
        Serial.println("❌ Could not find BMP280. Check wiring or I2C address!");
        while (1)
            delay(500);
    }
    Serial.println("✅ BMP280 found");

    bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,
        Adafruit_BMP280::SAMPLING_X16,
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_500);

    Serial.println("All sensors initialized.\n");
}

void loop()
{
    sensors_event_t a, g, temp_mpu;
    mpu.getEvent(&a, &g, &temp_mpu);

    float temp_bmp = bmp.readTemperature();
    float pressure = bmp.readPressure() / 100.0F; // hPa
    float altitude = bmp.readAltitude(1013.25);   // Sea level pressure

    Serial.println("========== Combined Sensor Readings ==========");

    // MPU6050
    Serial.print("MPU6050 Accel (m/s²) -> X: ");
    Serial.print(a.acceleration.x);
    Serial.print(" Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(" Z: ");
    Serial.println(a.acceleration.z);

    Serial.print("MPU6050 Gyro (rad/s) -> X: ");
    Serial.print(g.gyro.x);
    Serial.print(" Y: ");
    Serial.print(g.gyro.y);
    Serial.print(" Z: ");
    Serial.println(g.gyro.z);

    Serial.print("MPU6050 Temp: ");
    Serial.print(temp_mpu.temperature);
    Serial.println(" °C");

    // BMP280
    Serial.print("BMP280 Temp: ");
    Serial.print(temp_bmp);
    Serial.println(" °C");

    Serial.print("BMP280 Pressure: ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("BMP280 Altitude: ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.println();
    delay(1000);
}
