#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10); // Wait for Serial

    Serial.println("MPU6050 Test");

    // Initialize I2C (ESP32 default: SDA = 21, SCL = 22)
    if (!mpu.begin())
    {
        Serial.println("❌ Failed to find MPU6050 chip. Check wiring!");
        while (1)
        {
            delay(500);
        }
    }

    Serial.println("✅ MPU6050 Found!");

    // Set up sensor ranges
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(100);
}

void loop()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.println("========== Sensor Readings ==========");
    Serial.print("Accel X: ");
    Serial.print(a.acceleration.x);
    Serial.print(" m/s², ");
    Serial.print("Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(" m/s², ");
    Serial.print("Z: ");
    Serial.println(a.acceleration.z);
    Serial.println(" m/s²");

    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    Serial.print(" rad/s, ");
    Serial.print("Y: ");
    Serial.print(g.gyro.y);
    Serial.print(" rad/s, ");
    Serial.print("Z: ");
    Serial.println(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temp: ");
    Serial.print(temp.temperature);
    Serial.println(" °C");

    delay(500);
}
