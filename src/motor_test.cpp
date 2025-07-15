#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <BH1750.h>
#include <TinyGPS++.h>

// NRF24L01 Pins
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5
#define NRF_SCK_PIN 18
#define NRF_MOSI_PIN 23
#define NRF_MISO_PIN 19

// ESC PWM Pins
#define ESC1_PIN 13
#define ESC2_PIN 12
#define ESC3_PIN 14
#define ESC4_PIN 27

// RGB LED Pins for status
#define LED_R1_PIN 25
#define LED_G1_PIN 26
#define LED_B1_PIN 32

// I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// Sensor Pins
#define GUVA_PIN 36   // UV sensor analog pin
#define GPS_RX_PIN 16 // GPS TX connected to ESP32 RX

// Sensor Addresses
#define MPU6050_ADDR 0x68
#define BME280_ADDR 0x76
#define CCS811_ADDR 0x5A
#define BH1750_ADDR 0x23

// Objects
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo esc1, esc2, esc3, esc4;

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
Adafruit_CCS811 ccs;
BH1750 lightMeter;
TinyGPSPlus gps;

// Joystick/Remote data structure
struct JoystickData
{
    uint16_t throttle; // 0-1023
    int16_t yaw;       // -512 to 512
    int16_t pitch;     // -512 to 512
    int16_t roll;      // -512 to 512
    bool btn1;         // Button 1 (emergency stop)
    bool btn2;         // Button 2 (arm/disarm)
};

// ACK Payload Structure (sent from drone to remote)
struct AckPayload
{
    float altitude;  // in meters
    float latitude;  // decimal degrees
    float longitude; // decimal degrees

    uint8_t sensorType; // 0 = MPU6050, 1 = UV/Light, 2 = Air Quality

    union
    {
        struct
        {
            float ax, ay, az;
            float gx, gy, gz;
        } mpu;
        struct
        {
            float uv, lux;
        } light;
        struct
        {
            float tvoc, co2;
        } air;
    } data;
};

JoystickData joyData = {0, 0, 0, 0, false, false};

// Round-robin sensor rotation
uint8_t currentSensor = 0; // 0 = BME280, 1 = UV/Light, 2 = Air Quality
unsigned long lastSensorSwitch = 0;
const unsigned long sensorSwitchInterval = 1000; // Switch every 1 second

// Real sensor data
float altitude = 0.0;
float latitude = 0.0;
float longitude = 0.0;
float temperature = 0.0;
float humidity = 0.0;
float ax = 0.0, ay = 0.0, az = 0.0;
float gx = 0.0, gy = 0.0, gz = 0.0;
float uv = 0.0, lux = 0.0;
float tvoc = 0.0, co2 = 0.0;

// Sensor status flags
bool mpuReady = false;
bool bmeReady = false;
bool ccsReady = false;
bool lightReady = false;
bool gpsReady = false;

// ESC parameters
const int pwmMin = 1000; // Min pulse width (us)
const int pwmMax = 2000; // Max pulse width (us)

// NRF24L01 communication settings
const byte address[6] = "00001";

// ESC arming state machine
enum ESCState
{
    ESC_ARM_MAX,
    ESC_ARM_MIN,
    ESC_READY,
    ESC_DISARMED
};
ESCState escState = ESC_ARM_MAX;
unsigned long escArmStart = 0;

void setLEDColor(int r, int g, int b)
{
    digitalWrite(LED_R1_PIN, r);
    digitalWrite(LED_G1_PIN, g);
    digitalWrite(LED_B1_PIN, b);
}

AckPayload prepareAckPayload()
{
    AckPayload ack;

    // Always include altitude and GPS
    ack.altitude = altitude;
    ack.latitude = latitude;
    ack.longitude = longitude;

    // Round-robin sensor selection
    ack.sensorType = currentSensor;

    switch (currentSensor)
    {
    case 0:                            // BME280 temperature and humidity
        ack.data.mpu.ax = temperature; // Reuse mpu struct for temp/humidity
        ack.data.mpu.ay = humidity;
        ack.data.mpu.az = 0.0;
        ack.data.mpu.gx = 0.0;
        ack.data.mpu.gy = 0.0;
        ack.data.mpu.gz = 0.0;
        break;

    case 1: // UV/Light data
        ack.data.light.uv = uv;
        ack.data.light.lux = lux;
        break;

    case 2: // Air Quality data
        ack.data.air.tvoc = tvoc;
        ack.data.air.co2 = co2;
        break;
    }

    return ack;
}

void readSensors()
{
    // Read BME280 (altitude, temperature, and humidity)
    if (bmeReady)
    {
        float newAltitude = bme.readAltitude(1013.25); // Sea level pressure
        float newTemperature = bme.readTemperature();
        float newHumidity = bme.readHumidity();

        // Check if readings are valid
        if (!isnan(newAltitude) && !isnan(newTemperature) && !isnan(newHumidity))
        {
            altitude = newAltitude;
            temperature = newTemperature;
            humidity = newHumidity;
        }
        else
        {
            Serial.println("BME280 reading failed - NaN values");
        }
    }
    else
    {
        // Set default values if sensor not ready
        altitude = 0.0;
        temperature = 0.0;
        humidity = 45.0 + sin(millis() / 10000.0) * 10.0; // Fallback simulated humidity
    }

    // Read MPU6050 (IMU data)
    if (mpuReady)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        ax = a.acceleration.x;
        ay = a.acceleration.y;
        az = a.acceleration.z;
        gx = g.gyro.x;
        gy = g.gyro.y;
        gz = g.gyro.z;
    }

    // Read GPS data
    while (Serial2.available())
    {
        if (gps.encode(Serial2.read()))
        {
            if (gps.location.isValid())
            {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                gpsReady = true;
            }
        }
    }

    // Read UV sensor (GUVA-S12SD on analog pin)
    int uvRaw = analogRead(GUVA_PIN);
    uv = (uvRaw * 3.3 / 4095.0) * 10.0; // Convert to UV index approximation

    // Read BH1750 (light sensor)
    if (lightReady)
    {
        lux = lightMeter.readLightLevel();
        if (isnan(lux))
        {
            lux = 0.0;
        }
    }

    // Read CCS811 (air quality)
    if (ccsReady && ccs.available())
    {
        if (!ccs.readData())
        {
            tvoc = ccs.getTVOC();
            co2 = ccs.geteCO2();
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32 Motor Test with Real Sensors - Starting...");

    // RGB LEDs
    pinMode(LED_R1_PIN, OUTPUT);
    pinMode(LED_G1_PIN, OUTPUT);
    pinMode(LED_B1_PIN, OUTPUT);

    // Set LED to RED during initialization
    setLEDColor(HIGH, LOW, LOW);

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("I2C initialized");

    // Initialize sensors
    Serial.println("Initializing sensors...");

    // MPU6050 Init
    if (mpu.begin(MPU6050_ADDR))
    {
        Serial.println("MPU6050 initialized successfully");
        mpuReady = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
    else
    {
        Serial.println("MPU6050 initialization failed!");
    }

    // BME280 Init
    if (bme.begin(BME280_ADDR))
    {
        Serial.println("BME280 initialized successfully");
        bmeReady = true;
        bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                        Adafruit_BME280::SAMPLING_X2,  // temperature
                        Adafruit_BME280::SAMPLING_X16, // pressure
                        Adafruit_BME280::SAMPLING_X1,  // humidity
                        Adafruit_BME280::FILTER_X16,
                        Adafruit_BME280::STANDBY_MS_500);
    }
    else
    {
        Serial.println("BME280 initialization failed!");
        // Try alternative address
        if (bme.begin(0x77))
        {
            Serial.println("BME280 initialized on alternative address 0x77");
            bmeReady = true;
            bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                            Adafruit_BME280::SAMPLING_X2,  // temperature
                            Adafruit_BME280::SAMPLING_X16, // pressure
                            Adafruit_BME280::SAMPLING_X1,  // humidity
                            Adafruit_BME280::FILTER_X16,
                            Adafruit_BME280::STANDBY_MS_500);
        }
        else
        {
            Serial.println("BME280 failed on both addresses (0x76 and 0x77)");
        }
    }

    // CCS811 Init
    if (ccs.begin(CCS811_ADDR))
    {
        Serial.println("CCS811 initialized successfully");
        ccsReady = true;
        // Wait for the sensor to be ready
        while (!ccs.available())
            delay(10);
    }
    else
    {
        Serial.println("CCS811 initialization failed!");
    }

    // BH1750 Init
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, BH1750_ADDR))
    {
        Serial.println("BH1750 initialized successfully");
        lightReady = true;
    }
    else
    {
        Serial.println("BH1750 initialization failed!");
    }

    // GPS Init
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, -1); // RX on GPIO 16, no TX
    Serial.println("GPS initialized on Serial2");

    // Initialize UV sensor pin
    pinMode(GUVA_PIN, INPUT);
    Serial.println("UV sensor initialized");

    // SPI Init
    SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);

    // NRF24L01 Init
    if (!radio.begin())
    {
        Serial.println("NRF24L01 not found!");
        setLEDColor(HIGH, LOW, HIGH); // Purple = NRF error
        while (1)
            delay(1000);
    }

    radio.openReadingPipe(0, address);
    radio.setChannel(76); // Channel 76 to avoid WiFi interference
    radio.setPALevel(RF24_PA_LOW);
    radio.enableAckPayload(); // Enable ACK payload feature
    radio.startListening();

    Serial.println("NRF24L01 initialized on channel 76 with ACK payload enabled");

    // Print sensor status
    Serial.println("=== SENSOR STATUS ===");
    Serial.printf("MPU6050: %s\n", mpuReady ? "OK" : "FAILED");
    Serial.printf("BME280: %s\n", bmeReady ? "OK" : "FAILED");
    Serial.printf("CCS811: %s\n", ccsReady ? "OK" : "FAILED");
    Serial.printf("BH1750: %s\n", lightReady ? "OK" : "FAILED");
    Serial.printf("GPS: %s\n", "INITIALIZED");
    Serial.printf("UV Sensor: %s\n", "OK");
    Serial.println("====================");

    // Test BME280 reading if available
    if (bmeReady)
    {
        float testAlt = bme.readAltitude(1013.25);
        float testTemp = bme.readTemperature();
        float testHum = bme.readHumidity();
        Serial.printf("BME280 Test - Temp: %.2f°C, Alt: %.2f m, Humidity: %.2f%%\n", testTemp, testAlt, testHum);
    }

    // ESC PWM setup (Servo)
    esc1.attach(ESC1_PIN, pwmMin, pwmMax);
    esc2.attach(ESC2_PIN, pwmMin, pwmMax);
    esc3.attach(ESC3_PIN, pwmMin, pwmMax);
    esc4.attach(ESC4_PIN, pwmMin, pwmMax);

    // Initialize ESCs to minimum
    esc1.writeMicroseconds(pwmMin);
    esc2.writeMicroseconds(pwmMin);
    esc3.writeMicroseconds(pwmMin);
    esc4.writeMicroseconds(pwmMin);

    Serial.println("ESCs initialized");

    // Set LED to YELLOW during calibration
    setLEDColor(HIGH, HIGH, LOW);

    escArmStart = millis();
    Serial.println("Starting ESC calibration sequence...");
}

void loop()
{
    // Read all sensors
    readSensors();

    // Handle round-robin sensor switching
    if (millis() - lastSensorSwitch > sensorSwitchInterval)
    {
        currentSensor = (currentSensor + 1) % 3; // Cycle through 0, 1, 2
        lastSensorSwitch = millis();

        const char *sensorNames[] = {"BME280", "UV/Light", "Air Quality"};
        Serial.printf("Switching to sensor: %s\n", sensorNames[currentSensor]);
    }

    // 1. Receive control signals (NRF24L01) and send ACK payload
    if (radio.available())
    {
        JoystickData rxData;
        radio.read(&rxData, sizeof(rxData));
        joyData = rxData;

        // Prepare and send ACK payload with telemetry data
        AckPayload ackPayload = prepareAckPayload();
        radio.writeAckPayload(0, &ackPayload, sizeof(ackPayload));

        // Debug output
        Serial.printf("RX: T:%d Y:%d P:%d R:%d B1:%d B2:%d | TX: Alt:%.1f Lat:%.6f Lng:%.6f Type:%d\n",
                      joyData.throttle, joyData.yaw, joyData.pitch, joyData.roll,
                      joyData.btn1, joyData.btn2,
                      ackPayload.altitude, ackPayload.latitude, ackPayload.longitude,
                      ackPayload.sensorType);
    }

    // 2. ESC arming state machine
    switch (escState)
    {
    case ESC_ARM_MAX:
        // Send max throttle for calibration
        esc1.writeMicroseconds(pwmMax);
        esc2.writeMicroseconds(pwmMax);
        esc3.writeMicroseconds(pwmMax);
        esc4.writeMicroseconds(pwmMax);

        setLEDColor(HIGH, HIGH, LOW); // Yellow = Calibrating MAX

        if (millis() - escArmStart > 2000) // 2 seconds as per README
        {
            escState = ESC_ARM_MIN;
            escArmStart = millis();
            Serial.println("ESC calibration: Switching to MIN throttle");
        }
        break;

    case ESC_ARM_MIN:
        // Send min throttle for calibration
        esc1.writeMicroseconds(pwmMin);
        esc2.writeMicroseconds(pwmMin);
        esc3.writeMicroseconds(pwmMin);
        esc4.writeMicroseconds(pwmMin);

        setLEDColor(HIGH, LOW, HIGH); // Purple = Calibrating MIN

        if (millis() - escArmStart > 1000) // 1 second
        {
            escState = ESC_READY;
            Serial.println("ESC calibration complete - Ready for control");
        }
        break;

    case ESC_READY:
    {
        // Normal operation: map joystick throttle to ESCs
        setLEDColor(LOW, HIGH, LOW); // Green = Ready

        // Emergency stop
        if (joyData.btn1)
        {
            escState = ESC_DISARMED;
            Serial.println("EMERGENCY STOP!");
            break;
        }

        // Motor control - all motors get same throttle for now (no mixing)
        uint16_t esc_pwm = map(joyData.throttle, 0, 1023, pwmMin, pwmMax);
        esc1.writeMicroseconds(esc_pwm);
        esc2.writeMicroseconds(esc_pwm);
        esc3.writeMicroseconds(esc_pwm);
        esc4.writeMicroseconds(esc_pwm);
        break;
    }

    case ESC_DISARMED:
        // Send min throttle (motors off)
        esc1.writeMicroseconds(pwmMin);
        esc2.writeMicroseconds(pwmMin);
        esc3.writeMicroseconds(pwmMin);
        esc4.writeMicroseconds(pwmMin);

        setLEDColor(HIGH, LOW, LOW); // Red = Disarmed

        // Re-arm with button 2
        if (joyData.btn2)
        {
            escState = ESC_READY;
            Serial.println("Re-armed");
        }
        break;
    }

    delay(20); // 50Hz update rate
}
