/*
 * Drone with Enhanced Sensor Suite
 * Complete telemetry system with BME280, AHT21, GPS, and simulated advanced sensors
 * Compatible with remote control system (12-byte control, 22-byte telemetry)
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPS++.h>
#include <BH1750.h>
#include <ScioSense_ENS160.h>

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5
#define BATTERY_PIN 35
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GUVA_PIN 36 // GUVA-S12SD UV sensor

// I2C addresses
#define BME280_ADDRESS 0x76
#define BH1750_ADDRESS 0x23
#define ENS160_ADDRESS 0x53

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Initialize real sensors
Adafruit_BME280 bme280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;
BH1750 lightMeter;
ScioSense_ENS160 ens160;

// Control packet (12 bytes) - Added button states and toggle switches to match remote
struct ControlPacket
{
    int16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    uint8_t joy1_btn; // Joystick 1 button state (0 = pressed, 1 = released)
    uint8_t joy2_btn; // Joystick 2 button state (0 = pressed, 1 = released)
    uint8_t toggle1;  // Toggle switch 1 state (0 = off, 1 = on)
    uint8_t toggle2;  // Toggle switch 2 state (0 = off, 1 = on)
};

// Enhanced telemetry packet (22 bytes) - matches remote with lux, altitude, UV index, eCO2, and TVOC
struct TelemetryPacket
{
    int16_t temperature; // x100 - Real BME280 data
    uint16_t pressure;   // x10 - Real BME280 data
    uint8_t humidity;    // % - Real AHT21 data
    uint16_t battery;    // mV - Real battery voltage
    int16_t latitude;    // GPS latitude (simplified)
    int16_t longitude;   // GPS longitude (simplified)
    uint8_t satellites;  // GPS satellite count
    uint8_t status;      // System status
    uint16_t lux;        // Light level in lux
    int16_t altitude;    // Altitude in centimeters from BME280 (for 2 decimal precision)
    uint16_t uvIndex;    // UV index x100 from GUVA sensor
    uint16_t eCO2;       // Equivalent CO2 in ppm from ENS160
    uint16_t TVOC;       // Total VOC in ppb from ENS160
};

ControlPacket receivedControl;
TelemetryPacket telemetryData;
unsigned long lastControlReceived = 0;
unsigned long lastSensorRead = 0;
unsigned long lastGPSRead = 0;
int packetsReceived = 0;
bool sensorsInitialized = false;
bool bh1750Ready = false;
bool ens160Ready = false;

// Altitude calibration variables
#define PRESSURE_BUFFER_SIZE 5    // Smaller buffer for faster response
float seaLevelPressure = 1013.25; // Will be calibrated at startup
bool altitudeCalibrated = false;
float pressureReadings[PRESSURE_BUFFER_SIZE]; // For smoothing pressure readings
int pressureIndex = 0;
int pressureCount = 0;

void setup()
{
    Serial.begin(115200);
    Serial.println("=== Drone with Real Sensors ===");

    // Initialize I2C for sensors
    Wire.begin(21, 22); // SDA=21, SCL=22
    delay(100);

    // Initialize pins
    pinMode(BATTERY_PIN, INPUT);
    pinMode(GUVA_PIN, INPUT); // UV sensor analog input

    // Initialize GPS (Serial2 on ESP32)
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // Initialize real sensors
    initializeRealSensors();

    // Initialize radio with PROVEN configuration
    SPI.begin();
    delay(100);

    if (!radio.begin() || !radio.isChipConnected())
    {
        Serial.println("ERROR: Radio initialization failed!");
        while (1)
            delay(1000);
    }

    // Use EXACT same configuration as remote
    radio.openReadingPipe(1, address);
    radio.setPALevel(RF24_PA_HIGH);  // Match remote power level
    radio.setDataRate(RF24_250KBPS); // Match remote data rate
    radio.setChannel(76);
    radio.setAutoAck(true);
    radio.setRetries(15, 15);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    // Additional range improvements to match remote
    radio.setCRCLength(RF24_CRC_16); // Use 16-bit CRC for better error detection
    radio.setAddressWidth(5);        // Use full 5-byte addresses

    // Start listening for control data
    radio.startListening();

    Serial.println("Radio configured successfully!");
    Serial.println("Waiting for control data...");
}

void loop()
{
    // Read sensors every 1 second for faster response
    if (millis() - lastSensorRead > 1000)
    {
        readRealSensors();
        lastSensorRead = millis();
    }

    // Update GPS every 100ms
    if (millis() - lastGPSRead > 100)
    {
        updateGPS();
        lastGPSRead = millis();
    }

    // Check for incoming control data
    if (radio.available())
    {
        handleControlData();
    }

    // Print status every 10 seconds
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 10000)
    {
        printStatus();
        lastStatusPrint = millis();
    }

    delay(10);
}

void initializeRealSensors()
{
    Serial.println("Initializing real sensors...");

    // Scan I2C bus first
    Serial.println("Scanning I2C bus...");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found");
    }

    // Try BME280 at common addresses
    bool bme280Found = false;
    uint8_t bmeAddresses[] = {0x76, 0x77};
    for (int i = 0; i < 2; i++)
    {
        if (bme280.begin(bmeAddresses[i]))
        {
            Serial.print("✓ BME280 initialized at address 0x");
            Serial.println(bmeAddresses[i], HEX);

            // Configure BME280 for stable readings
            bme280.setSampling(Adafruit_BME280::MODE_FORCED,      // Use forced mode for manual readings
                               Adafruit_BME280::SAMPLING_X16,     // Temperature oversampling
                               Adafruit_BME280::SAMPLING_X16,     // Pressure oversampling
                               Adafruit_BME280::SAMPLING_X16,     // Humidity oversampling
                               Adafruit_BME280::FILTER_X16,       // IIR filter
                               Adafruit_BME280::STANDBY_MS_1000); // 1 second standby

            sensorsInitialized = true;
            bme280Found = true;
            break;
        }
    }

    if (!bme280Found)
    {
        Serial.println("✗ BME280 initialization failed - using simulated data");
    }

    // Initialize AHT21
    if (aht.begin())
    {
        Serial.println("✓ AHT21 initialized");
    }
    else
    {
        Serial.println("✗ AHT21 initialization failed - using simulated data");
    }

    // Initialize BH1750 light sensor
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
    {
        Serial.println("✓ BH1750 light sensor initialized");
        bh1750Ready = true;
    }
    else
    {
        Serial.println("✗ BH1750 initialization failed - using simulated data");
        bh1750Ready = false;
    }

    // Initialize ENS160 air quality sensor
    if (ens160.begin())
    {
        Serial.println("✓ ENS160 air quality sensor initialized");
        ens160.setMode(ENS160_OPMODE_STD);
        ens160Ready = true;
    }
    else
    {
        Serial.println("✗ ENS160 initialization failed - using simulated data");
        ens160Ready = false;
    }

    Serial.println("✓ GPS Serial2 initialized");
    Serial.println("✓ GUVA-S12SD UV sensor on analog pin");

    // Calibrate altitude (assume current location is reference point)
    calibrateAltitude();

    Serial.println("Sensor initialization complete");
}

void calibrateAltitude()
{
    if (!sensorsInitialized)
    {
        Serial.println("⚠ BME280 not available - altitude will use default sea level pressure");
        return;
    }

    Serial.println("Calibrating altitude reference...");

    // Wait for sensor to stabilize
    delay(2000);

    // Take multiple pressure readings for calibration
    float pressureSum = 0;
    int validReadings = 0;

    for (int i = 0; i < 30; i++)
    {
        // Force a reading and wait
        bme280.takeForcedMeasurement();
        delay(100);

        float pressure = bme280.readPressure() / 100.0; // Convert Pa to hPa
        if (pressure > 800 && pressure < 1200)          // Reasonable pressure range
        {
            pressureSum += pressure;
            validReadings++;
            Serial.print(".");
        }
        delay(100);
    }
    Serial.println();

    if (validReadings > 20)
    {
        seaLevelPressure = pressureSum / validReadings;
        altitudeCalibrated = true;

        Serial.print("✓ Altitude calibrated - Reference pressure: ");
        Serial.print(seaLevelPressure);
        Serial.println(" hPa (current location = 0m)");
    }
    else
    {
        Serial.println("✗ Altitude calibration failed - using standard sea level pressure");
    }
}

void readRealSensors()
{
    // Read BME280 (temperature and pressure)
    if (sensorsInitialized)
    {
        // Force a measurement for consistent timing
        bme280.takeForcedMeasurement();

        float temp = bme280.readTemperature();
        float currentPressure = bme280.readPressure() / 100.0; // Convert Pa to hPa

        telemetryData.temperature = (int16_t)(temp * 100);
        telemetryData.pressure = (uint16_t)(currentPressure * 10);

        // Validate pressure reading
        if (currentPressure > 800 && currentPressure < 1200)
        {
            // Add to pressure buffer for simple smoothing
            pressureReadings[pressureIndex] = currentPressure;
            pressureIndex = (pressureIndex + 1) % PRESSURE_BUFFER_SIZE;
            if (pressureCount < PRESSURE_BUFFER_SIZE)
                pressureCount++;

            // Calculate simple moving average
            float pressureSum = 0;
            for (int i = 0; i < pressureCount; i++)
            {
                pressureSum += pressureReadings[i];
            }
            float smoothedPressure = pressureSum / pressureCount;

            // Calculate altitude using smoothed pressure
            float altitude;
            if (altitudeCalibrated)
            {
                // Use calibrated reference for relative altitude
                altitude = 44330.0 * (1.0 - pow(smoothedPressure / seaLevelPressure, 0.1903));
            }
            else
            {
                // Use standard sea level pressure
                altitude = 44330.0 * (1.0 - pow(smoothedPressure / 1013.25, 0.1903));
            }

            // Debug output for pressure and altitude calculation
            static unsigned long lastDebug = 0;
            if (millis() - lastDebug > 5000) // Debug every 5 seconds
            {
                Serial.print("DEBUG - Current P: ");
                Serial.print(currentPressure, 2);
                Serial.print(" hPa, Smoothed P: ");
                Serial.print(smoothedPressure, 2);
                Serial.print(" hPa, Ref P: ");
                Serial.print(seaLevelPressure, 2);
                Serial.print(" hPa, Raw Alt: ");
                Serial.print(altitude, 2);
                Serial.println(" m");
                lastDebug = millis();
            }

            // Simple range limiting
            if (altitude < -500)
                altitude = -500;
            if (altitude > 5000)
                altitude = 5000;

            // Store altitude in centimeters for 2 decimal precision
            telemetryData.altitude = (int16_t)(altitude * 100); // Convert meters to centimeters
        }
        else
        {
            // Invalid pressure reading - keep last altitude
            Serial.println("Invalid pressure reading");
        }
    }
    else
    {
        // Fallback to simulated data
        static float temp = 25.0;
        temp += (random(-10, 11) / 100.0);
        telemetryData.temperature = (int16_t)(temp * 100);

        static float pressure = 1013.2;
        pressure += (random(-5, 6) / 10.0);
        telemetryData.pressure = (uint16_t)(pressure * 10);

        telemetryData.altitude = 10000 + random(-2000, 2001); // Simulated altitude in cm
    }

    // Read AHT21 (humidity and temperature)
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp))
    {
        telemetryData.humidity = (uint8_t)humidity.relative_humidity;
        // Use AHT21 temperature if BME280 failed
        if (!sensorsInitialized)
        {
            telemetryData.temperature = (int16_t)(temp.temperature * 100);
        }
    }
    else
    {
        // Fallback to simulated humidity
        telemetryData.humidity = 60 + random(-10, 11);
    }

    // Read real battery voltage
    int batteryRaw = analogRead(BATTERY_PIN);
    if (batteryRaw < 100)
    {
        // Simulate battery if not connected
        telemetryData.battery = 3700 + random(-100, 101);
    }
    else
    {
        telemetryData.battery = map(batteryRaw, 0, 4095, 0, 5000);
    }

    // Read real light sensor (BH1750)
    if (bh1750Ready)
    {
        float lux = lightMeter.readLightLevel();
        if (lux >= 0)
        {
            telemetryData.lux = (uint16_t)lux;
        }
        else
        {
            telemetryData.lux = 500 + random(-100, 101); // Fallback simulated
        }
    }
    else
    {
        telemetryData.lux = 500 + random(-100, 101); // Simulated if sensor not ready
    }

    // Read real UV sensor (GUVA-S12SD)
    int uvRaw = analogRead(GUVA_PIN);
    // Convert to UV index (calibration may need adjustment based on your setup)
    float uvVoltage = (uvRaw / 4095.0) * 3.3;
    float uvIndex = uvVoltage / 0.1; // Approximate conversion for GUVA-S12SD
    if (uvIndex < 0)
        uvIndex = 0; // Clamp to positive values
    if (uvIndex > 15)
        uvIndex = 15; // Reasonable UV index maximum
    telemetryData.uvIndex = (uint16_t)(uvIndex * 100);

    // Read real air quality sensor (ENS160)
    if (ens160Ready && ens160.available())
    {
        ens160.measure();
        uint16_t eco2 = ens160.geteCO2();
        uint16_t tvoc = ens160.getTVOC();

        // Validate readings (ENS160 sometimes gives invalid values during warm-up)
        if (eco2 > 400 && eco2 < 5000) // Reasonable CO2 range
        {
            telemetryData.eCO2 = eco2;
        }
        else
        {
            telemetryData.eCO2 = 400 + random(-50, 51); // Fallback
        }

        if (tvoc < 1000) // Reasonable TVOC range
        {
            telemetryData.TVOC = tvoc;
        }
        else
        {
            telemetryData.TVOC = 50 + random(-20, 21); // Fallback
        }
    }
    else
    {
        // Fallback to simulated values
        telemetryData.eCO2 = 400 + random(-50, 51);
        telemetryData.TVOC = 50 + random(-20, 21);
    }

    // Update status
    telemetryData.status = 0x01; // All systems OK
    if (!sensorsInitialized)
        telemetryData.status |= 0x02; // Sensor warning
}

void updateGPS()
{
    // Read GPS data from Serial2
    while (Serial2.available() > 0)
    {
        if (gps.encode(Serial2.read()))
        {
            if (gps.location.isValid())
            {
                // Store simplified GPS coordinates (for 12-byte packet)
                telemetryData.latitude = (int16_t)(gps.location.lat() * 100);
                telemetryData.longitude = (int16_t)(gps.location.lng() * 100);
                telemetryData.satellites = gps.satellites.value();
            }
        }
    }

    // If no GPS fix, use default values
    if (!gps.location.isValid())
    {
        telemetryData.latitude = 0;
        telemetryData.longitude = 0;
        telemetryData.satellites = 0;
    }
}

void handleControlData()
{
    if (radio.available())
    {
        uint8_t len = radio.getDynamicPayloadSize();

        if (len == sizeof(receivedControl))
        {
            radio.read(&receivedControl, sizeof(receivedControl));
            lastControlReceived = millis();
            packetsReceived++;

            // Print received control data with real sensor info
            Serial.print("Control #");
            Serial.print(packetsReceived);
            Serial.print(" - T:");
            Serial.print(receivedControl.throttle);
            Serial.print(" R:");
            Serial.print(receivedControl.roll);
            Serial.print(" P:");
            Serial.print(receivedControl.pitch);
            Serial.print(" Y:");
            Serial.print(receivedControl.yaw);

            // Show button states when pressed
            if (receivedControl.joy1_btn == 0)
                Serial.print(" [JOY1-BTN]");
            if (receivedControl.joy2_btn == 0)
                Serial.print(" [JOY2-BTN]");

            // Show toggle switch states
            if (receivedControl.toggle1 == 1)
                Serial.print(" [SW1-ON]");
            if (receivedControl.toggle2 == 1)
                Serial.print(" [SW2-ON]");

            Serial.print(" | Sensors - Temp:");
            Serial.print(telemetryData.temperature / 100.0);
            Serial.print("°C Hum:");
            Serial.print(telemetryData.humidity);
            Serial.print("% Press:");
            Serial.print(telemetryData.pressure / 10.0);
            Serial.print("hPa Alt:");
            Serial.print(telemetryData.altitude / 100.0); // Display altitude in meters with 2 decimals
            Serial.print("m GPS:");
            Serial.print(telemetryData.satellites);
            Serial.print(" sats Lux:");
            Serial.print(telemetryData.lux);
            Serial.print("lx UV:");
            Serial.print(telemetryData.uvIndex / 100.0);
            Serial.print(" CO2:");
            Serial.print(telemetryData.eCO2);
            Serial.print("ppm TVOC:");
            Serial.print(telemetryData.TVOC);
            Serial.println("ppb");

            // Load telemetry data as ACK payload for NEXT packet
            radio.writeAckPayload(1, &telemetryData, sizeof(telemetryData));
        }
        else
        {
            Serial.print("Wrong packet size: ");
            Serial.println(len);
        }
    }
}

void printStatus()
{
    unsigned long uptime = millis() / 1000;
    bool controlActive = (millis() - lastControlReceived) < 3000;

    Serial.print("Drone status - Running ");
    Serial.print(uptime);
    Serial.print("s, Control packets: ");
    Serial.print(packetsReceived);
    Serial.print(", Active: ");
    Serial.print(controlActive ? "YES" : "NO");
    Serial.print(", Sensors: Temp:");
    Serial.print(telemetryData.temperature / 100.0);
    Serial.print("°C Hum:");
    Serial.print(telemetryData.humidity);
    Serial.print("% Press:");
    Serial.print(telemetryData.pressure / 10.0);
    Serial.print("hPa Alt:");
    Serial.print(telemetryData.altitude / 100.0); // Display altitude in meters with 2 decimals
    Serial.print("m GPS:");
    Serial.print(telemetryData.satellites);
    Serial.print(" sats Batt:");
    Serial.print(telemetryData.battery);
    Serial.print("mV Lux:");
    Serial.print(telemetryData.lux);
    Serial.print("lx UV:");
    Serial.print(telemetryData.uvIndex / 100.0);
    Serial.print(" CO2:");
    Serial.print(telemetryData.eCO2);
    Serial.print("ppm TVOC:");
    Serial.print(telemetryData.TVOC);
    Serial.println("ppb");
}
