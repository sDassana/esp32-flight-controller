/*
 * Drone with Enhanced Sensor Suite - FreeRTOS Version
 * Complete telemetry system with BME280, AHT21, GPS, and simulated advanced sensors
 * Compatible with remote control system (12-byte control, 22-byte telemetry)
 *
 * FreeRTOS Task Architecture:
 * - SensorTask: Reads all sensors (1Hz for environmental, 10Hz for GPS)
 * - RadioTask: Handles RF24 communication (5Hz control reception, 1Hz telemetry transmission)
 * - StatusTask: Prints system status and diagnostics (0.1Hz)
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPS++.h>
#include <BH1750.h>
#include <ScioSense_ENS160.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5
#define BATTERY_PIN 35
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GUVA_PIN 36 // GUVA-S12SD UV sensor

// ESC Control Pins (PWM Outputs to Brushless Motors)
#define ESC1_PIN 13 // Front Right
#define ESC2_PIN 12 // Front Left
#define ESC3_PIN 14 // Back Right
#define ESC4_PIN 27 // Back Left

// ESC PWM Configuration
#define ESC_MIN_PULSE 1000 // Minimum pulse width (microseconds)
#define ESC_MAX_PULSE 2000 // Maximum pulse width (microseconds)
#define ESC_ARM_PULSE 1000 // Arming pulse width
#define ESC_FREQUENCY 50   // 50Hz PWM frequency for ESCs

// I2C addresses
#define BME280_ADDRESS 0x76
#define BH1750_ADDRESS 0x23
#define ENS160_ADDRESS 0x53

// Task stack sizes
#define SENSOR_TASK_STACK 8192
#define RADIO_TASK_STACK 4096
#define STATUS_TASK_STACK 2048

// Task priorities
#define SENSOR_TASK_PRIORITY 2
#define RADIO_TASK_PRIORITY 3
#define STATUS_TASK_PRIORITY 1

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Initialize real sensors
Adafruit_BME280 bme280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;
BH1750 lightMeter;
ScioSense_ENS160 ens160;

// ESC Servo objects
Servo esc1, esc2, esc3, esc4;

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

// FreeRTOS Task Handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t radioTaskHandle = NULL;
TaskHandle_t statusTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

// FreeRTOS Synchronization
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t telemetryMutex;
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t controlMutex;

// Shared data structures (protected by mutexes)
ControlPacket receivedControl;
TelemetryPacket telemetryData;
unsigned long lastControlReceived = 0;
int packetsReceived = 0;
bool sensorsInitialized = false;
bool bh1750Ready = false;
bool ens160Ready = false;

// Motor control variables
volatile bool motorsArmed = false;
volatile bool emergencyStop = false;
volatile int motorSpeeds[4] = {ESC_ARM_PULSE, ESC_ARM_PULSE, ESC_ARM_PULSE, ESC_ARM_PULSE};
unsigned long lastValidControl = 0;
#define CONTROL_TIMEOUT_MS 1000   // 1 second timeout for safety
#define MIN_THROTTLE_FOR_ARM 1100 // Minimum throttle to consider arming
#define MOTOR_TASK_STACK 4096
#define MOTOR_TASK_PRIORITY 4 // High priority for motor control

// Altitude calibration variables
#define PRESSURE_BUFFER_SIZE 5    // Smaller buffer for faster response
float seaLevelPressure = 1013.25; // Will be calibrated at startup
bool altitudeCalibrated = false;
float pressureReadings[PRESSURE_BUFFER_SIZE]; // For smoothing pressure readings
int pressureIndex = 0;
int pressureCount = 0;

// Task timing variables
unsigned long sensorTaskCounter = 0;
unsigned long radioTaskCounter = 0;
unsigned long statusTaskCounter = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000); // Give serial time to initialize

    Serial.println("=== Drone with Real Sensors - FreeRTOS Version ===");
    Serial.println("Initializing system components...");

    // CRITICAL: Initialize ESCs FIRST - immediately on power-on for proper calibration
    Serial.println("PRIORITY: Initializing ESCs immediately on power-on...");
    initializeESCs();

    // Initialize I2C for sensors
    Wire.begin(21, 22); // SDA=21, SCL=22
    delay(100);

    // Initialize pins
    pinMode(BATTERY_PIN, INPUT);
    pinMode(GUVA_PIN, INPUT); // UV sensor analog input

    // Initialize GPS (Serial2 on ESP32)
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    // Create FreeRTOS mutexes
    i2cMutex = xSemaphoreCreateMutex();
    telemetryMutex = xSemaphoreCreateMutex();
    serialMutex = xSemaphoreCreateMutex();
    controlMutex = xSemaphoreCreateMutex();

    if (i2cMutex == NULL || telemetryMutex == NULL || serialMutex == NULL || controlMutex == NULL)
    {
        Serial.println("ERROR: Failed to create mutexes!");
        while (1)
            delay(1000);
    }

    // Initialize real sensors
    initializeRealSensors();

    // Initialize radio with PROVEN configuration
    initializeRadio();

    // Initialize telemetry data with safe defaults
    initializeTelemetryData();

    Serial.println("Creating FreeRTOS tasks...");

    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        SENSOR_TASK_STACK,
        NULL,
        SENSOR_TASK_PRIORITY,
        &sensorTaskHandle,
        0 // Core 0
    );

    xTaskCreatePinnedToCore(
        radioTask,
        "RadioTask",
        RADIO_TASK_STACK,
        NULL,
        RADIO_TASK_PRIORITY,
        &radioTaskHandle,
        1 // Core 1
    );

    xTaskCreatePinnedToCore(
        statusTask,
        "StatusTask",
        STATUS_TASK_STACK,
        NULL,
        STATUS_TASK_PRIORITY,
        &statusTaskHandle,
        0 // Core 0
    );

    xTaskCreatePinnedToCore(
        motorTask,
        "MotorTask",
        MOTOR_TASK_STACK,
        NULL,
        MOTOR_TASK_PRIORITY,
        &motorTaskHandle,
        1 // Core 1 - dedicated core for motor control
    );

    // Check if tasks were created successfully
    if (sensorTaskHandle == NULL || radioTaskHandle == NULL || statusTaskHandle == NULL || motorTaskHandle == NULL)
    {
        Serial.println("ERROR: Failed to create FreeRTOS tasks!");
        while (1)
            delay(1000);
    }

    Serial.println("✓ All FreeRTOS tasks created successfully");
    Serial.println("✓ System initialization complete");
    Serial.println("Starting multitasking operation...");
}

void loop()
{
    // FreeRTOS handles all tasks - main loop can be minimal
    // Print memory usage periodically
    static unsigned long lastMemCheck = 0;
    if (millis() - lastMemCheck > 30000) // Every 30 seconds
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("\n=== FreeRTOS Task Status ===");
            Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
            Serial.printf("Sensor task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(sensorTaskHandle));
            Serial.printf("Radio task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(radioTaskHandle));
            Serial.printf("Status task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(statusTaskHandle));
            Serial.printf("Motor task stack free: %d bytes\n", uxTaskGetStackHighWaterMark(motorTaskHandle));
            Serial.printf("Task counters - Sensor: %lu, Radio: %lu, Status: %lu\n",
                          sensorTaskCounter, radioTaskCounter, statusTaskCounter);
            Serial.println("===========================\n");
            xSemaphoreGive(serialMutex);
        }
        lastMemCheck = millis();
    }

    // Yield to FreeRTOS scheduler
    vTaskDelay(pdMS_TO_TICKS(5000)); // Sleep for 5 seconds
}

void sensorTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t sensorFrequency = pdMS_TO_TICKS(1000); // 1Hz for environmental sensors
    const TickType_t gpsFrequency = pdMS_TO_TICKS(100);     // 10Hz for GPS

    TickType_t lastGPSRead = 0;

    for (;;)
    {
        sensorTaskCounter++;

        // Read environmental sensors (1Hz)
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            readEnvironmentalSensors();
            xSemaphoreGive(i2cMutex);
        }

        // Read GPS more frequently (10Hz)
        TickType_t currentTime = xTaskGetTickCount();
        if ((currentTime - lastGPSRead) >= gpsFrequency)
        {
            updateGPS();
            lastGPSRead = currentTime;
        }

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, sensorFrequency);
    }
}

void radioTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t radioFrequency = pdMS_TO_TICKS(200); // 5Hz radio check frequency

    for (;;)
    {
        radioTaskCounter++;

        // Check for incoming control data
        handleControlData();

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, radioFrequency);
    }
}

void statusTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t statusFrequency = pdMS_TO_TICKS(10000); // 0.1Hz (every 10 seconds)

    for (;;)
    {
        statusTaskCounter++;

        // Print system status
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            printStatus();
            xSemaphoreGive(serialMutex);
        }

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, statusFrequency);
    }
}

void initializeRadio()
{
    Serial.println("Initializing RF24 radio...");

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

    Serial.println("✓ Radio configured successfully!");
    Serial.println("✓ Waiting for control data...");
}

void initializeTelemetryData()
{
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        telemetryData.temperature = 2500; // 25.00°C
        telemetryData.pressure = 10132;   // 1013.2 hPa
        telemetryData.humidity = 60;      // 60%
        telemetryData.battery = 3700;     // 3.7V
        telemetryData.latitude = 0;
        telemetryData.longitude = 0;
        telemetryData.satellites = 0;
        telemetryData.status = 0x01; // System OK
        telemetryData.lux = 500;     // 500 lux
        telemetryData.altitude = 0;  // 0.00m
        telemetryData.uvIndex = 300; // UV index 3.00
        telemetryData.eCO2 = 400;    // 400 ppm
        telemetryData.TVOC = 50;     // 50 ppb
        xSemaphoreGive(telemetryMutex);
    }
}

void initializeRealSensors()
{
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        Serial.println("Initializing real sensors...");
        xSemaphoreGive(serialMutex);
    }

    // Take I2C mutex for sensor initialization
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
        // Scan I2C bus first
        scanI2CBus();

        // Try BME280 at common addresses
        bool bme280Found = false;
        uint8_t bmeAddresses[] = {0x76, 0x77};
        for (int i = 0; i < 2; i++)
        {
            if (bme280.begin(bmeAddresses[i]))
            {
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
                {
                    Serial.print("✓ BME280 initialized at address 0x");
                    Serial.println(bmeAddresses[i], HEX);
                    xSemaphoreGive(serialMutex);
                }

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
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ BME280 initialization failed - using simulated data");
                xSemaphoreGive(serialMutex);
            }
        }

        // Initialize AHT21
        if (aht.begin())
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✓ AHT21 initialized");
                xSemaphoreGive(serialMutex);
            }
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ AHT21 initialization failed - using simulated data");
                xSemaphoreGive(serialMutex);
            }
        }

        // Initialize BH1750 light sensor
        if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✓ BH1750 light sensor initialized");
                xSemaphoreGive(serialMutex);
            }
            bh1750Ready = true;
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ BH1750 initialization failed - using simulated data");
                xSemaphoreGive(serialMutex);
            }
            bh1750Ready = false;
        }

        // Initialize ENS160 air quality sensor
        if (ens160.begin())
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✓ ENS160 air quality sensor initialized");
                xSemaphoreGive(serialMutex);
            }
            ens160.setMode(ENS160_OPMODE_STD);
            ens160Ready = true;
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.println("✗ ENS160 initialization failed - using simulated data");
                xSemaphoreGive(serialMutex);
            }
            ens160Ready = false;
        }

        xSemaphoreGive(i2cMutex);
    }

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("✓ GPS Serial2 initialized");
        Serial.println("✓ GUVA-S12SD UV sensor on analog pin");
        xSemaphoreGive(serialMutex);
    }

    // Calibrate altitude (assume current location is reference point)
    calibrateAltitude();

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("✓ Sensor initialization complete");
        xSemaphoreGive(serialMutex);
    }
}

void scanI2CBus()
{
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        Serial.println("Scanning I2C bus...");
        xSemaphoreGive(serialMutex);
    }

    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0)
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                Serial.print("I2C device found at address 0x");
                if (address < 16)
                    Serial.print("0");
                Serial.println(address, HEX);
                xSemaphoreGive(serialMutex);
            }
            nDevices++;
        }
    }

    if (nDevices == 0)
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("No I2C devices found");
            xSemaphoreGive(serialMutex);
        }
    }
}

void calibrateAltitude()
{
    if (!sensorsInitialized)
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("⚠ BME280 not available - altitude will use default sea level pressure");
            xSemaphoreGive(serialMutex);
        }
        return;
    }

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println("Calibrating altitude reference...");
        xSemaphoreGive(serialMutex);
    }

    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Take multiple pressure readings for calibration
    float pressureSum = 0;
    int validReadings = 0;

    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5000)) == pdTRUE)
    {
        for (int i = 0; i < 30; i++)
        {
            // Force a reading and wait
            bme280.takeForcedMeasurement();
            vTaskDelay(pdMS_TO_TICKS(100));

            float pressure = bme280.readPressure() / 100.0; // Convert Pa to hPa
            if (pressure > 800 && pressure < 1200)          // Reasonable pressure range
            {
                pressureSum += pressure;
                validReadings++;
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    Serial.print(".");
                    xSemaphoreGive(serialMutex);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        xSemaphoreGive(i2cMutex);
    }

    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        Serial.println();
        xSemaphoreGive(serialMutex);
    }

    if (validReadings > 20)
    {
        seaLevelPressure = pressureSum / validReadings;
        altitudeCalibrated = true;

        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.print("✓ Altitude calibrated - Reference pressure: ");
            Serial.print(seaLevelPressure);
            Serial.println(" hPa (current location = 0m)");
            xSemaphoreGive(serialMutex);
        }
    }
    else
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            Serial.println("✗ Altitude calibration failed - using standard sea level pressure");
            xSemaphoreGive(serialMutex);
        }
    }
}

void readEnvironmentalSensors()
{
    // Create local copy to minimize mutex lock time
    TelemetryPacket localTelemetry;

    // Read BME280 (temperature and pressure)
    if (sensorsInitialized)
    {
        // Force a measurement for consistent timing
        bme280.takeForcedMeasurement();

        float temp = bme280.readTemperature();
        float currentPressure = bme280.readPressure() / 100.0; // Convert Pa to hPa

        localTelemetry.temperature = (int16_t)(temp * 100);
        localTelemetry.pressure = (uint16_t)(currentPressure * 10);

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

            // Debug output for pressure and altitude calculation (less frequent)
            static unsigned long lastDebug = 0;
            static int debugCounter = 0;
            if (millis() - lastDebug > 5000) // Debug every 5 seconds
            {
                if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
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
                    xSemaphoreGive(serialMutex);
                }
                lastDebug = millis();
                debugCounter++;
            }

            // Simple range limiting
            if (altitude < -500)
                altitude = -500;
            if (altitude > 5000)
                altitude = 5000;

            // Store altitude in centimeters for 2 decimal precision
            localTelemetry.altitude = (int16_t)(altitude * 100); // Convert meters to centimeters
        }
        else
        {
            // Invalid pressure reading - keep current altitude
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                localTelemetry.altitude = telemetryData.altitude;
                xSemaphoreGive(telemetryMutex);
            }
        }
    }
    else
    {
        // Fallback to simulated data
        static float temp = 25.0;
        temp += (random(-10, 11) / 100.0);
        localTelemetry.temperature = (int16_t)(temp * 100);

        static float pressure = 1013.2;
        pressure += (random(-5, 6) / 10.0);
        localTelemetry.pressure = (uint16_t)(pressure * 10);

        localTelemetry.altitude = 10000 + random(-2000, 2001); // Simulated altitude in cm
    }

    // Read AHT21 (humidity and temperature)
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp))
    {
        localTelemetry.humidity = (uint8_t)humidity.relative_humidity;
        // Use AHT21 temperature if BME280 failed
        if (!sensorsInitialized)
        {
            localTelemetry.temperature = (int16_t)(temp.temperature * 100);
        }
    }
    else
    {
        // Fallback to simulated humidity
        localTelemetry.humidity = 60 + random(-10, 11);
    }

    // Read real battery voltage
    int batteryRaw = analogRead(BATTERY_PIN);
    if (batteryRaw < 100)
    {
        // Simulate battery if not connected
        localTelemetry.battery = 3700 + random(-100, 101);
    }
    else
    {
        localTelemetry.battery = map(batteryRaw, 0, 4095, 0, 5000);
    }

    // Read real light sensor (BH1750)
    if (bh1750Ready)
    {
        float lux = lightMeter.readLightLevel();
        if (lux >= 0)
        {
            localTelemetry.lux = (uint16_t)lux;
        }
        else
        {
            localTelemetry.lux = 500 + random(-100, 101); // Fallback simulated
        }
    }
    else
    {
        localTelemetry.lux = 500 + random(-100, 101); // Simulated if sensor not ready
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
    localTelemetry.uvIndex = (uint16_t)(uvIndex * 100);

    // Read real air quality sensor (ENS160)
    if (ens160Ready && ens160.available())
    {
        ens160.measure();
        uint16_t eco2 = ens160.geteCO2();
        uint16_t tvoc = ens160.getTVOC();

        // Validate readings (ENS160 sometimes gives invalid values during warm-up)
        if (eco2 > 400 && eco2 < 5000) // Reasonable CO2 range
        {
            localTelemetry.eCO2 = eco2;
        }
        else
        {
            localTelemetry.eCO2 = 400 + random(-50, 51); // Fallback
        }

        if (tvoc < 1000) // Reasonable TVOC range
        {
            localTelemetry.TVOC = tvoc;
        }
        else
        {
            localTelemetry.TVOC = 50 + random(-20, 21); // Fallback
        }
    }
    else
    {
        // Fallback to simulated values
        localTelemetry.eCO2 = 400 + random(-50, 51);
        localTelemetry.TVOC = 50 + random(-20, 21);
    }

    // Update status
    localTelemetry.status = 0x01; // All systems OK
    if (!sensorsInitialized)
        localTelemetry.status |= 0x02; // Sensor warning

    // Get GPS data from shared telemetry (GPS is updated in its own function)
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        localTelemetry.latitude = telemetryData.latitude;
        localTelemetry.longitude = telemetryData.longitude;
        localTelemetry.satellites = telemetryData.satellites;

        // Update all telemetry data atomically
        telemetryData = localTelemetry;
        xSemaphoreGive(telemetryMutex);
    }
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
                // Update GPS data in shared telemetry
                if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    telemetryData.latitude = (int16_t)(gps.location.lat() * 100);
                    telemetryData.longitude = (int16_t)(gps.location.lng() * 100);
                    telemetryData.satellites = gps.satellites.value();
                    xSemaphoreGive(telemetryMutex);
                }
            }
        }
    }

    // If no GPS fix, use default values
    if (!gps.location.isValid())
    {
        if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            telemetryData.latitude = 0;
            telemetryData.longitude = 0;
            telemetryData.satellites = 0;
            xSemaphoreGive(telemetryMutex);
        }
    }
}

void handleControlData()
{
    if (radio.available())
    {
        uint8_t len = radio.getDynamicPayloadSize();

        if (len == sizeof(ControlPacket))
        {
            ControlPacket localControl;
            radio.read(&localControl, sizeof(localControl));

            // Update shared control data (for telemetry display)
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                lastControlReceived = millis();
                packetsReceived++;
                xSemaphoreGive(telemetryMutex);
            }

            // Update control data for motor task (separate mutex for higher priority)
            if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                receivedControl = localControl;
                lastValidControl = millis(); // Update motor control timeout
                xSemaphoreGive(controlMutex);
            }

            // Print received control data with real sensor info (thread-safe)
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                Serial.print("Control #");
                Serial.print(packetsReceived);
                Serial.print(" - T:");
                Serial.print(localControl.throttle);
                Serial.print(" R:");
                Serial.print(localControl.roll);
                Serial.print(" P:");
                Serial.print(localControl.pitch);
                Serial.print(" Y:");
                Serial.print(localControl.yaw);

                // Show button states when pressed
                if (localControl.joy1_btn == 0)
                    Serial.print(" [JOY1-BTN]");
                if (localControl.joy2_btn == 0)
                    Serial.print(" [JOY2-BTN]");

                // Show toggle switch states
                if (localControl.toggle1 == 1)
                    Serial.print(" [SW1-ARM]");
                if (localControl.toggle2 == 1)
                    Serial.print(" [SW2-ESTOP]");

                // Get current telemetry for display
                TelemetryPacket currentTelemetry;
                if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    currentTelemetry = telemetryData;
                    xSemaphoreGive(telemetryMutex);
                }

                Serial.print(" | Sensors - Temp:");
                Serial.print(currentTelemetry.temperature / 100.0);
                Serial.print("°C Hum:");
                Serial.print(currentTelemetry.humidity);
                Serial.print("% Press:");
                Serial.print(currentTelemetry.pressure / 10.0);
                Serial.print("hPa Alt:");
                Serial.print(currentTelemetry.altitude / 100.0); // Display altitude in meters with 2 decimals
                Serial.print("m GPS:");
                Serial.print(currentTelemetry.satellites);
                Serial.print(" sats Lux:");
                Serial.print(currentTelemetry.lux);
                Serial.print("lx UV:");
                Serial.print(currentTelemetry.uvIndex / 100.0);
                Serial.print(" CO2:");
                Serial.print(currentTelemetry.eCO2);
                Serial.print("ppm TVOC:");
                Serial.print(currentTelemetry.TVOC);
                Serial.println("ppb");

                xSemaphoreGive(serialMutex);
            }

            // Load telemetry data as ACK payload for NEXT packet
            TelemetryPacket ackTelemetry;
            if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                ackTelemetry = telemetryData;
                xSemaphoreGive(telemetryMutex);
            }
            radio.writeAckPayload(1, &ackTelemetry, sizeof(ackTelemetry));
        }
        else
        {
            if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
            {
                Serial.print("Wrong packet size: ");
                Serial.println(len);
                xSemaphoreGive(serialMutex);
            }
        }
    }
}

void printStatus()
{
    unsigned long uptime = millis() / 1000;
    unsigned long lastControl;
    int packets;
    TelemetryPacket currentTelemetry;

    // Get shared data safely
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        lastControl = lastControlReceived;
        packets = packetsReceived;
        currentTelemetry = telemetryData;
        xSemaphoreGive(telemetryMutex);
    }

    bool controlActive = (millis() - lastControl) < 3000;

    Serial.print("Drone status [FreeRTOS] - Running ");
    Serial.print(uptime);
    Serial.print("s, Control packets: ");
    Serial.print(packets);
    Serial.print(", Active: ");
    Serial.print(controlActive ? "YES" : "NO");
    Serial.print(", Motors: ");
    Serial.print(motorsArmed ? "ARMED" : "DISARMED");
    if (emergencyStop)
        Serial.print(" [E-STOP]");
    Serial.print(", Sensors: Temp:");
    Serial.print(currentTelemetry.temperature / 100.0);
    Serial.print("°C Hum:");
    Serial.print(currentTelemetry.humidity);
    Serial.print("% Press:");
    Serial.print(currentTelemetry.pressure / 10.0);
    Serial.print("hPa Alt:");
    Serial.print(currentTelemetry.altitude / 100.0); // Display altitude in meters with 2 decimals
    Serial.print("m GPS:");
    Serial.print(currentTelemetry.satellites);
    Serial.print(" sats Batt:");
    Serial.print(currentTelemetry.battery);
    Serial.print("mV Lux:");
    Serial.print(currentTelemetry.lux);
    Serial.print("lx UV:");
    Serial.print(currentTelemetry.uvIndex / 100.0);
    Serial.print(" CO2:");
    Serial.print(currentTelemetry.eCO2);
    Serial.print("ppm TVOC:");
    Serial.print(currentTelemetry.TVOC);
    Serial.println("ppb");
}

// ESC Initialization Function - CRITICAL: Called immediately on power-on
void initializeESCs()
{
    // Direct serial output - no mutex needed since FreeRTOS not started yet
    Serial.println("CRITICAL: Initializing ESCs immediately on power-on for proper calibration...");

    // Configure servo library for ESCs
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Attach ESCs to pins
    esc1.setPeriodHertz(ESC_FREQUENCY);
    esc2.setPeriodHertz(ESC_FREQUENCY);
    esc3.setPeriodHertz(ESC_FREQUENCY);
    esc4.setPeriodHertz(ESC_FREQUENCY);

    esc1.attach(ESC1_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    esc2.attach(ESC2_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    esc3.attach(ESC3_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);
    esc4.attach(ESC4_PIN, ESC_MIN_PULSE, ESC_MAX_PULSE);

    // ESC Calibration Sequence (as per README requirements) - IMMEDIATE on power-on
    Serial.println("Starting IMMEDIATE ESC calibration sequence...");
    Serial.println("1. Setting MAX throttle for 2 seconds (POWER-ON CALIBRATION)...");

    // Step 1: Maximum throttle for 2 seconds
    esc1.writeMicroseconds(ESC_MAX_PULSE);
    esc2.writeMicroseconds(ESC_MAX_PULSE);
    esc3.writeMicroseconds(ESC_MAX_PULSE);
    esc4.writeMicroseconds(ESC_MAX_PULSE);

    delay(2000); // Use regular delay since FreeRTOS not started yet

    Serial.println("2. Setting MIN throttle (arming)...");

    // Step 2: Minimum throttle (arming position)
    esc1.writeMicroseconds(ESC_ARM_PULSE);
    esc2.writeMicroseconds(ESC_ARM_PULSE);
    esc3.writeMicroseconds(ESC_ARM_PULSE);
    esc4.writeMicroseconds(ESC_ARM_PULSE);

    delay(1000); // Use regular delay since FreeRTOS not started yet

    Serial.println("✓ ESC calibration complete - Motors ready");

    // Initialize motor speeds to safe values
    for (int i = 0; i < 4; i++)
    {
        motorSpeeds[i] = ESC_ARM_PULSE;
    }

    motorsArmed = false;
    emergencyStop = false;
}

// Motor Control Task
void motorTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t motorFrequency = pdMS_TO_TICKS(20); // 50Hz motor update (20ms)

    for (;;)
    {
        // Check for control timeout (safety feature)
        unsigned long currentTime = millis();
        bool controlValid = (currentTime - lastValidControl) < CONTROL_TIMEOUT_MS;

        if (!controlValid || emergencyStop)
        {
            // Safety: Stop all motors if no recent control or emergency stop
            for (int i = 0; i < 4; i++)
            {
                motorSpeeds[i] = ESC_ARM_PULSE;
            }
            motorsArmed = false;
        }
        else if (xSemaphoreTake(controlMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            // Process control inputs and calculate motor speeds
            calculateMotorSpeeds();
            xSemaphoreGive(controlMutex);
        }

        // Write motor speeds to ESCs
        esc1.writeMicroseconds(motorSpeeds[0]); // Front Right
        esc2.writeMicroseconds(motorSpeeds[1]); // Front Left
        esc3.writeMicroseconds(motorSpeeds[2]); // Back Right
        esc4.writeMicroseconds(motorSpeeds[3]); // Back Left

        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, motorFrequency);
    }
}

// Calculate Motor Speeds from Control Inputs
void calculateMotorSpeeds()
{
    // Check emergency stop (toggle switch 2)
    if (receivedControl.toggle2 == 1)
    {
        emergencyStop = true;
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            static unsigned long lastEmergencyMsg = 0;
            if (millis() - lastEmergencyMsg > 5000)
            {
                Serial.println("EMERGENCY STOP ACTIVATED!");
                lastEmergencyMsg = millis();
            }
            xSemaphoreGive(serialMutex);
        }
        return;
    }
    else
    {
        emergencyStop = false;
    }

    // Map joystick inputs (±3000 range) to motor control values
    // Note: The remote now uses ±3000 range for reduced sensitivity

    // Convert control inputs to standard range (-500 to +500 for mixing)
    int throttleInput = map(receivedControl.throttle, -3000, 3000, -500, 500);
    int rollInput = map(receivedControl.roll, -3000, 3000, -500, 500);
    int pitchInput = map(receivedControl.pitch, -3000, 3000, -500, 500);
    int yawInput = map(receivedControl.yaw, -3000, 3000, -500, 500);

    // Base throttle (center stick should be minimum safe throttle)
    int baseThrottle = ESC_ARM_PULSE;

    // Only add throttle if stick is above center (for safety)
    if (throttleInput > 0)
    {
        baseThrottle += map(throttleInput, 0, 500, 0, 400); // Max 400 additional pulse width
    }

    // Check if motors should be armed (toggle switch 1 controls arming)
    if (receivedControl.toggle1 == 1 && !emergencyStop)
    {
        motorsArmed = true;
    }
    else
    {
        motorsArmed = false;
    }

    if (!motorsArmed)
    {
        // Motors disarmed - all ESCs to minimum
        for (int i = 0; i < 4; i++)
        {
            motorSpeeds[i] = ESC_ARM_PULSE;
        }
        return;
    }

    // Quadcopter Motor Mixing
    // Standard X-configuration:
    // Motor 1 (Front Right): +Throttle -Roll +Pitch -Yaw
    // Motor 2 (Front Left):  +Throttle +Roll +Pitch +Yaw
    // Motor 3 (Back Right):  +Throttle -Roll -Pitch +Yaw
    // Motor 4 (Back Left):   +Throttle +Roll -Pitch -Yaw

    motorSpeeds[0] = baseThrottle - rollInput + pitchInput - yawInput; // Front Right
    motorSpeeds[1] = baseThrottle + rollInput + pitchInput + yawInput; // Front Left
    motorSpeeds[2] = baseThrottle - rollInput - pitchInput + yawInput; // Back Right
    motorSpeeds[3] = baseThrottle + rollInput - pitchInput - yawInput; // Back Left

    // Constrain motor speeds to safe range
    for (int i = 0; i < 4; i++)
    {
        motorSpeeds[i] = constrain(motorSpeeds[i], ESC_ARM_PULSE, ESC_MAX_PULSE);
    }

    // Debug output (less frequent to avoid overwhelming serial)
    static unsigned long lastMotorDebug = 0;
    if (millis() - lastMotorDebug > 1000) // Every 1 second
    {
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            Serial.print("Motors [Armed:");
            Serial.print(motorsArmed ? "YES" : "NO");
            Serial.print(", SW1:");
            Serial.print(receivedControl.toggle1 ? "ARM" : "DISARM");
            Serial.print(", SW2:");
            Serial.print(receivedControl.toggle2 ? "ESTOP" : "OK");
            Serial.print("] T:");
            Serial.print(throttleInput);
            Serial.print(" R:");
            Serial.print(rollInput);
            Serial.print(" P:");
            Serial.print(pitchInput);
            Serial.print(" Y:");
            Serial.print(yawInput);
            Serial.print(" -> M1:");
            Serial.print(motorSpeeds[0]);
            Serial.print(" M2:");
            Serial.print(motorSpeeds[1]);
            Serial.print(" M3:");
            Serial.print(motorSpeeds[2]);
            Serial.print(" M4:");
            Serial.println(motorSpeeds[3]);
            xSemaphoreGive(serialMutex);
        }
        lastMotorDebug = millis();
    }
}
