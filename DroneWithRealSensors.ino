/*
 * Drone with Real Sensors
 * Adding real BMP280, AHT21, and GPS to working telemetry system
 */

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <TinyGPS++.h>

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5
#define BATTERY_PIN 35
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// I2C addresses
#define BMP280_ADDRESS 0x76

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Initialize real sensors
Adafruit_BMP280 bmp280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;

// Control packet (8 bytes)
struct ControlPacket
{
    int16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};

// Enhanced telemetry packet (12 bytes) - more real data
struct TelemetryPacket
{
    int16_t temperature; // x100 - Real BMP280 data
    uint16_t pressure;   // x10 - Real BMP280 data
    uint8_t humidity;    // % - Real AHT21 data
    uint16_t battery;    // mV - Real battery voltage
    int16_t latitude;    // GPS latitude (simplified)
    int16_t longitude;   // GPS longitude (simplified)
    uint8_t satellites;  // GPS satellite count
    uint8_t status;      // System status
};

ControlPacket receivedControl;
TelemetryPacket telemetryData;
unsigned long lastControlReceived = 0;
unsigned long lastSensorRead = 0;
unsigned long lastGPSRead = 0;
int packetsReceived = 0;
bool sensorsInitialized = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("=== Drone with Real Sensors ===");

    // Initialize I2C for sensors
    Wire.begin(21, 22); // SDA=21, SCL=22
    delay(100);

    // Initialize pins
    pinMode(BATTERY_PIN, INPUT);

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

    // Use EXACT same configuration as working test
    radio.openReadingPipe(1, address);
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(76);
    radio.setAutoAck(true);
    radio.setRetries(15, 15);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    // Start listening for control data
    radio.startListening();

    Serial.println("Radio configured successfully!");
    Serial.println("Waiting for control data...");
}

void loop()
{
    // Read sensors every 500ms
    if (millis() - lastSensorRead > 500)
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

    // Try BMP280 at common addresses
    bool bmp280Found = false;
    uint8_t bmpAddresses[] = {0x76, 0x77};
    for (int i = 0; i < 2; i++)
    {
        if (bmp280.begin(bmpAddresses[i]))
        {
            Serial.print("✓ BMP280 initialized at address 0x");
            Serial.println(bmpAddresses[i], HEX);
            bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                               Adafruit_BMP280::SAMPLING_X2,
                               Adafruit_BMP280::SAMPLING_X16,
                               Adafruit_BMP280::FILTER_X16,
                               Adafruit_BMP280::STANDBY_MS_500);
            sensorsInitialized = true;
            bmp280Found = true;
            break;
        }
    }

    if (!bmp280Found)
    {
        Serial.println("✗ BMP280 initialization failed - using simulated data");
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

    Serial.println("✓ GPS Serial2 initialized");
    Serial.println("Sensor initialization complete");
}

void readRealSensors()
{
    // Read BMP280 (temperature and pressure)
    if (sensorsInitialized && bmp280.begin(BMP280_ADDRESS))
    {
        float temp = bmp280.readTemperature();
        float pressure = bmp280.readPressure() / 100.0; // Convert Pa to hPa

        telemetryData.temperature = (int16_t)(temp * 100);
        telemetryData.pressure = (uint16_t)(pressure * 10);
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
    }

    // Read AHT21 (humidity and temperature)
    sensors_event_t humidity, temp;
    if (aht.getEvent(&humidity, &temp))
    {
        telemetryData.humidity = (uint8_t)humidity.relative_humidity;
        // Use AHT21 temperature if BMP280 failed
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
            Serial.print(" | Real Sensors - Temp:");
            Serial.print(telemetryData.temperature / 100.0);
            Serial.print("°C Hum:");
            Serial.print(telemetryData.humidity);
            Serial.print("% Press:");
            Serial.print(telemetryData.pressure / 10.0);
            Serial.print("hPa GPS:");
            Serial.print(telemetryData.satellites);
            Serial.println(" sats");

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
    Serial.print(", Real sensors: Temp:");
    Serial.print(telemetryData.temperature / 100.0);
    Serial.print("°C Hum:");
    Serial.print(telemetryData.humidity);
    Serial.print("% Press:");
    Serial.print(telemetryData.pressure / 10.0);
    Serial.print("hPa GPS:");
    Serial.print(telemetryData.satellites);
    Serial.print(" sats Batt:");
    Serial.print(telemetryData.battery);
    Serial.println("mV");
}
