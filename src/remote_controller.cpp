/*
 * Weather Drone Remote Controller
 * ESP32-based remote control with NRF24L01 communication and Firebase integration
 *
 * This code sends control commands to the drone and receives telemetry data
 * via NRF24L01 ACK payloads, then uploads the data to Firebase Realtime Database.
 *
 * Hardware specifications from README.md
 */

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <Wire.h>
#include <FirebaseESP32.h>
#include <ArduinoJson.h>
#include <time.h>

// Firebase configuration
#define FIREBASE_HOST "https://skyforge-4606b-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "HBgETEJwFCsLswgXH2SuzYupJPWhJqWnJdfkFvGO"
#define WIFI_SSID "Dialog 4G"
#define WIFI_PASSWORD "0N7NT00ANTQ"

// Pin Definitions (From README.md specifications)
#define CE_PIN 4  // NRF24L01 CE pin
#define CSN_PIN 5 // NRF24L01 CSN pin
// NRF24L01 SPI pins: SCK=18, MOSI=23, MISO=19 (hardware SPI)

// Joystick 1 pins (Throttle/Yaw)
#define JOY1_X_PIN 34 // Throttle/Yaw X-axis
#define JOY1_Y_PIN 35 // Throttle/Yaw Y-axis
#define JOY1_BTN_PIN 32

// Joystick 2 pins (Roll/Pitch)
#define JOY2_X_PIN 36   // Roll/Pitch X-axis
#define JOY2_Y_PIN 39   // Roll/Pitch Y-axis
#define JOY2_BTN_PIN 33 // Note: Not functioning well

// Toggle Switches (pin1 to pin2=GND, pin3=none)
#define TOGGLE_SW1_PIN 27 // Toggle switch 1
#define TOGGLE_SW2_PIN 14 // Toggle switch 2

// Display (I2C)
#define DISPLAY_SDA 21 // I2C SDA
#define DISPLAY_SCL 22 // I2C SCL

// NRF24L01 Configuration
RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"00001", "00002"};

// Firebase objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Control Input Structure (sent to drone)
#pragma pack(push, 1)
typedef struct
{
    int16_t throttle; // 0-1000
    int16_t roll;     // -500 to +500
    int16_t pitch;    // -500 to +500
    int16_t yaw;      // -500 to +500
    uint8_t switches; // Bit flags for toggle switches
    uint8_t mode;     // Flight mode
} ControlPacket;
#pragma pack(pop)

// Telemetry Structure (received from drone via ACK payload)
#pragma pack(push, 1)
typedef struct
{
    int16_t temperature;       // AHT21 Temperature (°C * 100)
    uint8_t humidity;          // AHT21 Humidity (% RH)
    uint16_t pressure;         // BME280 Pressure (hPa * 10)
    uint16_t gas_resistance;   // ENS160 TVOC in ppb
    uint8_t air_quality_index; // ENS160 AQI (1–5)
    uint16_t uvIndex;          // GUVA-S12SD UV Index (*100)
    uint16_t lightIntensity;   // BH1750 Light intensity (lux)
    int32_t latitude;          // GPS Latitude (*1e6)
    int32_t longitude;         // GPS Longitude (*1e6)
    int16_t altitude;          // Altitude in meters
    uint8_t satellites;        // Number of GPS satellites
    uint16_t batteryVoltage;   // Battery voltage in mV
    uint8_t esp32Temp;         // ESP32 internal temp (°C)
    uint32_t timestamp;        // Timestamp (millis/1000)
} TelemetryPacket;
#pragma pack(pop)

// Global Variables
ControlPacket controlData;
TelemetryPacket telemetryData;
unsigned long lastControlSend = 0;
unsigned long lastFirebaseUpload = 0;
unsigned long lastTelemetryDisplay = 0;
bool droneConnected = false;
bool firebaseReady = false;

// Joystick calibration values
int joy1_x_center = 2048, joy1_y_center = 2048;
int joy2_x_center = 2048, joy2_y_center = 2048;
int deadband = 100; // Deadband around center position

void setup()
{
    Serial.begin(115200);
    Serial.println("Weather Drone Remote Controller Starting...");

    // Initialize joystick pins
    initializeJoysticks();

    // Initialize NRF24L01
    initializeRadio();

    // Initialize WiFi and Firebase
    initializeWiFi();
    initializeFirebase();

    // Initialize control packet
    memset(&controlData, 0, sizeof(controlData));

    // Calibrate joysticks (read center positions)
    calibrateJoysticks();

    Serial.println("Remote Controller Ready!");
}

void loop()
{
    // Read joystick inputs and send control data (every 500ms for stability)
    if (millis() - lastControlSend > 500)
    {
        readJoystickInputs();
        sendControlData();
        lastControlSend = millis();
    }

    // Display telemetry data (every 5000ms)
    if (millis() - lastTelemetryDisplay > 5000)
    {
        displayTelemetryData();
        lastTelemetryDisplay = millis();
    }

    // Upload telemetry to Firebase (every 5000ms = 5 seconds)
    if (millis() - lastFirebaseUpload > 5000 && firebaseReady && droneConnected)
    {
        uploadTelemetryToFirebase();
        lastFirebaseUpload = millis();
    }

    delay(100); // Longer delay for better stability
}

void initializeJoysticks()
{
    Serial.println("Initializing joysticks and switches...");

    // Configure joystick pins as inputs
    pinMode(JOY1_X_PIN, INPUT);
    pinMode(JOY1_Y_PIN, INPUT);
    pinMode(JOY1_BTN_PIN, INPUT_PULLUP);

    pinMode(JOY2_X_PIN, INPUT);
    pinMode(JOY2_Y_PIN, INPUT);
    pinMode(JOY2_BTN_PIN, INPUT_PULLUP); // Note: Not functioning well

    // Configure toggle switches (pullup enabled, switch connects to GND)
    pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP);

    // Initialize I2C for display
    Wire.begin(DISPLAY_SDA, DISPLAY_SCL);

    Serial.println("Joysticks, switches, and I2C initialized!");
}

void initializeRadio()
{
    Serial.println("Initializing NRF24L01...");

    // Initialize SPI first
    SPI.begin();
    delay(100);

    if (!radio.begin())
    {
        Serial.println("NRF24L01 radio hardware not responding!");
        while (1)
        {
            delay(1000);
        }
    }

    // Basic radio test
    Serial.print("Radio chip connected: ");
    Serial.println(radio.isChipConnected());

    if (!radio.isChipConnected())
    {
        Serial.println("CRITICAL: NRF24L01 not detected! Check wiring.");
        while (1)
            delay(1000);
    }

    // Power up and reset the radio
    radio.powerUp();
    delay(150); // Longer power-up delay

    // Configure radio - Remote sends control data to drone, receives telemetry via ACK payload
    radio.openWritingPipe(address[1]); // Send to drone (drone listens on address[1])
    // Note: No reading pipe needed for ACK payloads - they're received automatically

    radio.setPALevel(RF24_PA_HIGH);  // Increase power for better range
    radio.setDataRate(RF24_1MBPS);   // Start with higher data rate for reliability
    radio.setChannel(76);            // Set channel
    radio.setRetries(15, 15);        // More aggressive retry settings
    radio.setCRCLength(RF24_CRC_16); // Enable 16-bit CRC
    radio.setAutoAck(true);          // Ensure auto-ack is enabled

    // Enable ACK payloads to receive telemetry
    radio.enableAckPayload();

    // Flush any existing data
    radio.flush_tx();
    radio.flush_rx();

    // Set as transmitter
    radio.stopListening();

    // Wait for radio to settle
    delay(200);

    Serial.println("NRF24L01 initialized successfully!");
    Serial.println("Radio details:");
    radio.printDetails();
    Serial.println("Remote ready to send control data...");

    // Debug: Print actual addresses being used
    Serial.println("Remote Address Configuration:");
    Serial.print("Writing to (drone RX): ");
    for (int i = 0; i < 5; i++)
    {
        Serial.print(address[1][i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println("ACK payloads will be received automatically after successful transmission");

    // Test radio functionality
    Serial.println("Testing radio connectivity...");
    Serial.print("Radio present: ");
    Serial.println(radio.isChipConnected());
    Serial.print("Radio power detected: ");
    Serial.println(radio.isPVariant());
}

void initializeWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println();
        Serial.print("WiFi connected! IP address: ");
        Serial.println(WiFi.localIP());

        // Configure time (needed for Firebase)
        configTime(0, 0, "pool.ntp.org", "time.nist.gov");
        Serial.println("Time configured for Firebase");
    }
    else
    {
        Serial.println("WiFi connection failed!");
    }
}

void initializeFirebase()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("WiFi not connected - skipping Firebase initialization");
        return;
    }

    Serial.println("Initializing Firebase...");

    // Configure Firebase
    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    // Initialize Firebase
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    // Test Firebase connection
    if (Firebase.ready())
    {
        Serial.println("Firebase connected successfully!");
        firebaseReady = true;

        // Upload initial status
        Firebase.setString(firebaseData, "/remote/status", "connected");
        Firebase.setInt(firebaseData, "/remote/startup_time", millis());
    }
    else
    {
        Serial.println("Firebase connection failed!");
    }
}

void calibrateJoysticks()
{
    Serial.println("Calibrating joysticks...");
    Serial.println("Make sure joysticks are in center position!");

    delay(2000); // Give user time to center joysticks

    // Read center positions
    joy1_x_center = analogRead(JOY1_X_PIN);
    joy1_y_center = analogRead(JOY1_Y_PIN);
    joy2_x_center = analogRead(JOY2_X_PIN);
    joy2_y_center = analogRead(JOY2_Y_PIN);

    Serial.print("Joystick 1 center: X=");
    Serial.print(joy1_x_center);
    Serial.print(", Y=");
    Serial.println(joy1_y_center);

    Serial.print("Joystick 2 center: X=");
    Serial.print(joy2_x_center);
    Serial.print(", Y=");
    Serial.println(joy2_y_center);

    Serial.println("Joystick calibration complete!");
}

void readJoystickInputs()
{
    // Read raw joystick values
    int joy1_x_raw = analogRead(JOY1_X_PIN);
    int joy1_y_raw = analogRead(JOY1_Y_PIN);
    int joy2_x_raw = analogRead(JOY2_X_PIN);
    int joy2_y_raw = analogRead(JOY2_Y_PIN);

    // Read button states (inverted due to pullup)
    bool joy1_btn = !digitalRead(JOY1_BTN_PIN);
    bool joy2_btn = !digitalRead(JOY2_BTN_PIN); // Note: Not functioning well

    // Read toggle switch states (inverted due to pullup, switch connects to GND)
    bool toggle_sw1 = !digitalRead(TOGGLE_SW1_PIN);
    bool toggle_sw2 = !digitalRead(TOGGLE_SW2_PIN);

    // Convert to control values with deadband
    // Joystick 1: Throttle (Y) and Yaw (X)
    controlData.throttle = mapJoystickWithDeadband(joy1_y_raw, joy1_y_center, 0, 1000);
    controlData.yaw = mapJoystickWithDeadband(joy1_x_raw, joy1_x_center, -500, 500);

    // Joystick 2: Roll (X) and Pitch (Y)
    controlData.roll = mapJoystickWithDeadband(joy2_x_raw, joy2_x_center, -500, 500);
    controlData.pitch = mapJoystickWithDeadband(joy2_y_raw, joy2_y_center, -500, 500);

    // Set switch states - using toggle switches instead of just joystick buttons
    controlData.switches = 0;
    if (joy1_btn)
        controlData.switches |= 0x01; // Joystick 1 button
    if (joy2_btn)
        controlData.switches |= 0x02; // Joystick 2 button (may not work)
    if (toggle_sw1)
        controlData.switches |= 0x04; // Toggle switch 1
    if (toggle_sw2)
        controlData.switches |= 0x08; // Toggle switch 2

    // Flight mode (can be expanded with additional switches)
    controlData.mode = 1; // Default flight mode
}

int mapJoystickWithDeadband(int rawValue, int centerValue, int minOutput, int maxOutput)
{
    int deviation = rawValue - centerValue;

    // Apply deadband
    if (abs(deviation) < deadband)
    {
        // If throttle, return 0 (minimum throttle)
        // If other controls, return center (0)
        return (minOutput == 0) ? 0 : 0;
    }

    // Map to output range
    if (deviation > 0)
    {
        // Positive direction
        return map(deviation, deadband, 2048, 0, maxOutput);
    }
    else
    {
        // Negative direction
        return map(deviation, -2048, -deadband, minOutput, 0);
    }
}

void sendControlData()
{
    // Full radio reset before each transmission for maximum reliability
    // This is a more aggressive approach to prevent state corruption
    radio.powerDown();
    delay(10);
    radio.powerUp();
    delay(10);

    // Reconfigure radio completely
    radio.openWritingPipe(address[1]);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(76);
    radio.enableAckPayload();
    radio.setAutoAck(true);
    radio.setRetries(5, 15); // Reduced retries for faster failure detection

    // Clear buffers and set mode
    radio.flush_tx();
    radio.flush_rx();
    radio.stopListening(); // Ensure TX mode
    delay(10);             // Let radio settle

    // Send control packet to drone
    bool result = radio.write(&controlData, sizeof(controlData));
    if (result)
    {
        // Print success message less frequently to reduce spam
        static unsigned long lastSuccessPrint = 0;
        static int successCount = 0;
        successCount++;

        if (millis() - lastSuccessPrint > 2000) // Print every 2 seconds
        {
            Serial.print("Control data sent successfully (");
            Serial.print(successCount);
            Serial.print(" packets) - T:");
            Serial.print(controlData.throttle);
            Serial.print(" R:");
            Serial.print(controlData.roll);
            Serial.print(" P:");
            Serial.print(controlData.pitch);
            Serial.print(" Y:");
            Serial.print(controlData.yaw);
            Serial.println();
            lastSuccessPrint = millis();
            successCount = 0;
        }

        droneConnected = true;

        // Wait for ACK payload
        delay(20); // Shorter delay, as ACK should be fast

        if (radio.isAckPayloadAvailable())
        {
            uint8_t len = radio.getDynamicPayloadSize();
            if (len == sizeof(telemetryData))
            {
                radio.read(&telemetryData, sizeof(telemetryData));
                // Print telemetry received message less frequently
                static unsigned long lastTelemetryPrint = 0;
                if (millis() - lastTelemetryPrint > 3000)
                {
                    Serial.println("Telemetry received successfully!");
                    lastTelemetryPrint = millis();
                }
            }
            else if (len > 0)
            {
                Serial.print("ACK payload size mismatch. Got: ");
                Serial.println(len);
                uint8_t dummy[32];
                radio.read(&dummy, len);
            }
        }
        else
        {
            // Only print this occasionally to reduce spam
            static unsigned long lastNoAckPrint = 0;
            if (millis() - lastNoAckPrint > 5000)
            {
                Serial.println("No ACK payload received");
                lastNoAckPrint = millis();
            }
        }
    }
    else
    {
        droneConnected = false;

        // Print failure message less frequently
        static unsigned long lastFailurePrint = 0;
        if (millis() - lastFailurePrint > 3000)
        {
            Serial.print("Transmission failed - Radio status: ");
            radio.printDetails(); // Print full details on failure
            lastFailurePrint = millis();
        }
    }
}
void displayTelemetryData()
{
    if (!droneConnected)
    {
        Serial.println("Drone not connected - no telemetry data");
        return;
    }

    Serial.println("=== REMOTE CONTROLLER STATUS ===");
    Serial.print("Control - Throttle: ");
    Serial.print(controlData.throttle);
    Serial.print(", Roll: ");
    Serial.print(controlData.roll);
    Serial.print(", Pitch: ");
    Serial.print(controlData.pitch);
    Serial.print(", Yaw: ");
    Serial.println(controlData.yaw);

    Serial.print("Switches - Joy1: ");
    Serial.print((controlData.switches & 0x01) ? "ON" : "OFF");
    Serial.print(", Joy2: ");
    Serial.print((controlData.switches & 0x02) ? "ON" : "OFF");
    Serial.print(", Toggle1: ");
    Serial.print((controlData.switches & 0x04) ? "ON" : "OFF");
    Serial.print(", Toggle2: ");
    Serial.print((controlData.switches & 0x08) ? "ON" : "OFF");
    Serial.println("");

    Serial.println("=== DRONE TELEMETRY ===");
    Serial.print("Temperature: ");
    Serial.print(telemetryData.temperature / 100.0);
    Serial.println("°C");

    Serial.print("Humidity: ");
    Serial.print(telemetryData.humidity);
    Serial.println("%");

    Serial.print("Pressure: ");
    Serial.print(telemetryData.pressure / 10.0);
    Serial.println(" hPa");

    Serial.print("Light: ");
    Serial.print(telemetryData.lightIntensity);
    Serial.println(" lux");

    Serial.print("UV Index: ");
    Serial.print(telemetryData.uvIndex / 100.0);
    Serial.println("");

    Serial.print("TVOC: ");
    Serial.print(telemetryData.gas_resistance);
    Serial.println(" ppb");

    Serial.print("AQI: ");
    Serial.print(telemetryData.air_quality_index);
    Serial.println("");

    Serial.print("GPS: ");
    Serial.print(telemetryData.latitude / 1e6, 6);
    Serial.print(", ");
    Serial.print(telemetryData.longitude / 1e6, 6);
    Serial.print(" Alt: ");
    Serial.print(telemetryData.altitude);
    Serial.print("m Sats: ");
    Serial.println(telemetryData.satellites);

    Serial.print("Battery: ");
    Serial.print(telemetryData.batteryVoltage);
    Serial.print("mV ESP32: ");
    Serial.print(telemetryData.esp32Temp);
    Serial.println("°C");

    Serial.print("WiFi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.print(" Firebase: ");
    Serial.println(firebaseReady ? "Ready" : "Not Ready");
    Serial.println("================================");
}

void uploadTelemetryToFirebase()
{
    if (!Firebase.ready())
    {
        Serial.println("Firebase not ready for upload");
        return;
    }

    // Get current timestamp
    time_t now = time(nullptr);

    // Create JSON object for telemetry data
    FirebaseJson json;

    // Add timestamp
    json.set("timestamp", (int)now);
    json.set("drone_timestamp", telemetryData.timestamp);

    // Environmental data
    json.set("temperature", telemetryData.temperature / 100.0);
    json.set("humidity", telemetryData.humidity);
    json.set("pressure", telemetryData.pressure / 10.0);
    json.set("light_intensity", telemetryData.lightIntensity);
    json.set("uv_index", telemetryData.uvIndex / 100.0);
    json.set("tvoc", telemetryData.gas_resistance);
    json.set("aqi", telemetryData.air_quality_index);

    // GPS data
    json.set("latitude", telemetryData.latitude / 1e6);
    json.set("longitude", telemetryData.longitude / 1e6);
    json.set("altitude", telemetryData.altitude);
    json.set("satellites", telemetryData.satellites);

    // System data
    json.set("battery_voltage", telemetryData.batteryVoltage);
    json.set("esp32_temp", telemetryData.esp32Temp);

    // Control data
    json.set("control_throttle", controlData.throttle);
    json.set("control_roll", controlData.roll);
    json.set("control_pitch", controlData.pitch);
    json.set("control_yaw", controlData.yaw);
    json.set("control_mode", controlData.mode);
    json.set("control_switches", controlData.switches);

    // Upload to Firebase
    String path = "/telemetry/" + String(now);

    if (Firebase.setJSON(firebaseData, path.c_str(), json))
    {
        Serial.println("Telemetry uploaded to Firebase successfully!");

        // Also update latest telemetry
        Firebase.setJSON(firebaseData, "/telemetry/latest", json);

        // Update connection status
        Firebase.setString(firebaseData, "/remote/last_upload", String(now));
        Firebase.setBool(firebaseData, "/remote/drone_connected", droneConnected);
    }
    else
    {
        Serial.print("Firebase upload failed: ");
        Serial.println(firebaseData.errorReason());
    }
}