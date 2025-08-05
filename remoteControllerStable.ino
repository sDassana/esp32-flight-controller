/*
 * Complete Remote Controller - Stable Non-FreeRTOS Version
 * Reliable control at 200ms intervals (5Hz) for stable drone communication
 * Compatible with droneFreeRTOS.ino
 */

#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

// Firebase configuration
#define FIREBASE_HOST "https://skyforge-4606b-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define FIREBASE_AUTH "HBgETEJwFCsLswgXH2SuzYupJPWhJqWnJdfkFvGO"
#define WIFI_SSID "Dialog 4G"
#define WIFI_PASSWORD "0N7NT00ANTQ"

// Upload frequency configuration (in milliseconds)
#define FIREBASE_UPLOAD_INTERVAL 60000 // 60 seconds (conservative for stability)
#define FIREBASE_ENABLED true          // Enabled with simple approach
// Common testing intervals:
// 10000  = 10 seconds (normal)
// 30000  = 30 seconds (slow/production)
// 60000  = 1 minute (very slow)

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5

// Joystick pins - Updated configuration
#define JOY1_X_PIN 39   // Joystick 1 X-axis
#define JOY1_Y_PIN 36   // Joystick 1 Y-axis
#define JOY1_BTN_PIN 33 // Joystick 1 Button (Note: Not functioning well)
#define JOY2_X_PIN 34   // Joystick 2 X-axis
#define JOY2_Y_PIN 35   // Joystick 2 Y-axis
#define JOY2_BTN_PIN 32 // Joystick 2 Button
#define TOGGLE_SW1_PIN 27
#define TOGGLE_SW2_PIN 14

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Firebase objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Control packet (12 bytes) - Added button states and toggle switches
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

// Enhanced telemetry packet (22 bytes) - matches drone with lux, altitude, UV index, eCO2, and TVOC
struct TelemetryPacket
{
    int16_t temperature; // x100 - Real BME280 data
    uint16_t pressure;   // x10 - Real BME280 data
    uint8_t humidity;    // % - Real BME280 data
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

ControlPacket controlData;
TelemetryPacket telemetryData;
unsigned long lastControlSend = 0;
unsigned long lastFirebaseUpload = 0;
unsigned long lastTelemetryPrint = 0;
unsigned long lastStatusPrint = 0;
bool telemetryReceived = false;
bool firebaseReady = false;
bool wifiConnected = false;

// Statistics tracking
int successCount = 0;
int failCount = 0;
int telemetryCount = 0;
int packetNumber = 1;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== Complete Remote Controller - Stable Version ===");
    Serial.println("Compatible with droneFreeRTOS.ino");

    // Initialize pins
    pinMode(JOY1_X_PIN, INPUT);
    pinMode(JOY1_Y_PIN, INPUT);
    pinMode(JOY1_BTN_PIN, INPUT_PULLUP); // Joystick 1 button (not functioning well)
    pinMode(JOY2_X_PIN, INPUT);
    pinMode(JOY2_Y_PIN, INPUT);
    pinMode(JOY2_BTN_PIN, INPUT_PULLUP); // Joystick 2 button
    pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP);

    // Initialize control data with safe defaults
    controlData.throttle = 0;
    controlData.roll = 0;
    controlData.pitch = 0;
    controlData.yaw = 0;
    controlData.joy1_btn = 1; // Released
    controlData.joy2_btn = 1; // Released
    controlData.toggle1 = 0;  // Off
    controlData.toggle2 = 0;  // Off

    // Initialize radio with PROVEN configuration
    SPI.begin();
    delay(100);

    if (!radio.begin() || !radio.isChipConnected())
    {
        Serial.println("ERROR: Radio initialization failed!");
        while (1)
            delay(1000);
    }

    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_HIGH);  // Increased from PA_LOW to PA_HIGH for better range
    radio.setDataRate(RF24_250KBPS); // Reduced from 1MBPS to 250KBPS for better range
    radio.setChannel(76);
    radio.setAutoAck(true);
    radio.setRetries(15, 15); // Maximum retries for reliability - PROVEN working configuration
    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    // Additional range improvements
    radio.setCRCLength(RF24_CRC_16); // Use 16-bit CRC for better error detection
    radio.setAddressWidth(5);        // Use full 5-byte addresses

    radio.stopListening();

    Serial.println("✓ Radio configured successfully!");

    // Initialize WiFi and Firebase only if enabled
    if (FIREBASE_ENABLED)
    {
        initializeWiFi();
        if (wifiConnected)
        {
            initializeFirebase();
        }
    }
    else
    {
        Serial.println("Firebase disabled - running in offline mode");
    }

    Serial.print("Starting PROVEN control transmission (5Hz) with telemetry");
    if (FIREBASE_ENABLED)
    {
        Serial.print(" and cloud upload every ");
        Serial.print(FIREBASE_UPLOAD_INTERVAL / 1000);
        Serial.println(" seconds...");
    }
    else
    {
        Serial.println(" (Firebase disabled)...");
    }

    // Print radio configuration for debugging
    Serial.println("Radio Configuration:");
    Serial.print("  Power Level: RF24_PA_HIGH");
    Serial.print(", Data Rate: RF24_250KBPS");
    Serial.print(", Channel: 76");
    Serial.print(", CRC: 16-bit");
    Serial.println(", Address Width: 5 bytes");
}

void loop()
{
    unsigned long currentTime = millis();

    // Send control data at 5Hz (200ms intervals) - PROVEN WORKING RATE
    if (currentTime - lastControlSend >= 200)
    {
        readJoystickInputs();
        sendControlData();
        lastControlSend = currentTime;
    }

    // Print telemetry every 2 seconds if available
    if (telemetryReceived && (currentTime - lastTelemetryPrint >= 2000))
    {
        printTelemetryData();
        lastTelemetryPrint = currentTime;
    }

    // Print status every 10 seconds
    if (currentTime - lastStatusPrint >= 10000)
    {
        printStatusReport();
        lastStatusPrint = currentTime;
    }

    // Upload to Firebase at specified interval
    if (FIREBASE_ENABLED && firebaseReady && telemetryReceived &&
        (currentTime - lastFirebaseUpload >= FIREBASE_UPLOAD_INTERVAL))
    {
        uploadTelemetryToFirebase();
        lastFirebaseUpload = currentTime;
    }

    // Check WiFi connection periodically
    if (FIREBASE_ENABLED && (currentTime % 30000 == 0)) // Every 30 seconds
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            wifiConnected = false;
            firebaseReady = false;
            Serial.println("WiFi disconnected, attempting reconnection...");
            initializeWiFi();
            if (wifiConnected)
            {
                initializeFirebase();
            }
        }
        else
        {
            wifiConnected = true;
        }
    }

    // Small delay to prevent overwhelming the system
    delay(10);
}

void readJoystickInputs()
{
    int joy1_x = analogRead(JOY1_X_PIN);
    int joy1_y = analogRead(JOY1_Y_PIN);
    int joy2_x = analogRead(JOY2_X_PIN);
    int joy2_y = analogRead(JOY2_Y_PIN);

    // Read joystick buttons (active LOW, so invert the reading)
    controlData.joy1_btn = digitalRead(JOY1_BTN_PIN); // Note: JOY1 button may not function well
    controlData.joy2_btn = digitalRead(JOY2_BTN_PIN);

    // Read toggle switches (active LOW, so invert the reading)
    controlData.toggle1 = !digitalRead(TOGGLE_SW1_PIN); // Invert: 0=off, 1=on
    controlData.toggle2 = !digitalRead(TOGGLE_SW2_PIN); // Invert: 0=off, 1=on

    // Map analog readings to control values (larger range = lower sensitivity)
    controlData.throttle = map(joy1_y, 0, 4095, -3000, 3000);
    controlData.yaw = map(joy1_x, 0, 4095, -3000, 3000);
    controlData.roll = map(joy2_x, 0, 4095, -3000, 3000);
    controlData.pitch = map(joy2_y, 0, 4095, -3000, 3000);
}

void sendControlData()
{
    bool result = false;
    int attempts = 0;
    const int maxAttempts = 3; // Proven working retry count

    // Try multiple times for reliability
    while (!result && attempts < maxAttempts)
    {
        result = radio.write(&controlData, sizeof(controlData));
        if (!result)
        {
            attempts++;
            delayMicroseconds(100); // Small delay between attempts
        }
    }

    if (result)
    {
        successCount++;

        // Check for ACK payload with telemetry
        if (radio.isAckPayloadAvailable())
        {
            uint8_t len = radio.getDynamicPayloadSize();
            if (len == sizeof(TelemetryPacket))
            {
                radio.read(&telemetryData, sizeof(telemetryData));
                telemetryReceived = true;
                telemetryCount++;
            }
        }
    }
    else
    {
        failCount++;
        // Print failures occasionally
        if (failCount % 10 == 0) // Every 10 failures
        {
            Serial.print("Packet #");
            Serial.print(packetNumber);
            Serial.print(" FAILED after ");
            Serial.print(attempts);
            Serial.println(" attempts");
        }
    }

    packetNumber++;
}

void printTelemetryData()
{
    Serial.print("Packet #");
    Serial.print(packetNumber);
    Serial.print(" SUCCESS");
    Serial.print(" - T:");
    Serial.print(controlData.throttle);
    Serial.print(" R:");
    Serial.print(controlData.roll);
    Serial.print(" P:");
    Serial.print(controlData.pitch);
    Serial.print(" Y:");
    Serial.print(controlData.yaw);

    // Show button states when pressed
    if (controlData.joy1_btn == 0)
        Serial.print(" [JOY1-BTN]");
    if (controlData.joy2_btn == 0)
        Serial.print(" [JOY2-BTN]");

    // Show toggle switch states
    if (controlData.toggle1 == 1)
        Serial.print(" [SW1-ON]");
    if (controlData.toggle2 == 1)
        Serial.print(" [SW2-ON]");

    Serial.print(" | Telemetry - Temp:");
    Serial.print(telemetryData.temperature / 100.0);
    Serial.print("°C Press:");
    Serial.print(telemetryData.pressure / 10.0);
    Serial.print("hPa Alt:");
    Serial.print(telemetryData.altitude / 100.0, 2); // Display altitude in meters with 2 decimals
    Serial.print("m Hum:");
    Serial.print(telemetryData.humidity);
    Serial.print("% GPS:");
    Serial.print(telemetryData.satellites);
    Serial.print(" sats Lat:");
    Serial.print(telemetryData.latitude / 100.0, 2);
    Serial.print(" Lng:");
    Serial.print(telemetryData.longitude / 100.0, 2);
    Serial.print(" Batt:");
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

void printStatusReport()
{
    Serial.print("Status - Success: ");
    Serial.print(successCount);
    Serial.print(", Failed: ");
    Serial.print(failCount);
    Serial.print(", Telemetry: ");
    Serial.print(telemetryCount);
    Serial.print(", Success rate: ");
    if (successCount + failCount > 0)
    {
        int successRate = (successCount * 100) / (successCount + failCount);
        Serial.print(successRate);
        Serial.print("%");
    }
    else
    {
        Serial.print("N/A");
    }

    if (FIREBASE_ENABLED)
    {
        Serial.print(", WiFi: ");
        Serial.print(wifiConnected ? "Connected" : "Disconnected");
        Serial.print(", Firebase: ");
        Serial.print(firebaseReady ? "Ready" : "Not Ready");
    }

    Serial.print(", Free heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    // Reset counters
    successCount = 0;
    failCount = 0;
}

void initializeWiFi()
{
    Serial.println("Connecting to WiFi...");
    WiFi.disconnect();
    delay(100);

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
        wifiConnected = true;
        Serial.println("\n✓ WiFi connected successfully!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Signal strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");

        // Configure time for Firebase
        configTime(0, 0, "pool.ntp.org");
    }
    else
    {
        wifiConnected = false;
        Serial.println("\n✗ WiFi connection failed!");
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status());
    }
}

void initializeFirebase()
{
    if (!wifiConnected)
    {
        Serial.println("Firebase init skipped - WiFi not connected");
        return;
    }

    Serial.println("Initializing Firebase connection...");

    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    // Simple timeout settings
    config.timeout.serverResponse = 10 * 1000;   // 10 seconds
    config.timeout.socketConnection = 10 * 1000; // 10 seconds

    // Initialize Firebase
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    delay(1000); // Give Firebase time to initialize

    // Simple ready check
    if (Firebase.ready())
    {
        Serial.println("✓ Firebase connected successfully!");
        firebaseReady = true;

        // Test upload
        FirebaseJson testJson;
        testJson.set("test", "connection_test");
        testJson.set("timestamp", (int)time(nullptr));

        if (Firebase.setJSON(firebaseData, "/connection_test", testJson))
        {
            Serial.println("✓ Firebase write test successful!");
        }
        else
        {
            Serial.print("✗ Firebase write test failed: ");
            Serial.println(firebaseData.errorReason());
            firebaseReady = false;
        }
    }
    else
    {
        Serial.println("✗ Firebase not ready - will retry later");
        firebaseReady = false;
    }
}

void uploadTelemetryToFirebase()
{
    if (!telemetryReceived || !wifiConnected || !firebaseReady)
    {
        return; // Skip if conditions not met
    }

    // Create JSON payload with essential data
    FirebaseJson json;
    unsigned long timestamp = time(nullptr);

    json.set("timestamp", (int)timestamp);
    json.set("temperature", telemetryData.temperature / 100.0);
    json.set("humidity", telemetryData.humidity);
    json.set("pressure", telemetryData.pressure / 10.0);
    json.set("altitude", telemetryData.altitude / 100.0);
    json.set("battery", telemetryData.battery);
    json.set("lux", telemetryData.lux);
    json.set("uvIndex", telemetryData.uvIndex / 100.0);
    json.set("eCO2", telemetryData.eCO2);
    json.set("TVOC", telemetryData.TVOC);
    json.set("gps_satellites", telemetryData.satellites);
    json.set("gps_latitude", telemetryData.latitude / 100.0);
    json.set("gps_longitude", telemetryData.longitude / 100.0);
    json.set("status", telemetryData.status);

    // Upload to Firebase
    String dataPath = "/telemetry/" + String(timestamp);

    if (Firebase.setJSON(firebaseData, dataPath, json))
    {
        static unsigned long lastSuccessMessage = 0;
        if (millis() - lastSuccessMessage > 30000) // Print success every 30 seconds max
        {
            Serial.println("✓ Firebase upload successful");
            lastSuccessMessage = millis();
        }
        firebaseReady = true;
    }
    else
    {
        Serial.print("✗ Firebase upload failed: ");
        Serial.println(firebaseData.errorReason());

        // Mark Firebase as not ready if upload fails
        if (firebaseData.errorReason().indexOf("SSL") >= 0 ||
            firebaseData.errorReason().indexOf("connection") >= 0)
        {
            firebaseReady = false;
        }
    }
}
