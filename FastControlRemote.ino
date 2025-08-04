/*
 * Complete Remote Controller - Fast Control
 * Increasing control frequency to 200ms (5Hz)
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

// Pin Definitions
#define CE_PIN 4
#define CSN_PIN 5

// Joystick pins
#define JOY1_X_PIN 34
#define JOY1_Y_PIN 35
#define JOY2_X_PIN 36
#define JOY2_Y_PIN 39
#define TOGGLE_SW1_PIN 27
#define TOGGLE_SW2_PIN 14

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

// Firebase objects
FirebaseData firebaseData;
FirebaseAuth auth;
FirebaseConfig config;

// Control packet (8 bytes)
struct ControlPacket
{
    int16_t throttle;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};

// Enhanced telemetry packet (12 bytes) - matches drone
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

ControlPacket controlData;
TelemetryPacket telemetryData;
unsigned long lastControlSend = 0;
unsigned long lastFirebaseUpload = 0;
bool telemetryReceived = false;
bool firebaseReady = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("=== Complete Remote Controller - Fast Control ===");

    // Initialize pins
    pinMode(JOY1_X_PIN, INPUT);
    pinMode(JOY1_Y_PIN, INPUT);
    pinMode(JOY2_X_PIN, INPUT);
    pinMode(JOY2_Y_PIN, INPUT);
    pinMode(TOGGLE_SW1_PIN, INPUT_PULLUP);
    pinMode(TOGGLE_SW2_PIN, INPUT_PULLUP);

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
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(76);
    radio.setAutoAck(true);
    radio.setRetries(15, 15);
    radio.enableDynamicPayloads();
    radio.enableAckPayload();

    radio.stopListening();

    Serial.println("Radio configured successfully!");

    // Initialize WiFi and Firebase
    initializeWiFi();
    initializeFirebase();

    Serial.println("Starting FAST control transmission (5Hz) with telemetry and cloud upload...");
}

void loop()
{
    // Send control data every 200ms (5Hz - drone-like responsiveness)
    if (millis() - lastControlSend > 200)
    {
        readJoystickInputs();
        sendControlData();
        lastControlSend = millis();
    }

    // Upload to Firebase every 10 seconds
    if (millis() - lastFirebaseUpload > 10000 && firebaseReady && telemetryReceived)
    {
        uploadTelemetryToFirebase();
        lastFirebaseUpload = millis();
    }

    delay(10); // Reduced delay for faster response
}

void sendControlData()
{
    bool result = radio.write(&controlData, sizeof(controlData));

    static int packetNumber = 1;
    static int successCount = 0, failCount = 0, telemetryCount = 0;
    static unsigned long lastDetailedPrint = 0;

    if (result)
    {
        successCount++;

        // Check for ACK payload with telemetry
        if (radio.isAckPayloadAvailable())
        {
            uint8_t len = radio.getDynamicPayloadSize();
            if (len == sizeof(telemetryData))
            {
                radio.read(&telemetryData, sizeof(telemetryData));
                telemetryReceived = true;
                telemetryCount++;
            }
        }

        // Print detailed info only every 2 seconds (to avoid spam at 5Hz)
        if (millis() - lastDetailedPrint > 2000 && telemetryReceived)
        {
            Serial.print("Packet #");
            Serial.print(packetNumber);
            Serial.print(" SUCCESS - T:");
            Serial.print(controlData.throttle);
            Serial.print(" R:");
            Serial.print(controlData.roll);
            Serial.print(" P:");
            Serial.print(controlData.pitch);
            Serial.print(" Y:");
            Serial.print(controlData.yaw);
            Serial.print(" | Telemetry - Temp:");
            Serial.print(telemetryData.temperature / 100.0);
            Serial.print("°C Press:");
            Serial.print(telemetryData.pressure / 10.0);
            Serial.print("hPa Hum:");
            Serial.print(telemetryData.humidity);
            Serial.print("% GPS:");
            Serial.print(telemetryData.satellites);
            Serial.print(" sats Batt:");
            Serial.print(telemetryData.battery);
            Serial.println("mV");
            lastDetailedPrint = millis();
        }
    }
    else
    {
        failCount++;
        // Only print failures immediately
        Serial.print("Packet #");
        Serial.print(packetNumber);
        Serial.println(" FAILED");
    }

    packetNumber++;

    // Print status every 50 packets (10 seconds at 5Hz)
    if (packetNumber % 50 == 0)
    {
        Serial.print("Fast Control Status - Success: ");
        Serial.print(successCount);
        Serial.print(", Failed: ");
        Serial.print(failCount);
        Serial.print(", Telemetry: ");
        Serial.print(telemetryCount);
        Serial.print(", Success rate: ");
        if (successCount + failCount > 0)
        {
            Serial.print((successCount * 100) / (successCount + failCount));
        }
        Serial.print("%, Rate: 5Hz, Firebase: ");
        Serial.println(firebaseReady ? "Connected" : "Disconnected");
        successCount = 0;
        failCount = 0;
        telemetryCount = 0;
    }
}

void readJoystickInputs()
{
    int joy1_x = analogRead(JOY1_X_PIN);
    int joy1_y = analogRead(JOY1_Y_PIN);
    int joy2_x = analogRead(JOY2_X_PIN);
    int joy2_y = analogRead(JOY2_Y_PIN);

    controlData.throttle = map(joy1_y, 0, 4095, -1000, 1000);
    controlData.yaw = map(joy1_x, 0, 4095, -1000, 1000);
    controlData.roll = map(joy2_x, 0, 4095, -1000, 1000);
    controlData.pitch = map(joy2_y, 0, 4095, -1000, 1000);
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
        Serial.println("\nWiFi connected!");
        configTime(0, 0, "pool.ntp.org");
    }
    else
    {
        Serial.println("\nWiFi connection failed!");
    }
}

void initializeFirebase()
{
    if (WiFi.status() != WL_CONNECTED)
        return;

    config.host = FIREBASE_HOST;
    config.signer.tokens.legacy_token = FIREBASE_AUTH;

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    if (Firebase.ready())
    {
        Serial.println("Firebase connected!");
        firebaseReady = true;
    }
}

void uploadTelemetryToFirebase()
{
    if (!Firebase.ready() || !telemetryReceived)
        return;

    FirebaseJson json;
    json.set("timestamp", (int)time(nullptr));
    json.set("temperature", telemetryData.temperature / 100.0);
    json.set("humidity", telemetryData.humidity);
    json.set("pressure", telemetryData.pressure / 10.0);
    json.set("battery", telemetryData.battery);
    json.set("control_throttle", controlData.throttle);
    json.set("control_roll", controlData.roll);
    json.set("control_pitch", controlData.pitch);
    json.set("control_yaw", controlData.yaw);
    json.set("status", telemetryData.status);
    json.set("control_rate", "5Hz");

    if (Firebase.setJSON(firebaseData, "/telemetry/latest", json))
    {
        Serial.println("✓ Data uploaded to Firebase!");
    }
    else
    {
        Serial.println("✗ Firebase upload failed");
    }
}
