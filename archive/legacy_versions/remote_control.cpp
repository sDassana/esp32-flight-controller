#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

// NRF24L01 Pins
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5
#define NRF_SCK_PIN 18
#define NRF_MOSI_PIN 23
#define NRF_MISO_PIN 19

// Joystick 1 Pins
#define JOY1_X_PIN 33   // Throttle
#define JOY1_Y_PIN 25   // Yaw
#define JOY1_BTN_PIN 26 // Button 1 (Emergency stop)

// Joystick 2 Pins
#define JOY2_X_PIN 34   // Pitch
#define JOY2_Y_PIN 35   // Roll
#define JOY2_BTN_PIN 32 // Button 2 (Arm/Disarm)

// Status LED (optional)
#define LED_PIN 2

// Objects
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

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

// ACK Payload Structure (received from drone)
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

// NRF24L01 communication settings
const byte address[6] = "00001";

// Deadband for joystick centering
const int deadband = 50;

// Button debouncing
bool btn1_last = false;
bool btn2_last = false;
unsigned long btn1_debounce = 0;
unsigned long btn2_debounce = 0;
const unsigned long debounce_delay = 50;

int16_t applyDeadband(int16_t value, int16_t center = 0)
{
    if (abs(value - center) < deadband)
        return center;
    return value;
}

void displayTelemetry(const AckPayload &ack)
{
    Serial.printf("=== TELEMETRY ===\n");
    Serial.printf("Altitude: %.2f m\n", ack.altitude);
    Serial.printf("GPS: %.6f, %.6f\n", ack.latitude, ack.longitude);

    switch (ack.sensorType)
    {
    case 0: // BME280 temperature and humidity
        Serial.printf("Weather - Temp: %.2f°C | Humidity: %.2f%%\n",
                      ack.data.mpu.ax, ack.data.mpu.ay);
        break;

    case 1: // UV/Light
        Serial.printf("Light - UV Index: %.2f | Lux: %.2f\n",
                      ack.data.light.uv, ack.data.light.lux);
        break;

    case 2: // Air Quality
        Serial.printf("Air - TVOC: %.2f ppb | CO2: %.2f ppm\n",
                      ack.data.air.tvoc, ack.data.air.co2);
        break;
    }
    Serial.printf("=================\n");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32 Remote Control - Starting...");

    // Button pins
    pinMode(JOY1_BTN_PIN, INPUT_PULLUP);
    pinMode(JOY2_BTN_PIN, INPUT_PULLUP);

    // Status LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // SPI Init
    SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);

    // NRF24L01 Init
    if (!radio.begin())
    {
        Serial.println("NRF24L01 not found!");
        while (1)
        {
            digitalWrite(LED_PIN, HIGH);
            delay(500);
            digitalWrite(LED_PIN, LOW);
            delay(500);
        }
    }

    radio.openWritingPipe(address);
    radio.setChannel(76); // Channel 76 to avoid WiFi interference
    radio.setPALevel(RF24_PA_LOW);
    radio.enableAckPayload(); // Enable ACK payload feature
    radio.stopListening();

    Serial.println("NRF24L01 initialized on channel 76 with ACK payload enabled");
    Serial.println("Remote control ready!");

    // LED on = ready
    digitalWrite(LED_PIN, HIGH);
}

void loop()
{
    JoystickData data;

    // Read joystick values
    int joy1_x = analogRead(JOY1_X_PIN); // Throttle (0-4095 on ESP32)
    int joy1_y = analogRead(JOY1_Y_PIN); // Yaw
    int joy2_x = analogRead(JOY2_X_PIN); // Pitch
    int joy2_y = analogRead(JOY2_Y_PIN); // Roll

    // Scale ESP32 ADC (0-4095) to expected range (0-1023 for throttle, -512 to 512 for others)
    data.throttle = map(joy1_x, 0, 4095, 0, 1023);
    data.yaw = applyDeadband(map(joy1_y, 0, 4095, -512, 512));
    data.pitch = applyDeadband(map(joy2_x, 0, 4095, -512, 512));
    data.roll = applyDeadband(map(joy2_y, 0, 4095, -512, 512));

    // Read buttons with debouncing
    bool btn1_current = !digitalRead(JOY1_BTN_PIN); // Inverted (pullup)
    bool btn2_current = !digitalRead(JOY2_BTN_PIN);

    // Button 1 debouncing
    if (btn1_current != btn1_last)
    {
        btn1_debounce = millis();
    }
    if ((millis() - btn1_debounce) > debounce_delay)
    {
        data.btn1 = btn1_current;
    }
    btn1_last = btn1_current;

    // Button 2 debouncing
    if (btn2_current != btn2_last)
    {
        btn2_debounce = millis();
    }
    if ((millis() - btn2_debounce) > debounce_delay)
    {
        data.btn2 = btn2_current;
    }
    btn2_last = btn2_current;

    // Send data and check for ACK payload
    bool result = radio.write(&data, sizeof(data));

    // Check if ACK payload was received
    if (result && radio.isAckPayloadAvailable())
    {
        AckPayload ackPayload;
        radio.read(&ackPayload, sizeof(ackPayload));

        // Display telemetry every 500ms to avoid spam
        static unsigned long lastTelemetryDisplay = 0;
        if (millis() - lastTelemetryDisplay > 500)
        {
            displayTelemetry(ackPayload);
            lastTelemetryDisplay = millis();
        }
    }

    // Debug output every 100ms
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 100)
    {
        Serial.printf("Control: T:%d Y:%d P:%d R:%d B1:%d B2:%d | TX:%s\n",
                      data.throttle, data.yaw, data.pitch, data.roll,
                      data.btn1, data.btn2, result ? "OK" : "FAIL");
        last_debug = millis();
    }

    // Blink LED to show transmission
    if (result)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(5);
        digitalWrite(LED_PIN, LOW);
    }

    delay(20); // 50Hz update rate
}
