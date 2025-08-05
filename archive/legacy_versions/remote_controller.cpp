#include <Arduino.h>
#include <RF24.h>

// NRF24L01 setup
#define CE_PIN 4
#define CSN_PIN 5
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "DRONE";

// Telemetry group enum
enum TelemetryGroup
{
    GROUP_A,
    GROUP_B,
    GROUP_C,
    GROUP_D
};

// Structure for essential telemetry
struct TelemetryEssential
{
    uint32_t timestamp;
    float altitude;
    float latitude;
    float longitude;
};

struct TelemetryGroupA
{
    float temperature;
    float humidity;
    float pressure;
};
struct TelemetryGroupB
{
    uint16_t co2;
    uint16_t tvoc;
};
struct TelemetryGroupC
{
    float uv;
    uint16_t light;
};

struct TelemetryPayload
{
    TelemetryEssential essential;
    union
    {
        TelemetryGroupA groupA;
        TelemetryGroupB groupB;
        TelemetryGroupC groupC;
        uint8_t reserved[8];
    } group;
    uint8_t groupId;
};

// Buffer for round-robin telemetry
TelemetryEssential essential;
TelemetryGroupA groupA;
TelemetryGroupB groupB;
TelemetryGroupC groupC;
uint32_t lastRoundTimestamp = 0;

void setup()
{
    Serial.begin(115200);
    radio.begin();
    radio.setPALevel(RF24_PA_HIGH);
    radio.setChannel(76);
    radio.openWritingPipe(address);
    radio.stopListening();
}

void sendControlSignal()
{
    uint8_t dummyCommand[32] = {0}; // Replace with actual control data
    radio.write(&dummyCommand, sizeof(dummyCommand));
}

void loop()
{
    sendControlSignal();
    // Listen for ACK payload
    if (radio.isAckPayloadAvailable())
    {
        TelemetryPayload payload;
        radio.read(&payload, sizeof(payload));
        essential = payload.essential;
        switch (payload.groupId)
        {
        case GROUP_A:
            groupA = payload.group.groupA;
            break;
        case GROUP_B:
            groupB = payload.group.groupB;
            break;
        case GROUP_C:
            groupC = payload.group.groupC;
            break;
        case GROUP_D:
            // Reserved
            break;
        }
        // After receiving all groups, print full telemetry
        if (payload.groupId == GROUP_C)
        { // End of round
            Serial.println("--- Full Telemetry Packet ---");
            Serial.print("Timestamp: ");
            Serial.println(essential.timestamp);
            Serial.print("Altitude: ");
            Serial.println(essential.altitude);
            Serial.print("Latitude: ");
            Serial.println(essential.latitude, 6);
            Serial.print("Longitude: ");
            Serial.println(essential.longitude, 6);
            Serial.print("Temperature: ");
            Serial.println(groupA.temperature);
            Serial.print("Humidity: ");
            Serial.println(groupA.humidity);
            Serial.print("Pressure: ");
            Serial.println(groupA.pressure);
            Serial.print("CO2: ");
            Serial.println(groupB.co2);
            Serial.print("TVOC: ");
            Serial.println(groupB.tvoc);
            Serial.print("UV: ");
            Serial.println(groupC.uv);
            Serial.print("Light: ");
            Serial.println(groupC.light);
            Serial.println("----------------------------");
            lastRoundTimestamp = essential.timestamp;
        }
    }
    delay(100); // Adjust as needed
}
