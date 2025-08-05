/*
 * Weather Drone: NRF24L01 ACK Payload Telemetry System
 * Drone-Side Implementation
 *
 * This code implements telemetry transmission using NRF24L01's ACK payload feature.
 * The drone receives control commands and responds with sensor data in the ACK payload.
 */

// Testing SimpleTelemetryDrone.ino
#include "../SimpleTelemetryDrone.ino"
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>

// Sensor Libraries
#include <Adafruit_AHTX0.h>   // AHT21 Temperature/Humidity
#include <Adafruit_BME280.h>  // BME280 Pressure sensor
#include <BH1750.h>           // BH1750 Light sensor
#include <ScioSense_ENS160.h> // ENS160 Gas sensor

// Pin Definitions
#define CE_PIN 4         // NRF24L01 CE pin (from README)
#define CSN_PIN 5        // NRF24L01 CSN pin (from README)
#define GPS_RX_PIN 16    // GPS TX connected to ESP32 GPIO 16 (Hardware Serial2 RX)
#define UV_SENSOR_PIN 36 // GUVA-S12SD UV sensor analog pin

// NRF24L01 Configuration
RF24 radio(CE_PIN, CSN_PIN);
const byte address[][6] = {"00001", "00002"};

// Sensor Objects
Adafruit_AHTX0 aht;
Adafruit_BME280 bme;
BH1750 lightMeter;
ScioSense_ENS160 ens160(ENS160_I2CADDR_1); // ENS160 sensor object

// Hardware Serial for GPS (using Serial2)
#define GPS_SERIAL Serial2

// Control Input Structure (received from remote)
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

// Telemetry Structure (sent back as ACK payload)
#pragma pack(push, 1)
typedef struct
{
  int16_t temperature;       // AHT21 Temperature (°C * 100)
  uint8_t humidity;          // AHT21 Humidity (% RH)
  uint16_t pressure;         // BME280 Pressure (hPa * 10)
  uint16_t gas_resistance;   // ENS160 gas sensor value (placeholder for now)
  uint8_t air_quality_index; // ENS160 AQI (1–5) (placeholder for now)
  uint16_t uvIndex;          // GUVA-S12SD UV Index (*100) (placeholder for now)
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
unsigned long lastSensorRead = 0;
unsigned long lastGPSRead = 0;
unsigned long lastBatteryRead = 0;
unsigned long lastStatusPrint = 0; // For periodic status messages

// GPS Variables
String gpsData = "";
bool gpsNewData = false;
float gpsLat = 0.0, gpsLon = 0.0, gpsAlt = 0.0;
int gpsSats = 0;

// Battery Monitoring
#define BATTERY_PIN 35            // From README: Battery monitoring uses GPIO 35
#define VOLTAGE_DIVIDER_RATIO 5.0 // From README: voltage divider with ratio 0.2, so multiply by 5.0

void setup()
{
  Serial.begin(115200);
  Serial.println("Weather Drone Telemetry System Starting...");

  // Initialize I2C
  Wire.begin();

  // Initialize GPS (Hardware Serial2)
  GPS_SERIAL.begin(9600, SERIAL_8N1, GPS_RX_PIN, -1); // RX=16, TX not used (GPS only transmits)

  // Initialize Sensors
  initializeSensors();

  // Initialize NRF24L01
  initializeRadio();

  // Initialize telemetry packet with default values
  memset(&telemetryData, 0, sizeof(telemetryData));

  Serial.println("System Ready - Waiting for control signals...");
  Serial.print("Telemetry packet size: ");
  Serial.print(sizeof(telemetryData));
  Serial.println(" bytes");
}

void loop()
{
  // Read sensors periodically (every 100ms)
  if (millis() - lastSensorRead > 100)
  {
    readSensors();
    lastSensorRead = millis();
  }

  // Read GPS data (every 1000ms)
  if (millis() - lastGPSRead > 1000)
  {
    readGPS();
    lastGPSRead = millis();
  }

  // Read battery voltage (every 500ms)
  if (millis() - lastBatteryRead > 500)
  {
    readBatteryVoltage();
    lastBatteryRead = millis();
  }

  // Check for incoming control data
  if (radio.available())
  {
    handleControlData();
  }

  // Print periodic status (every 5 seconds)
  if (millis() - lastStatusPrint > 5000)
  {
    Serial.print("Drone status - Running for ");
    Serial.print(millis() / 1000);
    Serial.print("s, Radio listening: ");
    Serial.print(radio.isListening());
    Serial.print(", Available: ");
    Serial.println(radio.available());
    lastStatusPrint = millis();
  }

  delay(10); // Small delay to prevent overwhelming the system
}

void initializeSensors()
{
  Serial.println("Initializing sensors...");

  // Initialize AHT21
  if (!aht.begin())
  {
    Serial.println("Could not find AHT21 sensor!");
  }
  else
  {
    Serial.println("AHT21 sensor found!");
  }

  // Initialize BME280
  if (!bme.begin(0x76))
  { // Try address 0x76 first
    if (!bme.begin(0x77))
    { // Then try 0x77
      Serial.println("Could not find BME280 sensor!");
    }
    else
    {
      Serial.println("BME280 sensor found at 0x77!");
    }
  }
  else
  {
    Serial.println("BME280 sensor found at 0x76!");
  }

  // Initialize BH1750
  if (lightMeter.begin())
  {
    Serial.println("BH1750 light sensor found!");
  }
  else
  {
    Serial.println("Could not find BH1750 sensor!");
  }

  // Initialize ENS160 with error handling
  Serial.println("Attempting to initialize ENS160...");
  delay(100); // Give I2C bus time to settle

  if (ens160.begin())
  {
    Serial.println("ENS160 gas sensor found!");
    // Set operating mode to standard
    if (ens160.setMode(ENS160_OPMODE_STD))
    {
      Serial.println("ENS160 mode set successfully!");
    }
    else
    {
      Serial.println("ENS160 mode setting failed, but continuing...");
    }
  }
  else
  {
    Serial.println("Could not find ENS160 sensor! Continuing without it...");
  }

  // Configure battery monitoring
  pinMode(BATTERY_PIN, INPUT);

  // Configure UV sensor
  pinMode(UV_SENSOR_PIN, INPUT);
}

void initializeRadio()
{
  Serial.println("Initializing NRF24L01...");

  if (!radio.begin())
  {
    Serial.println("NRF24L01 radio hardware not responding!");
    while (1)
    {
      delay(1000);
    }
  }

  // Configure radio - Drone receives control data and sends telemetry via ACK
  radio.openWritingPipe(address[0]);    // For sending (not used much in ACK mode)
  radio.openReadingPipe(1, address[1]); // Receive control data from remote

  radio.setPALevel(RF24_PA_HIGH);  // Set power level
  radio.setDataRate(RF24_250KBPS); // Lower data rate for better range
  radio.setChannel(76);            // Set channel
  radio.setRetries(5, 15);         // Retry settings

  // Enable ACK payloads
  radio.enableAckPayload();

  // Start listening for control data
  radio.startListening();

  // Pre-load an initial ACK payload (this is important for first transmission)
  radio.writeAckPayload(1, &telemetryData, sizeof(telemetryData));
  Serial.println("Initial ACK payload loaded");

  Serial.println("NRF24L01 initialized successfully!");
  Serial.println("Radio details:");
  radio.printDetails();
  Serial.println("Drone ready to receive control data...");
}

void readSensors()
{
  // Read AHT21 (Temperature and Humidity)
  sensors_event_t humidity_event, temp_event;
  aht.getEvent(&humidity_event, &temp_event);

  telemetryData.temperature = (int16_t)(temp_event.temperature * 100);  // °C * 100
  telemetryData.humidity = (uint8_t)(humidity_event.relative_humidity); // % RH

  // Read BME280 (Pressure)
  float pressure = bme.readPressure() / 100.0F;       // Convert Pa to hPa
  telemetryData.pressure = (uint16_t)(pressure * 10); // hPa * 10

  // Read BH1750 (Light Intensity)
  float lux = lightMeter.readLightLevel();
  telemetryData.lightIntensity = (uint16_t)lux;

  // Read GUVA-S12SD UV sensor (GPIO 36)
  int uvRaw = analogRead(UV_SENSOR_PIN);
  float uvVoltage = (uvRaw * 3.3) / 4095.0; // Convert ADC to voltage
  // GUVA-S12SD: UV Index = uvVoltage / 0.1 (approximately)
  float uvIndex = uvVoltage / 0.1;
  telemetryData.uvIndex = (uint16_t)(uvIndex * 100); // UV Index * 100

  // Read ESP32 internal temperature
  telemetryData.esp32Temp = (uint8_t)temperatureRead();

  // Update timestamp
  telemetryData.timestamp = millis() / 1000;

  // Read ENS160 gas sensor
  if (ens160.available())
  {
    ens160.measure(true);    // Take measurement
    ens160.measureRaw(true); // Take raw measurement

    telemetryData.gas_resistance = ens160.getTVOC();   // TVOC in ppb
    telemetryData.air_quality_index = ens160.getAQI(); // AQI (1-5)
  }
  else
  {
    // Fallback values if ENS160 not ready
    telemetryData.gas_resistance = 0;
    telemetryData.air_quality_index = 0;
  }
}

void readGPS()
{
  // Read GPS data from Hardware Serial2
  while (GPS_SERIAL.available())
  {
    char c = GPS_SERIAL.read();
    gpsData += c;

    if (c == '\n')
    {
      parseGPSData();
      gpsData = "";
    }
  }

  // Update telemetry with GPS data
  telemetryData.latitude = (int32_t)(gpsLat * 1e6);  // Latitude * 1e6
  telemetryData.longitude = (int32_t)(gpsLon * 1e6); // Longitude * 1e6
  telemetryData.altitude = (int16_t)gpsAlt;          // Altitude in meters
  telemetryData.satellites = gpsSats;                // Number of satellites
}

void parseGPSData()
{
  // Simple NMEA parsing for GPGGA sentences
  if (gpsData.startsWith("$GPGGA") || gpsData.startsWith("$GNGGA"))
  {
    // Parse basic GPS data
    // This is a simplified parser - you might want to use a GPS library
    int commaCount = 0;
    String field = "";

    for (int i = 0; i < gpsData.length(); i++)
    {
      if (gpsData[i] == ',')
      {
        processGPSField(commaCount, field);
        field = "";
        commaCount++;
      }
      else
      {
        field += gpsData[i];
      }
    }
    gpsNewData = true;
  }
}

void processGPSField(int fieldNum, String field)
{
  switch (fieldNum)
  {
  case 2: // Latitude
    if (field.length() > 0)
    {
      gpsLat = convertDMSToDecimal(field);
    }
    break;
  case 4: // Longitude
    if (field.length() > 0)
    {
      gpsLon = convertDMSToDecimal(field);
    }
    break;
  case 7: // Number of satellites
    if (field.length() > 0)
    {
      gpsSats = field.toInt();
    }
    break;
  case 9: // Altitude
    if (field.length() > 0)
    {
      gpsAlt = field.toFloat();
    }
    break;
  }
}

float convertDMSToDecimal(String dms)
{
  // Convert DDMM.MMMM or DDDMM.MMMM to decimal degrees
  if (dms.length() < 7)
    return 0.0;

  float degrees, minutes;
  if (dms.length() >= 10)
  { // Longitude format DDDMM.MMMM
    degrees = dms.substring(0, 3).toFloat();
    minutes = dms.substring(3).toFloat();
  }
  else
  { // Latitude format DDMM.MMMM
    degrees = dms.substring(0, 2).toFloat();
    minutes = dms.substring(2).toFloat();
  }

  return degrees + (minutes / 60.0);
}

void readBatteryVoltage()
{
  // Read battery voltage through voltage divider
  int adcValue = analogRead(BATTERY_PIN);
  float voltage = (adcValue * 3.3 / 4095.0) * VOLTAGE_DIVIDER_RATIO;
  telemetryData.batteryVoltage = (uint16_t)(voltage * 1000); // Convert to mV
}

void handleControlData()
{
  // Check if control packet is available and read it
  if (radio.available())
  {
    uint8_t len = radio.getDynamicPayloadSize();

    if (len == sizeof(controlData))
    {
      radio.read(&controlData, sizeof(controlData));

      Serial.print("Control received - Throttle: ");
      Serial.print(controlData.throttle);
      Serial.print(", Roll: ");
      Serial.print(controlData.roll);
      Serial.print(", Pitch: ");
      Serial.print(controlData.pitch);
      Serial.print(", Yaw: ");
      Serial.println(controlData.yaw);

      // Prepare and send telemetry as ACK payload
      sendTelemetryACK();

      // Process control data (this is where you'd apply the control inputs to your flight controller)
      processControlInputs();
    }
    else
    {
      Serial.print("Control packet size mismatch. Expected: ");
      Serial.print(sizeof(controlData));
      Serial.print(", Got: ");
      Serial.println(len);
      // Clear the payload
      uint8_t dummy[32];
      radio.read(&dummy, len);
    }
  }
}

void sendTelemetryACK()
{
  // Send telemetry data as ACK payload
  bool ackResult = radio.writeAckPayload(1, &telemetryData, sizeof(telemetryData));

  Serial.print("ACK payload loaded: ");
  Serial.print(ackResult ? "SUCCESS" : "FAILED");
  Serial.print(" (Size: ");
  Serial.print(sizeof(telemetryData));
  Serial.println(" bytes)");

  Serial.println("Telemetry sent as ACK payload:");
  printTelemetryData();
}

void processControlInputs()
{
  // This is where you would process the control inputs
  // For now, just log the values
  Serial.println("Processing control inputs...");

  // Extract switch states
  bool switch1 = controlData.switches & 0x01;
  bool switch2 = controlData.switches & 0x02;
  bool switch3 = controlData.switches & 0x04;
  bool switch4 = controlData.switches & 0x08;

  Serial.print("Switches: ");
  Serial.print(switch1);
  Serial.print(" ");
  Serial.print(switch2);
  Serial.print(" ");
  Serial.print(switch3);
  Serial.print(" ");
  Serial.print(switch4);
  Serial.print(" ");
  Serial.print("Mode: ");
  Serial.println(controlData.mode);
}

void printTelemetryData()
{
  Serial.println("--- Telemetry Data ---");
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
  Serial.println(telemetryData.longitude / 1e6, 6);
  Serial.print("Altitude: ");
  Serial.print(telemetryData.altitude);
  Serial.println("m");
  Serial.print("Satellites: ");
  Serial.println(telemetryData.satellites);
  Serial.print("Battery: ");
  Serial.print(telemetryData.batteryVoltage);
  Serial.println("mV");
  Serial.print("ESP32 Temp: ");
  Serial.print(telemetryData.esp32Temp);
  Serial.println("°C");
  Serial.print("Timestamp: ");
  Serial.println(telemetryData.timestamp);
  Serial.print("Packet size: ");
  Serial.print(sizeof(telemetryData));
  Serial.println(" bytes");
  Serial.println("--------------------");
}

// Function to test telemetry without radio (for debugging)
void testTelemetryOnly()
{
  readSensors();
  readGPS();
  readBatteryVoltage();
  printTelemetryData();
  delay(2000);
}
