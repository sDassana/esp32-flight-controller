# Weather Drone: NRF24L01 ACK Payload Telemetry System

This project implements a lightweight and efficient **telemetry transmission system** using the NRF24L01 module’s **ACK payload feature**. The system allows the **drone to send sensor and GPS data** back to the remote **immediately after receiving a control packet**, minimizing latency and avoiding extra transmissions.

---

## ✅ How It Works

- The **remote sends control commands** (joystick, toggle states) to the drone via NRF24L01.
- The **drone responds with an ACK payload** containing the latest sensor, GPS, and system data packed in a struct.
- The **remote reads the ACK payload** from each response.
- The **remote uploads the received telemetry** to **Firebase Realtime Database**.
- The system ensures **bidirectional communication** in a single TX/RX cycle without needing separate telemetry transmissions.

---

## 📦 Telemetry Data Structure

We use a tightly packed `struct` to keep data size within NRF24L01's 32-byte limit.

```cpp
#pragma pack(push, 1)
typedef struct {
  int16_t temperature;       // AHT21 Temperature (°C * 100)
  uint8_t humidity;          // AHT21 Humidity (% RH)
  uint16_t pressure;         // BME280 Pressure (hPa * 10)
  uint16_t gas_resistance;   // ENS160 gas sensor value
  uint8_t air_quality_index; // ENS160 AQI (1–5)
  uint16_t uvIndex;          // GUVA-S12SD UV Index (*100)
  uint16_t lightIntensity;   // BH1750 Light intensity (lux)
  int32_t latitude;          // GPS Latitude (*1e6)
  int32_t longitude;         // GPS Longitude (*1e6)
  int16_t altitude;          // Altitude in meters
  uint8_t satellites;        // Number of GPS satellites
  uint16_t batteryVoltage;   // Battery voltage in mV
  uint8_t esp32Temp;         // ESP32 internal temp (°C)
  uint32_t timestamp;        // Timestamp (Unix or millis/1000)
} TelemetryPacket;
#pragma pack(pop)
📏 Total size: 30 bytes — fits within NRF24L01’s 32-byte ACK payload.

🛰 Drone-Side Implementation
On receiving a control packet:

Parse input values (e.g., joystick positions).

Read and pack all sensor values into a TelemetryPacket struct.

Send it using radio.writeAckPayload().


radio.writeAckPayload(1, &packet, sizeof(packet));
Data includes GPS, AHT21, ENS160, BH1750, GUVA-S12SD, BME280, and battery voltage.

📡 Remote-Side Implementation
Send control data (e.g., joystick, toggle states) to drone.

On acknowledgment, check for available ACK payload.

Read TelemetryPacket and decode values.

Upload data to Firebase Realtime Database using Firebase.set() or equivalent functions.

☁️ Firebase Data Structure (Example)
json

{
  "telemetry": {
    "timestamp": 1722262750,
    "temperature": 25.63,
    "humidity": 64,
    "pressure": 1006.3,
    "gas_resistance": 120,
    "aqi": 3,
    "uv_index": 0.83,
    "light_intensity": 315,
    "latitude": 7.2906,
    "longitude": 80.6337,
    "altitude": 553,
    "satellites": 8,
    "battery_voltage": 7430,
    "esp32_temp": 52
  }
}

🔄 System Overview
Module	Task
Drone	Sensor reading, ACK payload packing
Receives control commands from remote
Remote	Sends control signals
Receives ACK payload, uploads to Firebase
Firebase	Stores telemetry, accessible via web dashboard

📊 Web Dashboard
The live telemetry data sent from the drone is visualized on a real-time web dashboard, with sensor interpretation and analytics handled on the web side (not on ESP32) to reduce processing load.

🛠️ Notes
The system avoids sending raw strings — instead, binary-packed data is transmitted using a struct, which is much more compact and efficient.

You can add status flags or error codes if needed using the remaining 2 bytes in the 32-byte payload limit.

```
