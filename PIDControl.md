## ✈️ ESP32 Weather Drone – Flight Controller Overview

This project implements a PID-based flight controller for a custom ESP32-powered weather drone, designed to stabilize flight, handle sensor data, and provide real-time telemetry. The project is structured around FreeRTOS for multitasking and uses WiFi for initial control/testing before transitioning to NRF24L01.

---

## ⚙️ Motor Pin Allocation (PWM Outputs)

| GPIO Pin | Motor Position | Rotation Direction      | Propeller Type |
| -------- | -------------- | ----------------------- | -------------- |
| GPIO 27  | Back Left      | Clockwise (CW)          | CW             |
| GPIO 14  | Front Left     | Counter-Clockwise (CCW) | CCW            |
| GPIO 12  | Back Right     | Counter-Clockwise (CCW) | CCW            |
| GPIO 13  | Front Right    | Clockwise (CW)          | CW             |

> ⚠️ Motors are physically fixed to these pins due to soldered PCB. Rotation direction is adjusted via ESC wiring and propeller type.

---

## 🧠 Control Architecture

- **Framework**: FreeRTOS with task-based scheduling
- **Servo Control**: Uses `ESP32Servo` library for PWM motor control
- **PID Controller**: `SimplePID` used with separate loops for:
  - Roll
  - Pitch
  - (Optional) Yaw
- **Motor Mixing**: Standard X-configuration quadcopter mixing

---

## 📡 Communication Strategy

- **Initial Phase**: Control & telemetry via WiFi dashboard
- **Later Phase**:
  - **Remote control** via NRF24L01 module
  - **Telemetry transmission** to remote using **NRF24L01 acknowledgment payloads**

> 🛰️ ACK-based telemetry will enable the remote unit to receive live flight data with minimal bandwidth overhead. This will be implemented after basic stabilization is verified.

---

## ⬆️ Altitude Control

- **Initial Lift-off Range**: Motors begin lift at ~1550–1600 µs PWM
- **Altitude Source**:
  - **Before GPS lock**: Use **BME280** barometric pressure
  - **After GPS lock**: Use GPS-based altitude for better long-term stability

---

## 🔍 Sensor Suite

| Sensor Module      | Function                                      |
| ------------------ | --------------------------------------------- |
| **MPU6050**        | Gyro + Accelerometer (orientation)            |
| **BME280**         | Pressure + Temp + Humidity                    |
| **VL53L0X x4**     | Short-range ToF sensors for obstacle distance |
| **ENS160 + AHT21** | Air quality (eCO₂, TVOC) + Temp/Humidity      |
| **GPS Module**     | Latitude, Longitude, Altitude, Speed          |
| **PCA9548A MUX**   | I2C multiplexer for sensor management         |

---

## 🛠️ Development Notes

- Begin tuning PID with only P term → then add D → finally add I
- Lift-off control testing done via WiFi sliders before enabling full stabilization
- Telemetry and PID output logs available through the WiFi dashboard
- NRF telemetry and remote control features are staged for later development

---

## 📅 Next Milestones

- ✅ WiFi control tested
- ⬜ Implement basic stabilization using PID
- ⬜ Switch to RF remote input via NRF24L01
- ⬜ Transmit telemetry using NRF24L01 with ACK payloads
- ⬜ Add dynamic altitude hold and GPS switching logic
