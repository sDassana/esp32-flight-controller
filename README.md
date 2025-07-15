Flight Controller for Custom DIY Drone using ESP32 (NodeMCU ESP-WROOM-32)

## Hardware Overview:

- Microcontroller: ESP32 (NodeMCU ESP-WROOM-32)
- Communication Module: NRF24L01 + PA + LNA (via SPI)

  - CE: GPIO 4
  - CSN: GPIO 5
  - SCK: GPIO 18
  - MOSI: GPIO 23
  - MISO: GPIO 19

- ESC Control (PWM Outputs to Brushless Motors):

  - ESC1: GPIO 13
  - ESC2: GPIO 12
  - ESC3: GPIO 14
  - ESC4: GPIO 27

- I2C Bus (Shared SDA/SCL):

  - SDA: GPIO 21
  - SCL: GPIO 22

- I2C Devices:

  - PCA9548A I2C Multiplexer (Address: 0x70; VL53L0X sensors connected to channels 0–3)
  - BMP280 (Barometric pressure + temperature sensor, Addr: 0x76 or 0x77)
  - MPU6050 (Gyroscope + Accelerometer, Addr: 0x68)
  - VL53L0X x4 (Time-of-Flight distance sensors, all with same address, handled via PCA9548A)

- RGB LEDs (Common Cathode for Navigation Lights):
  - R/G/B pins: GPIOs 25, 26, 32, 33, 14, 27 (exact mapping to be defined)
  - Use PWM or digitalWrite for color control

## Power:

- All components powered via filtered power supply (5V and 3.3V regulated lines)
- NRF24L01 is powered via a dedicated filtered 3.3V module (NOT from ESP32)

## Goals:

- Receive control signals from remote via NRF24L01
- Read IMU data from MPU6050 for orientation
- Use BMP280 for altitude estimation
- Read proximity from 4x VL53L0X sensors via PCA9548A
- Control ESCs via PWM based on input and sensor feedback (PID control)
- Light up RGB LEDs based on status/mode
- Build modular, non-blocking code using state machine or task scheduler

## Setup Requirements:

- Use Wire.h for I2C communication
- Use SPI.h for NRF24L01
- Include Adafruit_BMP280, Adafruit_MPU6050, VL53L0X libraries
- Use RF24 library for wireless communication
- Use PCA9548A-compatible I2C switching logic
- Consider FreeRTOS tasks or timer interrupts for consistent loop timing

# ESP32 Weather Drone Firmware

This firmware powers a custom-built drone using an ESP32 (NodeMCU ESP-WROOM-32). It supports sensor fusion, telemetry, and wireless control.

---

## 🚁 Components Overview

| Component       | Purpose                               |
| --------------- | ------------------------------------- |
| ESP32           | Main flight controller + WiFi         |
| MPU6050         | Roll, pitch, yaw (IMU)                |
| BMP280 / BME280 | Barometric altitude + temperature     |
| VL53L0X ×4      | Obstacle detection (via PCA9548A mux) |
| CCS811 + AHT21  | CO₂, TVOC, temp, humidity             |
| BH1750 + GUVA   | Light + UV intensity                  |
| GPS NEO-6M      | Position and velocity                 |
| NRF24L01+       | RF telemetry and remote control       |
| ESCs + Motors   | Propulsion (EMAX 980KV motors)        |
| RGB LEDs        | Navigation + status indicators        |
| Buzzer          | Status sound                          |

---

## 📡 Communication

- **NRF24L01+** used for telemetry & command between drone and remote
- **I2C** used for most sensors with level-shifter and pull-ups

---

## ⚙️ Key Features

- Full **PID-based stabilization** (Roll, Pitch, Yaw, Altitude)
- **Live sensor data** sent via NRF
- ESCs armed at boot, sensors calibrated dynamically
- **4-layer PCB** for compact, efficient layout (ESP + MUX underside)

---

## 🛠️ Build Environment

- **Board:** `esp32doit-devkit-v1`
- **Framework:** Arduino
- **Tool:** PlatformIO or Arduino IDE
