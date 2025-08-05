## ✈️ ESP32 Weather Drone – Flight Controller Overview

This project implements a PID-based flight controller for a custom ESP32-powered weather drone, designed to stabilize flight, handle sensor data, and provide real-time telemetry. The project is structured around FreeRTOS for multitasking and uses WiFi for initial control/testing before transitioning to NRF24L01.

---

## ⚙️ Motor Pin Allocation (PWM Outputs)

| GPIO Pin | Motor Position | Rotation Direction      | Propeller Type | Status    |
| -------- | -------------- | ----------------------- | -------------- | --------- |
| GPIO 27  | Back Left      | Clockwise (CW)          | CW             | ✅ Active |
| GPIO 14  | Front Left     | Counter-Clockwise (CCW) | CCW            | ✅ Active |
| GPIO 12  | Back Right     | Counter-Clockwise (CCW) | CCW            | ✅ Active |
| GPIO 13  | Front Right    | Clockwise (CW)          | CW             | ✅ Active |

> ✅ **Current Status**: Motors are fully operational with RF remote control. ESC calibration occurs automatically on power-on. Toggle switch 1 arms/disarms motors, toggle switch 2 provides emergency stop.

---

## 🧠 Control Architecture

- **Framework**: FreeRTOS with task-based scheduling ✅
- **Servo Control**: Uses `ESP32Servo` library for PWM motor control ✅
- **RF Communication**: NRF24L01 at 5Hz with ACK payload telemetry ✅
- **Motor Control**: X-configuration mixing with toggle switch arming ✅
- **PID Controller**: `SimplePID` planned for implementation:
  - Roll stabilization (next priority)
  - Pitch stabilization (next priority)
  - Yaw control (optional)
- **Sensor Integration**: MPU6050 gyro/accelerometer integration needed

---

## 📡 Communication Strategy

- ✅ **Phase 1 Complete**: Control & telemetry via WiFi dashboard
- ✅ **Phase 2 Complete**: **RF remote control** via NRF24L01 module
- ✅ **Phase 3 Complete**: **Telemetry transmission** to remote using **NRF24L01 acknowledgment payloads**
- ⬜ **Phase 4 Next**: **PID stabilization** with RF remote parameter tuning

> 🛰️ **Current Status**: Full RF communication system operational! Remote sends control commands at 5Hz, drone responds with real-time telemetry via ACK payloads. System includes comprehensive sensor data, motor arming controls, and emergency stop functionality.

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

- ✅ **ESC Control**: Motors respond to joystick inputs with X-configuration mixing
- ✅ **RF Communication**: 5Hz control loop with telemetry feedback operational
- ✅ **Joystick Sensitivity**: Tuned to ±3000 range for precise flight control
- ✅ **Safety Systems**: Toggle switch arming, emergency stop, control timeout protection
- ⬜ **Next Phase**: Begin PID tuning with P term → then add D → finally add I
- ⬜ **PID Testing**: Real-time parameter adjustment via RF remote controls
- ⬜ **Stabilization**: Implement MPU6050 feedback for roll/pitch/yaw control
- ⬜ **Flight Testing**: Progressive testing from basic lift to full stabilization

---

## 📅 Progress Tracking

- ✅ WiFi control tested
- ✅ **FreeRTOS drone implementation complete** (droneFreeRTOS.ino)
- ✅ **RF remote control implemented** (remoteControllerStable.ino)
- ✅ **ESC control with joystick inputs** - Motors respond to remote commands
- ✅ **5Hz RF24 communication** - Stable control at 200ms intervals
- ✅ **Toggle switch arming system** - SW1=ARM, SW2=EMERGENCY_STOP
- ✅ **Comprehensive telemetry system** - BME280, GPS, air quality sensors
- ✅ **Joystick sensitivity tuning** - ±3000 range for precise control
- ✅ **ESC calibration on power-on** - Immediate MAX→MIN calibration sequence
- ⬜ **Implement PID stabilization** - Next critical milestone
- ⬜ **PID tuning with RF remote** - Real-time PID parameter adjustment
- ⬜ **Add MPU6050 integration** - Gyro/accelerometer for stabilization
- ⬜ **Add dynamic altitude hold and GPS switching logic**
