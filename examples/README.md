# 🔬 Examples Directory

This directory contains example code, sensor tests, and component verification programs for development and troubleshooting.

## 📁 Directory Structure

### 🔧 `/sensor_tests`

Individual sensor test programs for verification and calibration:

- `BMP280.cpp` - Environmental sensor (pressure, temperature, humidity) testing
- `MPU6050.cpp` - IMU sensor (gyroscope, accelerometer) testing
- `MPU6050 + BMP280.cpp` - Combined sensor integration testing
- `PCA9548A.cpp` - I2C multiplexer testing for multiple VL53L0X sensors
- `gps_test.cpp` - GPS module testing and NMEA data parsing

### ⚡ `/component_tests`

System component and communication testing:

- `motor_test.cpp` - ESC and motor control testing and calibration
- `Wifi Control.cpp` - WiFi connectivity and web interface testing

## 🎯 Usage

These examples are designed for:

- **Hardware verification** - Confirm sensors and components are working
- **Troubleshooting** - Isolate issues with specific components
- **Development** - Reference implementations for new features
- **Calibration** - Sensor calibration and configuration procedures

## 🚀 Getting Started

1. **Select test program** based on component to verify
2. **Upload to ESP32** using PlatformIO or Arduino IDE
3. **Open Serial Monitor** to view test results and data
4. **Follow test instructions** for component-specific procedures

## ⚠️ Safety Notes

- Remove propellers when testing motor/ESC functions
- Test individual components before integration
- Verify power supply stability during testing
- Document calibration values for production use
