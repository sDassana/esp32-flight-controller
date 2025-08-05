# 🔧 Firmware Directory

This directory contains the main firmware files for the ESP32 flight controller system.

## 📁 Directory Structure

### 🚁 `/drone`

Contains the main drone flight controller firmware:

- `droneFreeRTOS.ino` - **Current production firmware** with FreeRTOS multi-threading, RF remote control, and comprehensive sensor integration

### 🎮 `/remote`

Contains the remote controller firmware:

- `remoteControllerStable.ino` - **Current stable remote controller** with dual joystick support and telemetry display
- `FastControlRemote.ino` - Alternative remote controller implementation

## 🚀 Getting Started

1. **For Drone**: Upload `drone/droneFreeRTOS.ino` to the ESP32 flight controller
2. **For Remote**: Upload `remote/remoteControllerStable.ino` to the ESP32 remote controller
3. **Configure**: Ensure matching RF24 channel settings (Channel 76)
4. **Test**: Verify communication and control response before flight

## ⚠️ Safety Notes

- Always test firmware changes in a safe environment
- Verify all sensor readings before flight operations
- Ensure emergency stop functionality is working
- Keep backup copies of working firmware versions
