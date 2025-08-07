# 📁 Legacy Firmware - Pre-August 7, 2025

This folder contains the previous stable firmware versions that were superseded on August 7, 2025, when the development versions with working motor control were promoted to stable status.

## 📋 Legacy Files

### Drone Firmware

- **`droneFreeRTOS_stable_pre_aug7_2025.ino`** - Previous stable drone firmware
  - Status: Pre-motor control integration
  - Features: Basic sensor suite, telemetry, but no working motor control
  - Superseded by: Development version with motor control fixes

### Remote Controller Firmware

- **`remoteControllerStable_pre_aug7_2025.ino`** - Previous stable remote controller

  - Status: Basic RF24 communication without motor integration
  - Features: Joystick reading, basic telemetry display
  - Superseded by: Development version with virtual throttle system

- **`FastControlRemote_pre_aug7_2025.ino`** - Alternative remote controller
  - Status: Legacy alternative control interface
  - Features: Simplified control system
  - Superseded by: Enhanced remote controller with safety systems

## 🔄 Migration Information

**Migration Date:** August 7, 2025  
**Reason:** Development versions achieved full motor control functionality with compilation fixes

### Key Improvements in New Stable Versions

- ✅ Complete motor control system working
- ✅ RF24 remote control with ESC integration
- ✅ Virtual throttle system with safety controls
- ✅ FreeRTOS multi-threading with dedicated motor task
- ✅ PID stabilization controllers implemented
- ✅ All compilation errors resolved
- ✅ Enhanced safety systems (ARM/DISARM, Emergency Stop)

### What Was Superseded

- ❌ Non-functional motor control in previous stable versions
- ❌ Basic telemetry without motor integration
- ❌ Limited safety systems
- ❌ Compilation issues preventing proper deployment

## 🗃️ Archive Purpose

These files are preserved for:

- Historical reference and development tracking
- Comparison with new implementations
- Fallback option if regression testing is needed
- Documentation of development progression

---

**Note:** These legacy versions should not be used for active development or production deployment. Use the current stable versions in `firmware/stable/` instead.

_Last Updated: August 7, 2025_
