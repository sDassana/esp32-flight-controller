# 🔧 Firmware Directory

This directory contains the firmware files for the ESP32 flight controller system, organized by stability and development status.

**Last Updated:** August 7, 2025 - Stable firmware updated with working motor control system

## 📁 Directory Structure

### ✅ **Stable** (Production Ready)

#### 🚁 `/stable/drone/`

**Production drone flight controller firmware:**

- `droneFreeRTOS.ino` - **✅ STABLE PRODUCTION FIRMWARE v4.0**
  - **NEW:** Complete RF remote control with working motor control system
  - **NEW:** All compilation errors resolved and fully functional
  - FreeRTOS multi-threading architecture with dedicated motor task (Core 1, 50Hz)
  - PID stabilization controllers (Roll, Pitch, Yaw, Altitude)
  - Flight modes: Manual, Stabilize, Altitude Hold
  - ESC integration with immediate power-on calibration
  - Safety systems (ARM/DISARM, Emergency Stop, Control Timeout)
  - Comprehensive sensor suite with ACK payload telemetry
  - **Status:** ✅ Ready for flight testing (Promoted from development Aug 7, 2025)

#### 🎮 `/stable/remote/`

**Production remote controller firmware:**

- `remoteControllerStable.ino` - **✅ STABLE PRODUCTION REMOTE v4.0**
  - **NEW:** Virtual throttle system with safety controls (0-3000 range)
  - **NEW:** Toggle switch arming system (SW1=ARM, SW2=EMERGENCY_STOP)
  - Dual joystick support with ±3000 precision control range
  - 5Hz RF24 control transmission with real-time telemetry feedback
  - Firebase cloud data logging integration
  - Enhanced safety systems and control timeout protection
  - **Status:** ✅ Fully operational (Promoted from development Aug 7, 2025)
- `FastControlRemote.ino` - **✅ STABLE ALTERNATIVE REMOTE**
  - Alternative control interface with simplified operation
  - **Status:** ✅ Updated with latest improvements

### 🔬 **Development** (Active Development)

#### 🚁 `/development/drone/`

**Development drone flight controller firmware:**

- `droneFreeRTOS.ino` - **🔬 DEVELOPMENT VERSION**
  - Working copy of stable firmware for ongoing feature development
  - **Next Phase:** Control mapping refinements and advanced flight modes
  - **Status:** Ready for control optimization and new feature development

#### 🎮 `/development/remote/`

**Development remote controller firmware:**

- `remoteControllerStable.ino` - **🔬 DEVELOPMENT VERSION**
  - Working copy of stable firmware for enhanced control features
  - **Next Phase:** Advanced control modes and real-time PID parameter tuning
  - **Status:** Ready for control refinement and feature enhancement

### 🗃️ **Legacy** (Archived Versions)

#### `/legacy/drone/` & `/legacy/remote/`

**Pre-August 7, 2025 stable firmware versions:**

- `droneFreeRTOS_stable_pre_aug7_2025.ino` - Previous stable drone firmware
- `remoteControllerStable_pre_aug7_2025.ino` - Previous stable remote firmware
- `FastControlRemote_pre_aug7_2025.ino` - Previous alternative remote firmware

**Status:** 🗃️ Archived - Superseded by working motor control implementations
**Note:** These versions had non-functional motor control and compilation issues

- `FastControlRemote.ino` - Alternative development remote

## 🚀 Getting Started

### For Production Use (Stable)

1. **For Drone**: Upload `stable/drone/droneFreeRTOS.ino` to the ESP32 flight controller
2. **For Remote**: Upload `stable/remote/remoteControllerStable.ino` to the ESP32 remote controller
3. **Configure**: Ensure matching RF24 channel settings (Channel 76)
4. **Test**: Verify communication and control response before flight

### For Development (Experimental)

1. **For Drone Development**: Upload `development/drone/droneFreeRTOS.ino` for PID implementation
2. **For Remote Development**: Upload `development/remote/remoteControllerStable.ino` for feature testing
3. **Backup**: Always keep stable versions as backup
4. **Test Thoroughly**: Extensive testing required before flight operations

## 📋 Version Management

- **Stable**: Tested, reliable firmware for production use
- **Development**: Active development for new features (PID stabilization)
- **Backup Strategy**: Always maintain working stable versions
- **Update Process**: Test development → Validate → Promote to stable

## ⚠️ Safety Notes

- **Use stable versions for actual flight operations**
- Always test firmware changes in a safe environment
- Verify all sensor readings before flight operations
- Ensure emergency stop functionality is working
- Keep backup copies of working firmware versions
