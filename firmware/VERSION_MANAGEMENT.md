# 📋 Firmware Version Management

**Last Updated:** August 7, 2025 - Major firmware reorganization and stable promotion

## 🚀 **VERSION 4.0 - MOTOR CONTROL SYSTEM COMPLETE** (August 7, 2025)

### ✅ **STABLE** (Production Ready)

### 🚁 **Stable Drone Firmware v4.0**

- **Location:** `firmware/stable/drone/droneFreeRTOS.ino`
- **Status:** ✅ **PRODUCTION READY - MOTOR CONTROL OPERATIONAL**
- **Version:** 4.0 (Promoted from development August 7, 2025)
- **Features:**
  - **NEW:** Complete motor control system with working ESC integration
  - **NEW:** All compilation errors resolved and fully functional
  - FreeRTOS multi-threading with dedicated motor task (Core 1, 50Hz)
  - PID stabilization controllers (Roll, Pitch, Yaw, Altitude)
  - Flight modes: Manual, Stabilize, Altitude Hold
  - RF24 remote control with 5Hz command reception
  - Immediate ESC calibration (MAX→MIN sequence on power-on)
  - Safety systems: ARM/DISARM, Emergency Stop, Control Timeout
  - ACK payload telemetry with comprehensive sensor data
- **Use For:** ✅ **FLIGHT TESTING AND PRODUCTION USE**

### 🎮 **Stable Remote Controller v4.0**

- **Location:** `firmware/stable/remote/remoteControllerStable.ino`
- **Status:** ✅ **PRODUCTION READY - MOTOR CONTROL INTEGRATION COMPLETE**
- **Version:** 4.0 (Promoted from development August 7, 2025)
- **Features:**
  - **NEW:** Virtual throttle system with 0-3000 internal range
  - **NEW:** Toggle switch arming (SW1=ARM, SW2=EMERGENCY_STOP)
  - **NEW:** Enhanced safety controls and emergency procedures
  - Dual joystick support with ±3000 precision control range
  - 5Hz RF24 control transmission with real-time response
  - ACK payload telemetry reception and display
  - Firebase cloud data logging integration
  - Professional control precision with configurable deadzones
- **Use For:** ✅ **FLIGHT OPERATIONS AND PRODUCTION USE**

---

## 🔬 **DEVELOPMENT** (Active Development)

### 🚁 **Development Drone Firmware v4.0-dev**

- **Location:** `firmware/development/drone/droneFreeRTOS.ino`
- **Status:** 🔬 **DEVELOPMENT VERSION - FEATURE ENHANCEMENT**
- **Purpose:** Control mapping refinements and advanced features
- **Next Features:**
  - Control response curve optimization
  - Advanced flight modes (GPS position hold)
  - Real-time parameter tuning interface
  - Enhanced telemetry and data logging
- **Base:** Working copy of stable v4.0 firmware
- **⚠️ Use For:** Development and testing of new features only

### 🎮 **Development Remote Controller v4.0-dev**

- **Location:** `firmware/development/remote/remoteControllerStable.ino`
- **Status:** 🔬 **DEVELOPMENT VERSION - FEATURE ENHANCEMENT**
- **Purpose:** Advanced control features and UI improvements
- **Next Features:**
  - Real-time PID parameter adjustment interface
  - Advanced control modes and flight assistance
  - Enhanced telemetry display and data visualization
  - Control mapping customization
- **Base:** Working copy of stable v4.0 firmware
- **⚠️ Use For:** Development and testing of new features only

---

## 🗃️ **LEGACY** (Archived)

### Pre-v4.0 Firmware (Before August 7, 2025)

- **Location:** `firmware/legacy/`
- **Status:** 🗃️ **ARCHIVED - SUPERSEDED**
- **Files:**
  - `droneFreeRTOS_stable_pre_aug7_2025.ino` - Previous stable drone firmware
  - `remoteControllerStable_pre_aug7_2025.ino` - Previous stable remote firmware
  - `FastControlRemote_pre_aug7_2025.ino` - Previous alternative remote firmware
- **Issues:** Non-functional motor control, compilation errors
- **Superseded By:** Working v4.0 implementations with motor control
- **Use For:** 📚 Historical reference only - NOT for active use
  - Advanced control modes
  - Enhanced telemetry display
  - Flight mode switching
- **⚠️ Use For:** Development and testing only - NOT for flight

---

## 🔄 **Development Workflow**

1. **Production Use:** Always use `stable/*` versions for actual flights
2. **Development Work:** Use `development/*` versions for new features
3. **Testing:** Thoroughly test development versions before promoting
4. **Promotion:** Move stable features from development to stable versions
5. **Backup:** Always maintain working stable versions

## 📅 **Version History**

- **August 2025:** RF Remote Control System Complete - Promoted to Stable
- **Next Phase:** PID Stabilization Development in Progress
