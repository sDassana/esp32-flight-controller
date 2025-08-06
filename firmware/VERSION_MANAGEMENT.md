# 📋 Firmware Version Management

## ✅ **STABLE VERSIONS** (Production Ready)

### 🚁 **Stable Drone Firmware**

- **Location:** `firmware/stable_drone/droneFreeRTOS.ino`
- **Status:** ✅ **PRODUCTION READY - TESTED AND OPERATIONAL**
- **Features:**
  - Complete RF remote control with ESC integration
  - FreeRTOS multi-threading (50Hz motor updates)
  - Comprehensive sensor integration (BME280, GPS, environmental)
  - Safety systems (arming, emergency stop, control timeout)
  - ACK payload telemetry system
- **Use For:** All flight operations and production use

### 🎮 **Stable Remote Controller**

- **Location:** `firmware/stable_remote/remoteControllerStable.ino`
- **Status:** ✅ **PRODUCTION READY - TESTED AND OPERATIONAL**
- **Features:**
  - Dual joystick support with toggle switches
  - 5Hz control transmission with telemetry display
  - Emergency stop and arming controls
  - Real-time telemetry feedback
- **Use For:** All flight operations and production use

---

## 🔬 **DEVELOPMENT VERSIONS** (Experimental)

### 🚁 **Development Drone Firmware**

- **Location:** `firmware/development_drone/droneFreeRTOS.ino`
- **Status:** 🔬 **DEVELOPMENT VERSION - EXPERIMENTAL**
- **Purpose:** PID stabilization integration development
- **Next Features:**
  - MPU6050 sensor fusion integration
  - Roll, pitch, yaw PID controllers
  - Advanced flight modes
  - Parameter tuning capabilities
- **⚠️ Use For:** Development and testing only - NOT for flight

### 🎮 **Development Remote Controller**

- **Location:** `firmware/development_remote/remoteControllerStable.ino`
- **Status:** 🔬 **DEVELOPMENT VERSION - EXPERIMENTAL**
- **Purpose:** Enhanced control features development
- **Next Features:**
  - Real-time PID parameter tuning
  - Advanced control modes
  - Enhanced telemetry display
  - Flight mode switching
- **⚠️ Use For:** Development and testing only - NOT for flight

---

## 🔄 **Development Workflow**

1. **Production Use:** Always use `stable_*` versions for actual flights
2. **Development Work:** Use `development_*` versions for new features
3. **Testing:** Thoroughly test development versions before promoting
4. **Promotion:** Move stable features from development to stable versions
5. **Backup:** Always maintain working stable versions

## 📅 **Version History**

- **August 2025:** RF Remote Control System Complete - Promoted to Stable
- **Next Phase:** PID Stabilization Development in Progress
