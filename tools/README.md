# 🛠️ Tools Directory

This directory contains utility programs, development tools, and helper applications for the ESP32 flight controller system.

## 📁 Contents

### 🌐 Web Interface Tools

- `web_dashboard.cpp` - **Web-based control dashboard** for testing and configuration
  - Real-time telemetry display
  - Manual motor control interface
  - System status monitoring
  - Configuration parameter adjustment

## 🎯 Usage

### Web Dashboard

The web dashboard provides a browser-based interface for:

- **System Testing** - Verify all components without remote controller
- **Parameter Tuning** - Adjust PID parameters and control settings
- **Telemetry Monitoring** - Real-time sensor data visualization
- **Development Aid** - Debug and troubleshoot system issues

### Setup Instructions

1. **Upload firmware** with web dashboard enabled
2. **Connect to WiFi** - System creates access point or connects to existing network
3. **Access interface** - Navigate to ESP32 IP address in web browser
4. **Configure settings** - Adjust parameters as needed for testing

## 🔧 Development Tools

These tools support the development process:

- **Real-time debugging** through web interface
- **Parameter experimentation** without code changes
- **System monitoring** during development
- **Integration testing** of complete system

## ⚠️ Safety Notes

- **Web interface should not replace** proper remote controller for flight operations
- **Use only for ground testing** and development purposes
- **Disable web interface** for production flight operations
- **Secure WiFi connections** to prevent unauthorized access
