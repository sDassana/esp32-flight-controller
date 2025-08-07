# OLED Display Troubleshooting Guide

## Issue: I2C NACK Errors

If you're seeing I2C NACK (No Acknowledge) errors like:

```
E (49479) i2c.master: s_i2c_synchronous_transaction(945): I2C transaction failed
E (49486) i2c.master: i2c_master_multi_buffer_transmit(1214): I2C transaction failed
E (49494) i2c.master: I2C hardware NACK detected
```

## Common Causes and Solutions

### 1. **Wrong I2C Address**

- Most 128x64 OLED displays use **0x3C**, but some use **0x3D**
- The updated firmware automatically tries both addresses
- Check your display datasheet or try both addresses

### 2. **Wiring Issues**

Check your connections:

```
OLED Display    ESP32
GND       -->   GND
VCC       -->   3.3V (or 5V depending on display)
SDA       -->   GPIO 21 (default SDA)
SCL       -->   GPIO 22 (default SCL)
```

### 3. **Power Issues**

- Ensure 3.3V or 5V power supply (check display requirements)
- Some displays need 5V, others work with 3.3V
- Check for loose connections

### 4. **I2C Bus Conflicts**

- Multiple devices on the same I2C address
- Remove other I2C devices temporarily to test

### 5. **Display Not Connected**

- The firmware now gracefully handles missing displays
- System continues to work without OLED functionality

## Troubleshooting Steps

1. **Run I2C Scanner**

   - The firmware now includes automatic I2C scanning
   - Check Serial Monitor for detected devices

2. **Check Wiring**

   - Verify all connections are secure
   - Use a multimeter to check continuity

3. **Try Different Address**

   - If display appears at 0x3D, the firmware will detect it automatically

4. **Test with Simple Code**

   ```cpp
   #include <Wire.h>
   #include <Adafruit_GFX.h>
   #include <Adafruit_SSD1306.h>

   Adafruit_SSD1306 display(128, 64, &Wire, -1);

   void setup() {
     Serial.begin(115200);
     Wire.begin();

     if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
       Serial.println("SSD1306 allocation failed");
       return;
     }

     display.display();
     Serial.println("Display OK!");
   }
   ```

## Current Firmware Improvements

The updated firmware now includes:

### ✅ **Graceful Error Handling**

- System continues without display if OLED not found
- No more repeated I2C error spam

### ✅ **Automatic I2C Scanning**

- Scans for all I2C devices on startup
- Identifies potential OLED displays

### ✅ **Dual Address Support**

- Tries both 0x3C and 0x3D addresses
- Automatically detects working address

### ✅ **Safe Operation**

- Display updates only happen if display is available
- No system crashes from missing hardware

## Status Messages

- ✅ **"OLED Display found at address 0x3C!"** - Display working normally
- ✅ **"OLED Display found at address 0x3D!"** - Display found at alternate address
- ⚠️ **"OLED Display not found - continuing without display"** - System working without display

## Hardware Requirements

- **0.96" SSD1306 OLED Display (128x64)**
- **I2C Interface**
- **3.3V or 5V power** (check display specs)

The drone controller works perfectly without the OLED display - it's an enhancement feature for real-time status monitoring.
