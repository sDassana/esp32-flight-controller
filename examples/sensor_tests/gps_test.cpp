#include <Arduino.h>
#include <TinyGPS++.h>

// GPS Pin Configuration
#define GPS_RX_PIN 16 // GPS TX connected to ESP32 RX (GPIO 16)

// GPS Object
TinyGPSPlus gps;

// Variables for GPS status
unsigned long lastGPSUpdate = 0;
unsigned long gpsStartTime = 0;
int satelliteCount = 0;
bool gpsFixed = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("=== ESP32 GPS Module Test ===");
    Serial.println("GPS Module: NEO-6M");
    Serial.println("Connection: GPS TX -> ESP32 GPIO 16");
    Serial.println("==============================");
    
    // Initialize GPS Serial2 (Hardware Serial)
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, -1); // RX on GPIO 16, no TX needed
    
    Serial.println("GPS Serial initialized on GPIO 16");
    Serial.println("Waiting for GPS data...");
    Serial.println("Note: GPS may take 30-60 seconds for first fix outdoors");
    Serial.println("==============================");
    
    gpsStartTime = millis();
}

void loop()
{
    // Read GPS data
    while (Serial2.available() > 0)
    {
        char c = Serial2.read();
        
        // Echo raw GPS data (uncomment to see NMEA sentences)
        // Serial.write(c);
        
        if (gps.encode(c))
        {
            lastGPSUpdate = millis();
            displayGPSInfo();
        }
    }
    
    // Check for GPS timeout
    if (millis() - lastGPSUpdate > 5000 && lastGPSUpdate != 0)
    {
        Serial.println("WARNING: No GPS data received for 5 seconds");
        lastGPSUpdate = millis(); // Reset to avoid spam
    }
    
    // Display status every 5 seconds if no GPS data
    if (millis() - gpsStartTime > 5000 && lastGPSUpdate == 0)
    {
        Serial.println("Still waiting for GPS data... Check connections:");
        Serial.println("- GPS VCC -> 3.3V or 5V");
        Serial.println("- GPS GND -> GND");
        Serial.println("- GPS TX -> ESP32 GPIO 16");
        Serial.println("- Ensure GPS has clear sky view");
        gpsStartTime = millis();
    }
    
    delay(100); // Small delay to prevent overwhelming serial output
}

void displayGPSInfo()
{
    Serial.println("==============================");
    Serial.printf("GPS Update Time: %lu ms\n", millis());
    
    // Location Information
    if (gps.location.isValid())
    {
        if (!gpsFixed)
        {
            gpsFixed = true;
            Serial.println("*** GPS FIX ACQUIRED! ***");
        }
        
        Serial.printf("Latitude: %.6f°\n", gps.location.lat());
        Serial.printf("Longitude: %.6f°\n", gps.location.lng());
        Serial.printf("Altitude: %.2f meters\n", gps.altitude.meters());
        Serial.printf("Speed: %.2f km/h\n", gps.speed.kmph());
        Serial.printf("Course: %.2f°\n", gps.course.deg());
        Serial.printf("Location Age: %lu ms\n", gps.location.age());
    }
    else
    {
        Serial.println("Location: INVALID");
        if (gpsFixed)
        {
            gpsFixed = false;
            Serial.println("*** GPS FIX LOST ***");
        }
    }
    
    // Date and Time Information
    if (gps.date.isValid() && gps.time.isValid())
    {
        Serial.printf("Date: %02d/%02d/%04d\n", 
                     gps.date.day(), gps.date.month(), gps.date.year());
        Serial.printf("Time: %02d:%02d:%02d UTC\n", 
                     gps.time.hour(), gps.time.minute(), gps.time.second());
    }
    else
    {
        Serial.println("Date/Time: INVALID");
    }
    
    // Satellite Information
    if (gps.satellites.isValid())
    {
        satelliteCount = gps.satellites.value();
        Serial.printf("Satellites: %d\n", satelliteCount);
        
        if (satelliteCount < 4)
        {
            Serial.println("Warning: Need at least 4 satellites for GPS fix");
        }
    }
    else
    {
        Serial.println("Satellites: UNKNOWN");
    }
    
    // HDOP (Horizontal Dilution of Precision)
    if (gps.hdop.isValid())
    {
        double hdop = gps.hdop.hdop();
        Serial.printf("HDOP: %.2f ", hdop);
        
        if (hdop < 1.0) Serial.println("(Excellent)");
        else if (hdop < 2.0) Serial.println("(Good)");
        else if (hdop < 5.0) Serial.println("(Moderate)");
        else if (hdop < 10.0) Serial.println("(Fair)");
        else Serial.println("(Poor)");
    }
    
    // Statistics
    Serial.printf("Characters processed: %lu\n", gps.charsProcessed());
    Serial.printf("Sentences with fix: %lu\n", gps.sentencesWithFix());
    Serial.printf("Failed checksum: %lu\n", gps.failedChecksum());
    
    // GPS Status Summary
    Serial.print("Status: ");
    if (gps.location.isValid())
    {
        Serial.print("FIXED");
        if (satelliteCount >= 4) Serial.print(" (Good)");
        else Serial.print(" (Weak)");
    }
    else
    {
        Serial.print("SEARCHING");
    }
    Serial.println();
    
    Serial.println("==============================");
    Serial.println();
    
    // Wait 2 seconds between updates to avoid spam
    delay(2000);
}

// Function to test if GPS is receiving data
void testGPSConnection()
{
    Serial.println("Testing GPS connection...");
    unsigned long startTime = millis();
    int byteCount = 0;
    
    while (millis() - startTime < 3000) // Test for 3 seconds
    {
        if (Serial2.available())
        {
            char c = Serial2.read();
            byteCount++;
            Serial.write(c); // Echo received data
        }
    }
    
    if (byteCount > 0)
    {
        Serial.printf("\nReceived %d bytes from GPS module\n", byteCount);
        Serial.println("GPS connection OK!");
    }
    else
    {
        Serial.println("\nNo data received from GPS module");
        Serial.println("Check wiring and power supply");
    }
}
