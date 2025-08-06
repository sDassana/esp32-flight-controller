#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <TinyGPS++.h>
#include "ScioSense_ENS160.h"
#include <Adafruit_AHTX0.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>

// WiFi Credentials
const char *ssid = "YourWiFiName";     // Change this to your WiFi name
const char *password = "YourPassword"; // Change this to your WiFi password

// Web Server
WebServer server(80);

// Component Pins (same as your main project)
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5
#define ESC1_PIN 13 // Front Right (CCW)
#define ESC2_PIN 12 // Back Right (CW)
#define ESC3_PIN 14 // Front Left (CW)
#define ESC4_PIN 27 // Back Left (CCW)
#define LED_R1_PIN 25
#define LED_G1_PIN 33
#define LED_B1_PIN 32
#define BUZZER_PIN 26
#define SDA_PIN 21
#define SCL_PIN 22
#define GUVA_PIN 36
#define GPS_RX_PIN 16
#define BATTERY_PIN 35

// PCA9548A I2C Multiplexer
#define PCA9548A_ADDR 0x70

// Component Objects
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo esc1, esc2, esc3, esc4;
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
ScioSense_ENS160 ens160(ENS160_I2CADDR_1); // Use 0x53 for combo module
Adafruit_AHTX0 aht21;
BH1750 lightMeter;
TinyGPSPlus gps;

// ToF Sensors via PCA9548A
VL53L0X tof_front; // Channel 0
VL53L0X tof_right; // Channel 1
VL53L0X tof_back;  // Channel 2
VL53L0X tof_left;  // Channel 3

// Component Status
struct ComponentStatus
{
    bool wifi = false;
    bool nrf24 = false;
    bool mpu6050 = false;
    bool bme280 = false;
    bool ens160 = false;
    bool aht21 = false;
    bool bh1750 = false;
    bool gps = false;
    bool escs = false;
    bool pca9548a = false;
    bool tof_front = false;
    bool tof_right = false;
    bool tof_back = false;
    bool tof_left = false;
};
ComponentStatus status;

// Sensor Data
struct SensorData
{
    // GPS
    float latitude = 0.0;
    float longitude = 0.0;
    float gps_altitude = 0.0;  // GPS altitude
    float baro_altitude = 0.0; // Barometric altitude
    int satellites = 0;

    // BME280
    float temperature = 0.0;
    float humidity = 0.0;
    float pressure = 0.0;

    // MPU6050
    float ax = 0.0, ay = 0.0, az = 0.0;
    float gx = 0.0, gy = 0.0, gz = 0.0;

    // Light sensors
    float uv = 0.0;
    float lux = 0.0;

    // Air quality (ENS160)
    float tvoc = 0.0;
    float co2 = 0.0;
    float aqi = 0.0;

    // AHT21 data (additional temp/humidity)
    float aht21_temperature = 0.0;
    float aht21_humidity = 0.0;

    // ToF distances (mm)
    int tof_front_distance = 0;
    int tof_right_distance = 0;
    int tof_back_distance = 0;
    int tof_left_distance = 0;

    // System
    float voltage = 0.0;
    float battery_voltage = 0.0;
    float battery_percentage = 0.0;
    int rssi = 0;
};
SensorData sensorData;

// Global reference pressure for altitude calculation
float reference_pressure = 1013.25; // Sea level pressure in hPa

// Motor Control
struct MotorControl
{
    int esc1_pwm = 1000;
    int esc2_pwm = 1000;
    int esc3_pwm = 1000;
    int esc4_pwm = 1000;
    int master_pwm = 1000;
    bool motors_armed = false;
};
MotorControl motorControl;

// Calibration Data Storage
struct CalibrationData
{
    int esc_min = 1000;
    int esc_max = 2000;
    int hover_throttle = 1300;
    float gyro_offset_x = 0.0;
    float gyro_offset_y = 0.0;
    float gyro_offset_z = 0.0;
    float accel_offset_x = 0.0;
    float accel_offset_y = 0.0;
    float accel_offset_z = 0.0;
};
CalibrationData calibration;

const char *dashboard_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Drone Test Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; padding: 20px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .status { display: flex; flex-wrap: wrap; gap: 10px; }
        .status-item { padding: 10px; border-radius: 4px; color: white; min-width: 100px; text-align: center; }
        .status-ok { background: #4CAF50; }
        .status-fail { background: #f44336; }
        .controls { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
        .sensor-value { padding: 10px; background: #e3f2fd; border-radius: 4px; text-align: center; }
        .battery-good { background: #c8e6c9 !important; color: #2e7d32; font-weight: bold; }
        .battery-medium { background: #fff3cd !important; color: #856404; font-weight: bold; }
        .battery-low { background: #f8d7da !important; color: #721c24; font-weight: bold; }
        .battery-critical { background: #f5c6cb !important; color: #721c24; font-weight: bold; animation: blink 1s infinite; }
        @keyframes blink { 0%, 50% { opacity: 1; } 51%, 100% { opacity: 0.5; } }
        button { padding: 10px 20px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; }
        .btn-primary { background: #2196F3; color: white; }
        .btn-danger { background: #f44336; color: white; }
        .btn-success { background: #4CAF50; color: white; }
        .btn-warning { background: #ff9800; color: white; }
        input[type="range"] { width: 100%; }
        .motor-control { display: grid; grid-template-columns: repeat(2, 1fr); gap: 15px; }
        .master-control { grid-column: 1 / -1; background: #fff3cd; padding: 15px; border-radius: 8px; margin-bottom: 20px; }
        .log { background: #000; color: #0f0; padding: 10px; height: 200px; overflow-y: auto; font-family: monospace; }
        #chart { width: 100%; height: 300px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Drone Test Dashboard</h1>
        
        <!-- Component Status -->
        <div class="card">
            <h2>Component Status</h2>
            <div class="status" id="componentStatus">
                <div class="status-item status-fail">WiFi: Connecting...</div>
            </div>
        </div>
        
        <!-- Sensor Readings -->
        <div class="card">
            <h2>Live Sensor Data</h2>
            <div class="sensor-grid" id="sensorData">
                <div class="sensor-value">GPS: Loading...</div>
            </div>
        </div>
        
        <!-- Motor Control -->
        <div class="card">
            <h2>Motor Control & Testing</h2>
            
            <!-- Master Control -->
            <div class="master-control">
                <h3>Master Motor Control (All Motors)</h3>
                <input type="range" id="master" min="1000" max="2000" value="1000" oninput="updateMasterMotor(this.value)">
                <span id="master_val">1000 us</span>
                <p style="margin-top: 10px; font-size: 0.9em; color: #856404;">
                    Use this to control all 4 motors simultaneously. Individual controls below override this setting.
                </p>
            </div>
            
            <div class="motor-control">
                <div>
                    <h3>ESC 1 (Front Left)</h3>
                    <input type="range" id="esc1" min="1000" max="2000" value="1000" oninput="updateMotor(1, this.value)">
                    <span id="esc1_val">1000 us</span>
                    <span id="esc1_status" style="color: green; font-size: 0.8em;"></span>
                </div>
                <div>
                    <h3>ESC 2 (Front Right)</h3>
                    <input type="range" id="esc2" min="1000" max="2000" value="1000" oninput="updateMotor(2, this.value)">
                    <span id="esc2_val">1000 us</span>
                    <span id="esc2_status" style="color: green; font-size: 0.8em;"></span>
                </div>
                <div>
                    <h3>ESC 3 (Rear Left)</h3>
                    <input type="range" id="esc3" min="1000" max="2000" value="1000" oninput="updateMotor(3, this.value)">
                    <span id="esc3_val">1000 us</span>
                    <span id="esc3_status" style="color: green; font-size: 0.8em;"></span>
                </div>
                <div>
                    <h3>ESC 4 (Rear Right)</h3>
                    <input type="range" id="esc4" min="1000" max="2000" value="1000" oninput="updateMotor(4, this.value)">
                    <span id="esc4_val">1000 us</span>
                    <span id="esc4_status" style="color: green; font-size: 0.8em;"></span>
                </div>
            </div>
            <div style="margin-top: 20px; text-align: center;">
                <button class="btn-success" onclick="armMotors()">ARM MOTORS</button>
                <button class="btn-danger" onclick="disarmMotors()">DISARM MOTORS</button>
                <button class="btn-warning" onclick="calibrateESCs()">CALIBRATE ESCs</button>
                <button class="btn-primary" onclick="findHoverThrottle()">FIND HOVER POINT</button>
            </div>
        </div>
        
        <!-- Component Tests -->
        <div class="card">
            <h2>Component Tests</h2>
            <div class="controls">
                <div>
                    <h3>LED Tests</h3>
                    <button class="btn-primary" onclick="testComponent('led', 'red')">Red LED</button>
                    <button class="btn-primary" onclick="testComponent('led', 'green')">Green LED</button>
                    <button class="btn-primary" onclick="testComponent('led', 'blue')">Blue LED</button>
                    <button class="btn-primary" onclick="testComponent('led', 'off')">LEDs Off</button>
                </div>
                <div>
                    <h3>Buzzer Test</h3>
                    <button class="btn-warning" onclick="testComponent('buzzer', 'short')">Short Beep</button>
                    <button class="btn-warning" onclick="testComponent('buzzer', 'long')">Long Beep</button>
                </div>
                <div>
                    <h3>Sensor Calibration</h3>
                    <button class="btn-success" onclick="calibrateComponent('mpu6050')">Calibrate IMU</button>
                    <button class="btn-success" onclick="calibrateComponent('bme280')">Reset Altitude</button>
                    <button class="btn-success" onclick="calibrateComponent('ens160')">Calibrate ENS160</button>
                </div>
                <div>
                    <h3>System Tests</h3>
                    <button class="btn-primary" onclick="testComponent('battery', 'test')">Test Battery</button>
                </div>
                <div>
                    <h3>NRF24 Test</h3>
                    <button class="btn-primary" onclick="testComponent('nrf24', 'test')">Test Range</button>
                    <button class="btn-primary" onclick="testComponent('nrf24', 'scan')">Scan Channels</button>
                </div>
            </div>
        </div>
        
        <!-- Calibration Values -->
        <div class="card">
            <h2>Calibration & Configuration</h2>
            <div class="controls">
                <div>
                    <h3>ESC Calibration</h3>
                    <label>Min PWM: <input type="number" id="esc_min" value="1000" min="800" max="1200"></label><br>
                    <label>Max PWM: <input type="number" id="esc_max" value="2000" min="1800" max="2200"></label><br>
                    <label>Hover Throttle: <input type="number" id="hover_throttle" value="1300" min="1000" max="1600"></label><br>
                    <button class="btn-success" onclick="saveCalibration()">Save Calibration</button>
                </div>
                <div>
                    <h3>IMU Offsets</h3>
                    <label>Gyro X: <input type="number" id="gyro_x" value="0" step="0.01"></label><br>
                    <label>Gyro Y: <input type="number" id="gyro_y" value="0" step="0.01"></label><br>
                    <label>Gyro Z: <input type="number" id="gyro_z" value="0" step="0.01"></label><br>
                    <label>Accel X: <input type="number" id="accel_x" value="0" step="0.01"></label><br>
                    <label>Accel Y: <input type="number" id="accel_y" value="0" step="0.01"></label><br>
                    <label>Accel Z: <input type="number" id="accel_z" value="0" step="0.01"></label><br>
                </div>
            </div>
        </div>
        
        <!-- Data Logging -->
        <div class="card">
            <h2>System Log</h2>
            <div class="log" id="systemLog">System initialized...\n</div>
            <button class="btn-primary" onclick="clearLog()">Clear Log</button>
            <button class="btn-success" onclick="downloadLog()">Download Log</button>
        </div>
    </div>

    <script>
        // Update data every 500ms
        setInterval(updateData, 500);
        
        // Throttle motor updates to prevent flooding
        let motorUpdateTimeout = null;
        let masterUpdateTimeout = null;
        
        function updateData() {
            fetch('/api/data')
                .then(response => response.json())
                .then(data => {
                    updateComponentStatus(data.status);
                    updateSensorData(data.sensors);
                })
                .catch(error => log('Error fetching data: ' + error));
        }
        
        function updateComponentStatus(status) {
            const container = document.getElementById('componentStatus');
            container.innerHTML = '';
            
            Object.keys(status).forEach(component => {
                const div = document.createElement('div');
                div.className = `status-item ${status[component] ? 'status-ok' : 'status-fail'}`;
                div.textContent = `${component.toUpperCase()}: ${status[component] ? 'OK' : 'FAIL'}`;
                container.appendChild(div);
            });
        }
        
        function updateSensorData(sensors) {
            const container = document.getElementById('sensorData');
            
            // Determine battery status class
            let batteryClass = 'sensor-value';
            if (sensors.battery_percentage >= 60) {
                batteryClass += ' battery-good';
            } else if (sensors.battery_percentage >= 30) {
                batteryClass += ' battery-medium';
            } else if (sensors.battery_percentage >= 15) {
                batteryClass += ' battery-low';
            } else {
                batteryClass += ' battery-critical';
            }
            
            container.innerHTML = `
                <div class="sensor-value">GPS: ${sensors.latitude.toFixed(6)}°, ${sensors.longitude.toFixed(6)}°</div>
                <div class="sensor-value">GPS Altitude: ${sensors.gps_altitude.toFixed(2)} m</div>
                <div class="sensor-value">Baro Altitude: ${sensors.baro_altitude.toFixed(2)} m</div>
                <div class="sensor-value">Combined Altitude: ${sensors.altitude.toFixed(2)} m</div>
                <div class="sensor-value">Satellites: ${sensors.satellites}</div>
                <div class="sensor-value">BME280 Temp: ${sensors.temperature.toFixed(2)}°C</div>
                <div class="sensor-value">BME280 Humidity: ${sensors.humidity.toFixed(1)}%</div>
                <div class="sensor-value">Pressure: ${sensors.pressure.toFixed(2)} hPa</div>
                <div class="sensor-value">AHT21 Temp: ${sensors.aht21_temperature.toFixed(2)}°C</div>
                <div class="sensor-value">AHT21 Humidity: ${sensors.aht21_humidity.toFixed(1)}%</div>
                <div class="sensor-value">Accel: X:${sensors.ax.toFixed(2)} Y:${sensors.ay.toFixed(2)} Z:${sensors.az.toFixed(2)}</div>
                <div class="sensor-value">Gyro: X:${sensors.gx.toFixed(2)} Y:${sensors.gy.toFixed(2)} Z:${sensors.gz.toFixed(2)}</div>
                <div class="sensor-value">UV Index: ${sensors.uv.toFixed(2)}</div>
                <div class="sensor-value">Light: ${sensors.lux.toFixed(0)} lux</div>
                <div class="sensor-value">ENS160 TVOC: ${sensors.tvoc.toFixed(0)} ppb</div>
                <div class="sensor-value">ENS160 CO2: ${sensors.co2.toFixed(0)} ppm</div>
                <div class="sensor-value">Air Quality Index: ${sensors.aqi.toFixed(1)}</div>
                <div class="sensor-value">ToF Front: ${sensors.tof_front_distance >= 0 ? sensors.tof_front_distance + ' mm' : 'Error'}</div>
                <div class="sensor-value">ToF Right: ${sensors.tof_right_distance >= 0 ? sensors.tof_right_distance + ' mm' : 'Error'}</div>
                <div class="sensor-value">ToF Back: ${sensors.tof_back_distance >= 0 ? sensors.tof_back_distance + ' mm' : 'Error'}</div>
                <div class="sensor-value">ToF Left: ${sensors.tof_left_distance >= 0 ? sensors.tof_left_distance + ' mm' : 'Error'}</div>
                <div class="sensor-value">System Voltage: ${sensors.voltage.toFixed(2)} V</div>
                <div class="${batteryClass}">🔋 Battery: ${sensors.battery_voltage.toFixed(2)} V (${sensors.battery_percentage.toFixed(1)}%)</div>
                <div class="sensor-value">WiFi RSSI: ${sensors.rssi} dBm</div>
            `;
        }
        
        function updateMasterMotor(value) {
            document.getElementById('master_val').textContent = value + ' us';
            
            // Update individual sliders to match immediately
            for(let i = 1; i <= 4; i++) {
                document.getElementById(`esc${i}`).value = value;
                document.getElementById(`esc${i}_val`).textContent = value + ' us';
            }
            
            // Throttle the actual motor update requests
            if (masterUpdateTimeout) clearTimeout(masterUpdateTimeout);
            masterUpdateTimeout = setTimeout(() => {
                const formData = new FormData();
                formData.append('pwm', value);
                
                fetch('/api/motor/master', { 
                    method: 'POST',
                    body: formData
                })
                .then(response => {
                    if (!response.ok) {
                        throw new Error(`HTTP ${response.status}`);
                    }
                    console.log('Master motor updated to:', value);
                })
                .catch(error => {
                    console.error('Master motor update failed:', error);
                    log('Error updating master motor: ' + error);
                });
            }, 100); // 100ms throttle
        }
        
        function updateMotor(motor, value) {
            document.getElementById(`esc${motor}_val`).textContent = value + ' us';
            document.getElementById(`esc${motor}_status`).textContent = 'Sending...';
            
            // Throttle the motor update requests
            if (motorUpdateTimeout) clearTimeout(motorUpdateTimeout);
            motorUpdateTimeout = setTimeout(() => {
                const formData = new FormData();
                formData.append('pwm', value);
                
                fetch(`/api/motor/${motor}`, { 
                    method: 'POST',
                    body: formData
                })
                .then(response => {
                    if (!response.ok) {
                        throw new Error(`HTTP ${response.status}`);
                    }
                    console.log(`Motor ${motor} updated to:`, value);
                    document.getElementById(`esc${motor}_status`).textContent = '✓ Sent';
                    setTimeout(() => {
                        document.getElementById(`esc${motor}_status`).textContent = '';
                    }, 1000);
                })
                .catch(error => {
                    console.error(`Motor ${motor} update failed:`, error);
                    document.getElementById(`esc${motor}_status`).textContent = '✗ Failed';
                    document.getElementById(`esc${motor}_status`).style.color = 'red';
                    log(`Error updating motor ${motor}: ` + error);
                    setTimeout(() => {
                        document.getElementById(`esc${motor}_status`).textContent = '';
                        document.getElementById(`esc${motor}_status`).style.color = 'green';
                    }, 2000);
                });
            }, 100); // 100ms throttle
        }
        
        function armMotors() {
            fetch('/api/motors/arm', { method: 'POST' })
                .then(() => log('Motors ARMED'))
                .catch(error => log('Error arming motors: ' + error));
        }
        
        function disarmMotors() {
            fetch('/api/motors/disarm', { method: 'POST' })
                .then(() => log('Motors DISARMED'))
                .catch(error => log('Error disarming motors: ' + error));
        }
        
        function calibrateESCs() {
            if(confirm('This will run ESC calibration sequence. Make sure propellers are OFF!')) {
                fetch('/api/calibrate/esc', { method: 'POST' })
                    .then(() => log('ESC calibration started'))
                    .catch(error => log('Error starting calibration: ' + error));
            }
        }
        
        function findHoverThrottle() {
            if(confirm('This will gradually increase throttle to find hover point. Ensure drone is secured!')) {
                fetch('/api/calibrate/hover', { method: 'POST' })
                    .then(() => log('Hover throttle test started'))
                    .catch(error => log('Error starting hover test: ' + error));
            }
        }
        
        function testComponent(component, action) {
            fetch(`/api/test/${component}/${action}`, { method: 'POST' })
                .then(() => log(`Testing ${component}: ${action}`))
                .catch(error => log(`Error testing ${component}: ` + error));
        }
        
        function calibrateComponent(component) {
            fetch(`/api/calibrate/${component}`, { method: 'POST' })
                .then(() => log(`Calibrating ${component}...`))
                .catch(error => log(`Error calibrating ${component}: ` + error));
        }
        
        function saveCalibration() {
            const data = {
                esc_min: document.getElementById('esc_min').value,
                esc_max: document.getElementById('esc_max').value,
                hover_throttle: document.getElementById('hover_throttle').value,
                gyro_x: document.getElementById('gyro_x').value,
                gyro_y: document.getElementById('gyro_y').value,
                gyro_z: document.getElementById('gyro_z').value,
                accel_x: document.getElementById('accel_x').value,
                accel_y: document.getElementById('accel_y').value,
                accel_z: document.getElementById('accel_z').value
            };
            
            fetch('/api/calibration/save', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(data)
            })
            .then(() => log('Calibration data saved'))
            .catch(error => log('Error saving calibration: ' + error));
        }
        
        function log(message) {
            const logDiv = document.getElementById('systemLog');
            const timestamp = new Date().toLocaleTimeString();
            logDiv.innerHTML += `[${timestamp}] ${message}\n`;
            logDiv.scrollTop = logDiv.scrollHeight;
        }
        
        function clearLog() {
            document.getElementById('systemLog').innerHTML = '';
        }
        
        function downloadLog() {
            const logContent = document.getElementById('systemLog').innerHTML;
            const blob = new Blob([logContent], { type: 'text/plain' });
            const url = window.URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'drone_test_log.txt';
            a.click();
            window.URL.revokeObjectURL(url);
        }
    </script>
</body>
</html>
)rawliteral";

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32 Drone Web Dashboard Starting...");

    // Initialize all pins
    initializePins();

    // Initialize components
    initializeComponents();

    // Connect to WiFi
    connectWiFi();

    // Setup web server routes
    setupWebServer();

    Serial.println("Web Dashboard ready!");
    Serial.print("Access dashboard at: http://");
    Serial.println(WiFi.localIP());
}

void loop()
{
    static unsigned long lastPrint = 0;

    // Handle web server frequently
    server.handleClient();

    // Update sensors less frequently
    updateSensorReadings();

    // Print debug info every 5 seconds
    if (millis() - lastPrint > 5000)
    {
        lastPrint = millis();
        Serial.printf("System running - Free heap: %d bytes\n", ESP.getFreeHeap());
        Serial.printf("Motors armed: %s\n", motorControl.motors_armed ? "YES" : "NO");
        Serial.printf("WiFi status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
        Serial.printf("Battery: %.2fV (%.1f%%)\n", sensorData.battery_voltage, sensorData.battery_percentage);

        // Battery warnings
        if (sensorData.battery_percentage < 15.0)
        {
            Serial.println("⚠️  CRITICAL BATTERY WARNING - LAND IMMEDIATELY!");
        }
        else if (sensorData.battery_percentage < 30.0)
        {
            Serial.println("⚠️  Low battery warning - Consider landing soon");
        }

        if (motorControl.motors_armed)
        {
            Serial.printf("Motor PWM values - ESC1:%d ESC2:%d ESC3:%d ESC4:%d\n",
                          motorControl.esc1_pwm, motorControl.esc2_pwm,
                          motorControl.esc3_pwm, motorControl.esc4_pwm);
        }
    }

    // Small delay to prevent watchdog issues
    delay(1);
}

void scanI2CDevices()
{
    Serial.println("Scanning I2C devices...");
    int deviceCount = 0;

    for (byte address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);
        byte error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.printf("I2C device found at address 0x%02X\n", address);
            deviceCount++;

            // Identify known devices
            switch (address)
            {
            case 0x23:
                Serial.println("  -> BH1750 Light Sensor");
                break;
            case 0x38:
                Serial.println("  -> AHT21 Temp/Humidity Sensor");
                break;
            case 0x52:
                Serial.println("  -> ENS160 Air Quality Sensor (primary)");
                break;
            case 0x53:
                Serial.println("  -> ENS160 Air Quality Sensor (secondary)");
                break;
            case 0x68:
                Serial.println("  -> MPU6050 IMU");
                break;
            case 0x70:
                Serial.println("  -> PCA9548A I2C Multiplexer");
                break;
            case 0x76:
                Serial.println("  -> BME280 Environmental Sensor (primary)");
                break;
            case 0x77:
                Serial.println("  -> BME280 Environmental Sensor (secondary)");
                break;
            default:
                Serial.printf("  -> Unknown device at 0x%02X\n", address);
                break;
            }
        }
        else if (error == 4)
        {
            Serial.printf("Unknown error at address 0x%02X\n", address);
        }
    }

    if (deviceCount == 0)
    {
        Serial.println("No I2C devices found!");
    }
    else
    {
        Serial.printf("Found %d I2C device(s)\n", deviceCount);
    }
    Serial.println("I2C scan complete\n");
}

// PCA9548A I2C Multiplexer Functions
void selectMuxChannel(uint8_t channel)
{
    if (channel > 7)
        return;

    Wire.beginTransmission(PCA9548A_ADDR);
    Wire.write(1 << channel); // Select channel
    Wire.endTransmission();
}

void disableMuxChannels()
{
    Wire.beginTransmission(PCA9548A_ADDR);
    Wire.write(0); // Disable all channels
    Wire.endTransmission();
}

void initializeToFSensors()
{
    Serial.println("Initializing ToF sensors...");

    // Initialize Front ToF (Channel 0)
    selectMuxChannel(0);
    delay(10);
    status.tof_front = tof_front.init();
    if (status.tof_front)
    {
        tof_front.setTimeout(500);
        tof_front.startContinuous();
        Serial.println("  -> ToF Front (Ch0) initialized");
    }
    else
    {
        Serial.println("  -> ToF Front (Ch0) FAILED!");
    }

    // Initialize Right ToF (Channel 1)
    selectMuxChannel(1);
    delay(10);
    status.tof_right = tof_right.init();
    if (status.tof_right)
    {
        tof_right.setTimeout(500);
        tof_right.startContinuous();
        Serial.println("  -> ToF Right (Ch1) initialized");
    }
    else
    {
        Serial.println("  -> ToF Right (Ch1) FAILED!");
    }

    // Initialize Back ToF (Channel 2)
    selectMuxChannel(2);
    delay(10);
    status.tof_back = tof_back.init();
    if (status.tof_back)
    {
        tof_back.setTimeout(500);
        tof_back.startContinuous();
        Serial.println("  -> ToF Back (Ch2) initialized");
    }
    else
    {
        Serial.println("  -> ToF Back (Ch2) FAILED!");
    }

    // Initialize Left ToF (Channel 3)
    selectMuxChannel(3);
    delay(10);
    status.tof_left = tof_left.init();
    if (status.tof_left)
    {
        tof_left.setTimeout(500);
        tof_left.startContinuous();
        Serial.println("  -> ToF Left (Ch3) initialized");
    }
    else
    {
        Serial.println("  -> ToF Left (Ch3) FAILED!");
    }

    // Disable all channels
    disableMuxChannels();
    Serial.println("ToF sensor initialization complete");
}

void initializePins()
{
    // LED pins
    pinMode(LED_R1_PIN, OUTPUT);
    pinMode(LED_G1_PIN, OUTPUT);
    pinMode(LED_B1_PIN, OUTPUT);

    // Buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // UV sensor
    pinMode(GUVA_PIN, INPUT);

    // Battery voltage monitoring
    pinMode(BATTERY_PIN, INPUT);
    analogSetAttenuation(ADC_11db); // For full 3.3V range

    Serial.println("Pins initialized");
}

void initializeComponents()
{
    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("I2C initialized");

    // Scan I2C devices first
    scanI2CDevices();

    // SPI for NRF24L01
    SPI.begin(18, 19, 23, 5);

    // Initialize NRF24L01
    status.nrf24 = radio.begin();
    if (status.nrf24)
    {
        radio.setChannel(76);
        radio.setPALevel(RF24_PA_LOW);
        Serial.println("NRF24L01 initialized successfully");
    }
    else
    {
        Serial.println("NRF24L01 initialization FAILED!");
    }

    // Initialize MPU6050
    Serial.print("Initializing MPU6050... ");
    status.mpu6050 = mpu.begin(0x68);
    if (status.mpu6050)
    {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("MPU6050 initialized");
    }
    else
    {
        Serial.println("MPU6050 initialization FAILED!");
    }

    // Initialize BME280
    Serial.print("Initializing BME280... ");
    status.bme280 = bme.begin(0x76) || bme.begin(0x77);
    if (status.bme280)
    {
        Serial.println("BME280 initialized");
    }
    else
    {
        Serial.println("BME280 initialization FAILED!");
    }

    // Initialize AHT21 first (needed for ENS160 environmental data)
    Serial.print("Initializing AHT21... ");
    status.aht21 = aht21.begin();
    if (status.aht21)
    {
        Serial.println("AHT21 initialized");
        delay(100); // Let AHT21 stabilize
    }
    else
    {
        Serial.println("AHT21 initialization FAILED!");
    }

    // Initialize ENS160 (after AHT21 for environmental data)
    Serial.print("Initializing ENS160... ");
    delay(200); // Additional delay before ENS160 init
    status.ens160 = ens160.begin();
    if (status.ens160)
    {
        delay(100);
        ens160.setMode(ENS160_OPMODE_STD);
        Serial.println("ENS160 initialized");

        // Set initial environmental data if AHT21 is available
        if (status.aht21)
        {
            sensors_event_t humidity, temp;
            if (aht21.getEvent(&humidity, &temp))
            {
                ens160.set_envdata(temp.temperature, humidity.relative_humidity);
                Serial.println("ENS160 environmental data set");
            }
        }
    }
    else
    {
        Serial.println("ENS160 initialization FAILED!");
    }

    // Initialize BH1750
    Serial.print("Initializing BH1750... ");
    status.bh1750 = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23);
    if (status.bh1750)
    {
        Serial.println("BH1750 initialized");
    }
    else
    {
        Serial.println("BH1750 initialization FAILED!");
    }

    // Initialize PCA9548A I2C Multiplexer
    Serial.print("Initializing PCA9548A... ");
    Wire.beginTransmission(PCA9548A_ADDR);
    status.pca9548a = (Wire.endTransmission() == 0);
    if (status.pca9548a)
    {
        Serial.println("PCA9548A initialized");

        // Initialize ToF sensors
        initializeToFSensors();
    }
    else
    {
        Serial.println("PCA9548A initialization FAILED!");
    }

    // Initialize GPS
    Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, -1);
    status.gps = true; // We'll update this based on data reception
    Serial.println("GPS initialized");

    // Initialize ESCs
    esc1.attach(ESC1_PIN, 1000, 2000);
    esc2.attach(ESC2_PIN, 1000, 2000);
    esc3.attach(ESC3_PIN, 1000, 2000);
    esc4.attach(ESC4_PIN, 1000, 2000);

    // Set to minimum
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);

    status.escs = true;
    Serial.println("ESCs initialized");
}

void connectWiFi()
{
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        status.wifi = true;
        Serial.println();
        Serial.print("WiFi connected! IP: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println();
        Serial.println("WiFi connection failed!");
    }
}

void setupWebServer()
{
    // Serve dashboard
    server.on("/", HTTP_GET, []()
              { server.send(200, "text/html", dashboard_html); });

    // API endpoint for sensor data
    server.on("/api/data", HTTP_GET, []()
              {
        StaticJsonDocument<2048> doc; // Increased size for all data
        
        // Component status
        JsonObject statusObj = doc.createNestedObject("status");
        statusObj["wifi"] = status.wifi;
        statusObj["nrf24"] = status.nrf24;
        statusObj["mpu6050"] = status.mpu6050;
        statusObj["bme280"] = status.bme280;
        statusObj["ens160"] = status.ens160;
        statusObj["aht21"] = status.aht21;
        statusObj["bh1750"] = status.bh1750;
        statusObj["gps"] = status.gps;
        statusObj["escs"] = status.escs;
        statusObj["pca9548a"] = status.pca9548a;
        statusObj["tof_front"] = status.tof_front;
        statusObj["tof_right"] = status.tof_right;
        statusObj["tof_back"] = status.tof_back;
        statusObj["tof_left"] = status.tof_left;
        
        // Sensor data
        JsonObject sensorsObj = doc.createNestedObject("sensors");
        sensorsObj["latitude"] = sensorData.latitude;
        sensorsObj["longitude"] = sensorData.longitude;
        sensorsObj["gps_altitude"] = sensorData.gps_altitude;
        sensorsObj["baro_altitude"] = sensorData.baro_altitude;
        sensorsObj["altitude"] = sensorData.gps_altitude > 0 ? sensorData.gps_altitude : sensorData.baro_altitude; // Prefer GPS if available
        sensorsObj["satellites"] = sensorData.satellites;
        sensorsObj["temperature"] = sensorData.temperature;
        sensorsObj["humidity"] = sensorData.humidity;
        sensorsObj["pressure"] = sensorData.pressure;
        sensorsObj["aht21_temperature"] = sensorData.aht21_temperature;
        sensorsObj["aht21_humidity"] = sensorData.aht21_humidity;
        sensorsObj["ax"] = sensorData.ax;
        sensorsObj["ay"] = sensorData.ay;
        sensorsObj["az"] = sensorData.az;
        sensorsObj["gx"] = sensorData.gx;
        sensorsObj["gy"] = sensorData.gy;
        sensorsObj["gz"] = sensorData.gz;
        sensorsObj["uv"] = sensorData.uv;
        sensorsObj["lux"] = sensorData.lux;
        sensorsObj["tvoc"] = sensorData.tvoc;
        sensorsObj["co2"] = sensorData.co2;
        sensorsObj["aqi"] = sensorData.aqi;
        sensorsObj["tof_front_distance"] = sensorData.tof_front_distance;
        sensorsObj["tof_right_distance"] = sensorData.tof_right_distance;
        sensorsObj["tof_back_distance"] = sensorData.tof_back_distance;
        sensorsObj["tof_left_distance"] = sensorData.tof_left_distance;
        sensorsObj["voltage"] = sensorData.voltage;
        sensorsObj["battery_voltage"] = sensorData.battery_voltage;
        sensorsObj["battery_percentage"] = sensorData.battery_percentage;
        sensorsObj["rssi"] = sensorData.rssi;
        
        String response;
        if (serializeJson(doc, response) == 0) {
            Serial.println("ERROR: Failed to serialize JSON");
            server.send(500, "text/plain", "JSON serialization failed");
            return;
        }
        
        server.send(200, "application/json", response); });

    // Motor control endpoints with explicit pattern matching
    server.on("/api/motor/master", HTTP_POST, []()
              { 
        Serial.println("=== MASTER MOTOR CONTROL REQUEST ===");
        if (!server.hasArg("pwm")) {
            server.send(400, "text/plain", "Missing PWM parameter");
            return;
        }
        
        String pwmStr = server.arg("pwm");
        Serial.printf("Received PWM string: %s\n", pwmStr.c_str());
        
        int pwm = pwmStr.toInt();
        Serial.printf("Parsed PWM value: %d\n", pwm);
        
        pwm = constrain(pwm, 1000, 2000); // Safety constraint
        Serial.printf("Constrained PWM: %d us\n", pwm);
        Serial.printf("Motors armed status: %s\n", motorControl.motors_armed ? "TRUE" : "FALSE");
        
        if (motorControl.motors_armed) {
            Serial.println("Writing to ESCs...");
            esc1.writeMicroseconds(pwm);
            delay(1);
            esc2.writeMicroseconds(pwm);
            delay(1);
            esc3.writeMicroseconds(pwm);
            delay(1);
            esc4.writeMicroseconds(pwm);
            delay(1);
            
            motorControl.esc1_pwm = pwm;
            motorControl.esc2_pwm = pwm;
            motorControl.esc3_pwm = pwm;
            motorControl.esc4_pwm = pwm;
            motorControl.master_pwm = pwm;
            Serial.printf("All motors updated to %d us\n", pwm);
        } else {
            Serial.println("Motors not armed - ignoring command");
        }
        Serial.println("=== END MASTER MOTOR CONTROL ===");
        server.send(200, "text/plain", "OK"); });

    server.on("/api/motor/1", HTTP_POST, []()
              { 
        Serial.println("=== MOTOR 1 CONTROL REQUEST ===");
        if (!server.hasArg("pwm")) {
            server.send(400, "text/plain", "Missing PWM parameter");
            return;
        }
        
        String pwmStr = server.arg("pwm");
        Serial.printf("Received PWM string: %s\n", pwmStr.c_str());
        
        int pwm = pwmStr.toInt();
        Serial.printf("Parsed PWM value: %d\n", pwm);
        
        pwm = constrain(pwm, 1000, 2000);
        Serial.printf("Constrained PWM: %d us\n", pwm);
        Serial.printf("Motors armed status: %s\n", motorControl.motors_armed ? "TRUE" : "FALSE");
        
        if (motorControl.motors_armed) {
            Serial.println("Writing to ESC1...");
            esc1.writeMicroseconds(pwm);
            motorControl.esc1_pwm = pwm;
            Serial.printf("Motor 1 updated to %d us\n", pwm);
        } else {
            Serial.println("Motors not armed - ignoring command");
        }
        Serial.println("=== END MOTOR 1 CONTROL ===");
        server.send(200, "text/plain", "OK"); });

    server.on("/api/motor/2", HTTP_POST, []()
              { 
        Serial.println("=== MOTOR 2 CONTROL REQUEST ===");
        if (!server.hasArg("pwm")) {
            server.send(400, "text/plain", "Missing PWM parameter");
            return;
        }
        
        String pwmStr = server.arg("pwm");
        Serial.printf("Received PWM string: %s\n", pwmStr.c_str());
        
        int pwm = pwmStr.toInt();
        Serial.printf("Parsed PWM value: %d\n", pwm);
        
        pwm = constrain(pwm, 1000, 2000);
        Serial.printf("Constrained PWM: %d us\n", pwm);
        Serial.printf("Motors armed status: %s\n", motorControl.motors_armed ? "TRUE" : "FALSE");
        
        if (motorControl.motors_armed) {
            Serial.println("Writing to ESC2...");
            esc2.writeMicroseconds(pwm);
            motorControl.esc2_pwm = pwm;
            Serial.printf("Motor 2 updated to %d us\n", pwm);
        } else {
            Serial.println("Motors not armed - ignoring command");
        }
        Serial.println("=== END MOTOR 2 CONTROL ===");
        server.send(200, "text/plain", "OK"); });

    server.on("/api/motor/3", HTTP_POST, []()
              { 
        Serial.println("=== MOTOR 3 CONTROL REQUEST ===");
        if (!server.hasArg("pwm")) {
            server.send(400, "text/plain", "Missing PWM parameter");
            return;
        }
        
        String pwmStr = server.arg("pwm");
        Serial.printf("Received PWM string: %s\n", pwmStr.c_str());
        
        int pwm = pwmStr.toInt();
        Serial.printf("Parsed PWM value: %d\n", pwm);
        
        pwm = constrain(pwm, 1000, 2000);
        Serial.printf("Constrained PWM: %d us\n", pwm);
        Serial.printf("Motors armed status: %s\n", motorControl.motors_armed ? "TRUE" : "FALSE");
        
        if (motorControl.motors_armed) {
            Serial.println("Writing to ESC3...");
            esc3.writeMicroseconds(pwm);
            motorControl.esc3_pwm = pwm;
            Serial.printf("Motor 3 updated to %d us\n", pwm);
        } else {
            Serial.println("Motors not armed - ignoring command");
        }
        Serial.println("=== END MOTOR 3 CONTROL ===");
        server.send(200, "text/plain", "OK"); });

    server.on("/api/motor/4", HTTP_POST, []()
              { 
        Serial.println("=== MOTOR 4 CONTROL REQUEST ===");
        if (!server.hasArg("pwm")) {
            server.send(400, "text/plain", "Missing PWM parameter");
            return;
        }
        
        String pwmStr = server.arg("pwm");
        Serial.printf("Received PWM string: %s\n", pwmStr.c_str());
        
        int pwm = pwmStr.toInt();
        Serial.printf("Parsed PWM value: %d\n", pwm);
        
        pwm = constrain(pwm, 1000, 2000);
        Serial.printf("Constrained PWM: %d us\n", pwm);
        Serial.printf("Motors armed status: %s\n", motorControl.motors_armed ? "TRUE" : "FALSE");
        
        if (motorControl.motors_armed) {
            Serial.println("Writing to ESC4...");
            esc4.writeMicroseconds(pwm);
            motorControl.esc4_pwm = pwm;
            Serial.printf("Motor 4 updated to %d us\n", pwm);
        } else {
            Serial.println("Motors not armed - ignoring command");
        }
        Serial.println("=== END MOTOR 4 CONTROL ===");
        server.send(200, "text/plain", "OK"); });

    // Motor arm/disarm
    server.on("/api/motors/arm", HTTP_POST, []()
              {
        Serial.println("=== MOTOR ARM REQUEST ===");
        Serial.println("Setting motors to armed state...");
        
        // First ensure all motors are at minimum
        esc1.writeMicroseconds(1000);
        delay(10);
        esc2.writeMicroseconds(1000);
        delay(10);
        esc3.writeMicroseconds(1000);
        delay(10);
        esc4.writeMicroseconds(1000);
        delay(10);
        
        motorControl.motors_armed = true;
        motorControl.esc1_pwm = 1000;
        motorControl.esc2_pwm = 1000;
        motorControl.esc3_pwm = 1000;
        motorControl.esc4_pwm = 1000;
        motorControl.master_pwm = 1000;
        
        Serial.printf("Motors ARMED - Status: %s\n", motorControl.motors_armed ? "TRUE" : "FALSE");
        Serial.println("=== END MOTOR ARM ===");
        server.send(200, "text/plain", "Motors Armed"); });

    server.on("/api/motors/disarm", HTTP_POST, []()
              {
        Serial.println("=== MOTOR DISARM REQUEST ===");
        
        motorControl.motors_armed = false;
        
        esc1.writeMicroseconds(1000);
        delay(10);
        esc2.writeMicroseconds(1000);
        delay(10);
        esc3.writeMicroseconds(1000);
        delay(10);
        esc4.writeMicroseconds(1000);
        delay(10);
        
        motorControl.esc1_pwm = 1000;
        motorControl.esc2_pwm = 1000;
        motorControl.esc3_pwm = 1000;
        motorControl.esc4_pwm = 1000;
        motorControl.master_pwm = 1000;
        
        Serial.printf("Motors DISARMED - Status: %s\n", motorControl.motors_armed ? "TRUE" : "FALSE");
        Serial.println("=== END MOTOR DISARM ===");
        server.send(200, "text/plain", "Motors Disarmed"); });

    // Component test endpoints
    server.on("/api/test/led/red", HTTP_POST, []()
              {
        digitalWrite(LED_R1_PIN, HIGH);
        digitalWrite(LED_G1_PIN, LOW);
        digitalWrite(LED_B1_PIN, LOW);
        server.send(200, "text/plain", "Red LED On"); });

    server.on("/api/test/led/green", HTTP_POST, []()
              {
        digitalWrite(LED_R1_PIN, LOW);
        digitalWrite(LED_G1_PIN, HIGH);
        digitalWrite(LED_B1_PIN, LOW);
        server.send(200, "text/plain", "Green LED On"); });

    server.on("/api/test/led/blue", HTTP_POST, []()
              {
        digitalWrite(LED_R1_PIN, LOW);
        digitalWrite(LED_G1_PIN, LOW);
        digitalWrite(LED_B1_PIN, HIGH);
        server.send(200, "text/plain", "Blue LED On"); });

    server.on("/api/test/led/off", HTTP_POST, []()
              {
        digitalWrite(LED_R1_PIN, LOW);
        digitalWrite(LED_G1_PIN, LOW);
        digitalWrite(LED_B1_PIN, LOW);
        server.send(200, "text/plain", "LEDs Off"); });

    server.on("/api/test/buzzer/short", HTTP_POST, []()
              {
        Serial.println("Buzzer short beep test");
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200);
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Buzzer test complete");
        server.send(200, "text/plain", "Short beep"); });

    server.on("/api/test/buzzer/long", HTTP_POST, []()
              {
        Serial.println("Buzzer long beep test");
        digitalWrite(BUZZER_PIN, HIGH);
        delay(1000);
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("Buzzer long test complete");
        server.send(200, "text/plain", "Long beep"); });

    // Calibration endpoints
    server.on("/api/calibrate/esc", HTTP_POST, []()
              {
        calibrateESCs();
        server.send(200, "text/plain", "ESC calibration started"); });

    server.on("/api/calibrate/hover", HTTP_POST, []()
              {
        findHoverThrottle();
        server.send(200, "text/plain", "Hover throttle test started"); });

    server.on("/api/calibrate/mpu6050", HTTP_POST, []()
              {
        calibrateMPU6050();
        server.send(200, "text/plain", "MPU6050 calibration started"); });

    server.on("/api/calibrate/bme280", HTTP_POST, []()
              {
        calibrateBME280Altitude();
        server.send(200, "text/plain", "BME280 altitude calibrated"); });

    server.on("/api/calibrate/ens160", HTTP_POST, []()
              {
        calibrateENS160();
        server.send(200, "text/plain", "ENS160 calibration started"); });

    server.on("/api/test/battery", HTTP_POST, []()
              {
        float battery_v = readBatteryVoltage();
        float battery_p = batteryVoltageToPercentage(battery_v);
        Serial.printf("Battery Test - Voltage: %.2fV, Percentage: %.1f%%\n", battery_v, battery_p);
        
        String result = "Battery: " + String(battery_v, 2) + "V (" + String(battery_p, 1) + "%)";
        server.send(200, "text/plain", result); });

    server.on("/api/calibration/save", HTTP_POST, []()
              {
        if (server.hasArg("plain")) {
            StaticJsonDocument<512> doc;
            deserializeJson(doc, server.arg("plain"));
            
            calibration.esc_min = doc["esc_min"];
            calibration.esc_max = doc["esc_max"];
            calibration.hover_throttle = doc["hover_throttle"];
            calibration.gyro_offset_x = doc["gyro_x"];
            calibration.gyro_offset_y = doc["gyro_y"];
            calibration.gyro_offset_z = doc["gyro_z"];
            calibration.accel_offset_x = doc["accel_x"];
            calibration.accel_offset_y = doc["accel_y"];
            calibration.accel_offset_z = doc["accel_z"];
            
            Serial.println("Calibration data saved");
            server.send(200, "text/plain", "Calibration saved");
        } else {
            server.send(400, "text/plain", "No data received");
        } });

    server.begin();
    Serial.println("Web server started");
}

void updateSensorReadings()
{
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 500)
        return; // Update every 500ms (slower to prevent blocking)
    lastUpdate = millis();

    // Update GPS (non-blocking)
    while (Serial2.available() > 0)
    {
        if (gps.encode(Serial2.read()))
        {
            if (gps.location.isValid())
            {
                sensorData.latitude = gps.location.lat();
                sensorData.longitude = gps.location.lng();
                status.gps = true;
            }
            if (gps.altitude.isValid())
            {
                sensorData.gps_altitude = gps.altitude.meters();
                Serial.printf("GPS Altitude: %.2f m\n", sensorData.gps_altitude);
            }
            if (gps.satellites.isValid())
            {
                sensorData.satellites = gps.satellites.value();
            }
        }
    }

    // Update BME280 with error handling
    if (status.bme280)
    {
        float temp = bme.readTemperature();
        if (!isnan(temp) && temp > -40 && temp < 85)
        {
            sensorData.temperature = temp;
        }

        float hum = bme.readHumidity();
        if (!isnan(hum) && hum >= 0 && hum <= 100)
        {
            sensorData.humidity = hum;
        }

        float press = bme.readPressure();
        if (!isnan(press) && press > 30000 && press < 110000)
        {
            sensorData.pressure = press / 100.0; // Convert to hPa

            // Calculate barometric altitude
            sensorData.baro_altitude = bme.readAltitude(reference_pressure);
            Serial.printf("Barometric Altitude: %.2f m (Pressure: %.2f hPa)\n",
                          sensorData.baro_altitude, sensorData.pressure);
        }
    }

    // Update MPU6050 with error handling
    if (status.mpu6050)
    {
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp))
        {
            if (!isnan(a.acceleration.x))
                sensorData.ax = a.acceleration.x;
            if (!isnan(a.acceleration.y))
                sensorData.ay = a.acceleration.y;
            if (!isnan(a.acceleration.z))
                sensorData.az = a.acceleration.z;
            if (!isnan(g.gyro.x))
                sensorData.gx = g.gyro.x;
            if (!isnan(g.gyro.y))
                sensorData.gy = g.gyro.y;
            if (!isnan(g.gyro.z))
                sensorData.gz = g.gyro.z;
        }
    }

    // Update UV sensor (simple analog read)
    int uvRaw = analogRead(GUVA_PIN);
    sensorData.uv = (uvRaw * 3.3 / 4095.0) * 10.0;

    // Update BH1750 with timeout
    if (status.bh1750)
    {
        float lux = lightMeter.readLightLevel();
        if (!isnan(lux) && lux >= 0)
        {
            sensorData.lux = lux;
        }
    }

    // Update AHT21 with error handling
    if (status.aht21)
    {
        sensors_event_t humidity, temp;
        if (aht21.getEvent(&humidity, &temp))
        {
            if (!isnan(temp.temperature) && temp.temperature > -40 && temp.temperature < 85)
            {
                sensorData.aht21_temperature = temp.temperature;
            }
            if (!isnan(humidity.relative_humidity) && humidity.relative_humidity >= 0 && humidity.relative_humidity <= 100)
            {
                sensorData.aht21_humidity = humidity.relative_humidity;
            }
        }
    }

    // Update ENS160 with environmental data (less frequently)
    static unsigned long lastENS160Update = 0;
    if (status.ens160 && (millis() - lastENS160Update > 2000)) // Every 2 seconds
    {
        lastENS160Update = millis();

        // Update environmental data from AHT21 if available
        if (status.aht21 && !isnan(sensorData.aht21_temperature) && !isnan(sensorData.aht21_humidity))
        {
            ens160.set_envdata(sensorData.aht21_temperature, sensorData.aht21_humidity);
        }

        if (ens160.available())
        {
            ens160.measure();
            float tvoc = ens160.getTVOC();
            float co2 = ens160.geteCO2();
            float aqi = ens160.getAQI();

            if (!isnan(tvoc) && tvoc >= 0)
                sensorData.tvoc = tvoc;
            if (!isnan(co2) && co2 >= 400)
                sensorData.co2 = co2;
            if (!isnan(aqi) && aqi >= 1)
                sensorData.aqi = aqi;
        }
    }

    // Update ToF sensors (less frequently)
    static unsigned long lastToFUpdate = 0;
    if (status.pca9548a && (millis() - lastToFUpdate > 1000)) // Every 1 second
    {
        lastToFUpdate = millis();
        updateToFSensors();
    }

    // Update system info
    sensorData.voltage = 3.3; // ESP32 system voltage

    // Update battery monitoring
    sensorData.battery_voltage = readBatteryVoltage();
    sensorData.battery_percentage = batteryVoltageToPercentage(sensorData.battery_voltage);

    if (WiFi.status() == WL_CONNECTED)
    {
        sensorData.rssi = WiFi.RSSI();
    }
}

void updateToFSensors()
{
    static int currentSensor = 0;
    static unsigned long lastSensorSwitch = 0;

    // Only update one sensor per call to prevent blocking
    if (millis() - lastSensorSwitch > 250)
    { // 250ms per sensor
        lastSensorSwitch = millis();

        switch (currentSensor)
        {
        case 0: // Front ToF
            if (status.tof_front)
            {
                selectMuxChannel(0);
                delay(5);
                int distance = tof_front.readRangeContinuousMillimeters();
                if (!tof_front.timeoutOccurred() && distance < 8000)
                {
                    sensorData.tof_front_distance = distance;
                }
                else
                {
                    sensorData.tof_front_distance = -1;
                }
            }
            break;

        case 1: // Right ToF
            if (status.tof_right)
            {
                selectMuxChannel(1);
                delay(5);
                int distance = tof_right.readRangeContinuousMillimeters();
                if (!tof_right.timeoutOccurred() && distance < 8000)
                {
                    sensorData.tof_right_distance = distance;
                }
                else
                {
                    sensorData.tof_right_distance = -1;
                }
            }
            break;

        case 2: // Back ToF
            if (status.tof_back)
            {
                selectMuxChannel(2);
                delay(5);
                int distance = tof_back.readRangeContinuousMillimeters();
                if (!tof_back.timeoutOccurred() && distance < 8000)
                {
                    sensorData.tof_back_distance = distance;
                }
                else
                {
                    sensorData.tof_back_distance = -1;
                }
            }
            break;

        case 3: // Left ToF
            if (status.tof_left)
            {
                selectMuxChannel(3);
                delay(5);
                int distance = tof_left.readRangeContinuousMillimeters();
                if (!tof_left.timeoutOccurred() && distance < 8000)
                {
                    sensorData.tof_left_distance = distance;
                }
                else
                {
                    sensorData.tof_left_distance = -1;
                }
            }
            break;
        }

        // Disable channels and move to next sensor
        disableMuxChannels();
        currentSensor = (currentSensor + 1) % 4;
    }
}

void calibrateESCs()
{
    Serial.println("Starting ESC calibration...");

    // Send max throttle
    esc1.writeMicroseconds(2000);
    esc2.writeMicroseconds(2000);
    esc3.writeMicroseconds(2000);
    esc4.writeMicroseconds(2000);

    digitalWrite(LED_R1_PIN, HIGH);
    digitalWrite(LED_G1_PIN, HIGH);
    digitalWrite(LED_B1_PIN, LOW); // Yellow

    delay(3000); // Wait 3 seconds

    // Send min throttle
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);

    digitalWrite(LED_R1_PIN, LOW);
    digitalWrite(LED_G1_PIN, HIGH);
    digitalWrite(LED_B1_PIN, LOW); // Green

    Serial.println("ESC calibration complete");
}

void findHoverThrottle()
{
    Serial.println("Starting hover throttle test...");

    if (!motorControl.motors_armed)
    {
        Serial.println("Motors must be armed first!");
        return;
    }

    // Gradually increase throttle
    for (int pwm = 1000; pwm <= 1500; pwm += 10)
    {
        esc1.writeMicroseconds(pwm);
        esc2.writeMicroseconds(pwm);
        esc3.writeMicroseconds(pwm);
        esc4.writeMicroseconds(pwm);

        Serial.printf("Testing PWM: %d\n", pwm);
        delay(500);

        // Check for user abort
        server.handleClient();
    }

    // Return to minimum
    esc1.writeMicroseconds(1000);
    esc2.writeMicroseconds(1000);
    esc3.writeMicroseconds(1000);
    esc4.writeMicroseconds(1000);

    Serial.println("Hover throttle test complete");
}

void calibrateMPU6050()
{
    Serial.println("Calibrating MPU6050... Keep device still!");

    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 1000;

    for (int i = 0; i < samples; i++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        ax_sum += a.acceleration.x;
        ay_sum += a.acceleration.y;
        az_sum += a.acceleration.z;
        gx_sum += g.gyro.x;
        gy_sum += g.gyro.y;
        gz_sum += g.gyro.z;

        delay(10);
    }

    calibration.accel_offset_x = ax_sum / samples;
    calibration.accel_offset_y = ay_sum / samples;
    calibration.accel_offset_z = (az_sum / samples) - 9.81; // Remove gravity
    calibration.gyro_offset_x = gx_sum / samples;
    calibration.gyro_offset_y = gy_sum / samples;
    calibration.gyro_offset_z = gz_sum / samples;

    Serial.println("MPU6050 calibration complete");
    Serial.printf("Accel offsets: X=%.3f Y=%.3f Z=%.3f\n",
                  calibration.accel_offset_x, calibration.accel_offset_y, calibration.accel_offset_z);
    Serial.printf("Gyro offsets: X=%.3f Y=%.3f Z=%.3f\n",
                  calibration.gyro_offset_x, calibration.gyro_offset_y, calibration.gyro_offset_z);
}

void calibrateENS160()
{
    Serial.println("Calibrating ENS160... This may take several minutes!");

    if (!status.ens160)
    {
        Serial.println("ENS160 not available!");
        return;
    }

    // Reset ENS160 baseline
    ens160.setMode(ENS160_OPMODE_RESET);
    delay(100);
    ens160.setMode(ENS160_OPMODE_STD);

    Serial.println("ENS160 reset complete. Allow 3-24 hours for full calibration in clean air.");
}

void calibrateBME280Altitude()
{
    Serial.println("Calibrating BME280 altitude reference...");

    if (!status.bme280)
    {
        Serial.println("BME280 not available!");
        return;
    }

    // Read current pressure and set as reference
    float current_pressure = bme.readPressure() / 100.0; // Convert to hPa
    if (!isnan(current_pressure) && current_pressure > 800 && current_pressure < 1200)
    {
        reference_pressure = current_pressure;
        Serial.printf("BME280 altitude reference set to current pressure: %.2f hPa\n", reference_pressure);
        Serial.println("Current location is now considered 0m altitude");
    }
    else
    {
        Serial.println("Failed to read valid pressure for calibration");
    }
}

// Battery voltage reading function
float readBatteryVoltage()
{
    // Read ADC value (12-bit resolution: 0-4095)
    int rawValue = analogRead(BATTERY_PIN);

    // Convert to voltage at ADC pin (0-3.3V range)
    float vOut = (rawValue / 4095.0) * 3.3;

    // Convert back to actual battery voltage using divider ratio
    // Voltage divider: 30kΩ top, 7.5kΩ bottom = 0.2 ratio
    float vIn = vOut * 5.0; // 1/0.2 = 5.0

    return vIn;
}

// Convert battery voltage to percentage (3S LiPo: 9.0V-12.6V)
float batteryVoltageToPercentage(float voltage)
{
    const float minVoltage = 9.0;  // 3.0V per cell (empty)
    const float maxVoltage = 12.6; // 4.2V per cell (full)

    if (voltage <= minVoltage)
        return 0.0;
    if (voltage >= maxVoltage)
        return 100.0;

    return ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
}
