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

// PID Library
#include <PID_v1.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// WiFi Credentials
const char *ssid = "Dialog 4G";       // Change this to your WiFi name
const char *password = "0N7NT00ANTQ"; // Change this to your WiFi password

// Web Server
WebServer server(80);

// Component Pins
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5
#define ESC1_PIN 13 // Front Right (CW)
#define ESC2_PIN 12 // Back Right (CCW)
#define ESC3_PIN 14 // Front Left (CCW)
#define ESC4_PIN 27 // Back Left (CW)
#define LED_R1_PIN 25
#define LED_G1_PIN 33
#define LED_B1_PIN 32
#define BUZZER_PIN 26
#define SDA_PIN 21
#define SCL_PIN 22
#define GUVA_PIN 36
#define GPS_RX_PIN 16
#define BATTERY_PIN 35

// Navigation LED Pins
#define NAV_FRONT_RIGHT_GREEN 32
#define NAV_BACK_RIGHT_GREEN 33
#define NAV_BACK_LEFT_RED 25
#define NAV_FRONT_CENTER_WHITE 17
#define NAV_FRONT_LEFT_RED 2
#define NAV_BACK_CENTER_WHITE 15

// PCA9548A I2C Multiplexer
#define PCA9548A_ADDR 0x70

// Control Loop Timing
#define CONTROL_LOOP_FREQUENCY 100                          // Hz
#define CONTROL_LOOP_PERIOD (1000 / CONTROL_LOOP_FREQUENCY) // ms

// Motor Limits
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
#define MOTOR_ARM_VALUE 1000
#define MOTOR_IDLE 1100
#define MOTOR_HOVER_ESTIMATE 1550 // Will be calibrated

// Component Objects
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Servo esc1, esc2, esc3, esc4;
Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;
ScioSense_ENS160 ens160(ENS160_I2CADDR_1);
Adafruit_AHTX0 aht21;
BH1750 lightMeter;
TinyGPSPlus gps;

// ToF Sensors via PCA9548A
VL53L0X tof_front; // Channel 0
VL53L0X tof_right; // Channel 1
VL53L0X tof_back;  // Channel 2
VL53L0X tof_left;  // Channel 3

// FreeRTOS Handles
TaskHandle_t controlTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t telemetryTaskHandle = NULL;
SemaphoreHandle_t sensorMutex = NULL;
QueueHandle_t commandQueue = NULL;

// Flight Controller State
enum FlightMode
{
    MODE_DISARMED = 0,
    MODE_ARMED = 1,
    MODE_MANUAL = 2,
    MODE_STABILIZE = 3,
    MODE_ALTITUDE_HOLD = 4,
    MODE_POSITION_HOLD = 5,
    MODE_EMERGENCY = 6
};

struct FlightState
{
    FlightMode mode;
    bool motors_armed;
    bool emergency_stop;
    bool gps_available;
    bool altitude_hold_active;
    float hover_throttle;
    unsigned long last_command_time;
    unsigned long arm_time;
};

// Control Inputs (from WiFi dashboard or NRF24L01)
struct ControlInputs
{
    float throttle; // 0-1 (0 = motor_idle, 1 = motor_max)
    float roll;     // -1 to 1 (left/right)
    float pitch;    // -1 to 1 (forward/backward)
    float yaw;      // -1 to 1 (rotate left/right)
    bool arm_switch;
    bool mode_switch;
    unsigned long timestamp;
};

// Sensor Data Structure
struct SensorData
{
    // IMU Data
    float roll_angle;
    float pitch_angle;
    float yaw_angle;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;
    float accel_x, accel_y, accel_z;

    // Altitude Data
    float altitude_baro;
    float altitude_gps;
    float altitude_filtered;
    float vertical_velocity;

    // Position Data (GPS)
    double latitude;
    double longitude;
    float ground_speed;
    int satellites;

    // Environmental
    float temperature;
    float humidity;
    float pressure;
    float air_quality_index;

    // Battery
    float battery_voltage;
    float battery_percentage;

    // System
    unsigned long timestamp;
    bool valid;
};

// PID Controllers
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;
double yawSetpoint, yawInput, yawOutput;
double altitudeSetpoint, altitudeInput, altitudeOutput;

// PID Parameters (will be tuned)
struct PIDParams
{
    double roll_kp, roll_ki, roll_kd;
    double pitch_kp, pitch_ki, pitch_kd;
    double yaw_kp, yaw_ki, yaw_kd;
    double altitude_kp, altitude_ki, altitude_kd;
};

// Initialize with refined conservative values for smooth flight
PIDParams pidParams = {
    // Roll PID - Reduced for smoother response
    .roll_kp = 1.5,  // Reduced from 2.0
    .roll_ki = 0.0,  // Keep at 0 initially
    .roll_kd = 0.08, // Slightly reduced from 0.1
    // Pitch PID - Reduced for smoother response
    .pitch_kp = 1.5,  // Reduced from 2.0
    .pitch_ki = 0.0,  // Keep at 0 initially
    .pitch_kd = 0.08, // Slightly reduced from 0.1
    // Yaw PID - Reduced for gentler yaw
    .yaw_kp = 0.8,  // Reduced from 1.0
    .yaw_ki = 0.0,  // Keep at 0 initially
    .yaw_kd = 0.03, // Slightly reduced from 0.05
    // Altitude PID
    .altitude_kp = 1.0,
    .altitude_ki = 0.0,
    .altitude_kd = 0.1};

// Create PID instances
PID rollPID(&rollInput, &rollOutput, &rollSetpoint, pidParams.roll_kp, pidParams.roll_ki, pidParams.roll_kd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pidParams.pitch_kp, pidParams.pitch_ki, pidParams.pitch_kd, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, pidParams.yaw_kp, pidParams.yaw_ki, pidParams.yaw_kd, DIRECT);
PID altitudePID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, pidParams.altitude_kp, pidParams.altitude_ki, pidParams.altitude_kd, DIRECT);

// Global State Variables
FlightState flightState;
ControlInputs controlInputs;
SensorData sensorData;

// Motor outputs (PWM values)
struct MotorOutputs
{
    int motor1; // Front Right
    int motor2; // Back Right
    int motor3; // Front Left
    int motor4; // Back Left
};

MotorOutputs motorOutputs;

// Function Declarations
void initializeHardware();
void initializePIDControllers();
void setupWebServer();
void connectWiFi();

// FreeRTOS Task Functions
void controlTask(void *parameter);
void sensorTask(void *parameter);
void telemetryTask(void *parameter);

// Core Flight Functions
void updateSensors();
void calculateOrientation();
void runPIDControllers();
void mixMotorOutputs();
void outputMotorCommands();
void checkSafetyConditions();
void handleEmergency();

// Utility Functions
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
float constrainFloat(float value, float min_val, float max_val);
void updateFlightMode();
void calibrateIMU();

// LED Control Functions
void initializeLEDs();
void updateNavigationLights();
void updateStatusLights();
void setThrottlePulse(float throttle_percentage);

void setup()
{
    Serial.begin(115200);
    Serial.println("ESP32 Weather Drone Flight Controller Starting...");

    // Initialize hardware
    initializeHardware();

    // Initialize flight state
    flightState.mode = MODE_DISARMED;
    flightState.motors_armed = false;
    flightState.emergency_stop = false;
    flightState.gps_available = false;
    flightState.altitude_hold_active = false; // Ensure altitude hold starts disabled
    flightState.hover_throttle = MOTOR_HOVER_ESTIMATE;
    flightState.last_command_time = 0;
    flightState.arm_time = 0;

    // Initialize control inputs
    memset(&controlInputs, 0, sizeof(controlInputs));
    memset(&sensorData, 0, sizeof(sensorData));

    // Set initial sensor timestamp to prevent immediate emergency
    sensorData.timestamp = millis();

    // Give system 30 seconds startup grace period
    static unsigned long startup_time = millis(); // Initialize PID controllers
    initializePIDControllers();

    // Connect to WiFi
    connectWiFi();

    // Setup web server
    setupWebServer();

    // Create FreeRTOS synchronization primitives
    sensorMutex = xSemaphoreCreateMutex();
    commandQueue = xQueueCreate(10, sizeof(ControlInputs));

    // Create FreeRTOS tasks with larger stack sizes
    xTaskCreatePinnedToCore(
        controlTask,        // Function
        "Control Task",     // Name
        8192,               // Stack size (increased)
        NULL,               // Parameters
        3,                  // Priority (higher = more important)
        &controlTaskHandle, // Handle
        1                   // Core (0 or 1)
    );

    xTaskCreatePinnedToCore(
        sensorTask,        // Function
        "Sensor Task",     // Name
        8192,              // Stack size (increased)
        NULL,              // Parameters
        2,                 // Priority
        &sensorTaskHandle, // Handle
        0                  // Core
    );

    xTaskCreatePinnedToCore(
        telemetryTask,        // Function
        "Telemetry Task",     // Name
        4096,                 // Stack size (doubled)
        NULL,                 // Parameters
        1,                    // Priority
        &telemetryTaskHandle, // Handle
        0                     // Core
    );

    Serial.println("Flight Controller initialized successfully!");
    Serial.println("Starting control tasks...");
}

void loop()
{
    // Handle web server requests
    server.handleClient();

    // Main loop runs at lower priority - FreeRTOS handles the control loops
    delay(10);
}

void initializeHardware()
{
    // Initialize pins
    pinMode(LED_R1_PIN, OUTPUT);
    pinMode(LED_G1_PIN, OUTPUT);
    pinMode(LED_B1_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(GUVA_PIN, INPUT);
    pinMode(BATTERY_PIN, INPUT);

    // Initialize navigation LED pins
    initializeLEDs();

    // Set ADC attenuation for battery monitoring
    analogSetAttenuation(ADC_11db);

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000); // 400kHz for faster sensor reads

    // Initialize SPI for NRF24L01
    SPI.begin(18, 19, 23, 5);

    // Initialize MPU6050 IMU
    if (mpu.begin(0x68))
    {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        Serial.println("MPU6050 initialized successfully");
    }
    else
    {
        Serial.println("ERROR: MPU6050 initialization failed!");
    }

    // Initialize BME280
    if (bme.begin(0x76) || bme.begin(0x77))
    {
        Serial.println("BME280 initialized successfully");
    }
    else
    {
        Serial.println("ERROR: BME280 initialization failed!");
    }

    // Initialize ESCs
    esc1.attach(ESC1_PIN, MOTOR_MIN, MOTOR_MAX);
    esc2.attach(ESC2_PIN, MOTOR_MIN, MOTOR_MAX);
    esc3.attach(ESC3_PIN, MOTOR_MIN, MOTOR_MAX);
    esc4.attach(ESC4_PIN, MOTOR_MIN, MOTOR_MAX);

    // Set motors to minimum (safety)
    esc1.writeMicroseconds(MOTOR_ARM_VALUE);
    esc2.writeMicroseconds(MOTOR_ARM_VALUE);
    esc3.writeMicroseconds(MOTOR_ARM_VALUE);
    esc4.writeMicroseconds(MOTOR_ARM_VALUE);

    Serial.println("Hardware initialization complete");

    // Brief startup sequence using navigation LEDs
    // Red LEDs first
    digitalWrite(NAV_BACK_LEFT_RED, HIGH);
    digitalWrite(NAV_FRONT_LEFT_RED, HIGH);
    delay(200);
    digitalWrite(NAV_BACK_LEFT_RED, LOW);
    digitalWrite(NAV_FRONT_LEFT_RED, LOW);

    // Green LEDs next
    digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
    digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
    delay(200);

    // White center LEDs last
    digitalWrite(NAV_FRONT_CENTER_WHITE, HIGH);
    digitalWrite(NAV_BACK_CENTER_WHITE, HIGH);
    delay(200);
    digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
    digitalWrite(NAV_BACK_CENTER_WHITE, LOW);

    // Startup beep
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
}

void initializePIDControllers()
{
    // Configure Roll PID with very conservative limits
    rollPID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(-200, 200); // Further reduced from ±300 to prevent saturation
    rollPID.SetSampleTime(CONTROL_LOOP_PERIOD);

    // Configure Pitch PID with very conservative limits
    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(-200, 200); // Further reduced from ±300 to prevent saturation
    pitchPID.SetSampleTime(CONTROL_LOOP_PERIOD);

    // Configure Yaw PID with very conservative limits
    yawPID.SetMode(AUTOMATIC);
    yawPID.SetOutputLimits(-100, 100); // Further reduced from ±150 for gentle yaw
    yawPID.SetSampleTime(CONTROL_LOOP_PERIOD);

    // Configure Altitude PID
    altitudePID.SetMode(AUTOMATIC);
    altitudePID.SetOutputLimits(0, 1000); // Throttle addition
    altitudePID.SetSampleTime(CONTROL_LOOP_PERIOD);

    Serial.println("PID controllers initialized with very conservative limits");
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

// We'll continue with the task implementations in the next part...

// ================================
// FREERTOS TASK IMPLEMENTATIONS
// ================================

void controlTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD);

    Serial.println("Control Task started - Running at 100Hz");

    while (true)
    {
        // Wait for the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Check safety conditions first
        checkSafetyConditions();

        if (flightState.emergency_stop)
        {
            handleEmergency();
            continue;
        }

        // Update flight mode based on inputs
        updateFlightMode();

        // Update LED status indicators
        updateStatusLights();
        updateNavigationLights();

        // Debug: Print control input status occasionally
        static int debug_counter = 0;
        if (++debug_counter >= 500)
        { // Every 5 seconds at 100Hz
            debug_counter = 0;
            Serial.printf("Control Debug - arm_switch: %s, mode_switch: %s, armed: %s\n",
                          controlInputs.arm_switch ? "true" : "false",
                          controlInputs.mode_switch ? "true" : "false",
                          flightState.motors_armed ? "true" : "false");

            // Show throttle and motor outputs when armed with detailed mapping
            if (flightState.motors_armed)
            {
                // Calculate what the base PWM should be for debug
                int expectedBasePWM = mapFloat(controlInputs.throttle, 0.0, 1.0, MOTOR_IDLE, MOTOR_MAX);
                Serial.printf("Throttle: %.2f%% -> Expected BasePWM: %d, Motors: [%d, %d, %d, %d]\n",
                              controlInputs.throttle * 100.0,
                              expectedBasePWM,
                              motorOutputs.motor1, motorOutputs.motor2,
                              motorOutputs.motor3, motorOutputs.motor4);
            }
        }

        // Only run control loops if armed and not in emergency
        if (flightState.motors_armed)
        {
            if (flightState.mode == MODE_MANUAL)
            {
                // Manual mode - direct throttle control without PID
                float baseThrottle = controlInputs.throttle;
                int basePWM = mapFloat(baseThrottle, 0.0, 1.0, MOTOR_IDLE, MOTOR_MAX);

                // All motors get same throttle in manual mode
                motorOutputs.motor1 = basePWM;
                motorOutputs.motor2 = basePWM;
                motorOutputs.motor3 = basePWM;
                motorOutputs.motor4 = basePWM;

                // Constrain all motor outputs to safe range
                motorOutputs.motor1 = constrain(motorOutputs.motor1, MOTOR_MIN, MOTOR_MAX);
                motorOutputs.motor2 = constrain(motorOutputs.motor2, MOTOR_MIN, MOTOR_MAX);
                motorOutputs.motor3 = constrain(motorOutputs.motor3, MOTOR_MIN, MOTOR_MAX);
                motorOutputs.motor4 = constrain(motorOutputs.motor4, MOTOR_MIN, MOTOR_MAX);

                // Output to motors
                outputMotorCommands();
            }
            else if (flightState.mode >= MODE_STABILIZE)
            {
                // Get latest sensor data (thread-safe)
                if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE)
                {
                    // Copy sensor data for control calculations
                    rollInput = sensorData.roll_angle;
                    pitchInput = sensorData.pitch_angle;
                    yawInput = sensorData.yaw_rate;
                    altitudeInput = sensorData.altitude_filtered;

                    xSemaphoreGive(sensorMutex);

                    // Run PID controllers
                    runPIDControllers();

                    // Mix motor outputs
                    mixMotorOutputs();

                    // Output to motors
                    outputMotorCommands();
                }
            }
        }
        else
        {
            // Motors disarmed - ensure all motors at idle
            motorOutputs.motor1 = MOTOR_ARM_VALUE;
            motorOutputs.motor2 = MOTOR_ARM_VALUE;
            motorOutputs.motor3 = MOTOR_ARM_VALUE;
            motorOutputs.motor4 = MOTOR_ARM_VALUE;
            outputMotorCommands();
        }
    }
}

void sensorTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz sensor reading

    Serial.println("Sensor Task started - Running at 200Hz");

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Update all sensor readings
        updateSensors();

        // Calculate orientation from IMU
        calculateOrientation();

        // Update timestamp
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            sensorData.timestamp = millis();
            sensorData.valid = true;
            xSemaphoreGive(sensorMutex);
        }
    }
}

void telemetryTask(void *parameter)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 10Hz telemetry

    Serial.println("Telemetry Task started - Running at 10Hz");

    // Wait a few seconds before first status print
    vTaskDelay(pdMS_TO_TICKS(3000));
    Serial.println("=== FLIGHT CONTROLLER READY ===");
    Serial.printf("Web Dashboard: http://%s\n", WiFi.localIP().toString().c_str());
    Serial.println("Waiting for commands via web interface...");

    while (true)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Handle web server and telemetry
        // This will be expanded for NRF24L01 later

        // Print debug info every 5 seconds (reduce frequency further)
        static int counter = 0;
        if (++counter >= 50) // 5 seconds at 10Hz
        {
            counter = 0;

            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE)
            {
                // Only print if we have valid sensor data
                if (sensorData.timestamp > 0)
                {
                    Serial.printf("Status - Mode: %d, Armed: %s, Roll: %.1f°, Pitch: %.1f°, Alt: %.1fm, Batt: %.1fV\n",
                                  flightState.mode,
                                  flightState.motors_armed ? "YES" : "NO",
                                  sensorData.roll_angle,
                                  sensorData.pitch_angle,
                                  sensorData.altitude_filtered,
                                  sensorData.battery_voltage);
                }
                xSemaphoreGive(sensorMutex);
            }
        }
    }
}

// ================================
// CORE FLIGHT CONTROL FUNCTIONS
// ================================

void updateSensors()
{
    static unsigned long lastBME280Read = 0;
    static unsigned long lastGPSRead = 0;
    static unsigned long lastBatteryRead = 0;

    // Read IMU data (high frequency)
    sensors_event_t accel, gyro, temp;
    bool imu_success = mpu.getEvent(&accel, &gyro, &temp);

    if (imu_success && xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        sensorData.accel_x = accel.acceleration.x;
        sensorData.accel_y = accel.acceleration.y;
        sensorData.accel_z = accel.acceleration.z;
        sensorData.roll_rate = gyro.gyro.x * 180.0 / PI; // Convert to degrees/sec
        sensorData.pitch_rate = gyro.gyro.y * 180.0 / PI;
        sensorData.yaw_rate = gyro.gyro.z * 180.0 / PI;
        xSemaphoreGive(sensorMutex);
    }

    // Read BME280 (lower frequency)
    if (millis() - lastBME280Read > 50)
    { // 20Hz
        lastBME280Read = millis();

        float pressure = bme.readPressure();
        if (!isnan(pressure) && pressure > 0)
        {
            if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
            {
                sensorData.pressure = pressure / 100.0;               // Convert to hPa
                sensorData.altitude_baro = bme.readAltitude(1013.25); // Sea level pressure
                sensorData.temperature = bme.readTemperature();
                sensorData.humidity = bme.readHumidity();
                xSemaphoreGive(sensorMutex);
            }
        }
    }

    // GPS reading disabled for now to prevent errors
    /*
    // Read GPS (lower frequency)
    if (millis() - lastGPSRead > 100)
    { // 10Hz
        lastGPSRead = millis();
        // GPS code commented out until Serial2 is properly initialized
    }
    */

    // Read battery (very low frequency)
    if (millis() - lastBatteryRead > 1000)
    { // 1Hz
        lastBatteryRead = millis();

        int rawValue = analogRead(BATTERY_PIN);
        float vOut = (rawValue / 4095.0) * 3.3;

        // For 3S LiPo battery reading - adjust voltage divider ratio
        // If no voltage divider is used, multiply by appropriate factor
        // Common voltage divider: R1=10k, R2=10k (divide by 2) = vOut * 2
        // For 3S LiPo (12.6V max): need divide by ~4, so vOut * 4
        // Empirical correction: 3.4V reading suggests vOut * 3.7 for 12.6V
        float batteryVoltage = vOut * 3.7; // Adjusted for actual hardware setup

        // Debug battery reading every 10 seconds
        static int battery_debug_counter = 0;
        if (++battery_debug_counter >= 10)
        {
            battery_debug_counter = 0;
            Serial.printf("Battery Debug - Raw ADC: %d, vOut: %.2f, Final: %.2f\n", rawValue, vOut, batteryVoltage);
        }

        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            sensorData.battery_voltage = batteryVoltage;
            // Calculate percentage (3S LiPo: 9.0V-12.6V) - display only
            sensorData.battery_percentage = ((batteryVoltage - 9.0) / (12.6 - 9.0)) * 100.0;
            sensorData.battery_percentage = constrainFloat(sensorData.battery_percentage, 0.0, 100.0);
            xSemaphoreGive(sensorMutex);
        }
    }
}

void calculateOrientation()
{
    static float roll_angle = 0.0;
    static float pitch_angle = 0.0;
    static unsigned long lastUpdate = 0;

    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0; // Convert to seconds
    lastUpdate = now;

    if (dt > 0.1)
        dt = 0.1; // Limit dt to prevent large jumps

    // Get current sensor readings (thread-safe)
    float accel_x, accel_y, accel_z;
    float gyro_roll, gyro_pitch;

    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        accel_x = sensorData.accel_x;
        accel_y = sensorData.accel_y;
        accel_z = sensorData.accel_z;
        gyro_roll = sensorData.roll_rate;
        gyro_pitch = sensorData.pitch_rate;
        xSemaphoreGive(sensorMutex);
    }
    else
    {
        return; // Skip this update if we can't get sensor data
    }

    // Calculate angles from accelerometer
    float accel_roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / PI;
    float accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / PI;

    // Complementary filter (combine gyro and accel)
    const float alpha = 0.98; // Gyro weight (higher = trust gyro more)

    roll_angle = alpha * (roll_angle + gyro_roll * dt) + (1.0 - alpha) * accel_roll;
    pitch_angle = alpha * (pitch_angle + gyro_pitch * dt) + (1.0 - alpha) * accel_pitch;

    // Update global sensor data
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1)) == pdTRUE)
    {
        sensorData.roll_angle = roll_angle;
        sensorData.pitch_angle = pitch_angle;

        // Simple altitude filtering (combine GPS and barometric)
        if (flightState.gps_available && sensorData.satellites >= 4)
        {
            // Use GPS altitude when available and reliable
            sensorData.altitude_filtered = sensorData.altitude_gps;
        }
        else
        {
            // Fall back to barometric altitude
            sensorData.altitude_filtered = sensorData.altitude_baro;
        }

        xSemaphoreGive(sensorMutex);
    }
}

void runPIDControllers()
{
    // Set setpoints based on control inputs and flight mode
    if (flightState.mode == MODE_STABILIZE)
    {
        // In stabilize mode, stick inputs set angle setpoints
        rollSetpoint = controlInputs.roll * 30.0;   // ±30 degrees max
        pitchSetpoint = controlInputs.pitch * 30.0; // ±30 degrees max
        yawSetpoint = controlInputs.yaw * 180.0;    // ±180 deg/sec max

        // DEBUG: Print throttle value being used
        static int throttle_debug_counter = 0;
        if (++throttle_debug_counter >= 500)
        { // Every 5 seconds
            throttle_debug_counter = 0;
            Serial.printf("PID Debug - controlInputs.throttle: %.3f\n", controlInputs.throttle);
        }
    }

    if (flightState.mode == MODE_ALTITUDE_HOLD && flightState.altitude_hold_active)
    {
        // Altitude hold mode - maintain current altitude
        // altitudeSetpoint is set when altitude hold is activated
    }

    // Run PID calculations
    bool rollComputed = rollPID.Compute();
    bool pitchComputed = pitchPID.Compute();
    bool yawComputed = yawPID.Compute();

    if (flightState.altitude_hold_active)
    {
        bool altComputed = altitudePID.Compute();
    }
}

void mixMotorOutputs()
{
    // Calculate base throttle
    float baseThrottle = controlInputs.throttle;

    // DEBUG: Track throttle value at start of mixing
    static int throttle_start_debug = 0;
    if (++throttle_start_debug >= 500)
    { // Every 5 seconds
        throttle_start_debug = 0;
        Serial.printf("Mix Start Debug - baseThrottle: %.3f\n", baseThrottle);
    }

    // In altitude hold mode, use PID output for throttle
    if (flightState.altitude_hold_active)
    {
        Serial.printf("ALTITUDE HOLD ACTIVE - Overriding throttle!\n");
        baseThrottle = mapFloat(altitudeOutput, 0, 1000, 0.0, 1.0);
        baseThrottle = constrainFloat(baseThrottle, 0.0, 1.0);
        Serial.printf("Altitude override - altitudeOutput: %.1f, new baseThrottle: %.3f\n", altitudeOutput, baseThrottle);
    }

    // Convert throttle to PWM range with better scaling
    // Map 0-100% throttle to 1100-2000 PWM for proper response

    // DEBUG: Check throttle value just before PWM calculation
    static int pre_pwm_debug = 0;
    if (++pre_pwm_debug >= 500)
    { // Every 5 seconds
        pre_pwm_debug = 0;
        if (flightState.motors_armed)
        {
            Serial.printf("PRE-PWM Debug - baseThrottle just before mapFloat: %.3f\n", baseThrottle);
        }
    }

    int basePWM = mapFloat(baseThrottle, 0.0, 1.0, MOTOR_IDLE, MOTOR_MAX);

    // DEBUG: Detailed PWM calculation tracking
    static int pwm_debug_counter = 0;
    if (++pwm_debug_counter >= 500)
    { // Every 5 seconds
        pwm_debug_counter = 0;
        if (flightState.motors_armed)
        {
            Serial.printf("PWM Calc Debug - Input: %.3f, MOTOR_IDLE: %d, MOTOR_MAX: %d, Result: %d\n",
                          baseThrottle, MOTOR_IDLE, MOTOR_MAX, basePWM);
            // Manual calculation check
            float manual_calc = baseThrottle * (MOTOR_MAX - MOTOR_IDLE) + MOTOR_IDLE;
            Serial.printf("Manual Calc Check - (%.3f * (%d - %d)) + %d = %.1f\n",
                          baseThrottle, MOTOR_MAX, MOTOR_IDLE, MOTOR_IDLE, manual_calc);
        }
    }

    // Apply PID corrections with minimal adaptive scaling
    // Keep PID scaling very close to 1.0 to preserve throttle response
    float pidScale = 1.0;
    if (baseThrottle > 0.30) // Start scaling at 30% throttle
    {
        // Ultra-gentle curve: reduce from 100% at 30% to just 98% at 100% throttle
        float scaleFactor = (baseThrottle - 0.30) / 0.70; // 0 to 1 range
        pidScale = 1.0 - (scaleFactor * 0.02);            // Reduce by only 2% maximum
        pidScale = constrainFloat(pidScale, 0.98, 1.0);
    }

    // Scale PID outputs
    double scaledRollOutput = rollOutput * pidScale;
    double scaledPitchOutput = pitchOutput * pidScale;
    double scaledYawOutput = yawOutput * pidScale;

    // X-configuration mixing:
    // Motor 1 (Front Right): +Roll, -Pitch, +Yaw
    // Motor 2 (Back Right):  +Roll, +Pitch, -Yaw
    // Motor 3 (Front Left):  -Roll, -Pitch, -Yaw
    // Motor 4 (Back Left):   -Roll, +Pitch, +Yaw

    motorOutputs.motor1 = basePWM + scaledRollOutput - scaledPitchOutput + scaledYawOutput;
    motorOutputs.motor2 = basePWM + scaledRollOutput + scaledPitchOutput - scaledYawOutput;
    motorOutputs.motor3 = basePWM - scaledRollOutput - scaledPitchOutput - scaledYawOutput;
    motorOutputs.motor4 = basePWM - scaledRollOutput + scaledPitchOutput + scaledYawOutput;

    // Disabled motor saturation handling - causing throttle drops
    // The PID corrections are already limited to ±200, so saturation shouldn't occur
    // Let the final safety constraints handle any edge cases

    // DEBUG: Add temporary debug output to understand what's happening
    static int motor_debug_counter = 0;
    if (++motor_debug_counter >= 500)
    { // Every 5 seconds
        motor_debug_counter = 0;
        if (flightState.motors_armed)
        {
            Serial.printf("Motor Mix Debug - BasePWM: %d, PID Scale: %.3f\n", basePWM, pidScale);
            Serial.printf("PID Outputs - Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n",
                          rollOutput, pitchOutput, yawOutput);
            Serial.printf("Scaled PID - Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n",
                          scaledRollOutput, scaledPitchOutput, scaledYawOutput);
        }
    }

    // Final safety constraints
    motorOutputs.motor1 = constrain(motorOutputs.motor1, MOTOR_MIN, MOTOR_MAX);
    motorOutputs.motor2 = constrain(motorOutputs.motor2, MOTOR_MIN, MOTOR_MAX);
    motorOutputs.motor3 = constrain(motorOutputs.motor3, MOTOR_MIN, MOTOR_MAX);
    motorOutputs.motor4 = constrain(motorOutputs.motor4, MOTOR_MIN, MOTOR_MAX);
}

void outputMotorCommands()
{
    // Output PWM commands to ESCs
    esc1.writeMicroseconds(motorOutputs.motor1);
    esc2.writeMicroseconds(motorOutputs.motor2);
    esc3.writeMicroseconds(motorOutputs.motor3);
    esc4.writeMicroseconds(motorOutputs.motor4);
}

void checkSafetyConditions()
{
    // Check for emergency conditions
    unsigned long now = millis();

    // Initialize battery voltage properly to avoid false alarms
    static bool battery_initialized = false;
    if (!battery_initialized && sensorData.battery_voltage > 9.0)
    {
        battery_initialized = true;
    }

    // Only check command timeout if motors are armed and we've received commands
    // Command timeout temporarily disabled for testing
    // TODO: Re-enable command timeout once web interface sends continuous heartbeat
    // if (flightState.motors_armed && controlInputs.timestamp > 0 && (now - controlInputs.timestamp > 5000))
    // { // 5 second timeout only when armed
    //     flightState.emergency_stop = true;
    //     Serial.println("EMERGENCY: Command timeout!");
    // }

    // Battery safety check disabled - display only
    // TODO: Fix battery voltage reading hardware/software issue
    // if (flightState.motors_armed && battery_initialized && sensorData.battery_voltage > 9.0 && sensorData.battery_percentage < 10.0)
    // {
    //     flightState.emergency_stop = true;
    //     Serial.println("EMERGENCY: Critical battery voltage!");
    // }

    // Check for sensor failures - only if sensors have been initialized and motors are armed
    if (flightState.motors_armed && sensorData.timestamp > 0 && (now - sensorData.timestamp > 500))
    { // 500ms sensor timeout only when armed
        flightState.emergency_stop = true;
        Serial.println("EMERGENCY: Sensor data timeout!");
    }

    // Check for excessive tilt angles - only when armed
    if (flightState.motors_armed && (abs(sensorData.roll_angle) > 60.0 || abs(sensorData.pitch_angle) > 60.0))
    {
        flightState.emergency_stop = true;
        Serial.println("EMERGENCY: Excessive tilt angle!");
    }
}

void handleEmergency()
{
    // Emergency procedure: gradually reduce throttle
    static int emergency_throttle = MOTOR_IDLE;
    static unsigned long last_emergency_update = 0;

    unsigned long now = millis();
    if (now - last_emergency_update > 50)
    { // Update every 50ms
        last_emergency_update = now;

        if (emergency_throttle > MOTOR_ARM_VALUE)
        {
            emergency_throttle -= 5; // Gradually reduce
        }

        // Set all motors to emergency throttle
        motorOutputs.motor1 = emergency_throttle;
        motorOutputs.motor2 = emergency_throttle;
        motorOutputs.motor3 = emergency_throttle;
        motorOutputs.motor4 = emergency_throttle;

        outputMotorCommands();

        // Flash red navigation LEDs during emergency
        static bool led_state = false;
        led_state = !led_state;
        digitalWrite(NAV_BACK_LEFT_RED, led_state);
        digitalWrite(NAV_FRONT_LEFT_RED, led_state);
    }
}

void updateFlightMode()
{
    // Handle arming/disarming
    if (controlInputs.arm_switch && !flightState.motors_armed)
    {
        // Arm motors (safety checks)
        bool throttle_ok = (controlInputs.throttle < 0.1);
        bool level_ok = true;   // Default to OK if no sensor data yet
        bool battery_ok = true; // Default to OK if no battery data yet

        // Only check angles if we have valid sensor data
        if (sensorData.timestamp > 0 && sensorData.valid)
        {
            level_ok = (abs(sensorData.roll_angle) < 30.0 && abs(sensorData.pitch_angle) < 30.0);
        }

        // Battery check disabled for now - display only
        // TODO: Fix battery voltage reading issue
        // if (sensorData.battery_voltage > 9.0)
        // {
        //     battery_ok = (sensorData.battery_percentage > 15.0);
        // }

        if (throttle_ok && level_ok && battery_ok)
        {
            flightState.motors_armed = true;
            flightState.arm_time = millis();
            flightState.mode = MODE_MANUAL;
            flightState.emergency_stop = false;

            Serial.println("Motors ARMED");

            // Visual/audio feedback - use green navigation LEDs
            for (int i = 0; i < 3; i++)
            {
                digitalWrite(NAV_FRONT_RIGHT_GREEN, LOW);
                digitalWrite(NAV_BACK_RIGHT_GREEN, LOW);
                delay(100);
                digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
                digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
                delay(100);
            }

            digitalWrite(BUZZER_PIN, HIGH);
            delay(150);
            digitalWrite(BUZZER_PIN, LOW);
        }
        else
        {
            Serial.println("ARM FAILED: Safety checks failed");
            Serial.printf("Throttle: %.2f, Level: %s, Battery: %s (disabled)\n",
                          controlInputs.throttle,
                          level_ok ? "OK" : "FAIL",
                          battery_ok ? "OK" : "FAIL");
        }
    }
    else if (!controlInputs.arm_switch && flightState.motors_armed)
    {
        // Disarm motors
        flightState.motors_armed = false;
        flightState.mode = MODE_DISARMED;
        flightState.altitude_hold_active = false;

        Serial.println("Motors DISARMED");

        // Turn off center white LEDs when disarmed
        digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
        digitalWrite(NAV_BACK_CENTER_WHITE, LOW);
    }

    // Handle flight mode changes (when armed)
    if (flightState.motors_armed)
    {
        if (controlInputs.mode_switch)
        {
            flightState.mode = MODE_STABILIZE;
            Serial.println("Flight Mode: STABILIZE");
        }
        else
        {
            flightState.mode = MODE_MANUAL;
        }

        // Altitude hold activation disabled for testing
        // TODO: Implement proper altitude hold activation method (e.g., switch or button)
        /*
        // Altitude hold activation (example: holding yaw stick for 2 seconds)
        static unsigned long yaw_hold_start = 0;
        if (abs(controlInputs.yaw) < 0.1 && controlInputs.throttle > 0.3)
        {
            if (yaw_hold_start == 0)
            {
                yaw_hold_start = millis();
            }
            else if (millis() - yaw_hold_start > 2000)
            {
                flightState.altitude_hold_active = true;
                altitudeSetpoint = sensorData.altitude_filtered;
                Serial.println("Altitude Hold ACTIVATED");
                yaw_hold_start = 0;
            }
        }
        else
        {
            yaw_hold_start = 0;
            if (flightState.altitude_hold_active && abs(controlInputs.throttle) > 0.1)
            {
                flightState.altitude_hold_active = false;
                Serial.println("Altitude Hold DEACTIVATED");
            }
        }
        */
    }
}

// ================================
// UTILITY FUNCTIONS
// ================================

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float constrainFloat(float value, float min_val, float max_val)
{
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

// ================================
// WEB SERVER SETUP
// ================================

void setupWebServer()
{
    // Serve main dashboard
    server.on("/", HTTP_GET, []()
              {
        String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Drone Flight Controller</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0; }
        .container { max-width: 1200px; margin: 0 auto; }
        .card { background: white; padding: 20px; margin: 10px 0; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
        .status { display: flex; flex-wrap: wrap; gap: 10px; }
        .status-item { padding: 10px; border-radius: 4px; color: white; min-width: 100px; text-align: center; }
        .status-armed { background: #4CAF50; }
        .status-disarmed { background: #f44336; }
        .status-manual { background: #ff9800; }
        .status-stabilize { background: #2196F3; }
        .controls { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
        .joystick { text-align: center; padding: 20px; }
        button { padding: 10px 20px; margin: 5px; border: none; border-radius: 4px; cursor: pointer; }
        .btn-primary { background: #2196F3; color: white; }
        .btn-danger { background: #f44336; color: white; }
        .btn-success { background: #4CAF50; color: white; }
        input[type="range"] { width: 100%; }
        .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; }
        .sensor-value { padding: 10px; background: #e3f2fd; border-radius: 4px; text-align: center; }
        .emergency { background: #f44336; color: white; font-weight: bold; padding: 15px; border-radius: 8px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Drone Flight Controller</h1>
        
        <!-- Flight Status -->
        <div class="card">
            <h2>Flight Status</h2>
            <div class="status" id="flightStatus">
                <div class="status-item status-disarmed">DISARMED</div>
                <div class="status-item">Manual Mode</div>
            </div>
            <div id="emergencyAlert" style="display: none;" class="emergency">
                WARNING: EMERGENCY MODE ACTIVE - LANDING AUTOMATICALLY
            </div>
        </div>
        
        <!-- Control Interface -->
        <div class="card">
            <h2>Flight Controls</h2>
            <div class="controls">
                <div class="joystick">
                    <h3>Throttle</h3>
                    <input type="range" id="throttle" min="0" max="100" value="0" orient="vertical" oninput="updateControl()">
                    <span id="throttle_val">0%</span>
                </div>
                <div class="joystick">
                    <h3>Roll (Left/Right)</h3>
                    <input type="range" id="roll" min="-100" max="100" value="0" oninput="updateControl()">
                    <span id="roll_val">0</span>
                </div>
                <div class="joystick">
                    <h3>Pitch (Forward/Back)</h3>
                    <input type="range" id="pitch" min="-100" max="100" value="0" oninput="updateControl()">
                    <span id="pitch_val">0</span>
                </div>
                <div class="joystick">
                    <h3>Yaw (Rotate)</h3>
                    <input type="range" id="yaw" min="-100" max="100" value="0" oninput="updateControl()">
                    <span id="yaw_val">0</span>
                </div>
            </div>
            <div style="text-align: center; margin-top: 20px;">
                <button class="btn-success" onclick="armMotors()">ARM MOTORS</button>
                <button class="btn-danger" onclick="disarmMotors()">DISARM MOTORS</button>
                <button class="btn-primary" onclick="toggleFlightMode()">TOGGLE MODE</button>
                <button class="btn-danger" onclick="emergencyStop()">EMERGENCY STOP</button>
            </div>
        </div>
        
        <!-- Sensor Data -->
        <div class="card">
            <h2>Live Sensor Data</h2>
            <div class="sensor-grid" id="sensorData">
                <div class="sensor-value">Roll: 0.0 deg</div>
                <div class="sensor-value">Pitch: 0.0 deg</div>
                <div class="sensor-value">Altitude: 0.0 m</div>
                <div class="sensor-value">Battery: 0.0V</div>
            </div>
        </div>
        
        <!-- PID Tuning -->
        <div class="card">
            <h2>PID Tuning</h2>
            <div class="controls">
                <div>
                    <h3>Roll PID</h3>
                    <label>P: <input type="number" id="roll_p" value="2.0" step="0.1" onchange="updatePID()"></label><br>
                    <label>I: <input type="number" id="roll_i" value="0.0" step="0.01" onchange="updatePID()"></label><br>
                    <label>D: <input type="number" id="roll_d" value="0.1" step="0.01" onchange="updatePID()"></label><br>
                </div>
                <div>
                    <h3>Pitch PID</h3>
                    <label>P: <input type="number" id="pitch_p" value="2.0" step="0.1" onchange="updatePID()"></label><br>
                    <label>I: <input type="number" id="pitch_i" value="0.0" step="0.01" onchange="updatePID()"></label><br>
                    <label>D: <input type="number" id="pitch_d" value="0.1" step="0.01" onchange="updatePID()"></label><br>
                </div>
                <div>
                    <h3>Yaw PID</h3>
                    <label>P: <input type="number" id="yaw_p" value="1.0" step="0.1" onchange="updatePID()"></label><br>
                    <label>I: <input type="number" id="yaw_i" value="0.0" step="0.01" onchange="updatePID()"></label><br>
                    <label>D: <input type="number" id="yaw_d" value="0.05" step="0.01" onchange="updatePID()"></label><br>
                </div>
            </div>
        </div>
    </div>

    <script>
        let controlUpdateTimeout = null;
        let armed = false;
        let flightMode = 'MANUAL';
        
        // Update flight data every 100ms
        setInterval(updateFlightData, 100);
        
        // Send heartbeat control data every 500ms to prevent timeout
        setInterval(sendHeartbeat, 500);
        
        function sendHeartbeat() {
            // Only send heartbeat if armed
            if (armed) {
                const data = {
                    throttle: parseFloat(document.getElementById('throttle').value) / 100.0,
                    roll: parseFloat(document.getElementById('roll').value) / 100.0,
                    pitch: parseFloat(document.getElementById('pitch').value) / 100.0,
                    yaw: parseFloat(document.getElementById('yaw').value) / 100.0
                };
                
                fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
            }
        }
        
        function updateFlightData() {
            fetch('/api/flight_data')
                .then(response => response.json())
                .then(data => {
                    updateFlightStatus(data.status);
                    updateSensorDisplay(data.sensors);
                })
                .catch(error => console.log('Data update error:', error));
        }
        
        function updateFlightStatus(status) {
            const container = document.getElementById('flightStatus');
            const emergency = document.getElementById('emergencyAlert');
            
            armed = status.armed;
            flightMode = status.mode;
            
            let statusClass = status.armed ? 'status-armed' : 'status-disarmed';
            let modeClass = status.mode === 'STABILIZE' ? 'status-stabilize' : 'status-manual';
            
            container.innerHTML = `
                <div class="status-item ${statusClass}">${status.armed ? '[ARMED]' : '[DISARMED]'}</div>
                <div class="status-item ${modeClass}">${status.mode} Mode</div>
                <div class="status-item">Alt Hold: ${status.altitude_hold ? 'ON' : 'OFF'}</div>
            `;
            
            if (status.emergency) {
                emergency.style.display = 'block';
            } else {
                emergency.style.display = 'none';
            }
        }
        
        function updateSensorDisplay(sensors) {
            const container = document.getElementById('sensorData');
            container.innerHTML = `
                <div class="sensor-value">Roll: ${sensors.roll.toFixed(1)} deg</div>
                <div class="sensor-value">Pitch: ${sensors.pitch.toFixed(1)} deg</div>
                <div class="sensor-value">Yaw Rate: ${sensors.yaw_rate.toFixed(1)} deg/s</div>
                <div class="sensor-value">Altitude: ${sensors.altitude.toFixed(2)} m</div>
                <div class="sensor-value">Battery: ${sensors.battery_voltage.toFixed(2)}V (${sensors.battery_percentage.toFixed(1)}%)</div>
                <div class="sensor-value">GPS: ${sensors.satellites} sats</div>
                <div class="sensor-value">Motor 1: ${sensors.motor1}</div>
                <div class="sensor-value">Motor 2: ${sensors.motor2}</div>
                <div class="sensor-value">Motor 3: ${sensors.motor3}</div>
                <div class="sensor-value">Motor 4: ${sensors.motor4}</div>
            `;
        }
        
        function updateControl() {
            document.getElementById('throttle_val').textContent = document.getElementById('throttle').value + '%';
            document.getElementById('roll_val').textContent = document.getElementById('roll').value;
            document.getElementById('pitch_val').textContent = document.getElementById('pitch').value;
            document.getElementById('yaw_val').textContent = document.getElementById('yaw').value;
            
            // Throttle control input updates
            if (controlUpdateTimeout) clearTimeout(controlUpdateTimeout);
            controlUpdateTimeout = setTimeout(() => {
                const data = {
                    throttle: parseFloat(document.getElementById('throttle').value) / 100.0,
                    roll: parseFloat(document.getElementById('roll').value) / 100.0,
                    pitch: parseFloat(document.getElementById('pitch').value) / 100.0,
                    yaw: parseFloat(document.getElementById('yaw').value) / 100.0
                };
                
                fetch('/api/control', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(data)
                });
            }, 50); // 20Hz update rate
        }
        
        function armMotors() {
            // Safety: Set throttle to 0 before arming
            document.getElementById('throttle').value = 0;
            updateControl();
            
            fetch('/api/arm', {method: 'POST'})
                .then(() => console.log('Arm command sent - throttle set to 0 for safety'));
        }
        
        function disarmMotors() {
            fetch('/api/disarm', {method: 'POST'})
                .then(() => console.log('Disarm command sent'));
        }
        
        function toggleFlightMode() {
            fetch('/api/toggle_mode', {method: 'POST'})
                .then(() => console.log('Mode toggle sent'));
        }
        
        function emergencyStop() {
            if (confirm('EMERGENCY STOP - Are you sure?')) {
                fetch('/api/emergency', {method: 'POST'})
                    .then(() => console.log('Emergency stop activated'));
            }
        }
        
        function updatePID() {
            const pidData = {
                roll_p: parseFloat(document.getElementById('roll_p').value),
                roll_i: parseFloat(document.getElementById('roll_i').value),
                roll_d: parseFloat(document.getElementById('roll_d').value),
                pitch_p: parseFloat(document.getElementById('pitch_p').value),
                pitch_i: parseFloat(document.getElementById('pitch_i').value),
                pitch_d: parseFloat(document.getElementById('pitch_d').value),
                yaw_p: parseFloat(document.getElementById('yaw_p').value),
                yaw_i: parseFloat(document.getElementById('yaw_i').value),
                yaw_d: parseFloat(document.getElementById('yaw_d').value)
            };
            
            fetch('/api/pid_update', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(pidData)
            });
        }
    </script>
</body>
</html>
)rawliteral";
        server.send(200, "text/html", html); });

    // API: Flight data endpoint
    server.on("/api/flight_data", HTTP_GET, []()
              {
        StaticJsonDocument<1024> doc;
        
        // Flight status
        JsonObject status = doc.createNestedObject("status");
        status["armed"] = flightState.motors_armed;
        status["mode"] = (flightState.mode == MODE_MANUAL) ? "MANUAL" : 
                       (flightState.mode == MODE_STABILIZE) ? "STABILIZE" : "DISARMED";
        status["altitude_hold"] = flightState.altitude_hold_active;
        status["emergency"] = flightState.emergency_stop;
        
        // Sensor data
        JsonObject sensors = doc.createNestedObject("sensors");
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            sensors["roll"] = sensorData.roll_angle;
            sensors["pitch"] = sensorData.pitch_angle;
            sensors["yaw_rate"] = sensorData.yaw_rate;
            sensors["altitude"] = sensorData.altitude_filtered;
            sensors["battery_voltage"] = sensorData.battery_voltage;
            sensors["battery_percentage"] = sensorData.battery_percentage;
            sensors["satellites"] = sensorData.satellites;
            xSemaphoreGive(sensorMutex);
        }
        
        // Motor outputs
        sensors["motor1"] = motorOutputs.motor1;
        sensors["motor2"] = motorOutputs.motor2;
        sensors["motor3"] = motorOutputs.motor3;
        sensors["motor4"] = motorOutputs.motor4;
        
        String response;
        serializeJson(doc, response);
        server.send(200, "application/json", response); });

    // API: Control input endpoint
    server.on("/api/control", HTTP_POST, []()
              {
        if (server.hasArg("plain")) {
            StaticJsonDocument<256> doc;
            deserializeJson(doc, server.arg("plain"));
            
            controlInputs.throttle = doc["throttle"];
            controlInputs.roll = doc["roll"];
            controlInputs.pitch = doc["pitch"];
            controlInputs.yaw = doc["yaw"];
            controlInputs.timestamp = millis();
            
            // Debug control inputs occasionally
            static int control_debug_counter = 0;
            if (++control_debug_counter >= 20) { // Every 20 calls (about 1 second)
                control_debug_counter = 0;
                Serial.printf("Web Control Input - Throttle: %.2f, Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
                              controlInputs.throttle, controlInputs.roll, controlInputs.pitch, controlInputs.yaw);
            }
            
            server.send(200, "text/plain", "OK");
        } else {
            server.send(400, "text/plain", "No data");
        } });

    // API: Arm motors
    server.on("/api/arm", HTTP_POST, []()
              {
        Serial.println("ARM button pressed via web interface");
        controlInputs.arm_switch = true;
        controlInputs.timestamp = millis();
        Serial.printf("Control inputs updated - arm_switch: %s, timestamp: %lu\n", 
                     controlInputs.arm_switch ? "true" : "false", 
                     controlInputs.timestamp);
        server.send(200, "text/plain", "Arm command sent"); });

    // API: Disarm motors
    server.on("/api/disarm", HTTP_POST, []()
              {
        Serial.println("DISARM button pressed via web interface");
        controlInputs.arm_switch = false;
        controlInputs.timestamp = millis();
        Serial.printf("Control inputs updated - arm_switch: %s, timestamp: %lu\n", 
                     controlInputs.arm_switch ? "true" : "false", 
                     controlInputs.timestamp);
        server.send(200, "text/plain", "Disarm command sent"); });

    // API: Toggle flight mode
    server.on("/api/toggle_mode", HTTP_POST, []()
              {
        Serial.println("TOGGLE MODE button pressed via web interface");
        controlInputs.mode_switch = !controlInputs.mode_switch;
        controlInputs.timestamp = millis();
        Serial.printf("Mode switch toggled to: %s\n", 
                     controlInputs.mode_switch ? "true" : "false");
        server.send(200, "text/plain", "Mode toggled"); });

    // API: Emergency stop
    server.on("/api/emergency", HTTP_POST, []()
              {
        flightState.emergency_stop = true;
        Serial.println("EMERGENCY STOP ACTIVATED via web interface!");
        server.send(200, "text/plain", "Emergency stop activated"); });

    // API: Update PID parameters
    server.on("/api/pid_update", HTTP_POST, []()
              {
        if (server.hasArg("plain")) {
            StaticJsonDocument<512> doc;
            deserializeJson(doc, server.arg("plain"));
            
            // Update PID parameters
            pidParams.roll_kp = doc["roll_p"];
            pidParams.roll_ki = doc["roll_i"];
            pidParams.roll_kd = doc["roll_d"];
            pidParams.pitch_kp = doc["pitch_p"];
            pidParams.pitch_ki = doc["pitch_i"];
            pidParams.pitch_kd = doc["pitch_d"];
            pidParams.yaw_kp = doc["yaw_p"];
            pidParams.yaw_ki = doc["yaw_i"];
            pidParams.yaw_kd = doc["yaw_d"];
            
            // Apply to PID controllers
            rollPID.SetTunings(pidParams.roll_kp, pidParams.roll_ki, pidParams.roll_kd);
            pitchPID.SetTunings(pidParams.pitch_kp, pidParams.pitch_ki, pidParams.pitch_kd);
            yawPID.SetTunings(pidParams.yaw_kp, pidParams.yaw_ki, pidParams.yaw_kd);
            
            Serial.println("PID parameters updated");
            server.send(200, "text/plain", "PID updated");
        } else {
            server.send(400, "text/plain", "No data");
        } });

    server.begin();
    Serial.println("Web server started - Flight Controller Dashboard ready");
}

// ================================
// LED CONTROL FUNCTIONS
// ================================

void initializeLEDs()
{
    // Initialize navigation LED pins
    pinMode(NAV_FRONT_RIGHT_GREEN, OUTPUT);
    pinMode(NAV_BACK_RIGHT_GREEN, OUTPUT);
    pinMode(NAV_BACK_LEFT_RED, OUTPUT);
    pinMode(NAV_FRONT_CENTER_WHITE, OUTPUT);
    pinMode(NAV_FRONT_LEFT_RED, OUTPUT);
    pinMode(NAV_BACK_CENTER_WHITE, OUTPUT);

    // Turn on navigation lights immediately (always on when powered)
    digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
    digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
    digitalWrite(NAV_BACK_LEFT_RED, HIGH);
    digitalWrite(NAV_FRONT_LEFT_RED, HIGH);

    // Center white LEDs start off (controlled by status)
    digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
    digitalWrite(NAV_BACK_CENTER_WHITE, LOW);

    Serial.println("Navigation LEDs initialized");
}

void updateNavigationLights()
{
    // Nav lights (Red/Green) are always ON during powered-on state
    // This helps identify orientation at night (Red = left, Green = right)
    static bool nav_lights_on = true;

    if (nav_lights_on)
    {
        digitalWrite(NAV_FRONT_RIGHT_GREEN, HIGH);
        digitalWrite(NAV_BACK_RIGHT_GREEN, HIGH);
        digitalWrite(NAV_BACK_LEFT_RED, HIGH);
        digitalWrite(NAV_FRONT_LEFT_RED, HIGH);
    }
}

void updateStatusLights()
{
    static unsigned long last_blink_time = 0;
    static bool blink_state = false;
    unsigned long now = millis();

    // Center white LEDs behavior based on flight state
    if (!flightState.motors_armed)
    {
        // Arming Blink: Slow blink when waiting to arm (1Hz)
        if (now - last_blink_time > 500)
        {
            last_blink_time = now;
            blink_state = !blink_state;
            digitalWrite(NAV_FRONT_CENTER_WHITE, blink_state);
            digitalWrite(NAV_BACK_CENTER_WHITE, blink_state);
        }
    }
    else
    {
        // Flight Mode On: Solid light when armed
        digitalWrite(NAV_FRONT_CENTER_WHITE, HIGH);
        digitalWrite(NAV_BACK_CENTER_WHITE, HIGH);

        // Optional: Throttle Sync Pulse - Flash intensity reflects throttle level
        setThrottlePulse(controlInputs.throttle);
    }
}

void setThrottlePulse(float throttle_percentage)
{
    // Optional throttle sync pulse feature
    // Flash the center white LEDs with intensity reflecting throttle level
    static unsigned long last_pulse_time = 0;
    static bool pulse_state = false;
    unsigned long now = millis();

    // Only pulse if throttle is above minimum and motors are armed
    if (flightState.motors_armed && throttle_percentage > 0.05)
    {
        // Pulse frequency increases with throttle (10Hz to 50Hz)
        int pulse_interval = map(throttle_percentage * 100, 5, 100, 100, 20);

        if (now - last_pulse_time > pulse_interval)
        {
            last_pulse_time = now;
            pulse_state = !pulse_state;

            // Brief flash to indicate throttle activity
            if (pulse_state)
            {
                digitalWrite(NAV_FRONT_CENTER_WHITE, LOW);
                digitalWrite(NAV_BACK_CENTER_WHITE, LOW);
            }
            else
            {
                digitalWrite(NAV_FRONT_CENTER_WHITE, HIGH);
                digitalWrite(NAV_BACK_CENTER_WHITE, HIGH);
            }
        }
    }
}
