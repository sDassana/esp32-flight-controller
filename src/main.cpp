#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <VL53L0X.h>
#include <RF24.h>
#include <Servo.h>

// NRF24L01 Pins
#define NRF_CE_PIN 4
#define NRF_CSN_PIN 5
#define NRF_SCK_PIN 18
#define NRF_MOSI_PIN 23
#define NRF_MISO_PIN 19

// ESC PWM Pins
#define ESC1_PIN 13
#define ESC2_PIN 12
#define ESC3_PIN 14
#define ESC4_PIN 27

// I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// I2C Multiplexer
#define PCA9548A_ADDR 0x70

// Sensor Addresses
#define MPU6050_ADDR 0x68
#define BMP280_ADDR 0x76

// RGB LED Pins (example mapping, adjust as needed)
#define LED_R1_PIN 25
#define LED_G1_PIN 26
#define LED_B1_PIN 32
#define LED_R2_PIN 33
#define LED_G2_PIN 14
#define LED_B2_PIN 27

// Objects
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;
VL53L0X vl53[4]; // 4 ToF sensors

// Joystick/Remote data structure
struct JoystickData
{
  uint16_t throttle; // 0-1023
  int16_t yaw;       // -512 to 512
  int16_t pitch;     // -512 to 512
  int16_t roll;      // -512 to 512
};

JoystickData joyData = {0, 0, 0, 0};

// Servo objects for ESCs
Servo esc1, esc2, esc3, esc4;
const int pwmMin = 1000; // Min pulse width (us)
const int pwmMax = 2000; // Max pulse width (us)

void selectI2CMuxChannel(uint8_t channel)
{
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(115200);

  // I2C and SPI Init
  Wire.begin(SDA_PIN, SCL_PIN);
  SPI.begin(NRF_SCK_PIN, NRF_MISO_PIN, NRF_MOSI_PIN, NRF_CSN_PIN);

  // NRF24L01 Init
  if (!radio.begin())
  {
    Serial.println("NRF24L01 not found!");
  }
  // radio.setChannel(...); // Set as needed

  // ESC PWM setup (Servo)
  esc1.attach(ESC1_PIN, pwmMin, pwmMax);
  esc2.attach(ESC2_PIN, pwmMin, pwmMax);
  esc3.attach(ESC3_PIN, pwmMin, pwmMax);
  esc4.attach(ESC4_PIN, pwmMin, pwmMax);
  // Initialize ESCs to minimum
  esc1.writeMicroseconds(pwmMin);
  esc2.writeMicroseconds(pwmMin);
  esc3.writeMicroseconds(pwmMin);
  esc4.writeMicroseconds(pwmMin);

  // RGB LEDs
  pinMode(LED_R1_PIN, OUTPUT);
  pinMode(LED_G1_PIN, OUTPUT);
  pinMode(LED_B1_PIN, OUTPUT);
  pinMode(LED_R2_PIN, OUTPUT);
  pinMode(LED_G2_PIN, OUTPUT);
  pinMode(LED_B2_PIN, OUTPUT);

  // MPU6050 Init
  if (!mpu.begin(MPU6050_ADDR))
  {
    Serial.println("MPU6050 not found!");
  }

  // BMP280 Init
  if (!bmp.begin(BMP280_ADDR))
  {
    Serial.println("BMP280 not found!");
  }

  // VL53L0X Init via PCA9548A
  for (uint8_t i = 0; i < 4; i++)
  {
    selectI2CMuxChannel(i);
    vl53[i].setTimeout(500);
    if (!vl53[i].init())
    {
      Serial.printf("VL53L0X #%d not found!\n", i);
    }
  }

  // Optional: Set up PWM channels for ESCs and LEDs (using ledcAttachPin/ledcWrite for ESP32)
}

// ESC arming state machine
enum ESCState
{
  ESC_ARM_MAX,
  ESC_ARM_MIN,
  ESC_READY
};
ESCState escState = ESC_ARM_MAX;
unsigned long escArmStart = 0;

void loop()
{
  static bool firstLoop = true;
  if (firstLoop)
  {
    escArmStart = millis();
    firstLoop = false;
  }

  // 1. Receive control signals (NRF24L01)
  if (radio.available())
  {
    JoystickData rxData;
    radio.read(&rxData, sizeof(rxData));
    joyData = rxData;
  }

  // 2. Read IMU (MPU6050)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 3. Read Barometer (BMP280)
  float altitude = bmp.readAltitude(1013.25);

  // 4. Read ToF Sensors (VL53L0X via PCA9548A)
  uint16_t distances[4];
  for (uint8_t i = 0; i < 4; i++)
  {
    selectI2CMuxChannel(i);
    distances[i] = vl53[i].readRangeSingleMillimeters();
  }

  // 5. ESC arming state machine
  switch (escState)
  {
  case ESC_ARM_MAX:
    // Send max throttle
    esc1.writeMicroseconds(pwmMax);
    esc2.writeMicroseconds(pwmMax);
    esc3.writeMicroseconds(pwmMax);
    esc4.writeMicroseconds(pwmMax);
    if (millis() - escArmStart > 1000)
    {
      escState = ESC_ARM_MIN;
      escArmStart = millis();
    }
    break;
  case ESC_ARM_MIN:
    // Send min throttle
    esc1.writeMicroseconds(pwmMin);
    esc2.writeMicroseconds(pwmMin);
    esc3.writeMicroseconds(pwmMin);
    esc4.writeMicroseconds(pwmMin);
    if (millis() - escArmStart > 1000)
    {
      escState = ESC_READY;
    }
    break;
  case ESC_READY:
    // Normal operation: map joystick throttle to ESCs
    {
      uint16_t esc_pwm = map(joyData.throttle, 0, 1023, pwmMin, pwmMax);
      esc1.writeMicroseconds(esc_pwm);
      esc2.writeMicroseconds(esc_pwm);
      esc3.writeMicroseconds(esc_pwm);
      esc4.writeMicroseconds(esc_pwm);
    }
    break;
  }

  // 6. RGB LED Status (example: set to green)
  digitalWrite(LED_R1_PIN, LOW);
  digitalWrite(LED_G1_PIN, HIGH);
  digitalWrite(LED_B1_PIN, LOW);

  // 7. Non-blocking delay or task scheduling
  delay(10); // Replace with FreeRTOS tasks or timers for real application
}