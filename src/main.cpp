#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <esp_now.h>
#include "functions.h"
#include "esp-now.h"
#include <math.h>


// Initialize PCA9865
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
const uint8_t servoCount = 5;

ServoConfig servos[servoCount] = {
  {0, 10, 170, 90, 90, 1, 0, 20},   // Base
  {1, 22, 180, 90, 160, 1, 0, 20},  // Shoulder
  {2, 20, 163, 163, 22, 1, 0, 20}, // Elbow
  {3, 0, 180, 0, 0, 1, 0, 20},     // Wrist
  {4, 0, 180, 0, 0, 1, 0, 20}       // Gripper
};

// Calibration settings
uint8_t currentServo = 0; // Index of the servo being calibrated
unsigned long lastCalibrationUpdate = 0;
int calibrationStep = 5;  // Degrees to move per step
int calibrationDirection = 1; // 1 = increasing, -1 = decreasing

void setup() {
  Serial.begin(115200);
  while (!Serial); // For some ESP32 boards

  Serial.println("Starting setup...");
  initializeESPNow();
  Serial.println("ESP NOW STARTED...");
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.println("PWM driver initialized.");

  // Initialize servos
  for (uint8_t i = 0; i < servoCount; i++) {
    pwm.setPWM(servos[i].channel, 0, angleToPWM(servos[i].currentAngle));
    servos[i].lastUpdate = millis();
    Serial.print("Initialized servo ");
    Serial.print(servos[i].channel);
    Serial.print(" at ");
    Serial.print(servos[i].currentAngle);
    Serial.println(" degrees.");
  }

  zeroAllMotors(servos);
  updateServos(servos, servoCount);
  delay(400);
}





void loop() {
  updateServos(servos, servoCount);
  delay(10);
}