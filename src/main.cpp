#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <esp_now.h>
#include "functions.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const uint8_t servoCount = 5;

ServoConfig servos[servoCount] = {
  {0, 10, 170, 90, 90, 1, 0, 20},   // Base
  {1, 15, 160, 90, 90, 1, 0, 20},  // Shoulder
  {2, 0, 180, 180, 180, 1, 0, 20}, // Elbow
  {3, 0, 180, 0, 0, 1, 0, 20},     // Wrist
  {4, 0, 90, 0, 0, 1, 0, 20}       // Gripper
};

void setup() {
  Serial.begin(115200);
  while (!Serial); // For some ESP32 boards

  Serial.println("Starting setup...");

  pwm.begin();
  pwm.setPWMFreq(50);
  Serial.println("PWM driver initialized.");

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
}

void loop() {
  updateServos(servos, servoCount);
}
