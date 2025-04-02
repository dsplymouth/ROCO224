#include "functions.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

extern Adafruit_PWMServoDriver pwm;

uint16_t angleToPWM(int angle) {
  angle = constrain(angle, 0, 180);
  float pulseMicros = map(angle, 0, 180, 500, 2500);
  return (uint16_t)((pulseMicros / 20000.0) * 4096);
}

void moveServoToAngle(ServoConfig &servo, int angle) {
  servo.targetAngle = constrain(angle, servo.minAngle, servo.maxAngle);
  Serial.print("Setting servo on channel ");
  Serial.print(servo.channel);
  Serial.print(" to ");
  Serial.print(servo.targetAngle);
  Serial.println(" degrees.");
}

void updateServos(ServoConfig servos[], uint8_t count) {
  unsigned long now = millis();
  for (uint8_t i = 0; i < count; i++) {
    ServoConfig &s = servos[i];
    if (now - s.lastUpdate >= s.updateInterval) {
      if (s.currentAngle != s.targetAngle) {
        if (s.currentAngle < s.targetAngle) {
          s.currentAngle += s.stepAngle;
          if (s.currentAngle > s.targetAngle)
            s.currentAngle = s.targetAngle;
        } else if (s.currentAngle > s.targetAngle) {
          s.currentAngle -= s.stepAngle;
          if (s.currentAngle < s.targetAngle)
            s.currentAngle = s.targetAngle;
        }

        Serial.print("Moving servo ");
        Serial.print(s.channel);
        Serial.print(" to ");
        Serial.print(s.currentAngle);
        Serial.println(" degrees");
      }

      pwm.setPWM(s.channel, 0, angleToPWM(s.currentAngle));
      s.lastUpdate = now;
    }
  }
}

void zeroAllMotors(ServoConfig servos[]) {
  Serial.println("Zeroing all motors...");
  moveServoToAngle(servos[0], 90);   // Base
  moveServoToAngle(servos[1], 90);   // Shoulder
  moveServoToAngle(servos[2], 180);  // Elbow
  moveServoToAngle(servos[3], 0);    // Wrist
  moveServoToAngle(servos[4], 0);    // Gripper
}