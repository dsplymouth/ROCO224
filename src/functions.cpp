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
  moveServoToAngle(servos[1], 160);   // Shoulder //lowest 22
  moveServoToAngle(servos[2], 25);  // Elbow 20 Lowest Highest 163
  moveServoToAngle(servos[3], 180);    // Wrist
  moveServoToAngle(servos[4], 180); // Gripper
}

//Functions for movement
void MoveForward(ServoConfig servos[]) {
  Serial.println("Command: Move Forward");
  moveServoToAngle(servos[1], servos[1].targetAngle - 5); // Shoulder
  moveServoToAngle(servos[2], servos[2].targetAngle + 5); // Elbow
}

void MoveBackward(ServoConfig servos[]) {
  Serial.println("Command: Move Backward");
  moveServoToAngle(servos[1], servos[1].targetAngle + 5); // Shoulder
  moveServoToAngle(servos[2], servos[2].targetAngle - 5); // Elbow
}

void RotateWrist(ServoConfig servos[]) {
  Serial.println("Command: Rotate Wrist");
  moveServoToAngle(servos[3], servos[3].targetAngle + 180); // Wrist rotation
}

void ToggleGripper(ServoConfig servos[]) {
  Serial.println("Command: Toggle Gripper");
  static bool open = false;
  open = !open;
  if (open) {
    moveServoToAngle(servos[4], 180); // Open
  } else {
    moveServoToAngle(servos[4], 0);   // Close
  }
}

void RotateWristReverse(ServoConfig servos[]){
  Serial.println("Command: Rotate Wrist (Reverse)");
  moveServoToAngle(servos[3], servos[3].targetAngle - 180);
}
