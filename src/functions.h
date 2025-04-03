#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

extern Adafruit_PWMServoDriver pwm;

struct ServoConfig {
  uint8_t channel;
  int minAngle;
  int maxAngle;
  int currentAngle;
  int targetAngle;
  int stepAngle;
  unsigned long lastUpdate;
  uint16_t updateInterval;
};

uint16_t angleToPWM(int angle);
void moveServoToAngle(ServoConfig &servo, int angle);
void updateServos(ServoConfig servos[], uint8_t count);
void zeroAllMotors(ServoConfig servos[]);


//Funcitons for movement
void MoveForward(ServoConfig servos[]);
void MoveBackward(ServoConfig servos[]);
void RotateWrist(ServoConfig servos[]);
void ToggleGripper(ServoConfig servos[]);
void RotateWristReverse(ServoConfig servos[]);

#endif