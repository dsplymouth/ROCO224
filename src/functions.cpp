#include "functions.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 instance
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// MG996R specific values
#define SERVOMIN  200   // 500µs
#define SERVOMAX  600   // 2500µs
#define SERVO_FREQ 40   // 50Hz update rate

// Define zero positions for each servo
int homePositions[NUM_SERVOS] = {60, 40, 60};  // ✅ Ensure semicolon
int limits[NUM_SERVOS][2] = {{0, 180}, {40, 150}, {0, 120}};
  // Limits for each servo

void initializeServos() {
    pwm.begin();
    pwm.setOscillatorFrequency(29000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
}
void moveServoInstant(uint8_t servo, int angle) {
    // Ensure angle is within the valid range
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    int pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);  // Map angle to pulse length
    pwm.setPWM(servo, 0, pulselen);  // Move the servo to the mapped pulse length
}

void moveServoSmooth(uint8_t servo, int startAngle, int endAngle, int stepDelay) {
    // Ensure start and end angles are within limits
    if (startAngle < 0) startAngle = 0;
    if (startAngle > 180) startAngle = 180;
    if (endAngle < 0) endAngle = 0;
    if (endAngle > 180) endAngle = 180;

    // Move from start angle to end angle smoothly
    if (startAngle > endAngle) {
        for (int angle = startAngle; angle >= endAngle; angle--) {
            moveServoInstant(servo, angle);
            delay(stepDelay);
        }
    } else {
        for (int angle = startAngle; angle <= endAngle; angle++) {
            moveServoInstant(servo, angle);
            delay(stepDelay);
        }
    }
}


// Move all servos to their zero (home) positions
void zeroServos() {
    Serial.println("Zeroing all servos...");
    for (uint8_t servo = 0; servo < NUM_SERVOS; servo++) {
        moveServoSmooth(servo, 0, homePositions[servo], 24);  // Move smoothly to home position
        delay(100);  // Small delay between servos
    }
    Serial.println("All servos zeroed.");
}
