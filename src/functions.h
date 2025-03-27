#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

// Define number of servos
#define NUM_SERVOS 3

// Function declarations
void initializeServos();
void moveServoInstant(uint8_t servo, int angle);
void moveServoSmooth(uint8_t servo, int startAngle, int endAngle, int stepDelay);
void zeroServos();

extern int homePositions[NUM_SERVOS];  // Declare home positions

#endif  // FUNCTIONS_H