#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <esp_now.h>
#include "functions.h"

void setup() {
    delay(1000);
    Serial.begin(9600);
    Serial.println("MG996R Servo Control - PCA9685 with ESP-NOW Receiver");

    WiFi.mode(WIFI_STA);  // Set ESP32 to station mode
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());

    // Setup ESP-NOW
    initializeServos();  // Setup PCA9685 and servos
    zeroServos();        // Move servos to home position
}

void loop() {
    // Move servos in a sequence for demonstration
    for (uint8_t servo = 0; servo < NUM_SERVOS; servo++) {
        Serial.print("Moving Servo: "); 
        Serial.println(servo);

        // Move the current servo from 0 to 180 degrees smoothly
        moveServoSmooth(servo, 0, 180, 40);
        delay(500);  // Wait for 500ms after moving to 180 degrees

        // Move the current servo back from 180 to 0 degrees smoothly
        moveServoSmooth(servo, 180, 0, 40);
        delay(500);  // Wait for 500ms after moving to 0 degrees
    }

    // Optionally, you can add other servo control sequences here
    // E.g., random movements, predefined patterns, or other behavior.
}