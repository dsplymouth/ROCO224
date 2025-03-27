#include <esp_now.h>
#include <WiFi.h>
#include "functions.h"

// Struct for received servo data
typedef struct {
    uint8_t servo;
    int angle;
} ServoCommand;

ServoCommand receivedData;  // Stores received data

// Callback function for incoming ESP-NOW messages
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received data for Servo ");
    Serial.print(receivedData.servo);
    Serial.print(": Angle ");
    Serial.println(receivedData.angle);

    if (receivedData.servo < NUM_SERVOS) {
        moveServoSmooth(receivedData.servo, homePositions[receivedData.servo], receivedData.angle, 10);
    } else {
        Serial.println("Invalid servo index received!");
    }
}

// Initializes ESP-NOW communication
void initializeESPNow() {
    WiFi.mode(WIFI_STA);  // ✅ Required for ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("ESP-NOW Receiver Initialized.");
}