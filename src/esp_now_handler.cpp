#include <esp_now.h>
#include <WiFi.h>
#include "functions.h"
#include "esp-now.h"

// Global variables
ServoCommand receivedData;  // Stores received data
extern ServoConfig servos[];


// Callback function for incoming ESP-NOW messages
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received number: ");
    Serial.println(receivedData.number);

    switch (receivedData.number) {
      case 1:
        MoveForward(servos);
        break;
      case 2:
        MoveBackward(servos);
        break;
      case 3:
        RotateWrist(servos);
        break;
      case 4:
        RotateWristReverse(servos);
        break;
      case 5:
        ToggleGripper(servos);
        break;
      default:
        Serial.println("Unknown command.");
        break;
    }
}

// Initializes ESP-NOW communication
void initializeESPNow() {
    WiFi.mode(WIFI_STA);  // Set ESP32 to station mode
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);  // Register callback function
    Serial.println("ESP-NOW Receiver Initialized.");
}