#include <esp_now.h>
#include <WiFi.h>
#include "functions.h"
#include "esp-now.h"

// Global variables
ServoCommand receivedData;  // Stores received data
extern ServoConfig servos[];


// Callback function for incoming ESP-NOW messages
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // Copy incoming data into the `receivedData` structure
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Print the received number to confirm connection
    Serial.print("Received number: ");
    Serial.println(receivedData.number);

    // Optional: Use the received number for further processing
    // For example, move a specific servo based on the number:
    
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