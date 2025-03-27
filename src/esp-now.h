#ifndef ESP_NOW_H
#define ESP_NOW_H

#include <esp_now.h>
#include <WiFi.h>

// Struct for received servo data
typedef struct {
    uint8_t servo;
    int angle;
} ServoCommand;

// Function declarations
void initializeESPNow();
void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

#endif
