#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>
#include "ESPAsyncWebServer.h"
#include "calibration.h"
#include "ina228_driver.h"
#include "ads1219_driver.h"

// Global web server instance
extern AsyncWebServer server;

// Global error tracking
extern uint32_t i2cErrorCount;

// Initialize and start web server
void webServerInit();

// Get current measurements as JSON string
String getJsonData();

// Get diagnostics data as JSON string
String getDiagnosticsJson();

// Get human-readable reset reason string
String getResetReasonString(uint32 reason);

#endif
