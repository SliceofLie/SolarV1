#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "ESPAsyncWebServer.h"

// EEPROM configuration
// NOTE: Calibration uses bytes 0-55, WiFi config starts at byte 64
#define EEPROM_SIZE 512
#define CAL_MAGIC 0xCAFEBABE
#define CAL_EEPROM_ADDR 0  // Calibration struct occupies bytes 0-55 (~56 bytes)

// Calibration structure
typedef struct {
  uint32_t magic;               // Validity check

  // Solar voltage calibrations
  float solar_minus_offset;     // Volts
  float solar_minus_scale;      // multiplier
  float solar_plus_offset;      // Volts
  float solar_plus_scale;       // multiplier

  // Battery calibrations
  float batt_voltage_scale;     // multiplier
  float batt_current_offset;    // Amps
  float batt_current_scale;     // multiplier

  // Load calibrations
  float load_voltage_scale;     // multiplier
  float load_current_offset;    // Amps
  float load_current_scale;     // multiplier

  // Solar current calibrations
  float solar_current_offset;   // Amps
  float solar_current_scale;    // multiplier

  uint32_t checksum;            // Data integrity
} CALIBRATION_DATA_t;

// Global calibration instance
extern CALIBRATION_DATA_t calibration;

// Core calibration functions
void calibrationInit(size_t eepromSize);
void calibrationLoadDefaults();
bool calibrationLoad();
void calibrationSave();
uint32_t calibrationCalculateChecksum();

// Web handler functions
void handleGetCalData(AsyncWebServerRequest *request);
void handleSaveCalibration(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);
void handleResetCalibration(AsyncWebServerRequest *request);

#endif
