#include "debug_config.h"
#include "calibration.h"

// Global calibration instance
CALIBRATION_DATA_t calibration;

void calibrationInit(size_t eepromSize) {
  EEPROM.begin(eepromSize);
}

void calibrationLoadDefaults() {
  calibration.magic = CAL_MAGIC;
  calibration.solar_minus_offset = 0.0;
  calibration.solar_minus_scale = 1.0;
  calibration.solar_plus_offset = 0.0;
  calibration.solar_plus_scale = 1.0;
  calibration.batt_voltage_scale = 1.0;
  calibration.batt_current_offset = 0.0;
  calibration.batt_current_scale = 1.0;
  calibration.load_voltage_scale = 1.0;
  calibration.load_current_offset = 0.0;
  calibration.load_current_scale = 1.0;
  calibration.solar_current_offset = 0.0;
  calibration.solar_current_scale = 1.0;
  calibration.checksum = calibrationCalculateChecksum();
}

uint32_t calibrationCalculateChecksum() {
  // Simple additive checksum
  uint32_t sum = 0;
  uint8_t* ptr = (uint8_t*)&calibration;
  // Calculate checksum over entire struct except checksum field itself
  for (size_t i = 0; i < sizeof(CALIBRATION_DATA_t) - sizeof(uint32_t); i++) {
    sum += ptr[i];
  }
  return sum;
}

bool calibrationLoad() {
  EEPROM.get(CAL_EEPROM_ADDR, calibration);

  // Check magic number
  if (calibration.magic != CAL_MAGIC) {
    #if DEBUG_CALIBRATION
    Serial.println("Calibration: Invalid magic number");
    #endif
    return false;
  }

  // Verify checksum
  uint32_t calculatedChecksum = calibrationCalculateChecksum();
  if (calculatedChecksum != calibration.checksum) {
    #if DEBUG_CALIBRATION
    Serial.println("Calibration: Checksum mismatch");
    #endif
    return false;
  }

  return true;
}

void calibrationSave() {
  calibration.magic = CAL_MAGIC;
  calibration.checksum = calibrationCalculateChecksum();
  EEPROM.put(CAL_EEPROM_ADDR, calibration);
  EEPROM.commit();
  #if DEBUG_CALIBRATION
  Serial.println("Calibration saved to EEPROM");
  #endif
}

void handleGetCalData(AsyncWebServerRequest *request) {
  // Build JSON with current calibration values
  JsonDocument doc;

  doc["solar_minus_offset"] = calibration.solar_minus_offset;
  doc["solar_minus_scale"] = calibration.solar_minus_scale;
  doc["solar_plus_offset"] = calibration.solar_plus_offset;
  doc["solar_plus_scale"] = calibration.solar_plus_scale;
  doc["batt_voltage_scale"] = calibration.batt_voltage_scale;
  doc["batt_current_offset"] = calibration.batt_current_offset;
  doc["batt_current_scale"] = calibration.batt_current_scale;
  doc["load_voltage_scale"] = calibration.load_voltage_scale;
  doc["load_current_offset"] = calibration.load_current_offset;
  doc["load_current_scale"] = calibration.load_current_scale;
  doc["solar_current_offset"] = calibration.solar_current_offset;
  doc["solar_current_scale"] = calibration.solar_current_scale;

  String response;
  serializeJson(doc, response);
  request->send(200, "application/json", response);
}

void handleSaveCalibration(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  // Use fixed buffer to avoid dynamic String allocation and heap fragmentation
  static char bodyBuffer[512];
  static size_t bodyIndex = 0;

  if (index == 0) {
    bodyIndex = 0;
  }

  // Copy data to buffer with bounds checking
  for (size_t i = 0; i < len && bodyIndex < sizeof(bodyBuffer) - 1; i++) {
    bodyBuffer[bodyIndex++] = (char)data[i];
  }

  if (index + len == total) {
    bodyBuffer[bodyIndex] = '\0';  // Null terminate

    // All data received, parse JSON from buffer
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, bodyBuffer);

    if (error) {
      request->send(400, "text/plain", "Invalid JSON");
      return;
    }

    // Update calibration values atomically (disable interrupts to prevent
    // measurement calculations from using partially-updated values)
    noInterrupts();
    calibration.solar_minus_offset = doc["solar_minus_offset"] | 0.0;
    calibration.solar_minus_scale = doc["solar_minus_scale"] | 1.0;
    calibration.solar_plus_offset = doc["solar_plus_offset"] | 0.0;
    calibration.solar_plus_scale = doc["solar_plus_scale"] | 1.0;
    calibration.batt_voltage_scale = doc["batt_voltage_scale"] | 1.0;
    calibration.batt_current_offset = doc["batt_current_offset"] | 0.0;
    calibration.batt_current_scale = doc["batt_current_scale"] | 1.0;
    calibration.load_voltage_scale = doc["load_voltage_scale"] | 1.0;
    calibration.load_current_offset = doc["load_current_offset"] | 0.0;
    calibration.load_current_scale = doc["load_current_scale"] | 1.0;
    calibration.solar_current_offset = doc["solar_current_offset"] | 0.0;
    calibration.solar_current_scale = doc["solar_current_scale"] | 1.0;
    interrupts();

    // Save to EEPROM
    calibrationSave();

    request->send(200, "text/plain", "Calibration saved successfully");
  }
}

void handleResetCalibration(AsyncWebServerRequest *request) {
  calibrationLoadDefaults();
  calibrationSave();
  request->send(200, "text/plain", "Calibration reset to defaults");
}
