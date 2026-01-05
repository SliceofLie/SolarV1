#include "ADS1219.h"

// ============================================================================
// ADS1219 Hardware Layer - Low-level I2C Interface
// ============================================================================
// This file provides pure hardware interface functions for the ADS1219 ADC.
// No application-specific knowledge (e.g., INA241, voltage dividers, solar
// monitoring) should exist in this layer.
//
// Responsibilities:
// - I2C communication primitives
// - Command execution (RESET, START/SYNC, RDATA, RREG, WREG)
// - 24-bit ADC value reading with sign extension
// - Configuration register manipulation
// - Comprehensive error checking on all I2C operations
// ============================================================================

bool ADS1219_HardwareInit(Adafruit_I2CDevice *device,
                          uint8_t *writeBuffer,
                          uint8_t *readBuffer) {
  if (device == NULL) {
    return false;
  }

  // Set I2C speed (470kHz target achieves ~400kHz on ESP8266)
  device->setSpeed(470000);

  // Check device presence on I2C bus
  if (!device->begin()) {
    return false;
  }

  // Send RESET command to ensure known state
  if (!ADS1219_Reset(device, writeBuffer)) {
    return false;
  }

  // Small delay after reset for device to stabilize
  delay(10);

  return true;
}

bool ADS1219_Reset(Adafruit_I2CDevice *device, uint8_t *writeBuffer) {
  if (device == NULL || writeBuffer == NULL) {
    return false;
  }

  writeBuffer[0] = ADS1219_CMD_RESET;
  if (!device->write(writeBuffer, 1)) {
    return false;
  }

  return true;
}

bool ADS1219_WriteConfig(Adafruit_I2CDevice *device,
                         uint8_t *writeBuffer,
                         const ADS1219_CONFIG_t *config) {
  if (device == NULL || writeBuffer == NULL || config == NULL) {
    return false;
  }

  writeBuffer[0] = ADS1219_CMD_WREG;
  writeBuffer[1] = config->u8;

  if (!device->write(writeBuffer, 2)) {
    return false;
  }

  return true;
}

bool ADS1219_ReadConfig(Adafruit_I2CDevice *device,
                        uint8_t *writeBuffer,
                        uint8_t *readBuffer,
                        ADS1219_CONFIG_t *config) {
  if (device == NULL || writeBuffer == NULL || readBuffer == NULL || config == NULL) {
    return false;
  }

  writeBuffer[0] = ADS1219_CMD_RREG;

  if (!device->write_then_read(writeBuffer, 1, readBuffer, 1)) {
    return false;
  }

  config->u8 = readBuffer[0];
  return true;
}

bool ADS1219_ReadData(Adafruit_I2CDevice *device,
                      uint8_t *writeBuffer,
                      uint8_t *readBuffer,
                      int32_t *value) {
  if (device == NULL || writeBuffer == NULL || readBuffer == NULL || value == NULL) {
    return false;
  }

  writeBuffer[0] = ADS1219_CMD_RDATA;

  if (!device->write_then_read(writeBuffer, 1, readBuffer, 3)) {
    return false;
  }

  // Unpack 24-bit value with sign extension to 32-bit signed integer
  // Shift bytes into upper 24 bits, then arithmetic right shift for sign extension
  *value = (uint32_t)readBuffer[0] << 24 |
           (uint32_t)readBuffer[1] << 16 |
           (uint32_t)readBuffer[2] << 8;
  *value = *value >> 8;  // Arithmetic shift preserves sign bit

  return true;
}

bool ADS1219_StartConversion(Adafruit_I2CDevice *device,
                             uint8_t *writeBuffer) {
  if (device == NULL || writeBuffer == NULL) {
    return false;
  }

  writeBuffer[0] = ADS1219_CMD_STARTSYNC;

  if (!device->write(writeBuffer, 1)) {
    return false;
  }

  return true;
}

bool ADS1219_PowerDown(Adafruit_I2CDevice *device,
                       uint8_t *writeBuffer) {
  if (device == NULL || writeBuffer == NULL) {
    return false;
  }

  writeBuffer[0] = ADS1219_CMD_POWERDOWN;

  if (!device->write(writeBuffer, 1)) {
    return false;
  }

  return true;
}
