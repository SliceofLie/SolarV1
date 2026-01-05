#ifndef ADS1219_H
#define ADS1219_H

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>

// ADS1219 Command Definitions
#define ADS1219_CMD_RESET     0x06  // 0000011x
#define ADS1219_CMD_STARTSYNC 0x08  // 0000100x
#define ADS1219_CMD_POWERDOWN 0x02  // 0000001x
#define ADS1219_CMD_RDATA     0x10  // 0001xxxx
#define ADS1219_CMD_RREG      0x20  // 00100rxx
#define ADS1219_CMD_WREG      0x40  // 010000xx

// ADS1219 Constants
#define INT24_MAX 8388607       // 2^24-1

// ADS1219 Enumerations
enum ADS1219_MUX_t {
  ADS1219_MUX_P0_N1,//AIN0 referenced to AIN1
  ADS1219_MUX_P2_N3,//AIN2 referenced to AIN3
  ADS1219_MUX_P1_N2,//AIN1 referenced to AIN2
  ADS1219_MUX_P0_NG,//AIN0 referenced to GND
  ADS1219_MUX_P1_NG,//AIN1 referenced to GND
  ADS1219_MUX_P2_NG,//AIN2 referenced to GND
  ADS1219_MUX_P3_NG,//AIN3 referenced to GND
  ADS1219_MUX_PN_CALIB
};

enum ADS1219_DR_t {
  ADS1219_DR_20SPS,
  ADS1219_DR_90SPS,
  ADS1219_DR_330SPS,
  ADS1219_DR_1000SPS
};

// ADS1219 Config Register Structure
typedef union {
  uint8_t u8;
  struct {
    unsigned VREF :1;
    unsigned CM   :1;
    unsigned DR   :2;
    unsigned GAIN :1;
    unsigned MUX  :3;
  };
} ADS1219_CONFIG_t;

// ADS1219 Conversion Factor Structure
typedef struct {
  double VREF_EXT_GAIN_1;
  double VREF_INT_GAIN_1;
  double VREF_EXT_GAIN_4;
  double VREF_INT_GAIN_4;
} ADS1219_FACTOR_t;

// ============================================================================
// Hardware Layer API - Low-level I2C Interface Functions
// ============================================================================

// Initialize I2C device and reset ADC
// Returns: true on success, false on failure
bool ADS1219_HardwareInit(Adafruit_I2CDevice *device,
                          uint8_t *writeBuffer,
                          uint8_t *readBuffer);

// Send RESET command
// Returns: true on success, false on I2C error
bool ADS1219_Reset(Adafruit_I2CDevice *device,
                   uint8_t *writeBuffer);

// Write configuration register
// Returns: true on success, false on I2C error
bool ADS1219_WriteConfig(Adafruit_I2CDevice *device,
                         uint8_t *writeBuffer,
                         const ADS1219_CONFIG_t *config);

// Read configuration register
// Returns: true on success, false on I2C error
bool ADS1219_ReadConfig(Adafruit_I2CDevice *device,
                        uint8_t *writeBuffer,
                        uint8_t *readBuffer,
                        ADS1219_CONFIG_t *config);

// Read 24-bit ADC conversion result with sign extension
// Returns: true on success, false on I2C error
bool ADS1219_ReadData(Adafruit_I2CDevice *device,
                      uint8_t *writeBuffer,
                      uint8_t *readBuffer,
                      int32_t *value);

// Send START/SYNC command to begin conversion
// Returns: true on success, false on I2C error
bool ADS1219_StartConversion(Adafruit_I2CDevice *device,
                             uint8_t *writeBuffer);

// Send POWERDOWN command
// Returns: true on success, false on I2C error
bool ADS1219_PowerDown(Adafruit_I2CDevice *device,
                       uint8_t *writeBuffer);

#endif // ADS1219_H
