#ifndef INA228_DRIVER_H
#define INA228_DRIVER_H

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include "INA228.h"
#include "calibration.h"

// I2C addresses
#define INA228_BATT_ADDR 0x44
#define INA228_LOAD_ADDR 0x45

// I2C speed
#define I2C_SPEED_HZ 470000  // 470kHz to achieve ~400kHz on ESP8266

// Shunt resistor values
#define R_BATT_SHUNT 0.005
#define R_LOAD_SHUNT 0.005
#define R_SHUNT_MICROVOLTS 5000000.0  // 5mΩ shunt = 5,000,000 nV per amp

// INA228 configuration constants
#define INA228_VSHUNT_CALIB -410
#define INA228_CURRENT_CALIB -0.025625
#define INA228_SHUNT_CAL_VALUE 4096

// INA228 configuration structure
struct INA228Config {
  uint8_t adcRange;
  INA228_MODE_t mode;
  INA228_CONVTIME_t busConvTime;
  INA228_CONVTIME_t shuntConvTime;
  INA228_CONVTIME_t tempConvTime;
  INA228_AVGCOUNT_t avgCount;
  uint16_t shuntCal;
  bool enableConversionAlert;
};

// ============================================================================
// Gain Switching Structures and Constants
// ============================================================================

// Gain switching thresholds for INA228 (based on 5mΩ shunt)
#define INA228_RANGE0_MAX_CURRENT 32.768    // ±32.768A @ ADCRANGE=0
#define INA228_RANGE1_MAX_CURRENT 8.192     // ±8.192A @ ADCRANGE=1

// Threshold percentages (moderate switching aggressiveness)
#define INA228_THRESHOLD_TO_HIGH_PERCENT 50    // Switch to high gain below 50%
#define INA228_THRESHOLD_TO_LOW_PERCENT 80     // Switch to low gain above 80%
#define INA228_THRESHOLD_EMERGENCY_PERCENT 90  // Emergency switch above 90%

// Debounce requirements (samples)
#define INA228_LOW_GAIN_SAMPLES_REQUIRED 10    // Samples before switching to high gain
#define INA228_HIGH_GAIN_SAMPLES_REQUIRED 3    // Samples before switching to low gain
#define INA228_EMERGENCY_SAMPLES_REQUIRED 1    // Samples for emergency switch

// Gain state structure (one per device: battery, load)
typedef struct {
  // Current state
  uint8_t currentRange;              // 0=wide (±163.84mV), 1=narrow (±40.96mV)
  uint16_t currentShuntCal;          // Current SHUNT_CAL value

  // Switching thresholds (in Amps)
  float threshold50Percent;          // Switch to high gain below this
  float threshold80Percent;          // Switch to low gain above this
  float threshold90Percent;          // Emergency switch above this

  // Debounce counters
  uint8_t consecutiveLowSamples;     // Count of samples < 50%
  uint8_t consecutiveHighSamples;    // Count of samples > 80%
  uint8_t emergencySamples;          // Count of samples > 90%

  // Configuration
  uint8_t lowGainSamplesRequired;    // Default: 10
  uint8_t highGainSamplesRequired;   // Default: 3
  bool autoSwitchingEnabled;         // Enable/disable feature

  // Statistics (for diagnostics)
  uint32_t switchToHighCount;        // Total switches to high gain
  uint32_t switchToLowCount;         // Total switches to low gain
  uint32_t emergencySwitchCount;     // Total emergency switches
  uint32_t lastSwitchMillis;         // Timestamp of last switch
} INA228_GAIN_STATE_t;

// Global I2C device objects
extern Adafruit_I2CDevice ina228BattDevice;
extern Adafruit_I2CDevice ina228LoadDevice;

// Global register and measurement data
extern INA228_REGISTERS_t ina228BattRegisters;
extern INA228_REGISTERS_t ina228LoadRegisters;
extern INA228_MEASUREMENTS_t ina228BattMeasurements;
extern INA228_MEASUREMENTS_t ina228LoadMeasurements;

// Global gain state tracking
extern INA228_GAIN_STATE_t ina228BattGainState;
extern INA228_GAIN_STATE_t ina228LoadGainState;

// Shared I2C buffers (declared in main.cpp)
extern uint8_t ina228WriteBuffer[8];
extern uint8_t ina228ReadBuffer[8];

// Initialize INA228 devices (call after I2C init)
bool ina228Init();

// Setup individual INA228 device
void setupINA228(Adafruit_I2CDevice *device, INA228_REGISTERS_t *regs, const INA228Config *config);

// Apply calibration and validate measurements
void applyINA228Calibration(INA228_MEASUREMENTS_t* measurements,
                             float voltageScale,
                             float currentOffset,
                             float currentScale);

// Print measurements to serial (debug)
void printINA228Measurements(const char* channel,
                              INA228_REGISTERS_t *regs,
                              INA228_MEASUREMENTS_t *meas);

// Process battery alert (call from main loop when ina228BattAlert is true)
void ina228ProcessBatteryAlert();

// Process load alert (call from main loop when ina228LoadAlert is true)
void ina228ProcessLoadAlert();

// ============================================================================
// Gain Switching Functions
// ============================================================================

// Initialize gain state structure with default values
void ina228InitGainState(INA228_GAIN_STATE_t *gainState, uint8_t initialRange);

// Switch to a new ADCRANGE (0=wide, 1=narrow)
// Returns true on success, false on error
bool ina228SwitchRange(Adafruit_I2CDevice *device,
                       INA228_REGISTERS_t *regs,
                       INA228_GAIN_STATE_t *gainState,
                       uint8_t newRange);

// Evaluate if gain switching is needed based on current measurement
// Called after each measurement is processed
void ina228EvaluateGainSwitch(INA228_MEASUREMENTS_t *measurements,
                              INA228_GAIN_STATE_t *gainState,
                              Adafruit_I2CDevice *device,
                              INA228_REGISTERS_t *regs);

#endif
