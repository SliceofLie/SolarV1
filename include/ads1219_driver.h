#ifndef ADS1219_DRIVER_H
#define ADS1219_DRIVER_H

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include "ADS1219.h"

// I2C address
#define ADS1219_ADDR 0x40

// I2C speed
#define I2C_SPEED_HZ 470000  // 470kHz to achieve ~400kHz on ESP8266

// ADS1219 configuration
#define ADS1219_CALIBRATION_SAMPLES 20
#define ADS1219_RECALIBRATION_INTERVAL_MS 600000UL  // 10 minutes
#define ADS1219_SAMPLES_PER_CHANNEL 10  // Number of samples to average per channel
// Note: ADS1219_DEBUG_CHANNEL_SWITCH is now defined in debug_config.h

// Channel rotation order: Current -> Solar+ -> Solar- -> repeat
#define ADS1219_NUM_CHANNELS 3

// ============================================================================
// Gain Switching Constants
// ============================================================================

// Gain switching thresholds for ADS1219 current channel
// Current channel uses INA241: ±100mV shunt × 20V/V gain = ±2.0V at ADC
// At 5mΩ shunt: ±100mV = ±20A, so ±2.0V represents ±10A at INA241 output
// With 5.0V external reference:
//   GAIN=0 (1x): ±5.0V range = ±12.5A effective max
//   GAIN=1 (4x): ±1.25V range = ±3.125A effective max

#define ADS1219_GAIN0_MAX_CURRENT 12.5    // ±12.5A @ GAIN=0 (1x)
#define ADS1219_GAIN1_MAX_CURRENT 3.125   // ±3.125A @ GAIN=1 (4x)

// Threshold percentages (moderate switching aggressiveness)
#define ADS1219_THRESHOLD_TO_HIGH_PERCENT 50    // Switch to high gain below 50%
#define ADS1219_THRESHOLD_TO_LOW_PERCENT 80     // Switch to low gain above 80%
#define ADS1219_THRESHOLD_EMERGENCY_PERCENT 90  // Emergency switch above 90%

// Debounce requirements (samples)
#define ADS1219_LOW_GAIN_SAMPLES_REQUIRED 10    // Samples before switching to high gain
#define ADS1219_HIGH_GAIN_SAMPLES_REQUIRED 3    // Samples before switching to low gain
#define ADS1219_EMERGENCY_SAMPLES_REQUIRED 1    // Samples for emergency switch

// Gain state structure (one per channel: current, solar+, solar-)
typedef struct {
  // Current state
  uint8_t currentGain;               // 0=1x gain, 1=4x gain
  uint8_t currentVref;               // 0=internal 2.048V, 1=external 5V

  // Per-configuration offset calibration (two modes for current channel)
  int32_t offsetHighGain;            // Offset for VREF=0 (internal) + GAIN=1 (4x)
  int32_t offsetLowGain;             // Offset for VREF=1 (external) + GAIN=0 (1x)

  // Switching thresholds (measurement-specific units)
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
  bool autoSwitchingEnabled;         // Enable/disable feature (per-channel)
  bool performOffsetCalibration;     // Perform offset cal after gain change

  // Statistics (for diagnostics)
  uint32_t switchToHighCount;        // Total switches to high gain
  uint32_t switchToLowCount;         // Total switches to low gain
  uint32_t emergencySwitchCount;     // Total emergency switches
  uint32_t lastSwitchMillis;         // Timestamp of last switch
} ADS1219_GAIN_STATE_t;

// ============================================================================
// Driver Layer Data Structures
// ============================================================================

// Channel identifier (index into channel sequence)
typedef uint8_t ADS1219_CHANNEL_ID_t;

// Channel measurement data (returned via callback)
typedef struct {
  ADS1219_CHANNEL_ID_t channelId;  // Channel index (0-2)
  ADS1219_MUX_t muxConfig;         // MUX setting used for this measurement
  int32_t rawValue;                // Raw 24-bit ADC value (before calibration)
  int32_t calibratedValue;         // ADC value after offset calibration
  double voltage;                  // Calibrated voltage (accounts for VREF/GAIN)
  uint8_t sampleCount;             // Number of samples averaged
} ADS1219_CHANNEL_DATA_t;

// Callback function type invoked when channel measurement completes
typedef void (*ADS1219DataReadyCallback)(const ADS1219_CHANNEL_DATA_t *channelData);

// Driver configuration structure
typedef struct {
  ADS1219_MUX_t channelSequence[ADS1219_NUM_CHANNELS];  // Channel rotation order
  uint8_t samplesPerChannel;                            // Samples to average per channel
  uint8_t calibrationSamples;                           // Samples for ADC calibration
  uint32_t recalibrationIntervalMs;                     // Periodic recalibration interval
  ADS1219DataReadyCallback dataReadyCallback;           // Callback function pointer
} ADS1219_DRIVER_CONFIG_t;


// Global I2C device object
extern Adafruit_I2CDevice ads1219Device;

// Calibration state
extern bool ads1219Calibrated;

// Global gain state tracking (one per channel)
extern ADS1219_GAIN_STATE_t ads1219GainState[ADS1219_NUM_CHANNELS];

// Shared I2C buffers (declared in main.cpp)
extern uint8_t ads1219WriteBuffer[8];
extern uint8_t ads1219ReadBuffer[8];

// Global error counter (declared in main.cpp)
extern uint32_t i2cErrorCount;

// Initialize ADS1219 ADC with configuration (call after I2C init)
bool ads1219Init(const ADS1219_DRIVER_CONFIG_t *config);

// Process data ready event (call from main loop when ads1219DataReady is true)
void ads1219ProcessDataReady();

// Check if recalibration is needed and trigger if necessary (called internally)
void ads1219CheckRecalibration();

// Internal helper functions (not typically called directly)
void ads1219ReadData();
void ads1219StartCalibration();
void ads1219ProcessCalibrationSample();
void ads1219FinishCalibration();
void ads1219ProcessMeasurement();
double getADS1219Factor(const ADS1219_CONFIG_t *config);

// ============================================================================
// Gain Switching Functions
// ============================================================================

// Initialize gain state structure for a specific channel
void ads1219InitGainState(ADS1219_GAIN_STATE_t *gainState,
                          uint8_t channel,
                          uint8_t initialGain);

// Switch to a new GAIN (0=1x, 1=4x) for a specific channel
// Returns true on success, false on error
bool ads1219SwitchGain(Adafruit_I2CDevice *device,
                       ADS1219_CONFIG_t *config,
                       ADS1219_GAIN_STATE_t *gainState,
                       uint8_t channel,
                       uint8_t newGain);

// Perform offset calibration at current gain setting
// Returns true on success, false on error
bool ads1219PerformOffsetCalibration(Adafruit_I2CDevice *device,
                                     ADS1219_CONFIG_t *config,
                                     ADS1219_GAIN_STATE_t *gainState);

// Evaluate if gain switching is needed for a specific channel
// Called after each measurement is processed
void ads1219EvaluateGainSwitch(uint8_t channel,
                               float measurement,
                               ADS1219_GAIN_STATE_t *gainState,
                               Adafruit_I2CDevice *device,
                               ADS1219_CONFIG_t *config);

#endif
