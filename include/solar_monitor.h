#ifndef SOLAR_MONITOR_H
#define SOLAR_MONITOR_H

#include <Arduino.h>
#include "ads1219_driver.h"
#include "INA241.h"
#include "calibration.h"

// ============================================================================
// Solar Monitoring Application Layer
// ============================================================================
// This layer handles solar panel monitoring application logic, including:
// - INA241 current sense amplifier calculations
// - Voltage divider scaling for Solar+ and Solar-
// - Power calculations
// - User calibration application
// - Global measurement variable management
//
// This layer sits above the ADS1219 driver and receives raw ADC voltages
// via callback, then transforms them into meaningful solar measurements.
// ============================================================================

// Channel identifier enum (maps to ADS1219 channel sequence indices)
typedef enum {
  SOLAR_CHANNEL_CURRENT = 0,    // INA241 current sense output
  SOLAR_CHANNEL_SOLAR_PLUS = 1, // Solar+ rail voltage
  SOLAR_CHANNEL_SOLAR_MINUS = 2 // Solar- rail voltage
} SOLAR_CHANNEL_ID_t;

// Solar hardware topology configuration
typedef struct {
  // INA241 current sense amplifier configuration
  double shuntResistorOhms;      // Shunt resistor value (0.005Ω)
  double ina241GainVV;           // INA241 amplifier gain (20V/V)

  // Solar+ voltage divider (R1=120k, R2=50k)
  // Vout = Vin × (50k/170k) = Vin × 0.294
  // Therefore: Vin = Vout × 3.4
  double solarPlusScale;         // Voltage divider inverse scale (3.4)

  // Solar- level-shifting triplet divider
  // Maps: -10V → 0V, 0V → 2.5V, +10V → 5V
  // Formula: Vout = (Vin + 10) × 0.25
  // Inverse: Vin = (Vout × 4) - 10
  double solarMinusScale;        // Scale factor (4.0)
  double solarMinusOffset;       // Offset voltage (10.0V)

  // Channel mapping (which ADS1219 channel corresponds to which measurement)
  SOLAR_CHANNEL_ID_t currentChannel;
  SOLAR_CHANNEL_ID_t solarPlusChannel;
  SOLAR_CHANNEL_ID_t solarMinusChannel;
} SOLAR_MONITOR_CONFIG_t;

// Solar panel measurements (application state)
typedef struct {
  double solarPlusVoltage;   // Solar+ rail voltage (V)
  double solarMinusVoltage;  // Solar- rail voltage (V)
  double solarVoltage;       // Panel voltage: Solar+ minus Solar- (V)
  double solarCurrent;       // Panel current (A)
  double solarPower;         // Panel power: V × I (W)
  uint32_t lastUpdateMillis; // Timestamp of last complete measurement
  bool dataValid;            // True after all three channels measured at least once
} SOLAR_MEASUREMENTS_t;

// ============================================================================
// Application API
// ============================================================================

// Initialize solar monitoring system
// Configures ADS1219 driver with solar-specific channel sequence and callback
// Returns: true on success, false on initialization failure
bool solarMonitorInit();

// Get current solar measurements (thread-safe copy with interrupt protection)
// Parameters:
//   measurements - pointer to structure to receive measurement data
void solarMonitorGetMeasurements(SOLAR_MEASUREMENTS_t *measurements);

// Get pointer to live measurements (for direct access with external interrupt protection)
// Returns: const pointer to internal measurements structure
// WARNING: Caller must protect access with noInterrupts()/interrupts() if needed
const SOLAR_MEASUREMENTS_t* solarMonitorGetMeasurementsPtr();

// Internal callback invoked by ADS1219 driver when channel data ready
// This function is called from the driver layer and should not be called directly
// Parameters:
//   channelData - pointer to channel data structure from driver
void solarMonitorDataReadyCallback(const ADS1219_CHANNEL_DATA_t *channelData);

// ============================================================================
// Global Variables (for backward compatibility with existing code)
// ============================================================================

extern double solarVoltage;       // Panel voltage (V)
extern double solarCurrent;       // Panel current (A)
extern double solarPower;         // Panel power (W)
extern double solarPlusVoltage;   // Solar+ rail voltage (V)
extern double solarMinusVoltage;  // Solar- rail voltage (V)

#endif // SOLAR_MONITOR_H
