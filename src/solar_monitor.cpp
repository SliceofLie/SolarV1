#include "debug_config.h"
#include "solar_monitor.h"

// ============================================================================
// Solar Monitoring Application Layer Implementation
// ============================================================================

// Solar hardware configuration (hardware topology constants)
static SOLAR_MONITOR_CONFIG_t solarConfig = {
  .shuntResistorOhms = 0.005,           // 5mΩ shunt resistor
  .ina241GainVV = INA241_GAIN,          // 20V/V from INA241.h
  .solarPlusScale = 3.4,                // Voltage divider: (120k+50k)/50k
  .solarMinusScale = 4.0,               // Level shifter scale
  .solarMinusOffset = 10.0,             // Level shifter offset
  .currentChannel = SOLAR_CHANNEL_CURRENT,
  .solarPlusChannel = SOLAR_CHANNEL_SOLAR_PLUS,
  .solarMinusChannel = SOLAR_CHANNEL_SOLAR_MINUS
};

// Solar measurement state
static SOLAR_MEASUREMENTS_t solarMeasurements = {
  .solarPlusVoltage = 0.0,
  .solarMinusVoltage = 0.0,
  .solarVoltage = 0.0,
  .solarCurrent = 0.0,
  .solarPower = 0.0,
  .lastUpdateMillis = 0,
  .dataValid = false
};

// Global variables (for backward compatibility with existing code)
double solarVoltage = 0.0;
double solarCurrent = 0.0;
double solarPower = 0.0;
double solarPlusVoltage = 0.0;
double solarMinusVoltage = 0.0;

// ============================================================================
// Application API Implementation
// ============================================================================

bool solarMonitorInit() {
  #if DEBUG_SYSTEM
  Serial.println("Solar Monitor Init");
  #endif

  // Configure ADS1219 driver for solar monitoring
  // Channel sequence: Current (P2_N3) -> Solar+ (P0_NG) -> Solar- (P1_NG)
  ADS1219_DRIVER_CONFIG_t driverConfig = {
    .channelSequence = {
      ADS1219_MUX_P2_N3,  // Channel 0: Current measurement (INA241 output)
      ADS1219_MUX_P0_NG,  // Channel 1: Solar+ voltage
      ADS1219_MUX_P1_NG   // Channel 2: Solar- voltage
    },
    .samplesPerChannel = 10,                         // Average 10 samples per channel
    .calibrationSamples = 20,                        // 20 samples for ADC calibration
    .recalibrationIntervalMs = 600000,               // Recalibrate every 10 minutes
    .dataReadyCallback = solarMonitorDataReadyCallback
  };

  return ads1219Init(&driverConfig);
}

void solarMonitorGetMeasurements(SOLAR_MEASUREMENTS_t *measurements) {
  if (measurements == NULL) {
    return;
  }

  // Copy measurements with interrupt protection to prevent partial reads
  noInterrupts();
  *measurements = solarMeasurements;
  interrupts();
}

const SOLAR_MEASUREMENTS_t* solarMonitorGetMeasurementsPtr() {
  return &solarMeasurements;
}

void solarMonitorDataReadyCallback(const ADS1219_CHANNEL_DATA_t *channelData) {
  if (channelData == NULL) {
    return;
  }

  // Process based on which channel completed
  if (channelData->channelId == solarConfig.currentChannel) {
    // ========================================================================
    // CURRENT MEASUREMENT - INA241 Current Sense Amplifier
    // ========================================================================
    // The INA241 amplifies the shunt voltage by 20V/V
    // To get current: I = (Vout / Gain) / R_shunt

    double ina241OutputVoltage = channelData->voltage;
    double shuntVoltage = ina241OutputVoltage / solarConfig.ina241GainVV;
    double rawCurrent = shuntVoltage / solarConfig.shuntResistorOhms;

    // Apply user calibration (offset and scale from EEPROM)
    noInterrupts();
    float currentOffset = calibration.solar_current_offset;
    float currentScale = calibration.solar_current_scale;
    interrupts();

    solarMeasurements.solarCurrent = (rawCurrent + currentOffset) * currentScale;

    // Update power calculation (using previously calculated voltage)
    solarMeasurements.solarPower = solarMeasurements.solarVoltage * solarMeasurements.solarCurrent;

    // Update global variables
    solarCurrent = solarMeasurements.solarCurrent;
    solarPower = solarMeasurements.solarPower;

    #if DEBUG_ADS1219
    Serial.printf("Solar Current: ADC=%.6fV, Shunt=%.6fV, I=%.3fA, P=%.3fW\n",
                  ina241OutputVoltage, shuntVoltage, solarCurrent, solarPower);
    #endif

    // Evaluate if gain switching is needed for current channel
    extern ADS1219_GAIN_STATE_t ads1219GainState[];
    extern Adafruit_I2CDevice ads1219Device;
    extern ADS1219_CONFIG_t ads1219Config;
    ads1219EvaluateGainSwitch(channelData->channelId, solarMeasurements.solarCurrent,
                              &ads1219GainState[channelData->channelId],
                              &ads1219Device, &ads1219Config);

  } else if (channelData->channelId == solarConfig.solarPlusChannel) {
    // ========================================================================
    // SOLAR+ VOLTAGE MEASUREMENT
    // ========================================================================
    // Voltage divider: R1=120k, R2=50k
    // Vout = Vin × (50k/170k) = Vin × 0.294
    // Therefore: Vin = Vout × (170k/50k) = Vout × 3.4

    double adcVoltage = channelData->voltage;
    double rawSolarPlus = adcVoltage * solarConfig.solarPlusScale;

    // Apply user calibration
    noInterrupts();
    float plusOffset = calibration.solar_plus_offset;
    float plusScale = calibration.solar_plus_scale;
    interrupts();

    solarMeasurements.solarPlusVoltage = (rawSolarPlus + plusOffset) * plusScale;

    // Update global variable
    solarPlusVoltage = solarMeasurements.solarPlusVoltage;

    #if DEBUG_ADS1219
    Serial.printf("Solar+: ADC=%.6fV, Scaled=%.3fV\n",
                  adcVoltage, solarPlusVoltage);
    #endif

  } else if (channelData->channelId == solarConfig.solarMinusChannel) {
    // ========================================================================
    // SOLAR- VOLTAGE MEASUREMENT
    // ========================================================================
    // Triplet level-shifting divider maps:
    //   -10V → 0V, 0V → 2.5V, +10V → 5V
    // Formula: Vout = (Vin + 10) × 0.25
    // Inverse: Vin = (Vout × 4) - 10

    double adcVoltage = channelData->voltage;
    double rawSolarMinus = (adcVoltage * solarConfig.solarMinusScale) - solarConfig.solarMinusOffset;

    // Apply user calibration
    noInterrupts();
    float minusOffset = calibration.solar_minus_offset;
    float minusScale = calibration.solar_minus_scale;
    interrupts();

    solarMeasurements.solarMinusVoltage = (rawSolarMinus + minusOffset) * minusScale;

    // Calculate panel voltage (Solar+ minus Solar-)
    solarMeasurements.solarVoltage = solarMeasurements.solarPlusVoltage - solarMeasurements.solarMinusVoltage;

    // Recalculate power with new voltage
    solarMeasurements.solarPower = solarMeasurements.solarVoltage * solarMeasurements.solarCurrent;

    // Update global variables
    solarMinusVoltage = solarMeasurements.solarMinusVoltage;
    solarVoltage = solarMeasurements.solarVoltage;
    solarPower = solarMeasurements.solarPower;

    // Mark data as valid and update timestamp (all three channels have been read)
    solarMeasurements.dataValid = true;
    solarMeasurements.lastUpdateMillis = millis();

    #if DEBUG_ADS1219
    Serial.printf("Solar-: ADC=%.6fV, Scaled=%.3fV\n", adcVoltage, solarMinusVoltage);
    Serial.printf("Panel: V=%.3fV (%.3fV - %.3fV), I=%.3fA, P=%.3fW\n",
                  solarVoltage, solarPlusVoltage, solarMinusVoltage, solarCurrent, solarPower);
    #endif
  }
}
