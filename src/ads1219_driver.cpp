#include "debug_config.h"
#include "ads1219_driver.h"

// Global I2C device object
Adafruit_I2CDevice ads1219Device = Adafruit_I2CDevice(ADS1219_ADDR);

// ADS1219 Variables
ADS1219_CONFIG_t ads1219Config;
int32_t ads1219CalibrationValue = 0;
bool ads1219Calibrated = false;
uint8_t ads1219CalibrationCount = 0;
int32_t ads1219Value = 0;
static int32_t ads1219ValueArray[ADS1219_CALIBRATION_SAMPLES];

// Global gain state tracking (one per channel)
ADS1219_GAIN_STATE_t ads1219GainState[ADS1219_NUM_CHANNELS];

// ADS1219 Channel switching and averaging
ADS1219_MUX_t ads1219CurrentChannel = ADS1219_MUX_P2_N3;  // Start with current measurement
uint8_t ads1219ChannelSampleCount = 0;
int32_t ads1219ChannelAccumulator = 0;

// Channel rotation order (configured during init)
static ADS1219_MUX_t ads1219ChannelSequence[ADS1219_NUM_CHANNELS];
uint8_t ads1219CurrentChannelIndex = 0;  // Start at first channel in sequence

// Driver configuration
static uint8_t ads1219SamplesPerChannel = ADS1219_SAMPLES_PER_CHANNEL;
static uint8_t ads1219CalibrationSamplesConfig = ADS1219_CALIBRATION_SAMPLES;
static uint32_t ads1219RecalibrationInterval = ADS1219_RECALIBRATION_INTERVAL_MS;
static ADS1219DataReadyCallback ads1219Callback = NULL;

// ADS1219 const factor initialization
// INA241 external voltage reference is 5.0V (from INA241 VOUT)
#define INA241_EXT_VREF 5.0
#define INA241_INT_VREF 2.048

const ADS1219_FACTOR_t ads1219Factor = {
  .VREF_EXT_GAIN_1 = INA241_EXT_VREF / (INT24_MAX),
  .VREF_INT_GAIN_1 = INA241_INT_VREF / (INT24_MAX),
  .VREF_EXT_GAIN_4 = INA241_EXT_VREF / (INT24_MAX * 4),
  .VREF_INT_GAIN_4 = INA241_INT_VREF / (INT24_MAX * 4),
};

// Calibration timestamp tracking
static uint32_t nextRecalibrationTime = 0;

bool ads1219Init(const ADS1219_DRIVER_CONFIG_t *config) {
  if (config == NULL) {
    return false;
  }

  #if DEBUG_ADS1219
  Serial.println("ADS1219 Setup");
  #endif

  // Store configuration
  for (uint8_t i = 0; i < ADS1219_NUM_CHANNELS; i++) {
    ads1219ChannelSequence[i] = config->channelSequence[i];
  }
  ads1219SamplesPerChannel = config->samplesPerChannel;
  ads1219CalibrationSamplesConfig = config->calibrationSamples;
  ads1219RecalibrationInterval = config->recalibrationIntervalMs;
  ads1219Callback = config->dataReadyCallback;

  // Use hardware layer for initialization
  if (!ADS1219_HardwareInit(&ads1219Device, ads1219WriteBuffer, ads1219ReadBuffer)) {
    #if DEBUG_ADS1219
    Serial.println("ADS1219 Hardware Init Failed");
    #endif
    return false;
  }

  #if DEBUG_ADS1219
  Serial.print("Device found on address 0x");
  Serial.println(ads1219Device.address(), HEX);
  #endif

  // Configure ADS1219
  ads1219Config.MUX = ADS1219_MUX_PN_CALIB;  // start at AVCC/2 for calibration
  ads1219Config.GAIN = 0;                     // gain of 1x
  ads1219Config.DR = ADS1219_DR_20SPS;
  ads1219Config.CM = 1;                       // continuous mode
  ads1219Config.VREF = 1;                     // external vref REFP and REFN

  // Write configuration using hardware layer
  if (!ADS1219_WriteConfig(&ads1219Device, ads1219WriteBuffer, &ads1219Config)) {
    #if DEBUG_ADS1219
    Serial.println("ADS1219 Config Write Failed");
    #endif
    i2cErrorCount++;
    return false;
  }

  // Read back register to verify
  ADS1219_CONFIG_t readbackConfig;
  if (!ADS1219_ReadConfig(&ads1219Device, ads1219WriteBuffer, ads1219ReadBuffer, &readbackConfig)) {
    #if DEBUG_ADS1219
    Serial.println("ADS1219 Config Read Failed");
    #endif
    i2cErrorCount++;
    return false;
  }

  #if DEBUG_ADS1219
  if (readbackConfig.u8 == ads1219Config.u8) {
    Serial.println("ADS1219 Write Verified");
  } else {
    Serial.println("ADS1219 Write Failed");
    Serial.println(ads1219Config.u8, BIN);
    Serial.println(readbackConfig.u8, BIN);
  }
  #endif

  // Begin conversion using hardware layer
  if (!ADS1219_StartConversion(&ads1219Device, ads1219WriteBuffer)) {
    #if DEBUG_ADS1219
    Serial.println("ADS1219 Start Conversion Failed");
    #endif
    i2cErrorCount++;
    return false;
  }

  // Initialize gain states for all channels (start in low gain for safety)
  for (uint8_t i = 0; i < ADS1219_NUM_CHANNELS; i++) {
    ads1219InitGainState(&ads1219GainState[i], i, 0);
  }

  return true;
}

double getADS1219Factor(const ADS1219_CONFIG_t *config) {
  if (config->VREF == 1 && config->GAIN == 0) return ads1219Factor.VREF_EXT_GAIN_1;
  if (config->VREF == 0 && config->GAIN == 0) return ads1219Factor.VREF_INT_GAIN_1;
  if (config->VREF == 1 && config->GAIN == 1) return ads1219Factor.VREF_EXT_GAIN_4;
  if (config->VREF == 0 && config->GAIN == 1) return ads1219Factor.VREF_INT_GAIN_4;
  return 0;
}

void ads1219ReadData() {
  // Read data first to clear interrupt pin - use hardware layer
  if (!ADS1219_ReadData(&ads1219Device, ads1219WriteBuffer, ads1219ReadBuffer, &ads1219Value)) {
    i2cErrorCount++;
    #if DEBUG_ADS1219
    Serial.println("ERROR: ADS1219 data read failed");
    #endif
    return;
  }

  // Read current configuration - use hardware layer
  if (!ADS1219_ReadConfig(&ads1219Device, ads1219WriteBuffer, ads1219ReadBuffer, &ads1219Config)) {
    i2cErrorCount++;
    #if DEBUG_ADS1219
    Serial.println("ERROR: ADS1219 config read failed");
    #endif
    return;
  }
}

void ads1219StartCalibration() {
  #if DEBUG_ADS1219
  Serial.println("ADS1219: Starting calibration sequence");
  #endif

  // Set calibration mode: MUX to AVDD/2 for offset measurement
  ads1219Config.CM = 1;  // Continuous mode
  ads1219Config.MUX = ADS1219_MUX_PN_CALIB;  // MUX set to AVDD/2

  // Write new config using hardware layer
  if (!ADS1219_WriteConfig(&ads1219Device, ads1219WriteBuffer, &ads1219Config)) {
    i2cErrorCount++;
    #if DEBUG_ADS1219
    Serial.println("ERROR: Calibration config write failed");
    #endif
    return;
  }

  // Restart conversion using hardware layer
  if (!ADS1219_StartConversion(&ads1219Device, ads1219WriteBuffer)) {
    i2cErrorCount++;
    #if DEBUG_ADS1219
    Serial.println("ERROR: Calibration start conversion failed");
    #endif
    return;
  }

  // Reset calibration state
  ads1219CalibrationCount = 0;
  ads1219Calibrated = false;

  // Reset channel sampling state (calibration interrupts normal measurement)
  ads1219ChannelSampleCount = 0;
  ads1219ChannelAccumulator = 0;
  ads1219CurrentChannelIndex = 0;
}

void ads1219ProcessCalibrationSample() {
  // Bounds check to prevent array overflow
  if (ads1219CalibrationCount >= ADS1219_CALIBRATION_SAMPLES) {
    // Safety: Should never happen, but prevent overflow
    #if DEBUG_ADS1219
    Serial.println("ERROR: Calibration count overflow detected!");
    #endif
    ads1219FinishCalibration();
    return;
  }

  #if DEBUG_ADS1219
  Serial.printf("ADS1219: %u = %i\n", ads1219CalibrationCount, ads1219Value);
  #endif
  ads1219ValueArray[ads1219CalibrationCount] = ads1219Value;
  ads1219CalibrationCount++;

  // Check if we've collected enough samples
  if (ads1219CalibrationCount >= ADS1219_CALIBRATION_SAMPLES) {
    ads1219FinishCalibration();
  }
}

void ads1219FinishCalibration() {
  // Calculate average calibration value
  ads1219CalibrationValue = 0;
  for (uint8_t i = 0; i < ads1219CalibrationSamplesConfig; i++) {
    ads1219CalibrationValue += ads1219ValueArray[i];
  }
  ads1219CalibrationValue = ads1219CalibrationValue / ads1219CalibrationSamplesConfig;
  #if DEBUG_ADS1219
  Serial.printf("ADS1219: Calibration complete. Offset = %i (avg of %d samples)\n",
                ads1219CalibrationValue, ads1219CalibrationSamplesConfig);
  #endif

  // Switch to measurement mode - start with first channel in sequence
  ads1219CurrentChannelIndex = 0;
  ads1219CurrentChannel = ads1219ChannelSequence[ads1219CurrentChannelIndex];
  ads1219Config.MUX = ads1219CurrentChannel;

  // Write new config using hardware layer
  if (!ADS1219_WriteConfig(&ads1219Device, ads1219WriteBuffer, &ads1219Config)) {
    i2cErrorCount++;
    #if DEBUG_ADS1219
    Serial.println("ERROR: Post-calibration config write failed");
    #endif
    return;
  }

  // Restart conversion using hardware layer
  if (!ADS1219_StartConversion(&ads1219Device, ads1219WriteBuffer)) {
    i2cErrorCount++;
    #if DEBUG_ADS1219
    Serial.println("ERROR: Post-calibration start conversion failed");
    #endif
    return;
  }

  // Reset channel sampling state
  ads1219ChannelSampleCount = 0;
  ads1219ChannelAccumulator = 0;
  ads1219CurrentChannelIndex = 0;

  // Mark as calibrated and set next recalibration time (overflow-safe)
  ads1219Calibrated = true;
  ads1219CalibrationCount = 0;
  nextRecalibrationTime = millis() + ads1219RecalibrationInterval;

  #if DEBUG_ADS1219
  Serial.printf("ADS1219: Next calibration in %u minutes\n",
                ads1219RecalibrationInterval / 60000);
  #endif
}

void ads1219ProcessMeasurement() {
  int32_t rawValue = ads1219Value;

  // Select the correct offset based on current channel's VREF+GAIN configuration
  int32_t gainOffset = 0;
  if (ads1219GainState[ads1219CurrentChannelIndex].currentVref == 0 &&
      ads1219GainState[ads1219CurrentChannelIndex].currentGain == 1) {
    // Internal VREF + high GAIN (4x)
    gainOffset = ads1219GainState[ads1219CurrentChannelIndex].offsetHighGain;
  } else {
    // External VREF + low GAIN (1x)
    gainOffset = ads1219GainState[ads1219CurrentChannelIndex].offsetLowGain;
  }

  // Apply ADC offset calibration and gain-specific offset
  int32_t calibratedValue = ads1219Value - ads1219CalibrationValue - gainOffset;

  // Accumulate samples for current channel
  ads1219ChannelAccumulator += calibratedValue;
  ads1219ChannelSampleCount++;

  // Check if we have enough samples for this channel
  if (ads1219ChannelSampleCount >= ads1219SamplesPerChannel) {
    // Calculate average
    int32_t averageValue = ads1219ChannelAccumulator / ads1219SamplesPerChannel;

    // Convert ADC counts to voltage using correct VREF/GAIN for this channel
    // Create temporary config with the channel's actual VREF/GAIN settings
    ADS1219_CONFIG_t tempConfig = ads1219Config;
    tempConfig.VREF = ads1219GainState[ads1219CurrentChannelIndex].currentVref;
    tempConfig.GAIN = ads1219GainState[ads1219CurrentChannelIndex].currentGain;

    double conversionFactor = getADS1219Factor(&tempConfig);
    double voltage = (double)averageValue * conversionFactor;

    // Prepare channel data structure for callback
    ADS1219_CHANNEL_DATA_t channelData = {
      .channelId = ads1219CurrentChannelIndex,
      .muxConfig = ads1219CurrentChannel,
      .rawValue = rawValue,
      .calibratedValue = calibratedValue,
      .voltage = voltage,
      .sampleCount = ads1219SamplesPerChannel
    };

    // Invoke application callback with channel data
    if (ads1219Callback != NULL) {
      ads1219Callback(&channelData);
    }

    // Advance to next channel in sequence
    ads1219CurrentChannelIndex = (ads1219CurrentChannelIndex + 1) % ADS1219_NUM_CHANNELS;
    ads1219CurrentChannel = ads1219ChannelSequence[ads1219CurrentChannelIndex];

    // Write new channel configuration with correct VREF and GAIN for target channel
    ads1219Config.MUX = ads1219CurrentChannel;
    ads1219Config.VREF = ads1219GainState[ads1219CurrentChannelIndex].currentVref;
    ads1219Config.GAIN = ads1219GainState[ads1219CurrentChannelIndex].currentGain;

    #if DEBUG_GAIN_SWITCHING
    Serial.printf("[GAIN] ADS1219: Switch to CH%u (MUX=%u, VREF=%s, GAIN=%ux)\n",
                  ads1219CurrentChannelIndex,
                  ads1219CurrentChannel,
                  ads1219Config.VREF == 1 ? "Ext" : "Int",
                  ads1219Config.GAIN == 1 ? 4 : 1);
    #endif

    if (!ADS1219_WriteConfig(&ads1219Device, ads1219WriteBuffer, &ads1219Config)) {
      i2cErrorCount++;
      #if DEBUG_ADS1219
      Serial.println("ERROR: Channel switch config write failed");
      #endif
      return;
    }

    #if ADS1219_DEBUG_CHANNEL_SWITCH
    Serial.printf("ADS1219: Switched to channel index %d (MUX=%d)\n",
                  ads1219CurrentChannelIndex, ads1219CurrentChannel);
    #endif

    // Restart conversion with new channel using hardware layer
    if (!ADS1219_StartConversion(&ads1219Device, ads1219WriteBuffer)) {
      i2cErrorCount++;
      #if DEBUG_ADS1219
      Serial.println("ERROR: Channel switch start conversion failed");
      #endif
      return;
    }

    // Reset accumulator for next channel
    ads1219ChannelSampleCount = 0;
    ads1219ChannelAccumulator = 0;
  }
}

void ads1219ProcessDataReady() {
  ads1219ReadData();

  if (ads1219Calibrated) {
    // Check if recalibration needed (overflow-safe millis() check)
    uint32_t now = millis();
    if (now >= nextRecalibrationTime || nextRecalibrationTime == 0) {
      #if DEBUG_ADS1219
      Serial.println("Starting periodic ADS1219 recalibration...");
      #endif
      ads1219StartCalibration();
    } else {
      // Process normal measurement
      ads1219ProcessMeasurement();
    }
  } else {
    // Still calibrating - process calibration sample
    ads1219ProcessCalibrationSample();
  }
}

// ============================================================================
// Gain Switching Implementation
// ============================================================================

void ads1219InitGainState(ADS1219_GAIN_STATE_t *gainState,
                          uint8_t channel,
                          uint8_t initialGain) {
  // Initialize to safe state (low gain/wide range)
  gainState->currentGain = initialGain;

  // Initialize VREF and offset calibration values
  if (channel == 0) {
    // Current channel: Start with external VREF for full range
    gainState->currentVref = 1;  // External 5V reference
  } else {
    // Voltage channels: Must use external VREF (locked)
    gainState->currentVref = 1;  // External 5V reference (required)
  }

  // Initialize offset calibration for both modes
  gainState->offsetHighGain = 0;  // VREF=0 (internal) + GAIN=1 (4x)
  gainState->offsetLowGain = 0;   // VREF=1 (external) + GAIN=0 (1x)

  // Calculate thresholds based on channel type
  if (channel == 0) {
    // Channel 0: Current measurement via INA241
    // Only this channel uses automatic gain switching
    if (initialGain == 0) {
      // GAIN=0 (1x): ±12.5A effective max
      gainState->threshold50Percent = ADS1219_GAIN0_MAX_CURRENT * ADS1219_THRESHOLD_TO_HIGH_PERCENT / 100.0;
      gainState->threshold80Percent = 0;  // Not used in low gain
      gainState->threshold90Percent = 0;  // Not used in low gain
    } else {
      // GAIN=1 (4x): ±3.125A effective max
      gainState->threshold50Percent = 0;  // Not used in high gain
      gainState->threshold80Percent = ADS1219_GAIN1_MAX_CURRENT * ADS1219_THRESHOLD_TO_LOW_PERCENT / 100.0;
      gainState->threshold90Percent = ADS1219_GAIN1_MAX_CURRENT * ADS1219_THRESHOLD_EMERGENCY_PERCENT / 100.0;
    }
    gainState->autoSwitchingEnabled = true;  // Enable for current channel
  } else {
    // Channel 1/2: Voltage measurements (Solar+, Solar-)
    // These stay at GAIN=0 + external VREF due to high voltage range (0-18V)
    gainState->threshold50Percent = 0;
    gainState->threshold80Percent = 0;
    gainState->threshold90Percent = 0;
    gainState->autoSwitchingEnabled = false;  // Disable for voltage channels
  }

  // Initialize debounce counters
  gainState->consecutiveLowSamples = 0;
  gainState->consecutiveHighSamples = 0;
  gainState->emergencySamples = 0;

  // Configuration
  gainState->lowGainSamplesRequired = ADS1219_LOW_GAIN_SAMPLES_REQUIRED;
  gainState->highGainSamplesRequired = ADS1219_HIGH_GAIN_SAMPLES_REQUIRED;
  gainState->performOffsetCalibration = true;  // Enable offset cal by default

  // Statistics
  gainState->switchToHighCount = 0;
  gainState->switchToLowCount = 0;
  gainState->emergencySwitchCount = 0;
  gainState->lastSwitchMillis = 0;
}

bool ads1219SwitchGain(Adafruit_I2CDevice *device,
                       ADS1219_CONFIG_t *config,
                       ADS1219_GAIN_STATE_t *gainState,
                       uint8_t channel,
                       uint8_t newGain) {
  // Validate newGain
  if (newGain > 1) {
    #if DEBUG_GAIN_SWITCHING
    Serial.println("ERROR: Invalid GAIN (must be 0 or 1)");
    #endif
    return false;
  }

  // No-op if already at target gain
  if (newGain == gainState->currentGain) {
    return true;
  }

  // Determine VREF based on new GAIN (for current channel only)
  uint8_t newVref;
  if (channel == 0) {
    // Current channel: Switch VREF with GAIN for optimal resolution
    if (newGain == 1) {
      // High gain (4x) → use internal 2.048V reference for best resolution
      newVref = 0;  // Internal
    } else {
      // Low gain (1x) → use external 5V reference for full range
      newVref = 1;  // External
    }
  } else {
    // Voltage channels: Always use external VREF (locked)
    newVref = 1;  // External
  }

  #if DEBUG_GAIN_SWITCHING
  if (newGain == 1) {
    Serial.printf("[GAIN] ADS1219[CH%u]: Switching 1x→4x (VREF: %s→%s)\n",
                  channel,
                  gainState->currentVref == 1 ? "Ext" : "Int",
                  newVref == 1 ? "Ext" : "Int");
  } else {
    Serial.printf("[GAIN] ADS1219[CH%u]: Switching 4x→1x (VREF: %s→%s)\n",
                  channel,
                  gainState->currentVref == 1 ? "Ext" : "Int",
                  newVref == 1 ? "Ext" : "Int");
  }
  #endif

  // Modify GAIN and VREF bits in config
  config->GAIN = newGain;
  config->VREF = newVref;

  // Write configuration using hardware layer
  if (!ADS1219_WriteConfig(device, ads1219WriteBuffer, config)) {
    i2cErrorCount++;
    #if DEBUG_GAIN_SWITCHING
    Serial.println("ERROR: Failed to write CONFIG register");
    #endif
    return false;
  }

  // Restart conversion with new gain
  if (!ADS1219_StartConversion(device, ads1219WriteBuffer)) {
    i2cErrorCount++;
    #if DEBUG_GAIN_SWITCHING
    Serial.println("ERROR: Failed to start conversion");
    #endif
    return false;
  }

  // Perform offset calibration if enabled
  if (gainState->performOffsetCalibration) {
    #if DEBUG_GAIN_SWITCHING
    Serial.printf("[GAIN] ADS1219[CH%u]: Performing offset calibration...\n", channel);
    #endif
    if (!ads1219PerformOffsetCalibration(device, config, gainState)) {
      #if DEBUG_GAIN_SWITCHING
      Serial.println("WARNING: Offset calibration failed");
      #endif
      // Continue anyway - offset calibration failure is not fatal
    }
  }

  // Update gain state
  gainState->currentGain = newGain;
  gainState->currentVref = newVref;
  gainState->lastSwitchMillis = millis();

  // Update thresholds for new gain (channel 0 only)
  if (channel == 0) {
    if (newGain == 0) {
      // Now in low gain (1x)
      gainState->threshold50Percent = ADS1219_GAIN0_MAX_CURRENT * ADS1219_THRESHOLD_TO_HIGH_PERCENT / 100.0;
      gainState->threshold80Percent = 0;
      gainState->threshold90Percent = 0;
      gainState->switchToLowCount++;
    } else {
      // Now in high gain (4x)
      gainState->threshold50Percent = 0;
      gainState->threshold80Percent = ADS1219_GAIN1_MAX_CURRENT * ADS1219_THRESHOLD_TO_LOW_PERCENT / 100.0;
      gainState->threshold90Percent = ADS1219_GAIN1_MAX_CURRENT * ADS1219_THRESHOLD_EMERGENCY_PERCENT / 100.0;
      gainState->switchToHighCount++;
    }
  }

  // Reset debounce counters
  gainState->consecutiveLowSamples = 0;
  gainState->consecutiveHighSamples = 0;
  gainState->emergencySamples = 0;

  #if DEBUG_GAIN_SWITCHING
  Serial.printf("[GAIN] ADS1219[CH%u]: Gain switch complete (new gain=%ux)\n",
                channel, newGain == 0 ? 1 : 4);
  #endif

  return true;
}

bool ads1219PerformOffsetCalibration(Adafruit_I2CDevice *device,
                                     ADS1219_CONFIG_t *config,
                                     ADS1219_GAIN_STATE_t *gainState) {
  // Save current MUX setting
  ADS1219_MUX_t savedMux = (ADS1219_MUX_t)config->MUX;

  // Switch to calibration mode (AVDD/2)
  config->MUX = ADS1219_MUX_PN_CALIB;
  if (!ADS1219_WriteConfig(device, ads1219WriteBuffer, config)) {
    i2cErrorCount++;
    return false;
  }

  // Restart conversion
  if (!ADS1219_StartConversion(device, ads1219WriteBuffer)) {
    i2cErrorCount++;
    return false;
  }

  // Collect calibration samples
  const uint8_t OFFSET_CAL_SAMPLES = 20;
  int32_t offsetSum = 0;
  int32_t tempValue;

  for (uint8_t i = 0; i < OFFSET_CAL_SAMPLES; i++) {
    // Wait for conversion (at 20 SPS, ~50ms per sample)
    delay(55);  // Slight margin

    // Read data
    if (!ADS1219_ReadData(device, ads1219WriteBuffer, ads1219ReadBuffer, &tempValue)) {
      i2cErrorCount++;
      #if DEBUG_GAIN_SWITCHING
      Serial.println("ERROR: Offset calibration read failed");
      #endif
      // Restore original MUX before returning
      config->MUX = savedMux;
      ADS1219_WriteConfig(device, ads1219WriteBuffer, config);
      return false;
    }

    offsetSum += tempValue;

    #if DEBUG_GAIN_SWITCHING
    if (i % 5 == 0) {
      Serial.printf("[GAIN] ADS1219: Offset cal sample %u/%u: %d\n",
                    i+1, OFFSET_CAL_SAMPLES, tempValue);
    }
    #endif
  }

  // Calculate average offset and store in appropriate field based on current VREF+GAIN
  int32_t calculatedOffset = offsetSum / OFFSET_CAL_SAMPLES;

  if (gainState->currentVref == 0 && gainState->currentGain == 1) {
    // Internal VREF + high GAIN (4x)
    gainState->offsetHighGain = calculatedOffset;
    #if DEBUG_GAIN_SWITCHING
    Serial.printf("[GAIN] ADS1219: Offset cal complete (Internal 2.048V + 4x). Offset = %d\n",
                  calculatedOffset);
    #endif
  } else {
    // External VREF + low GAIN (1x)
    gainState->offsetLowGain = calculatedOffset;
    #if DEBUG_GAIN_SWITCHING
    Serial.printf("[GAIN] ADS1219: Offset cal complete (External 5V + 1x). Offset = %d\n",
                  calculatedOffset);
    #endif
  }

  // Restore original MUX setting
  config->MUX = savedMux;
  if (!ADS1219_WriteConfig(device, ads1219WriteBuffer, config)) {
    i2cErrorCount++;
    return false;
  }

  // Restart conversion with original channel
  if (!ADS1219_StartConversion(device, ads1219WriteBuffer)) {
    i2cErrorCount++;
    return false;
  }

  return true;
}

void ads1219EvaluateGainSwitch(uint8_t channel,
                               float measurement,
                               ADS1219_GAIN_STATE_t *gainState,
                               Adafruit_I2CDevice *device,
                               ADS1219_CONFIG_t *config) {
  // Skip if auto-switching is disabled for this channel
  if (!gainState->autoSwitchingEnabled) {
    return;
  }

  // Only channel 0 (current) uses automatic gain switching
  if (channel != 0) {
    return;
  }

  // Get absolute value of measurement
  float absMeasurement = fabs(measurement);

  if (gainState->currentGain == 0) {
    // Currently in LOW gain (GAIN=0, 1x)
    // Check if we should switch to HIGH gain (4x)
    if (absMeasurement < gainState->threshold50Percent) {
      gainState->consecutiveLowSamples++;
      if (gainState->consecutiveLowSamples >= gainState->lowGainSamplesRequired) {
        // Switch to high gain
        #if DEBUG_GAIN_SWITCHING
        Serial.printf("[GAIN] ADS1219[CH%u]: 1x→4x (I=%.2fA, samples=%u)\n",
                      channel, absMeasurement, gainState->consecutiveLowSamples);
        #endif
        ads1219SwitchGain(device, config, gainState, channel, 1);
      }
    } else {
      // Reset counter if measurement goes above threshold
      gainState->consecutiveLowSamples = 0;
    }
  } else {
    // Currently in HIGH gain (GAIN=1, 4x)
    // Check for emergency switch first
    if (absMeasurement > gainState->threshold90Percent) {
      gainState->emergencySamples++;
      if (gainState->emergencySamples >= ADS1219_EMERGENCY_SAMPLES_REQUIRED) {
        // Emergency switch to low gain
        #if DEBUG_GAIN_SWITCHING
        Serial.printf("[GAIN] ADS1219[CH%u]: 4x→1x EMERGENCY (I=%.2fA)\n",
                      channel, absMeasurement);
        #endif
        gainState->emergencySwitchCount++;
        ads1219SwitchGain(device, config, gainState, channel, 0);
      }
    }
    // Check for normal switch to low gain
    else if (absMeasurement > gainState->threshold80Percent) {
      gainState->consecutiveHighSamples++;
      gainState->emergencySamples = 0;  // Reset emergency counter
      if (gainState->consecutiveHighSamples >= gainState->highGainSamplesRequired) {
        // Switch to low gain
        #if DEBUG_GAIN_SWITCHING
        Serial.printf("[GAIN] ADS1219[CH%u]: 4x→1x (I=%.2fA, samples=%u)\n",
                      channel, absMeasurement, gainState->consecutiveHighSamples);
        #endif
        ads1219SwitchGain(device, config, gainState, channel, 0);
      }
    } else {
      // Reset counters if measurement is in normal range
      gainState->consecutiveHighSamples = 0;
      gainState->emergencySamples = 0;
    }
  }
}
