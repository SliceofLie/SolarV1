#include "debug_config.h"
#include "ina228_driver.h"

// Global error counter (declared in main.cpp)
extern uint32_t i2cErrorCount;

// Global I2C device objects
Adafruit_I2CDevice ina228BattDevice = Adafruit_I2CDevice(INA228_BATT_ADDR);
Adafruit_I2CDevice ina228LoadDevice = Adafruit_I2CDevice(INA228_LOAD_ADDR);

// Global register and measurement data
INA228_REGISTERS_t ina228BattRegisters = {.reg_adc_config = {.u16 = 0xFB68}, .reg_diag_alrt = {.MEMSTAT = 1}};
INA228_REGISTERS_t ina228LoadRegisters = {.reg_adc_config = {.u16 = 0xFB68}, .reg_diag_alrt = {.MEMSTAT = 1}};
INA228_MEASUREMENTS_t ina228BattMeasurements;
INA228_MEASUREMENTS_t ina228LoadMeasurements;

// Global gain state tracking
INA228_GAIN_STATE_t ina228BattGainState;
INA228_GAIN_STATE_t ina228LoadGainState;

bool ina228Init() {
  #if DEBUG_INA228
  Serial.println("I2C address detection test");
  #endif

  // Set I2C speed for INA228 devices
  ina228BattDevice.setSpeed(I2C_SPEED_HZ);
  ina228LoadDevice.setSpeed(I2C_SPEED_HZ);

  bool allDevicesFound = true;

  if (ina228BattDevice.begin()) {
    #if DEBUG_INA228
    Serial.print("Device found on address 0x");
    Serial.println(ina228BattDevice.address(), HEX);
    #endif
  } else {
    #if DEBUG_INA228
    Serial.print("Did not find device at 0x");
    Serial.println(ina228BattDevice.address(), HEX);
    #endif
    allDevicesFound = false;
  }

  if (ina228LoadDevice.begin()) {
    #if DEBUG_INA228
    Serial.print("Device found on address 0x");
    Serial.println(ina228LoadDevice.address(), HEX);
    #endif
  } else {
    #if DEBUG_INA228
    Serial.print("Did not find device at 0x");
    Serial.println(ina228LoadDevice.address(), HEX);
    #endif
    allDevicesFound = false;
  }

  if (!allDevicesFound) {
    return false;
  }

  // Initialize both INA228 devices
  #if DEBUG_INA228
  Serial.println("INA228 Setup");
  #endif

  // Configuration for both INA228 devices
  // 4120us ADC conversion time with 128 sample averaging yields 19.7 bits noise-free ENOB with ADCRANGE = 0
  INA228Config config = {
    .adcRange = 0,
    .mode = INA228_MODE_CONT_BUS_SHUNT_TEMP,
    .busConvTime = INA228_CONVTIME_4120,
    .shuntConvTime = INA228_CONVTIME_4120,
    .tempConvTime = INA228_CONVTIME_4120,
    .avgCount = INA228_AVGCOUNT_128,
    .shuntCal = INA228_SHUNT_CAL_VALUE,
    .enableConversionAlert = true
  };

  // Setup both INA228 devices with same configuration
  setupINA228(&ina228BattDevice, &ina228BattRegisters, &config);
  setupINA228(&ina228LoadDevice, &ina228LoadRegisters, &config);

  // Initialize gain states (start in wide range for safety)
  ina228InitGainState(&ina228BattGainState, 0);
  ina228InitGainState(&ina228LoadGainState, 0);

  return true;
}

void setupINA228(Adafruit_I2CDevice *device, INA228_REGISTERS_t *regs, const INA228Config *config) {
  // Reset
  ina228WriteBuffer[0] = INA228_REG_CONFIG;
  ina228WriteBuffer[1] = 0x80;
  ina228WriteBuffer[2] = 0x00;
  if (!device->write(ina228WriteBuffer, 3)) {
    i2cErrorCount++;
    #if DEBUG_INA228
    Serial.println("ERROR: INA228 reset failed");
    #endif
    return;
  }

  // Configure ADCRANGE
  regs->reg_config.ADCRANGE = config->adcRange;
  ina228WriteBuffer[0] = INA228_REG_ADC_CONFIG;
  ina228WriteBuffer[1] = regs->reg_config.u16 >> 8;
  ina228WriteBuffer[2] = regs->reg_config.u16 & 0xFF;
  if (!device->write(ina228WriteBuffer, 3)) {
    i2cErrorCount++;
    #if DEBUG_INA228
    Serial.println("ERROR: INA228 ADCRANGE config failed");
    #endif
    return;
  }

  // Generate and write ADC_CONFIG
  regs->reg_adc_config.u16 = ina228AdcConfig(config->mode, config->busConvTime,
                                             config->shuntConvTime, config->tempConvTime,
                                             config->avgCount);
  ina228WriteBuffer[0] = INA228_REG_ADC_CONFIG;
  ina228WriteBuffer[1] = regs->reg_adc_config.u16 >> 8;
  ina228WriteBuffer[2] = regs->reg_adc_config.u16 & 0xFF;
  if (!device->write(ina228WriteBuffer, 3)) {
    i2cErrorCount++;
    #if DEBUG_INA228
    Serial.println("ERROR: INA228 ADC_CONFIG write failed");
    #endif
    return;
  }

  // Set shunt calibration
  ina228WriteBuffer[0] = INA228_REG_SHUNT_CAL;
  ina228WriteBuffer[1] = config->shuntCal >> 8;
  ina228WriteBuffer[2] = config->shuntCal & 0xFF;
  if (!device->write(ina228WriteBuffer, 3)) {
    i2cErrorCount++;
    #if DEBUG_INA228
    Serial.println("ERROR: INA228 shunt calibration write failed");
    #endif
    return;
  }

  // Configure alerts
  if (config->enableConversionAlert) {
    regs->reg_diag_alrt.CNVR = 1;
    ina228WriteBuffer[0] = INA228_REG_DIAG_ALRT;
    ina228WriteBuffer[1] = regs->reg_diag_alrt.u16 >> 8;
    ina228WriteBuffer[2] = regs->reg_diag_alrt.u16 & 0xFF;
    if (!device->write(ina228WriteBuffer, 3)) {
      i2cErrorCount++;
      #if DEBUG_INA228
      Serial.println("ERROR: INA228 alert config write failed");
      #endif
      return;
    }
  }
}

void applyINA228Calibration(INA228_MEASUREMENTS_t* measurements, float voltageScale, float currentOffset, float currentScale) {
  // Apply calibration factors to measurements
  measurements->busvoltage *= voltageScale;
  measurements->current = (measurements->current + currentOffset) * currentScale;
  measurements->power = measurements->busvoltage * measurements->current;

  // Validate physical limits (INA228 max: 85V bus, ±32A with 5mΩ shunt)
  #if DEBUG_INA228
  if (abs(measurements->busvoltage) > 90.0) {
    Serial.printf("WARNING: Bus voltage out of range: %.2fV\n", measurements->busvoltage);
  }
  if (abs(measurements->current) > 35.0) {
    Serial.printf("WARNING: Current out of range: %.2fA\n", measurements->current);
  }
  #endif
}

void printINA228Measurements(const char* channel, INA228_REGISTERS_t *regs, INA228_MEASUREMENTS_t *meas) {
  #if DEBUG_INA228
  Serial.printf("%s VSHUNT: 0x%X = %i = %f nV\n", channel, regs->reg_vshunt, regs->reg_vshunt, meas->shuntvoltage);
  Serial.printf("%s VBUS: 0x%X = %i = %f V\n", channel, regs->reg_vbus, regs->reg_vbus, meas->busvoltage);
  Serial.printf("%s DIETEMP: 0x%X = %i = %f C\n", channel, regs->reg_dietemp, regs->reg_dietemp, meas->dietemp);
  Serial.printf("%s CURRENT: 0x%X = %i = %f A\n", channel, regs->reg_current, regs->reg_current, meas->current);
  Serial.printf("%s POWER: 0x%X = %i = %f W\n", channel, regs->reg_power, regs->reg_power, meas->power);
  Serial.printf("%s ENERGY: 0x%" PRIX64 " = %f Joules\n", channel, regs->reg_energy, meas->energy);
  Serial.printf("%s CHARGE: 0x%" PRIX64 " = %f Coulombs\n", channel, regs->reg_charge, meas->charge);
  Serial.printf("%s Calculated Current = %f A\n", channel, meas->shuntvoltage / R_SHUNT_MICROVOLTS);
  Serial.printf("%s Calculated Power = %f W\n", channel, (meas->shuntvoltage / R_SHUNT_MICROVOLTS) * meas->busvoltage);
  #endif
}

void ina228ProcessBatteryAlert() {
  #if DEBUG_INA228
  Serial.println("ina228BattAlert Triggered");
  #endif
  ina228Processor(&ina228BattDevice, &ina228BattRegisters, &ina228BattMeasurements);

  // Read calibration values with interrupt protection (race condition fix)
  noInterrupts();
  float voltageScale = calibration.batt_voltage_scale;
  float currentOffset = calibration.batt_current_offset;
  float currentScale = calibration.batt_current_scale;
  interrupts();

  applyINA228Calibration(&ina228BattMeasurements, voltageScale, currentOffset, currentScale);
  printINA228Measurements("BATT", &ina228BattRegisters, &ina228BattMeasurements);

  // Evaluate if gain switching is needed
  ina228EvaluateGainSwitch(&ina228BattMeasurements, &ina228BattGainState,
                           &ina228BattDevice, &ina228BattRegisters);
}

void ina228ProcessLoadAlert() {
  #if DEBUG_INA228
  Serial.println("ina228LoadAlert Triggered");
  #endif
  ina228Processor(&ina228LoadDevice, &ina228LoadRegisters, &ina228LoadMeasurements);

  // Read calibration values with interrupt protection (race condition fix)
  noInterrupts();
  float voltageScale = calibration.load_voltage_scale;
  float currentOffset = calibration.load_current_offset;
  float currentScale = calibration.load_current_scale;
  interrupts();

  applyINA228Calibration(&ina228LoadMeasurements, voltageScale, currentOffset, currentScale);
  printINA228Measurements("LOAD", &ina228LoadRegisters, &ina228LoadMeasurements);

  // Evaluate if gain switching is needed
  ina228EvaluateGainSwitch(&ina228LoadMeasurements, &ina228LoadGainState,
                           &ina228LoadDevice, &ina228LoadRegisters);
}

// ============================================================================
// Gain Switching Implementation
// ============================================================================

void ina228InitGainState(INA228_GAIN_STATE_t *gainState, uint8_t initialRange) {
  // Initialize to safe state (wide range)
  gainState->currentRange = initialRange;
  gainState->currentShuntCal = INA228_SHUNT_CAL_VALUE;

  // Calculate thresholds based on range
  if (initialRange == 0) {
    // ADCRANGE=0: ±32.768A
    gainState->threshold50Percent = INA228_RANGE0_MAX_CURRENT * INA228_THRESHOLD_TO_HIGH_PERCENT / 100.0;
    gainState->threshold80Percent = 0;  // Not used in wide range
    gainState->threshold90Percent = 0;  // Not used in wide range
  } else {
    // ADCRANGE=1: ±8.192A
    gainState->threshold50Percent = 0;  // Not used in narrow range
    gainState->threshold80Percent = INA228_RANGE1_MAX_CURRENT * INA228_THRESHOLD_TO_LOW_PERCENT / 100.0;
    gainState->threshold90Percent = INA228_RANGE1_MAX_CURRENT * INA228_THRESHOLD_EMERGENCY_PERCENT / 100.0;
  }

  // Initialize debounce counters
  gainState->consecutiveLowSamples = 0;
  gainState->consecutiveHighSamples = 0;
  gainState->emergencySamples = 0;

  // Configuration
  gainState->lowGainSamplesRequired = INA228_LOW_GAIN_SAMPLES_REQUIRED;
  gainState->highGainSamplesRequired = INA228_HIGH_GAIN_SAMPLES_REQUIRED;
  gainState->autoSwitchingEnabled = true;  // Enable by default

  // Statistics
  gainState->switchToHighCount = 0;
  gainState->switchToLowCount = 0;
  gainState->emergencySwitchCount = 0;
  gainState->lastSwitchMillis = 0;
}

bool ina228SwitchRange(Adafruit_I2CDevice *device,
                       INA228_REGISTERS_t *regs,
                       INA228_GAIN_STATE_t *gainState,
                       uint8_t newRange) {
  // Validate newRange
  if (newRange > 1) {
    #if DEBUG_GAIN_SWITCHING
    Serial.println("ERROR: Invalid ADCRANGE (must be 0 or 1)");
    #endif
    return false;
  }

  // No-op if already at target range
  if (newRange == gainState->currentRange) {
    return true;
  }

  uint16_t newShuntCal;

  // Calculate new SHUNT_CAL value
  if (newRange == 1) {
    // Switching to narrow range (ADCRANGE 0→1): multiply by 4
    newShuntCal = gainState->currentShuntCal * 4;
    #if DEBUG_GAIN_SWITCHING
    Serial.printf("[GAIN] INA228: Switching WIDE→NARROW (SHUNT_CAL: %u→%u)\n",
                  gainState->currentShuntCal, newShuntCal);
    #endif
  } else {
    // Switching to wide range (ADCRANGE 1→0): divide by 4
    newShuntCal = gainState->currentShuntCal / 4;
    #if DEBUG_GAIN_SWITCHING
    Serial.printf("[GAIN] INA228: Switching NARROW→WIDE (SHUNT_CAL: %u→%u)\n",
                  gainState->currentShuntCal, newShuntCal);
    #endif
  }

  // Validate SHUNT_CAL is in valid range
  if (newShuntCal < 1 || newShuntCal > 65535) {
    #if DEBUG_GAIN_SWITCHING
    Serial.printf("ERROR: SHUNT_CAL out of range: %u\n", newShuntCal);
    #endif
    return false;
  }

  // Read current CONFIG register
  ina228WriteBuffer[0] = INA228_REG_CONFIG;
  if (!device->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 2)) {
    i2cErrorCount++;
    #if DEBUG_GAIN_SWITCHING
    Serial.println("ERROR: Failed to read CONFIG register");
    #endif
    return false;
  }

  // Modify ADCRANGE bit (bit 4)
  uint16_t configValue = (ina228ReadBuffer[0] << 8) | ina228ReadBuffer[1];
  if (newRange == 1) {
    configValue |= (1 << 4);  // Set bit 4
  } else {
    configValue &= ~(1 << 4);  // Clear bit 4
  }

  // Write modified CONFIG register
  ina228WriteBuffer[0] = INA228_REG_CONFIG;
  ina228WriteBuffer[1] = configValue >> 8;
  ina228WriteBuffer[2] = configValue & 0xFF;
  if (!device->write(ina228WriteBuffer, 3)) {
    i2cErrorCount++;
    #if DEBUG_GAIN_SWITCHING
    Serial.println("ERROR: Failed to write CONFIG register");
    #endif
    return false;
  }

  // Write new SHUNT_CAL value
  ina228WriteBuffer[0] = INA228_REG_SHUNT_CAL;
  ina228WriteBuffer[1] = newShuntCal >> 8;
  ina228WriteBuffer[2] = newShuntCal & 0xFF;
  if (!device->write(ina228WriteBuffer, 3)) {
    i2cErrorCount++;
    #if DEBUG_GAIN_SWITCHING
    Serial.println("ERROR: Failed to write SHUNT_CAL register");
    #endif
    return false;
  }

  // Update gain state
  gainState->currentRange = newRange;
  gainState->currentShuntCal = newShuntCal;
  gainState->lastSwitchMillis = millis();

  // Update thresholds for new range
  if (newRange == 0) {
    // Now in wide range
    gainState->threshold50Percent = INA228_RANGE0_MAX_CURRENT * INA228_THRESHOLD_TO_HIGH_PERCENT / 100.0;
    gainState->threshold80Percent = 0;
    gainState->threshold90Percent = 0;
    gainState->switchToLowCount++;
  } else {
    // Now in narrow range
    gainState->threshold50Percent = 0;
    gainState->threshold80Percent = INA228_RANGE1_MAX_CURRENT * INA228_THRESHOLD_TO_LOW_PERCENT / 100.0;
    gainState->threshold90Percent = INA228_RANGE1_MAX_CURRENT * INA228_THRESHOLD_EMERGENCY_PERCENT / 100.0;
    gainState->switchToHighCount++;
  }

  // Reset debounce counters
  gainState->consecutiveLowSamples = 0;
  gainState->consecutiveHighSamples = 0;
  gainState->emergencySamples = 0;

  // Update register structure ADCRANGE bit
  regs->reg_config.ADCRANGE = newRange;

  #if DEBUG_GAIN_SWITCHING
  Serial.printf("[GAIN] INA228: Range switch complete (new range=%u)\n", newRange);
  #endif

  return true;
}

void ina228EvaluateGainSwitch(INA228_MEASUREMENTS_t *measurements,
                              INA228_GAIN_STATE_t *gainState,
                              Adafruit_I2CDevice *device,
                              INA228_REGISTERS_t *regs) {
  // Skip if auto-switching is disabled
  if (!gainState->autoSwitchingEnabled) {
    return;
  }

  // Get absolute value of current
  float absCurrent = fabs(measurements->current);

  if (gainState->currentRange == 0) {
    // Currently in WIDE range (ADCRANGE=0)
    // Check if we should switch to NARROW range
    if (absCurrent < gainState->threshold50Percent) {
      gainState->consecutiveLowSamples++;
      if (gainState->consecutiveLowSamples >= gainState->lowGainSamplesRequired) {
        // Switch to narrow range
        #if DEBUG_GAIN_SWITCHING
        Serial.printf("[GAIN] INA228: WIDE→NARROW (I=%.2fA, samples=%u)\n",
                      absCurrent, gainState->consecutiveLowSamples);
        #endif
        ina228SwitchRange(device, regs, gainState, 1);
      }
    } else {
      // Reset counter if current goes above threshold
      gainState->consecutiveLowSamples = 0;
    }
  } else {
    // Currently in NARROW range (ADCRANGE=1)
    // Check for emergency switch first
    if (absCurrent > gainState->threshold90Percent) {
      gainState->emergencySamples++;
      if (gainState->emergencySamples >= INA228_EMERGENCY_SAMPLES_REQUIRED) {
        // Emergency switch to wide range
        #if DEBUG_GAIN_SWITCHING
        Serial.printf("[GAIN] INA228: NARROW→WIDE EMERGENCY (I=%.2fA)\n", absCurrent);
        #endif
        gainState->emergencySwitchCount++;
        ina228SwitchRange(device, regs, gainState, 0);
      }
    }
    // Check for normal switch to wide range
    else if (absCurrent > gainState->threshold80Percent) {
      gainState->consecutiveHighSamples++;
      gainState->emergencySamples = 0;  // Reset emergency counter
      if (gainState->consecutiveHighSamples >= gainState->highGainSamplesRequired) {
        // Switch to wide range
        #if DEBUG_GAIN_SWITCHING
        Serial.printf("[GAIN] INA228: NARROW→WIDE (I=%.2fA, samples=%u)\n",
                      absCurrent, gainState->consecutiveHighSamples);
        #endif
        ina228SwitchRange(device, regs, gainState, 0);
      }
    } else {
      // Reset counters if current is in normal range
      gainState->consecutiveHighSamples = 0;
      gainState->emergencySamples = 0;
    }
  }
}
