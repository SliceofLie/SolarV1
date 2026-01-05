#include "INA228.h"

// Note: I2C buffers are declared in main.cpp as global variables
// (ina228WriteBuffer and ina228ReadBuffer) and used by this module
extern uint8_t ina228WriteBuffer[8];
extern uint8_t ina228ReadBuffer[8];

uint16_t ina228AdcConfig(
  INA228_MODE_t ina228Mode, 
  INA228_CONVTIME_t busConvTime, 
  INA228_CONVTIME_t shuntConvTime, 
  INA228_CONVTIME_t tempConvTime, 
  INA228_AVGCOUNT_t avgCount
) {
  return (uint16_t)(ina228Mode << 12 | busConvTime << 9 | shuntConvTime << 6 | tempConvTime << 3 | avgCount);
}

void ina228Processor(
  Adafruit_I2CDevice *i2cDevice, 
  INA228_REGISTERS_t *ina228Registers, 
  INA228_MEASUREMENTS_t *ina228Measurements
) {
  // ALERT pin has triggered an interrupt, determine source of interrupt by reading DIAG_ALRT
  ina228WriteBuffer[0] = INA228_REG_DIAG_ALRT;
  i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 2);
  ina228Registers->reg_diag_alrt.u16 = ina228ReadBuffer[0] << 8 | ina228ReadBuffer[1];

  if (ina228Registers->reg_diag_alrt.CNVRF == 1) { // CONVERSION COMPLETED
    // Read out values
    ina228WriteBuffer[0] = INA228_REG_CONFIG;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 2);
    ina228Registers->reg_config.u16 = ina228ReadBuffer[0] << 8 | ina228ReadBuffer[1];

    ina228WriteBuffer[0] = INA228_REG_VSHUNT;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 3);
    ina228Registers->reg_vshunt = ina228ReadBuffer[0] << 24 | ina228ReadBuffer[1] << 16 | ina228ReadBuffer[2] << 8;
    ina228Registers->reg_vshunt = (ina228Registers->reg_vshunt >> 12);

    ina228WriteBuffer[0] = INA228_REG_VBUS;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 3);
    ina228Registers->reg_vbus = ina228ReadBuffer[0] << 24 | ina228ReadBuffer[1] << 16 | ina228ReadBuffer[2] << 8;
    ina228Registers->reg_vbus = (ina228Registers->reg_vbus >> 12);

    ina228WriteBuffer[0] = INA228_REG_DIETEMP;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 2);
    ina228Registers->reg_dietemp = ina228ReadBuffer[0] << 8 | ina228ReadBuffer[1];

    ina228WriteBuffer[0] = INA228_REG_CURRENT;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 3);
    ina228Registers->reg_current = ina228ReadBuffer[0] << 24 | ina228ReadBuffer[1] << 16 | ina228ReadBuffer[2] << 8;
    ina228Registers->reg_current = (ina228Registers->reg_current >> 12);

    ina228WriteBuffer[0] = INA228_REG_POWER;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 3);
    ina228Registers->reg_power = ina228ReadBuffer[0] << 16 | ina228ReadBuffer[1] << 8 | ina228ReadBuffer[2]; // full 24bit unsigned

    ina228WriteBuffer[0] = INA228_REG_ENERGY;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 5);
    ina228Registers->reg_energy = (uint64_t)ina228ReadBuffer[0] << 32 | ina228ReadBuffer[1] << 24 | ina228ReadBuffer[2] << 16 | ina228ReadBuffer[3] << 8 | ina228ReadBuffer[4]; // 40 bit unsigned
    
    ina228WriteBuffer[0] = INA228_REG_CHARGE;
    i2cDevice->write_then_read(ina228WriteBuffer, 1, ina228ReadBuffer, 5);
    ina228Registers->reg_charge = (uint64_t)ina228ReadBuffer[0] << 32 | ina228ReadBuffer[1] << 24 | ina228ReadBuffer[2] << 16 | ina228ReadBuffer[3] << 8 | ina228ReadBuffer[4]; // 40 bit two's complement

    if (((ina228Registers->reg_charge >> 39) & 0x01) == 1) { // if negative 40 bit value, need to sign extend the 64bit variable
      ina228Registers->reg_charge = ina228Registers->reg_charge | ((uint64_t)0xFFFFFF << 40);
    }

    // Final calculations from register values
    if (ina228Registers->reg_config.ADCRANGE == 0) {
      ina228Measurements->shuntvoltage = (ina228Registers->reg_vshunt * INA228_VSHUNT0_FACTOR);
    } else {
      ina228Measurements->shuntvoltage = (ina228Registers->reg_vshunt * INA228_VSHUNT1_FACTOR);
    }
    ina228Measurements->busvoltage = (ina228Registers->reg_vbus * INA228_VBUS_FACTOR) / 1E6; // get value in volts
    ina228Measurements->dietemp = (ina228Registers->reg_dietemp * INA228_DIETEMP_FACTOR) / 1000; // get value in degrees C
    ina228Measurements->current = INA228_CURRENT_LSB * ina228Registers->reg_current;
    ina228Measurements->power = 3.2 * INA228_CURRENT_LSB * ina228Registers->reg_power;
    ina228Measurements->energy = 51.2 * INA228_CURRENT_LSB * ina228Registers->reg_energy;
    ina228Measurements->charge = INA228_CURRENT_LSB * ina228Registers->reg_charge;
  }
}
