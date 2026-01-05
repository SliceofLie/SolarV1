#ifndef INA228_H
#define INA228_H

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>

// INA228 Register Addresses
#define INA228_REG_CONFIG       0x00
#define INA228_REG_ADC_CONFIG   0x01
#define INA228_REG_SHUNT_CAL    0x02
#define INA228_REG_SHUNT_TEMPCO 0x03
#define INA228_REG_VSHUNT       0x04
#define INA228_REG_VBUS         0x05
#define INA228_REG_DIETEMP      0x06
#define INA228_REG_CURRENT      0x07
#define INA228_REG_POWER        0x08
#define INA228_REG_ENERGY       0x09
#define INA228_REG_CHARGE       0x0A
#define INA228_REG_DIAG_ALRT    0x0B

// INA228 Conversion Factors
#define INA228_VSHUNT0_FACTOR 312.500   // 312.5 nV/LSB when ADCRANGE = 0
#define INA228_VSHUNT1_FACTOR 78.125    // 78.125 nV/LSB when ADCRANGE = 1
#define INA228_VBUS_FACTOR 195.3125     // 195.3125 µV/LSB
#define INA228_DIETEMP_FACTOR 7.8125    // 7.8125 m°C/LSB

// INA228 Constants
#define VALPOW2TO19 524288.0
#define INA228_MAXCURRENT 32.768
#define INA228_CURRENT_LSB 0.0000625    // 32.768 Amps / 2^19

// INA228 Enumerations
enum INA228_MODE_t {
  INA228_MODE_SHUTDOWN0,
  INA228_MODE_TRIG_BUS,
  INA228_MODE_TRIG_SHUNT,
  INA228_MODE_TRIG_BUS_SHUNT,
  INA228_MODE_TRIG_TEMP,
  INA228_MODE_TRIG_BUS_TEMP,
  INA228_MODE_TRIG_SHUNT_TEMP,
  INA228_MODE_TRIG_BUS_SHUNT_TEMP,
  INA228_MODE_SHUTDOWN1,              // filler, same as INA228_MODE_SHUTDOWN0
  INA228_MODE_CONT_BUS,
  INA228_MODE_CONT_SHUNT,
  INA228_MODE_CONT_BUS_SHUNT,
  INA228_MODE_CONT_TEMP,
  INA228_MODE_CONT_BUS_TEMP,
  INA228_MODE_CONT_SHUNT_TEMP,
  INA228_MODE_CONT_BUS_SHUNT_TEMP     // Power On Reset state
};

enum INA228_CONVTIME_t {
  INA228_CONVTIME_50,                 // 50us
  INA228_CONVTIME_84,
  INA228_CONVTIME_150,
  INA228_CONVTIME_280,
  INA228_CONVTIME_540,
  INA228_CONVTIME_1052,
  INA228_CONVTIME_2074,
  INA228_CONVTIME_4120                // 4120us
};

enum INA228_AVGCOUNT_t {
  INA228_AVGCOUNT_1,
  INA228_AVGCOUNT_4,
  INA228_AVGCOUNT_16,
  INA228_AVGCOUNT_64,
  INA228_AVGCOUNT_128,
  INA228_AVGCOUNT_256,
  INA228_AVGCOUNT_512,
  INA228_AVGCOUNT_1024
};

// INA228 Register Structures
typedef union {
  uint16_t u16;
  struct {
    unsigned          :4;             // reserved
    unsigned ADCRANGE :1;
    unsigned TEMPCOMP :1;
    unsigned CONVDLY  :8;
    unsigned RSTACC   :1;
    unsigned RST      :1;
  };
} INA228_CONFIG_t;

typedef union {
  uint16_t u16;
  struct {
    unsigned AVG    :3;
    unsigned CTCT   :3;
    unsigned VSHCT  :3;
    unsigned VBUSCT :3;
    unsigned MODE   :4;
  };
} INA228_ADC_CONFIG_t;

typedef union {
  uint16_t u16;
  struct {
    unsigned MEMSTAT  :1;
    unsigned CNVRF    :1;
    unsigned POL      :1;
    unsigned BUSUL    :1;
    unsigned BUSOL    :1;
    unsigned SHNTUL   :1;
    unsigned SHNTOL   :1;
    unsigned TMPOL    :1;
    unsigned          :1;             // bit 8 reserved
    unsigned MATHOF   :1;
    unsigned CHARGEOF :1;
    unsigned ENERGYOF :1;
    unsigned APOL     :1;
    unsigned SLOWALERT:1;
    unsigned CNVR     :1;
    unsigned ALATCH   :1;
  };
} INA228_DIAG_ALRT_t;

typedef struct {
  INA228_CONFIG_t reg_config;         // 16 bits
  INA228_ADC_CONFIG_t reg_adc_config; // 16 bits
  INA228_DIAG_ALRT_t reg_diag_alrt;   // 16 bits
  int16_t   reg_dietemp;              // 16 bit two's complement
  int32_t   reg_vshunt;               // 20 bit left aligned two's complement value
  int32_t   reg_vbus;                 // 20 bit left aligned two's complement value (always positive)
  int32_t   reg_current;              // 20 bit left aligned two's complement value
  uint32_t  reg_power;                // 24bit unsigned positive value
  uint64_t  reg_energy;               // 40 bit unsigned positive value
  int64_t   reg_charge;               // 40 bit two's complement value
} INA228_REGISTERS_t;

typedef struct {
  double shuntvoltage;                // Volts
  double busvoltage;                  // Volts
  double dietemp;                     // degC
  double current;                     // Amps
  double power;                       // Watts
  double energy;                      // Joules
  double charge;                      // Coulombs
} INA228_MEASUREMENTS_t;

// Function Prototypes
/*!
 *    @brief  Builds the 16-bit ADC_CONFIG register value from the input settings.
 *    @param ina228Mode The conversion mode, choosing what values will be acquired.
 *    @param busConvTime The conversion time for the bus voltage measurement in us
 *    @param shuntConvTime The conversion time for the shunt voltage measurement in us
 *    @param tempConvTime The conversion time for the temperature measurement in us
 *    @param avgCount The ADC sample averaging count
 *    @return The 16-bit register value created from the input settings
 */
uint16_t ina228AdcConfig(
  INA228_MODE_t ina228Mode, 
  INA228_CONVTIME_t busConvTime, 
  INA228_CONVTIME_t shuntConvTime, 
  INA228_CONVTIME_t tempConvTime, 
  INA228_AVGCOUNT_t avgCount
);

/*!
 *    @brief  Reads and calculates values from an INA228
 *    @param i2cDevice The I2C bus device (Adafruit_I2CDevice) to communicate with
 *    @param ina228Registers The raw register values struct to use (INA228_REGISTERS_t)
 *    @param ina228Measurements The calculated values struct to use (INA228_MEASUREMENTS_t)
 */
void ina228Processor(
  Adafruit_I2CDevice *i2cDevice, 
  INA228_REGISTERS_t *ina228Registers, 
  INA228_MEASUREMENTS_t *ina228Measurements
);

#endif // INA228_H
