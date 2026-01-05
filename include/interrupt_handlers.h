#ifndef INTERRUPT_HANDLERS_H
#define INTERRUPT_HANDLERS_H

#include <Arduino.h>

// Pin definitions
#define INA228_BATT_ALERT_PIN 12
#define INA228_LOAD_ALERT_PIN 13
#define ADS1219_DRDY_PIN 14

// Interrupt flags
extern volatile bool ina228BattAlert;
extern volatile bool ina228LoadAlert;
extern volatile bool ads1219DataReady;

// Setup interrupt pins and attach handlers
void interruptsInit();

// ISR handlers (not called directly)
void IRAM_ATTR ina228BattAlertHandler(void);
void IRAM_ATTR ina228LoadAlertHandler(void);
void IRAM_ATTR ads1219DataReadyHandler(void);

#endif
