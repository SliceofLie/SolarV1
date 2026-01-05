#include "interrupt_handlers.h"

// Interrupt flags
volatile bool ina228BattAlert = false;
volatile bool ina228LoadAlert = false;
volatile bool ads1219DataReady = false;

void IRAM_ATTR ina228BattAlertHandler(void) {
  ina228BattAlert = true;
}

void IRAM_ATTR ina228LoadAlertHandler(void) {
  ina228LoadAlert = true;
}

void IRAM_ATTR ads1219DataReadyHandler(void) {
  ads1219DataReady = true;
}

void interruptsInit() {
  pinMode(INA228_BATT_ALERT_PIN, INPUT_PULLUP);
  pinMode(INA228_LOAD_ALERT_PIN, INPUT_PULLUP);
  pinMode(ADS1219_DRDY_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(INA228_BATT_ALERT_PIN), ina228BattAlertHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(INA228_LOAD_ALERT_PIN), ina228LoadAlertHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(ADS1219_DRDY_PIN), ads1219DataReadyHandler, FALLING);
}
