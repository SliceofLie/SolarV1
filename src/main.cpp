#include <Arduino.h>

// Module includes
#include "debug_config.h"
#include "calibration.h"
#include "wifi_manager.h"
#include "interrupt_handlers.h"
#include "ina228_driver.h"
#include "solar_monitor.h"
#include "web_server.h"
#include "history.h"

// Global I2C buffers (shared across all I2C devices)
uint8_t ina228WriteBuffer[8];
uint8_t ina228ReadBuffer[8];
uint8_t ads1219WriteBuffer[8];
uint8_t ads1219ReadBuffer[8];

// Global error tracking (shared across modules)
uint32_t i2cErrorCount = 0;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Serial initialization
  Serial.begin(74880);
  #if DEBUG_SYSTEM
  Serial.println("\n\nSolar Power Monitor V1");
  #endif

  // Initialize calibration system
  calibrationInit(512);
  if (!calibrationLoad()) {
    calibrationLoadDefaults();
    #if DEBUG_SYSTEM
    Serial.println("Using default calibration");
    #endif
  } else {
    #if DEBUG_SYSTEM
    Serial.println("Calibration loaded from EEPROM");
    #endif
  }

  // Network setup
  wifiInit();
  otaInit("espSolar");

  // Hardware interrupts
  interruptsInit();

  // I2C sensors
  if (!ina228Init()) {
    #if DEBUG_SYSTEM
    Serial.println("INA228 Init Fail! Halting");
    #endif
    while (1) yield();
  }

  if (!solarMonitorInit()) {
    #if DEBUG_SYSTEM
    Serial.println("Solar Monitor Init Fail! Halting");
    #endif
    while (1) yield();
  }

  // Web server
  webServerInit();

  // History tracking
  historyInit();

  #if DEBUG_SYSTEM
  Serial.println("Setup complete!");
  #endif
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Handle INA228 Battery Alert (atomic flag check/clear)
  noInterrupts();
  bool battAlert = ina228BattAlert;
  ina228BattAlert = false;
  interrupts();

  if (battAlert) {
    ina228ProcessBatteryAlert();
    yield();
  }

  // Handle INA228 Load Alert (atomic flag check/clear)
  noInterrupts();
  bool loadAlert = ina228LoadAlert;
  ina228LoadAlert = false;
  interrupts();

  if (loadAlert) {
    ina228ProcessLoadAlert();
    yield();
  }

  // Handle ADS1219 Data Ready (atomic flag check/clear)
  noInterrupts();
  bool adsReady = ads1219DataReady;
  ads1219DataReady = false;
  interrupts();

  if (adsReady) {
    ads1219ProcessDataReady();
    yield();
  }

  // WiFi and OTA management
  wifiUpdate();

  // Update history with current readings (overflow-safe millis check)
  static unsigned long nextHistoryUpdate = 0;
  unsigned long now = millis();
  if (now >= nextHistoryUpdate || nextHistoryUpdate == 0) {
    historyUpdate(
      ina228BattMeasurements.busvoltage,
      ina228BattMeasurements.current,
      solarVoltage,
      solarCurrent,
      ina228LoadMeasurements.busvoltage,
      ina228LoadMeasurements.current
    );
    nextHistoryUpdate = now + 1000;
  }

  // Periodic heap monitoring for leak detection (overflow-safe millis check)
  #if DEBUG_HEAP
  static unsigned long nextHeapReport = 0;
  if (now >= nextHeapReport || nextHeapReport == 0) {
    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t maxFreeBlock = ESP.getMaxFreeBlockSize();
    uint8_t fragmentation = ESP.getHeapFragmentation();

    Serial.printf("Heap: Free=%u bytes, MaxBlock=%u bytes, Frag=%u%%\n",
                  freeHeap, maxFreeBlock, fragmentation);

    nextHeapReport = now + 10000;  // Report every 10 seconds
  }
  #endif

  // Regular yield for background tasks
  yield();
}
