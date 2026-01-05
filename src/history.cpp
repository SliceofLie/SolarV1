#include "debug_config.h"
#include "history.h"
#include "wifi_manager.h"

// History storage arrays (initialized to zero by default)
HISTORY_MEASUREMENT_t historyMinute[HISTORY_MINUTE_SAMPLES];
HISTORY_MEASUREMENT_t historyHourly[HISTORY_HOURLY_SAMPLES];
uint8_t historyMinuteIndex = 0;
uint8_t historyHourlyIndex = 0;
bool historyMinuteFilled = false;
bool historyHourlyFilled = false;

// Current minute accumulators
HISTORY_POINT_t minuteAccumBattery = {0, 0};
HISTORY_POINT_t minuteAccumSolar = {0, 0};
HISTORY_POINT_t minuteAccumLoad = {0, 0};
uint16_t minuteAccumCount = 0;
time_t lastMinuteSave = 0;
time_t lastHourlySave = 0;

void historyInit() {
  // Clear all arrays
  memset(historyMinute, 0, sizeof(historyMinute));
  memset(historyHourly, 0, sizeof(historyHourly));

  historyMinuteIndex = 0;
  historyHourlyIndex = 0;
  historyMinuteFilled = false;
  historyHourlyFilled = false;

  minuteAccumBattery = {0, 0};
  minuteAccumSolar = {0, 0};
  minuteAccumLoad = {0, 0};
  minuteAccumCount = 0;
  lastMinuteSave = 0;
  lastHourlySave = 0;

  #if DEBUG_HISTORY
  Serial.println("History system initialized");
  #endif
}

void historyUpdate(double battV, double battC, double solarV, double solarC, double loadV, double loadC) {
  // Only collect history if time is synced
  if (!isTimeSynced()) {
    return;
  }

  time_t now = getUnixTime();

  // Initialize timestamps on first run - align to current minute/hour boundaries
  if (lastMinuteSave == 0) {
    lastMinuteSave = (now / 60) * 60;  // Round down to minute boundary
    lastHourlySave = (now / 3600) * 3600;  // Round down to hour boundary
  }

  // Accumulate current readings
  minuteAccumBattery.voltage += battV;
  minuteAccumBattery.current += battC;
  minuteAccumSolar.voltage += solarV;
  minuteAccumSolar.current += solarC;
  minuteAccumLoad.voltage += loadV;
  minuteAccumLoad.current += loadC;
  minuteAccumCount++;

  // Sanity check: prevent overflow and detect excessive calls
  if (minuteAccumCount > 65000) {
    #if DEBUG_HISTORY
    Serial.printf("WARNING: minuteAccumCount overflow: %u\n", minuteAccumCount);
    #endif
    return;  // Skip this update
  }

  // Check if we've crossed a minute boundary (seconds = 0)
  time_t currentMinuteBoundary = (now / 60) * 60;
  if (currentMinuteBoundary > lastMinuteSave) {
    // Calculate averages
    if (minuteAccumCount > 0) {
      historyMinute[historyMinuteIndex].battery.voltage = minuteAccumBattery.voltage / minuteAccumCount;
      historyMinute[historyMinuteIndex].battery.current = minuteAccumBattery.current / minuteAccumCount;
      historyMinute[historyMinuteIndex].solar.voltage = minuteAccumSolar.voltage / minuteAccumCount;
      historyMinute[historyMinuteIndex].solar.current = minuteAccumSolar.current / minuteAccumCount;
      historyMinute[historyMinuteIndex].load.voltage = minuteAccumLoad.voltage / minuteAccumCount;
      historyMinute[historyMinuteIndex].load.current = minuteAccumLoad.current / minuteAccumCount;
      // Save timestamp aligned to the minute boundary
      historyMinute[historyMinuteIndex].timestamp = currentMinuteBoundary;

      #if DEBUG_HISTORY
      Serial.printf("History: Saved minute %d (avg of %d samples) at %lld\n",
                    historyMinuteIndex, minuteAccumCount, (long long)currentMinuteBoundary);
      Serial.printf("  Battery: %.2fV / %.3fA, Solar: %.2fV / %.3fA, Load: %.2fV / %.3fA\n",
                    historyMinute[historyMinuteIndex].battery.voltage,
                    historyMinute[historyMinuteIndex].battery.current,
                    historyMinute[historyMinuteIndex].solar.voltage,
                    historyMinute[historyMinuteIndex].solar.current,
                    historyMinute[historyMinuteIndex].load.voltage,
                    historyMinute[historyMinuteIndex].load.current);
      #endif

      // Advance minute index
      historyMinuteIndex = (historyMinuteIndex + 1) % HISTORY_MINUTE_SAMPLES;
      if (historyMinuteIndex == 0) {
        historyMinuteFilled = true;
      }

      // Reset accumulators
      minuteAccumBattery = {0, 0};
      minuteAccumSolar = {0, 0};
      minuteAccumLoad = {0, 0};
      minuteAccumCount = 0;
      lastMinuteSave = currentMinuteBoundary;
    }
  }

  // Check if we've crossed an hour boundary (minutes = 0, seconds = 0)
  time_t currentHourBoundary = (now / 3600) * 3600;
  if (currentHourBoundary > lastHourlySave) {
    // Average all 60 minute samples to create one hourly sample
    HISTORY_POINT_t hourlyBattery = {0, 0};
    HISTORY_POINT_t hourlySolar = {0, 0};
    HISTORY_POINT_t hourlyLoad = {0, 0};
    uint8_t validSamples = 0;

    // Determine how many samples to average and where to start
    uint8_t samplesToAverage = historyMinuteFilled ? HISTORY_MINUTE_SAMPLES : historyMinuteIndex;
    uint8_t startIndex = historyMinuteFilled ? historyMinuteIndex : 0;

    for (uint8_t i = 0; i < samplesToAverage; i++) {
      uint8_t index = (startIndex + i) % HISTORY_MINUTE_SAMPLES;
      if (historyMinute[index].timestamp > 0) {  // Only count valid samples
        hourlyBattery.voltage += historyMinute[index].battery.voltage;
        hourlyBattery.current += historyMinute[index].battery.current;
        hourlySolar.voltage += historyMinute[index].solar.voltage;
        hourlySolar.current += historyMinute[index].solar.current;
        hourlyLoad.voltage += historyMinute[index].load.voltage;
        hourlyLoad.current += historyMinute[index].load.current;
        validSamples++;
      }
    }

    if (validSamples > 0) {
      historyHourly[historyHourlyIndex].battery.voltage = hourlyBattery.voltage / validSamples;
      historyHourly[historyHourlyIndex].battery.current = hourlyBattery.current / validSamples;
      historyHourly[historyHourlyIndex].solar.voltage = hourlySolar.voltage / validSamples;
      historyHourly[historyHourlyIndex].solar.current = hourlySolar.current / validSamples;
      historyHourly[historyHourlyIndex].load.voltage = hourlyLoad.voltage / validSamples;
      historyHourly[historyHourlyIndex].load.current = hourlyLoad.current / validSamples;
      // Save timestamp aligned to the hour boundary
      historyHourly[historyHourlyIndex].timestamp = currentHourBoundary;

      #if DEBUG_HISTORY
      Serial.printf("History: Saved hourly %d (avg of %d minute samples) at %lld\n",
                    historyHourlyIndex, validSamples, (long long)currentHourBoundary);
      #endif

      // Advance hourly index
      historyHourlyIndex = (historyHourlyIndex + 1) % HISTORY_HOURLY_SAMPLES;
      if (historyHourlyIndex == 0) {
        historyHourlyFilled = true;
      }

      lastHourlySave = currentHourBoundary;
    }
  }
}

void serveHistoryJsonMinute(AsyncWebServerRequest* request) {
  AsyncJsonResponse* response = new AsyncJsonResponse(true);
  JsonArray root = response->getRoot().to<JsonArray>();

  // Determine iteration parameters
  uint8_t samplesToReturn = historyMinuteFilled ? HISTORY_MINUTE_SAMPLES : historyMinuteIndex;
  uint8_t startIndex = historyMinuteFilled ? historyMinuteIndex : 0;

  // Iterate through valid samples
  for (uint8_t i = 0; i < samplesToReturn; i++) {
    uint8_t idx = (startIndex + i) % HISTORY_MINUTE_SAMPLES;

    // Skip invalid timestamps
    if (historyMinute[idx].timestamp == 0) continue;

    // Copy float values
    float battV = historyMinute[idx].battery.voltage;
    float battC = historyMinute[idx].battery.current;
    float solarV = historyMinute[idx].solar.voltage;
    float solarC = historyMinute[idx].solar.current;
    float loadV = historyMinute[idx].load.voltage;
    float loadC = historyMinute[idx].load.current;

    // Skip NaN/Inf
    if (isnan(battV) || isinf(battV) || isnan(battC) || isinf(battC) ||
        isnan(solarV) || isinf(solarV) || isnan(solarC) || isinf(solarC) ||
        isnan(loadV) || isinf(loadV) || isnan(loadC) || isinf(loadC)) {
      continue;
    }

    // Create JSON object for this sample
    JsonObject sample = root.add<JsonObject>();
    sample["t"] = (long long)historyMinute[idx].timestamp;

    JsonObject battery = sample["bt"].to<JsonObject>();
    battery["v"] = round(battV * 1000.0) / 1000.0;
    battery["i"] = round(battC * 1000.0) / 1000.0;

    JsonObject solar = sample["pv"].to<JsonObject>();
    solar["v"] = round(solarV * 1000.0) / 1000.0;
    solar["i"] = round(solarC * 1000.0) / 1000.0;

    JsonObject load = sample["ld"].to<JsonObject>();
    load["v"] = round(loadV * 1000.0) / 1000.0;
    load["i"] = round(loadC * 1000.0) / 1000.0;
  }

  #if DEBUG_HISTORY
  Serial.printf("Served /history-minute (ArduinoJson)\n");
  #endif

  response->setLength();
  request->send(response);
}

void serveHistoryJsonHourly(AsyncWebServerRequest* request) {
  AsyncJsonResponse* response = new AsyncJsonResponse(true);
  JsonArray root = response->getRoot().to<JsonArray>();

  // Determine iteration parameters
  uint8_t samplesToReturn = historyHourlyFilled ? HISTORY_HOURLY_SAMPLES : historyHourlyIndex;
  uint8_t startIndex = historyHourlyFilled ? historyHourlyIndex : 0;

  // Iterate through valid samples
  for (uint8_t i = 0; i < samplesToReturn; i++) {
    uint8_t idx = (startIndex + i) % HISTORY_HOURLY_SAMPLES;

    // Skip invalid timestamps
    if (historyHourly[idx].timestamp == 0) continue;

    // Copy float values
    float battV = historyHourly[idx].battery.voltage;
    float battC = historyHourly[idx].battery.current;
    float solarV = historyHourly[idx].solar.voltage;
    float solarC = historyHourly[idx].solar.current;
    float loadV = historyHourly[idx].load.voltage;
    float loadC = historyHourly[idx].load.current;

    // Skip NaN/Inf
    if (isnan(battV) || isinf(battV) || isnan(battC) || isinf(battC) ||
        isnan(solarV) || isinf(solarV) || isnan(solarC) || isinf(solarC) ||
        isnan(loadV) || isinf(loadV) || isnan(loadC) || isinf(loadC)) {
      continue;
    }

    // Create JSON object for this sample
    JsonObject sample = root.add<JsonObject>();
    sample["t"] = (long long)historyHourly[idx].timestamp;

    JsonObject battery = sample["bt"].to<JsonObject>();
    battery["v"] = round(battV * 1000.0) / 1000.0;
    battery["i"] = round(battC * 1000.0) / 1000.0;

    JsonObject solar = sample["pv"].to<JsonObject>();
    solar["v"] = round(solarV * 1000.0) / 1000.0;
    solar["i"] = round(solarC * 1000.0) / 1000.0;

    JsonObject load = sample["ld"].to<JsonObject>();
    load["v"] = round(loadV * 1000.0) / 1000.0;
    load["i"] = round(loadC * 1000.0) / 1000.0;
  }

  #if DEBUG_HISTORY
  Serial.printf("Served /history-hourly (ArduinoJson)\n");
  #endif

  response->setLength();
  request->send(response);
}
