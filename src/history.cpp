#include "debug_config.h"
#include "history.h"
#include "wifi_manager.h"
#include <float.h>

// History storage arrays (initialized to zero by default)
HISTORY_MEASUREMENT_t historyMinute[HISTORY_MINUTE_SAMPLES];
HISTORY_MEASUREMENT_t historyHourly[HISTORY_HOURLY_SAMPLES];
uint8_t historyMinuteIndex = 0;
uint8_t historyHourlyIndex = 0;
bool historyMinuteFilled = false;
bool historyHourlyFilled = false;

// Current minute accumulators
HISTORY_POINT_t minuteAccumBattery = {0, 0, 0, 0, 0, 0};
HISTORY_POINT_t minuteAccumSolar = {0, 0, 0, 0, 0, 0};
HISTORY_POINT_t minuteAccumLoad = {0, 0, 0, 0, 0, 0};
uint16_t minuteAccumCount = 0;
time_t lastMinuteSave = 0;
time_t lastHourlySave = 0;

// Min/max tracking during accumulation period
float minuteMinBattV = FLT_MAX, minuteMaxBattV = -FLT_MAX;
float minuteMinBattC = FLT_MAX, minuteMaxBattC = -FLT_MAX;
float minuteMinSolarV = FLT_MAX, minuteMaxSolarV = -FLT_MAX;
float minuteMinSolarC = FLT_MAX, minuteMaxSolarC = -FLT_MAX;
float minuteMinLoadV = FLT_MAX, minuteMaxLoadV = -FLT_MAX;
float minuteMinLoadC = FLT_MAX, minuteMaxLoadC = -FLT_MAX;

void historyInit() {
  // Clear all arrays
  memset(historyMinute, 0, sizeof(historyMinute));
  memset(historyHourly, 0, sizeof(historyHourly));

  historyMinuteIndex = 0;
  historyHourlyIndex = 0;
  historyMinuteFilled = false;
  historyHourlyFilled = false;

  minuteAccumBattery = {0, 0, 0, 0, 0, 0};
  minuteAccumSolar = {0, 0, 0, 0, 0, 0};
  minuteAccumLoad = {0, 0, 0, 0, 0, 0};
  minuteAccumCount = 0;
  lastMinuteSave = 0;
  lastHourlySave = 0;

  // Reset min/max accumulators
  minuteMinBattV = FLT_MAX; minuteMaxBattV = -FLT_MAX;
  minuteMinBattC = FLT_MAX; minuteMaxBattC = -FLT_MAX;
  minuteMinSolarV = FLT_MAX; minuteMaxSolarV = -FLT_MAX;
  minuteMinSolarC = FLT_MAX; minuteMaxSolarC = -FLT_MAX;
  minuteMinLoadV = FLT_MAX; minuteMaxLoadV = -FLT_MAX;
  minuteMinLoadC = FLT_MAX; minuteMaxLoadC = -FLT_MAX;

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

  // Track min/max for battery
  if (battV < minuteMinBattV) minuteMinBattV = battV;
  if (battV > minuteMaxBattV) minuteMaxBattV = battV;
  if (battC < minuteMinBattC) minuteMinBattC = battC;
  if (battC > minuteMaxBattC) minuteMaxBattC = battC;

  // Track min/max for solar
  if (solarV < minuteMinSolarV) minuteMinSolarV = solarV;
  if (solarV > minuteMaxSolarV) minuteMaxSolarV = solarV;
  if (solarC < minuteMinSolarC) minuteMinSolarC = solarC;
  if (solarC > minuteMaxSolarC) minuteMaxSolarC = solarC;

  // Track min/max for load
  if (loadV < minuteMinLoadV) minuteMinLoadV = loadV;
  if (loadV > minuteMaxLoadV) minuteMaxLoadV = loadV;
  if (loadC < minuteMinLoadC) minuteMinLoadC = loadC;
  if (loadC > minuteMaxLoadC) minuteMaxLoadC = loadC;

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

      // Store min/max values
      historyMinute[historyMinuteIndex].battery.voltageMin = minuteMinBattV;
      historyMinute[historyMinuteIndex].battery.voltageMax = minuteMaxBattV;
      historyMinute[historyMinuteIndex].battery.currentMin = minuteMinBattC;
      historyMinute[historyMinuteIndex].battery.currentMax = minuteMaxBattC;
      historyMinute[historyMinuteIndex].solar.voltageMin = minuteMinSolarV;
      historyMinute[historyMinuteIndex].solar.voltageMax = minuteMaxSolarV;
      historyMinute[historyMinuteIndex].solar.currentMin = minuteMinSolarC;
      historyMinute[historyMinuteIndex].solar.currentMax = minuteMaxSolarC;
      historyMinute[historyMinuteIndex].load.voltageMin = minuteMinLoadV;
      historyMinute[historyMinuteIndex].load.voltageMax = minuteMaxLoadV;
      historyMinute[historyMinuteIndex].load.currentMin = minuteMinLoadC;
      historyMinute[historyMinuteIndex].load.currentMax = minuteMaxLoadC;

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
      minuteAccumBattery = {0, 0, 0, 0, 0, 0};
      minuteAccumSolar = {0, 0, 0, 0, 0, 0};
      minuteAccumLoad = {0, 0, 0, 0, 0, 0};
      minuteAccumCount = 0;
      lastMinuteSave = currentMinuteBoundary;

      // Reset min/max accumulators for next period
      minuteMinBattV = FLT_MAX; minuteMaxBattV = -FLT_MAX;
      minuteMinBattC = FLT_MAX; minuteMaxBattC = -FLT_MAX;
      minuteMinSolarV = FLT_MAX; minuteMaxSolarV = -FLT_MAX;
      minuteMinSolarC = FLT_MAX; minuteMaxSolarC = -FLT_MAX;
      minuteMinLoadV = FLT_MAX; minuteMaxLoadV = -FLT_MAX;
      minuteMinLoadC = FLT_MAX; minuteMaxLoadC = -FLT_MAX;
    }
  }

  // Check if we've crossed an hour boundary (minutes = 0, seconds = 0)
  time_t currentHourBoundary = (now / 3600) * 3600;
  if (currentHourBoundary > lastHourlySave) {
    // Average all 60 minute samples to create one hourly sample
    HISTORY_POINT_t hourlyBattery = {0, 0, FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX};
    HISTORY_POINT_t hourlySolar = {0, 0, FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX};
    HISTORY_POINT_t hourlyLoad = {0, 0, FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX};
    uint8_t validSamples = 0;

    // Determine how many samples to average and where to start
    uint8_t samplesToAverage = historyMinuteFilled ? HISTORY_MINUTE_SAMPLES : historyMinuteIndex;
    uint8_t startIndex = historyMinuteFilled ? historyMinuteIndex : 0;

    for (uint8_t i = 0; i < samplesToAverage; i++) {
      uint8_t index = (startIndex + i) % HISTORY_MINUTE_SAMPLES;
      if (historyMinute[index].timestamp > 0) {  // Only count valid samples
        // Accumulate averages
        hourlyBattery.voltage += historyMinute[index].battery.voltage;
        hourlyBattery.current += historyMinute[index].battery.current;
        hourlySolar.voltage += historyMinute[index].solar.voltage;
        hourlySolar.current += historyMinute[index].solar.current;
        hourlyLoad.voltage += historyMinute[index].load.voltage;
        hourlyLoad.current += historyMinute[index].load.current;

        // Track true min/max across all minute samples
        if (historyMinute[index].battery.voltageMin < hourlyBattery.voltageMin)
          hourlyBattery.voltageMin = historyMinute[index].battery.voltageMin;
        if (historyMinute[index].battery.voltageMax > hourlyBattery.voltageMax)
          hourlyBattery.voltageMax = historyMinute[index].battery.voltageMax;
        if (historyMinute[index].battery.currentMin < hourlyBattery.currentMin)
          hourlyBattery.currentMin = historyMinute[index].battery.currentMin;
        if (historyMinute[index].battery.currentMax > hourlyBattery.currentMax)
          hourlyBattery.currentMax = historyMinute[index].battery.currentMax;

        if (historyMinute[index].solar.voltageMin < hourlySolar.voltageMin)
          hourlySolar.voltageMin = historyMinute[index].solar.voltageMin;
        if (historyMinute[index].solar.voltageMax > hourlySolar.voltageMax)
          hourlySolar.voltageMax = historyMinute[index].solar.voltageMax;
        if (historyMinute[index].solar.currentMin < hourlySolar.currentMin)
          hourlySolar.currentMin = historyMinute[index].solar.currentMin;
        if (historyMinute[index].solar.currentMax > hourlySolar.currentMax)
          hourlySolar.currentMax = historyMinute[index].solar.currentMax;

        if (historyMinute[index].load.voltageMin < hourlyLoad.voltageMin)
          hourlyLoad.voltageMin = historyMinute[index].load.voltageMin;
        if (historyMinute[index].load.voltageMax > hourlyLoad.voltageMax)
          hourlyLoad.voltageMax = historyMinute[index].load.voltageMax;
        if (historyMinute[index].load.currentMin < hourlyLoad.currentMin)
          hourlyLoad.currentMin = historyMinute[index].load.currentMin;
        if (historyMinute[index].load.currentMax > hourlyLoad.currentMax)
          hourlyLoad.currentMax = historyMinute[index].load.currentMax;

        validSamples++;
      }
    }

    if (validSamples > 0) {
      // Store averages
      historyHourly[historyHourlyIndex].battery.voltage = hourlyBattery.voltage / validSamples;
      historyHourly[historyHourlyIndex].battery.current = hourlyBattery.current / validSamples;
      historyHourly[historyHourlyIndex].solar.voltage = hourlySolar.voltage / validSamples;
      historyHourly[historyHourlyIndex].solar.current = hourlySolar.current / validSamples;
      historyHourly[historyHourlyIndex].load.voltage = hourlyLoad.voltage / validSamples;
      historyHourly[historyHourlyIndex].load.current = hourlyLoad.current / validSamples;

      // Store min/max values (true min/max across all minute samples in the hour)
      historyHourly[historyHourlyIndex].battery.voltageMin = hourlyBattery.voltageMin;
      historyHourly[historyHourlyIndex].battery.voltageMax = hourlyBattery.voltageMax;
      historyHourly[historyHourlyIndex].battery.currentMin = hourlyBattery.currentMin;
      historyHourly[historyHourlyIndex].battery.currentMax = hourlyBattery.currentMax;
      historyHourly[historyHourlyIndex].solar.voltageMin = hourlySolar.voltageMin;
      historyHourly[historyHourlyIndex].solar.voltageMax = hourlySolar.voltageMax;
      historyHourly[historyHourlyIndex].solar.currentMin = hourlySolar.currentMin;
      historyHourly[historyHourlyIndex].solar.currentMax = hourlySolar.currentMax;
      historyHourly[historyHourlyIndex].load.voltageMin = hourlyLoad.voltageMin;
      historyHourly[historyHourlyIndex].load.voltageMax = hourlyLoad.voltageMax;
      historyHourly[historyHourlyIndex].load.currentMin = hourlyLoad.currentMin;
      historyHourly[historyHourlyIndex].load.currentMax = hourlyLoad.currentMax;

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

    // Copy min/max values and validate (fallback to average if invalid)
    float battVMin = historyMinute[idx].battery.voltageMin;
    float battVMax = historyMinute[idx].battery.voltageMax;
    float battCMin = historyMinute[idx].battery.currentMin;
    float battCMax = historyMinute[idx].battery.currentMax;
    if (isnan(battVMin) || isinf(battVMin) || battVMin > 1e6) battVMin = battV;
    if (isnan(battVMax) || isinf(battVMax) || battVMax < -1e6) battVMax = battV;
    if (isnan(battCMin) || isinf(battCMin) || battCMin > 1e6) battCMin = battC;
    if (isnan(battCMax) || isinf(battCMax) || battCMax < -1e6) battCMax = battC;

    float solarVMin = historyMinute[idx].solar.voltageMin;
    float solarVMax = historyMinute[idx].solar.voltageMax;
    float solarCMin = historyMinute[idx].solar.currentMin;
    float solarCMax = historyMinute[idx].solar.currentMax;
    if (isnan(solarVMin) || isinf(solarVMin) || solarVMin > 1e6) solarVMin = solarV;
    if (isnan(solarVMax) || isinf(solarVMax) || solarVMax < -1e6) solarVMax = solarV;
    if (isnan(solarCMin) || isinf(solarCMin) || solarCMin > 1e6) solarCMin = solarC;
    if (isnan(solarCMax) || isinf(solarCMax) || solarCMax < -1e6) solarCMax = solarC;

    float loadVMin = historyMinute[idx].load.voltageMin;
    float loadVMax = historyMinute[idx].load.voltageMax;
    float loadCMin = historyMinute[idx].load.currentMin;
    float loadCMax = historyMinute[idx].load.currentMax;
    if (isnan(loadVMin) || isinf(loadVMin) || loadVMin > 1e6) loadVMin = loadV;
    if (isnan(loadVMax) || isinf(loadVMax) || loadVMax < -1e6) loadVMax = loadV;
    if (isnan(loadCMin) || isinf(loadCMin) || loadCMin > 1e6) loadCMin = loadC;
    if (isnan(loadCMax) || isinf(loadCMax) || loadCMax < -1e6) loadCMax = loadC;

    // Create JSON object for this sample
    JsonObject sample = root.add<JsonObject>();
    sample["t"] = (long long)historyMinute[idx].timestamp;

    JsonObject battery = sample["bt"].to<JsonObject>();
    battery["v"] = round(battV * 1000.0) / 1000.0;
    battery["i"] = round(battC * 1000.0) / 1000.0;
    battery["vn"] = round(battVMin * 1000.0) / 1000.0;
    battery["vx"] = round(battVMax * 1000.0) / 1000.0;
    battery["in"] = round(battCMin * 1000.0) / 1000.0;
    battery["ix"] = round(battCMax * 1000.0) / 1000.0;

    JsonObject solar = sample["pv"].to<JsonObject>();
    solar["v"] = round(solarV * 1000.0) / 1000.0;
    solar["i"] = round(solarC * 1000.0) / 1000.0;
    solar["vn"] = round(solarVMin * 1000.0) / 1000.0;
    solar["vx"] = round(solarVMax * 1000.0) / 1000.0;
    solar["in"] = round(solarCMin * 1000.0) / 1000.0;
    solar["ix"] = round(solarCMax * 1000.0) / 1000.0;

    JsonObject load = sample["ld"].to<JsonObject>();
    load["v"] = round(loadV * 1000.0) / 1000.0;
    load["i"] = round(loadC * 1000.0) / 1000.0;
    load["vn"] = round(loadVMin * 1000.0) / 1000.0;
    load["vx"] = round(loadVMax * 1000.0) / 1000.0;
    load["in"] = round(loadCMin * 1000.0) / 1000.0;
    load["ix"] = round(loadCMax * 1000.0) / 1000.0;
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

    // Copy min/max values and validate (fallback to average if invalid)
    float battVMin = historyHourly[idx].battery.voltageMin;
    float battVMax = historyHourly[idx].battery.voltageMax;
    float battCMin = historyHourly[idx].battery.currentMin;
    float battCMax = historyHourly[idx].battery.currentMax;
    if (isnan(battVMin) || isinf(battVMin) || battVMin > 1e6) battVMin = battV;
    if (isnan(battVMax) || isinf(battVMax) || battVMax < -1e6) battVMax = battV;
    if (isnan(battCMin) || isinf(battCMin) || battCMin > 1e6) battCMin = battC;
    if (isnan(battCMax) || isinf(battCMax) || battCMax < -1e6) battCMax = battC;

    float solarVMin = historyHourly[idx].solar.voltageMin;
    float solarVMax = historyHourly[idx].solar.voltageMax;
    float solarCMin = historyHourly[idx].solar.currentMin;
    float solarCMax = historyHourly[idx].solar.currentMax;
    if (isnan(solarVMin) || isinf(solarVMin) || solarVMin > 1e6) solarVMin = solarV;
    if (isnan(solarVMax) || isinf(solarVMax) || solarVMax < -1e6) solarVMax = solarV;
    if (isnan(solarCMin) || isinf(solarCMin) || solarCMin > 1e6) solarCMin = solarC;
    if (isnan(solarCMax) || isinf(solarCMax) || solarCMax < -1e6) solarCMax = solarC;

    float loadVMin = historyHourly[idx].load.voltageMin;
    float loadVMax = historyHourly[idx].load.voltageMax;
    float loadCMin = historyHourly[idx].load.currentMin;
    float loadCMax = historyHourly[idx].load.currentMax;
    if (isnan(loadVMin) || isinf(loadVMin) || loadVMin > 1e6) loadVMin = loadV;
    if (isnan(loadVMax) || isinf(loadVMax) || loadVMax < -1e6) loadVMax = loadV;
    if (isnan(loadCMin) || isinf(loadCMin) || loadCMin > 1e6) loadCMin = loadC;
    if (isnan(loadCMax) || isinf(loadCMax) || loadCMax < -1e6) loadCMax = loadC;

    // Create JSON object for this sample
    JsonObject sample = root.add<JsonObject>();
    sample["t"] = (long long)historyHourly[idx].timestamp;

    JsonObject battery = sample["bt"].to<JsonObject>();
    battery["v"] = round(battV * 1000.0) / 1000.0;
    battery["i"] = round(battC * 1000.0) / 1000.0;
    battery["vn"] = round(battVMin * 1000.0) / 1000.0;
    battery["vx"] = round(battVMax * 1000.0) / 1000.0;
    battery["in"] = round(battCMin * 1000.0) / 1000.0;
    battery["ix"] = round(battCMax * 1000.0) / 1000.0;

    JsonObject solar = sample["pv"].to<JsonObject>();
    solar["v"] = round(solarV * 1000.0) / 1000.0;
    solar["i"] = round(solarC * 1000.0) / 1000.0;
    solar["vn"] = round(solarVMin * 1000.0) / 1000.0;
    solar["vx"] = round(solarVMax * 1000.0) / 1000.0;
    solar["in"] = round(solarCMin * 1000.0) / 1000.0;
    solar["ix"] = round(solarCMax * 1000.0) / 1000.0;

    JsonObject load = sample["ld"].to<JsonObject>();
    load["v"] = round(loadV * 1000.0) / 1000.0;
    load["i"] = round(loadC * 1000.0) / 1000.0;
    load["vn"] = round(loadVMin * 1000.0) / 1000.0;
    load["vx"] = round(loadVMax * 1000.0) / 1000.0;
    load["in"] = round(loadCMin * 1000.0) / 1000.0;
    load["ix"] = round(loadCMax * 1000.0) / 1000.0;
  }

  #if DEBUG_HISTORY
  Serial.printf("Served /history-hourly (ArduinoJson)\n");
  #endif

  response->setLength();
  request->send(response);
}
