#ifndef HISTORY_H
#define HISTORY_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

// History configuration
#define HISTORY_MINUTE_SAMPLES 60  // 60 seconds of 1-second averaged data
#define HISTORY_HOURLY_SAMPLES 24  // 24 hours of hourly averaged data

// Single data point for one source (battery, solar, or load)
typedef struct {
  float voltage;      // Average voltage
  float current;      // Average current
  float voltageMin;   // Minimum voltage in period
  float voltageMax;   // Maximum voltage in period
  float currentMin;   // Minimum current in period
  float currentMax;   // Maximum current in period
  // Power is calculated on-demand as voltage * current to save memory
} HISTORY_POINT_t;

// Complete measurement at one time point
typedef struct {
  HISTORY_POINT_t battery;
  HISTORY_POINT_t solar;
  HISTORY_POINT_t load;
  time_t timestamp;  // Unix timestamp for this data point
} HISTORY_MEASUREMENT_t;

// History storage arrays
extern HISTORY_MEASUREMENT_t historyMinute[HISTORY_MINUTE_SAMPLES];  // Last 60 seconds
extern HISTORY_MEASUREMENT_t historyHourly[HISTORY_HOURLY_SAMPLES];  // Last 24 hours
extern uint8_t historyMinuteIndex;   // Current write position in minute array (0-59)
extern uint8_t historyHourlyIndex;   // Current write position in hourly array (0-23)
extern bool historyMinuteFilled;     // Has minute array wrapped around at least once?
extern bool historyHourlyFilled;     // Has hourly array wrapped around at least once?

// Accumulator for current minute (collects samples to average)
extern HISTORY_POINT_t minuteAccumBattery;
extern HISTORY_POINT_t minuteAccumSolar;
extern HISTORY_POINT_t minuteAccumLoad;
extern uint16_t minuteAccumCount;
extern time_t lastMinuteSave;        // Timestamp of last minute save
extern time_t lastHourlySave;        // Timestamp of last hourly save

// Min/max tracking during accumulation period
extern float minuteMinBattV, minuteMaxBattV, minuteMinBattC, minuteMaxBattC;
extern float minuteMinSolarV, minuteMaxSolarV, minuteMinSolarC, minuteMaxSolarC;
extern float minuteMinLoadV, minuteMaxLoadV, minuteMinLoadC, minuteMaxLoadC;

// Initialize history system
void historyInit();

// Update with new data (call frequently with current readings)
void historyUpdate(double battV, double battC, double solarV, double solarC, double loadV, double loadC);

// Serve JSON data for history directly to web request
void serveHistoryJsonMinute(AsyncWebServerRequest* request);
void serveHistoryJsonHourly(AsyncWebServerRequest* request);

#endif
