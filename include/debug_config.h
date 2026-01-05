#ifndef DEBUG_CONFIG_H
#define DEBUG_CONFIG_H

// Debug output control flags (set to 1 to enable, 0 to disable)
// These flags use preprocessor conditionals - disabled categories are
// removed at compile time with no runtime or memory overhead

#define DEBUG_SYSTEM      1  // Startup, initialization, web server
#define DEBUG_INA228      0  // INA228 sensor data and alerts
#define DEBUG_ADS1219     0  // ADS1219 sensor data and calibration (init, errors, calibration)
#define DEBUG_WIFI        1  // WiFi connection, NTP, OTA updates
#define DEBUG_CALIBRATION 1  // EEPROM calibration operations
#define DEBUG_HISTORY     1  // Historical data logging
#define DEBUG_HEAP        1  // Heap memory statistics for leak detection

// ADS1219 verbose channel measurement debug (very high frequency output)
#define ADS1219_DEBUG_CHANNEL_SWITCH 0  // Channel switching and measurement details

// Gain switching diagnostics
#define DEBUG_GAIN_SWITCHING 0  // Automatic gain/range switching for INA228 and ADS1219

#endif
