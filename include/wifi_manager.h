#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <time.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <EEPROM.h>

// WiFi configuration - keeping hardcoded as fallback during transition
#define WIFI_AP1_SSID "TP-Link_IoT_79EA"
#define WIFI_AP1_PASS "21189067"
#define WIFI_RECONNECT_INTERVAL_MS 30000
#define WIFI_CONNECTION_TIMEOUT_MS 15000
#define WIFI_MAX_RETRY_BEFORE_AP 5

// AP Mode configuration
#define AP_SSID_PREFIX "SolarV1-"
#define AP_PASSWORD "solar123"
#define AP_TIMEOUT_MS 0  // 0 = no timeout (stay until configured)
#define CONFIG_PORTAL_HOSTNAME "solarsetup"
#define DNS_PORT 53

// EEPROM configuration storage layout (persists through filesystem uploads)
// NOTE: Calibration data occupies bytes 0-55, WiFi config starts at 64
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xCAFEBEEF
#define WIFI_EEPROM_START 64        // Start after calibration data (0-55) with padding
#define EEPROM_ADDR_MAGIC 64        // 4 bytes: Magic validation header
#define EEPROM_ADDR_SSID 68         // 33 bytes: SSID
#define EEPROM_ADDR_PASS 101        // 64 bytes: Encrypted password
#define EEPROM_ADDR_DEVICE 165      // 32 bytes: Device name
#define EEPROM_ADDR_CONFIGURED 197  // 1 byte: Configured flag
#define EEPROM_ADDR_CHECKSUM 198    // 1 byte: Simple XOR checksum
// WiFi config ends at byte 198 (total: 135 bytes used from 64-198)

// WiFi Manager States
enum WiFiManagerState {
  WIFI_STATE_INIT,
  WIFI_STATE_CONNECTING,
  WIFI_STATE_CONNECTED,
  WIFI_STATE_DISCONNECTED,
  WIFI_STATE_AP_MODE,
  WIFI_STATE_PORTAL_ACTIVE
};

// WiFi configuration structure
typedef struct {
  char ssid[33];          // Max SSID length is 32 + null terminator
  char password[64];      // Max WPA2 password length is 63 + null terminator
  char deviceName[32];    // mDNS hostname
  bool configured;        // Has credentials been set?
  uint8_t failCount;      // Connection failure counter
} WiFiConfig_t;

// NTP configuration
#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "time.nist.gov"
#define NTP_TIMEZONE "EST5EDT,M3.2.0,M11.1.0"  // Eastern Time with automatic DST
#define NTP_RESYNC_INTERVAL_MS 600000  // Re-sync NTP every 10 minutes

// WiFi and time sync state tracking
extern bool wifiConnected;
extern bool timeSynced;
extern WiFiManagerState wifiState;
extern WiFiConfig_t wifiConfig;

// Core functions
void wifiInit();
void otaInit(const char* hostname);
void wifiUpdate();
bool isTimeSynced();
time_t getUnixTime();

// Configuration management functions
bool loadWiFiConfig();
bool saveWiFiConfig();
void clearWiFiConfig();
void setPendingRestart();
void encryptPassword(const char* input, char* output, size_t outputSize);
void decryptPassword(const char* input, char* output, size_t outputSize);

// AP Mode and portal functions
void setupCaptivePortal();
void stopCaptivePortal();
void startAPMode();
void stopAPMode();
bool isAPMode();
String getAPSSID();

// Serial console functions
void serialConsoleUpdate();
void serialPrintMenu();
void serialHandleCommand(String cmd);

// Status and diagnostic functions
String getWiFiStatusString();
int getRSSI();
String getLocalIP();

#endif
