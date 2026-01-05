#include "debug_config.h"
#include "web_server.h"
#include "wifi_manager.h"
#include "history.h"
#include "solar_monitor.h"
#include <LittleFS.h>

// Global web server instance
AsyncWebServer server(80);

void webServerInit() {
  // Initialize LittleFS
  if (!LittleFS.begin()) {
    #if DEBUG_SYSTEM
    Serial.println("LittleFS mount failed! Web pages will not be available.");
    #endif
  } else {
    #if DEBUG_SYSTEM
    Serial.println("LittleFS mounted successfully");
    #endif
  }

  // Serve main page from LittleFS (redirect to WiFi config in AP mode)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    if (isAPMode()) {
      request->redirect("/wifi");
    } else {
      request->send(LittleFS, "/index.html", "text/html");
    }
  });

  // Serve JSON data
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getJsonData());
  });

  // Serve history page from LittleFS
  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/history.html", "text/html");
  });

  // Serve history data endpoints
  server.on("/history-minute", HTTP_GET, [](AsyncWebServerRequest *request){
    serveHistoryJsonMinute(request);
  });

  server.on("/history-hourly", HTTP_GET, [](AsyncWebServerRequest *request){
    serveHistoryJsonHourly(request);
  });

  // Serve calibration page from LittleFS
  server.on("/calibration", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/calibration.html", "text/html");
  });

  // Get calibration data as JSON
  server.on("/cal-data", HTTP_GET, handleGetCalData);

  // Save calibration (POST with body)
  server.on("/cal-save", HTTP_POST,
    [](AsyncWebServerRequest *request){},
    NULL,
    handleSaveCalibration
  );

  // Reset calibration
  server.on("/cal-reset", HTTP_POST, handleResetCalibration);

  // ========== WiFi Configuration Endpoints ==========

  // Serve WiFi settings page from LittleFS
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/wifi.html", "text/html");
  });

  // Get WiFi status as JSON
  server.on("/wifi-status", HTTP_GET, [](AsyncWebServerRequest *request){
    JsonDocument doc;

    doc["connected"] = wifiConnected;
    doc["ssid"] = wifiConfig.configured ? wifiConfig.ssid : "";
    doc["ip"] = getLocalIP();
    doc["rssi"] = getRSSI();
    doc["state"] = getWiFiStatusString();
    doc["configured"] = wifiConfig.configured;
    doc["deviceName"] = wifiConfig.configured ? wifiConfig.deviceName : "SolarV1";

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // Scan for WiFi networks (async to prevent WDT in AP mode)
  server.on("/wifi-scan", HTTP_GET, [](AsyncWebServerRequest *request){
    // Check if a scan is already in progress
    int8_t scanStatus = WiFi.scanComplete();

    if (scanStatus == WIFI_SCAN_RUNNING) {
      // Scan already running, tell client to wait
      request->send(202, "application/json", "{\"status\":\"scanning\"}");
      return;
    }

    if (scanStatus >= 0) {
      // Scan complete, return results
      JsonDocument doc;
      JsonArray networks = doc.to<JsonArray>();

      for (int i = 0; i < scanStatus; i++) {
        JsonObject net = networks.add<JsonObject>();
        net["ssid"] = WiFi.SSID(i);
        net["rssi"] = WiFi.RSSI(i);
        net["encryption"] = (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? 0 : 1;
      }

      String response;
      serializeJson(doc, response);
      WiFi.scanDelete();
      request->send(200, "application/json", response);
    } else {
      // No scan in progress, start async scan
      WiFi.scanNetworks(true); // true = async
      request->send(202, "application/json", "{\"status\":\"scanning\"}");
    }
  });

  // Set WiFi credentials (POST with JSON body)
  server.on("/wifi-set", HTTP_POST,
    [](AsyncWebServerRequest *request){},
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
      // Parse JSON body
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, data, len);

      if (error) {
        request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
      }

      const char* ssid = doc["ssid"];
      const char* password = doc["password"];
      const char* deviceName = doc["deviceName"] | "SolarV1";

      if (!ssid || strlen(ssid) == 0) {
        request->send(400, "application/json", "{\"error\":\"SSID required\"}");
        return;
      }

      // Atomically update config (protect from interrupts like calibration does)
      noInterrupts();
      strlcpy(wifiConfig.ssid, ssid, sizeof(wifiConfig.ssid));
      strlcpy(wifiConfig.password, password ? password : "", sizeof(wifiConfig.password));
      strlcpy(wifiConfig.deviceName, deviceName, sizeof(wifiConfig.deviceName));
      wifiConfig.configured = true;
      interrupts();

      // Save to EEPROM (outside interrupt-disabled section)
      if (saveWiFiConfig()) {
        request->send(200, "application/json", "{\"success\":true,\"message\":\"Restarting...\"}");
        // Set flag instead of calling ESP.restart() directly
        setPendingRestart();
      } else {
        request->send(500, "application/json", "{\"error\":\"Failed to save\"}");
      }
    }
  );

  // Reset WiFi credentials
  server.on("/wifi-reset", HTTP_POST, [](AsyncWebServerRequest *request){
    clearWiFiConfig();
    request->send(200, "application/json", "{\"success\":true,\"message\":\"Credentials cleared. Restarting...\"}");
    // Set flag instead of calling ESP.restart() directly
    setPendingRestart();
  });

  // Serve static files (CSS, JS) from LittleFS
  server.serveStatic("/styles/", LittleFS, "/styles/").setCacheControl("max-age=86400");
  server.serveStatic("/scripts/", LittleFS, "/scripts/").setCacheControl("max-age=86400");

  // Handle favicon.ico (return 204 No Content to suppress browser error)
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(204);
  });

  // Handle 404 - redirect to /wifi in AP mode (captive portal behavior)
  server.onNotFound([](AsyncWebServerRequest *request){
    #if DEBUG_SYSTEM
    Serial.print("404 request: ");
    Serial.println(request->url());
    #endif

    // Captive portal behavior - redirect unknown URLs to WiFi config page
    if (isAPMode()) {
      request->redirect("/wifi");
    } else {
      // Suppress 404 for source map files (they're optional debug files)
      if (request->url().endsWith(".map")) {
        request->send(204);
      } else {
        request->send(404, "text/plain", "Not found");
      }
    }
  });

  server.begin();
  #if DEBUG_SYSTEM
  Serial.println("Web server started");
  #endif
}

String getJsonData() {
  // Atomically copy measurement data to prevent race conditions
  // with main loop updates (disable interrupts during copy)
  noInterrupts();
  INA228_MEASUREMENTS_t battCopy = ina228BattMeasurements;
  INA228_MEASUREMENTS_t loadCopy = ina228LoadMeasurements;
  double solarV = solarVoltage;
  double solarVPlus = solarPlusVoltage;
  double solarVMinus = solarMinusVoltage;
  double solarC = solarCurrent;
  double solarP = solarPower;
  time_t timestamp = getUnixTime();
  bool timeValid = isTimeSynced();
  interrupts();

  // Build JSON using fixed buffer to avoid heap fragmentation
  // (replaces 16+ String concatenation operations with single allocation)
  char buffer[512];
  snprintf(buffer, sizeof(buffer),
    "{\"time\":%lld,\"time_synced\":%s,"
    "\"battery\":{\"voltage\":%.3f,\"current\":%.3f,\"power\":%.3f},"
    "\"solar\":{\"voltage\":%.3f,\"voltage_plus\":%.3f,\"voltage_minus\":%.3f,\"current\":%.3f,\"power\":%.3f},"
    "\"load\":{\"voltage\":%.3f,\"current\":%.3f,\"power\":%.3f},"
    "\"errors\":%u}",
    (long long)timestamp,
    timeValid ? "true" : "false",
    battCopy.busvoltage, battCopy.current, battCopy.power,
    solarV, solarVPlus, solarVMinus, solarC, solarP,
    loadCopy.busvoltage, loadCopy.current, loadCopy.power,
    i2cErrorCount
  );

  return String(buffer);
}
