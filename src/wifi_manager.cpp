#include "debug_config.h"
#include "wifi_manager.h"

// WiFi and time sync state tracking
bool wifiConnected = false;
bool timeSynced = false;
static unsigned long lastNtpSync = 0;

// New WiFi manager state
WiFiManagerState wifiState = WIFI_STATE_INIT;
WiFiConfig_t wifiConfig;
bool usingHardcodedDefaults = false;  // Track if using hardcoded defaults
static uint8_t connectionFailCount = 0;
static unsigned long lastConnectionAttempt = 0;
static bool pendingRestart = false;

// DNS server for captive portal
static DNSServer* dnsServer = nullptr;

// WiFi event handlers (must be stored to prevent automatic unsubscription)
static WiFiEventHandler wifiConnectHandler;
static WiFiEventHandler wifiDisconnectHandler;

// WiFi event handler for connection
void onWiFiConnected(const WiFiEventStationModeGotIP& event) {
  wifiConnected = true;
  wifiState = WIFI_STATE_CONNECTED;
  connectionFailCount = 0;
  wifiConfig.failCount = 0;

  #if DEBUG_WIFI
  Serial.print("WiFi connected: ");
  Serial.print(WiFi.SSID());
  Serial.print(" IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(" Gateway: ");
  Serial.print(WiFi.gatewayIP());
  Serial.print(" RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  #endif

  // If we connected using hardcoded defaults, save them to EEPROM
  if (usingHardcodedDefaults) {
    #if DEBUG_WIFI
    Serial.println("Hardcoded defaults worked - saving to EEPROM");
    #endif
    wifiConfig.configured = true;
    saveWiFiConfig();
    usingHardcodedDefaults = false;  // Clear the flag
  }

  // Start NTP sync with automatic DST handling
  configTime(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2);
  lastNtpSync = millis();
  #if DEBUG_WIFI
  Serial.println("NTP sync started (Eastern Time)");
  #endif

  ArduinoOTA.begin();
}

// WiFi event handler for disconnection
void onWiFiDisconnected(const WiFiEventStationModeDisconnected& event) {
  wifiConnected = false;
  timeSynced = false;

  // Don't change state if we're actively connecting - let timeout logic handle it
  // This allows connectionFailCount to increment properly in wifiUpdate()
  if (wifiState != WIFI_STATE_CONNECTING) {
    wifiState = WIFI_STATE_DISCONNECTED;
  }

  #if DEBUG_WIFI
  Serial.print("WiFi disconnected - Reason: ");
  Serial.println(event.reason);
  #endif

  // Don't call ArduinoOTA.end() - it can cause crashes
  // OTA will be reinitialized on next connection
}

// ========== PASSWORD ENCRYPTION FUNCTIONS ==========

void encryptPassword(const char* input, char* output, size_t outputSize) {
  uint32_t key = ESP.getChipId();
  size_t len = strlen(input);
  if (len >= outputSize) len = outputSize - 1;

  for (size_t i = 0; i < len; i++) {
    output[i] = input[i] ^ ((key >> ((i % 4) * 8)) & 0xFF);
  }
  output[len] = '\0';
}

void decryptPassword(const char* input, char* output, size_t outputSize) {
  // XOR is symmetric, so decrypt is same as encrypt
  encryptPassword(input, output, outputSize);
}

// ========== EEPROM CONFIGURATION MANAGEMENT ==========

bool loadWiFiConfig() {
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Check magic header
  uint32_t magic = 0;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);

  if (magic != EEPROM_MAGIC) {
    #if DEBUG_WIFI
    Serial.println("WiFi config not found in EEPROM (invalid magic)");
    #endif
    EEPROM.end();
    return false;
  }

  // Read configuration from EEPROM
  char encryptedPass[64];
  for (int i = 0; i < 33; i++) {
    wifiConfig.ssid[i] = EEPROM.read(EEPROM_ADDR_SSID + i);
  }
  for (int i = 0; i < 64; i++) {
    encryptedPass[i] = EEPROM.read(EEPROM_ADDR_PASS + i);
  }
  for (int i = 0; i < 32; i++) {
    wifiConfig.deviceName[i] = EEPROM.read(EEPROM_ADDR_DEVICE + i);
  }
  wifiConfig.configured = EEPROM.read(EEPROM_ADDR_CONFIGURED);

  // Verify checksum
  uint8_t storedChecksum = EEPROM.read(EEPROM_ADDR_CHECKSUM);
  uint8_t calculatedChecksum = 0;
  for (int i = 0; i < 33; i++) calculatedChecksum ^= wifiConfig.ssid[i];
  for (int i = 0; i < 64; i++) calculatedChecksum ^= encryptedPass[i];
  for (int i = 0; i < 32; i++) calculatedChecksum ^= wifiConfig.deviceName[i];
  calculatedChecksum ^= wifiConfig.configured;

  if (storedChecksum != calculatedChecksum) {
    #if DEBUG_WIFI
    Serial.println("WiFi config checksum mismatch - data corrupted");
    #endif
    EEPROM.end();
    return false;
  }

  // Decrypt password
  decryptPassword(encryptedPass, wifiConfig.password, sizeof(wifiConfig.password));
  wifiConfig.failCount = 0;

  EEPROM.end();

  #if DEBUG_WIFI
  Serial.println("WiFi config loaded from EEPROM");
  Serial.print("  SSID: ");
  Serial.println(wifiConfig.ssid);
  Serial.print("  Device: ");
  Serial.println(wifiConfig.deviceName);
  #endif

  return wifiConfig.configured && strlen(wifiConfig.ssid) > 0;
}

bool saveWiFiConfig() {
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Encrypt password
  char encryptedPass[64];
  encryptPassword(wifiConfig.password, encryptedPass, sizeof(encryptedPass));

  // Write magic header
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);

  // Write configuration to EEPROM
  for (int i = 0; i < 33; i++) {
    EEPROM.write(EEPROM_ADDR_SSID + i, wifiConfig.ssid[i]);
  }
  for (int i = 0; i < 64; i++) {
    EEPROM.write(EEPROM_ADDR_PASS + i, encryptedPass[i]);
  }
  for (int i = 0; i < 32; i++) {
    EEPROM.write(EEPROM_ADDR_DEVICE + i, wifiConfig.deviceName[i]);
  }
  EEPROM.write(EEPROM_ADDR_CONFIGURED, wifiConfig.configured);

  // Calculate and write checksum
  uint8_t checksum = 0;
  for (int i = 0; i < 33; i++) checksum ^= wifiConfig.ssid[i];
  for (int i = 0; i < 64; i++) checksum ^= encryptedPass[i];
  for (int i = 0; i < 32; i++) checksum ^= wifiConfig.deviceName[i];
  checksum ^= wifiConfig.configured;
  EEPROM.write(EEPROM_ADDR_CHECKSUM, checksum);

  // Commit changes to EEPROM
  bool success = EEPROM.commit();
  EEPROM.end();

  #if DEBUG_WIFI
  if (success) {
    Serial.println("WiFi config saved to EEPROM");
  } else {
    Serial.println("Failed to save WiFi config to EEPROM");
  }
  #endif

  return success;
}

void clearWiFiConfig() {
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Clear magic header to invalidate config
  EEPROM.put(EEPROM_ADDR_MAGIC, (uint32_t)0x00000000);

  // Clear all config data (optional, but good practice)
  for (int i = EEPROM_ADDR_SSID; i < EEPROM_ADDR_CHECKSUM + 1; i++) {
    EEPROM.write(i, 0);
  }

  EEPROM.commit();
  EEPROM.end();

  #if DEBUG_WIFI
  Serial.println("WiFi config cleared from EEPROM");
  #endif

  memset(&wifiConfig, 0, sizeof(wifiConfig));
  wifiConfig.configured = false;
}

void setPendingRestart() {
  pendingRestart = true;
}

// ========== AP MODE AND PORTAL FUNCTIONS ==========

void setupCaptivePortal() {
  if (!dnsServer) {
    dnsServer = new DNSServer();
  }

  // Start DNS server to redirect all domains to our AP IP (192.168.4.1)
  dnsServer->start(DNS_PORT, "*", WiFi.softAPIP());

  #if DEBUG_WIFI
  Serial.println("Captive portal DNS started");
  Serial.print("  AP IP: ");
  Serial.println(WiFi.softAPIP());
  #endif
}

void stopCaptivePortal() {
  if (dnsServer) {
    dnsServer->stop();
    delete dnsServer;
    dnsServer = nullptr;
  }

  #if DEBUG_WIFI
  Serial.println("Captive portal DNS stopped");
  #endif
}

void startAPMode() {
  if (wifiState == WIFI_STATE_AP_MODE || wifiState == WIFI_STATE_PORTAL_ACTIVE) {
    return; // Already in AP mode
  }

  // Clear hardcoded defaults flag since we're falling back to AP mode
  if (usingHardcodedDefaults) {
    #if DEBUG_WIFI
    Serial.println("Hardcoded defaults failed - falling back to AP mode");
    #endif
    usingHardcodedDefaults = false;
    wifiConfig.configured = false;  // Mark as unconfigured
  }

  #if DEBUG_WIFI
  Serial.println("Starting AP mode with captive portal");
  #endif

  // Generate unique AP SSID
  String apSSID = getAPSSID();

  // Switch to AP+STA mode (allows scanning while in AP mode)
  WiFi.mode(WIFI_AP_STA);

  // Configure and start SoftAP
  WiFi.softAP(apSSID.c_str(), AP_PASSWORD);

  // Small delay for AP to stabilize
  delay(100);

  #if DEBUG_WIFI
  Serial.println("=== AP Mode Active ===");
  Serial.print("  SSID: ");
  Serial.println(apSSID);
  Serial.print("  Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("  IP Address: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("  Connect and navigate to http://192.168.4.1");
  Serial.println("======================");
  #endif

  // Start captive portal DNS
  setupCaptivePortal();

  wifiState = WIFI_STATE_PORTAL_ACTIVE;
}

void stopAPMode() {
  #if DEBUG_WIFI
  Serial.println("Stopping AP mode");
  #endif

  stopCaptivePortal();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  wifiState = WIFI_STATE_CONNECTING;
}

bool isAPMode() {
  return (wifiState == WIFI_STATE_AP_MODE || wifiState == WIFI_STATE_PORTAL_ACTIVE);
}

String getAPSSID() {
  return String(AP_SSID_PREFIX) + String(ESP.getChipId(), HEX);
}

// ========== SERIAL CONSOLE FUNCTIONS ==========

void serialConsoleUpdate() {
  static String commandBuffer = "";
  const size_t MAX_COMMAND_LENGTH = 128;

  while (Serial.available()) {
    char c = Serial.read();

    if (commandBuffer.length() >= MAX_COMMAND_LENGTH) {
      commandBuffer = "";
      Serial.println("ERROR: Command too long");
      continue;
    }

    if (c == '\n' || c == '\r') {
      if (commandBuffer.length() > 0) {
        serialHandleCommand(commandBuffer);
        commandBuffer = "";
      }
    } else {
      commandBuffer += c;
    }
  }
}

void serialPrintMenu() {
  Serial.println("\n=== WiFi Configuration Menu ===");
  Serial.println("Commands:");
  Serial.println("  scan              - Scan for WiFi networks");
  Serial.println("  set <ssid> <pass> - Set WiFi credentials");
  Serial.println("  status            - Show connection status");
  Serial.println("  resetwifi         - Clear credentials and restart");
  Serial.println("  portal            - Start AP mode portal");
  Serial.println("  menu              - Show this menu");
  Serial.println("===============================\n");
}

void serialHandleCommand(String cmd) {
  cmd.trim();

  // Extract first word (command keyword) and convert only that to lowercase
  // This preserves case for parameters like SSID and password
  int firstSpace = cmd.indexOf(' ');
  String keyword = (firstSpace > 0) ? cmd.substring(0, firstSpace) : cmd;
  keyword.toLowerCase();

  if (keyword == "menu") {
    serialPrintMenu();
  }
  else if (keyword == "scan") {
    Serial.println("Scanning WiFi networks...");
    int n = WiFi.scanNetworks();
    Serial.print("Found ");
    Serial.print(n);
    Serial.println(" networks:");
    for (int i = 0; i < n; i++) {
      Serial.print("  ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm) ");
      Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
    }
  }
  else if (keyword == "set") {
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);

    if (secondSpace > 0) {
      String ssid = cmd.substring(firstSpace + 1, secondSpace);
      String pass = cmd.substring(secondSpace + 1);

      ssid.trim();
      pass.trim();

      strlcpy(wifiConfig.ssid, ssid.c_str(), sizeof(wifiConfig.ssid));
      strlcpy(wifiConfig.password, pass.c_str(), sizeof(wifiConfig.password));
      wifiConfig.configured = true;

      if (saveWiFiConfig()) {
        Serial.println("Credentials saved! Restarting...");
        setPendingRestart();
      } else {
        Serial.println("ERROR: Failed to save credentials");
      }
    } else {
      Serial.println("Usage: set <ssid> <password>");
    }
  }
  else if (keyword == "status") {
    Serial.println("\n=== WiFi Status ===");
    Serial.print("State: ");
    Serial.println(getWiFiStatusString());
    Serial.print("SSID: ");
    Serial.println(wifiConfig.configured ? wifiConfig.ssid : "(not configured)");
    if (wifiConnected) {
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      Serial.print("RSSI: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
      Serial.print("Time synced: ");
      Serial.println(timeSynced ? "Yes" : "No");
    }
    Serial.println("===================\n");
  }
  else if (keyword == "resetwifi") {
    Serial.println("Clearing WiFi credentials and restarting...");
    clearWiFiConfig();
    setPendingRestart();
  }
  else if (keyword == "portal") {
    Serial.println("Starting configuration portal...");
    startAPMode();
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
    Serial.println("Type 'menu' for help");
  }
}

// ========== STATUS AND DIAGNOSTIC FUNCTIONS ==========

String getWiFiStatusString() {
  switch (wifiState) {
    case WIFI_STATE_INIT: return "Initializing";
    case WIFI_STATE_CONNECTING: return "Connecting";
    case WIFI_STATE_CONNECTED: return "Connected";
    case WIFI_STATE_DISCONNECTED: return "Disconnected";
    case WIFI_STATE_AP_MODE: return "AP Mode";
    case WIFI_STATE_PORTAL_ACTIVE: return "Portal Active";
    default: return "Unknown";
  }
}

int getRSSI() {
  return wifiConnected ? WiFi.RSSI() : 0;
}

String getLocalIP() {
  return wifiConnected ? WiFi.localIP().toString() : "0.0.0.0";
}

// ========== CORE WIFI FUNCTIONS (MODIFIED) ==========

void wifiInit() {
  // Initialize LittleFS for web pages (WiFi config is stored in EEPROM)
  if (!LittleFS.begin()) {
    #if DEBUG_WIFI
    Serial.println("LittleFS mount failed - formatting...");
    #endif
    LittleFS.format();
    if (!LittleFS.begin()) {
      #if DEBUG_WIFI
      Serial.println("LittleFS format failed - web pages unavailable");
      #endif
    }
  }

  // Try to load saved credentials from EEPROM
  bool hasConfig = loadWiFiConfig();

  if (!hasConfig) {
    #if DEBUG_WIFI
    Serial.println("No WiFi config found - trying hardcoded defaults");
    #endif

    // Try hardcoded default credentials before falling back to AP mode
    strlcpy(wifiConfig.ssid, WIFI_AP1_SSID, sizeof(wifiConfig.ssid));
    strlcpy(wifiConfig.password, WIFI_AP1_PASS, sizeof(wifiConfig.password));
    strlcpy(wifiConfig.deviceName, "SolarV1", sizeof(wifiConfig.deviceName));
    wifiConfig.configured = true;  // Mark as configured to attempt connection
    usingHardcodedDefaults = true;  // Set flag to auto-save if connection succeeds

    #if DEBUG_WIFI
    Serial.print("Attempting connection with hardcoded SSID: ");
    Serial.println(wifiConfig.ssid);
    #endif

    // Continue to normal connection flow below (don't return here)
    // If connection fails, wifiUpdate() will fall back to AP mode after retries
  }

  // Configure WiFi in station mode
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_MODEM_SLEEP);
  WiFi.hostname(wifiConfig.deviceName);

  // Register event handlers and STORE the returned handlers to prevent auto-unsubscription
  wifiConnectHandler = WiFi.onStationModeGotIP(onWiFiConnected);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWiFiDisconnected);

  // Begin connection
  wifiState = WIFI_STATE_CONNECTING;
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);
  lastConnectionAttempt = millis();

  #if DEBUG_WIFI
  Serial.print("WiFi init started - connecting to: ");
  Serial.println(wifiConfig.ssid);
  Serial.println("Type 'menu' for WiFi configuration commands");
  #endif
}

void otaInit(const char* hostname) {
  ArduinoOTA.setHostname(hostname);

  ArduinoOTA.onStart([]() {
    #if DEBUG_WIFI
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
    #endif
  });

  ArduinoOTA.onEnd([]() {
    #if DEBUG_WIFI
    Serial.println("\nEnd");
    #endif
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    #if DEBUG_WIFI
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });

  ArduinoOTA.onError([](ota_error_t error) {
    #if DEBUG_WIFI
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
    #endif
  });
}

void wifiUpdate() {
  // Check for pending restart and execute outside async context
  if (pendingRestart) {
    delay(500);  // Allow HTTP response to be sent
    ESP.restart();
  }

  // Handle serial console commands
  serialConsoleUpdate();

  // Process DNS requests if in AP mode
  if (isAPMode() && dnsServer) {
    dnsServer->processNextRequest();
    return; // Don't run STA connection logic while in AP mode
  }

  // Check WiFi connection status (fallback in case event handler doesn't fire)
  if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    // WiFi connected but event handler didn't fire - manually trigger connection logic
    #if DEBUG_WIFI
    Serial.println("WiFi connected (detected via polling)");
    Serial.print("  SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("  IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("  Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("  RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    #endif

    wifiConnected = true;
    wifiState = WIFI_STATE_CONNECTED;
    connectionFailCount = 0;
    wifiConfig.failCount = 0;

    configTime(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2);
    lastNtpSync = millis();
    #if DEBUG_WIFI
    Serial.println("NTP sync started (Eastern Time)");
    #endif
    ArduinoOTA.begin();
  }

  // Handle OTA updates if connected
  if (wifiConnected) {
    ArduinoOTA.handle();

    // Check and update NTP sync status
    if (!timeSynced && time(nullptr) > 100000) {
      timeSynced = true;
      #if DEBUG_WIFI
      Serial.print("NTP synced: ");
      Serial.println(getUnixTime());
      #endif
    }

    // NTP sync retry logic:
    // - Before first sync: retry every 60 seconds
    // - After first sync: re-sync every 10 minutes
    static unsigned long nextNtpSync = 0;
    unsigned long now = millis();
    if (now >= nextNtpSync || nextNtpSync == 0) {
      unsigned long interval = timeSynced ? NTP_RESYNC_INTERVAL_MS : 60000;
      nextNtpSync = now + interval;
      configTime(NTP_TIMEZONE, NTP_SERVER1, NTP_SERVER2);
      #if DEBUG_WIFI
      Serial.printf("NTP %s (next in %lus)\n", timeSynced ? "re-sync" : "sync attempt", interval / 1000);
      #endif
    }
  } else {
    // Handle disconnected state with retry logic
    static unsigned long nextReconnectAttempt = 0;
    unsigned long now = millis();

    // Check for connection timeout
    if (wifiState == WIFI_STATE_CONNECTING) {
      if (now - lastConnectionAttempt > WIFI_CONNECTION_TIMEOUT_MS) {
        connectionFailCount++;
        wifiConfig.failCount++;

        #if DEBUG_WIFI
        Serial.print("Connection timeout (attempt ");
        Serial.print(connectionFailCount);
        Serial.println(")");
        #endif

        // Too many failures? Start AP mode
        if (connectionFailCount >= WIFI_MAX_RETRY_BEFORE_AP) {
          #if DEBUG_WIFI
          Serial.println("Max retries reached - starting AP mode");
          #endif
          startAPMode();
          return; // Don't continue retry logic
        }

        wifiState = WIFI_STATE_DISCONNECTED;
      }
    }

    // Attempt reconnection at intervals (overflow-safe)
    if (now >= nextReconnectAttempt || nextReconnectAttempt == 0) {
      nextReconnectAttempt = now + WIFI_RECONNECT_INTERVAL_MS;
      if (WiFi.status() != WL_CONNECTED && wifiState != WIFI_STATE_CONNECTING) {
        #if DEBUG_WIFI
        Serial.println("Attempting WiFi reconnect...");
        #endif

        wifiState = WIFI_STATE_CONNECTING;
        lastConnectionAttempt = now;
        // Use disconnect + begin instead of reconnect() for better stability
        WiFi.disconnect();
        delay(100);  // Small delay to let WiFi settle
        WiFi.begin(wifiConfig.ssid, wifiConfig.password);
      }
    }
  }
}

bool isTimeSynced() {
  return timeSynced;
}

time_t getUnixTime() {
  if (!timeSynced) {
    return 0;
  }
  return time(nullptr);
}
