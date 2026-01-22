/**
 * RD03Radar Web Server Example
 *
 * This example demonstrates how to use RD03Radar with built-in web interface.
 * Provides a web-based control panel for monitoring and controlling the radar.
 *
 * Hardware Required:
 * - ESP32 or ESP8266
 * - Ai-Thinker RD-03 mmWave Radar Sensor
 * - WiFi connection
 *
 * Wiring:
 * ESP32/ESP8266  RD-03 Radar
 * -----------------  -----------
 * GPIO17/D5      ── TX
 * GPIO16/D6      ── RX
 * GND            ── GND
 * 5V             ── VCC
 *
 * Web Interface:
 * - Main page: http://esp-ip/
 * - API Status: http://esp-ip/api/status
 * - API Commands: POST to http://esp-ip/api/command
 *
 * Author: Mohamed Eid (gomgom-40)
 * Version: 1.1.0
 */

#if defined(ESP8266)
  #include <SoftwareSerial.h>
#endif

#include <RD03Radar.h>

// WiFi credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// Web server port
const uint16_t WEB_PORT = 80;

// Radar configuration
RD03Config radarConfig = {
    .minRange = 20.0f,
    .maxRange = 500.0f,
    .sensitivity = 3,
    .holdTime = 30,
    .maxAbsenceTime = 300
};

// Create radar instance - use different Serial for different platforms
#if defined(ESP8266)
  // ESP8266 uses SoftwareSerial to avoid conflict with Serial monitor
  SoftwareSerial radarSerial(12, 14);  // RX, TX pins
  RD03Radar radar(radarSerial, radarConfig);
#else
  // ESP32 uses Serial2 (HardwareSerial)
  RD03Radar radar(Serial2, radarConfig);
#endif

// LED for visual feedback (optional)
const int STATUS_LED = 2;

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    Serial.println("RD03Radar Web Server Example");
    Serial.println("============================");

    // Connect to WiFi
    setupWiFi();

    // Setup web server
    radar.setupWebServer(WEB_PORT);

    // Setup presence callback
    radar.onPresenceChange([](RD03PresenceState state, float distance) {
        Serial.printf("Presence: %s, Distance: %.1f cm\n",
                     state == RD03PresenceState::PRESENCE_DETECTED ? "DETECTED" : "NONE",
                     distance);

        // Visual feedback
        digitalWrite(STATUS_LED, state == RD03PresenceState::PRESENCE_DETECTED ? HIGH : LOW);
    });

    // Setup status callback
    radar.onStatusChange([](RD03Status status, const char* message) {
        Serial.print("Status: ");
        switch (status) {
            case RD03Status::OK: Serial.println("OK"); break;
            case RD03Status::ERROR: Serial.println("ERROR"); break;
            case RD03Status::NO_SIGNAL: Serial.println("NO_SIGNAL"); break;
            case RD03Status::BUFFER_OVERFLOW: Serial.println("BUFFER_OVERFLOW"); break;
            case RD03Status::INVALID_DATA: Serial.println("INVALID_DATA"); break;
            case RD03Status::WATCHDOG_RESET: Serial.println("WATCHDOG_RESET"); break;
        }
        if (strlen(message) > 0) {
            Serial.print("Message: ");
            Serial.println(message);
        }
    });

    // Initialize radar
    Serial.println("Initializing radar...");
    #if defined(ESP8266)
      if (radar.begin()) {  // ESP8266 uses SoftwareSerial (already initialized)
    #else
      if (radar.begin(16, 17)) {  // ESP32 uses HardwareSerial with custom pins
    #endif
        Serial.println("✅ Radar initialized successfully!");
        Serial.println("Starting web server...");
    } else {
        Serial.println("❌ Failed to initialize radar!");
        while (1) {
            delay(1000);
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); // Blink LED on error
        }
    }

    // Start web server
    radar.startWebServer();
    Serial.printf("Web server started at: http://%s:%d/\n",
                  WiFi.localIP().toString().c_str(), WEB_PORT);
}

void loop() {
    radar.loop();

    // Print status every minute
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 60000) {
        lastStatus = millis();
        Serial.printf("Uptime: %lu seconds, Status: %s\n",
                     radar.getUptime(),
                     radar.isOperational() ? "Operational" : "Error");
    }

    // Small delay to prevent overwhelming the serial output
    delay(10);
}

void setupWiFi() {
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connection failed!");
        ESP.restart();
    }
}
