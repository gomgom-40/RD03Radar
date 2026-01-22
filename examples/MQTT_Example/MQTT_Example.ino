/**
 * RD03Radar MQTT Example
 *
 * This example demonstrates how to use RD03Radar with MQTT for IoT connectivity.
 * The sensor will publish its status and accept commands via MQTT broker.
 *
 * Hardware Required:
 * - ESP32 or ESP8266
 * - Ai-Thinker RD-03 mmWave Radar Sensor
 * - WiFi connection
 * - MQTT Broker (local or cloud)
 *
 * Wiring:
 * ESP32/ESP8266 RD-03 Radar
 * ----------------- -----------
 * GPIO17/D5 ── TX
 * GPIO16/D6 ── RX
 * GND ── GND
 * 5V ── VCC
 *
 * MQTT Topics:
 * - rd03radar/status (published) - JSON status updates
 * - rd03radar/commands (subscribed) - Command messages
 *
 * Author: Mohamed Eid (gomgom-40)
 * Version: 1.1.0
 */
#if defined(ESP8266)
  #include <SoftwareSerial.h>
#endif
#include <RD03Radar.h>

// Add WiFi include depending on platform
#if defined(ESP32)
  #include <WiFi.h>
#elif defined(ESP8266)
  #include <ESP8266WiFi.h>
#else
  #error "This example is intended for ESP32 or ESP8266 only"
#endif

// WiFi credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT broker settings
const char* MQTT_SERVER = "192.168.1.100"; // Change to your MQTT broker IP
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USERNAME = nullptr; // Set if authentication required
const char* MQTT_PASSWORD = nullptr; // Set if authentication required

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
  SoftwareSerial radarSerial(12, 14); // RX, TX pins
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
    Serial.println("RD03Radar MQTT Example");
    Serial.println("======================");

    // Connect to WiFi
    setupWiFi();

    // Setup MQTT
    radar.setupMQTT(MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD);

    // Setup presence callback
    radar.onPresenceChange([](RD03PresenceState state, float distance) {
        Serial.print("Presence: ");
        Serial.print(state == RD03PresenceState::PRESENCE_DETECTED ? "DETECTED" : "NONE");
        Serial.print(", Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
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
      if (radar.begin()) { // ESP8266 uses SoftwareSerial (already initialized)
    #else
      if (radar.begin(16, 17)) { // ESP32 uses HardwareSerial with custom pins
    #endif
        Serial.println("✅ Radar initialized successfully!");
        Serial.println("Waiting for MQTT connection and presence detection...");
    } else {
        Serial.println("❌ Failed to initialize radar!");
        while (1) {
            delay(1000);
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); // Blink LED on error
        }
    }
}

void loop() {
    radar.loop();

    // Publish status every 30 seconds
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 30000) {
        lastPublish = millis();
        radar.publishStatus();
        Serial.println("Status published to MQTT");
    }

    // Small delay to prevent overwhelming the serial output
    delay(10);
}

void setupWiFi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP().toString());
    } else {
        Serial.println("\nWiFi connection failed!");
        while (1) {
            delay(1000);
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED)); // Blink LED on error
        }
    }
}