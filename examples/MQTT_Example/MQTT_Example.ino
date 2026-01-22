/**
 * RD03Radar MQTT Example - Fixed
 *
 * Demonstrates RD03Radar with MQTT for IoT.
 *
 * Hardware Required:
 * - ESP32 or ESP8266
 * - Ai-Thinker RD-03 mmWave Radar Sensor
 * - WiFi connection
 * - MQTT Broker
 *
 * Wiring:
 * ESP32/ESP8266  RD-03 Radar
 * -----------------  -----------
 * GPIO17/D5      ── TX
 * GPIO16/D6      ── RX
 * GND            ── GND
 * 5V             ── VCC
 *
 * Author: Mohamed Eid (gomgom-40)
 * Version: 1.1.1
 */

#include <RD03Radar.h>
#include <ESP8266WiFi.h>  // For ESP8266, change to <WiFi.h> for ESP32
#include <PubSubClient.h> // MQTT

// WiFi credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT broker settings
const char* MQTT_SERVER = "192.168.1.100";  // Change to your MQTT broker IP
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USERNAME = nullptr;        // Set if authentication required
const char* MQTT_PASSWORD = nullptr;        // Set if authentication required

// Radar configuration (order matches RD03Config declaration)
RD03Config radarConfig = {
    20.0f,   // minRange
    500.0f,  // maxRange
    3,       // sensitivity
    30,      // holdTime
    300,     // maxAbsenceTime
    2,       // motionHitsRequired
    10.0f    // motionThreshold
};

// Use Serial1 for ESP8266, Serial2 for ESP32
RD03Radar radar(Serial1, radarConfig);  

// LED for visual feedback (optional)
const int STATUS_LED = 2;

// Helper to convert status to string
const char* statusToString(RD03Status status) {
    switch (status) {
        case RD03Status::OK: return "OK";
        case RD03Status::ERROR: return "ERROR";
        case RD03Status::NO_SIGNAL: return "NO_SIGNAL";
        case RD03Status::BUFFER_OVERFLOW: return "BUFFER_OVERFLOW";
        case RD03Status::INVALID_DATA: return "INVALID_DATA";
        case RD03Status::WATCHDOG_RESET: return "WATCHDOG_RESET";
        default: return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    Serial.println("RD03Radar MQTT Example - Fixed");
    Serial.println("==============================");

    // Connect to WiFi
    setupWiFi();

    // Setup MQTT
    radar.setupMQTT(MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD);

    // Setup presence callback
    radar.onPresenceChange([](RD03PresenceState state, float distance) {
        Serial.printf("Presence: %s, Distance: %.1f cm\n",
                     state == RD03PresenceState::PRESENCE_DETECTED ? "DETECTED" : "NONE",
                     distance);

        // Visual feedback
        digitalWrite(STATUS_LED, state == RD03PresenceState::PRESENCE_DETECTED ? HIGH : LOW);
    });

    // Setup status callback
    radar.onStatusChange([](RD03Status status, const char* msg) {
        Serial.printf("Status: %s - %s\n", msg, statusToString(status));
    });

    // Initialize radar
    if (radar.begin(16, 17)) {  // ESP32 pins (TX=17, RX=16)
        Serial.println("Radar initialized successfully");
    } else {
        Serial.println("Failed to initialize radar!");
        while (1) delay(1000);
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
