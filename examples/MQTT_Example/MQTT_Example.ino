/**
 * RD03Radar MQTT Example (Modified for RD03Radar v1.0.0)
 *
 * Demonstrates RD03Radar with MQTT using ESP8266/ESP32.
 * Publishes presence status and accepts commands via MQTT.
 *
 * Hardware Required:
 * - ESP32 or ESP8266
 * - Ai-Thinker RD-03 Radar
 * - WiFi connection
 * - MQTT Broker
 *
 * Wiring:
 * ESP32/ESP8266  RD-03 Radar
 * -----------------  ---------
 * TX (ESP)       ── RX
 * RX (ESP)       ── TX
 * GND            ── GND
 * 5V             ── VCC
 *
 * MQTT Topics:
 * - rd03radar/status (publish)
 * - rd03radar/commands (subscribe)
 *
 * Author: Mohamed Eid (gomgom-40)
 * Version: 1.0.0 compatible
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>       // ESP8266 WiFi
#include <PubSubClient.h>      // MQTT
#include "RD03Radar.h"

// =====================
// WiFi & MQTT Settings
// =====================
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_SERVER = "192.168.1.100";  // MQTT broker IP
const uint16_t MQTT_PORT = 1883;

// =====================
// Radar Configuration
// =====================
RD03Config radarConfig = {
    .minRange = 20.0f,
    .maxRange = 500.0f,
    .sensitivity = 3,
    .holdTime = 30,
    .maxAbsenceTime = 300,
    .motionHitsRequired = 2,    // defaults from library
    .motionThreshold = 10.0f
};

RD03Radar radar(Serial, radarConfig); // Use Serial (ESP8266 has no Serial2)

// LED for visual feedback
const int STATUS_LED = 2;

// =====================
// MQTT Setup
// =====================
WiFiClient espClient;
PubSubClient mqttClient(espClient);

void connectWiFi() {
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
        Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connection failed!");
        ESP.restart();
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }
    Serial.print("MQTT command received: ");
    Serial.println(msg);

    if (msg == "force_on") radar.setControlMode(RD03ControlMode::FORCE_ON);
    else if (msg == "force_off") radar.setControlMode(RD03ControlMode::FORCE_OFF);
    else if (msg == "automatic") radar.setControlMode(RD03ControlMode::AUTOMATIC);
    else if (msg == "reset") radar.resetPresence();
}

void connectMQTT() {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    while (!mqttClient.connected()) {
        Serial.print("Connecting to MQTT...");
        if (mqttClient.connect("ESP_RD03")) {
            Serial.println("connected");
            mqttClient.subscribe("rd03radar/commands");
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" retry in 2s");
            delay(2000);
        }
    }
}

void publishStatus() {
    String json = "{";
    json += "\"presence\":" + String(radar.getPresenceState() != RD03PresenceState::NO_PRESENCE ? "true" : "false") + ",";
    json += "\"distance\":" + String(radar.getDistance(), 1);
    json += "}";
    mqttClient.publish("rd03radar/status", json.c_str());
}

// =====================
// Setup
// =====================
void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    Serial.println("RD03Radar MQTT Example (v1.0.0)");

    connectWiFi();
    connectMQTT();

    // Setup callbacks
    radar.onPresenceChange([](RD03PresenceState state, float distance) {
        Serial.printf("Presence: %s, Distance: %.1f cm\n",
                      state != RD03PresenceState::NO_PRESENCE ? "DETECTED" : "NONE",
                      distance);
        digitalWrite(STATUS_LED, state != RD03PresenceState::NO_PRESENCE ? HIGH : LOW);
    });

    radar.onStatusChange([](RD03Status status, const char* msg) {
        Serial.printf("Status: %s - %s\n", msg, radar.statusToString(status));
    });

    if (radar.begin()) {
        Serial.println("Radar initialized successfully");
    } else {
        Serial.println("Failed to initialize radar!");
        while (1) delay(1000);
    }
}

// =====================
// Main loop
// =====================
void loop() {
    radar.loop();
    mqttClient.loop();

    // Publish status every 30s
    static unsigned long lastPublish = 0;
    if (millis() - lastPublish > 30000) {
        lastPublish = millis();
        publishStatus();
        Serial.println("Status published to MQTT");
    }

    delay(10);
}
