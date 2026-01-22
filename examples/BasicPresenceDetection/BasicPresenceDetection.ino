/*
 * Basic Presence Detection Example for RD03Radar Library
 *
 * This example demonstrates the basic usage of the RD03Radar library
 * to detect presence and control an LED based on motion detection.
 *
 * Hardware Required:
 * - ESP32 or ESP8266 board
 * - Ai-Thinker RD-03 mmWave radar sensor
 * - LED with resistor (optional) connected to GPIO2
 *
 * Wiring:
 * ESP32:    GPIO17 → TX, GPIO16 → RX, GND → GND, 5V → VCC, GPIO2 → LED+
 * ESP8266:  GPIO14 (D5) → TX, GPIO12 (D6) → RX, GND → GND, 5V → VCC, GPIO2 → LED+
 *
 * Author: Mohamed Eid (gomgom-40)
 * License: MIT
 */

#if defined(ESP8266)
  #include <SoftwareSerial.h>
#endif

#include <RD03Radar.h>

// LED pin for visual feedback (optional)
#define LED_PIN 2

// ============================================================================
// Radar Configuration & Instance
// ============================================================================
RD03Config radarConfig;

#if defined(ESP8266)
  // ESP8266 uses SoftwareSerial (RX=12/D6, TX=14/D5)
  SoftwareSerial radarSerial(12, 14);
  RD03Radar radar(radarSerial, radarConfig);
#else
  // ESP32 uses Serial2 (default pins GPIO16 RX, GPIO17 TX)
  RD03Radar radar(Serial2, radarConfig);
#endif

// ============================================================================
// Callback Functions
// ============================================================================
void onPresenceChange(RD03PresenceState state, float distance) {
    Serial.print("Presence State: ");
    switch (state) {
        case RD03PresenceState::NO_PRESENCE:
            Serial.println("No Presence");
            digitalWrite(LED_PIN, LOW);
            break;
        case RD03PresenceState::PRESENCE_DETECTED:
            Serial.println("Presence Detected");
            digitalWrite(LED_PIN, HIGH);
            break;
        case RD03PresenceState::MOTION_DETECTED:
            Serial.println("Motion Detected!");
            digitalWrite(LED_PIN, HIGH);
            break;
        case RD03PresenceState::MAINTAINING:
            Serial.println("Maintaining Presence");
            digitalWrite(LED_PIN, HIGH);
            break;
        case RD03PresenceState::FAST_EXIT:
            Serial.println("Fast Exit");
            digitalWrite(LED_PIN, LOW);
            break;
        case RD03PresenceState::SAFETY_TIMEOUT:
            Serial.println("Safety Timeout");
            digitalWrite(LED_PIN, LOW);
            break;
    }
    if (distance > 0) {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}

void onStatusChange(RD03Status status, const char* message) {
    Serial.print("Radar Status: ");
    switch (status) {
        case RD03Status::OK:              Serial.println("OK"); break;
        case RD03Status::ERROR:           Serial.println("ERROR"); break;
        case RD03Status::NO_SIGNAL:       Serial.println("No Signal"); break;
        case RD03Status::BUFFER_OVERFLOW: Serial.println("Buffer Overflow"); break;
        case RD03Status::INVALID_DATA:    Serial.println("Invalid Data"); break;
        case RD03Status::WATCHDOG_RESET:  Serial.println("Watchdog Reset"); break;
    }
    if (message && strlen(message) > 0) {
        Serial.print("Message: ");
        Serial.println(message);
    }
}

void onDistanceMeasurement(float distance, bool valid) {
    if (valid && distance > 0) {
        // Optional: add custom logic here if needed
    }
}

void onLightControl(bool turnOn, const char* reason) {
    Serial.print("Light Control: ");
    Serial.print(turnOn ? "ON" : "OFF");
    Serial.print(" - Reason: ");
    Serial.println(reason ? reason : "Unknown");
    digitalWrite(LED_PIN, turnOn ? HIGH : LOW);
}

// ============================================================================
// Setup & Loop
// ============================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);  // Wait for serial on some boards

    Serial.println("\n\nRD03Radar Basic Example");
    Serial.println("========================");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Apply radar configuration
    radarConfig.minRange         = 20.0f;
    radarConfig.maxRange         = 500.0f;
    radarConfig.sensitivity      = 3;
    radarConfig.holdTime         = 30;
    radarConfig.maxAbsenceTime   = 300;

    radar.setConfig(radarConfig);

    // Register callbacks
    radar.onPresenceChange(onPresenceChange);
    radar.onStatusChange(onStatusChange);
    radar.onDistanceMeasurement(onDistanceMeasurement);
    radar.onLightControl(onLightControl);

    // Initialize radar
    Serial.println("Initializing radar...");

#if defined(ESP8266)
    // ESP8266: SoftwareSerial is already constructed and initialized
    if (radar.begin()) {
#else
    // ESP32: Specify RX/TX pins for Serial2
    if (radar.begin(16, 17)) {
#endif
        Serial.println("✅ Radar initialized successfully!");
        Serial.println("Waiting for presence detection...");
    } else {
        Serial.println("❌ Failed to initialize radar!");
        while (true) {
            delay(1000);
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Blink on error
        }
    }
}

void loop() {
    radar.loop();

    // Print status report every 30 seconds
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 30000) {
        lastPrint = millis();

        Serial.println("\n--- Status Report ---");
        Serial.printf("Uptime: %lu seconds\n", radar.getUptime() / 1000);
        Serial.printf("Operational: %s\n", radar.isOperational() ? "Yes" : "No");
        Serial.printf("Distance: %.1f cm\n", radar.getDistance());
        Serial.print("Presence State: ");
        Serial.println(radar.getPresenceStateString());
        Serial.println("-------------------");
    }

    delay(10);  // Small delay to avoid flooding serial
}