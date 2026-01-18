/*
 * Basic Presence Detection Example for RD03Radar Library
 *
 * This example demonstrates the basic usage of the RD03Radar library
 * to detect presence and control an LED based on motion detection.
 *
 * Hardware Required:
 * - ESP32 or ESP8266 board
 * - Ai-Thinker RD-03 mmWave radar sensor
 * - LED with resistor (optional)
 *
 * ESP32 Connections:
 * ESP32          RD-03 Radar       LED
 * -----          -----------       ---
 * GPIO17 ────── TX
 * GPIO16 ────── RX
 * GND    ────── GND
 * 5V     ────── VCC
 * GPIO2  ────── LED+ (optional)
 *
 * ESP8266 Connections:
 * ESP8266        RD-03 Radar       LED
 * -------        -----------       ---
 * GPIO14 ────── TX (D5)
 * GPIO12 ────── RX (D6)
 * GND    ────── GND
 * 5V     ────── VCC
 * GPIO2  ────── LED+ (optional)
 *
 * Author: Mohamed Eid (gomgom-40)
 * License: MIT
 */

#if defined(ESP8266)
  #include <SoftwareSerial.h>
#endif

// Include the RD03Radar library
#include <RD03Radar.h>

// ============================================================================
// Hardware Configuration
// ============================================================================

// LED pin for visual feedback (optional)
#define LED_PIN 2

// ============================================================================
// Radar Configuration
// ============================================================================

// Create radar configuration
RD03Config radarConfig;

// Create radar instance - use different Serial for different platforms
#if defined(ESP8266)
  // ESP8266 uses SoftwareSerial to avoid conflict with Serial monitor
  SoftwareSerial radarSerial(12, 14);  // RX, TX pins
  RD03Radar radar(radarSerial, radarConfig);
#else
  // ESP32 uses Serial2 (HardwareSerial)
  HardwareSerial& radarSerial = Serial2;
  RD03Radar radar(radarSerial, radarConfig);
#endif

// ============================================================================
// Callback Functions
// ============================================================================

/**
 * @brief Called when presence state changes
 */
void onPresenceChange(RD03PresenceState state, float distance) {
    Serial.print("Presence State: ");

    switch (state) {
        case RD03PresenceState::NO_PRESENCE:
            Serial.println("No Presence");
            digitalWrite(LED_PIN, LOW);  // Turn off LED
            break;

        case RD03PresenceState::PRESENCE_DETECTED:
            Serial.println("Presence Detected");
            digitalWrite(LED_PIN, HIGH); // Turn on LED
            break;

        case RD03PresenceState::MOTION_DETECTED:
            Serial.println("Motion Detected!");
            digitalWrite(LED_PIN, HIGH); // Turn on LED
            break;

        case RD03PresenceState::MAINTAINING:
            Serial.println("Maintaining Presence");
            digitalWrite(LED_PIN, HIGH); // Keep LED on
            break;

        case RD03PresenceState::FAST_EXIT:
            Serial.println("Fast Exit");
            digitalWrite(LED_PIN, LOW);  // Turn off LED
            break;

        case RD03PresenceState::SAFETY_TIMEOUT:
            Serial.println("Safety Timeout");
            digitalWrite(LED_PIN, LOW);  // Turn off LED
            break;
    }

    if (distance > 0) {
        Serial.print("Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}

/**
 * @brief Called when radar status changes
 */
void onStatusChange(RD03Status status, const char* message) {
    Serial.print("Radar Status: ");

    switch (status) {
        case RD03Status::OK:
            Serial.println("OK");
            break;
        case RD03Status::ERROR:
            Serial.println("ERROR");
            break;
        case RD03Status::NO_SIGNAL:
            Serial.println("No Signal");
            break;
        case RD03Status::BUFFER_OVERFLOW:
            Serial.println("Buffer Overflow");
            break;
        case RD03Status::INVALID_DATA:
            Serial.println("Invalid Data");
            break;
        case RD03Status::WATCHDOG_RESET:
            Serial.println("Watchdog Reset");
            break;
    }

    if (strlen(message) > 0) {
        Serial.print("Message: ");
        Serial.println(message);
    }
}

/**
 * @brief Called for each distance measurement
 */
void onDistanceMeasurement(float distance, bool valid) {
    if (valid && distance > 0) {
        // You can add additional processing here
        // For example, logging or additional logic
    }
}

/**
 * @brief Called when light control action is needed
 * This is where you would control your actual lights/appliances
 */
void onLightControl(bool turnOn, const char* reason) {
    Serial.print("Light Control: ");
    Serial.print(turnOn ? "ON" : "OFF");
    Serial.print(" - Reason: ");
    Serial.println(reason);

    // Control actual light/appliance here
    // For example: relay control, smart bulb API call, etc.
    digitalWrite(LED_PIN, turnOn ? HIGH : LOW);
}

// ============================================================================
// Arduino Setup & Loop
// ============================================================================

void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    Serial.println("\n\nRD03Radar Basic Example");
    Serial.println("========================");

    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Configure radar settings
    radarConfig.minRange = 20.0f;      // Minimum detection range (cm)
    radarConfig.maxRange = 500.0f;     // Maximum detection range (cm)
    radarConfig.sensitivity = 3;       // Sensitivity level (1-5)
    radarConfig.holdTime = 30;         // Hold time in seconds
    radarConfig.maxAbsenceTime = 300;  // Safety timeout in seconds

    // Apply configuration
    radar.setConfig(radarConfig);

    // Register callback functions
    radar.onPresenceChange(onPresenceChange);
    radar.onStatusChange(onStatusChange);
    radar.onDistanceMeasurement(onDistanceMeasurement);
    radar.onLightControl(onLightControl);

    // Initialize radar
    Serial.println("Initializing radar...");
    #if defined(ESP8266)
      radarSerial.begin(115200);  // Initialize SoftwareSerial
      if (radar.begin()) {  // ESP8266 uses SoftwareSerial
    #else
      if (radar.begin(16, 17)) {  // ESP32 uses custom pins
    #endif
        Serial.println("✅ Radar initialized successfully!");
        Serial.println("Waiting for presence detection...");
    } else {
        Serial.println("❌ Failed to initialize radar!");
        while (1) {
            delay(1000);
            digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink LED on error
        }
    }
}

void loop() {
    // Main radar processing - call this in every loop iteration
    radar.loop();

    // Optional: Print status every 30 seconds
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint > 30000) {
        lastStatusPrint = millis();

        Serial.println("\n--- Status Report ---");
        Serial.print("Uptime: ");
        Serial.print(radar.getUptime() / 1000);
        Serial.println(" seconds");

        Serial.print("Operational: ");
        Serial.println(radar.isOperational() ? "Yes" : "No");

        Serial.print("Current Distance: ");
        Serial.print(radar.getDistance());
        Serial.println(" cm");

        Serial.print("Presence State: ");
        switch (radar.getPresenceState()) {
            case RD03PresenceState::NO_PRESENCE: Serial.println("No Presence"); break;
            case RD03PresenceState::PRESENCE_DETECTED: Serial.println("Presence Detected"); break;
            case RD03PresenceState::MOTION_DETECTED: Serial.println("Motion Detected"); break;
            case RD03PresenceState::MAINTAINING: Serial.println("Maintaining"); break;
            case RD03PresenceState::FAST_EXIT: Serial.println("Fast Exit"); break;
            case RD03PresenceState::SAFETY_TIMEOUT: Serial.println("Safety Timeout"); break;
        }
        Serial.println("-------------------");
    }

    // Small delay to prevent overwhelming the serial output
    delay(10);
}
