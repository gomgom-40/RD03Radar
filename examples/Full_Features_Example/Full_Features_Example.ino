/**
 * RD03Radar Full Features Example
 *
 * Complete example demonstrating all RD03Radar capabilities:
 * - MQTT connectivity for IoT integration
 * - Web interface for local control
 * - Advanced presence detection
 * - Comprehensive status monitoring
 *
 * Hardware Required:
 * - ESP32 or ESP8266
 * - Ai-Thinker RD-03 mmWave Radar Sensor
 * - WiFi connection
 * - MQTT Broker (optional but recommended)
 *
 * Wiring:
 * ESP32/ESP8266  RD-03 Radar
 * -----------------  -----------
 * GPIO17/D5      ── TX
 * GPIO16/D6      ── RX
 * GND            ── GND
 * 5V             ── VCC
 *
 * Features Demonstrated:
 * - Motion-based presence detection
 * - MQTT status publishing and command subscription
 * - Web-based control interface
 * - Real-time status monitoring
 * - Automatic reconnection and error recovery
 *
 * Author: Mohamed Eid (gomgom-40)
 * Version: 1.1.0
 */

#include <RD03Radar.h>

// WiFi credentials
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT settings (optional - comment out if not using MQTT)
const char* MQTT_SERVER = "192.168.1.100";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USERNAME = nullptr;
const char* MQTT_PASSWORD = nullptr;

// Web server port
const uint16_t WEB_PORT = 80;

// Radar configuration with optimized settings
RD03Config radarConfig = {
    .minRange = 20.0f,           // Minimum detection range (cm)
    .maxRange = 500.0f,          // Maximum detection range (cm)
    .sensitivity = 3,            // Sensitivity level (1-5)
    .holdTime = 30,              // Hold time in seconds
    .maxAbsenceTime = 300,       // Safety timeout in seconds
    .motionThreshold = 2.0f,     // Motion detection threshold (cm)
    .motionHitsRequired = 1,     // Motion confirmation hits
    .baudRate = 115200,          // UART baud rate
    .rxBufferSize = 256          // UART buffer size
};

RD03Radar radar(Serial2, radarConfig);

// Status LED for visual feedback
const int STATUS_LED = 2;

// Control variables
bool useMQTT = true;  // Set to false to disable MQTT
bool useWebServer = true;  // Set to false to disable web server

void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LOW);

    Serial.println("RD03Radar Full Features Example");
    Serial.println("===============================");

    // Connect to WiFi
    setupWiFi();

    // Setup MQTT (if enabled)
    if (useMQTT) {
        Serial.println("Setting up MQTT...");
        radar.setupMQTT(MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD);
    }

    // Setup Web Server (if enabled)
    if (useWebServer) {
        Serial.println("Setting up Web Server...");
        radar.setupWebServer(WEB_PORT);
    }

    // Setup callbacks
    setupCallbacks();

    // Initialize radar
    Serial.println("Initializing radar...");
    if (radar.begin(16, 17)) {  // ESP32 pins (TX=17, RX=16)
        Serial.println("✅ Radar initialized successfully");
    } else {
        Serial.println("❌ Failed to initialize radar!");
        while (1) {
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
            delay(500);
        }
    }

    // Start services
    if (useWebServer) {
        radar.startWebServer();
        Serial.printf("🌐 Web server started at: http://%s:%d/\n",
                      WiFi.localIP().toString().c_str(), WEB_PORT);
    }

    Serial.println("🚀 System ready!");
    Serial.println("==================");
}

void loop() {
    radar.loop();

    // Publish MQTT status periodically
    static unsigned long lastMQTT = 0;
    if (useMQTT && millis() - lastMQTT > 30000) {  // Every 30 seconds
        lastMQTT = millis();
        radar.publishStatus();
        Serial.println("📤 MQTT status published");
    }

    // Print system status periodically
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 60000) {  // Every minute
        lastStatus = millis();
        printSystemStatus();
    }

    delay(10);  // Small delay to prevent overwhelming
}

void setupCallbacks() {
    // Presence change callback
    radar.onPresenceChange([](RD03PresenceState state, float distance) {
        const char* stateStr = getPresenceStateString(state);
        Serial.printf("👤 Presence: %s (%.1f cm)\n", stateStr, distance);

        // Visual feedback
        digitalWrite(STATUS_LED, state == RD03PresenceState::PRESENCE_DETECTED ? HIGH : LOW);

        // Publish to MQTT immediately on presence change
        if (useMQTT) {
            radar.publishStatus();
        }
    });

    // Status change callback
    radar.onStatusChange([](RD03Status status) {
        const char* statusStr = getStatusString(status);
        Serial.printf("📊 Status: %s\n", statusStr);

        // Flash LED on error
        if (status != RD03Status::OK) {
            for (int i = 0; i < 3; i++) {
                digitalWrite(STATUS_LED, HIGH);
                delay(100);
                digitalWrite(STATUS_LED, LOW);
                delay(100);
            }
        }
    });

    // Distance measurement callback (optional - for debugging)
    radar.onDistanceMeasurement([](float distance) {
        // Only print if distance changed significantly
        static float lastPrinted = 0;
        if (abs(distance - lastPrinted) >= 10.0f) {
            Serial.printf("📏 Distance: %.1f cm\n", distance);
            lastPrinted = distance;
        }
    });

    // Light control callback
    radar.onLightControl([](bool turnOn, const char* reason) {
        Serial.printf("💡 Light: %s (%s)\n", turnOn ? "ON" : "OFF", reason);

        // Publish status update
        if (useMQTT) {
            radar.publishStatus();
        }
    });
}

void setupWiFi() {
    Serial.printf("🔗 Connecting to WiFi: %s\n", WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi connected!");
        Serial.printf("📡 IP address: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n❌ WiFi connection failed!");
        ESP.restart();
    }
}

void printSystemStatus() {
    Serial.println("\n📈 System Status");
    Serial.println("===============");
    Serial.printf("⏱️  Uptime: %lu seconds\n", radar.getUptime());
    Serial.printf("📍 Distance: %.1f cm\n", radar.getDistance());
    Serial.printf("👤 Presence: %s\n", radar.getPresenceState() == RD03PresenceState::PRESENCE_DETECTED ? "DETECTED" : "NONE");
    Serial.printf("⚙️  Operational: %s\n", radar.isOperational() ? "YES" : "NO");

    if (useMQTT) {
        Serial.printf("📡 MQTT: %s\n", radar.isMQTTConnected() ? "CONNECTED" : "DISCONNECTED");
    }

    if (useWebServer) {
        Serial.printf("🌐 Web Server: %s\n", radar.isWebServerRunning() ? "RUNNING" : "STOPPED");
    }

    Serial.println("===============");
}

// Helper functions
const char* getPresenceStateString(RD03PresenceState state) {
    switch (state) {
        case RD03PresenceState::NO_PRESENCE: return "NO PRESENCE";
        case RD03PresenceState::PRESENCE_DETECTED: return "PRESENCE DETECTED";
        case RD03PresenceState::MOTION_DETECTED: return "MOTION DETECTED";
        case RD03PresenceState::MAINTAINING: return "MAINTAINING";
        case RD03PresenceState::FAST_EXIT: return "FAST EXIT";
        case RD03PresenceState::SAFETY_TIMEOUT: return "SAFETY TIMEOUT";
        default: return "UNKNOWN";
    }
}

const char* getStatusString(RD03Status status) {
    switch (status) {
        case RD03Status::OK: return "OK";
        case RD03Status::ERROR: return "ERROR";
        case RD03Status::NO_SIGNAL: return "NO SIGNAL";
        case RD03Status::BUFFER_OVERFLOW: return "BUFFER OVERFLOW";
        case RD03Status::INVALID_DATA: return "INVALID DATA";
        case RD03Status::WATCHDOG_RESET: return "WATCHDOG RESET";
        default: return "UNKNOWN";
    }
}
