#ifndef RD03RADAR_H
#define RD03RADAR_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <vector>
#include <functional>

// ============================================================================
// RD03Radar Library for Arduino
// Smart Presence Detection System for Ai-Thinker RD-03 24GHz mmWave Radar
//
// Author: Mohamed Eid (gomgom-40)
// Version: 1.1.0
// License: MIT
// ============================================================================

// ============================================================================
// Configuration Structures
// ============================================================================
struct RD03Config {
    float minRange = 20.0f;
    float maxRange = 500.0f;
    uint8_t sensitivity = 3;
    uint16_t holdTime = 30;
    uint16_t maxAbsenceTime = 300;
    float motionThreshold = 2.0f;
    uint8_t motionHitsRequired = 1;
    uint32_t baudRate = 115200;
    uint8_t rxBufferSize = 255;  // غيرناه لـ 255 عشان نتجنب narrowing error
};

// ============================================================================
// Status & State Enumerations
// ============================================================================
enum class RD03Status {
    OK, ERROR, NO_SIGNAL, BUFFER_OVERFLOW, INVALID_DATA, WATCHDOG_RESET
};

enum class RD03PresenceState {
    NO_PRESENCE, PRESENCE_DETECTED, MOTION_DETECTED, MAINTAINING, FAST_EXIT, SAFETY_TIMEOUT
};

enum class RD03ControlMode {
    AUTOMATIC, MANUAL_ON, FORCE_ON, FORCE_OFF
};

// ============================================================================
// Callback Types (always available)
// ============================================================================
using PresenceCallback = std::function<void(RD03PresenceState, float)>;
using StatusCallback   = std::function<void(RD03Status, const char*)>;
using DistanceCallback = std::function<void(float, bool)>;
using LightControlCallback = std::function<void(bool, const char*)>;

// ============================================================================
// Main Class
// ============================================================================
class RD03Radar {
public:
    // Constructors
    RD03Radar(HardwareSerial& serial, const RD03Config& config = RD03Config());
    RD03Radar(Stream& serial, const RD03Config& config = RD03Config());
    ~RD03Radar();

    // Core methods (always available)
    bool begin(int rxPin = -1, int txPin = -1);
    bool begin();
    void end();
    void loop();

    // Configuration
    void setConfig(const RD03Config& config);
    RD03Config getConfig() const;
    void setRange(float minRange, float maxRange);
    void setSensitivity(uint8_t sensitivity);
    void setHoldTime(uint16_t holdTimeSeconds);

    // Control & State
    void setControlMode(RD03ControlMode mode);
    RD03ControlMode getControlMode() const;
    void manualLightControl(bool turnOn);
    void resetPresence();

    // Status & Readings
    RD03PresenceState getPresenceState() const;
    String getPresenceStateString() const;
    float getDistance() const;
    RD03Status getStatus() const;
    bool isOperational() const;
    uint32_t getUptime() const;
    uint32_t getLastActivityTime() const;

    // Callbacks (always available)
    void onPresenceChange(PresenceCallback cb);
    void onStatusChange(StatusCallback cb);
    void onDistanceMeasurement(DistanceCallback cb);
    void onLightControl(LightControlCallback cb);

    // Utility
    void resetRadar();
    static const char* getVersion();
    static const char* getInfo();

    // ──────────────────────────────────────────────
    //          MQTT Support – Optional Feature
    // ──────────────────────────────────────────────
#if defined(RD03_ENABLE_MQTT)

    void setupMQTT(const char* server, uint16_t port = 1883,
                   const char* username = nullptr, const char* password = nullptr);
    void connectMQTT();
    void disconnectMQTT();
    bool isMQTTConnected();
    void publishStatus();
    void subscribeCommands();
    void setMQTTCallback(std::function<void(char*, uint8_t*, unsigned int)> cb);

private:
    void handleMQTTMessage(char* topic, uint8_t* payload, unsigned int length);

    WiFiClient _wifiClient;
    PubSubClient _mqttClient;
    String _mqttServer;
    uint16_t _mqttPort;
    String _mqttUsername;
    String _mqttPassword;
    bool _mqttEnabled = false;
    std::function<void(char*, uint8_t*, unsigned int)> _mqttCallback;

#endif  // RD03_ENABLE_MQTT

    // ──────────────────────────────────────────────
    //       WebServer Support – Optional Feature
    // ──────────────────────────────────────────────
#if defined(RD03_ENABLE_WEBSERVER)

public:
    void setupWebServer(uint16_t port = 80);
    void startWebServer();
    void stopWebServer();
    bool isWebServerRunning() const;

private:
    void handleWebRoot();
    void handleAPIStatus();
    void handleAPICommand();
    void handleWebNotFound();

    uint16_t _webPort;
    bool _webServerEnabled = false;

#if defined(ESP32)
    WebServer _webServer;
#elif defined(ESP8266)
    ESP8266WebServer _webServer;
#endif

#endif  // RD03_ENABLE_WEBSERVER

private:
    // Common private members
    HardwareSerial* _hardwareSerial = nullptr;
    Stream* _stream = nullptr;
    bool _usingHardwareSerial = false;
    RD03Config _config;
    RD03PresenceState _presenceState = RD03PresenceState::NO_PRESENCE;
    RD03ControlMode _controlMode = RD03ControlMode::AUTOMATIC;
    RD03Status _status = RD03Status::OK;

    uint32_t _startTime = 0;
    uint32_t _lastActivityTime = 0;
    uint32_t _lastPublishTime = 0;
    uint32_t _radarLastSeenTime = 0;
    uint32_t _watchdogActivityTime = 0;
    uint32_t _noTargetSince = 0;

    float _lastValidDistance = 0.0f;
    float _lastDistanceForMotion = 0.0f;
    uint8_t _motionHits = 0;

    bool _presenceActive = false;
    bool _manualOffRecent = false;
    bool _initialized = false;

    std::vector<uint8_t> _uartBuffer;
    uint32_t _lastByteTime = 0;

    PresenceCallback _presenceCallback;
    StatusCallback   _statusCallback;
    DistanceCallback _distanceCallback;
    LightControlCallback _lightCallback;

    // Private helpers
    Stream* getSerial();
    String processUART();
    bool initializeRadar();
    void sendResetCommands();
    void clearUARTBuffer();

    void handleEntryDetection(float distance);
    void handlePresenceMaintenance(float distance);
    void updatePresenceState(RD03PresenceState newState, float distance = 0.0f);
    void updateStatus(RD03Status newStatus, const char* message = "");
    void handleLightControl();
    void watchdogCheck();

    const char* statusToString(RD03Status s) const;
    const char* presenceStateToString(RD03PresenceState s) const;
};

#endif // RD03RADAR_H