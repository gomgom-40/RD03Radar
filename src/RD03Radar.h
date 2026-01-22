#ifndef RD03RADAR_H
#define RD03RADAR_H

#include <Arduino.h>
#include <vector>
#include <functional>

// ──────────────────────────────────────────────
// IMPORTANT: For MQTT examples to compile without linker errors on ESP8266
// Uncomment the line below during development/testing of MQTT examples
#define RD03_ENABLE_MQTT  // <--- Uncomment this line to force-enable MQTT

// ──────────────────────────────────────────────
// Platform-specific includes
// ──────────────────────────────────────────────
#if defined(ESP32)
  #include <HardwareSerial.h>
#endif

// ──────────────────────────────────────────────
// Optional Features
// ──────────────────────────────────────────────
// MQTT (enable by defining RD03_ENABLE_MQTT in this file or .ino)
#if defined(RD03_ENABLE_MQTT)
  #if defined(ESP32) || defined(ESP8266)
    #include <WiFiClient.h>
    #include <PubSubClient.h>
  #else
    #error "MQTT is only supported on ESP32 or ESP8266"
  #endif
#endif

// WebServer (enable by defining RD03_ENABLE_WEBSERVER in this file or .ino)
#if defined(RD03_ENABLE_WEBSERVER)
  #if defined(ESP32)
    #include <WebServer.h>
  #elif defined(ESP8266)
    #include <ESP8266WebServer.h>
  #else
    #error "WebServer is only supported on ESP32 or ESP8266"
  #endif
#endif

// ============================================================================
// Configuration Structure
// ============================================================================
struct RD03Config {
    float    minRange           = 20.0f;   // cm
    float    maxRange           = 500.0f;  // cm
    uint8_t  sensitivity        = 3;       // 1 (high) to 5 (low)
    uint16_t holdTime           = 30;      // seconds
    uint16_t maxAbsenceTime     = 300;     // seconds
    float    motionThreshold    = 2.0f;    // cm
    uint8_t  motionHitsRequired = 1;       // consecutive detections
    uint32_t baudRate           = 115200;
    uint8_t  rxBufferSize       = 255;     // max safe value for uint8_t
};

// ============================================================================
// Enums
// ============================================================================
enum class RD03Status {
    OK,
    ERROR,
    NO_SIGNAL,
    BUFFER_OVERFLOW,
    INVALID_DATA,
    WATCHDOG_RESET
};

enum class RD03PresenceState {
    NO_PRESENCE,
    PRESENCE_DETECTED,
    MOTION_DETECTED,
    MAINTAINING,
    FAST_EXIT,
    SAFETY_TIMEOUT
};

enum class RD03ControlMode {
    AUTOMATIC,
    MANUAL_ON,
    FORCE_ON,
    FORCE_OFF
};

// ============================================================================
// Callback Types
// ============================================================================
using PresenceCallback     = std::function<void(RD03PresenceState, float)>;
using StatusCallback       = std::function<void(RD03Status, const char*)>;
using DistanceCallback     = std::function<void(float, bool)>;
using LightControlCallback = std::function<void(bool, const char*)>;

// ============================================================================
// Main RD03Radar Class
// ============================================================================
class RD03Radar {
public:
    RD03Radar(HardwareSerial& serial, const RD03Config& config = RD03Config());
    RD03Radar(Stream& serial, const RD03Config& config = RD03Config());
    ~RD03Radar();

    // Initialization
    bool begin();                           // For SoftwareSerial / Stream (ESP8266)
    bool begin(int rxPin, int txPin);       // For HardwareSerial (ESP32)
    void end();

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

    // Core
    void loop();

    // Status & Readings
    RD03PresenceState getPresenceState() const;
    String getPresenceStateString() const;
    float getDistance() const;
    RD03Status getStatus() const;
    bool isOperational() const;
    uint32_t getUptime() const;
    uint32_t getLastActivityTime() const;

    // Callbacks
    void onPresenceChange(PresenceCallback cb);
    void onStatusChange(StatusCallback cb);
    void onDistanceMeasurement(DistanceCallback cb);
    void onLightControl(LightControlCallback cb);

    // Utility
    void resetRadar();
    static const char* getVersion();
    static const char* getInfo();

    // MQTT Support (enabled if RD03_ENABLE_MQTT defined)
#if defined(RD03_ENABLE_MQTT)
    void setupMQTT(const char* server, uint16_t port = 1883,
                   const char* username = nullptr, const char* password = nullptr);
    void connectMQTT();
    void disconnectMQTT();
    bool isMQTTConnected();
    void publishStatus();
    void subscribeCommands();
    void setMQTTCallback(std::function<void(char*, uint8_t*, unsigned int)> cb);
#endif

    // WebServer Support (enabled if RD03_ENABLE_WEBSERVER defined)
#if defined(RD03_ENABLE_WEBSERVER)
    void setupWebServer(uint16_t port = 80);
    void startWebServer();
    void stopWebServer();
    bool isWebServerRunning() const;
#endif

private:
    HardwareSerial* _hardwareSerial = nullptr;
    Stream*         _stream         = nullptr;
    bool            _isHardwareSerial = false;

    RD03Config _config;

    RD03PresenceState _presenceState = RD03PresenceState::NO_PRESENCE;
    RD03ControlMode   _controlMode   = RD03ControlMode::AUTOMATIC;
    RD03Status        _status        = RD03Status::OK;

    uint32_t _startTime = 0;
    uint32_t _lastActivityTime = 0;
    uint32_t _lastPublishTime  = 0;
    uint32_t _radarLastSeenTime = 0;
    uint32_t _watchdogActivityTime = 0;
    uint32_t _noTargetSince = 0;

    float    _lastValidDistance = 0.0f;
    float    _lastDistanceForMotion = 0.0f;
    uint8_t  _motionHits = 0;

    bool _presenceActive  = false;
    bool _manualOffRecent = false;
    bool _initialized     = false;

    std::vector<uint8_t> _uartBuffer;
    uint32_t _lastByteTime = 0;

    PresenceCallback     _presenceCallback;
    StatusCallback       _statusCallback;
    DistanceCallback     _distanceCallback;
    LightControlCallback _lightCallback;

#if defined(RD03_ENABLE_MQTT)
    WiFiClient _wifiClient;
    PubSubClient _mqttClient;
    String _mqttServer;
    uint16_t _mqttPort = 1883;
    String _mqttUsername;
    String _mqttPassword;
    bool _mqttEnabled = false;
    std::function<void(char*, uint8_t*, unsigned int)> _mqttCallback;
#endif

#if defined(RD03_ENABLE_WEBSERVER)
    uint16_t _webPort = 80;
    bool _webServerEnabled = false;
#if defined(ESP32)
    WebServer _webServer;
#elif defined(ESP8266)
    ESP8266WebServer _webServer;
#endif
#endif

    // Private helpers
    Stream* getSerial() { return _stream; }

    String processUART();
    bool initializeRadar();
    void sendResetCommands();
    void clearUARTBuffer();

    float extractDistance(const String& message);
    bool isValidDistance(float distance);

    void handleEntryDetection(float distance);
    void handlePresenceMaintenance(float distance);
    void updatePresenceState(RD03PresenceState newState, float distance = 0.0f);
    void updateStatus(RD03Status newStatus, const char* message = "");

    void handleLightControl();
    void watchdogCheck();

    const char* statusToString(RD03Status status) const;
    const char* presenceStateToString(RD03PresenceState state) const;

#if defined(RD03_ENABLE_MQTT)
    void handleMQTTMessage(char* topic, uint8_t* payload, unsigned int length);
#endif

#if defined(RD03_ENABLE_WEBSERVER)
    void handleWebRoot();
    void handleAPIStatus();
    void handleAPICommand();
    void handleWebNotFound();
#endif
};

#endif // RD03RADAR_H