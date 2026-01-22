#ifndef RD03RADAR_H
#define RD03RADAR_H

#include <Arduino.h>
#include <vector>
#include <functional>

// =========================
// Platform specific serial
// =========================
#if defined(ESP32)
  #include <HardwareSerial.h>
#endif

// =========================
// Optional features flags
// =========================
// Enabled by examples, not by default
// #define RD03_ENABLE_MQTT
// #define RD03_ENABLE_WEBSERVER

// =========================
// MQTT Support (optional)
// =========================
#if defined(RD03_ENABLE_MQTT)
  #if defined(ESP32) || defined(ESP8266)
    #include <WiFiClient.h>
    #include <PubSubClient.h>
  #else
    #error "MQTT only supported on ESP32 / ESP8266"
  #endif
#endif

// =========================
// WebServer Support (optional)
// =========================
#if defined(RD03_ENABLE_WEBSERVER)
  #if defined(ESP32)
    #include <WebServer.h>
  #elif defined(ESP8266)
    #include <ESP8266WebServer.h>
  #else
    #error "WebServer only supported on ESP32 / ESP8266"
  #endif
#endif

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
    uint8_t rxBufferSize = 256;
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
// Callbacks
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
    RD03Radar(HardwareSerial& serial, const RD03Config& config = RD03Config());
    RD03Radar(Stream& serial, const RD03Config& config = RD03Config());
    ~RD03Radar();

    bool begin(int rxPin, int txPin);
    bool begin();
    void end();

    void setConfig(const RD03Config& config);
    RD03Config getConfig() const;
    void setRange(float minRange, float maxRange);
    void setSensitivity(uint8_t sensitivity);
    void setHoldTime(uint16_t holdTimeSeconds);

    void setControlMode(RD03ControlMode mode);
    RD03ControlMode getControlMode() const;
    void manualLightControl(bool turnOn);
    void resetPresence();

    void loop();

    RD03PresenceState getPresenceState() const;
    String getPresenceStateString() const;
    float getDistance() const;
    RD03Status getStatus() const;
    bool isOperational() const;
    uint32_t getUptime() const;
    uint32_t getLastActivityTime() const;

    void onPresenceChange(PresenceCallback callback);
    void onStatusChange(StatusCallback callback);
    void onDistanceMeasurement(DistanceCallback callback);
    void onLightControl(LightControlCallback callback);

    void resetRadar();
    static const char* getVersion();
    static const char* getInfo();

    // ================= MQTT =================
    #if defined(RD03_ENABLE_MQTT)
      void setupMQTT(const char* server, uint16_t port = 1883,
                     const char* username = nullptr, const char* password = nullptr);
      void connectMQTT();
      void disconnectMQTT();
      bool isMQTTConnected();
      void publishStatus();
      void subscribeCommands();
      void setMQTTCallback(std::function<void(char*, uint8_t*, unsigned int)> callback);
    #endif

    // ================= WebServer =================
    #if defined(RD03_ENABLE_WEBSERVER)
      void setupWebServer(uint16_t port = 80);
      void startWebServer();
      void stopWebServer();
      bool isWebServerRunning() const;
    #endif

private:
    Stream* getSerial();

    HardwareSerial* _hardwareSerial = nullptr;
    Stream* _softwareSerial = nullptr;
    bool _isHardwareSerial = false;

    RD03Config _config;
    RD03PresenceState _presenceState;
    RD03ControlMode _controlMode;
    RD03Status _status;

    uint32_t _startTime;
    uint32_t _lastActivityTime;
    uint32_t _lastPublishTime;
    uint32_t _radarLastSeenTime;
    uint32_t _watchdogActivityTime;
    uint32_t _noTargetSince;

    float _lastValidDistance;
    float _lastDistanceForMotion;
    uint8_t _motionHits;

    bool _presenceActive;
    bool _manualOffRecent;
    bool _initialized;

    // ================= MQTT =================
    #if defined(RD03_ENABLE_MQTT)
      WiFiClient _wifiClient;
      PubSubClient _mqttClient;
      String _mqttServer;
      uint16_t _mqttPort;
      String _mqttUsername;
      String _mqttPassword;
      bool _mqttEnabled;
      std::function<void(char*, uint8_t*, unsigned int)> _mqttCallback;
    #endif

    // ================= WebServer =================
    #if defined(RD03_ENABLE_WEBSERVER)
      uint16_t _webPort;
      bool _webServerEnabled;
      #if defined(ESP32)
        WebServer _webServer;
      #elif defined(ESP8266)
        ESP8266WebServer _webServer;
      #endif
    #endif

    std::vector<uint8_t> _uartBuffer;
    uint32_t _lastByteTime;

    PresenceCallback _presenceCallback;
    StatusCallback _statusCallback;
    DistanceCallback _distanceCallback;
    LightControlCallback _lightCallback;

    String processUART();
    bool initializeRadar();
    void sendResetCommands();
    void clearUARTBuffer();

    void handleWebRoot();
    void handleAPIStatus();
    void handleAPICommand();
    void handleWebNotFound();
    void handleMQTTMessage(char* topic, uint8_t* payload, unsigned int length);

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
};

#endif
