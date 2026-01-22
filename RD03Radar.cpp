#include "RD03Radar.h"

// ============================================================================
// RD03Radar Implementation
// Smart Presence Detection System for Ai-Thinker RD-03 24GHz mmWave Radar
//
// Author: Mohamed Eid (gomgom-40)
// Version: 1.0.0
// License: MIT
// ============================================================================

// ============================================================================
// Constants
// ============================================================================

#define RADAR_INIT_DELAY_MS     3000   // Delay after power-on before initialization
#define UART_STALE_TIMEOUT_MS   100    // UART buffer stale timeout
#define MAX_BUFFER_SIZE         256    // Maximum UART buffer size
#define BUFFER_CLEAR_TIMEOUT_MS 30000  // Buffer clear timeout
#define PUBLISH_INTERVAL_MS     40     // Minimum publish interval
#define FAST_EXIT_TIMEOUT_MS    15000  // Fast exit timeout (15 seconds)
#define WATCHDOG_SOFT_RESET_MS  90000  // Soft reset after 90 seconds silence
#define WATCHDOG_HARD_RESET_MS  180000 // Hard reset after 180 seconds silence
#define LONG_SILENCE_RESET_MS   43200000UL // 12 hours in milliseconds

// Radar initialization commands
const uint8_t RADAR_INIT_CMD1[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
const uint8_t RADAR_INIT_CMD2[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};

// ============================================================================
// Constructor & Destructor
// ============================================================================

RD03Radar::RD03Radar(HardwareSerial& serial, const RD03Config& config)
    : _serial(serial)
    , _config(config)
    , _presenceState(RD03PresenceState::NO_PRESENCE)
    , _controlMode(RD03ControlMode::AUTOMATIC)
    , _status(RD03Status::OK)
    , _startTime(0)
    , _lastActivityTime(0)
    , _lastPublishTime(0)
    , _radarLastSeenTime(0)
    , _watchdogActivityTime(0)
    , _noTargetSince(0)
    , _lastValidDistance(0.0f)
    , _lastDistanceForMotion(0.0f)
    , _motionHits(0)
    , _presenceActive(false)
    , _manualOffRecent(false)
    #if defined(ESP32) || defined(ESP8266)
    , _mqttClient(_wifiClient)
    , _mqttEnabled(false)
    , _webServer(_webPort)
    , _webServerEnabled(false)
    #endif
    , _initialized(false)
    , _lastByteTime(0)
{
    // Initialize UART buffer
    _uartBuffer.reserve(MAX_BUFFER_SIZE);
}

RD03Radar::~RD03Radar() {
    end();
}

// ============================================================================
// Initialization & Setup
// ============================================================================

bool RD03Radar::begin(int rxPin, int txPin) {
    if (_initialized) {
        return true;
    }

    // Initialize serial with hardware pins
    _serial.begin(_config.baudRate, SERIAL_8N1, rxPin, txPin);

    // Wait for radar to power up
    delay(RADAR_INIT_DELAY_MS);

    // Initialize radar
    if (initializeRadar()) {
        _initialized = true;
        _startTime = millis();
        updateStatus(RD03Status::OK, "Radar initialized successfully");
        return true;
    }

    updateStatus(RD03Status::ERROR, "Failed to initialize radar");
    return false;
}

bool RD03Radar::begin() {
    if (_initialized) {
        return true;
    }

    // Initialize serial (assumes it's already configured)
    _serial.begin(_config.baudRate);

    // Wait for radar to power up
    delay(RADAR_INIT_DELAY_MS);

    // Initialize radar
    if (initializeRadar()) {
        _initialized = true;
        _startTime = millis();
        updateStatus(RD03Status::OK, "Radar initialized successfully");
        return true;
    }

    updateStatus(RD03Status::ERROR, "Failed to initialize radar");
    return false;
}

void RD03Radar::end() {
    if (_initialized) {
        _serial.end();
        _initialized = false;
        updateStatus(RD03Status::OK, "Radar stopped");
    }
}

bool RD03Radar::initializeRadar() {
    // Send initialization commands
    sendResetCommands();

    // Wait for response
    delay(100);

    // Check if radar responds
    String response = processUART();
    if (response.length() > 0) {
        return true;
    }

    return false;
}

// ============================================================================
// Configuration Methods
// ============================================================================

void RD03Radar::setConfig(const RD03Config& config) {
    _config = config;
    updateStatus(RD03Status::OK, "Configuration updated");
}

RD03Config RD03Radar::getConfig() const {
    return _config;
}

void RD03Radar::setRange(float minRange, float maxRange) {
    _config.minRange = minRange;
    _config.maxRange = maxRange;
    updateStatus(RD03Status::OK, "Range updated");
}

void RD03Radar::setSensitivity(uint8_t sensitivity) {
    if (sensitivity >= 1 && sensitivity <= 5) {
        _config.sensitivity = sensitivity;
        updateStatus(RD03Status::OK, "Sensitivity updated");
    }
}

void RD03Radar::setHoldTime(uint16_t holdTimeSeconds) {
    _config.holdTime = holdTimeSeconds;
    updateStatus(RD03Status::OK, "Hold time updated");
}

// ============================================================================
// Control Methods
// ============================================================================

void RD03Radar::setControlMode(RD03ControlMode mode) {
    _controlMode = mode;
    updateStatus(RD03Status::OK, "Control mode changed");
}

RD03ControlMode RD03Radar::getControlMode() const {
    return _controlMode;
}

void RD03Radar::manualLightControl(bool turnOn) {
    _manualOffRecent = !turnOn;
    _controlMode = turnOn ? RD03ControlMode::MANUAL_ON : RD03ControlMode::AUTOMATIC;

    if (_lightCallback) {
        const char* reason = turnOn ? "Manual ON" : "Manual OFF";
        _lightCallback(turnOn, reason);
    }

    updateStatus(RD03Status::OK, turnOn ? "Manual ON activated" : "Manual OFF activated");
}

void RD03Radar::resetPresence() {
    _presenceActive = false;
    _motionHits = 0;
    _lastDistanceForMotion = 0.0f;
    _noTargetSince = 0;
    _lastActivityTime = 0;
    updatePresenceState(RD03PresenceState::NO_PRESENCE);
    updateStatus(RD03Status::OK, "Presence reset");
}

// ============================================================================
// Main Processing Method
// ============================================================================

void RD03Radar::loop() {
    if (!_initialized) {
        return;
    }

    uint32_t now = millis();

    // Process UART data
    String message = processUART();

    // Extract distance if message received
    float distance = 0.0f;
    if (message.length() > 0) {
        distance = extractDistance(message);
        _radarLastSeenTime = now;
        _watchdogActivityTime = now;

        // Call distance callback
        if (_distanceCallback) {
            _distanceCallback(distance, isValidDistance(distance));
        }
    }

    // Handle presence detection logic
    if (_controlMode == RD03ControlMode::AUTOMATIC) {
        if (!_presenceActive) {
            handleEntryDetection(distance);
        } else {
            handlePresenceMaintenance(distance);
        }
    }

    // Handle light control
    handleLightControl();

    // Watchdog monitoring
    watchdogCheck();
}

// ============================================================================
// Status & Information Methods
// ============================================================================

RD03PresenceState RD03Radar::getPresenceState() const {
    return _presenceState;
}

float RD03Radar::getDistance() const {
    return _lastValidDistance;
}

RD03Status RD03Radar::getStatus() const {
    return _status;
}

// ============================================================================
// MQTT Support Implementation (ESP32/ESP8266 only)
// ============================================================================
#if defined(ESP32) || defined(ESP8266)

void RD03Radar::setupMQTT(const char* server, uint16_t port, const char* username, const char* password) {
    _mqttServer = server;
    _mqttPort = port;
    _mqttUsername = username ? username : "";
    _mqttPassword = password ? password : "";
    _mqttEnabled = true;

    _mqttClient.setServer(server, port);
    _mqttClient.setCallback([this](char* topic, uint8_t* payload, unsigned int length) {
        this->handleMQTTMessage(topic, payload, length);
    });
}

void RD03Radar::connectMQTT() {
    if (!_mqttEnabled || _mqttServer.isEmpty()) return;

    String clientId = "RD03Radar-";
    clientId += String(random(0xffff), HEX);

    bool connected = false;
    if (_mqttUsername.length() > 0) {
        connected = _mqttClient.connect(clientId.c_str(), _mqttUsername.c_str(), _mqttPassword.c_str());
    } else {
        connected = _mqttClient.connect(clientId.c_str());
    }

    if (connected) {
        ESP_LOGI("MQTT", "Connected to MQTT broker");
        subscribeCommands();
        publishStatus();
    } else {
        ESP_LOGW("MQTT", "Failed to connect to MQTT broker");
    }
}

void RD03Radar::disconnectMQTT() {
    if (_mqttClient.connected()) {
        _mqttClient.disconnect();
        ESP_LOGI("MQTT", "Disconnected from MQTT broker");
    }
}

bool RD03Radar::isMQTTConnected() const {
    return _mqttClient.connected();
}

void RD03Radar::publishStatus() {
    if (!_mqttClient.connected()) return;

    String jsonMessage = "{";
    jsonMessage += "\"presence\":" + String(_presenceActive ? "true" : "false") + ",";
    jsonMessage += "\"distance\":" + String(_lastValidDistance, 1) + ",";
    jsonMessage += "\"state\":\"" + getPresenceStateString() + "\",";
    jsonMessage += "\"uptime\":" + String(getUptime()) + ",";
    jsonMessage += "\"operational\":" + String(isOperational() ? "true" : "false");
    jsonMessage += "}";

    _mqttClient.publish("rd03radar/status", jsonMessage.c_str());
}

void RD03Radar::subscribeCommands() {
    if (_mqttClient.connected()) {
        _mqttClient.subscribe("rd03radar/commands");
    }
}

void RD03Radar::setMQTTCallback(std::function<void(char*, uint8_t*, unsigned int)> callback) {
    _mqttCallback = callback;
    _mqttClient.setCallback(callback);
}

void RD03Radar::handleMQTTMessage(char* topic, uint8_t* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    ESP_LOGI("MQTT", "Received command: %s", message.c_str());

    if (message == "force_on") {
        setControlMode(RD03ControlMode::FORCE_ON);
    } else if (message == "force_off") {
        setControlMode(RD03ControlMode::FORCE_OFF);
    } else if (message == "automatic") {
        setControlMode(RD03ControlMode::AUTOMATIC);
    } else if (message == "reset") {
        resetPresence();
    }
}

#endif // ESP32 || ESP8266

// ============================================================================
// Web Server Support Implementation (ESP32/ESP8266 only)
// ============================================================================
#if defined(ESP32) || defined(ESP8266)

void RD03Radar::setupWebServer(uint16_t port) {
    _webPort = port;
    _webServerEnabled = true;

    _webServer.on("/", HTTP_GET, [this]() { handleWebRoot(); });
    _webServer.on("/api/status", HTTP_GET, [this]() { handleAPIStatus(); });
    _webServer.on("/api/command", HTTP_POST, [this]() { handleAPICommand(); });
    _webServer.onNotFound([this]() { handleWebNotFound(); });
}

void RD03Radar::startWebServer() {
    if (_webServerEnabled) {
        _webServer.begin();
        ESP_LOGI("WebServer", "Web server started on port %d", _webPort);
    }
}

void RD03Radar::stopWebServer() {
    if (_webServerEnabled) {
        _webServer.stop();
        ESP_LOGI("WebServer", "Web server stopped");
    }
}

bool RD03Radar::isWebServerRunning() const {
    return _webServerEnabled;
}

void RD03Radar::handleWebRoot() {
    String html = "<!DOCTYPE html><html><head><title>RD03Radar Control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>body{font-family:Arial;text-align:center;margin:20px;}";
    html += ".status{color:" + String(_presenceActive ? "green" : "red") + ";}";
    html += "button{margin:10px;padding:10px 20px;font-size:16px;}</style>";
    html += "</head><body><h1>RD03Radar Control Panel</h1>";

    html += "<div class='status'><h2>Status: " + String(_presenceActive ? "PRESENCE DETECTED" : "NO PRESENCE") + "</h2></div>";
    html += "<p>Distance: " + String(_lastValidDistance, 1) + " cm</p>";
    html += "<p>State: " + getPresenceStateString() + "</p>";
    html += "<p>Uptime: " + String(getUptime()) + " seconds</p>";

    html += "<button onclick=\"sendCommand('automatic')\">Automatic</button>";
    html += "<button onclick=\"sendCommand('force_on')\">Force ON</button>";
    html += "<button onclick=\"sendCommand('force_off')\">Force OFF</button>";
    html += "<button onclick=\"sendCommand('reset')\">Reset</button>";

    html += "<script>function sendCommand(cmd){fetch('/api/command',{method:'POST',body:cmd});location.reload();}</script>";
    html += "</body></html>";

    _webServer.send(200, "text/html", html);
}

void RD03Radar::handleAPIStatus() {
    String json = "{";
    json += "\"presence\":" + String(_presenceActive ? "true" : "false") + ",";
    json += "\"distance\":" + String(_lastValidDistance, 1) + ",";
    json += "\"state\":\"" + getPresenceStateString() + "\",";
    json += "\"uptime\":" + String(getUptime()) + ",";
    json += "\"operational\":" + String(isOperational() ? "true" : "false");
    json += "}";

    _webServer.send(200, "application/json", json);
}

void RD03Radar::handleAPICommand() {
    if (_webServer.hasArg("plain")) {
        String command = _webServer.arg("plain");
        ESP_LOGI("WebAPI", "Received command: %s", command.c_str());

        if (command == "automatic") {
            setControlMode(RD03ControlMode::AUTOMATIC);
        } else if (command == "force_on") {
            setControlMode(RD03ControlMode::FORCE_ON);
        } else if (command == "force_off") {
            setControlMode(RD03ControlMode::FORCE_OFF);
        } else if (command == "reset") {
            resetPresence();
        }

        _webServer.send(200, "text/plain", "OK");
    } else {
        _webServer.send(400, "text/plain", "Bad Request");
    }
}

void RD03Radar::handleWebNotFound() {
    _webServer.send(404, "text/plain", "Not Found");
}

#endif // ESP32 || ESP8266
}

bool RD03Radar::isOperational() const {
    if (!_initialized) return false;
    uint32_t now = millis();
    return (now - _radarLastSeenTime) < 10000; // OK if seen within 10 seconds
}

uint32_t RD03Radar::getUptime() const {
    if (!_initialized) return 0;
    return millis() - _startTime;
}

uint32_t RD03Radar::getLastActivityTime() const {
    return _lastActivityTime;
}

// ============================================================================
// Callback Registration
// ============================================================================

void RD03Radar::onPresenceChange(PresenceCallback callback) {
    _presenceCallback = callback;
}

void RD03Radar::onStatusChange(StatusCallback callback) {
    _statusCallback = callback;
}

void RD03Radar::onDistanceMeasurement(DistanceCallback callback) {
    _distanceCallback = callback;
}

void RD03Radar::onLightControl(LightControlCallback callback) {
    _lightCallback = callback;
}

// ============================================================================
// Utility Methods
// ============================================================================

void RD03Radar::resetRadar() {
    sendResetCommands();
    clearUARTBuffer();
    _radarLastSeenTime = 0;
    _watchdogActivityTime = 0;
    updateStatus(RD03Status::WATCHDOG_RESET, "Radar reset initiated");
}

const char* RD03Radar::getVersion() {
    return "1.0.0";
}

const char* RD03Radar::getInfo() {
    return "RD03Radar Library v1.0.0 - Smart Presence Detection for Ai-Thinker RD-03";
}

// ============================================================================
// Private Methods - UART Processing
// ============================================================================

String RD03Radar::processUART() {
    uint8_t byte;
    bool newLineFound = false;

    // Read available bytes
    while (_serial.available() && _uartBuffer.size() < MAX_BUFFER_SIZE) {
        if (_serial.readBytes(&byte, 1) > 0) {
            uint32_t now = millis();

            // Clear stale data if timeout exceeded
            if (_lastByteTime && (now - _lastByteTime > UART_STALE_TIMEOUT_MS)) {
                _uartBuffer.clear();
            }

            _lastByteTime = now;
            _uartBuffer.push_back(byte);

            if (byte == '\n') {
                newLineFound = true;
                break;
            }
        }
    }

    // Process complete line
    if (newLineFound && !_uartBuffer.empty()) {
        String line = String((char*)_uartBuffer.data(), _uartBuffer.size());
        _uartBuffer.clear();

        // Trim whitespace and remove carriage returns
        line.trim();
        line.replace("\r", "");

        if (line.length() > 0) {
            return line;
        }
    }

    // Buffer overflow protection
    if (_uartBuffer.size() >= MAX_BUFFER_SIZE) {
        _uartBuffer.clear();
        updateStatus(RD03Status::BUFFER_OVERFLOW, "UART buffer overflow");
    }

    // Clear stale buffer
    if (_lastByteTime && (millis() - _lastByteTime > BUFFER_CLEAR_TIMEOUT_MS) && !_uartBuffer.empty()) {
        _uartBuffer.clear();
    }

    return "";
}

float RD03Radar::extractDistance(const String& message) {
    // Check if message starts with "Range "
    if (!message.startsWith("Range ")) {
        return 0.0f;
    }

    // Find first numeric character
    int pos = message.indexOf("0123456789", 6); // Start after "Range "
    if (pos == -1) {
        return 0.0f;
    }

    // Extract numeric part
    String numStr = message.substring(pos);
    char* endPtr;
    float distance = strtof(numStr.c_str(), &endPtr);

    // Validate conversion
    return (endPtr != numStr.c_str()) ? distance : 0.0f;
}

bool RD03Radar::isValidDistance(float distance) {
    if (distance < _config.minRange || distance > _config.maxRange) {
        return false;
    }

    // Prevent false readings (large jumps)
    if (_lastValidDistance > 100.0f && distance < 10.0f) {
        return false;
    }

    return true;
}

// ============================================================================
// Private Methods - Presence Detection Logic
// ============================================================================

void RD03Radar::handleEntryDetection(float distance) {
    uint32_t now = millis();

    // Reset state after manual turn-off
    if (_manualOffRecent) {
        _lastValidDistance = 0.0f;
        _noTargetSince = now;
        _manualOffRecent = false;
    }

    if (isValidDistance(distance)) {
        // Check for significant distance change (motion)
        if (_lastDistanceForMotion == 0.0f ||
            fabs(distance - _lastDistanceForMotion) >= _config.motionThreshold) {
            _motionHits++;
            _lastDistanceForMotion = distance;
        }

        // Activate presence when motion threshold reached
        if (_motionHits >= _config.motionHitsRequired) {
            _presenceActive = true;
            _lastActivityTime = now;
            _noTargetSince = 0;
            _motionHits = 0;
            updatePresenceState(RD03PresenceState::MOTION_DETECTED, distance);
        }
    } else {
        // Reset motion detection on invalid readings
        _motionHits = 0;
        _lastDistanceForMotion = 0.0f;
    }
}

void RD03Radar::handlePresenceMaintenance(float distance) {
    uint32_t now = millis();

    if (isValidDistance(distance)) {
        _lastValidDistance = distance;
        _noTargetSince = 0; // Reset no-target timer

        // Update activity timestamp (only in automatic mode)
        if (_controlMode == RD03ControlMode::AUTOMATIC) {
            _lastActivityTime = now;
        }

        updatePresenceState(RD03PresenceState::MAINTAINING, distance);
    } else {
        // Start counting time without target
        if (_noTargetSince == 0) {
            _noTargetSince = now;
        }
    }

    // Rate limiting for publishing
    if (now - _lastPublishTime < PUBLISH_INTERVAL_MS) {
        return;
    }
    _lastPublishTime = now;

    // Return 0 if no valid target for extended period
    if (_noTargetSince && (now - _noTargetSince > 10000)) {
        updatePresenceState(RD03PresenceState::NO_PRESENCE, 0.0f);
    }
}

// ============================================================================
// Private Methods - State Management
// ============================================================================

void RD03Radar::updatePresenceState(RD03PresenceState newState, float distance) {
    if (_presenceState != newState || (distance > 0 && fabs(distance - _lastValidDistance) > 0.1f)) {
        _presenceState = newState;
        if (distance > 0) {
            _lastValidDistance = distance;
        }

        // Call presence callback
        if (_presenceCallback) {
            _presenceCallback(newState, distance);
        }
    }
}

void RD03Radar::updateStatus(RD03Status newStatus, const char* message) {
    if (_status != newStatus) {
        _status = newStatus;

        // Call status callback
        if (_statusCallback) {
            _statusCallback(newStatus, message);
        }
    }
}

// ============================================================================
// Private Methods - Light Control Logic
// ============================================================================

void RD03Radar::handleLightControl() {
    uint32_t now = millis();
    uint32_t elapsed = now - _lastActivityTime;

    // Force controls override everything
    if (_controlMode == RD03ControlMode::FORCE_ON) {
        if (_lightCallback) {
            _lightCallback(true, "Force ON");
        }
        updatePresenceState(RD03PresenceState::PRESENCE_DETECTED);
        return;
    }

    if (_controlMode == RD03ControlMode::FORCE_OFF) {
        if (_lightCallback) {
            _lightCallback(false, "Force OFF");
        }
        updatePresenceState(RD03PresenceState::NO_PRESENCE);
        return;
    }

    // Manual control
    if (_controlMode == RD03ControlMode::MANUAL_ON) {
        if (_lightCallback) {
            _lightCallback(true, "Manual ON");
        }
        updatePresenceState(RD03PresenceState::PRESENCE_DETECTED);
        return;
    }

    // Automatic mode
    if (_controlMode == RD03ControlMode::AUTOMATIC) {
        // Fast exit: Turn off quickly when no target detected for 15 seconds
        if (_presenceActive && _noTargetSince &&
            (now - _noTargetSince > FAST_EXIT_TIMEOUT_MS)) {
            if (_lightCallback) {
                _lightCallback(false, "Fast OFF (No Target)");
            }
            _presenceActive = false;
            updatePresenceState(RD03PresenceState::FAST_EXIT);
            return;
        }

        // Hold time: Keep light on based on sensitivity setting
        uint32_t holdMs = _config.sensitivity * 10000UL; // 10-50 seconds
        if (elapsed < holdMs) {
            if (_lightCallback) {
                _lightCallback(true, "Auto ON");
            }
            updatePresenceState(RD03PresenceState::PRESENCE_DETECTED);
            return;
        }

        // Safety timeout: Force off after maximum absence time
        if (elapsed > _config.maxAbsenceTime * 1000UL) {
            if (_lightCallback) {
                _lightCallback(false, "Safety Auto OFF");
            }
            _presenceActive = false;
            updatePresenceState(RD03PresenceState::SAFETY_TIMEOUT);
            return;
        }

        // Default: Turn off light
        if (_lightCallback) {
            _lightCallback(false, "Auto OFF");
        }
        updatePresenceState(RD03PresenceState::NO_PRESENCE);
    }
}

// ============================================================================
// Private Methods - Watchdog & Recovery
// ============================================================================

void RD03Radar::watchdogCheck() {
    if (!_watchdogActivityTime) return;

    uint32_t now = millis();
    uint32_t silentMs = now - _watchdogActivityTime;

    // Soft recovery: Reset radar after 90-180 seconds of silence
    if (silentMs > WATCHDOG_SOFT_RESET_MS && silentMs < WATCHDOG_HARD_RESET_MS) {
        updateStatus(RD03Status::WATCHDOG_RESET, "Soft reset - radar silent");
        resetRadar();
        return;
    }

    // Hard recovery: Restart ESP after 12 hours of silence
    if (silentMs >= LONG_SILENCE_RESET_MS) {
        updateStatus(RD03Status::WATCHDOG_RESET, "Hard reset - long silence detected");
        // Note: In Arduino, we can't restart the board directly
        // This would need to be handled by the user application
        return;
    }
}

// ============================================================================
// Private Methods - Utility Functions
// ============================================================================

void RD03Radar::sendResetCommands() {
    // Send first initialization command
    _serial.write(RADAR_INIT_CMD1, sizeof(RADAR_INIT_CMD1));
    delay(100);

    // Send second initialization command
    _serial.write(RADAR_INIT_CMD2, sizeof(RADAR_INIT_CMD2));
}

void RD03Radar::clearUARTBuffer() {
    _uartBuffer.clear();
    _lastByteTime = 0;

    // Clear any remaining data in serial buffer
    while (_serial.available()) {
        _serial.read();
    }
}

const char* RD03Radar::statusToString(RD03Status status) const {
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

const char* RD03Radar::presenceStateToString(RD03PresenceState state) const {
    switch (state) {
        case RD03PresenceState::NO_PRESENCE: return "NO_PRESENCE";
        case RD03PresenceState::PRESENCE_DETECTED: return "PRESENCE_DETECTED";
        case RD03PresenceState::MOTION_DETECTED: return "MOTION_DETECTED";
        case RD03PresenceState::MAINTAINING: return "MAINTAINING";
        case RD03PresenceState::FAST_EXIT: return "FAST_EXIT";
        case RD03PresenceState::SAFETY_TIMEOUT: return "SAFETY_TIMEOUT";
        default: return "UNKNOWN";
    }
}
