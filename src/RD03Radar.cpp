/**
 * RD03Radar Implementation
 * Smart Presence Detection System for Ai-Thinker RD-03 24GHz mmWave Radar
 *
 * Author: Mohamed Eid (gomgom-40)
 * Version: 1.1.0
 * License: MIT
 */

#include "RD03Radar.h"

// ──────────────────────────────────────────────
// Constants
// ──────────────────────────────────────────────
#define RADAR_INIT_DELAY_MS       3000
#define UART_STALE_TIMEOUT_MS     100
#define MAX_BUFFER_SIZE           256
#define BUFFER_CLEAR_TIMEOUT_MS   30000
#define PUBLISH_INTERVAL_MS       40
#define FAST_EXIT_TIMEOUT_MS      15000
#define WATCHDOG_SOFT_RESET_MS    90000
#define WATCHDOG_HARD_RESET_MS    180000
#define LONG_SILENCE_RESET_MS     43200000UL

const uint8_t RADAR_INIT_CMD1[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x01, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
const uint8_t RADAR_INIT_CMD2[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x00, 0x01, 0x04, 0x03, 0x02, 0x01};

// ──────────────────────────────────────────────
// Constructors
// ──────────────────────────────────────────────
RD03Radar::RD03Radar(HardwareSerial& serial, const RD03Config& config)
    : _hardwareSerial(&serial)
    , _stream(&serial)
    , _isHardwareSerial(true)
    , _config(config)
    , _presenceState(RD03PresenceState::NO_PRESENCE)
    , _controlMode(RD03ControlMode::AUTOMATIC)
    , _status(RD03Status::OK)
    , _initialized(false)
#if defined(RD03_ENABLE_MQTT)
    , _mqttClient(_wifiClient)
    , _mqttEnabled(false)
#endif
#if defined(RD03_ENABLE_WEBSERVER)
    , _webPort(80)
    , _webServerEnabled(false)
#endif
{
    _uartBuffer.reserve(MAX_BUFFER_SIZE);
}

RD03Radar::RD03Radar(Stream& serial, const RD03Config& config)
    : _stream(&serial)
    , _isHardwareSerial(false)
    , _config(config)
    , _presenceState(RD03PresenceState::NO_PRESENCE)
    , _controlMode(RD03ControlMode::AUTOMATIC)
    , _status(RD03Status::OK)
    , _initialized(false)
#if defined(RD03_ENABLE_MQTT)
    , _mqttClient(_wifiClient)
    , _mqttEnabled(false)
#endif
#if defined(RD03_ENABLE_WEBSERVER)
    , _webPort(80)
    , _webServerEnabled(false)
#endif
{
    _uartBuffer.reserve(MAX_BUFFER_SIZE);
}

RD03Radar::~RD03Radar() {
    end();
}

// ──────────────────────────────────────────────
// Initialization
// ──────────────────────────────────────────────
bool RD03Radar::begin(int rxPin, int txPin) {
    if (_initialized) return true;
    if (!_isHardwareSerial) return false;

#if defined(ESP32)
    // ESP32: begin with RX/TX pins
    _hardwareSerial->begin(_config.baudRate, SERIAL_8N1, rxPin, txPin);
#else
    // ESP8266: begin without pins (HardwareSerial on ESP8266 uses default pins or pre-configured)
    _hardwareSerial->begin(_config.baudRate);
#endif

    delay(RADAR_INIT_DELAY_MS);

    if (initializeRadar()) {
        _initialized = true;
        _startTime = millis();
        updateStatus(RD03Status::OK, "Radar initialized (HardwareSerial)");
        return true;
    }
    updateStatus(RD03Status::ERROR, "Initialization failed");
    return false;
}

bool RD03Radar::begin() {
    if (_initialized) return true;
    if (_isHardwareSerial) return false;

    // ESP8266 SoftwareSerial is already initialized by user
    delay(RADAR_INIT_DELAY_MS);

    if (initializeRadar()) {
        _initialized = true;
        _startTime = millis();
        updateStatus(RD03Status::OK, "Radar initialized (SoftwareSerial)");
        return true;
    }
    updateStatus(RD03Status::ERROR, "Initialization failed");
    return false;
}

void RD03Radar::end() {
    if (_initialized) {
        if (_isHardwareSerial && _hardwareSerial) {
            _hardwareSerial->end();
        }
        _initialized = false;
        updateStatus(RD03Status::OK, "Radar stopped");
    }
}

bool RD03Radar::initializeRadar() {
    sendResetCommands();
    delay(100);
    String response = processUART();
    return response.length() > 0;
}

// ──────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────
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

// ──────────────────────────────────────────────
// Control Methods
// ──────────────────────────────────────────────
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

// ──────────────────────────────────────────────
// Main Loop
// ──────────────────────────────────────────────
void RD03Radar::loop() {
    if (!_initialized) return;

    uint32_t now = millis();
    String message = processUART();
    float distance = 0.0f;

    if (message.length() > 0) {
        distance = extractDistance(message);
        _radarLastSeenTime = now;
        _watchdogActivityTime = now;

        if (_distanceCallback) {
            _distanceCallback(distance, isValidDistance(distance));
        }
    }

    if (_controlMode == RD03ControlMode::AUTOMATIC) {
        if (!_presenceActive) {
            handleEntryDetection(distance);
        } else {
            handlePresenceMaintenance(distance);
        }
    }

    handleLightControl();
    watchdogCheck();
}

// ──────────────────────────────────────────────
// Status & Readings
// ──────────────────────────────────────────────
RD03PresenceState RD03Radar::getPresenceState() const {
    return _presenceState;
}

String RD03Radar::getPresenceStateString() const {
    switch (_presenceState) {
        case RD03PresenceState::NO_PRESENCE: return "No Presence";
        case RD03PresenceState::PRESENCE_DETECTED: return "Presence Detected";
        case RD03PresenceState::MOTION_DETECTED: return "Motion Detected";
        case RD03PresenceState::MAINTAINING: return "Maintaining";
        case RD03PresenceState::FAST_EXIT: return "Fast Exit";
        case RD03PresenceState::SAFETY_TIMEOUT: return "Safety Timeout";
        default: return "Unknown";
    }
}

float RD03Radar::getDistance() const {
    return _lastValidDistance;
}

RD03Status RD03Radar::getStatus() const {
    return _status;
}

bool RD03Radar::isOperational() const {
    if (!_initialized) return false;
    return (millis() - _radarLastSeenTime) < 10000UL;
}

uint32_t RD03Radar::getUptime() const {
    return _initialized ? millis() - _startTime : 0;
}

uint32_t RD03Radar::getLastActivityTime() const {
    return _lastActivityTime;
}

// ──────────────────────────────────────────────
// Callback Registration
// ──────────────────────────────────────────────
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

// ──────────────────────────────────────────────
// Utility Methods
// ──────────────────────────────────────────────
void RD03Radar::resetRadar() {
    sendResetCommands();
    clearUARTBuffer();
    _radarLastSeenTime = 0;
    _watchdogActivityTime = 0;
    updateStatus(RD03Status::WATCHDOG_RESET, "Radar reset initiated");
}

const char* RD03Radar::getVersion() {
    return "1.1.0";
}

const char* RD03Radar::getInfo() {
    return "RD03Radar Library v1.1.0 - Smart Presence Detection for Ai-Thinker RD-03";
}

// ──────────────────────────────────────────────
// Private: UART Processing
// ──────────────────────────────────────────────
String RD03Radar::processUART() {
    uint8_t byte;
    bool newLineFound = false;

    while (getSerial()->available() && _uartBuffer.size() < MAX_BUFFER_SIZE) {
        if (getSerial()->readBytes(&byte, 1) > 0) {
            uint32_t now = millis();
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

    if (newLineFound && !_uartBuffer.empty()) {
        String line;
        line.reserve(_uartBuffer.size());
        for (size_t i = 0; i < _uartBuffer.size(); ++i) {
            char c = static_cast<char>(_uartBuffer[i]);
            if (c == '\r' || c == '\n') break;
            line += c;
        }
        _uartBuffer.clear();
        line.trim();
        return line.length() > 0 ? line : "";
    }

    if (_uartBuffer.size() >= MAX_BUFFER_SIZE) {
        _uartBuffer.clear();
        updateStatus(RD03Status::BUFFER_OVERFLOW, "UART buffer overflow");
    }

    if (_lastByteTime && (millis() - _lastByteTime > BUFFER_CLEAR_TIMEOUT_MS) && !_uartBuffer.empty()) {
        _uartBuffer.clear();
    }

    return "";
}

// ──────────────────────────────────────────────
// Private: Distance Parsing & Validation
// ──────────────────────────────────────────────
float RD03Radar::extractDistance(const String& message) {
    if (!message.startsWith("Range ")) return 0.0f;

    int pos = -1;
    for (int i = 6; i < message.length(); ++i) {
        if (isdigit(message[i])) {
            pos = i;
            break;
        }
    }
    if (pos == -1) return 0.0f;

    String numStr = message.substring(pos);
    return numStr.toFloat();
}

bool RD03Radar::isValidDistance(float distance) {
    if (distance < _config.minRange || distance > _config.maxRange) return false;
    if (_lastValidDistance > 100.0f && distance < 10.0f) return false;
    return true;
}

// ──────────────────────────────────────────────
// Private: Presence Detection Logic
// ──────────────────────────────────────────────
void RD03Radar::handleEntryDetection(float distance) {
    uint32_t now = millis();

    if (_manualOffRecent) {
        _lastValidDistance = 0.0f;
        _noTargetSince = now;
        _manualOffRecent = false;
    }

    if (isValidDistance(distance)) {
        if (_lastDistanceForMotion == 0.0f ||
            fabs(distance - _lastDistanceForMotion) >= _config.motionThreshold) {
            _motionHits++;
            _lastDistanceForMotion = distance;
        }

        if (_motionHits >= _config.motionHitsRequired) {
            _presenceActive = true;
            _lastActivityTime = now;
            _noTargetSince = 0;
            _motionHits = 0;
            updatePresenceState(RD03PresenceState::MOTION_DETECTED, distance);
        }
    } else {
        _motionHits = 0;
        _lastDistanceForMotion = 0.0f;
    }
}

void RD03Radar::handlePresenceMaintenance(float distance) {
    uint32_t now = millis();

    if (isValidDistance(distance)) {
        _lastValidDistance = distance;
        _noTargetSince = 0;

        if (_controlMode == RD03ControlMode::AUTOMATIC) {
            _lastActivityTime = now;
        }

        updatePresenceState(RD03PresenceState::MAINTAINING, distance);
    } else if (_noTargetSince == 0) {
        _noTargetSince = now;
    }

    if (now - _lastPublishTime < PUBLISH_INTERVAL_MS) return;
    _lastPublishTime = now;

    if (_noTargetSince && (now - _noTargetSince > 10000)) {
        updatePresenceState(RD03PresenceState::NO_PRESENCE, 0.0f);
    }
}

// ──────────────────────────────────────────────
// Private: State & Status Updates
// ──────────────────────────────────────────────
void RD03Radar::updatePresenceState(RD03PresenceState newState, float distance) {
    if (_presenceState != newState || (distance > 0 && fabs(distance - _lastValidDistance) > 0.1f)) {
        _presenceState = newState;
        if (distance > 0) _lastValidDistance = distance;
        if (_presenceCallback) _presenceCallback(newState, distance);
    }
}

void RD03Radar::updateStatus(RD03Status newStatus, const char* message) {
    if (_status != newStatus) {
        _status = newStatus;
        if (_statusCallback) _statusCallback(newStatus, message);
    }
}

// ──────────────────────────────────────────────
// Private: Light Control Logic
// ──────────────────────────────────────────────
void RD03Radar::handleLightControl() {
    uint32_t now = millis();
    uint32_t elapsed = now - _lastActivityTime;

    if (_controlMode == RD03ControlMode::FORCE_ON) {
        if (_lightCallback) _lightCallback(true, "Force ON");
        updatePresenceState(RD03PresenceState::PRESENCE_DETECTED);
        return;
    }

    if (_controlMode == RD03ControlMode::FORCE_OFF) {
        if (_lightCallback) _lightCallback(false, "Force OFF");
        updatePresenceState(RD03PresenceState::NO_PRESENCE);
        return;
    }

    if (_controlMode == RD03ControlMode::MANUAL_ON) {
        if (_lightCallback) _lightCallback(true, "Manual ON");
        updatePresenceState(RD03PresenceState::PRESENCE_DETECTED);
        return;
    }

    // Automatic mode
    if (_presenceActive && _noTargetSince && (now - _noTargetSince > FAST_EXIT_TIMEOUT_MS)) {
        if (_lightCallback) _lightCallback(false, "Fast OFF (No Target)");
        _presenceActive = false;
        updatePresenceState(RD03PresenceState::FAST_EXIT);
        return;
    }

    uint32_t holdMs = _config.sensitivity * 10000UL; // 10–50 seconds
    if (elapsed < holdMs) {
        if (_lightCallback) _lightCallback(true, "Auto ON");
        updatePresenceState(RD03PresenceState::PRESENCE_DETECTED);
        return;
    }

    if (elapsed > _config.maxAbsenceTime * 1000UL) {
        if (_lightCallback) _lightCallback(false, "Safety Auto OFF");
        _presenceActive = false;
        updatePresenceState(RD03PresenceState::SAFETY_TIMEOUT);
        return;
    }

    if (_lightCallback) _lightCallback(false, "Auto OFF");
    updatePresenceState(RD03PresenceState::NO_PRESENCE);
}

// ──────────────────────────────────────────────
// Private: Watchdog
// ──────────────────────────────────────────────
void RD03Radar::watchdogCheck() {
    if (!_watchdogActivityTime) return;

    uint32_t now = millis();
    uint32_t silentMs = now - _watchdogActivityTime;

    if (silentMs > WATCHDOG_SOFT_RESET_MS && silentMs < WATCHDOG_HARD_RESET_MS) {
        updateStatus(RD03Status::WATCHDOG_RESET, "Soft reset - radar silent");
        resetRadar();
        return;
    }

    if (silentMs >= LONG_SILENCE_RESET_MS) {
        updateStatus(RD03Status::WATCHDOG_RESET, "Long silence detected - hard reset recommended");
        // User can add ESP.restart() in main sketch if needed
    }
}

// ──────────────────────────────────────────────
// Private: Reset & Buffer
// ──────────────────────────────────────────────
void RD03Radar::sendResetCommands() {
    getSerial()->write(RADAR_INIT_CMD1, sizeof(RADAR_INIT_CMD1));
    delay(100);
    getSerial()->write(RADAR_INIT_CMD2, sizeof(RADAR_INIT_CMD2));
}

void RD03Radar::clearUARTBuffer() {
    _uartBuffer.clear();
    _lastByteTime = 0;
    while (getSerial()->available()) getSerial()->read();
}

// ──────────────────────────────────────────────
// Private: String Helpers
// ──────────────────────────────────────────────
const char* RD03Radar::statusToString(RD03Status status) const {
    switch (status) {
        case RD03Status::OK:              return "OK";
        case RD03Status::ERROR:           return "ERROR";
        case RD03Status::NO_SIGNAL:       return "NO_SIGNAL";
        case RD03Status::BUFFER_OVERFLOW: return "BUFFER_OVERFLOW";
        case RD03Status::INVALID_DATA:    return "INVALID_DATA";
        case RD03Status::WATCHDOG_RESET:  return "WATCHDOG_RESET";
        default:                          return "UNKNOWN";
    }
}

const char* RD03Radar::presenceStateToString(RD03PresenceState state) const {
    switch (state) {
        case RD03PresenceState::NO_PRESENCE:      return "NO_PRESENCE";
        case RD03PresenceState::PRESENCE_DETECTED: return "PRESENCE_DETECTED";
        case RD03PresenceState::MOTION_DETECTED:   return "MOTION_DETECTED";
        case RD03PresenceState::MAINTAINING:       return "MAINTAINING";
        case RD03PresenceState::FAST_EXIT:         return "FAST_EXIT";
        case RD03PresenceState::SAFETY_TIMEOUT:    return "SAFETY_TIMEOUT";
        default:                                   return "UNKNOWN";
    }
}