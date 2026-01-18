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

// Constructor for HardwareSerial
RD03Radar::RD03Radar(HardwareSerial& serial, const RD03Config& config)
    : _serialType(SerialType::HARDWARE_SERIAL)
    , _serialPtr(&serial)
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
    , _initialized(false)
    , _lastByteTime(0)
{
    // Initialize UART buffer
    _uartBuffer.reserve(MAX_BUFFER_SIZE);
}

#if defined(ESP8266) || defined(__AVR__)
// Constructor for SoftwareSerial
RD03Radar::RD03Radar(SoftwareSerial& serial, const RD03Config& config)
    : _serialType(SerialType::SOFTWARE_SERIAL)
    , _serialPtr(&serial)
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
    , _initialized(false)
    , _lastByteTime(0)
{
    // Initialize UART buffer
    _uartBuffer.reserve(MAX_BUFFER_SIZE);
}
#endif

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

    // Initialize serial with hardware pins (ESP32) or without pins (ESP8266/other)
    #if defined(ESP32)
        serialBegin(_config.baudRate, rxPin, txPin);
    #else
        serialBegin(_config.baudRate);
    #endif

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
    serialBegin(_config.baudRate);

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
        // Note: Stream class doesn't have end() method
        // HardwareSerial and SoftwareSerial have it, but we can't call it safely
        // from a Stream reference. Just mark as not initialized.
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
    Stream* serial = getSerialStream();
    if (serial) {
        while (serial->available() && _uartBuffer.size() < MAX_BUFFER_SIZE) {
            if (serial->readBytes(&byte, 1) > 0) {
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
    } else {
        // Serial not available
        return "";
    }

    // Process complete line
    if (newLineFound && !_uartBuffer.empty()) {
        String line;
        line.concat((const char*)_uartBuffer.data(), _uartBuffer.size());
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

    // Default return for all other cases
    return "";

    // This should never be reached, but added for compiler safety
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
    Stream* serial = getSerialStream();
    if (!serial) return;

    // Send first initialization command
    serial->write(RADAR_INIT_CMD1, sizeof(RADAR_INIT_CMD1));
    delay(100);

    // Send second initialization command
    serial->write(RADAR_INIT_CMD2, sizeof(RADAR_INIT_CMD2));
}

void RD03Radar::clearUARTBuffer() {
    _uartBuffer.clear();
    _lastByteTime = 0;

    // Clear any remaining data in serial buffer
    Stream* serial = getSerialStream();
    if (serial) {
        while (serial->available()) {
            serial->read();
        }
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

// ============================================================================
// Private Helper Methods for Serial Interface
// ============================================================================

Stream* RD03Radar::getSerialStream() const {
    if (_serialType == SerialType::HARDWARE_SERIAL) {
        return static_cast<HardwareSerial*>(_serialPtr);
    }
#if defined(ESP8266) || defined(__AVR__)
    else if (_serialType == SerialType::SOFTWARE_SERIAL) {
        return static_cast<SoftwareSerial*>(_serialPtr);
    }
#endif
    return nullptr;
}

void RD03Radar::serialBegin(uint32_t baudRate) {
    if (_serialType == SerialType::HARDWARE_SERIAL) {
        static_cast<HardwareSerial*>(_serialPtr)->begin(baudRate);
    }
#if defined(ESP8266) || defined(__AVR__)
    else if (_serialType == SerialType::SOFTWARE_SERIAL) {
        static_cast<SoftwareSerial*>(_serialPtr)->begin(baudRate);
    }
#endif
}

void RD03Radar::serialBegin(uint32_t baudRate, int rxPin, int txPin) {
    if (_serialType == SerialType::HARDWARE_SERIAL) {
        #if defined(ESP32)
            // ESP32 HardwareSerial signature
            static_cast<HardwareSerial*>(_serialPtr)->begin(baudRate, SERIAL_8N1, rxPin, txPin);
        #elif defined(ESP8266)
            // ESP8266 HardwareSerial signature - pins are fixed for each serial instance
            static_cast<HardwareSerial*>(_serialPtr)->begin(baudRate);
        #else
            // Other platforms - try standard signature
            static_cast<HardwareSerial*>(_serialPtr)->begin(baudRate);
        #endif
    }
    // SoftwareSerial doesn't need pins - they're set in constructor
}

void RD03Radar::serialEnd() {
    if (_serialType == SerialType::HARDWARE_SERIAL) {
        static_cast<HardwareSerial*>(_serialPtr)->end();
    }
#if defined(ESP8266) || defined(__AVR__)
    else if (_serialType == SerialType::SOFTWARE_SERIAL) {
        static_cast<SoftwareSerial*>(_serialPtr)->end();
    }
#endif
}
