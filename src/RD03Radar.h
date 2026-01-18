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
// Version: 1.0.0
// License: MIT
// ============================================================================

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Configuration structure for radar settings
 */
struct RD03Config {
    // Detection range (cm)
    float minRange = 20.0f;        // Minimum detection range
    float maxRange = 500.0f;       // Maximum detection range

    // Sensitivity settings
    uint8_t sensitivity = 3;       // Sensitivity level (1-5)

    // Timeouts (seconds)
    uint16_t holdTime = 30;        // How long to keep light on after activity (seconds)
    uint16_t maxAbsenceTime = 300; // Safety timeout (seconds)

    // Motion detection
    float motionThreshold = 2.0f;  // Minimum distance change to detect motion (cm)
    uint8_t motionHitsRequired = 1; // Motion detection sensitivity

    // UART settings
    uint32_t baudRate = 115200;
    uint8_t rxBufferSize = 256;
};

// ============================================================================
// Status Enumeration
// ============================================================================

/**
 * @brief Radar operational status
 */
enum class RD03Status {
    OK,                    // Everything working normally
    ERROR,                 // General error
    NO_SIGNAL,            // No signal from radar
    BUFFER_OVERFLOW,      // UART buffer overflow
    INVALID_DATA,         // Invalid data received
    WATCHDOG_RESET        // Watchdog initiated reset
};

/**
 * @brief Presence detection state
 */
enum class RD03PresenceState {
    NO_PRESENCE,          // No presence detected
    PRESENCE_DETECTED,    // Presence detected
    MOTION_DETECTED,      // Motion detected (entry)
    MAINTAINING,          // Maintaining presence
    FAST_EXIT,            // Fast exit (no target)
    SAFETY_TIMEOUT        // Safety timeout reached
};

/**
 * @brief Control mode
 */
enum class RD03ControlMode {
    AUTOMATIC,            // Full automatic mode
    MANUAL_ON,            // Manual override ON
    FORCE_ON,             // Force ON override
    FORCE_OFF             // Force OFF override
};

// ============================================================================
// Callback Function Types
// ============================================================================

/**
 * @brief Callback for presence state changes
 * @param state New presence state
 * @param distance Current distance reading (0 = no target)
 */
using PresenceCallback = std::function<void(RD03PresenceState state, float distance)>;

/**
 * @brief Callback for status changes
 * @param status New radar status
 * @param message Status message
 */
using StatusCallback = std::function<void(RD03Status status, const char* message)>;

/**
 * @brief Callback for distance measurements
 * @param distance Measured distance in cm (0 = no target)
 * @param valid True if measurement is valid
 */
using DistanceCallback = std::function<void(float distance, bool valid)>;

/**
 * @brief Callback for light control
 * @param turnOn True to turn light ON, false to turn OFF
 * @param reason Reason for the control action
 */
using LightControlCallback = std::function<void(bool turnOn, const char* reason)>;

// ============================================================================
// Main RD03Radar Class
// ============================================================================

class RD03Radar {
public:
    // ============================================================================
    // Constructor & Destructor
    // ============================================================================

    /**
     * @brief Constructor
     * @param serial UART interface for radar communication
     * @param config Radar configuration (optional)
     */
    RD03Radar(HardwareSerial& serial, const RD03Config& config = RD03Config());

    /**
     * @brief Destructor
     */
    ~RD03Radar();

    // ============================================================================
    // Initialization & Setup
    // ============================================================================

    /**
     * @brief Initialize the radar sensor
     * @param rxPin RX pin for UART
     * @param txPin TX pin for UART
     * @return true if initialization successful
     */
    bool begin(int rxPin, int txPin);

    /**
     * @brief Initialize with existing Serial object
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Stop radar operation
     */
    void end();

    // ============================================================================
    // Configuration Methods
    // ============================================================================

    /**
     * @brief Update radar configuration
     * @param config New configuration
     */
    void setConfig(const RD03Config& config);

    /**
     * @brief Get current configuration
     * @return Current configuration
     */
    RD03Config getConfig() const;

    /**
     * @brief Set detection range
     * @param minRange Minimum range in cm
     * @param maxRange Maximum range in cm
     */
    void setRange(float minRange, float maxRange);

    /**
     * @brief Set sensitivity level
     * @param sensitivity 1=Most sensitive, 5=Most relaxed
     */
    void setSensitivity(uint8_t sensitivity);

    /**
     * @brief Set hold time
     * @param holdTimeSeconds Time to keep light on after activity (seconds)
     */
    void setHoldTime(uint16_t holdTimeSeconds);

    // ============================================================================
    // Control Methods
    // ============================================================================

    /**
     * @brief Set control mode
     * @param mode Control mode
     */
    void setControlMode(RD03ControlMode mode);

    /**
     * @brief Get current control mode
     * @return Current control mode
     */
    RD03ControlMode getControlMode() const;

    /**
     * @brief Force manual light control
     * @param turnOn True to turn ON, false to turn OFF
     */
    void manualLightControl(bool turnOn);

    /**
     * @brief Reset presence detection state
     */
    void resetPresence();

    // ============================================================================
    // Main Processing Method (Call in loop())
    // ============================================================================

    /**
     * @brief Main processing function - call this in Arduino loop()
     * Handles UART communication, presence detection, and light control
     */
    void loop();

    // ============================================================================
    // Status & Information Methods
    // ============================================================================

    /**
     * @brief Get current presence state
     * @return Current presence state
     */
    RD03PresenceState getPresenceState() const;

    /**
     * @brief Get current distance reading
     * @return Distance in cm (0 = no target)
     */
    float getDistance() const;

    /**
     * @brief Get radar status
     * @return Current radar status
     */
    RD03Status getStatus() const;

    /**
     * @brief Check if radar is operational
     * @return true if radar is working
     */
    bool isOperational() const;

    /**
     * @brief Get uptime in milliseconds
     * @return Time since initialization
     */
    uint32_t getUptime() const;

    /**
     * @brief Get last activity timestamp
     * @return Milliseconds since last activity
     */
    uint32_t getLastActivityTime() const;

    // ============================================================================
    // Callback Registration
    // ============================================================================

    /**
     * @brief Register presence callback
     * @param callback Function to call on presence state changes
     */
    void onPresenceChange(PresenceCallback callback);

    /**
     * @brief Register status callback
     * @param callback Function to call on status changes
     */
    void onStatusChange(StatusCallback callback);

    /**
     * @brief Register distance callback
     * @param callback Function to call on distance measurements
     */
    void onDistanceMeasurement(DistanceCallback callback);

    /**
     * @brief Register light control callback
     * @param callback Function to call for light control actions
     */
    void onLightControl(LightControlCallback callback);

    // ============================================================================
    // Utility Methods
    // ============================================================================

    /**
     * @brief Reset radar sensor (send reset commands)
     */
    void resetRadar();

    /**
     * @brief Get library version
     * @return Version string
     */
    static const char* getVersion();

    /**
     * @brief Get library information
     * @return Info string
     */
    static const char* getInfo();

private:
    // ============================================================================
    // Private Member Variables
    // ============================================================================

    HardwareSerial& _serial;                    // UART interface
    RD03Config _config;                         // Current configuration
    RD03PresenceState _presenceState;           // Current presence state
    RD03ControlMode _controlMode;               // Current control mode
    RD03Status _status;                         // Current radar status

    // Timestamps and counters
    uint32_t _startTime;                        // Initialization timestamp
    uint32_t _lastActivityTime;                 // Last activity timestamp
    uint32_t _lastPublishTime;                  // Last data publish timestamp
    uint32_t _radarLastSeenTime;                // Last radar message timestamp
    uint32_t _watchdogActivityTime;             // Watchdog activity timestamp
    uint32_t _noTargetSince;                    // No target detection timestamp

    // Distance tracking
    float _lastValidDistance;                   // Last valid distance reading
    float _lastDistanceForMotion;               // Last distance for motion detection
    uint8_t _motionHits;                        // Motion detection counter

    // Control flags
    bool _presenceActive;                       // Presence detection active flag
    bool _manualOffRecent;                      // Manual turn-off flag
    bool _initialized;                          // Initialization flag

    // UART buffer
    std::vector<uint8_t> _uartBuffer;           // UART receive buffer
    uint32_t _lastByteTime;                     // Last byte received timestamp

    // Callbacks
    PresenceCallback _presenceCallback;         // Presence change callback
    StatusCallback _statusCallback;             // Status change callback
    DistanceCallback _distanceCallback;         // Distance measurement callback
    LightControlCallback _lightCallback;        // Light control callback

    // ============================================================================
    // Private Methods
    // ============================================================================

    /**
     * @brief Process UART data and extract messages
     * @return Extracted message string or empty string
     */
    String processUART();

    /**
     * @brief Extract distance from radar message
     * @param message Radar message
     * @return Distance in cm or 0.0 if invalid
     */
    float extractDistance(const String& message);

    /**
     * @brief Validate distance reading
     * @param distance Distance to validate
     * @return true if distance is valid
     */
    bool isValidDistance(float distance);

    /**
     * @brief Handle entry detection logic
     * @param distance Current distance reading
     */
    void handleEntryDetection(float distance);

    /**
     * @brief Handle presence maintenance logic
     * @param distance Current distance reading
     */
    void handlePresenceMaintenance(float distance);

    /**
     * @brief Update presence state and trigger callbacks
     * @param newState New presence state
     * @param distance Current distance
     */
    void updatePresenceState(RD03PresenceState newState, float distance = 0.0f);

    /**
     * @brief Update radar status and trigger callbacks
     * @param newStatus New radar status
     * @param message Status message
     */
    void updateStatus(RD03Status newStatus, const char* message = "");

    /**
     * @brief Handle light control logic
     */
    void handleLightControl();

    /**
     * @brief Watchdog monitoring and recovery
     */
    void watchdogCheck();

    /**
     * @brief Send radar reset commands
     */
    void sendResetCommands();

    /**
     * @brief Clear UART buffer
     */
    void clearUARTBuffer();

    /**
     * @brief Get status string for given status
     * @param status Status to convert
     * @return Status string
     */
    const char* statusToString(RD03Status status) const;

    /**
     * @brief Get presence state string
     * @param state State to convert
     * @return State string
     */
    const char* presenceStateToString(RD03PresenceState state) const;
};

#endif // RD03RADAR_H
