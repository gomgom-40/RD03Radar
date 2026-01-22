#include "RD03Radar.h"

#define RADAR_INIT_DELAY_MS 3000
#define UART_STALE_TIMEOUT_MS 100
#define MAX_BUFFER_SIZE 256

const uint8_t RADAR_INIT_CMD1[] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0x01,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
const uint8_t RADAR_INIT_CMD2[] = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x00,0x01,0x04,0x03,0x02,0x01};

Stream* RD03Radar::getSerial() {
    return _isHardwareSerial ? static_cast<Stream*>(_hardwareSerial) : _softwareSerial;
}

RD03Radar::RD03Radar(HardwareSerial& serial, const RD03Config& config)
: _hardwareSerial(&serial), _config(config), _isHardwareSerial(true) {
    _uartBuffer.reserve(MAX_BUFFER_SIZE);
}

RD03Radar::RD03Radar(Stream& serial, const RD03Config& config)
: _softwareSerial(&serial), _config(config), _isHardwareSerial(false) {
    _uartBuffer.reserve(MAX_BUFFER_SIZE);
}

RD03Radar::~RD03Radar() {
    end();
}

bool RD03Radar::begin(int rxPin, int txPin) {
    if (_initialized) return true;

#if defined(ESP32)
    if (_isHardwareSerial) {
        _hardwareSerial->begin(_config.baudRate, SERIAL_8N1, rxPin, txPin);
    }
#endif

    delay(RADAR_INIT_DELAY_MS);

    if (initializeRadar()) {
        _initialized = true;
        _startTime = millis();
        updateStatus(RD03Status::OK, "Initialized");
        return true;
    }

    updateStatus(RD03Status::ERROR, "Init failed");
    return false;
}

bool RD03Radar::begin() {
    if (_initialized) return true;

    if (_isHardwareSerial) {
        _hardwareSerial->begin(_config.baudRate);
    }

    delay(RADAR_INIT_DELAY_MS);

    if (initializeRadar()) {
        _initialized = true;
        _startTime = millis();
        updateStatus(RD03Status::OK, "Initialized");
        return true;
    }

    updateStatus(RD03Status::ERROR, "Init failed");
    return false;
}

void RD03Radar::end() {
    if (_isHardwareSerial && _hardwareSerial) {
        _hardwareSerial->end();
    }
    _initialized = false;
}

bool RD03Radar::initializeRadar() {
    sendResetCommands();
    delay(100);
    return true;
}

void RD03Radar::loop() {
    if (!_initialized) return;

    String msg = processUART();
    if (msg.length()) {
        float dist = extractDistance(msg);
        if (isValidDistance(dist)) {
            _lastValidDistance = dist;
            _lastActivityTime = millis();
            updatePresenceState(RD03PresenceState::PRESENCE_DETECTED, dist);
        }
    }
}

String RD03Radar::processUART() {
    while (getSerial()->available()) {
        char c = getSerial()->read();
        if (c == '\n') {
            String out((char*)_uartBuffer.data());
            _uartBuffer.clear();
            out.trim();
            return out;
        }
        _uartBuffer.push_back(c);
        if (_uartBuffer.size() > MAX_BUFFER_SIZE) _uartBuffer.clear();
    }
    return "";
}

float RD03Radar::extractDistance(const String& message) {
    if (!message.startsWith("Range")) return 0;
    return message.substring(6).toFloat();
}

bool RD03Radar::isValidDistance(float d) {
    return d >= _config.minRange && d <= _config.maxRange;
}

void RD03Radar::sendResetCommands() {
    getSerial()->write(RADAR_INIT_CMD1, sizeof(RADAR_INIT_CMD1));
    delay(50);
    getSerial()->write(RADAR_INIT_CMD2, sizeof(RADAR_INIT_CMD2));
}

void RD03Radar::clearUARTBuffer() {
    _uartBuffer.clear();
}

void RD03Radar::updatePresenceState(RD03PresenceState state, float distance) {
    _presenceState = state;
    if (_presenceCallback) _presenceCallback(state, distance);
}

void RD03Radar::updateStatus(RD03Status s, const char* msg) {
    _status = s;
    if (_statusCallback) _statusCallback(s, msg);
}

RD03PresenceState RD03Radar::getPresenceState() const { return _presenceState; }
float RD03Radar::getDistance() const { return _lastValidDistance; }
RD03Status RD03Radar::getStatus() const { return _status; }

void RD03Radar::onPresenceChange(PresenceCallback cb){ _presenceCallback = cb; }
void RD03Radar::onStatusChange(StatusCallback cb){ _statusCallback = cb; }
void RD03Radar::onDistanceMeasurement(DistanceCallback cb){ _distanceCallback = cb; }
void RD03Radar::onLightControl(LightControlCallback cb){ _lightCallback = cb; }

const char* RD03Radar::getVersion(){ return "1.0.1"; }
const char* RD03Radar::getInfo(){ return "RD03Radar Stable Library"; }

bool RD03Radar::isOperational() const {
    return _initialized;
}

uint32_t RD03Radar::getUptime() const {
    return millis() - _startTime;
}

uint32_t RD03Radar::getLastActivityTime() const {
    return _lastActivityTime;
}
