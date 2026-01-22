# RD03Radar Examples

This directory contains example sketches demonstrating various RD03Radar library features.

## Available Examples

### 1. BasicPresenceDetection
**File**: `BasicPresenceDetection/BasicPresenceDetection.ino`

**Description**: Simple presence detection with LED control
- Basic radar setup and initialization
- Presence detection with LED feedback
- Serial debugging output

**Hardware Requirements**:
- ESP32 or ESP8266
- RD-03 radar sensor
- LED on GPIO 2 (optional)

**Features Demonstrated**:
- ✅ Basic radar initialization
- ✅ Presence detection
- ✅ LED control
- ✅ Serial debugging

---

### 2. MQTT_Example
**File**: `MQTT_Example/MQTT_Example.ino`

**Description**: Complete MQTT integration with broker connectivity
- MQTT broker connection and authentication
- Real-time status publishing
- Remote command subscription
- WiFi connectivity

**Hardware Requirements**:
- ESP32 or ESP8266 with WiFi
- RD-03 radar sensor
- MQTT broker (local or cloud)

**Features Demonstrated**:
- ✅ WiFi connectivity
- ✅ MQTT broker connection
- ✅ Status publishing (JSON)
- ✅ Command subscription
- ✅ Error handling and reconnection

---

### 3. WebServer_Example
**File**: `WebServer_Example/WebServer_Example.ino`

**Description**: Web-based control interface
- Built-in web server
- HTML control panel
- REST API endpoints
- Real-time monitoring

**Hardware Requirements**:
- ESP32 or ESP8266 with WiFi
- RD-03 radar sensor

**Features Demonstrated**:
- ✅ Web server setup
- ✅ HTML interface
- ✅ REST API (GET/POST)
- ✅ Real-time status updates
- ✅ Mobile-responsive UI

---

### 4. Full_Features_Example
**File**: `Full_Features_Example/Full_Features_Example.ino`

**Description**: Complete integration with all features
- MQTT connectivity
- Web server interface
- Advanced presence detection
- Comprehensive monitoring
- Multi-protocol communication

**Hardware Requirements**:
- ESP32 or ESP8266 with WiFi
- RD-03 radar sensor
- MQTT broker (optional)

**Features Demonstrated**:
- ✅ All MQTT features
- ✅ All Web server features
- ✅ Advanced presence algorithms
- ✅ Comprehensive logging
- ✅ Multi-protocol operation
- ✅ Error handling and recovery

---

## Getting Started

### 1. Install the Library
```bash
# Arduino IDE Library Manager
Sketch → Include Library → Manage Libraries
Search: "RD03Radar"
Install version 1.1.0
```

### 2. Hardware Wiring
```
ESP32/ESP8266  RD-03 Radar
---------------  -----------
GPIO17/D5      ── TX
GPIO16/D6      ── RX
GND            ── GND
5V             ── VCC
```

### 3. Upload Example
1. Open desired example in Arduino IDE
2. Configure WiFi/MQTT credentials (if applicable)
3. Select your board (ESP32 DevKit V1 / NodeMCU 1.0)
4. Upload and monitor serial output

### 4. Test the Features
- **BasicPresenceDetection**: LED should blink on presence detection
- **MQTT_Example**: Monitor MQTT broker for status messages
- **WebServer_Example**: Access web interface at ESP IP address
- **Full_Features_Example**: Use both MQTT and web interface

---

## Troubleshooting

### Common Issues

#### "RD03Radar.h: No such file or directory"
- Make sure the library is installed via Arduino Library Manager
- Or copy the library files to your `~/Arduino/libraries/` directory

#### "Failed to connect to WiFi"
- Check WiFi credentials in the sketch
- Verify ESP32/ESP8266 has good WiFi signal

#### "MQTT connection failed"
- Check MQTT broker IP and port
- Verify MQTT credentials (if authentication required)
- Ensure MQTT broker is running and accessible

#### "Web server not accessible"
- Check ESP IP address in serial monitor
- Verify firewall settings
- Try different port if 80 is blocked

### Debug Tips
- Open Serial Monitor at 115200 baud
- Look for status messages and error codes
- Use Arduino IDE's built-in debugger (ESP32 only)

---

## Configuration

### WiFi Settings
```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

### MQTT Settings
```cpp
const char* MQTT_SERVER = "192.168.1.100";
const uint16_t MQTT_PORT = 1883;
const char* MQTT_USERNAME = nullptr;  // Optional
const char* MQTT_PASSWORD = nullptr;  // Optional
```

### Radar Configuration
```cpp
RD03Config radarConfig = {
    .minRange = 20.0f,
    .maxRange = 500.0f,
    .sensitivity = 3,
    .holdTime = 30,
    .maxAbsenceTime = 300
};
```

---

## Need Help?

- 📖 **Documentation**: https://github.com/gomgom-40/RD03Radar#readme
- 🐛 **Bug Reports**: https://github.com/gomgom-40/RD03Radar/issues
- 💬 **Discussions**: https://github.com/gomgom-40/RD03Radar/discussions
- 📧 **Email**: engineer.mohamed.eid@gmail.com

---

*Made with ❤️ by Mohamed Eid (gomgom-40)*
