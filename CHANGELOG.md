# RD03Radar Library Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] - 2026-01-20

### Added
- 📡 **MQTT Integration** - Full MQTT support for IoT connectivity with status publishing and command subscription
- 🌐 **Web Server Interface** - Built-in web control panel with real-time monitoring and API endpoints
- 🔄 **Enhanced Connectivity** - Automatic MQTT reconnection with exponential backoff
- 📊 **Advanced Monitoring** - Real-time status publishing via MQTT and web API
- 🎮 **Remote Control** - Control radar settings and modes via MQTT commands or web interface
- 📱 **REST API** - JSON API endpoints for status monitoring and command execution
- 🔧 **Configuration Flexibility** - Runtime configuration changes via MQTT or web interface

### Enhanced
- 🚀 **Performance Improvements** - Optimized MQTT and web server processing
- 🛡️ **Error Handling** - Better error recovery for network connectivity issues
- 📈 **Status Reporting** - More detailed status information with uptime and diagnostics
- 🔗 **Multi-Protocol Support** - Simultaneous MQTT and web server operation

### New Examples
- **MQTT_Example.ino** - Complete MQTT integration with broker connectivity
- **WebServer_Example.ino** - Web interface demonstration with control panel
- **Full_Features_Example.ino** - Combined MQTT + Web Server + Advanced monitoring

### New API Methods
- `setupMQTT()` - Configure MQTT connection parameters
- `connectMQTT()` - Establish MQTT connection
- `publishStatus()` - Publish current status to MQTT
- `setupWebServer()` - Configure web server
- `startWebServer()` - Start web interface
- `isMQTTConnected()` - Check MQTT connection status
- `isWebServerRunning()` - Check web server status

### MQTT Topics
- `rd03radar/status` - JSON status updates (published)
- `rd03radar/commands` - Command messages (subscribed)

### Web API Endpoints
- `GET /` - Main control interface (HTML)
- `GET /api/status` - JSON status information
- `POST /api/command` - Execute control commands

---

## [1.0.0] - 2026-01-18

### Added
- 🎯 **Initial Release** - Complete Arduino library for Ai-Thinker RD-03 radar sensor
- 🚀 **Motion-based Entry Detection** - Advanced algorithm that detects actual movement, not just static presence
- 🛡️ **Advanced Watchdog Protection** - Automatic recovery from communication failures with soft and hard reset
- 🎛️ **Comprehensive Configuration** - Configurable detection range, sensitivity, hold time, and safety timeouts
- 🔄 **Multiple Control Modes** - Automatic, Manual, Force ON/OFF with seamless switching
- 📡 **Real-time Distance Monitoring** - Continuous distance measurements with validation
- 🔔 **Event-driven Architecture** - Callback system for presence changes, status updates, and light control
- 💡 **Light Control Integration** - Ready-to-use callbacks for smart home automation
- 📚 **Comprehensive Documentation** - Dual-language documentation (English & Arabic)
- 🔧 **Extensive Examples** - Multiple example sketches for different use cases
- 🎨 **Arduino IDE Integration** - Full syntax highlighting and library manager support

### Features
- **Hardware Support**: ESP32, ESP8266, and Arduino Mega/Uno
- **Detection Range**: 20cm - 600cm (configurable)
- **Sensitivity Levels**: 5 levels (1=Most sensitive, 5=Most relaxed)
- **Hold Time**: 10-50 seconds based on sensitivity
- **Safety Timeout**: Configurable maximum absence time
- **UART Communication**: 115200 baud rate with buffer protection
- **Power Consumption**: ~0.5-1W (radar only)

### Examples Included
- **BasicPresenceDetection.ino** - Simple presence detection with LED control for ESP32/ESP8266

### Compatibility
- **Arduino IDE**: 1.8.x and later
- **ESP32 Core**: 1.0.x and later
- **ESP8266 Core**: 2.7.x and later
- **PlatformIO**: Compatible

---

## Development Roadmap

### Planned for v1.1.0
- [ ] MQTT integration for IoT connectivity
- [ ] Multi-zone presence detection support
- [ ] Energy monitoring features
- [ ] Mobile app companion
- [ ] Additional hardware platform support
- [ ] Advanced calibration tools

### Planned for v1.2.0
- [ ] Home Assistant integration
- [ ] Web-based configuration interface
- [ ] Data logging and analytics
- [ ] Custom sensor profiles
- [ ] Wireless sensor mesh support

---

## Migration Guide

### From ESPHome
If you're migrating from the original ESPHome configuration:

1. **Install the library** via Arduino IDE Library Manager
2. **Update your code** to use the new API:
   ```cpp
   // Old ESPHome way
   // New Arduino way
   RD03Radar radar(Serial2, radarConfig);
   radar.onPresenceChange(yourCallback);
   radar.begin(16, 17);
   ```
3. **Map your configuration**:
   - `min_range` → `radarConfig.minRange`
   - `max_range` → `radarConfig.maxRange`
   - `sensitivity` → `radarConfig.sensitivity`
   - `max_absence_time` → `radarConfig.maxAbsenceTime`

### API Changes
- All global variables are now encapsulated in the `RD03Radar` class
- Lambda functions are replaced with callback functions
- Configuration is done through `RD03Config` struct
- Status monitoring uses callback system instead of sensors

---

## Credits

- **Ai-Thinker** - For the affordable RD-03 radar sensor
- **ESPHome Community** - For inspiration and testing
- **Arduino Community** - For the amazing platform
- **Open Source Contributors** - For feedback and improvements

---

**Made with ❤️ by Mohamed Eid (gomgom-40)**

*Transforming budget sensors into intelligent solutions*
