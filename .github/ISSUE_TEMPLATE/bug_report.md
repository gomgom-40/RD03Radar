---
name: Bug Report
about: Create a report to help us improve
title: "[BUG] "
labels: bug
assignees: gomgom-40

---

## Bug Description
A clear and concise description of what the bug is.

## Steps to Reproduce
1. Go to '...'
2. Click on '....'
3. Scroll down to '....'
4. See error

## Expected Behavior
A clear and concise description of what you expected to happen.

## Actual Behavior
A clear and concise description of what actually happened.

## Environment
- **Arduino IDE Version**: [e.g., 1.8.19]
- **ESP32 Core Version**: [e.g., 1.0.6]
- **ESP8266 Core Version**: [e.g., 2.7.4]
- **RD03Radar Library Version**: [e.g., 1.1.0]
- **Board**: [e.g., ESP32 DevKit V1, NodeMCU]
- **OS**: [e.g., Windows 10, Ubuntu 20.04]

## Hardware Setup
- **ESP32/ESP8266 Pin Connections**:
  - GPIO16: RX (Radar)
  - GPIO17: TX (Radar)
  - GPIO19: Relay (optional)
- **RD-03 Radar Sensor**: Yes/No
- **Power Supply**: [e.g., 5V USB]

## Code Example
```cpp
// Please provide a minimal code example that reproduces the issue
#include <RD03Radar.h>

RD03Radar radar(Serial2);

void setup() {
    Serial.begin(115200);
    radar.begin(16, 17);
}

void loop() {
    radar.loop();
}
```

## Additional Context
- **Logs/Serial Output**: [Copy and paste any relevant logs]
- **Frequency**: [How often does this happen? Always, sometimes, once?]
- **Workarounds**: [Any temporary solutions you've found?]
- **Screenshots**: [If applicable, add screenshots to help explain the problem]

## Checklist
- [ ] I have read the [README](https://github.com/gomgom-40/RD03Radar#readme)
- [ ] I have searched for similar issues in the [issue tracker](https://github.com/gomgom-40/RD03Radar/issues)
- [ ] I have tried the latest version of the library
- [ ] I have tested with different hardware configurations
