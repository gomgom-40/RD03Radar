---
name: Bug Report
about: Report a bug or issue with the library
title: "[BUG] "
labels: bug
assignees: ''

---

## 🐛 Bug Report

### **Describe the Bug**
A clear and concise description of what the bug is.

### **Steps to Reproduce**
Steps to reproduce the behavior:
1. Go to '...'
2. Click on '....'
3. Scroll down to '....'
4. See error

### **Expected Behavior**
A clear and concise description of what you expected to happen.

### **Actual Behavior**
What actually happened instead.

### **Code Example**
```cpp
// Add your code here that demonstrates the issue
#include <RD03Radar.h>

RD03Radar radar(Serial2);

void setup() {
    // Your setup code
}

void loop() {
    // Your loop code
}
```

### **Hardware & Software**
- **Board**: ESP32 / ESP8266 / Arduino Uno / Other: ____
- **Arduino IDE Version**: ____
- **Library Version**: ____
- **RD-03 Firmware Version**: ____ (if known)
- **OS**: Windows / macOS / Linux: ____

### **Hardware Connections**
```
ESP32          RD-03 Radar
-----          -----------
GPIO16 ────── TX
GPIO17 ────── RX
GND ───────── GND
5V ────────── VCC
```

### **Additional Context**
- Any error messages in Serial Monitor
- Screenshots or videos
- Any modifications to the library
- Environmental factors (temperature, interference, etc.)

### **Possible Solution**
If you have any ideas on how to fix this issue, please describe them here.

---

**Thank you for helping improve RD03Radar!** 🚀
