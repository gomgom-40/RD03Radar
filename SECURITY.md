# Security Policy

## Supported Versions

We actively support the following versions with security updates:

| Version | Supported          |
| ------- | ------------------ |
| 1.1.x   | ✅ Fully Supported |
| 1.0.x   | ⚠️ Critical fixes only |
| < 1.0   | ❌ Not supported   |

## Reporting a Vulnerability

If you discover a security vulnerability in RD03Radar, please report it responsibly.

### How to Report

1. **Do not create public issues** for security vulnerabilities
2. Email security reports to: `gomgom-40@github-security.com`
3. Include detailed information about:
   - The vulnerability
   - Steps to reproduce
   - Potential impact
   - Suggested fix (if available)

### What to Include

Please include as much information as possible to help us understand and fix the issue:

- **Description**: Clear description of the vulnerability
- **Impact**: What an attacker could achieve
- **Reproduction Steps**: Step-by-step instructions
- **Environment**: Arduino IDE version, board type, library versions
- **Code Sample**: Minimal code that demonstrates the issue

### Response Timeline

- **Acknowledgment**: Within 48 hours
- **Investigation**: Within 7 days
- **Fix**: Within 30 days for critical issues
- **Disclosure**: After fix is deployed

### Security Considerations

#### Network Security
- MQTT connections use standard authentication
- Web server endpoints are not encrypted by default
- Consider using VPN or secure networks for IoT deployments

#### Physical Security
- ESP32/ESP8266 devices should be in secure locations
- Consider enabling flash encryption
- Use secure boot where possible

#### Code Security
- Input validation on all MQTT messages
- Buffer overflow protection in UART processing
- Safe string handling to prevent crashes

### Responsible Disclosure

We follow responsible disclosure practices:

1. We will acknowledge receipt of your report
2. We will investigate the issue
3. We will keep you informed of our progress
4. We will credit you (if desired) when the fix is released
5. We will not disclose details until a fix is available

### Bug Bounty

While we don't currently offer a formal bug bounty program, we greatly appreciate security researchers who help make our software safer. Significant findings may be eligible for recognition in our Hall of Fame.

---

## Security Best Practices for Users

### MQTT Setup
```cpp
// Use authentication
radar.setupMQTT("192.168.1.100", 1883, "username", "password");

// Use secure MQTT (MQTT over TLS) when possible
```

### Web Server Security
```cpp
// Consider adding authentication for web interface
// Use firewall rules to restrict access
// Deploy in secure network segments
```

### General Security
- Keep ESP32/ESP8266 firmware updated
- Use strong passwords for MQTT and WiFi
- Monitor device logs for suspicious activity
- Deploy in physically secure locations

---

Thank you for helping keep RD03Radar secure! 🛡️
