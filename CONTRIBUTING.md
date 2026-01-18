# Contributing to RD03Radar Library

Thank you for your interest in contributing to the RD03Radar library! 🎉

We welcome contributions from the community to help improve and expand this library. Whether you're fixing bugs, adding features, improving documentation, or helping with testing, your contributions are valuable.

## 📋 How to Contribute

### 1. **Fork the Repository**
- Click the "Fork" button on GitHub
- Clone your fork: `git clone https://github.com/YOUR_USERNAME/RD03Radar.git`
- Create a feature branch: `git checkout -b feature/amazing-feature`

### 2. **Development Setup**
```bash
cd RD03Radar
# Install Arduino IDE or PlatformIO
# Open any example and test your changes
```

### 3. **Make Your Changes**
- Follow the existing code style
- Add comprehensive comments
- Test on multiple platforms (ESP32, ESP8266, Arduino Uno)
- Update documentation if needed

### 4. **Test Your Changes**
```cpp
// Test on real hardware with RD-03 radar sensor
// Verify all examples still work
// Test edge cases and error conditions
```

### 5. **Submit a Pull Request**
- Push your changes: `git push origin feature/amazing-feature`
- Create a Pull Request with a clear description
- Reference any related issues

## 🐛 Reporting Bugs

Found a bug? Please help us by:

1. **Check existing issues** first
2. **Create a new issue** with:
   - Clear title and description
   - Steps to reproduce
   - Expected vs actual behavior
   - Hardware details (board, sensor, connections)
   - Library version and Arduino IDE version
   - Code snippets if applicable

## 💡 Feature Requests

Have an idea for a new feature?

1. **Check existing issues** for similar requests
2. **Create a feature request** with:
   - Clear description of the feature
   - Use case and benefits
   - Implementation suggestions (optional)
   - Mock code examples (optional)

## 📝 Code Style Guidelines

### C++ Standards
- Use C++11 features compatible with Arduino
- Follow Arduino naming conventions
- Use meaningful variable and function names
- Add Doxygen-style comments for public APIs

### Example:
```cpp
/**
 * @brief Set detection range for radar sensor
 * @param minRange Minimum detection range in cm
 * @param maxRange Maximum detection range in cm
 */
void setRange(float minRange, float maxRange) {
    // Implementation
}
```

### File Organization
```
RD03Radar/
├── src/                    # Source files
│   ├── RD03Radar.h        # Header file
│   ├── RD03Radar.cpp      # Implementation
│   └── keywords.txt       # Arduino IDE keywords
├── examples/              # Example sketches
├── docs/                  # Documentation (future)
├── tests/                 # Unit tests (future)
└── library.properties    # Arduino IDE config
```

## 🧪 Testing Guidelines

### Hardware Testing
- Test on ESP32, ESP8266, and Arduino Uno
- Verify UART communication stability
- Test watchdog recovery mechanisms
- Validate distance measurements accuracy

### Unit Testing (Future)
```cpp
// We plan to add unit tests using ArduinoUnit
// or Google Test framework for embedded systems
```

## 📚 Documentation

### README Updates
- Keep both English and Arabic versions in sync
- Update API reference when adding new functions
- Add examples for new features

### Code Comments
- Use English for all code comments
- Explain complex algorithms
- Document function parameters and return values

## 🔄 Pull Request Process

1. **Fork and Branch**: Create a feature branch from `main`
2. **Code**: Implement your changes with tests
3. **Commit**: Use clear commit messages
   ```
   feat: add MQTT integration support
   fix: resolve UART buffer overflow issue
   docs: update API reference for new functions
   ```
4. **Push**: Push to your fork
5. **PR**: Create a Pull Request with:
   - Clear title and description
   - Reference to related issues
   - Screenshots/videos for UI changes
   - Test results

## 🏷️ Commit Message Format

We follow conventional commit format:

```
type(scope): description

[optional body]

[optional footer]
```

### Types:
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes
- `refactor`: Code refactoring
- `test`: Testing related changes
- `chore`: Maintenance tasks

### Examples:
```
feat: add multi-zone presence detection
fix: resolve distance measurement accuracy issue
docs: update installation guide for PlatformIO
```

## 🎯 Development Priorities

### High Priority
- [ ] Bug fixes and stability improvements
- [ ] Performance optimizations
- [ ] Additional hardware support
- [ ] Comprehensive testing

### Medium Priority
- [ ] New features (MQTT, multi-zone, etc.)
- [ ] PlatformIO integration
- [ ] Mobile app development
- [ ] Web configuration interface

### Low Priority
- [ ] Advanced analytics
- [ ] Machine learning integration
- [ ] Cloud connectivity
- [ ] Custom firmware options

## 🤝 Code of Conduct

### Be Respectful
- Treat all contributors with respect
- Use inclusive language
- Focus on constructive feedback

### Be Collaborative
- Help newcomers get started
- Share knowledge and best practices
- Work together to solve problems

### Be Responsible
- Test your changes thoroughly
- Document breaking changes
- Follow licensing requirements

## 📞 Getting Help

- **GitHub Issues**: For bugs and feature requests
- **GitHub Discussions**: For questions and general discussion
- **Documentation**: Check README and examples first

## 🙏 Recognition

Contributors will be:
- Listed in CHANGELOG.md
- Added to CONTRIBUTORS file
- Mentioned in release notes
- Featured in documentation

Thank you for contributing to RD03Radar! 🚀
