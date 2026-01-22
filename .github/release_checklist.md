# RD03Radar Release Checklist

## Pre-Release Checklist
- [ ] Run full CI/CD pipeline (GitHub Actions)
- [ ] Test all examples on ESP32 and ESP8266
- [ ] Verify Arduino Library Manager compatibility
- [ ] Update version numbers in all files
- [ ] Update CHANGELOG.md with release notes
- [ ] Review and update README.md
- [ ] Check all links in documentation
- [ ] Validate library.properties file

## Release Steps
- [ ] Create git tag: `git tag v1.x.x`
- [ ] Push tag: `git push origin v1.x.x`
- [ ] Create GitHub release with:
  - Release title: "RD03Radar v1.x.x - [Feature Summary]"
  - Release notes from CHANGELOG.md
  - ZIP file attached
- [ ] Verify release appears on GitHub
- [ ] Test download and installation
- [ ] Announce release on relevant platforms

## Post-Release Checklist
- [ ] Monitor GitHub Actions for any failures
- [ ] Check Arduino Library Manager update (24-48 hours)
- [ ] Monitor issue tracker for new bug reports
- [ ] Update any dependent projects/repositories
- [ ] Send release announcement emails if applicable

## Version Numbering
- **Major (X.y.z)**: Breaking changes
- **Minor (x.Y.z)**: New features, backward compatible
- **Patch (x.y.Z)**: Bug fixes, no new features

## Files to Update with New Version
- [ ] RD03Radar.h (version define)
- [ ] RD03Radar.cpp (version define)
- [ ] library.properties (version field)
- [ ] README.md (version badges)
- [ ] CHANGELOG.md (new entry)
- [ ] examples (version comments if applicable)

## Communication Channels
- [ ] GitHub Release page
- [ ] Arduino Library Manager (automatic)
- [ ] Reddit (r/arduino, r/esp32, r/esp8266)
- [ ] ESPHome community forums
- [ ] Discord servers (if applicable)
- [ ] Twitter/GitHub social features
