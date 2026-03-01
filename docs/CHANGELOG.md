# Changelog

All notable changes to this project are documented in this file.

## [1.0.0] - 2025-01-24

### Added
- ✅ ESP32 firmware with TCP/UDP support
- ✅ ROS2 nodes (Python and C++)
- ✅ GUI control (Tkinter)
- ✅ SLAM Toolbox integration
- ✅ Automatic mapping
- ✅ Complete documentation
- ✅ Config loader with interactive wizard
- ✅ Robot monitor with diagnostics

### Features
- TCP motor control
- UDP LiDAR data reception
- State machine for autopilot
- Real-time visualization in RViz2
- Map saving in PGM/YAML format
- Battery monitoring
- WiFi auto-reconnection

### Fixed
- Fixed all PyYAML imports
- Correct TCP error handling
- Fallback mode when config.yaml is missing

### Known Issues
- lidar_processor.cpp requires optimization (not critical)
- No unit tests (planned)

---

**Version 1.0.0 is ready for production use.**
