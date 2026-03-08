# Change: Add Web Portal for Monitoring and Control

## Why
Operators need a way to monitor vehicle telemetry, update firmware, and manually control the vehicle when S-bus/ArduPilot is not connected. Currently, all control and monitoring requires serial connection, which is impractical for field operations and testing.

## What Changes
- **NEW** Web server running on ESP32 as WiFi Access Point
- **NEW** OTA (Over-The-Air) firmware update capability via web interface
- **NEW** Real-time telemetry display showing:
  - Current gear selection (R/N/L/H)
  - Transmission hall sensor position (encoder counts)
  - Brake status (applied/released, position %)
  - Throttle servo position (angle/%)
  - Steering servo position (angle/%)
- **NEW** Manual control interface (active only when S-bus inactive):
  - Gear selector buttons (R/N/L/H)
  - Steering position slider (-100% to +100%)
  - Throttle position slider (0% to 100%)
- **MODIFIED** Main control loop to accept commands from web interface when S-bus is not active
- WebSocket-based real-time communication (5 Hz telemetry updates)

## Impact
- **Affected specs**:
  - `sbus-input` - MODIFIED to support input source priority
  - `vehicle-systems` - MODIFIED to integrate web-based control commands
  - `web-server` - NEW capability
  - `ota-updates` - NEW capability
  - `web-telemetry` - NEW capability
  - `web-control` - NEW capability
- **Affected code**:
  - `src/main.cpp` - Add web server initialization and control integration
  - `platformio.ini` - Add ESP32 AsyncWebServer and ArduinoOTA libraries
  - `include/Constants.h` - Add WiFi and web server configuration constants
  - NEW `include/WebPortal.h` / `src/WebPortal.cpp` - Web server and WebSocket management
  - NEW `data/index.html` - Web interface HTML/CSS/JavaScript
- **External dependencies**:
  - ESPAsyncWebServer library for web server
  - AsyncTCP library for async networking
  - ArduinoOTA library for firmware updates
- **Network**: ESP32 will create WiFi Access Point (SSID: "QuadBike-Control", no password)
- **Safety**: Web control only active when S-bus signal is lost or inactive (fail-safe maintained)
