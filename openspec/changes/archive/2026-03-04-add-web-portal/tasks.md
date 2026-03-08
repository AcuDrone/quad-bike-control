# Implementation Tasks

## 1. Project Setup and Dependencies
- [x] 1.1 Add ESPAsyncWebServer library to platformio.ini lib_deps
- [x] 1.2 Add AsyncTCP library to platformio.ini lib_deps
- [x] 1.3 Add SPIFFS/LittleFS filesystem partition configuration to platformio.ini
- [x] 1.4 Add WiFi, web server, and OTA constants to include/Constants.h
- [ ] 1.5 Verify library installation builds successfully (requires user to build)

## 2. WebPortal Class Implementation
- [x] 2.1 Create include/WebPortal.h with class declaration
- [x] 2.2 Create src/WebPortal.cpp with WiFi AP initialization
- [x] 2.3 Implement AsyncWebServer setup (port 80)
- [x] 2.4 Implement WebSocket endpoint (/ws) for bidirectional communication
- [x] 2.5 Add static file serving from SPIFFS/LittleFS
- [x] 2.6 Implement telemetry broadcast method (JSON over WebSocket, 5 Hz)
- [x] 2.7 Implement web command handler (parse JSON commands, validate, return response)
- [x] 2.8 Add connection/disconnection event handlers for WebSocket
- [ ] 2.9 Test WiFi AP creation and connection from laptop/phone (requires hardware)

## 3. Input Source Priority System
- [x] 3.1 Define InputSource enum (SBUS, WEB, FAILSAFE) in Constants.h or WebPortal.h
- [x] 3.2 Add input source state tracking in main.cpp
- [x] 3.3 Implement input source priority logic in control loop
- [x] 3.4 Add web command acceptance only when S-bus inactive
- [ ] 3.5 Test S-bus priority (web commands ignored when S-bus active) (requires hardware)
- [ ] 3.6 Test failsafe activation when both sources inactive (requires hardware)

## 4. Telemetry Collection and Broadcasting
- [x] 4.1 Add telemetry collection function in main.cpp (gather all vehicle state)
- [x] 4.2 Create JSON telemetry message format
- [x] 4.3 Integrate telemetry broadcast in control loop (every 200ms)
- [x] 4.4 Add input source indicator to telemetry (SBUS/WEB/FAILSAFE)
- [ ] 4.5 Test telemetry streaming to web client (requires hardware)
- [ ] 4.6 Verify control loop timing remains <10ms with telemetry active (requires hardware)

## 5. Web Control Command Handlers
- [x] 5.1 Implement set_gear command handler (validate gear, call TransmissionController)
- [x] 5.2 Implement set_steering command handler (validate range -100 to +100, call ServoController)
- [x] 5.3 Implement set_throttle command handler (validate range 0 to 100, call ServoController)
- [x] 5.4 Add command validation and error responses
- [ ] 5.5 Test manual control via web interface (with S-bus disconnected) (requires hardware)
- [ ] 5.6 Test web control rejection when S-bus active (requires hardware)

## 6. Web Interface (HTML/CSS/JavaScript)
- [x] 6.1 Create data/index.html with basic HTML structure
- [x] 6.2 Add CSS styling for responsive layout (desktop and mobile)
- [x] 6.3 Implement WebSocket client connection in JavaScript
- [x] 6.4 Add telemetry display section (gear, hall position, brake, throttle, steering, input source)
- [x] 6.5 Add manual control section (gear buttons: R/N/L/H, steering slider, throttle slider)
- [x] 6.6 Implement real-time telemetry updates (parse JSON, update DOM elements)
- [x] 6.7 Implement control UI event handlers (send JSON commands over WebSocket)
- [x] 6.8 Add input source indicator (show if S-bus active, disable controls when S-bus active)
- [x] 6.9 Add connection status indicator (connected/disconnected WebSocket)
- [x] 6.10 Add OTA firmware upload section (file input, upload button, progress bar)
- [ ] 6.11 Test web UI on desktop browser (Chrome/Firefox/Safari) (requires hardware)
- [ ] 6.12 Test web UI on mobile device (responsive design) (requires hardware)

## 7. OTA Firmware Update
- [x] 7.1 Initialize ArduinoOTA in WebPortal.cpp
- [x] 7.2 Configure OTA settings (hostname, password if desired)
- [x] 7.3 Add OTA event handlers (start, progress, end, error)
- [x] 7.4 Implement vehicle safe state enforcement before OTA start (brakes on, neutral, servos disabled)
- [ ] 7.5 Add HTTP POST endpoint for firmware upload (/update) (requires ElegantOTA or custom handler)
- [x] 7.6 Implement OTA progress reporting to web client (WebSocket or HTTP response)
- [x] 7.7 Add ArduinoOTA.handle() call in main loop
- [ ] 7.8 Test OTA update with test firmware (LED blink or version change) (requires hardware)
- [ ] 7.9 Verify automatic reboot after successful OTA (requires hardware)
- [ ] 7.10 Test OTA rollback on failed update (ESP32 dual partition) (requires hardware)

## 8. Integration and System Testing
- [ ] 8.1 Upload SPIFFS/LittleFS filesystem image with index.html (user action required)
- [ ] 8.2 Test complete system: WiFi AP → web access → telemetry display (requires hardware)
- [ ] 8.3 Test manual control workflow: disconnect S-bus → use web sliders/gear buttons (requires hardware)
- [ ] 8.4 Test S-bus priority: connect S-bus → verify web control disabled (requires hardware)
- [ ] 8.5 Test fail-safe: disconnect both S-bus and web → verify safe state (requires hardware)
- [ ] 8.6 Test OTA update process end-to-end (requires hardware)
- [ ] 8.7 Load test: connect 3-5 web clients simultaneously, verify performance (requires hardware)
- [ ] 8.8 Measure and verify control loop timing remains stable (<10ms average) (requires hardware)

## 9. Documentation
- [ ] 9.1 Add WiFi credentials to README (SSID, IP address, URL) (user action required)
- [ ] 9.2 Document OTA update procedure (user action required)
- [ ] 9.3 Document manual control usage (when available, safety notes) (user action required)
- [ ] 9.4 Add troubleshooting guide (connection issues, OTA failures) (user action required)
