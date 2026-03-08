# Design: Web Portal Architecture

## Context
The ESP32-C6 quad bike control system currently operates via S-bus commands from ArduPilot Rover with serial debugging only. This design adds a web-based interface for monitoring, manual control, and firmware updates while maintaining safety-critical S-bus control priority and fail-safe behavior.

### Constraints
- ESP32-C6 has limited RAM (~400KB) - must use async web server to avoid blocking
- Web portal must not interfere with real-time control loop (100 Hz / 10ms cycle)
- Safety-critical: Web control only when S-bus inactive
- No internet connectivity in field - must work as standalone AP

### Stakeholders
- Operators: Need telemetry visibility and manual control for testing/emergency
- Developers: Need OTA updates for firmware deployment in field
- Safety: Must maintain fail-safe behavior and S-bus priority

## Goals / Non-Goals

### Goals
- Provide real-time telemetry dashboard accessible via WiFi
- Enable OTA firmware updates without serial connection
- Allow manual control when S-bus not available
- Maintain 100 Hz control loop performance
- Zero configuration required (fixed SSID/IP)

### Non-Goals
- Internet connectivity or cloud integration
- Video streaming or camera integration
- Mobile app (web responsive design sufficient)
- User authentication system (single operator scenario)
- Historical data logging (display current state only)
- Station mode WiFi (AP mode only keeps architecture simple)

## Decisions

### Decision 1: WiFi Access Point Mode Only
**Choice**: ESP32 operates as WiFi AP with fixed SSID "QuadBike-Control" (no password)

**Rationale**:
- Always accessible regardless of location/existing networks
- No configuration required
- Operators connect directly to vehicle
- Simple IP addressing (ESP32 always 192.168.4.1)

**Alternatives considered**:
- Station mode: Rejected - requires WiFi credentials, not available in field
- Dual mode (AP + Station): Rejected - adds complexity, not needed for use case

### Decision 2: AsyncWebServer + WebSocket for Telemetry
**Choice**: Use ESPAsyncWebServer library with WebSocket for bidirectional real-time communication

**Rationale**:
- Async pattern prevents blocking control loop
- WebSocket efficient for high-frequency telemetry (5 Hz)
- Single persistent connection vs HTTP polling overhead
- ESP32-optimized library with low memory footprint

**Alternatives considered**:
- HTTP polling: Rejected - inefficient, higher latency
- MQTT: Rejected - overkill, requires broker
- Server-Sent Events: Rejected - unidirectional only

### Decision 3: Single-Page Application (SPA) Architecture
**Choice**: HTML/CSS/JavaScript SPA served from SPIFFS/LittleFS, embedded at compile time

**Rationale**:
- No external dependencies at runtime
- Fast load time (all assets local)
- Works offline by design
- Simple deployment (upload filesystem image)

**Implementation**:
- `data/index.html` - Complete SPA with embedded CSS/JS
- SPIFFS/LittleFS partition in flash
- Served statically via AsyncWebServer

### Decision 4: Input Source Priority: S-bus > Web > Fail-safe
**Choice**: Command priority hierarchy enforced in control loop

**Priority levels**:
1. **S-bus Active**: S-bus commands control vehicle, web control disabled (read-only telemetry)
2. **S-bus Inactive** (>500ms timeout): Web commands accepted if provided, otherwise fail-safe
3. **Fail-safe**: Both sources inactive - brakes applied, neutral gear, center steering

**Implementation**:
```cpp
enum class InputSource { SBUS, WEB, FAILSAFE };
InputSource currentSource = FAILSAFE;

// In control loop:
if (sbus.isSignalValid()) {
    currentSource = SBUS;
    applyCommandsFromSbus();
} else if (webPortal.hasActiveControl()) {
    currentSource = WEB;
    applyCommandsFromWeb();
} else {
    currentSource = FAILSAFE;
    applyFailsafeCommands();
}
```

### Decision 5: ArduinoOTA for Firmware Updates
**Choice**: Use ArduinoOTA library with web UI upload button

**Rationale**:
- Standard ESP32 OTA solution, well-tested
- Handles flash partitioning, rollback, verification
- Simple integration with AsyncWebServer
- Supports both HTTP upload and Arduino IDE OTA

**Safety**:
- Force vehicle to safe state before OTA begins (brakes on, neutral, servos disabled)
- Display update progress on web UI
- Automatic reboot after successful update

### Decision 6: Telemetry Data Format (JSON over WebSocket)
**Choice**: Send telemetry as JSON messages at 5 Hz (200ms interval)

**Message format**:
```json
{
  "timestamp": 1234567890,
  "gear": "N",
  "hall_position": -5000,
  "brake_pct": 30,
  "throttle_angle": 0,
  "steering_angle": 90,
  "input_source": "SBUS",
  "sbus_active": true
}
```

**Rationale**:
- Human-readable for debugging
- Extensible (add fields without breaking clients)
- Small payload size (~120 bytes)
- Native browser JSON parsing

### Decision 7: Web Control Command Format
**Choice**: Client sends JSON commands over WebSocket, server validates and applies

**Command format**:
```json
{
  "cmd": "set_gear",
  "value": "N"
}
```

```json
{
  "cmd": "set_steering",
  "value": -50.0
}
```

```json
{
  "cmd": "set_throttle",
  "value": 25.0
}
```

**Validation**:
- Server checks input source priority before accepting command
- Range validation (steering: -100 to +100, throttle: 0 to 100)
- Sends error response if command rejected

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32-C6 Firmware                     │
│                                                          │
│  ┌────────────┐                                         │
│  │   S-bus    │────────┐                                │
│  │   Input    │        │                                │
│  └────────────┘        │                                │
│                        ▼                                │
│  ┌────────────┐   ┌──────────────┐   ┌──────────────┐ │
│  │   WiFi AP  │   │ Input Source │   │   Vehicle    │ │
│  │ Web Server │──▶│   Priority   │──▶│   Systems    │ │
│  │ WebSocket  │   │   Arbiter    │   │  (Steering,  │ │
│  └────────────┘   └──────────────┘   │  Throttle,   │ │
│        │                              │  Brake, etc) │ │
│        │                              └──────────────┘ │
│        │                                      │         │
│        │          ┌──────────────┐            │         │
│        └─────────▶│ Telemetry    │◀───────────┘         │
│                   │   Collector  │                      │
│                   └──────────────┘                      │
│                                                          │
└─────────────────────────────────────────────────────────┘
                           │
                           │ WebSocket (JSON)
                           │
                           ▼
                  ┌─────────────────┐
                  │   Web Browser   │
                  │  (SPA Client)   │
                  │                 │
                  │  - Telemetry    │
                  │    Display      │
                  │  - Control UI   │
                  │  - OTA Upload   │
                  └─────────────────┘
```

## Risks / Trade-offs

### Risk: WiFi interference with control loop timing
- **Mitigation**: Use async web server, profile control loop timing with WiFi active, ensure <10ms loop time maintained
- **Monitoring**: Add loop timing diagnostics to telemetry

### Risk: Web command conflicts with S-bus during transition
- **Mitigation**: Implement clean state machine for input source switching, ignore web commands for 1 second after S-bus recovery
- **Testing**: Test rapid S-bus connect/disconnect scenarios

### Risk: OTA update failure bricks device
- **Mitigation**: ESP32 has dual partition OTA - automatic rollback on boot failure. Always test OTA on dev board first.
- **Recovery**: Keep serial bootloader access for emergency reflash

### Risk: Multiple web clients cause command conflicts
- **Mitigation**: WebSocket broadcasts telemetry to all clients, but only accept commands from most recent active client
- **UX**: Display warning on web UI if multiple clients connected

### Trade-off: 5 Hz telemetry vs higher rates
- **Choice**: 5 Hz (200ms updates)
- **Reasoning**: Sufficient for human monitoring, lower network/CPU load, allows more clients
- **Future**: Could make configurable if needed

### Trade-off: No authentication
- **Choice**: Open WiFi AP, no password or authentication
- **Reasoning**: Single operator scenario, physical access required, simpler UX
- **Risk**: Anyone in WiFi range can connect - acceptable for private vehicle
- **Future**: Could add optional WPA2 password in constants

## Migration Plan

### Phase 1: Core Web Infrastructure
1. Add library dependencies to platformio.ini
2. Implement WebPortal class (web server, AP setup, WebSocket)
3. Create basic index.html with telemetry display only (read-only)
4. Test WiFi AP connectivity and telemetry streaming

### Phase 2: Telemetry Integration
1. Add telemetry collector in main loop
2. Broadcast telemetry to WebSocket clients at 5 Hz
3. Verify control loop timing unaffected
4. Test with multiple clients

### Phase 3: Manual Control
1. Add input source priority arbiter in main loop
2. Implement web command handlers (gear, steering, throttle)
3. Add manual control UI (sliders, gear buttons)
4. Test control switching between S-bus and web

### Phase 4: OTA Updates
1. Integrate ArduinoOTA library
2. Add firmware upload UI to web page
3. Test OTA update process
4. Document OTA procedure

### Rollback
- Remove web server initialization from main.cpp
- Revert to previous firmware version via serial upload
- No persistent state changes - safe to rollback anytime

## Implementation Files

### New Files
- `include/WebPortal.h` - WebPortal class declaration
- `src/WebPortal.cpp` - Web server, WebSocket, telemetry, OTA
- `data/index.html` - Single-page web application

### Modified Files
- `src/main.cpp` - Add WebPortal initialization, input source priority, telemetry collection
- `include/Constants.h` - Add WiFi/web server constants (SSID, IP, ports)
- `platformio.ini` - Add library dependencies

### Library Dependencies
```ini
lib_deps =
    bolderflight/Bolder Flight Systems SBUS@^8.1.4
    me-no-dev/ESPAsyncWebServer@^1.2.3
    me-no-dev/AsyncTCP@^1.1.1
```
(ArduinoOTA is built into ESP32 Arduino core)

## Open Questions
- None (all requirements clarified with user)
