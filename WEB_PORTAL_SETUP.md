# Web Portal Setup Guide

## Overview
The web portal provides a WiFi-based interface for monitoring telemetry, manual control, and OTA firmware updates for the QuadBike control system.

## Features
✅ Real-time telemetry display (5 Hz updates)
✅ Manual control when S-bus inactive (gear selection, steering, throttle)
✅ OTA firmware updates
✅ Input source priority: S-bus > Web > Fail-safe
✅ Mobile-responsive web interface

## Quick Start

### 1. Build and Upload Firmware

```bash
# Build the project (this will download all libraries)
pio run

# Upload firmware to ESP32-C6
pio run -t upload

# Monitor serial output
pio run -t monitor
```

### 2. Upload Filesystem (Web Interface)

The web interface HTML file must be uploaded to the ESP32's SPIFFS filesystem:

```bash
# Upload filesystem image containing data/index.html
pio run -t uploadfs
```

**Important:** You must upload the filesystem after any changes to `data/index.html`.

### 3. Connect to WiFi

After the ESP32 boots:

1. **WiFi Network:**
   - SSID: `QuadBike-Control`
   - Password: None (open network)
   - IP Address: `192.168.4.1`

2. **Connect your device:**
   - On your laptop/phone, connect to the `QuadBike-Control` WiFi network
   - Open a web browser and navigate to: `http://192.168.4.1`

### 4. Use the Web Portal

**Telemetry Display** (always active):
- Current gear (R/N/L/H)
- Hall sensor position (encoder counts)
- Brake percentage
- Throttle angle
- Steering angle
- Input source indicator
- S-bus status

**Manual Control** (only when S-bus inactive):
- Gear selection buttons (R/N/L/H)
- Steering slider (-100% to +100%)
- Throttle slider (0% to 100%)

**OTA Firmware Update:**
1. Build new firmware: `pio run`
2. Locate firmware binary: `.pio/build/esp32-c6-devkitc-1/firmware.bin`
3. Click "Click to select firmware (.bin)" on web portal
4. Select the firmware.bin file
5. Click "Upload Firmware"
6. Vehicle will enter safe state, update, and reboot automatically

## Input Source Priority

The system enforces this priority hierarchy:

1. **S-bus (Highest)** - When S-bus signal is active, web control is disabled
2. **Web** - When S-bus inactive, web commands are accepted
3. **Fail-safe (Lowest)** - When both sources inactive, vehicle enters safe state:
   - Steering: Center (90°)
   - Throttle: Idle (0%)
   - Brakes: Applied
   - Transmission: Neutral

## Configuration

All settings are in [include/Constants.h](include/Constants.h):

```cpp
// WiFi Configuration
#define WIFI_AP_SSID          "QuadBike-Control"
#define WIFI_AP_PASSWORD      ""  // Open network
#define WIFI_AP_IP            IPAddress(192, 168, 4, 1)

// Telemetry Rate
#define TELEMETRY_INTERVAL    200  // ms (5 Hz)

// OTA Configuration
#define OTA_HOSTNAME          "quadbike-control"
#define OTA_PASSWORD          ""   // No password
```

## Troubleshooting

### WiFi AP not appearing
- Check serial monitor for "Web portal initialized" message
- Verify ESP32 has booted successfully
- Check if WiFi is disabled in your region's frequency band

### Cannot access web interface (http://192.168.4.1)
- Ensure you're connected to the `QuadBike-Control` WiFi network
- Check that filesystem was uploaded: `pio run -t uploadfs`
- Check serial monitor for "SPIFFS mounted" and "index.html" found messages
- Try accessing by IP: `http://192.168.4.1/index.html`

### Telemetry not updating
- Check WebSocket connection indicator (should show "Connected")
- Open browser developer console (F12) and check for WebSocket errors
- Verify control loop is running (serial monitor should show activity)

### Web control not working
- Check input source indicator - must show "Web" (not "S-bus" or "Fail-safe")
- If S-bus is active, disconnect S-bus to enable web control
- Check that manual control buttons/sliders are enabled (not greyed out)

### OTA update fails
- Ensure firmware file is valid `.bin` file from PlatformIO build
- Check that device has enough free space for new firmware
- Try OTA update via Arduino IDE OTA (hostname: `quadbike-control`)
- If update fails, device will automatically rollback to previous firmware

### Build errors
If you see library-related errors:

```bash
# Clean and rebuild
pio run -t clean
pio run
```

Common missing libraries are installed automatically by PlatformIO:
- ESPAsyncWebServer
- AsyncTCP
- ArduinoJson
- SPIFFS (built-in)

## Serial Commands

Serial commands still work alongside web portal:

**Actuator Control:**
- `s<angle>` - Set steering (0-180°)
- `t<angle>` - Set throttle (0-180°)
- `g<speed>` - Transmission actuator (-255 to +255)
- `b<speed>` - Brake actuator (-255 to +255)

**Gear Selection:**
- `R` - Reverse gear
- `N` - Neutral gear
- `L` - Low gear
- `H` - High gear
- `G` - Display current gear

**Safety:**
- `x` - Emergency stop
- `c` - Safe position (center/idle/stop)

## Architecture

```
ESP32-C6
├── WiFi AP (192.168.4.1)
├── AsyncWebServer (port 80)
│   ├── HTTP: Serves /index.html
│   └── WebSocket (/ws): Real-time telemetry & commands
├── ArduinoOTA (port 3232)
└── Control Loop (100 Hz)
    ├── Input source priority
    ├── Telemetry broadcasting (5 Hz)
    └── Web command processing
```

## Files

**Implementation:**
- [include/WebPortal.h](include/WebPortal.h) - WebPortal class declaration
- [src/WebPortal.cpp](src/WebPortal.cpp) - Web server, WebSocket, OTA
- [data/index.html](data/index.html) - Web interface (HTML/CSS/JS)
- [src/main.cpp](src/main.cpp) - Integration and control loop

**Configuration:**
- [include/Constants.h](include/Constants.h) - WiFi and web server constants
- [platformio.ini](platformio.ini) - Library dependencies and build config

## Safety Notes

⚠️ **Important Safety Considerations:**

1. **Web control is secondary** - S-bus always has priority when active
2. **Open WiFi network** - Anyone within range can connect (suitable for single operator)
3. **Manual control testing** - Test in safe environment before field use
4. **OTA updates** - Vehicle enters safe state during updates (brakes applied, neutral)
5. **Fail-safe behavior** - System defaults to safe state when no control source active
6. **Control loop timing** - Web portal designed not to interfere with 100 Hz control loop

## Next Steps

1. **Test WiFi AP** - Verify you can connect and access web interface
2. **Test telemetry** - Check that all vehicle data displays correctly
3. **Test manual control** - With S-bus disconnected, test gear/steering/throttle
4. **Test S-bus priority** - Connect S-bus and verify web control is disabled
5. **Test OTA update** - Perform test firmware update in safe environment
6. **Load testing** - Connect multiple clients and verify performance

## Support

For issues or questions:
- Check serial monitor output for detailed logs
- Review OpenSpec documentation: `openspec/changes/add-web-portal/`
- Submit issues to project repository

---

**Access URL:** http://192.168.4.1 (after connecting to QuadBike-Control WiFi)
