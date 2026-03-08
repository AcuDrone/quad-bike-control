# Proposal: Add CAN Controller Class

## Summary
Add CAN bus interface using MCP2515 SPI controller to read vehicle OBD-II data (engine RPM, speed, temperature) and integrate with transmission control system for safe gear switching.

## Motivation
The quad bike's vehicle data (RPM, speed, temperature) is available on the CAN bus but currently not accessible to the ESP32 control system. Reading this data enables:
1. **Safety**: Prevent gear switching when vehicle is moving
2. **Smooth shifting**: Increase engine RPM during gear changes
3. **Telemetry**: Display real-time engine data in web portal
4. **Diagnostics**: Monitor engine health and performance

## Proposed Changes

### 1. CANController Class
New class to interface with MCP2515 CAN controller via SPI:
- Initialize MCP2515 on reserved SPI pins (GPIO 8/0/23/22)
- Configure for 500 kbps CAN bus speed
- Read standard OBD-II PIDs (Mode 01):
  - 0x0C: Engine RPM
  - 0x0D: Vehicle Speed
  - 0x05: Engine Coolant Temperature
  - Additional PIDs as needed
- Periodic polling (100ms interval)
- Error handling and CAN bus status monitoring

### 2. OBD-II Data Structure
```cpp
struct VehicleData {
    uint16_t engineRPM;      // RPM (0-16383)
    uint8_t vehicleSpeed;    // km/h (0-255)
    int8_t coolantTemp;      // °C (-40 to +215)
    uint32_t lastUpdateTime; // millis()
    bool dataValid;          // CAN communication status
};
```

### 3. Transmission Safety Integration
Modify `TransmissionController` to use CAN data:
- **Speed Interlock**: Block gear changes if `vehicleSpeed > 5 km/h`
- **RPM Boost**: Increase throttle to 20% during gear transitions
- **Safety Timeout**: Allow manual override after 5 seconds if speed sensor fails

### 4. Telemetry Integration
Extend `TelemetryManager` to broadcast CAN data:
- Add vehicle data to WebSocket telemetry
- Update web portal UI to display engine metrics
- Add CAN status indicator (connected/error)

### 5. Library Dependencies
Add to `platformio.ini`:
```ini
lib_deps =
    autowp/mcp_can@^1.5.1  # MCP2515 CAN library
```

## Architecture Impact

### Data Flow
```
MCP2515 (CAN Bus) → CANController → VehicleController
                                  ↓
                            TransmissionController (safety checks)
                                  ↓
                            TelemetryManager (web display)
```

### Integration Points
1. **VehicleController**: Create CANController instance, call `update()` in main loop
2. **TransmissionController**: Check speed before allowing gear changes
3. **TelemetryManager**: Include CAN data in telemetry broadcasts
4. **Constants.h**: Add CAN configuration constants

## Implementation Phases

1. **Phase 1**: CANController class with OBD-II reading
2. **Phase 2**: Transmission safety integration (speed interlock)
3. **Phase 3**: RPM boost during gear switching
4. **Phase 4**: Telemetry and web UI integration
5. **Phase 5**: Hardware testing and calibration

## Testing Strategy

### Unit Testing
- MCP2515 initialization and SPI communication
- OBD-II PID request/response parsing
- Speed interlock logic

### Integration Testing
- CAN bus connection to vehicle ECU
- Gear switching with speed > 5 km/h (should block)
- RPM boost activation during gear change
- Telemetry data accuracy

### Hardware Requirements
- MCP2515 CAN module (SPI interface)
- OBD-II to CAN cable/adapter
- Access to vehicle CAN bus (via OBD-II port)

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| CAN communication failure | Gear switching blocked | Timeout fallback (5s) allows manual override |
| Incorrect OBD-II PIDs | Wrong data displayed | Validate against known good values, add PID configuration |
| SPI pin conflicts | Hardware failure | Use reserved CAN pins, verify no conflicts |
| CAN bus errors corrupt ECU | Vehicle damage | Read-only mode, no CAN writes, proper termination |

## Alternatives Considered

1. **ESP32-C6 built-in TWAI**: Would require pin reassignment, decided to use existing SPI pins
2. **Polling vs Interrupt**: Chose polling for simplicity, interrupt mode can be added later
3. **Full CAN protocol**: Chose OBD-II subset for MVP, can expand later

## Success Criteria

- ✅ CAN controller reads RPM, speed, temperature at 10 Hz
- ✅ Gear switching blocked when speed > 5 km/h
- ✅ Throttle automatically increases during gear changes
- ✅ Vehicle data visible in web telemetry
- ✅ System remains safe if CAN fails (timeout fallback)

## References
- MCP2515 datasheet: https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf
- OBD-II PIDs: https://en.wikipedia.org/wiki/OBD-II_PIDs
- ESP32-C6 SPI: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/api-reference/peripherals/spi_master.html
