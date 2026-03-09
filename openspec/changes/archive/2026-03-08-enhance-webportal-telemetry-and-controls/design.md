# Design: enhance-webportal-telemetry-and-controls

## Architecture Overview

This change extends the existing telemetry and control architecture without introducing new patterns. Data flow remains:

```
SBusInput + CANController + TransmissionController
           ↓
    TelemetryManager (collects)
           ↓
    WebPortal (serializes & broadcasts)
           ↓
    WebSocket clients (HTML/JS)
           ↓
    User interaction (sliders/buttons)
           ↓
    WebSocket commands
           ↓
    VehicleController (processes & actuates)
```

## Key Design Decisions

### 1. SBUS Channel Telemetry

**Decision**: Include all 16 raw channel values in telemetry broadcast

**Rationale**:
- Provides complete visibility for diagnostics (e.g., ArduPilot channel mapping)
- Array of 16 uint16_t adds only 32 bytes to telemetry
- Frontend can filter/highlight mapped channels for user clarity

**Alternative considered**: Only send mapped channels (Ch1-4)
- **Rejected**: Loses diagnostic capability, minimal bandwidth savings

**Implementation**:
- `WebPortal::Telemetry` gets `uint16_t sbus_channels[16]` field
- `TelemetryManager` calls `sbusInput.getRawChannels()` once per broadcast
- JSON serialization uses array: `"sbus_channels": [880, 1520, ..., 2160]`

### 2. CAN Data Display

**Decision**: Reuse existing CAN fields in telemetry, only add frontend display

**Rationale**:
- Backend already collects CAN data (TelemetryManager.cpp lines 71-89)
- No backend changes needed, only frontend HTML/JS
- CAN data already in telemetry JSON, just not displayed

**Implementation**:
- Add new HTML card "Vehicle Data (CAN)" with telemetry items
- JavaScript parses existing fields: `engine_rpm`, `vehicle_speed`, `coolant_temp`, etc.
- Color coding for temperatures uses CSS classes applied dynamically

### 3. Brake Control Command

**Decision**: Add `set_brake` command parallel to `set_steering` and `set_throttle`

**Rationale**:
- Consistent with existing control pattern
- Same safety constraints (SBUS priority, input source validation)
- Same rate limiting (100ms debounce)

**Implementation**:
- `VehicleController::processBrakeCommand(float value, WebPortal& webPortal)` method
- WebSocket command: `{"cmd": "set_brake", "value": 0-100}`
- Frontend: Brake slider with same event handling as steering/throttle sliders

**Trade-off**: Brake control from web adds safety risk (accidental engagement)
- **Mitigation**: Enforce SBUS priority (S-bus overrides web immediately)
- **Mitigation**: Fail-safe releases brake on client disconnect
- **Mitigation**: Slider default position is 0% (released)

### 4. Gear Transition State

**Decision**: Use `TransmissionController.isPositionControlActive()` as gear switching indicator

**Rationale**:
- This flag is already reliable (set during `moveToPosition()`, cleared when target reached)
- No need for separate state machine or timing logic
- Position control is the source of truth for gear transitions

**Implementation**:
- Telemetry adds boolean field: `gear_switching`
- Set to `vehicleController.getTransmission().isPositionControlActive()`
- Frontend shows indicator when `gear_switching === true`
- Indicator text: "⚙️ Switching to {target_gear}..." (target gear from `targetGear_`)

**Alternative considered**: Track transition with timer (start gear change → wait N seconds → clear)
- **Rejected**: Position control flag is more accurate (handles stalls, variable speeds)

## Data Structure Changes

### WebPortal::Telemetry struct (include/WebPortal.h)

```cpp
struct Telemetry {
    // Existing fields...
    uint32_t timestamp;
    String gear;
    int32_t hall_position;
    // ... (all current fields remain)

    // NEW: SBUS channel data
    uint16_t sbus_channels[16];  // Raw channel values in microseconds
    float sbus_frame_rate;       // Frame rate in Hz
    float sbus_error_rate;       // Error percentage (0-100)
    uint32_t sbus_signal_age;    // Time since last valid frame (ms)

    // NEW: Gear transition state
    bool gear_switching;         // True if position control active
};
```

**Size impact**: +38 bytes (32 bytes for channels, 4+4+4+1 for metrics/state)
- Total telemetry struct: ~150 bytes (estimated)
- JSON message: ~800-900 bytes (estimated, under 1KB limit)

## Frontend Layout Changes

### New Cards Added:
1. **📡 S-bus Channels** - Grid showing all 16 channels
2. **🚗 Vehicle Data (CAN)** - Telemetry items for RPM, speed, temps

### Modified Cards:
1. **🎮 Manual Control** - Add brake slider (between throttle and bottom)

### Card Layout (responsive grid):
```
┌──────────────┬──────────────┬──────────────┐
│ Telemetry    │ Manual       │ S-bus        │
│ (existing)   │ Control      │ Channels     │
│              │ (+ brake)    │ (NEW)        │
├──────────────┼──────────────┼──────────────┤
│ Vehicle Data │ System       │ Firmware     │
│ (CAN) NEW    │ Maintenance  │ Update       │
│              │ (existing)   │ (existing)   │
└──────────────┴──────────────┴──────────────┘
```

On mobile: Single column, all cards stack vertically

## Performance Considerations

### Telemetry Message Size
- **Current**: ~500-600 bytes JSON
- **After change**: ~800-900 bytes JSON
- **Limit**: 1KB (safe for WebSocket frame)
- **Broadcast frequency**: 5 Hz (every 200ms)
- **Bandwidth per client**: ~4.5 KB/s (negligible on WiFi)

### Control Loop Impact
- SBUS channel collection: `getRawChannels()` is O(1) memcpy from library buffer (~1µs)
- Position control check: `isPositionControlActive()` is O(1) boolean read (~0.1µs)
- JSON serialization: Adding 16 uint16_t increases by ~10-20µs (negligible)
- **Expected impact**: <0.1ms added to telemetry collection (< 5% of 2ms budget)

### Frontend Rendering
- SBUS channel table: 16 DOM updates per telemetry message (5 Hz)
- CAN data: 6 DOM updates per message
- **Expected impact**: ~0.5-1ms render time on modern browsers (acceptable for 5 Hz)

## Testing Strategy

### Unit Testing (Manual)
1. **Backend - SBUS collection**: Print telemetry JSON to serial, verify channels present
2. **Backend - Gear switching**: Monitor flag during gear changes, verify true→false transition
3. **Backend - Brake command**: Send WebSocket message, verify actuator responds

### Integration Testing (Hardware-in-Loop)
1. **SBUS display**: Connect ArduPilot, move sticks, verify web UI updates
2. **CAN display**: Connect vehicle CAN bus, verify RPM/speed/temps show correctly
3. **Brake control**: Move slider, measure brake actuator response time
4. **Gear transition**: Change gears, verify indicator shows during movement only

### Performance Testing
1. **Message size**: Capture WebSocket frame, verify <1KB
2. **Broadcast timing**: Measure time from telemetry collection to WS send, verify <5ms
3. **Multi-client**: Connect 5 clients, verify all receive updates at 5 Hz
4. **Control loop**: Monitor loop timing with oscilloscope, verify <10ms average

## Rollback Plan

If issues arise during implementation:

1. **SBUS display breaks telemetry**: Remove SBUS fields from struct, revert JSON serialization
2. **Brake control causes safety issue**: Disable brake slider in HTML (comment out), remove backend handler
3. **Performance degradation**: Reduce telemetry rate to 3 Hz (333ms interval)
4. **WebSocket message too large**: Remove SBUS channels 9-16 (keep only 1-8)

## Future Enhancements (Out of Scope)

- Historical telemetry logging with SD card or SPIFFS
- Graphical charts for RPM/speed/temps over time
- Configurable temperature warning thresholds from web UI
- SBUS channel remapping from web UI
- Export telemetry as CSV
