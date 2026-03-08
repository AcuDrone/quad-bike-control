# Design: SBusInput Class Implementation

## Context
The quad bike control system requires SBUS input from ArduPilot Rover as the primary control source. The Bolder Flight SBUS library is already included as a dependency, and pin/channel constants are defined in Constants.h. However, no SBusInput class exists to wrap this library and provide vehicle-specific command mapping.

**Constraints:**
- ESP32-C6 UART1 on GPIO 20 with inverted signal handling
- SBUS protocol: 100000 baud, 8E2, 25-byte frames at 7-14ms intervals
- Must support 16 channels with 11-bit resolution (172-1811 raw → 880-2160μs)
- Safety-critical: signal loss must trigger fail-safe within 500ms
- Input priority: SBUS > WEB > FAILSAFE

**Stakeholders:**
- VehicleController: consumes mapped commands
- TelemetryManager: monitors SBUS validity for input source selection
- Main application: lifecycle management

## Goals / Non-Goals

**Goals:**
- Wrap Bolder Flight SBUS library with vehicle-specific API
- Provide both raw channel values AND mapped vehicle commands
- Implement robust signal loss detection and fail-safe integration
- Support signal quality monitoring for diagnostics
- Use compile-time channel configuration (Constants.h)
- **NEW:** Add ignition control (OFF/ACC/IGNITION states via Ch5)
- **NEW:** Add front light control (on/off via Ch6)
- **NEW:** Create relay controller for safe switching of ignition and lights

**Non-Goals:**
- Runtime channel reconfiguration via web UI (out of scope)
- Custom SBUS protocol implementation (use existing library)
- Command rate limiting (handled by VehicleController)
- Multi-protocol support (SBUS only for now)
- Proportional dimming for lights (simple on/off only)

## Decisions

### Decision 1: Wrapper Pattern Over Direct Library Usage
**Choice:** Create SBusInput class that wraps `bfs::SbusRx` object internally.

**Rationale:**
- Isolates library implementation details from application code
- Provides vehicle-specific API (getSteering, getGear) vs generic getChannel()
- Enables easy library replacement if needed
- Centralizes SBUS-specific logic

**Alternatives Considered:**
- Direct library usage in VehicleController: Too coupled, harder to test
- Multiple specialized classes (SBusSteering, SBusThrottle): Over-engineered

### Decision 2: Channel Configuration in Constants.h
**Choice:** Define channel mapping as compile-time constants with struct-like organization.

**Rationale:**
- Vehicle channel mapping is stable (rarely changes)
- Compile-time configuration is safer for vehicle control
- No NVS complexity or runtime errors
- User preference from clarification questions

**Configuration Structure:**
```cpp
// S-bus Channel Assignments
struct SBusChannelConfig {
    static constexpr uint8_t STEERING = 1;
    static constexpr uint8_t THROTTLE = 2;
    static constexpr uint8_t TRANSMISSION = 3;
    static constexpr uint8_t BRAKE = 4;
    static constexpr uint8_t IGNITION = 5;
    static constexpr uint8_t FRONT_LIGHT = 6;
};

// Mapping Constants
#define SBUS_STEERING_DEADBAND    2.0f   // % center deadband
#define SBUS_THROTTLE_DEADBAND    2.0f   // % idle deadband

// Gear Selection Ranges (in microseconds) - 3-position switch
#define SBUS_GEAR_REVERSE_MIN     880
#define SBUS_GEAR_REVERSE_MAX     1200
#define SBUS_GEAR_NEUTRAL_MIN     1201
#define SBUS_GEAR_NEUTRAL_MAX     1520
#define SBUS_GEAR_LOW_MIN         1521
#define SBUS_GEAR_LOW_MAX         2160

// Ignition State Ranges (in microseconds)
#define SBUS_IGNITION_OFF_MIN     880
#define SBUS_IGNITION_OFF_MAX     1200
#define SBUS_IGNITION_ACC_MIN     1201
#define SBUS_IGNITION_ACC_MAX     1520
#define SBUS_IGNITION_ON_MIN      1521
#define SBUS_IGNITION_ON_MAX      2160

// Front Light Threshold (in microseconds)
#define SBUS_FRONT_LIGHT_THRESHOLD 1520  // >1520 = ON, <=1520 = OFF
```

### Decision 3: Dual API - Raw and Mapped Values
**Choice:** Provide both `getChannel(n)` for raw values AND `getSteering()` etc. for mapped commands.

**Rationale:**
- Raw values needed for diagnostics, calibration, and debugging
- Mapped values needed for direct vehicle control
- Separation of concerns: channel reading vs. interpretation
- Flexibility for future use cases (e.g., auxiliary channels)

### Decision 4: Polling-Based Update Pattern
**Choice:** Require explicit `update()` call in main loop rather than interrupt-driven.

**Rationale:**
- Matches existing codebase pattern (ServoController, TransmissionController)
- Simpler threading model (no ISR complications)
- SBUS frame rate (70-140Hz) is slower than main loop (100Hz)
- Library already handles buffering

**Update Pattern:**
```cpp
void loop() {
    sbusInput.update();  // Read and process frames

    InputSource source = telemetryManager.determineInputSource();
    if (source == InputSource::SBUS) {
        vehicleController.processSBusCommands(sbusInput);
    }

    vehicleController.update();
}
```

### Decision 5: Ignition State Mapping (3-Position Switch)
**Choice:** Map Ch5 (SBUS channel 5) to ignition states using 3 ranges on a single channel.

**Rationale:**
- Simpler wiring: single 3-position switch on transmitter
- Matches typical vehicle ignition switch behavior (rotary switch)
- Efficient use of channels (only 1 channel vs 2)
- Clear range separation reduces ambiguity

**Mapping:**
```cpp
enum class IgnitionState {
    OFF,        // 880-1200μs (low range)
    ACC,        // 1201-1520μs (mid range) - Accessory power
    IGNITION    // 1521-2160μs (high range) - Full ignition
};
```

**Relay Control:**
- OFF: Both RELAY1 and RELAY2 are LOW (all power off)
- ACC: RELAY2 HIGH (accessory power only), RELAY1 LOW
- IGNITION: Both RELAY1 and RELAY2 HIGH (full power)

**Alternatives Considered:**
- 2-bit encoding (2 channels): More complex, wastes a channel
- 2-position switches: Doesn't match vehicle ignition semantics

### Decision 6: Front Light Control (Simple On/Off)
**Choice:** Map Ch6 (SBUS channel 6) to boolean on/off using center threshold (1520μs).

**Rationale:**
- Matches typical 2-position switch on transmitter
- Simple and reliable: no ambiguity
- No need for proportional control (vehicle has standard headlights)
- Relay-based control (RELAY3) for high-current switching

**Mapping:**
```cpp
bool getFrontLight() {
    uint16_t value = getChannel(6);
    return (value > SBUS_FRONT_LIGHT_THRESHOLD);  // 1520μs threshold
}
```

**Relay Control:**
- RELAY3 HIGH = front light ON
- RELAY3 LOW = front light OFF

**Alternatives Considered:**
- Proportional dimming: Not needed, adds complexity, requires PWM
- Multiple light zones: Out of scope for this change

### Decision 7: RelayController Class for Output Management
**Choice:** Create dedicated RelayController class to manage relay outputs.

**Rationale:**
- Separation of concerns: relay logic isolated from vehicle control logic
- Reusable for future relay-based controls
- Fail-safe methods centralized (turn all relays OFF)
- Thread-safe state tracking

**Interface:**
```cpp
class RelayController {
public:
    enum class IgnitionState { OFF, ACC, IGNITION };

    bool begin(uint8_t relay1Pin, uint8_t relay2Pin, uint8_t relay3Pin);
    void setIgnitionState(IgnitionState state);
    void setFrontLight(bool on);
    IgnitionState getIgnitionState() const;
    bool getFrontLight() const;
    void allOff();  // Fail-safe: turn all relays OFF
};
```

**Pin Assignments (from Constants.h):**
- RELAY1 (GPIO 12): Ignition main power
- RELAY2 (GPIO 19): Accessory power
- RELAY3 (GPIO 9): Front light

**Alternatives Considered:**
- Direct GPIO control in VehicleController: Less maintainable, no reusability
- Single "OutputController" for all outputs: Too broad, mixing concerns

### Decision 8: Fail-Safe Behavior for Relays
**Choice:** On SBUS signal loss, all relays must switch to OFF state.

**Rationale:**
- Safety: Vehicle should power down if control signal is lost
- Prevents accidental ignition engagement
- Prevents lights from staying on and draining battery
- Consistent with other fail-safe behaviors (brakes applied, neutral gear)

**Implementation:**
```cpp
void VehicleController::applyFailsafe() {
    if (currentInputSource_ == InputSource::FAILSAFE && !failsafeApplied_) {
        // ... existing fail-safe code ...
        relayController_.allOff();  // Turn off ignition and lights
        failsafeApplied_ = true;
    }
}
```

### Decision 9: Signal Quality Metrics Structure
**Choice:** Provide structured quality metrics rather than individual getters.

**Structure:**
```cpp
struct SignalQuality {
    uint32_t totalFrames;        // Total frames received
    uint32_t errorFrames;        // Frames with errors
    uint32_t lostFrames;         // Estimated lost frames
    float frameRate;             // Current frame rate (Hz)
    float errorRate;             // Error percentage (0-100)
    uint32_t signalAge;          // Time since last frame (ms)
    bool isValid;                // Signal within timeout
};
```

**Rationale:**
- Single method call for all metrics (efficient)
- Struct can be extended without API changes
- Useful for telemetry broadcasting

## Class Interface

### SBusInput.h (Abbreviated)
```cpp
class SBusInput {
public:
    struct SignalQuality {
        uint32_t totalFrames;
        uint32_t errorFrames;
        float frameRate;
        float errorRate;
        uint32_t signalAge;
        bool isValid;
    };

    SBusInput();

    // Lifecycle
    bool begin(uint8_t rxPin, uint8_t uartNum);
    void update();

    // Raw channel access
    uint16_t getChannel(uint8_t channel) const;  // Returns μs (880-2160)
    void getRawChannels(uint16_t* channels) const;

    // Mapped vehicle commands
    float getSteering() const;    // Returns -100 to +100 (%)
    float getThrottle() const;    // Returns 0 to 100 (%)
    TransmissionController::Gear getGear() const;
    float getBrake() const;       // Returns 0 to 100 (%)
    IgnitionState getIgnitionState() const;  // Returns OFF/ACC/IGNITION
    bool getFrontLight() const;   // Returns true=ON, false=OFF

    // Signal monitoring
    bool isSignalValid() const;
    uint32_t getSignalAge() const;
    SignalQuality getSignalQuality() const;

private:
    bfs::SbusRx sbus_;
    bfs::SbusData sbusData_;
    uint32_t lastFrameTime_;
    uint32_t totalFrames_;
    uint32_t errorFrames_;

    // Helper methods
    float applyDeadband(float value, float center, float deadband) const;
    uint16_t rawToMicroseconds(uint16_t rawValue) const;
    float mapToPercentage(uint16_t valueUs, uint16_t minUs, uint16_t maxUs) const;
};
```

## Integration Points

### TelemetryManager Changes
```cpp
InputSource TelemetryManager::determineInputSource() {
    // Priority 1: SBUS (if signal valid)
    if (sbusInput_.isSignalValid()) {
        return InputSource::SBUS;
    }

    // Priority 2: WEB (if clients connected)
    if (webPortal_.getClientCount() > 0) {
        return InputSource::WEB;
    }

    // Priority 3: FAILSAFE
    return InputSource::FAILSAFE;
}

WebPortal::Telemetry TelemetryManager::collectTelemetry() {
    // ...
    telemetry.sbus_active = sbusInput_.isSignalValid();
    // ...
}
```

### VehicleController Changes
```cpp
void VehicleController::update() {
    // Process commands based on input source
    if (currentInputSource_ == InputSource::SBUS) {
        processSBusCommands();
    }

    // Apply fail-safe if needed
    applyFailsafe();

    // Update actuators
    transmission_.update();
}

void VehicleController::processSBusCommands() {
    if (!sbusInput_.isSignalValid()) {
        return;  // Redundant check for safety
    }

    // Apply steering
    float steeringPct = sbusInput_.getSteering();
    int angle = map((int)steeringPct, -100, 100,
                    STEERING_MIN_ANGLE, STEERING_MAX_ANGLE);
    steering_.setAngle(angle);

    // Apply throttle
    float throttlePct = sbusInput_.getThrottle();
    angle = map((int)throttlePct, 0, 100,
                THROTTLE_MIN_ANGLE, THROTTLE_MAX_ANGLE);
    throttle_.setAngle(angle);

    // Apply gear
    TransmissionController::Gear gear = sbusInput_.getGear();
    transmission_.setGear(gear);

    // Apply brake
    float brakePct = sbusInput_.getBrake();
    // TODO: Implement brake control when BrakeController supports percentage

    // Apply ignition state
    IgnitionState ignitionState = sbusInput_.getIgnitionState();
    relayController_.setIgnitionState(ignitionState);

    // Apply front light
    bool frontLightOn = sbusInput_.getFrontLight();
    relayController_.setFrontLight(frontLightOn);
}
```

## Risks / Trade-offs

### Risk: Signal Loss Not Detected Promptly
**Mitigation:**
- Check signal validity in both TelemetryManager and VehicleController
- Use conservative 500ms timeout (allows 35-70 missed frames at 70-140Hz)
- Require 3 consecutive good frames before recovery

### Risk: Channel Mapping Incorrect
**Mitigation:**
- Document channel assignments clearly in Constants.h
- Provide debug logging mode to display raw channel values
- Test with ArduPilot Rover before vehicle integration

### Risk: Deadband Values Too Large/Small
**Mitigation:**
- Use conservative 2% deadzones initially
- Make constants configurable in Constants.h
- Test with actual ArduPilot transmitter stick behavior

### Trade-off: No Runtime Reconfiguration
**Accepted:** Channel mapping is compile-time only. If channels need changing, must recompile and flash. This is acceptable for vehicle hardware that rarely changes.

## Migration Plan

### Implementation Sequence
1. Create SBusInput class with basic begin/update/getChannel
2. Add signal validity detection
3. Implement channel mapping methods
4. Add signal quality monitoring
5. Integrate with TelemetryManager (input source)
6. Integrate with VehicleController (command processing)
7. Test with ArduPilot Rover

### Testing Strategy
1. **Unit testing (no hardware):** Validate mapping functions with mock channel values
2. **SBUS receiver testing:** Connect receiver, verify frame decoding
3. **ArduPilot integration:** Connect to ArduPilot Rover, verify channel mapping
4. **Signal loss testing:** Disconnect receiver, verify fail-safe activation within 500ms
5. **Input priority testing:** Switch between SBUS, web, and fail-safe modes

### Rollback Plan
If issues arise:
- SBusInput can be disabled by not calling update() in main loop
- System falls back to WEB or FAILSAFE input sources
- No breaking changes to existing VehicleController web command handling

## Open Questions

None at this time. All requirements clarified through user questions.
