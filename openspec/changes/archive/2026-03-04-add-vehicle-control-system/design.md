# Design: Vehicle Control System

## Context
The ESP32-C6 DevKit will control a quad bike's primary systems, receiving commands from ArduPilot Rover via S-bus protocol and controlling 4 actuators:
- **Input**: S-bus protocol (16-channel serial) from ArduPilot Rover
- **Servos (2x)**: Standard PWM servos for analog control (steering, throttle)
- **Linear Actuators (2x)**: BTS7960 H-bridge motor driver for position control (transmission, brakes)

The system acts as an actuator driver for ArduPilot, translating S-bus commands into vehicle control while providing safety fail-safes for signal loss.

## Goals / Non-Goals

### Goals
- Reliable S-bus protocol decoding with signal loss detection
- Safe control of all vehicle actuators with fail-safe mechanisms
- ArduPilot Rover integration as primary command source
- Modular architecture: Input → Mapping → Systems → Hardware
- Configurable pin assignments and channel mappings
- Emergency stop and fail-safe on signal loss

### Non-Goals
- Direct manual control (ArduPilot handles this)
- Autonomous navigation logic (ArduPilot's responsibility)
- Multi-vehicle coordination
- CAN bus integration (may add later if needed)
- S-bus transmitter (only receiver/decoder)

## Decisions

### Architecture: Layered Control with S-bus Input
- **Layer 0 (Input)**: S-bus decoder (SBusInput class)
- **Layer 1 (Mapping)**: Channel-to-command mapping (CommandMapper class)
- **Layer 2 (Hardware)**: Direct actuator control classes (ServoController, BTS7960Controller)
- **Layer 3 (Systems)**: Vehicle system abstractions (SteeringSystem, ThrottleSystem, TransmissionSystem, BrakeSystem)
- **Layer 4 (Coordinator)**: Main control loop with safety checks and fail-safe logic (VehicleController)

**Rationale**: Layered separation allows:
- Testing S-bus decoder independently with simulated inputs
- Swapping input sources (S-bus, PWM, serial) without changing vehicle logic
- Hardware testing without ArduPilot
- Clear fail-safe implementation at coordinator level

### S-bus Input: Hardware UART with Inversion
Use ESP32 hardware UART for S-bus (100000 baud, 8E2). S-bus uses inverted signal, requiring either:
- Hardware inverter circuit (NOT gate or transistor)
- ESP32 GPIO inversion feature (if available on C6)
- Software inversion in UART driver

**Rationale**: Hardware UART provides reliable decoding at 100kbaud with minimal CPU overhead. S-bus frames arrive every 7-14ms (70-140Hz).

**Alternatives considered**:
- Software serial: Too unreliable at 100kbaud, high CPU usage
- PWM input: ArduPilot doesn't output individual PWM, only S-bus

### Servo Control: ESP32 LEDC (Hardware PWM)
Use ESP32's LEDC peripheral for PWM generation instead of software PWM.

**Rationale**: Hardware PWM provides precise, jitter-free signals essential for servo stability. ESP32-C6 has 6 LEDC channels available.

### Linear Actuator Control: BTS7960 Dual H-Bridge
Use BTS7960 module with:
- RPWM (Forward PWM)
- LPWM (Reverse PWM)
- R_EN, L_EN (Enable pins - **hardwired to 5V**, not controlled by ESP32)

**Rationale**: BTS7960 provides high current capacity (43A) needed for linear actuators under load, with thermal protection. Enable pins are permanently active (hardwired high), simplifying wiring and pin usage. Motor control achieved through PWM signals only (0% = coast, both high = brake).

### Transmission Control: Position-Based with Detents
Implement 4 discrete positions (R, N, H, L) with calibrated actuator positions.

**Rationale**: Mechanical detents in transmission likely correspond to specific actuator extensions. Store calibration values in EEPROM/NVS.

### Safety: Multi-Level Fail-Safes
1. **Watchdog timer**: System reset if main loop hangs
2. **Input validation**: Range checks on all commands
3. **Emergency stop**: Single function to safe all actuators
4. **Brake priority**: Brakes can override throttle
5. **Neutral default**: Transmission defaults to Neutral on startup

**Rationale**: Vehicle control systems require defense in depth. Each layer catches different failure modes.

## Component Design

### Pin Assignment Strategy
Define all pins in Constants.h with logical naming:
```cpp
// S-bus Input (UART)
#define PIN_SBUS_RX         GPIO_NUM_1   // UART RX with inverted signal
#define SBUS_UART_NUM       UART_NUM_1   // UART1 for S-bus

// Steering Servo (PWM)
#define PIN_STEERING_PWM    GPIO_NUM_2

// Throttle Servo (PWM)
#define PIN_THROTTLE_PWM    GPIO_NUM_3

// Transmission Linear Actuator (BTS7960)
#define PIN_TRANS_RPWM      GPIO_NUM_4
#define PIN_TRANS_LPWM      GPIO_NUM_5
// Note: R_EN and L_EN hardwired to 5V (always enabled)

// Brake Linear Actuator (BTS7960)
#define PIN_BRAKE_RPWM      GPIO_NUM_6
#define PIN_BRAKE_LPWM      GPIO_NUM_7
// Note: R_EN and L_EN hardwired to 5V (always enabled)
```

**Note**: Actual GPIO numbers TBD based on ESP32-C6 DevKit pinout and routing constraints.

### S-bus Channel Mapping
Define default channel mappings in Constants.h:
```cpp
// S-bus Channel Assignments
#define SBUS_CH_STEERING      1   // Channel 1: Steering (-100% to +100%)
#define SBUS_CH_THROTTLE      2   // Channel 2: Throttle (0% to 100%)
#define SBUS_CH_TRANSMISSION  3   // Channel 3: Gear selector (4 positions)
#define SBUS_CH_BRAKE         4   // Channel 4: Brake (0% to 100%)

// S-bus Parameters
#define SBUS_SIGNAL_TIMEOUT   500  // ms before fail-safe activates
#define SBUS_MIN_VALUE        880  // Minimum S-bus channel value
#define SBUS_MAX_VALUE        2160 // Maximum S-bus channel value
```

### Class Structure

#### SBusInput
- `begin(uart, rxPin)`: Initialize UART for S-bus reception
- `update()`: Poll for new frames (call in main loop)
- `getChannel(n)`: Get channel value in microseconds (880-2160)
- `isSignalValid()`: Check if signal is fresh (<500ms old)
- `getSignalAge()`: Time since last valid frame in ms
- `getFrameRate()`: Current frame rate in Hz
- `getSignalQuality()`: Frame success rate percentage

#### CommandMapper
- `mapSteering(sbusValue)`: Convert S-bus to steering % (-100 to +100)
- `mapThrottle(sbusValue)`: Convert S-bus to throttle % (0 to 100)
- `mapGear(sbusValue)`: Convert S-bus to gear enum (R/N/H/L)
- `mapBrake(sbusValue)`: Convert S-bus to brake % (0 to 100)
- `setDeadband(channel, percent)`: Configure channel deadbands

#### ServoController
- `begin(pin, channel, minUs, maxUs)`: Initialize LEDC PWM
- `setAngle(degrees)`: Set servo position (0-180°)
- `setMicroseconds(us)`: Direct pulse width control
- `disable()`: Stop PWM output

#### BTS7960Controller
- `begin(rpwm, lpwm)`: Initialize pins and PWM (enable pins hardwired)
- `setSpeed(speed)`: -255 to +255 (negative=reverse)
- `stop()`: Coast to stop (both PWM = 0)
- `brake()`: Active braking (both PWM = 255)

**Note**: Enable pins (R_EN, L_EN) are hardwired to 5V, so drivers are always active. No enable/disable control needed.

#### SteeringSystem
- `setPosition(percent)`: -100% (full left) to +100% (full right)
- `center()`: Return to center position
- Uses ServoController internally

#### ThrottleSystem
- `setPosition(percent)`: 0% (closed) to 100% (full throttle)
- `idle()`: Return to idle position
- Uses ServoController internally

#### TransmissionSystem
- `setGear(gear)`: Enum {REVERSE, NEUTRAL, HIGH, LOW}
- `calibrate()`: Learn actuator positions for each gear
- `getCurrentGear()`: Return current gear position
- Uses BTS7960Controller internally

#### BrakeSystem
- `setPosition(percent)`: 0% (released) to 100% (full braking)
- `release()`: Fully release brakes
- Uses BTS7960Controller internally

## Risks / Trade-offs

### Risk: Linear Actuator Position Feedback
**Without position sensors**, transmission and brake control rely on time-based movement which may drift.

**Mitigation**:
- Implement calibration routine on startup
- Store calibration in NVS (non-volatile storage)
- Add optional position sensor support (potentiometer/encoder) in future

### Risk: Power Supply Noise
BTS7960 high current switching may cause voltage spikes affecting ESP32.

**Mitigation**:
- Separate power supplies for logic (ESP32) and motors (BTS7960)
- Add bulk capacitors near BTS7960 modules
- Implement brownout detection on ESP32

### Trade-off: Hardwired Enable Pins
Enable pins permanently active means drivers consume quiescent current even when stopped.

**Implications**:
- Slightly higher idle power consumption (~100mA per driver)
- Cannot achieve true high-impedance state
- Brake mode (both PWM high) provides adequate holding force
- Simplifies wiring and reduces ESP32 GPIO usage (saves 4 pins)

**Acceptable because**: Vehicle is powered while operating; quiescent current negligible compared to actuator drive current (multi-amp).

### Risk: Servo Jitter on Steering
Critical for vehicle stability.

**Mitigation**:
- Use hardware PWM (LEDC) with high resolution
- Implement servo signal filtering/smoothing
- Verify 50Hz update rate (20ms period)

## Migration Plan

### Phase 1: Hardware Layer (Week 1)
1. Implement ServoController and BTS7960Controller classes
2. Create test harness for each actuator independently
3. Verify pin assignments and PWM signals with oscilloscope

### Phase 2: System Layer (Week 2)
4. Implement vehicle system classes (Steering, Throttle, Transmission, Brake)
5. Add calibration routines
6. Test each system in isolation

### Phase 3: Integration (Week 3)
7. Integrate all systems in main control loop
8. Implement safety mechanisms (watchdog, emergency stop)
9. System-level testing with full vehicle

### Phase 4: Tuning (Week 4)
10. Calibrate servo ranges and linear actuator positions
11. Tune control response and filtering
12. Validate safety features

## Open Questions

### S-bus and ArduPilot
1. **S-bus Connection**: Direct wire from ArduPilot or separate S-bus receiver? Does it require external signal inverter?
2. **ArduPilot Output Mapping**: Confirm which ArduPilot output channels map to S-bus 1-4
3. **Fail-safe Configuration**: What fail-safe values does ArduPilot send on signal loss? Should ESP32 use them or override?
4. **ArduPilot Modes**: How should different ArduPilot modes (Manual, Auto, Hold) affect vehicle behavior?

### Hardware
5. **Position Sensors**: Are there position sensors on the linear actuators? If yes, analog or digital?
6. **Servo Specifications**: What are the pulse width ranges for the servos (typically 1000-2000μs)?
7. **Linear Actuator Travel**: What is the stroke length of each linear actuator in mm?
8. **Transmission Detents**: Are there mechanical detents or is it continuous positioning?
9. **Power Supply**: What voltage/current ratings for the power supply? Separate supplies for logic and motors?
10. **Emergency Stop**: Hardware e-stop button wired to ESP32 GPIO in addition to S-bus fail-safe?

### Integration
11. **Testing Without ArduPilot**: Need S-bus simulator or test harness for development?
12. **Calibration Procedure**: Should calibration be triggered via S-bus channel or separate mechanism?
