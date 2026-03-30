## Context
The ESP32-C6 has limited GPIOs. Currently 8 GPIOs are used for simple digital I/O (relays, gear sensors, brake sensor) that don't require PWM or special peripherals. Moving these to an MCP23017 I2C expander frees those GPIOs for PWM-capable peripherals (e.g., additional BTS7960 motor driver needing 2 LEDC channels).

## Goals / Non-Goals
- Goals: Provide a reusable MCP23017 wrapper; migrate relays, gear sensors, and brake sensor to it; free 8 ESP32 GPIOs
- Non-Goals: PWM output via MCP23017 (not supported by hardware); interrupt-driven input (can add later if needed)

## Decisions

### MCP23017 Pin Allocation
- **Port A (GPA0–GPA7)**: Outputs — relays and future expansion
  - GPA0: Relay 1 (main power)
  - GPA1: Relay 2 (accessory power)
  - GPA2: Relay 3 (front light / safety)
  - GPA3–GPA7: Available for future outputs
- **Port B (GPB0–GPB7)**: Inputs with pull-ups — sensors
  - GPB0: Gear selector REVERSE (active-low)
  - GPB1: Gear selector NEUTRAL (active-low)
  - GPB2: Gear selector LOW (active-low)
  - GPB3: Gear selector HIGH (active-low)
  - GPB4: Brake sensor (active-low)
  - GPB5–GPB7: Available for future inputs

### I2C Configuration
- Default I2C address: 0x20 (all address pins grounded)
- I2C speed: 400 kHz (fast mode — MCP23017 supports up to 1.7 MHz)
- I2C pins: to be determined (will be defined in Constants.h)

### MCP23017Controller Class Design
```
class MCP23017Controller {
public:
    bool begin(uint8_t sdaPin, uint8_t sclPin, uint8_t address = 0x20);

    // Pin I/O
    void pinMode(uint8_t pin, uint8_t mode);  // INPUT, OUTPUT, INPUT_PULLUP
    void digitalWrite(uint8_t pin, uint8_t value);
    uint8_t digitalRead(uint8_t pin);

    // Bulk operations (faster — single I2C transaction)
    void writePort(uint8_t port, uint8_t value);  // port 0=A, 1=B
    uint8_t readPort(uint8_t port);

    bool isInitialized() const;
};
```

### Integration Pattern
- `MCP23017Controller` is instantiated as a global in `main.cpp` (like other controllers)
- Passed by reference to `RelayController` and `TransmissionController`/`VehicleController`
- Controllers call `mcp.digitalWrite()` / `mcp.digitalRead()` instead of native GPIO functions
- I2C read latency is ~100μs at 400 kHz — negligible for relay switching and sensor polling at 100Hz

### Polling vs Interrupts
Using polling (read in main loop) rather than MCP23017 interrupt pin. Rationale:
- Gear selector and brake sensor are polled at 100Hz already — no latency requirement
- Saves one ESP32 GPIO (no INT pin needed)
- Simpler implementation
- Can add interrupt support later if needed

## Risks / Trade-offs
- **I2C bus failure**: If MCP23017 becomes unresponsive, relays and sensor reads will fail. Mitigation: `isInitialized()` check; fallback to safe state (all relays off) on I2C error.
- **Latency**: I2C reads add ~100μs per operation. Using bulk `readPort()` for all gear sensors in one transaction minimizes this.
- **Library dependency**: Adafruit MCP23017 library is well-maintained and widely used in Arduino ecosystem.

## Open Questions
- Which ESP32-C6 GPIOs to use for I2C SDA/SCL (to be decided by user)
