## Context
The steering system currently uses a hobby servo (ServoController) driven by LEDC PWM. This is being replaced with a BTS7960 H-bridge motor driver and hall sensor encoder for more torque and closed-loop position control. The motor runs at full speed only (no PWM modulation needed), so plain GPIO digital write is used instead of LEDC channels.

## Goals / Non-Goals
- Goals: Replace servo with BTS7960 + encoder; auto-home and calibrate center; enforce software steering limits; persist calibration in NVS
- Non-Goals: Proportional speed control (full speed only); interrupt-driven encoder reading; steering rate limiting (can add later)

## Decisions

### GPIO Pin Assignment
Using freed ESP32 GPIOs (from MCP23017 migration):
- GPIO 9: Steering BTS7960 RPWM (digital out — full speed right)
- GPIO 10: Steering BTS7960 LPWM (digital out — full speed left)
- GPIO 11: Steering encoder channel A (PCNT Unit 1)
- GPIO 12: Steering encoder channel B (PCNT Unit 1)

R_EN and L_EN pins hardwired to 5V (always enabled), same as transmission and brake BTS7960.

### No LEDC — Plain GPIO Digital Write
Since the steering motor runs at full speed only, RPWM/LPWM are driven by `digitalWrite(pin, HIGH/LOW)` rather than LEDC PWM. This:
- Avoids consuming 2 LEDC channels (all 6 are allocated)
- Frees LEDC Ch0 + GPIO 13 (previously steering servo)
- Simplifies the driver

### SteeringController Class Design
New class (not extending BTS7960Controller, since that requires LEDC):
```
class SteeringController {
public:
    bool begin(gpio_num_t rpwmPin, gpio_num_t lpwmPin);
    void attachEncoder(EncoderCounter* encoder);

    // Movement (full speed only)
    void moveRight();    // RPWM=HIGH, LPWM=LOW
    void moveLeft();     // RPWM=LOW, LPWM=HIGH
    void stop();         // Both LOW

    // Position control
    void setPosition(int32_t targetPosition);
    void update();       // Call in main loop — stops when at target
    bool isAtPosition(int32_t target, int32_t tolerance) const;
    int32_t getPosition() const;
    bool isMoving() const;

    // Calibration
    bool autoHome(uint32_t timeout = 30000);  // Home to limit, reset encoder to 0
    void setCenter(int32_t centerPosition);
    void setLimits(int32_t leftLimit, int32_t rightLimit);
    int32_t getCenter() const;
    int32_t getLeftLimit() const;
    int32_t getRightLimit() const;

    // Steering interface (percentage to position)
    void setSteeringPercent(float percent);  // -100 (left) to +100 (right)
    float getSteeringPercent() const;

    // NVS persistence
    bool saveCalibration();
    bool loadCalibration();
    void clearCalibration();
    bool isCalibrated() const;
};
```

### Startup Sequence
1. Initialize encoder (PCNT Unit 1)
2. Initialize SteeringController with GPIO pins
3. Attach encoder
4. Load calibration from NVS (center + limits)
5. Auto-home to physical limit (move left until stall, reset encoder to 0)
6. Move to calibrated center position
7. Ready for steering commands

### Position Control Logic
Simple bang-bang control at full speed:
- If `target - current > tolerance`: moveRight()
- If `current - target > tolerance`: moveLeft()
- Otherwise: stop()

No speed ramping — the mechanical load of the steering provides natural deceleration. If overshoot occurs, the controller corrects immediately.

### Steering Percentage Mapping
```
encoderTarget = center + (percent / 100.0) * halfRange
```
Where:
- `percent` ranges from -100 (full left) to +100 (full right)
- `halfRange = min(center - leftLimit, rightLimit - center)` (symmetric range)
- Software limits prevent commanding beyond `leftLimit` / `rightLimit`

### Calibration
Manual calibration via web portal (similar to transmission):
1. Auto-home to left limit (stall detection)
2. User triggers "set center" command when steering is centered
3. User triggers "set right limit" when at max right
4. Left limit = 0 (home position)
5. All values stored in NVS

### NVS Keys
- `steer_center`: Center encoder position
- `steer_left`: Left limit encoder position (typically 0 or near 0)
- `steer_right`: Right limit encoder position

### Stall Detection
Same approach as BTS7960Controller auto-home:
- Monitor encoder for movement (threshold: 3 counts)
- Stall = no movement for 1 second
- On stall during auto-home: stop, reset encoder to 0

### Safety
- On signal loss (failsafe): move to center position
- On I2C/encoder failure: stop motor immediately
- Software limits enforced: never command beyond leftLimit/rightLimit

## Risks / Trade-offs
- **Overshoot**: Full speed + bang-bang control may cause oscillation around target position. Mitigation: reasonable tolerance (e.g., 10-20 encoder counts). If problematic, can add a deceleration zone later.
- **No speed ramping**: Mechanical steering load provides natural resistance. If this proves insufficient, can add a slow-down zone near target in future.
- **Encoder failure**: If encoder disconnects, motor would run indefinitely. Mitigation: timeout on any movement command (same as transmission TRANS_MOVE_TIMEOUT pattern).

## Open Questions
- None — GPIO pins and design decisions are settled.
