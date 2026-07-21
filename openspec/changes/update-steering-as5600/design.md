# Design: AS5600 + BTS7960 Steering Control

## Context

Steering feedback moves from a relative quadrature encoder (PCNT) to an AS5600 absolute magnetic angle sensor, and the drive moves from digital bang-bang to proportional PWM through the existing `BTS7960Controller`. The AS5600 magnet sits on the steering shaft and lock-to-lock travel is under one full turn, so a single 12-bit reading is fully absolute.

## Decisions

### Sensor driver: minimal in-repo class
`AS5600Sensor` implemented in-repo using `Wire`. `STATUS` (reg 0x0B, MD bit = magnet detected) is directly followed by `RAW ANGLE` (reg 0x0C/0x0D, 12-bit), so a single 3-byte burst read returns both — the magnet check runs every control-loop iteration at no extra I2C transaction (~170 µs per read at 400 kHz). ~60 lines.
- Alternative considered: `robtillaart/AS5600` PlatformIO library — rejected; we need two register reads and the project already favors small in-repo drivers.
- I2C: `Wire.begin(SDA=GPIO 41, SCL=GPIO 42)`, 400 kHz, fixed address 0x36. These are the freed encoder pins, so no wiring-harness pin changes beyond the sensor itself.

### Units: raw 12-bit counts end-to-end
All positions (center, limits, tolerance, telemetry) are AS5600 counts (0–4095, ≈0.088°/LSB). No float degrees anywhere — matches the existing integer-count style and avoids conversion noise.

### Wrap-safe relative position
The magnet's zero crossing may fall inside the travel range. All comparisons use the signed shortest delta relative to the calibrated center:

```
rel = ((raw - center + 2048) & 4095) - 2048   // -2048..+2047, positive = right
```

Targets, limits, and errors are computed in this `rel` space; wrap never appears above the sensor layer.

### Asymmetric percent mapping
`-100%` → left limit, `0%` → center, `+100%` → right limit, interpolated per side (left and right ranges may differ). This replaces today's symmetric `2×center` model and is why both limits must be calibrated.

### Control loop: proportional PWM with backstops
In `update()` (called every loop):

```
error = target_rel - current_rel                  // counts
if |error| <= STEER_POSITION_TOLERANCE: stop()
duty  = clamp(|error| * STEER_KP, STEER_PWM_MIN_DUTY, STEER_PWM_MAX_DUTY)
bts.setSpeed(sign(error) * duty)                  // -255..+255
```

- `STEER_PWM_MIN_DUTY` overcomes static friction so the actuator doesn't crawl-stall near target.
- Kept as safety backstops (re-scoped to AS5600 counts): move timeout (`STEER_MOVE_TIMEOUT`) and stall detection (position delta < `STEER_STALL_THRESHOLD` over `STEER_STALL_TIMEOUT` while driving → stop).

### Drive: compose existing BTS7960Controller
`SteeringController` owns a `BTS7960Controller` initialized with `PIN_STEER_RPWM`/`PIN_STEER_LPWM` on free LEDC channels 6/7 (same 10 kHz `MOTOR_PWM_FREQ` timer setup as the brake). Removes the raw `digitalWrite` path and reuses the guaranteed never-both-high logic.

### Sensor-fault safety
Any I2C read failure or MD bit clear ⇒ `stop()` immediately, mark sensor invalid, log via `DebugFeature::SERVO`. While invalid, position commands are ignored (motor stays stopped); recovery is automatic when reads succeed again. Telemetry exposes a magnet/sensor-ok flag.

### Boot behavior
No homing. `begin()` initializes I2C and checks magnet presence; if OK and calibration exists, move to center (same end state as today's home-then-center). If the magnet is missing or calibration is absent, stay stopped.

### Calibration: capture-in-place, NVS-persisted
NVS namespace `steering`, new keys: `c_ang`, `l_ang`, `r_ang` (old `center` key is ignored/stale). Uncalibrated ⇒ percent commands rejected, actuator holds still.

Web flow: user jogs the wheel with momentary left/right jog commands, then presses "Set center" / "Set left limit" / "Set right limit" — each captures the *current* AS5600 angle and persists it. This replaces the numeric `set_steer_center` count entry and the `move_steer_center` raw-count preview (a jog replaces the preview move). Sanity check on save: left and right must be on opposite sides of center and at least a minimum span apart.

## Risks / Trade-offs

- **P-only control** can leave a small steady-state error under load; tolerance + min-duty are tuned to make this acceptable. PID is deliberately out of scope until bench data says otherwise.
- **I2C in the control path**: a hung bus would freeze feedback — mitigated by `Wire` timeouts and the sensor-fault stop.
- **Recalibration required** after flashing (old counts are meaningless in AS5600 space). Called out in the proposal as breaking.

## Migration

1. Flash firmware + `pio run -t uploadfs` (web UI changes).
2. Physically install AS5600 (shaft magnet + board on SDA 41 / SCL 42); remove quadrature encoder.
3. Calibrate via web UI: jog to center → Set center; jog to left lock → Set left limit; jog to right lock → Set right limit.
