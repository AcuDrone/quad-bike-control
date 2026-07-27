# Update Steering to AS5600 Absolute Sensor + BTS7960 PWM

## Why

Steering position feedback currently comes from an incremental quadrature hall encoder (PCNT on GPIO 42/41). Because counts are relative, the actuator must auto-home by stalling into the left mechanical limit on every boot — slow, noisy, and mechanically stressful — and any missed counts or power loss corrupts the position until the next home. The BTS7960 is also driven as plain digital HIGH/LOW (full speed bang-bang), which overshoots and oscillates around the target.

Replacing the encoder with an AS5600 absolute magnetic angle sensor (magnet on the steering shaft, < 360° lock-to-lock) gives true absolute position at all times — no homing, no drift — and switching the BTS7960 to proportional PWM lets the actuator slow down as it approaches the target for smooth, precise centering.

## What Changes

- New minimal in-repo `AS5600Sensor` driver (I2C via `Wire` on GPIO 41/42, addr 0x36): raw 12-bit angle + magnet-detect status. No new library dependency.
- `SteeringController` reworked:
  - Position feedback from AS5600 (wrap-safe 12-bit counts) instead of `EncoderCounter`.
  - Drive via the existing `BTS7960Controller` (LEDC channels 6/7, 10 kHz) with proportional P-control speed instead of raw `digitalWrite` bang-bang.
  - Auto-home removed entirely; boot verifies magnet presence and moves to calibrated center.
  - Calibration model changes from "home=0, center count, right=2×center" to explicit **center + left limit + right limit angles** stored in NVS, captured from the current wheel position via web UI.
  - Sensor-fault safety: I2C failure or magnet loss stops the motor immediately.
- `EncoderCounter` deleted (steering was its last user).
- Web calibration commands and `data/index.html` calibration UI updated (capture-current-position buttons; live angle readout); telemetry `steer_position`/`steer_center` now in AS5600 counts plus limits and magnet status.
- **BREAKING**: existing NVS steering calibration (`steering/center` encoder counts) is invalidated; steering must be recalibrated (center + both limits) after flashing. Web UI requires `pio run -t uploadfs`.

## Impact

- Affected specs: `vehicle-systems` (Steering Control System), `vehicle-actuators` (BTS7960 Motor Driver Control)
- Affected code:
  - New: `include/AS5600Sensor.h`, `src/AS5600Sensor.cpp`
  - Reworked: `include/SteeringController.h`, `src/SteeringController.cpp`
  - Deleted: `include/EncoderCounter.h`, `src/EncoderCounter.cpp`
  - Touched: `include/Constants.h`, `src/main.cpp`, `src/VehicleController.cpp` (+ header), `src/TelemetryManager.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`, `data/index.html`
- Hardware: AS5600 wired to 3V3/GND + SDA (GPIO 41) / SCL (GPIO 42); quadrature encoder removed
