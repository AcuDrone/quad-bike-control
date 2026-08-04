# Change: Replace steering BTS7960 H-bridge with a Flipsky 75200 VESC (UART, brushed-DC mode)

## Why

- The steering motor is currently driven by a BTS7960 H-bridge that has **no
  current sensing, no electrical braking, and no fault reporting**. Overload
  protection is purely inferred from AS5600 position (stall = "no movement"),
  which is coarse and only works when the sensor and control loop cooperate.
- **A discovered bug defeats all steering stall protection in MAVLink mode.**
  `VehicleController::processMavlinkCommands()` calls
  `steering_.setSteeringPercent()` unconditionally on **every** main-loop
  iteration (`src/VehicleController.cpp:258-260`), and `setSteeringPercent()`
  resets `moveStartTime_`, `lastStallCheckTime_`, `dutyBoost_`, and the
  progress trackers on **every** call (`src/SteeringController.cpp:249-258`).
  Because ArduPilot streams steering at ~25 Hz, the 500 ms stall window and
  7500 ms move timeout are reset before they can ever elapse, and the
  anti-stall boost never accumulates. Net effect: with a blocked wheel the
  motor is driven at up to duty 255 continuously and never stops. This is a
  safety defect on a safety-critical actuator.
- The user owns a **Flipsky 75200 VESC** (75 V / 200 A class, VESC 6 protocol).
  In brushed-DC motor mode it provides a hardware motor-current limit, plus
  UART telemetry (motor current, FET temperature, input voltage, fault codes)
  that give the firmware a real over-current/fault backstop instead of an
  inferred one.

## What Changes

- **Motor-driver abstraction.** Introduce a small `IMotorDriver` interface
  (`setSpeed(int16_t -255..+255)`, `stop()`). `SteeringController` drives the
  abstraction instead of a concrete `BTS7960Controller`. The existing AS5600
  position P-loop is **unchanged** — its `-255..+255` output is mapped to a
  VESC duty command.
- **VESC UART driver.** Add a VESC serial driver that maps `setSpeed()` to
  `COMM_SET_DUTY` (`duty = speed / 255.0`, clamped `-1.0..+1.0`) and polls
  `COMM_GET_VALUES` for telemetry. A **minimal in-repo VESC packet layer**
  (framing + CRC16, two commands only) is added rather than a heavy external
  dependency (see design.md).
- **Stall-protection fix + latch.** (a) Add a **re-command guard**:
  `setSteeringPercent()` does not restart the move/stall timers when the new
  target is within tolerance of the current target (no-op), so the stall
  window and move timeout can actually elapse in MAVLink mode. (b) Add a
  **stall latch**: after a stall-stop, moves that push further in the stalled
  direction are refused for a cooldown, while opposite-direction moves and
  post-cooldown retries are always accepted (must-not-brick guarantees in
  design.md).
- **VESC telemetry monitor.** Poll VESC at ~3 Hz; a sustained motor current
  above a firmware threshold or a VESC fault code routes into the same
  stall-stop/latch path and raises a telemetry flag.
- **VESC-unresponsive failsafe.** If the VESC stops replying to
  `COMM_GET_VALUES` for N cycles, treat it like the existing sensor-fault
  path: stop commanding, reject moves, flag `steer_driver_ok=false`. The
  vehicle remains safe with the VESC disconnected.
- **Telemetry + UI.** Add steering motor current (A), VESC FET temp (°C), and
  a VESC fault / driver-ok flag to the WS telemetry JSON and a compact web-UI
  display (new labels require i18n keys in **both** `en` and `uk`
  dictionaries).
- **Configuration + wiring.** Document the VESC Tool configuration procedure
  (DC mode, motor current limit, UART baud, comm timeout) and the physical
  rewiring (motor across two VESC phases; ESP32 UART ↔ VESC UART at 3.3 V).
- `BTS7960Controller` is **retained** — the brake actuator still uses it; only
  the steering actuator stops using it. **BREAKING** for the steering wiring
  and the `SteeringController::begin()` signature.
- MAVLink interface behavior is unchanged.

## Impact

- **Affected specs:** `vehicle-actuators` (steering driver, safety limits,
  stall protection, failsafe), `web-telemetry` (steering motor telemetry).
- **Affected code:** `include/SteeringController.h`, `src/SteeringController.cpp`,
  new `IMotorDriver` / `VescMotorDriver` / `VescProtocol` sources,
  `src/VehicleController.cpp` (re-command bug), `include/Constants.h` (VESC
  pins, baud, thresholds, cooldown), `src/main.cpp` (steering init),
  `include/WebPortal.h` + `src/TelemetryManager.cpp` + `src/WebPortal.cpp`
  (telemetry fields), `data/index.html` (UI + i18n en/uk).
  `BTS7960Controller` is unchanged (brake keeps it).
- **Deployment:** firmware flash **and** `uploadfs` (web assets) **and** VESC
  Tool configuration **and** physical rewiring. Steering is safety-critical:
  **on-bench validation (calibration, jog, position moves, deliberate stall
  test, VESC-disconnect failsafe) is required before the vehicle is driven.**
