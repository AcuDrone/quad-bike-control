# Change: Aztec buggy relay assignment

## Why
The aztec buggy uses the same relay PCB as the quad bike (PCA9557 @ `0x1F` on I2C2 / X15, same
non-1:1 IO routing), but its harness is wired differently. The quad-bike masks would drive the
starter from the ignition request, the front light from the starter request, and energise two of
the buggy's lock relays as a "front-wheel lock" that does not exist on this vehicle.

## What Changes
- **Relay map (aztec):** Relay_1 (IO4) = ignition / ECU line, now a **single** relay (no longer
  paired with Relay_2); Relay_2 (IO5) = starter solenoid; Relay_3 (IO6) = front light.
  `RELAY_MASK_IGNITION = 0b00010000`, `RELAY_MASK_STARTER = 0b00100000`,
  `RELAY_MASK_FRONT_LIGHT = 0b01000000`.
- **Reserved relays:** Relay_4 (IO7), Relay_5 (IO3), Relay_6 (IO2) drive three separate locks whose
  functions are specified in a later change; Relay_7 (IO0) and Relay_8 (IO1) are spare. All five are
  grouped as `RELAY_MASK_RESERVED = 0b10001111` and written 0 outside the web relay test.
- **Web relay test (wiring check):** a "Relay Test" card in the web UI has an on/off toggle for each of
  the eight relays (labelled 1 ignition, 2 starter, 3 light, 4–6 lock (unassigned), 7–8 spare; uk +
  en). WebSocket command `{"cmd":"set_relay_test","relay":1..8,"value":bool}` →
  `VehicleController::processRelayTestCommand()` → `RelayController::setRelayTest()`. A relay can be
  switched ON only with the ignition `OFF`, no crank armed and the front light off; relay numbers
  outside 1..8 are rejected. Test bits share the shadow byte and verified `writePort()` with the
  functions via a relay-number → IO-bit lookup (`relayIoMask()`, static_assert-checked against the
  masks). Each test relay auto-releases after `RELAY_TEST_TIMEOUT_MS` (30 s); the starter relay
  (Relay_2) after `RELAY_TEST_STARTER_MAX_MS` (2 s). All test relays go off immediately on any
  ignition/crank/light-on command (web or MAVLink), `allOff()`, and a relay write fault / starter
  escalation. Telemetry gains `relay_test` (bit0 = Relay_1 … bit7 = Relay_8, from the shadow byte).
  Not gated by input source — the steering-calibration jog, the comparable bench command, has no
  input-source gate either; the ignition-OFF gate covers MAVLink taking over.
- **Front-wheel lock not fitted (BREAKING for the wheel-lock function):** `RELAY_MASK_WHEEL_LOCK` is
  removed and `RELAY_WHEEL_LOCK_FITTED = 0`. `RelayController::setWheelLock()` stays (so the MAVLink
  servo channel 7 and the web `set_wheel_lock` command keep compiling) but rejects a LOCK request,
  logs that once per boot, and never touches the port; `getWheelLock()` always returns `false`.
- **Telemetry / UI:** web telemetry gains `wheel_lock_fitted` (bool, `false` on aztec);
  `wheel_lock_on` stays `false`. The web UI disables the wheel-lock button and shows "not fitted".
  The web `set_wheel_lock` command is answered with an error response. The MAVLink `EFI_STATUS`
  digital-output bitmask keeps its layout; bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) is always 0.
- **Unchanged:** the IO routing, whole-port shadow writes with read-back verification, starter
  escalation, pre-crank dwell, crank timeout and RPM exit. The servo-channel mapping and telemetry
  bit layout are kept for the later change that replaces the wheel lock with the three locks.
- Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6 (aztec tables and harness note; the quad-bike assignment
  lives on `master`), `MAVLINK_SETUP.md`, `include/Constants.h`, `include/RelayController.h`.
  Changelog: three lines added to the existing 1.2.0 entry (no version bump).

## Impact
- Affected specs: `gpio-expander`, `mavlink-interface`, `vehicle-systems`, `web-control`,
  `web-telemetry`
- Affected code: `include/Constants.h`, `include/RelayController.h`, `src/RelayController.cpp`,
  `include/VehicleController.h`, `src/VehicleController.cpp`, `include/WebPortal.h`,
  `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `src/MavlinkInterface.cpp` (comment),
  `src/main.cpp` (comment), `data/index.html`, `include/Changelog.h`
- Hardware: the aztec harness must be wired ignition → X4, starter → X5, front light → X6, locks →
  X7/X8/X9. A quad-bike harness on this firmware would crank from the ignition request.
- The `vehicle-actuators` spec only lists `PIN_WHEEL_LOCK` as a removed GPIO define and needs no
  change.
