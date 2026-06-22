# Change: Replace SBUS input with a MAVLink 2 vehicle interface

## Why
The link between the Pixhawk 2.4.8 and the ESP32 is currently SBUS-Out: a one-way,
inverted-UART RC protocol that carries actuator commands but gives the autopilot no
visibility into vehicle state (engine RPM, temperatures, gear, ignition). A single
MAVLink 2 connection on a dedicated telemetry port replaces SBUS, carries the same
per-channel commands via `SERVO_OUTPUT_RAW`, and adds a return path so the ESP32 can
report vehicle state back to the MAVLink network. This removes the SBUS dependency and
the inverter circuit, and unifies command + telemetry on one bidirectional wire.

## What Changes
- **BREAKING**: Remove the SBUS transport. Drop the Bolder Flight SBUS library and the
  `SBusInput` class; retire the `sbus-input` capability.
- **Add** a `mavlink-interface` capability and a `MavlinkInterface` class that:
  - Speaks MAVLink 2 over a dedicated UART to Pixhawk TELEM2 at 115200 baud.
  - Decodes `SERVO_OUTPUT_RAW` (servo PWM microseconds) at the autopilot's stream rate
    (target 50 Hz) and exposes the same typed command API the vehicle layer already
    consumes (`getSteering`/`getThrottle`/`getGear`/`getBrake`/`getIgnitionState`/
    `getFrontLight`).
  - Detects command link loss (no fresh `SERVO_OUTPUT_RAW`/`HEARTBEAT` within the
    timeout) and signals fail-safe, replacing the SBUS frame-loss/fail-safe flag logic.
  - Reports vehicle state back to the MAVLink network using **standard messages**
    (`HEARTBEAT`, `EFI_STATUS` for engine RPM / coolant temp / throttle) sourced from the
    existing CAN `VehicleData`, plus a `NAMED_VALUE_FLOAT` (`GEAR`, sequence-encoded
    `[R,N,H,L]=[-1,0,1,2]`, midpoint between gears while shifting so multi-step changes render
    as a 0.5 staircase; the assumed/commanded gear) so the GCS shows gear as a live value, and
    `STATUSTEXT` for human-readable state transitions
    (gear, ignition state, fail-safe entry/exit). Vehicle speed and oil temperature are not
    reported (not available from the ECU).
- **Reuse** the existing microsecond→command mapping (`SERVO_OUTPUT_RAW` values are µs,
  identical in range to the old SBUS µs values), the per-function range constants, and
  the `RelayController`. The function→channel map (steering=1, throttle/brake=2,
  transmission=3, ignition=4, front light=6) is preserved against servo-output indices.
- **Keep** the combined ignition model (one channel selects OFF/ACC/IGNITION, reaching
  IGNITION auto-cranks with RPM/timeout cutoff) and the CAN bus as the source of engine
  RPM/temperatures.
- **Change** input-source arbitration from `SBUS > WEB > FAILSAFE` to
  `MAVLINK > WEB > FAILSAFE`. The Web Portal stays as the secondary control + telemetry
  source.

## Impact
- **Affected specs**:
  - `mavlink-interface` (new capability — ADDED)
  - `sbus-input` (REMOVED — capability retired; reusable parts migrate to
    `mavlink-interface`)
  - `vehicle-systems` (MODIFIED — multi-source arbitration and fail-safe now keyed on
    MAVLink; input-source telemetry naming)
  - `web-control` (MODIFIED — manual-control gating and source-priority now keyed on
    MAVLink instead of S-bus)
  - `web-telemetry` (MODIFIED — broadcast/display MAVLink link + decoded command
    telemetry instead of raw SBUS channels)
- **Affected code**:
  - New: `include/MavlinkInterface.h`, `src/MavlinkInterface.cpp`
  - Removed: `include/SBusInput.h`, `src/SBusInput.cpp`
  - `platformio.ini` (remove `bolderflight/SBUS`, add MAVLink C library)
  - `include/Constants.h` (UART/pin config, message rate, link-loss timeout; rename
    `SBUS_*` range constants to transport-neutral names; rename `InputSource::SBUS`)
  - `src/main.cpp` (instantiate `MavlinkInterface`, call `update()`/reporting in loop)
  - `src/VehicleController.cpp` + `include/VehicleController.h` (consume
    `MavlinkInterface` instead of `SBusInput`)
  - `src/TelemetryManager.cpp` + `include/TelemetryManager.h` (arbitration + telemetry
    source rename)
  - `src/WebPortal.cpp` + `data/index.html` (link/command telemetry rendering)

## Out of Scope
- TELEM1 → companion computer integration (future).
- Moving engine RPM/temperature off the CAN bus to direct ESP32 inputs.
- A separate starter command channel (ignition/starter stays combined).
- Parameter (`PARAM_*`) or mission protocol support; only command decode + state
  reporting are in scope.
- Tuning of existing channel ranges, deadbands, or motion profiles.
