# Change: Odometer and trip meter, persisted in NVS and reported over the existing EFI_STATUS

## Why
The vehicle has no odometer. Distance driven is the number every service interval, fuel estimate
and field report needs, and today it exists nowhere — not on the vehicle, not at the ground
station. The hall sensor already counts every wheel pulse and already knows the millimetres each
pulse represents; the only missing pieces are two accumulators and somewhere to keep them.

Two counters, like a car dashboard: **ODO** never resets; **TRIP** accumulates across ignition
cycles until the operator clears it from Mission Planner. Both survive power loss.

No new MAVLink message and no new NVS namespace: `EFI_STATUS` already flies at 5 Hz with two
permanently unused float fields, and the `speed` namespace is already owned by the sensor that
produces the pulses.

## What Changes
- **Two distance accumulators in `SpeedSensor`.** `odoMm_` and `tripMm_` are `uint64_t`
  **millimetres**, incremented only in the `delta > 0` branch of `update()` by
  `delta × distancePerPulseMm_` — the same arithmetic that already produces the speed. They count
  **real counted pulses only**: the decayed/suspicious estimate emitted during pulse silence adds
  nothing, and a sample rejected by the existing `SPEED_MAX_PULSES_PER_SAMPLE` wrap guard adds
  nothing, because that guard returns before the accumulation. Direction is not considered —
  reverse adds distance exactly like forward, as a car's odometer does.
- **Millimetres, not pulses, are what is stored.** A later `speed_cal_ppr` / `speed_cal_circ`
  recalibration changes millimetres-per-pulse from that moment on; it must not retroactively
  rewrite the distance already driven.
- **Persistence in the existing NVS namespace `speed`**, two new `uint64` keys `odo_mm` and
  `trip_mm`, loaded in `begin()` (a missing key reads 0). Written on exactly three triggers:
  (a) **ignition OFF**, on the transition detected in `VehicleController::processMavlinkCommands()`
  where `previousIgnitionState_` is already tracked; (b) whenever ODO has grown by
  `ODO_NVS_WRITE_INTERVAL_MM` (1 000 000 mm = 1 km) since the last write; (c) immediately on an
  accepted trip reset. **No new config keys, no web commands, no runtime tuning** — the interval is
  a hardcoded constant in `Constants.h`.
- **Reported in `EFI_STATUS`, in two fields that are transmitted as `0.0f` today.**
  `barometric_pressure` ← **ODO in kilometres**, `fuel_pressure` ← **TRIP in kilometres**, both as
  `odo_mm / 1e6` in float. The message, its component (25) and its 5 Hz rate are unchanged; both
  values are **always valid** and never `NaN`, because neither depends on CAN health.
- **A trip reset arrives as a `COMMAND_LONG`.** `MavlinkInterface` gains an inbound
  `MAVLINK_MSG_ID_COMMAND_LONG` case: a command addressed to **this** system *and* component
  (`target_system == MAVLINK_SYSTEM_ID`, `target_component == MAVLINK_COMPONENT_ID`) carrying
  `MAV_CMD_USER_1` (31010) with `param1 == MAVLINK_CMD_TRIP_RESET_MAGIC` zeroes TRIP, persists it,
  and is answered `MAV_RESULT_ACCEPTED`. Any other `param1` is `MAV_RESULT_DENIED`; any other
  command addressed specifically to this component is `MAV_RESULT_UNSUPPORTED`. A broadcast
  (`target_component == 0`) or a command for another component is **ignored silently** — this is a
  peripheral sharing the autopilot's system id and must not answer for it. The `COMMAND_ACK` is
  addressed back to the **sender's** `msg.sysid` / `msg.compid`, and ArduPilot routes it to the GCS
  on the route it already learned from the ESP32's own `EFI_STATUS` / `HEARTBEAT` traffic.
- **BREAKING (`VFR_HUD.groundspeed` contract): an invalid hall reading is now `NaN`, not `0`.**
  `src/MavlinkInterface.cpp:478-488` currently sends `0.0f` whenever `!state.speedValid`, which
  makes "the sensor has never pulsed / is latched suspicious" indistinguishable from "the vehicle is
  stopped". The field becomes `state.speedValid ? state.speedMs : NAN`, so a genuine `0.0` stays
  `0.0` and only an unknown reading is `NaN`. The ground-station widget now reads this field from
  component 25 directly (companion change `add-odometer-trip-readout` in WindowsHelper) and needs
  that distinction; MAVLink permits `NaN` in a float field for "unknown", and component-25 packets
  do not feed Mission Planner's own vehicle state, so its HUD and `cs.*` are unaffected.
  `VFR_HUD.throttle` is unchanged (a `uint16_t` percentage has no `NaN` encoding), and
  `VISION_POSITION_DELTA` is unchanged — it already goes silent on an invalid reading.
- **ODO can never be reset over any interface.** There is no command, no web control and no
  constant that clears it; the only way back to zero is an NVS erase.
- **Web telemetry shows both, read-only.** `odo_km` and `trip_km` (3 decimals) join the telemetry
  JSON unconditionally, and `data/index.html` displays them in the speed section. **No reset button
  in the portal** — the operator sits at the ground station, not at the vehicle's WiFi page.
- **One log line per inbound command**, on the existing `DebugFeature::MAVLINK` channel:
  `[MAV] TRIP reset accepted from 255/190`, `[MAV] TRIP reset DENIED (param1=3.00) from 255/190`,
  `[MAV] command 400 UNSUPPORTED from 255/190`.
- **Not touched:** the speed reading itself, `isValid()` / `isSuspicious()`, the speed limiter,
  `VFR_HUD.throttle`, `VISION_POSITION_DELTA`, the PCNT configuration, the CAN poll schedule, and
  every other `EFI_STATUS` field.

## Impact
- Affected specs:
  - `speed-sensor` — **ADDED** `Odometer and Trip Distance Accumulation` and
    `Odometer and Trip Persistence`.
  - `mavlink-interface` — **MODIFIED** `Vehicle State Reporting via Standard MAVLink Messages`
    (the `EFI_STATUS` field mapping gains ODO and TRIP, the "repurpose only permanently-free
    fields" rule is amended to release `barometric_pressure` and `fuel_pressure`, and the
    `VFR_HUD` ground-speed scenario reports an invalid reading as `NaN`); **MODIFIED**
    `Outbound Speed Reporting in Metres per Second` (the "report zero when invalid" clause becomes
    a `NaN` clause, with `VFR_HUD.throttle` and `VISION_POSITION_DELTA` explicitly unaffected);
    **ADDED** `Inbound Command Handling and Trip Reset`.
  - `web-telemetry` — **ADDED** `Odometer and Trip Telemetry`.
- Affected code: `include/Constants.h`, `include/SpeedSensor.h` + `src/SpeedSensor.cpp`,
  `include/VehicleController.h` + `src/VehicleController.cpp`, `include/MavlinkInterface.h` +
  `src/MavlinkInterface.cpp`, `src/TelemetryManager.cpp`, `include/WebPortal.h` +
  `src/WebPortal.cpp`, `data/index.html`, `MAVLINK_SETUP.md`, `WEB_PORTAL_SETUP.md`.
- `data/` changes, so this needs **both** `pio run -t upload` and `pio run -t uploadfs`.
- Out of repo: the QuadBike Mission Planner plugin gains two read-only fields, the trip-reset
  `COMMAND_LONG`, and the component-25 `VFR_HUD.groundspeed` readout — companion change
  `add-odometer-trip-readout` in the WindowsHelper repository. Nothing it decodes today moves, so
  an un-updated plugin keeps working and simply does not show the new values; a consumer that
  *does* read `VFR_HUD.groundspeed` from component 25 MUST handle `NaN` (render "--"), which is
  the only behavioural break in this change.
- **Field behaviour on first flash:** both keys are absent, so ODO and TRIP start at 0. The
  odometer therefore reads "distance since this firmware", not "distance since the vehicle was
  built"; there is no back-dating mechanism and none is wanted.

## Sequencing
This change's `mavlink-interface` delta **MODIFIES** `Vehicle State Reporting via Standard MAVLink
Messages`, the same requirement carried as a MODIFIED block by the active `add-ecu-telemetry-pids`
and `remap-efi-native-fields` changes. Spec deltas apply to `openspec/specs/` **as it exists at
archive time**, so this change MUST archive **after** both of them:

1. `add-ecu-telemetry-pids`
2. `remap-efi-native-fields`
3. `add-odometer-and-trip` (this change)

**This change's MODIFIED block is written as a superset of
`openspec/changes/remap-efi-native-fields/specs/mavlink-interface/spec.md`, not of the current
`openspec/specs/` text**, exactly as that change is a superset of its own two predecessors.
Archiving this one first would overwrite the native field remap with the older single-gear mapping.
Before archiving, diff this delta against `remap-efi-native-fields`'s and confirm the only
differences are the ODO/TRIP additions and the `VFR_HUD` `NaN` clause.

The second MODIFIED requirement, `Outbound Speed Reporting in Metres per Second`, adds **no**
sequencing constraint: no other active change touches it (it was last written by the archived
`add-mavlink-speed-max-limit`), so its full text is copied from the current `openspec/specs/`.

`add-extnav-velocity` is not a sequencing constraint: its `mavlink-interface` delta is ADDED-only
and touches different requirements. The new `Inbound Command Handling and Trip Reset` requirement
is likewise ADDED and orthogonal — nothing else in the repo handles inbound `COMMAND_LONG`.
