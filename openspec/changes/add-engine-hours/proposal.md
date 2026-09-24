# Change: Engine hour meter, persisted in NVS and reported over the existing EFI_STATUS

## Why
The vehicle has an odometer but no hour meter. Distance is the wrong unit for most of what this
machine wears out: oil, filters, belts, valve clearances and the ECU's own duty cycle track
**running time**, not kilometres, and a quad bike spends a great deal of its life idling, winching,
manoeuvring and standing still with the engine turning — hours that the odometer records as zero.
Every service schedule this engine has is written in motohours ("мотогодини"), and today that
number exists nowhere: not on the vehicle, not at the ground station, not in a log.

The ECU already publishes engine RPM at 5 Hz over the CAN bus, and the main loop already knows what
time it is. The only missing pieces are one accumulator and somewhere to keep it.

Two counters, like the odometer's ODO and TRIP: a **total** that only ever increases and has no
reset on any interface, and a **trip** total ("мотогодини місії") that answers "how long did the
engine run on this mission" and is cleared by the *existing* trip-reset command. Both survive power
loss. No new MAVLink message, no new command and no new message rate: `EFI_STATUS` already flies at
5 Hz with two permanently unused float fields.

## What Changes
- **A new `EngineHourMeter` class** (`include/EngineHourMeter.h` + `src/EngineHourMeter.cpp`, one
  class per pair per project convention), owned by `VehicleController` as a member and updated once
  per `VehicleController::update()` with the CAN snapshot that function already reads. It is a
  counter and a persistence policy and nothing else: no actuator, no validity flag of its own, no
  interaction with the speed sensor.
- **The counting rule is "the crank is turning under CAN's own testimony".** Time accrues only
  while `VehicleData.dataValid` is true **and** `engineRPM >= ENGINE_HOURS_MIN_RPM`. While CAN is
  invalid or stale, nothing accumulates — an unknown engine state must not invent hours, the same
  principle by which the odometer ignores the decayed speed estimate. The elapsed interval is
  **discarded**, not banked, so a CAN outage cannot be paid back later.
- **A NEW threshold constant, `ENGINE_HOURS_MIN_RPM` (300 RPM)**, deliberately NOT a reuse of
  `ENGINE_RUNNING_RPM_THRESHOLD` (1500). That existing constant is a *gear-change safety* threshold
  — it answers "is it unsafe to shift / is it unsafe to crank" — and it sits far above this
  engine's idle. Reusing it would build an hour meter that does not count idling, which is exactly
  the running time an hour meter exists to record. Nothing in the repository documents this
  engine's idle RPM, so the threshold is set low (300) on the physical argument that anything above
  ~300 RPM means the crank is turning under its own power or under the starter, and a few seconds
  of cranking per start is far below the resolution anyone reads an hour meter at.
- **Wall-clock deltas, not a fixed tick.** The increment is the measured `millis()` difference
  since the previous update, so the total tracks real time regardless of the loop rate. A single
  delta longer than `ENGINE_HOURS_MAX_DELTA_MS` (5 s) is **discarded whole** — a stalled loop, a
  long blocking operation or a `millis()` wrap must not be able to inject time.
- **A second, RESETTABLE counter: trip engine hours** ("мотогодини місії"), the hour-meter analogue
  of the odometer's TRIP distance. `tripSeconds_` sits beside `engineSeconds_` in the same class and
  is incremented by the **same whole-second amount, in the same place, from the single shared
  sub-second carry** — the two advance in lockstep and differ only in when they were last zeroed.
  Exposed as `getTripSeconds()` / `getTripHours()`.
- **Storage is `uint64_t` whole seconds** with a `uint32_t` millisecond remainder carried between
  updates, so nothing is lost to truncation and the accumulation is exact. Hours are a
  presentation: `seconds / 3600.0f`.
- **Persistence in a NEW NVS namespace `"engine"`**, two `uint64` keys `"hours_s"` and `"trip_s"`,
  both loaded in `begin()` (a missing key reads 0) and both written by the **same** `persist()`
  call under the **same** dirty flag, on the same triggers — a write counts as successful only if
  both keys wrote, so a shutdown cannot save one and lose the other. A new namespace, not a borrowed one: the `"speed"` namespace
  is owned by `SpeedSensor` and holds wheel calibration and distance, and running time is neither.
  Written on exactly three triggers: (a) whenever the meter has grown by
  `ENGINE_HOURS_NVS_WRITE_INTERVAL_S` (600 s = 10 min) since the last write; (b) on the
  ignition-OFF transition, on both the MAVLink and the web path; (c) on entry into fail-safe.
  `persist()` never blocks and never retries in a loop, and tolerates a failed
  `Preferences::begin()` by logging and returning. **No new config keys, no web-settable
  parameters, no runtime tuning.**
- **Reported in `EFI_STATUS`, in the field transmitted as `0.0f` today.** `spark_dwell_time` ←
  **engine hours** (float, hours). Of the four reserved fields this is the only one with **no
  standard OBD-II Mode 01 PID at all**: ignition timing is PID `0x0E`, exhaust gas temperature is
  PID `0x78`, and injection time is derivable from the fuel-trim PIDs, so all three could
  plausibly appear on this bus one day — spark dwell never can. That makes it permanently free by
  the same test the odometer's `barometric_pressure` had to pass. The value is **always valid** and
  never `NaN`: it depends on history, not on current CAN health.
- **The trip hours are reported in `injection_time`**, the second field transmitted as `0.0f`
  today. Of the fields that remain reserved after the total takes `spark_dwell_time`, injection
  time is the one with **no direct OBD-II Mode 01 PID**: it is only *derivable*, from the fuel-trim
  PIDs plus engine load, unlike ignition timing (PID `0x0E`) and exhaust gas temperature (PID
  `0x78`), which this ECU could one day answer directly. Always valid, never `NaN`, and **`0` is a
  genuine zero** after a reset — the same note `fuel_pressure` already carries.
- **The TOTAL can never be reset over any interface.** There is no command, no web control and no
  constant that clears it; the only way back to zero is an NVS erase.
- **The trip hours reset PIGGYBACKS on the existing trip-distance reset.** `EngineHourMeter::
  resetTrip()` zeroes `tripSeconds_`, persists immediately and never touches `engineSeconds_`, and
  its **one and only call site** is inside the `if (mavlink_.consumeTripResetRequest())` block in
  `VehicleController::update()`, beside `speedSensor_.resetTrip()`. **No new MAVLink command, no
  new `param1` magic, no web control, no new constant.** `MAV_CMD_USER_1` / `param1 = 1` to
  component 25 now zeroes TRIP km and trip hours in one shot; ODO and total hours are untouched.
- **Web telemetry shows both, read-only.** `engine_hours` and `engine_trip_hours` (2 decimals each)
  join the telemetry JSON unconditionally, and `data/index.html` displays them in the Vehicle Data
  (CAN) card beside engine RPM — but **outside** that card's `can_status === 'connected'` gate,
  because accumulated running time is not invalidated by the bus going quiet. No reset button for
  either.
- **Not touched:** the odometer and the trip DISTANCE code, `SpeedSensor` in any respect,
  `isEngineRunning()` and `ENGINE_RUNNING_RPM_THRESHOLD`, every other `EFI_STATUS` field, the CAN
  poll schedule, the inbound `COMMAND_LONG` handler (`handleCommandLong()` is unchanged — it still
  latches exactly one request), and the two remaining reserved `EFI_STATUS` fields.

## Impact
- Affected specs:
  - `vehicle-systems` — **ADDED** `Engine Hour Meter`.
  - `mavlink-interface` — **MODIFIED** `Vehicle State Reporting via Standard MAVLink Messages` (the
    `EFI_STATUS` field mapping gains the total and trip engine hours, and the "repurpose only
    permanently-free fields" rule releases `spark_dwell_time` and `injection_time`, narrowing the
    reserved set from four fields to two); **MODIFIED** `Inbound Command Handling and Trip Reset`
    (the accepted trip reset now zeroes the trip HOURS as well as the trip DISTANCE, still under
    one command and one magic `param1`, with both totals left unchanged).
  - `web-telemetry` — **ADDED** `Engine Hours Telemetry`.
- Affected code: `include/Constants.h`, **new** `include/EngineHourMeter.h` +
  `src/EngineHourMeter.cpp`, `include/VehicleController.h` + `src/VehicleController.cpp`,
  `include/MavlinkInterface.h` + `src/MavlinkInterface.cpp`, `src/main.cpp`,
  `include/WebPortal.h`, `src/TelemetryManager.cpp`, `src/WebPortal.cpp`, `data/index.html`,
  `MAVLINK_SETUP.md`, `WEB_PORTAL_SETUP.md`.
- `data/` changes, so this needs **both** `pio run -t upload` and `pio run -t uploadfs`.
- Out of repo: the QuadBike Mission Planner plugin gains **two read-only fields**. This is
  **non-breaking** — nothing it decodes today moves, `spark_dwell_time` and `injection_time` are
  fields it currently ignores, and an un-updated plugin keeps working and simply does not show the
  hours. **The existing trip-reset command needs no change**: the same `MAV_CMD_USER_1` /
  `param1 = 1` it already sends now clears both trip readings. The hand-off note lives in
  `MAVLINK_SETUP.md` (read-only, hours, 2 decimals, always valid/never `NaN`, `0` a genuine zero on
  the trip field, and both degrade to `0.00` against legacy firmware rather than to a wrong
  number). That repository is owned by another agent; nothing in it is touched here.
- **Field behaviour on first flash:** both keys are absent, so both meters start at 0. They
  therefore read "hours since this firmware", not "hours since the engine was built"; there is no
  back-dating mechanism and none is wanted. A vehicle upgrading from a build that had only the
  total keeps its total (`"hours_s"` is untouched) and starts the trip at zero.

## Sequencing
No sequencing constraint. The `mavlink-interface` delta **MODIFIES** two requirements — `Vehicle
State Reporting via Standard MAVLink Messages` and `Inbound Command Handling and Trip Reset` — so
the check that mattered for `add-odometer-and-trip` was run again, for both: the two active changes,
`add-vesc-esc-telemetry` and `add-extnav-velocity`, each carry an **ADDED-only** `mavlink-interface`
delta (`Steering VESC Telemetry via NAMED_VALUE_FLOAT` and `External Navigation Odometry Reporting`
respectively) and neither touches either requirement. Both MODIFIED blocks below are therefore
copied from the CURRENT `openspec/specs/mavlink-interface/spec.md` — which already carries the
odometer/trip text, the native field remap and the ECU PID additions, all three of those changes
having archived — with the engine-hours additions layered on top.

This change may archive in any order relative to the two active ones. If either of them later gains
a MODIFIED block for either of these requirements, that block must be re-based on this one, or this
one on that, before whichever archives second.
