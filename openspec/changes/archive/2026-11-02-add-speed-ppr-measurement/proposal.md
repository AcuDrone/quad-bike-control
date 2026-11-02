# Change: Measure wheel pulses per revolution from the web portal; split the speed sensor card

## Why
Pulses per revolution (`speed_cal_ppr`) is the one speed calibration that cannot be read off the
vehicle: it depends on the toothed ring and on where the sensor sits. Today the operator has to
count pulses some other way (serial log, a scope) and type the number in. The firmware already
keeps a running pulse total since boot (`SpeedSensor::getPulseTotal()`, `include/SpeedSensor.h:70`),
but nothing exports it. Exporting it lets the page measure ppr by hand: mark the tyre, turn the
wheel N times, divide.

The Speed sensor card has also grown into five unrelated settings: the sensor calibration, the
limiter floor, the speed-limit readout, the steering scaling base and the bench test speed. Adding
a measurement block to it would make it too long. The sensor calibration and the things that
*use* the speed get separate cards.

## What Changes
- **Telemetry key `speed_pulse_total`.** `SpeedSensor::getPulseTotal()` (uint32, since boot) is
  sent in every telemetry frame, via a new `VehicleController::getSpeedPulseTotal()` passthrough,
  `TelemetryManager` and `WebPortal::createTelemetryJSON()`. No command, no firmware state, no
  NVS key: the firmware stays stateless about the measurement (user decision 2026-11-02,
  variant A).
- **Measurement block `#ppr-meas` in the Speed sensor card** (all state in the page):
  - a wheel-turns number input (1–20, default 5), Start / Stop / Cancel;
  - while measuring, a live line "Pulses: N · turns: T" from the telemetry delta
    `(speed_pulse_total − start) >>> 0`, and a hint: lift the wheel, select neutral, turn it by
    hand exactly T turns in one direction, and press Stop once the count stops changing;
  - Stop freezes the result at once, with no settle delay;
  - after Stop, a result line "Result: N / T = x.x → R pulses/rev", a warning when the
    fractional part is ≥ 0.2, "Save as pulses/rev" (sends the existing `speed_cal_ppr`) and
    "Measure again";
  - zero pulses or a rounded result outside 1–1000 (`SPEED_PPR_MIN`..`SPEED_PPR_MAX`) is shown and
    cannot be saved;
  - the state is lost on page reload, and a WebSocket disconnect returns the block to idle.
    Nothing is persisted.
- **Card split** (user decision 2026-11-02, as in the approved mockup):
  - card 1 `speed-sensor-card` "🛞 Датчик швидкості / Speed Sensor": ppr input + Save, the
    measurement block, circumference input + Save, `#speed-status-msg`;
  - card 2 `speed-limit-card` "🚦 Обмеження швидкості та кермо / Speed limit and steering":
    limiter floor (a number input, not a slider, user decision 2026-11-02), the speed-limit row
    with the limiter/saturation row, steering authority, scaling base rows / reason / input, bench
    test speed, and a new `#speed-limit-status-msg`;
  - each save routes its firmware reply to the status line of the card it came from
    (`speedStatusTarget`);
  - help: card 1 ⓘ shows `speed_sensor_help` (extended with the three-sentence measuring
    procedure), card 2 ⓘ shows a new `speed_limit_help` plus the four existing hints
    (`speed_limiter_hint`, `speed_limit_floor_hint`, `steer_scale_hint`, `steer_test_speed_hint`).
- **Docs.** `WEB_PORTAL_SETUP.md` Calibration list and the limiter-floor paragraph name the two
  cards; a short "Measuring pulses per revolution" paragraph is added.
- **Version.** `FIRMWARE_VERSION` 1.5.3 → **1.5.4** (patch per `openspec/project.md`: a new
  service-console convenience, no recalibration and no wiring change), unless
  `move-can-rx-to-interrupt` (1.6.0) lands first (see task 0.2). A `Changelog.h` entry (uk/en) is
  added.
- **Branch.** Implemented on `master`, then `master` is merged into `aztec`.

## Impact
- Affected specs:
  - `web-telemetry`:
    - ADDED "Speed Sensor Pulse Total Telemetry" (`speed_pulse_total`);
    - MODIFIED "Speed-Limit and Limiter-Ceiling Telemetry" (the limit help moves to the Speed
      limit and steering card's ⓘ);
    - MODIFIED "Speed State Card" (the settings now live in two cards).
  - `web-control`:
    - ADDED "Pulses-per-Revolution Measurement via Web Interface";
    - MODIFIED "Service Console Section Layout" (Calibration lists both cards);
    - MODIFIED "Card Help Modal" (two help buttons instead of one combined one);
    - MODIFIED "Speed-Limiter Floor Configuration via Web Interface". This requirement is ADDED by
      the applied but not yet archived `add-speed-limiter-integral-floor`; the block quotes that
      pending delta in full and changes only the card name and the help location.
  - `speed-sensor`: no delta. PCNT counting, calibration storage and `speed_cal_ppr` are unchanged.
    The pulse total already exists. Its direction behaviour (every falling edge counts up, in
    either rotation direction) is stated in the new telemetry requirement.
- Affected code:
  - `include/VehicleController.h` (one inline getter);
  - `include/TelemetryManager.h`, `src/TelemetryManager.cpp`, `src/WebPortal.cpp` (one field, JSON
    capacity comment);
  - `data/index.html` (card split, measurement block, help, i18n);
  - `WEB_PORTAL_SETUP.md`, `include/Constants.h` (version), `include/Changelog.h`.
- Archive order: **archive `add-speed-limiter-integral-floor` first.** This change MODIFIES a
  requirement that only that change adds. The other MODIFIED requirements are not touched by
  any pending change (`move-can-rx-to-interrupt` and `move-aztec-vesc-to-can` do not modify them).
- Safety: none. The firmware gains one read-only telemetry value. Saving goes through the
  existing `speed_cal_ppr` command with its existing range check and privilege level. Turning the
  wheel by hand adds the distance turned to the odometer and trip (≈ 10 m for 5 turns), the same
  as pushing the vehicle.
- Operators: no recalibration, no wiring change. The settings they know are in the same section,
  split across two cards.
