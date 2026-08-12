## 1. Hardware (manual / user task)
- [ ] 1.1 **[USER/MANUAL]** Wire the 12V hall sensor to the **X2** opto-isolated input on
  `Control_v0` (6N137 → GPIO8), **not** to a Hall connector. Signal path:
  `X2.1 / X2.2 → 470Ω R22 → 6N137 LED → open-collector output + pull-up → GPIO8`
  (`GPIO_PINOUT_CUSTOM_BOARD_S3.md` §3/§8.9). Wiring depends on the sensor type:
  - **push-pull 12V:** sensor signal → `X2.1`, sensor GND → `X2.2` (series resistor of task 1.2 in
    line with the signal);
  - **open-collector NPN:** +12V through the series resistor → `X2.1`, sensor output → `X2.2` (the
    sensor sinks the LED current).
- [ ] 1.2 **[USER/MANUAL] ⚠ REQUIRED — limit the 6N137 LED current before first power-up.** At 12V
  the stock 470Ω `R22` alone gives ≈22 mA, slightly **over the 6N137's 20 mA absolute maximum**. Fit
  **560Ω–1kΩ in series in the sensor cable** (preferred; ≈7–10 mA, in the 6–15 mA sweet spot) **or**
  rework `R22` to 1kΩ on the board. Measure the LED current (or the drop across the series resistor)
  and confirm it is inside 6–15 mA, and scope/meter GPIO8 to confirm it toggles with the sensor.

## 2. Constants
- [x] 2.1 Add to `include/Constants.h`: `PIN_SPEED_SENSOR` (**GPIO 8**, 6N137 opto input on X2 —
  comment that the opto **inverts** the signal), `SPEED_SAMPLE_INTERVAL_MS`,
  `SPEED_STALE_TIMEOUT_MS`, `SPEED_GLITCH_FILTER_NS`, `SPEED_DEFAULT_PULSES_PER_REV`,
  `SPEED_DEFAULT_WHEEL_CIRCUMFERENCE_MM`, `SPEED_LIMIT_ENABLE_DEFAULT` (false),
  `SPEED_LIMIT_MAX_KMH_DEFAULT`, and `SPEED_LIMIT_THROTTLE_CAP_PCT`.

## 3. SpeedSensor subsystem (PCNT pulse counting + km/h)
- [x] 3.1 Create `include/SpeedSensor.h` + `src/SpeedSensor.cpp` with a `begin()` + `update()` class
  matching the existing subsystem pattern (no FreeRTOS task).
- [x] 3.2 In `begin()`, configure an ESP32-S3 PCNT unit (`driver/pulse_cnt.h`) on `PIN_SPEED_SENSOR`
  counting **one edge — falling or rising, either works; pick one and note in the comment that the
  6N137 inverts the signal (LED on → GPIO8 low)** — with the glitch filter set from
  `SPEED_GLITCH_FILTER_NS` and overflow watch-points accumulating into a 32-bit running pulse total.
- [x] 3.3 In `update()`, on each `SPEED_SAMPLE_INTERVAL_MS` window sample Δpulses and compute
  `speed_kmh` from `distance_per_pulse = wheel_circumference_mm / pulses_per_rev`; expose
  `getSpeedKmh()`.
- [x] 3.4 Implement staleness: if no edges arrive within `SPEED_STALE_TIMEOUT_MS`, decay reported
  speed to 0 km/h.
- [x] 3.5 Implement `isValid()` health: false until at least one plausible pulse since boot; flagged
  suspicious when pulses cease faster than a plausible decel after moving above the interlock
  threshold (mid-motion fault fingerprint).
- [x] 3.6 Load `pulses_per_rev` and `wheel_circumference_mm` from NVS namespace `"speed"` in
  `begin()` (defaults from `Constants.h`); add setters that persist to NVS immediately.
- [x] 3.7 Construct, `begin()`, and `update()` the `SpeedSensor` in `src/main.cpp` within the
  cooperative `loop()`, rate-limited like the other subsystems.

## 4. Runtime calibration + limiter web commands
- [x] 4.1 Route `speed_cal_ppr` and `speed_cal_circ` through
  `WebPortal::WebCommand` → `VehicleController::processWebCommand` (same pattern as `steer_cal_*`),
  validating the value and calling the `SpeedSensor` setters; respond `{"ok":true,...}` /
  `{"ok":false,"error":...}`. Accept regardless of active input source.
- [x] 4.2 Route `speed_limit_enable` (bool) and `speed_limit_set` (max km/h) the same way, persisting
  to NVS `"speed"`.

## 5. Control logic consumers
- [x] 5.1 Rewire the gear-change interlock (`src/TransmissionController.cpp:180-199`) to read the
  hall-sensor speed + validity instead of `vehicleData_.vehicleSpeed`; when the sensor is
  invalid/suspicious, fall back to the existing CAN-timeout fail-open behavior (log the reason).
- [x] 5.2 Add the max-speed throttle limiter in `src/VehicleController.cpp`: when
  `limit_enable` and speed is valid and `> limit_max_kmh`, clamp throttle authority to
  `SPEED_LIMIT_THROTTLE_CAP_PCT`; below the limit pass throttle through. On invalid/stale speed, do
  not clamp (fail open) and log rate-limited. Ensure it does not fight the gear-boost PID (applies to
  arbitrated driver/MAVLink/web throttle only).

## 6. Web telemetry (decouple from CAN gate)
- [x] 6.1 Add sensor speed + `speed_valid` to the `Telemetry` struct (`include/WebPortal.h`) and
  populate from the `SpeedSensor` in `src/TelemetryManager.cpp` (independent of CAN validity).
- [x] 6.2 In `src/WebPortal.cpp`, emit `vehicle_speed` and `speed_valid` **outside** the
  `if (can_status == "connected")` block (`:604-606`), and remove `vehicle_speed` from that block.
- [x] 6.3 In `data/index.html`, display speed from `vehicle_speed`/`speed_valid` regardless of
  `can_status`; add the speed calibration inputs (pulses/rev, circumference) and limiter controls
  (enable + max km/h) with i18n keys present in both `en` and `uk` dictionaries.
- [ ] 6.4 Redeploy the UI with `pio run -t uploadfs` (LittleFS) — a firmware flash alone does not
  update `data/index.html`.

## 7. MAVLink speed reporting
- [x] 7.1 In `src/MavlinkInterface.cpp` / `include/MavlinkInterface.h`, send a `VFR_HUD` message on
  the engine-telemetry schedule with `groundspeed` (m/s) from the hall sensor; send 0 (or suppress)
  when the reading is invalid. Update the stale "speed unavailable" note.

## 8. Build
- [x] 8.1 Clean firmware build passes with no errors: `~/.platformio/penv/bin/pio run`.

## 9. Bench verification
- [ ] 9.1 **[USER/MANUAL]** Spin the sensor target with a drill or hand-spin a lifted wheel a known
  number of revolutions; confirm the pulse count and that `vehicle_speed` tracks the input.
- [ ] 9.2 **[USER/MANUAL]** Calibrate: measure the tyre circumference and determine pulses/rev, set
  both via the web UI (`speed_cal_*`), and confirm the displayed km/h is plausible against a known
  reference (e.g. GPS speed at a steady pace).
- [ ] 9.3 **[USER/MANUAL]** Verify the live interlock: above ~5 km/h a non-NEUTRAL gear change is
  blocked and logged; at rest it is allowed.
- [ ] 9.4 **[USER/MANUAL]** Enable the limiter with a low test max speed and confirm throttle
  authority is reduced above it and restored below it; confirm sensor-loss fails open (no phantom
  clamp).
- [ ] 9.5 **[USER/MANUAL]** Confirm speed appears in the web UI regardless of CAN status and as
  `VFR_HUD` groundspeed in the GCS.

## 10. Validate
- [x] 10.1 `openspec validate add-hall-speed-sensor --strict` passes with no errors.
