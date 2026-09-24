## 1. Constants (`include/Constants.h`)

- [x] 1.1 Extend the "Autopilot parameter subscription" block (~line 256) with
  `MAVLINK_PARAM_SPD_SCA_BASE_ID` (`"MOT_SPD_SCA_BASE"`) and `MAVLINK_PARAM_SPD_SCA_BASE_MAX`
  (10.0f — ArduPilot's own range bound for that parameter), and rewrite the block comment from
  "exactly one parameter is subscribed" to "two parameters are subscribed, both read-only".
- [x] 1.2 Add a `STEER_SCALE_*` block next to the `SPEED_LIMIT_*` block, in the file's existing
  comment style (one line of rationale per constant):
  `STEER_SCALE_SLEW_PER_S` (2.0f — scale units/s, both directions; full 0 → 1 in 0.5 s, fast
  enough not to feel laggy, slow enough not to snap the wheel),
  `STEER_SCALE_BASE_MAX_MS` (10.0f — matches `MOT_SPD_SCA_BASE`'s range so a local value and an
  autopilot value are interchangeable),
  `STEER_SCALE_LOG_MIN_MS` (1000), `STEER_SCALE_LOG_EPSILON` (0.02f),
  `STEER_SCALE_WARN_MS` (10000 — hold-off for "no base" / "speed invalid" warnings, modelled on
  `SPEED_LIMIT_WARN_MS`),
  `STEER_SCALE_TEST_SPEED_MS` (60000 — bench override fuse),
  `STEER_SCALE_TEST_SPEED_MAX_MS` (30.0f).
- [x] 1.3 Add the NVS key name `steer_sca_base` as a named constant, and note in the comment that
  it lives in the existing `"steering"` namespace alongside the steering calibration.

## 2. MavlinkInterface: heartbeat mode, second subscribed parameter, `STEER_SCA`

- [x] 2.1 `include/MavlinkInterface.h`: add `hasAutopilotMode()` and `getAutopilotCustomMode()`
  (and `getAutopilotBaseMode()`), plus `hasSpdScaBaseParam()`, `getSpdScaBaseMs()` and
  `getSpdScaBaseAgeMs()`. The header stays free of mavlink headers.
- [x] 2.2 `handleHeartbeat(uint8_t sysid, uint8_t compid)` (`src/MavlinkInterface.cpp:230`) gains
  `uint8_t baseMode, uint32_t customMode`; the `case MAVLINK_MSG_ID_HEARTBEAT` at
  `src/MavlinkInterface.cpp:113` decodes the message and passes them (the `handleServoOutputRaw`
  decoded-scalars pattern).
- [x] 2.3 Store `baseMode_` / `customMode_` / `modeRxMs_` only when the sender matches the learned
  autopilot; clear them in `begin()`. `hasAutopilotMode()` = received AND link up.
- [x] 2.4 Generalise the parameter subscription into a fixed two-entry table (id string, upper
  bound, stored value, receipt time, poll timestamp). `requestSpeedMaxParam()` becomes a
  parameterised `requestParam(index)`; `handleParamValue()` matches the incoming `param_id`
  against the table, keeping the 17-byte copy, the sysid/compid filter and the
  NaN/inf/negative/out-of-range rejection verbatim.
- [x] 2.5 Reimplement `hasSpeedMaxParam()` / `getSpeedMaxMs()` / `getSpeedMaxAgeMs()` on the table
  with **identical** semantics, and add the three `SpdScaBase` accessors with the same rules
  (received AND > 0 AND link up AND age < `MAVLINK_PARAM_STALE_MS`).
- [x] 2.6 Stagger the two polls so the requests are not emitted in the same loop iteration.
- [x] 2.7 Extend the 1 Hz `[MAV] viso:` debug line with `scabase` / `age` / `valid` and the
  autopilot's `custom_mode`.
- [x] 2.8 Add a `STEER_SCA` `NAMED_VALUE_FLOAT` to the `MAVLINK_STEER_SLOW_TX_MS` tick, next to
  `VESC_TEMP` and `VESC_OK`, sourced from a new `StateReport` field carrying the applied scale.
  Never `NaN`; `1.0` when not scaling.

## 3. VehicleController: base value, scale computation, wiring

- [x] 3.1 Load `steer_sca_base` from NVS namespace `"steering"` in `begin()` using the existing
  `prefs.begin("boost", ...)` pattern at `src/VehicleController.cpp:119`; reject NaN, negative and
  `> STEER_SCALE_BASE_MAX_MS`, treating them and an absent key as "not set" (0).
- [x] 3.2 Add `processSetSteerScaBaseCommand(float, WebPortal&)` modelled on
  `processSpeedCalPprCommand` / `processSpeedCalCircCommand` (`src/VehicleController.cpp:648-665`):
  validate, apply, persist, respond. Dispatch `set_steer_sca_base` from `processWebCommand`
  (`src/VehicleController.cpp:255`) in the "works regardless of input source" group.
- [x] 3.3 Add `processSetTestSpeedCommand(float, WebPortal&)`: RAM-only override plus its set time,
  validated against `STEER_SCALE_TEST_SPEED_MAX_MS`, zero clearing it; dispatch `set_test_speed`
  in the same group. Expire it after `STEER_SCALE_TEST_SPEED_MS` in `update()`.
- [x] 3.4 Add `getSteerScaleBaseMs()` returning the stored base when positive, else
  `mavlink_.getSpdScaBaseMs()` when `hasSpdScaBaseParam()`, else 0 — plus a source enum for
  telemetry and logging.
- [x] 3.5 Add `applySteeringScale(float steeringPct)`:
  base <= 0 → scale 1 (+ `STEER_SCALE_WARN_MS` warning);
  `!speedSensor_.isValid()` → scale 1 (+ `STEER_SCALE_WARN_MS` warning);
  autopilot mode known and not Rover MANUAL (`custom_mode != 0`) → scale 1;
  otherwise `v` = test override if live else `speedSensor_.getSpeedMs()`, and
  `scale = (v > base) ? base / v : 1`.
- [x] 3.6 Slew-limit the applied scale to `STEER_SCALE_SLEW_PER_S` in both directions, `dt` capped
  at 1 s, first evaluation after becoming active adopting the target directly — the shape used by
  `applySpeedLimit()` (`src/VehicleController.cpp:588`, ~638).
- [x] 3.7 Add `logSteerScaleChange()` modelled on `logSpeedLimitChange()`
  (`src/VehicleController.cpp:564`): epsilon `STEER_SCALE_LOG_EPSILON`, hold-off
  `STEER_SCALE_LOG_MIN_MS`, the remembered value NOT updated while suppressed; the line carries
  the scale, the speed (marked `TEST` when overridden), the base and its source.
- [x] 3.8 Wire it in `processMavlinkCommands()` (`src/VehicleController.cpp:421-422`):
  `steering_.setSteeringPercent(applySteeringScale(mavlink_.getSteering()))`. Leave the web
  `set_steering` path (`src/VehicleController.cpp:332`) unscaled.
- [x] 3.9 Publish the applied scale into the MAVLink `StateReport` so `STEER_SCA` has a source,
  and expose `getSteerScale()` / base / source / test-override accessors for telemetry.

## 4. Web telemetry and portal

- [x] 4.1 `include/WebPortal.h`: add `steerScale`, `steerScaBaseMs`, `steerScaSrc` and
  `steerTestSpeedMs` to the telemetry struct.
- [x] 4.2 `src/TelemetryManager.cpp`: populate them from the new `VehicleController` accessors.
- [x] 4.3 `src/WebPortal.cpp`: serialise `steer_scale` (2 decimals), `steer_sca_base` (m/s, 2
  decimals), `steer_sca_src` and `steer_test_speed`; re-check the JSON peak size against the 4096
  guard.
- [x] 4.4 `data/index.html`: a steering-scaling row showing the scale, the base and its source, and
  the inactive reason; a base input with a Save button sending `set_steer_sca_base`; a test-speed
  input sending `set_test_speed` with a visible "temporary, expires by itself" marker.
- [x] 4.5 `data/index.html`: add every new string to BOTH the `en` and `uk` dictionaries.

## 5. Documentation (`MAVLINK_SETUP.md`)

- [x] 5.1 Add `MANUAL_OPTIONS` = `0` to the "ArduPilot parameters" table (~line 48) as a
  **required** step, stating that steering speed scaling now happens on the ESP32 and that leaving
  bit 0 set scales the steering twice.
- [x] 5.2 In the same section, explain why: `get_forward_speed()` falls back to raw GPS ground
  speed when EKF3 has no velocity, which on this vehicle is exactly when the ESP32 has gone silent;
  record the 2026-09-24 observation (parked, GPS 56 m/s, scale 0.018). Keep `VISO_TYPE = 1`
  unchanged and say so explicitly, so nobody "cleans up" the odometry path along with this.
- [x] 5.3 Behaviour note: clearing `MANUAL_OPTIONS` also removes ArduPilot's ground-speed-based
  steering reversal in reverse; the reversal then keys off throttle sign, which this vehicle's
  mechanical gearbox never drives negative, so the change is inert here.
- [x] 5.4 Add `STEER_SCA` to the `NAMED_VALUE_FLOAT` section (~line 353), to the message table row
  at ~line 98 and to the value-to-name summary at ~line 111: six names now, `STEER_SCA` on the
  1 Hz slow timer, never `NaN`, `1.0` = not scaling.
- [x] 5.5 Update the unarchived `add-vesc-esc-telemetry` delta (or, if it has already been
  archived, `openspec/specs/mavlink-interface/spec.md`) so its "exactly five `NAMED_VALUE_FLOAT`"
  wording reads six and names `STEER_SCA` under the same scoped exception.

## 6. Build and bench verification

There are no host tests in this repository (`test/` holds only a README and `platformio.ini` has
no `native` environment), so verification is build plus bench.

- [x] 6.1 `/Users/s.zalozniy/.platformio/penv/bin/pio run -e esp32-s3-devkitc-1` completes with no
  errors and no warnings from `src/` or `include/`.
- [ ] 6.2 Flash firmware **and** `pio run -t uploadfs` (`data/` changed).
- [ ] 6.3 Boot with MAVLINK debug on: the 1 Hz line shows `scabase` with an `age` sawtooth
  0 → 5000 and the autopilot's `custom_mode`; `SPEED_MAX` behaviour on that line is unchanged from
  before this change.
- [ ] 6.4 Stationary, **disarmed** bench, `steer_sca_base = 1.0`: `set_test_speed 5.56` and a 100 %
  steering command from the autopilot gives roughly 18 % of travel; `STEER_SCA` reads ~0.18.
- [ ] 6.5 `set_test_speed 0` → full travel returns, `STEER_SCA` reads 1.00, and the transition is
  paced by `STEER_SCALE_SLEW_PER_S` rather than snapping.
- [ ] 6.6 Leave the override alone for 60 s → it clears itself, the portal stops showing it and the
  log drops the `TEST` marker.
- [ ] 6.7 Put the autopilot in a mode other than MANUAL with `set_test_speed 5.56` still live →
  full travel, `STEER_SCA` 1.00.
- [ ] 6.8 Force the speed sensor invalid (pull the hall lead while the suspicious latch can set, or
  boot with it disconnected) with `set_test_speed 5.56` live → full travel, `STEER_SCA` 1.00, and
  the `STEER_SCALE_WARN_MS` warning appears at most once per hold-off.
- [ ] 6.9 `set_steer_sca_base 0` with no `MOT_SPD_SCA_BASE` on the autopilot → full travel and the
  "no base" warning; set `MOT_SPD_SCA_BASE = 1` on the autopilot → within ~5 s the portal shows the
  base as autopilot-sourced and scaling resumes.
- [ ] 6.10 Web `set_steering` at 100 % with `set_test_speed 5.56` live → full travel, confirming
  the web path is not scaled.
- [ ] 6.11 Power-cycle: `steer_sca_base` survives, the test override does not.
- [ ] 6.12 `[WEB] telemetry JSON peak:` stays below 4096 with no overflow warning.
- [ ] 6.13 Field checklist for the operator (describe in the docs, do not execute here):
  confirm `MANUAL_OPTIONS = 0` on the autopilot; accelerate in a straight line to 20 km/h; apply
  full lock; confirm the travel matches `base / v` and that the reduction is felt as a gradual
  weighting rather than a step.
