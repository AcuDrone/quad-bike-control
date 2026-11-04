## 1. Pure math (host-testable)
- [x] 1.1 `include/TachoLaw.h` (no Arduino deps): `tachoHzFromPeriodUs`, `tachoRpm(hz, ppr)`, `tachoMeanPeriod` (up to N periods), `tachoPlausible` (≤ `TACHO_MAX_RPM`), `tachoTimedOut` (wrap-safe), `tachoPprAllowed`, `tachoSnapPpr` (nearest of {0.5,1,2,3,4}, ±25 % window), `tachoComputeCal` (+ `TachoCalResult` OK / NO_PULSES / REF_OUT_OF_RANGE / RATIO_OFF), `RpmSource` + `tachoSelectRpm(fitted, live, tachoRpm, tachoMs, canValid, canRpm, canMs)`
- [x] 1.2 `test/test_tacho/test_main.cpp` (≥ 10 cases: period→RPM at ppr 0.5/1/2, mean of 4, timeout + `millis()` wrap, implausible RPM, allowed set, snapping, cal OK incl. the 26.1 Hz / 1550 example, each refusal, the four source-selection branches)
- [x] 1.3 Register `test_tacho` in `platformio.ini` `[env:native]` `test_filter` as ONE indented line

## 2. Constants
- [x] 2.1 `PIN_TACHO` GPIO_NUM_14 with the wiring / inversion comment; update the "GPIO14 spare" header comment
- [x] 2.2 `TACHO_TIMEOUT_MS` 500, `TACHO_AVG_PERIODS` 4, `TACHO_MIN_PERIOD_US` 1000, `TACHO_MAX_RPM` 10000, `TACHO_PPR_DEFAULT` 1.0, `TACHO_CAL_REF_MIN_RPM` 300 / `MAX` 8000, `TACHO_CAL_TOLERANCE` 0.25, `TACHO_NVS_NS` "tacho", `TACHO_NVS_KEY` "ppr"
- [x] 2.3 `ENGINE_RUNNING_RPM_THRESHOLD` 1500 → 800; rewrite the "1500 sits above idle" comment in the engine-hours block and the `isEngineRunning()` doc comment

## 3. Firmware
- [x] 3.1 `include/TachoSensor.h` / `src/TachoSensor.cpp` (pattern of `SpeedSensor`): `begin()` (pin input, IRAM ISR on RISING, NVS load with validation), `update()` (critical-section copy of the ring buffer, mean period, plausibility, timeout + buffer clear, fitted latch), accessors `getHz()`, `getRpm()`, `getPpr()`, `isLive()`, `isFitted()`, `lastEdgeMs()`, `setPpr()` (persist-then-apply), `resetPpr()` (remove key)
- [x] 3.2 `VehicleController`: own `TachoSensor`, call `update()` before the RPM consumers; compute `EffectiveRpm` once per `update()` via `tachoSelectRpm`; accessors `getEngineRpm()`, `isEngineRpmValid()`, `getRpmSource()`, `getTacho()`
- [x] 3.3 Switch consumers to the effective RPM: `isEngineRunning()`, `relayController_.update()`, web START interlock, boost PID (activation, stale freeze, new-sample check via `sampleMs`, error, log), hour meter
- [x] 3.4 Commands `cal_tacho_ppr`, `set_tacho_ppr`, `reset_tacho_ppr` in `processWebCommand` (accepted for any input source; replies start with "Tacho:"), following `processBatteryCalCommand`; WebPortal command parsing if a whitelist needs the new names
- [x] 3.5 `main.cpp`: `tacho.begin()` in setup (or via VehicleController init); MAVLink report `engineRpm` / new `rpmValid` from the effective RPM
- [x] 3.6 `TelemetryManager` / `WebPortal`: `engine_rpm` = effective RPM moved outside the CAN block; add `tacho_hz`, `tacho_ppr`, `tacho_rpm`, `rpm_source`
- [x] 3.7 `MavlinkInterface.cpp`: `EFI_STATUS.rpm` gated on `rpmValid` instead of `canValid`; update the field-map comment

## 4. Web UI (`data/index.html`)
- [x] 4.1 Existing RPM tile: value from `engine_rpm` regardless of `can_status`, "TACHO"/"CAN" badge, "—" for NONE
- [x] 4.2 New card "Оберти (TACHO)" / "Tacho (RPM) calibration" in Calibration after "Speed limit and steering": live Hz / ppr / RPM / source, "no pulses" state, hint (dash or crank Hz × 60 / 23)
- [x] 4.3 "Еталонні оберти" input + "Калібрувати" (`cal_tacho_ppr`), ppr select + apply (`set_tacho_ppr`), "Скинути" (`reset_tacho_ppr`); route replies starting "Tacho" to the card status line; i18n uk/en

## 5. Docs, version, changelog
- [x] 5.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: GPIO14 row and X3 row → tacho; PC817 wiring note (§ new "X3 tacho input (aztec)"); warnings § (emitter-follower wiring, no pull-up, X3.4 unused, crank sensor not tapped, twisted pair); firmware migration summary
- [x] 5.2 `WEB_PORTAL_SETUP.md`: tacho card, `cal_tacho_ppr` / `set_tacho_ppr` / `reset_tacho_ppr`, new telemetry fields
- [x] 5.3 `MAVLINK_SETUP.md`: `EFI_STATUS.rpm` source (tacho, CAN fallback, NaN only with no source)
- [x] 5.4 `FIRMWARE_VERSION` "1.5.8" → "1.5.9"; `include/Changelog.h` 1.5.9 entry (uk + en)

## 6. Verification
- [x] 6.1 `pio test -e native` — all suites pass, including `test_tacho`
- [x] 6.2 `pio run -e esp32-s3-devkitc-1` builds
- [x] 6.3 `openspec validate add-tacho-rpm-input --strict`

## 7. Bench
- [ ] 7.1 Before connecting to the ECU: drive the PC817 LED from 12 V via 1.5 kΩ and measure GPIO14 with the LED on and off (expected ≈3.1 V high — must be ≥ 2.5 V before the counts are trusted; ≈0 V low, ≤ 0.8 V)
- [ ] 7.2 Wire the TACHO input; at idle the card shows ≈25–26 Hz and `rpm_source` TACHO
- [ ] 7.3 Calibrate against the dash, or against the crank sensor (RPM = crank Hz × 60 / 23); the reply snaps to ppr 1 (expected); value survives a reboot
- [ ] 7.4 Gear shift from the TX16S over MAVLink is accepted with the engine idling
- [ ] 7.5 Cranking RPM read on the card stays below 800; the starter is released as soon as the engine fires; START is refused with the engine running
- [ ] 7.6 After engine stop the RPM reads 0 within 0.5 s and `isEngineRunning()` drops (gear gate closes)
- [ ] 7.7 Mission Planner shows `EFI_STATUS.rpm` matching the card
