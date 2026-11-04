## 1. Pure math (host-testable)
- [x] 1.1 `include/BatteryVoltage.h`: `BatParams`, `batRawVolts`, `batVolts`, `batPresent`, `batCalValid`, `batLowStep` (hysteresis), `batComputeCal` (+ `BatCalResult`)
- [x] 1.2 `test/test_battery_voltage/test_main.cpp` (≥ 8 cases: scaling, present, hysteresis set/hold/clear/sequence, cal OK, measured out of range, not connected, factor out of range, valid range)
- [x] 1.3 Register `test_battery_voltage` in `[env:native]` `test_filter` (one indented line)

## 2. Constants
- [x] 2.1 Replace the 24 V boost rail block with `BAT12_*` constants (interval, count, divider, present, low/clear, crank hold-off, cal range, measured range, NVS ns/key); remove `RAIL_24V_LOW_THRESHOLD`, `BOOST_STARTUP_GRACE_MS`, `ECU_VOLTAGE_LOW_V`
- [x] 2.2 Update the `PIN_BOOST_EN` / `PIN_ADC_24V` comments

## 3. Firmware
- [x] 3.1 `RelayController::isStarterEnergized()`
- [x] 3.2 `VehicleController`: `initBatterySense()` (ADC + NVS cal load), `updateBatteryMonitor()` (sampling, valid, hysteresis, crank suppression, rate-limited log), accessors; remove rail members / grace / `[RAIL]` log
- [x] 3.3 Commands `cal_bat_v`, `reset_bat_v_cal`; `set_boost` → "boost rail not fitted"
- [x] 3.4 `main.cpp`: keep GPIO46 HIGH, call `initBatterySense()`, feed the MAVLink report
- [x] 3.5 CAN: drop PID 0x42 from the poll table (PID_COUNT 6, stagger), remove `moduleVoltageMv` and its parse/log; keep the probe
- [x] 3.6 Telemetry/WebPortal: `ecu_voltage`, `ecu_voltage_low`, `bat_v_raw`, `bat_v_cal`, `bat_v_valid` outside the CAN block; remove `rail_24v`, `boost_on`, `rail_low`
- [x] 3.7 MAVLink: `ignition_voltage` = battery V when valid else NaN; VESC_V comments

## 4. Web UI (`data/index.html`)
- [x] 4.1 Battery tile in the ignition card (label i18n uk/en, calibrated V, "raw × cal" line, "not connected")
- [x] 4.2 Calibration input (step 0.01) + "Калібрувати" button (`cal_bat_v`) + "Скинути" link (`reset_bat_v_cal`), reply shown next to it
- [x] 4.3 Attention line gated on `bat_v_valid`, not CAN
- [x] 4.4 Remove the boost button, rail tile, `toggleBoostRail`, the CAN-card ECU voltage tile and the obsolete i18n strings; update the help text

## 5. Docs, version, changelog
- [x] 5.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md` (X10 row, ADC table, §7 "X10 battery sense (aztec)", pin tables)
- [x] 5.2 `WEB_PORTAL_SETUP.md` (battery tile, `cal_bat_v` / `reset_bat_v_cal`)
- [x] 5.3 `FIRMWARE_VERSION` "1.5.7" → "1.5.8"; `include/Changelog.h` 1.5.8 entry (uk + en)
- [ ] 5.4 `MAVLINK_SETUP.md`: stale `ignition_voltage` / VESC_V lines — left to the user (file has uncommitted user edits)

## 6. Verification
- [x] 6.1 `pio test -e native` all suites pass
- [x] 6.2 `pio run -e esp32-s3-devkitc-1` builds
- [ ] 6.3 Bench: battery on X10 reads within 0.1 V of a multimeter after calibration; "not connected" with X10 open; no low warning while cranking; `ignition_voltage` in Mission Planner
