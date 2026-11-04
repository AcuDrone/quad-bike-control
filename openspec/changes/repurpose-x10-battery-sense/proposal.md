# Change: Repurpose X10 as the 12 V battery sense (aztec)

## Why
The aztec buggy has no Buck-Boost board, so X10 and its 200K/24K divider to GPIO9 are idle, and the
Delphi MT05 ECU never answers OBD PID 0x42, so the "ECU/module voltage" telemetry is always empty.
Wiring the 12 V battery (+) to X10 pin 3 gives a real battery measurement on the same divider.

## What Changes
- New pure header `include/BatteryVoltage.h` (scaling, user calibration factor, low flag with
  11.8 / 12.1 V hysteresis, calibration computation and validation) + native suite
  `test/test_battery_voltage`.
- `VehicleController`: the 24 V rail sampler becomes a 12 V battery sampler (500 ms, 8 samples,
  ADC_11db); low warning suppressed while the starter relay is on and 2 s after (new
  `RelayController::isStarterEnergized()`); GPIO46 still driven HIGH.
- Web commands `cal_bat_v` (multimeter reading → factor, validated, NVS `bat12`/`cal`) and
  `reset_bat_v_cal`; boot load with validation. `set_boost` still recognised, answers "boost rail
  not fitted".
- **BREAKING (telemetry)**: `rail_24v`, `boost_on`, `rail_low` removed; `ecu_voltage` /
  `ecu_voltage_low` now come from the battery sense and are emitted outside the CAN block; new
  `bat_v_raw`, `bat_v_cal`, `bat_v_valid`.
- CAN: PID 0x42 removed from the poll table (7 → 6 PIDs), kept in the probe;
  `VehicleData::moduleVoltageMv` removed.
- MAVLink: `EFI_STATUS.ignition_voltage` = calibrated battery V when valid, else NaN (no longer
  gated on CAN). VESC_V comments say "24 V VESC supply (7s LiFePO4 on aztec)".
- Web UI: battery tile "АКБ 12 В" / "12 V battery" with "raw × cal" line and calibration input; the
  boost button, rail tile and their i18n strings removed; attention line not gated on CAN.
- Docs: GPIO pinout (X10, ADC table, §7), WEB_PORTAL_SETUP, Changelog 1.5.8; firmware 1.5.8.
- aztec branch only; no build macro.

## Impact
- Affected specs: vehicle-systems, web-telemetry, web-control, can-controller, mavlink-interface,
  vehicle-actuators
- Affected code: `include/BatteryVoltage.h` (new), `include/Constants.h`, `include/RelayController.h`,
  `include/VehicleController.h`, `src/VehicleController.cpp`, `src/main.cpp`,
  `include/CANController.h`, `src/CANController.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`,
  `src/TelemetryManager.cpp`, `include/MavlinkInterface.h`, `src/MavlinkInterface.cpp`,
  `data/index.html`, `include/Changelog.h`, `platformio.ini`, `test/test_battery_voltage/`
- Hardware: battery (+) to X10 pin 3, battery (−) to X10 pin 2 (GND) — no board change.
