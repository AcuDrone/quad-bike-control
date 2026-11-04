# Change: Engine RPM from the ECU tachometer output (aztec)

## Why
The aztec buggy's Delphi MT05 ECU never reports engine RPM over CAN (PID 0x0C always 0, no broadcast
frame carries it — verified by probe and sniffer). Without RPM the MAVLink gear gate never opens, the
starter cut-off only ends on `CRANKING_TIMEOUT`, the crank interlock cannot see a running engine, and
the gear-change boost PID, hour meter, telemetry and `EFI_STATUS.rpm` are dead. The ECU drives a
tachometer pulse output (grey connector J1 pin 6, ≈12 V square wave, ≈50 % duty) that the original
dash reads; this change reads it.

## What Changes
- Hardware (no board change): TACHO → 1.5 kΩ → PC817 LED → GND; PC817 emitter follower — collector
  to X3 pin 1 (5 V), emitter to X3 pin 2 (GPIO14, Hall_1 A), no pull-up; signal not inverted. Crank
  sensor not tapped.
- New `TachoSensor` module (`include/TachoSensor.h`, `src/TachoSensor.cpp`): GPIO14 (`PIN_TACHO`)
  edge ISR with `micros()` timestamps, software glitch filter, RPM from the mean of the last 4
  periods, 500 ms timeout → 0 RPM, ppr in NVS `"tacho"`/`"ppr"` (allowed {0.5, 1, 2, 3, 4},
  default 1), "fitted" = pulse seen since boot OR ppr stored in NVS.
- New pure header `include/TachoLaw.h` (period → RPM, plausibility, timeout, ppr validation and
  snapping, calibration, RPM source selection) + native suite `test/test_tacho` (≥ 10 cases).
- `VehicleController` composes an EFFECTIVE engine RPM with `rpm_source` TACHO / CAN / NONE; all
  RPM consumers (`isEngineRunning()`, relay starter cut-off and crank interlock, web START interlock,
  boost PID, hour meter, telemetry, MAVLink) switch to it. `CANController::VehicleData.engineRPM`
  stays the raw CAN value and is the fallback.
- `ENGINE_RUNNING_RPM_THRESHOLD` 1500 → 800 (idle is ≈1500 RPM).
- Web: new "Оберти (TACHO)" / "Tacho (RPM) calibration" card (live Hz / ppr / RPM / source,
  `cal_tacho_ppr` from a reference RPM with snapping and refusals, `set_tacho_ppr`,
  `reset_tacho_ppr`); the existing RPM tile shows the effective RPM outside the CAN gate with a
  "TACHO"/"CAN" badge.
- Telemetry JSON: `tacho_hz`, `tacho_ppr`, `tacho_rpm`, `rpm_source`; `engine_rpm` = effective RPM,
  emitted outside the CAN block.
- MAVLink: `EFI_STATUS.rpm` = effective RPM (NaN only with no source); no new `NAMED_VALUE_FLOAT`.
- Docs (pinout, web portal, MAVLink), `Changelog.h` 1.5.9, `FIRMWARE_VERSION` "1.5.9".
- aztec branch only; no build macro.

## Impact
- Affected specs: tacho-sensor (new), vehicle-systems, can-controller, web-telemetry, web-control,
  mavlink-interface
- Affected code: `include/TachoLaw.h` (new), `include/TachoSensor.h` (new), `src/TachoSensor.cpp`
  (new), `include/Constants.h`, `include/VehicleController.h`, `src/VehicleController.cpp`,
  `src/main.cpp`, `src/TelemetryManager.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`,
  `src/MavlinkInterface.cpp`, `data/index.html`, `include/Changelog.h`, `platformio.ini`,
  `test/test_tacho/`
- Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md`, `WEB_PORTAL_SETUP.md`, `MAVLINK_SETUP.md`
- Hardware: PC817 + 1.5 kΩ on X3, emitter follower (see design.md; GPIO14 high level bench-checked)
