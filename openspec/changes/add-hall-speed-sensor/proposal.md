# Change: Add vehicle speed reading from a 3-pin hall-effect speed sensor

## Why
The vehicle has no live speed source: `CANController::VehicleData.vehicleSpeed`
(`include/CANController.h:22`) is always 0 because the OBD-II speed PID (0x0D) is not read, and
the ECU is not trusted to provide it. As a result the speed-based gear-change interlock
(`src/TransmissionController.cpp:184`) never fires, the web/MAVLink speed readouts are dead, and
there is no over-speed protection. A physical 3-pin **12V** hall-effect speed sensor is being fitted
to the driveline; this change reads it directly and makes speed a real, independent signal that
drives telemetry and control logic.

## What Changes
- Add a new **`SpeedSensor`** subsystem (class with `begin()` + `update()`, polled from the single
  cooperative `loop()` like every other subsystem) that counts hall pulses on one GPIO using the
  ESP32-S3 **PCNT** hardware pulse counter (hardware glitch filter, no ISR), and derives km/h from
  runtime calibration. This is the project's first pulse-input peripheral (greenfield pattern —
  see design.md for the PCNT-vs-interrupt decision).
- Define the **hardware interface requirement**: the sensor signal is **12V** and reaches the
  ESP32-S3 through the board's **6N137 opto-isolated input on connector X2** — signal path
  `X2.1/X2.2 → 470Ω R22 → 6N137 LED → open-collector output + pull-up → GPIO8`
  (`GPIO_PINOUT_CUSTOM_BOARD_S3.md` §2/§3/§8.9). Input pin: **GPIO 8** (`PIN_SPEED_SENSOR`). The
  opto input is current-driven and galvanically isolated, so it takes 12V cleanly with **one
  external series resistor** (see the wiring requirement below); the Hall channels (X3/X4/X5) are
  scaled for **5V** sensors and stay spare. The opto **inverts** the signal (LED conducting → GPIO8
  low), which is irrelevant for pulse counting but is recorded in the PCNT configuration. There is
  no fallback pin: GPIO1/2 (the pins this change originally proposed) are now I2C1 (AS5600 + the
  opto-input expander) on `Control_v0`.
- Make calibration **runtime-configurable** because pulses-per-revolution and wheel circumference
  are unknown: store `pulses_per_rev` and `wheel_circumference_mm` in a new NVS namespace `"speed"`
  (Preferences), with defaults in `include/Constants.h`. Set them via the existing web-command path
  (`WebPortal::WebCommand` → `VehicleController::processWebCommand`, same pattern as `steer_cal_*`)
  using new commands `speed_cal_ppr` and `speed_cal_circ`.
- Feed speed into **three consumers**:
  1. **Web telemetry** — emit `vehicle_speed` (km/h) **decoupled from the CAN "connected" gate**
     (today speed is only emitted inside `if (can_status == "connected")` at
     `src/WebPortal.cpp:604-606`), plus a `speed_valid` health flag, so it displays whenever the
     sensor is live regardless of CAN state.
  2. **MAVLink telemetry** — report ground speed via a standard `VFR_HUD` message from
     `MavlinkInterface` (which today sends only `EFI_STATUS` and explicitly omits speed —
     `include/MavlinkInterface.h:55`, `src/MavlinkInterface.cpp:262`).
  3. **Control logic** — make the existing gear-change interlock **live** by sourcing its speed
     from the hall sensor instead of the dead CAN field, and add a **configurable maximum-speed
     throttle limiter** (disabled by default) that reduces throttle authority above a settable
     speed, set via `speed_limit_set` / `speed_limit_enable` web commands.
- **BREAKING (telemetry contract):** `vehicle_speed` in the WebSocket JSON is no longer gated by
  `can_status == "connected"` and is now hall-sensor-sourced, not CAN-sourced; it is removed from
  the CAN-connected payload block and emitted independently.

## Impact
- Affected specs:
  - `speed-sensor` (NEW capability — ADDED: hardware interface, PCNT pulse counting, runtime
    calibration, signal validity/timeout, speed telemetry, maximum-speed throttle limiter).
  - `can-controller` (MODIFIED: "Speed-Based Gear Change Prevention" now sourced from the hall
    sensor; "CAN Data Telemetry Broadcasting" drops `vehicleSpeed` from the CAN-only JSON block).
  - `web-telemetry` (MODIFIED: "Telemetry Display on Web Interface" — speed display decoupled from
    the CAN gate and sourced from the sensor).
  - `mavlink-interface` (MODIFIED: "Vehicle State Reporting via Standard MAVLink Messages" — add
    `VFR_HUD` ground-speed reporting from the hall sensor).
- Affected code: new `include/SpeedSensor.h` + `src/SpeedSensor.cpp`; `src/main.cpp`
  (construct/`begin()`/`update()` the sensor in the cooperative loop); `include/Constants.h`
  (`PIN_SPEED_SENSOR`, calibration + limiter defaults, sample/stale timing); `src/VehicleController.cpp`
  (`speed_cal_*` / `speed_limit_*` dispatch, speed limiter application, feed sensor speed to the
  transmission interlock); `src/TransmissionController.cpp` (interlock reads sensor speed);
  `include/WebPortal.h` + `src/WebPortal.cpp` (`vehicle_speed` / `speed_valid` emission decoupled
  from CAN gate); `src/TelemetryManager.cpp` (populate speed from the sensor); `src/MavlinkInterface.cpp`
  + `include/MavlinkInterface.h` (`VFR_HUD` speed report); `data/index.html` (speed calibration +
  limiter controls; speed readout independent of CAN status).
- Deployment note: updating `data/index.html` requires `pio run -t uploadfs` (LittleFS upload), not
  just a firmware flash.
- Hardware note (**REQUIRED wiring change**): at 12V the stock 470Ω `R22` alone drives ≈22 mA
  through the 6N137 LED — slightly over its **20 mA absolute maximum**. An extra **560Ω–1kΩ in
  series in the sensor cable** (preferred; ≈7–10 mA, in the 6–15 mA sweet spot) or a rework of `R22`
  to 1kΩ is mandatory before first power-up with the sensor connected. Wiring by sensor type:
  push-pull 12V → signal to `X2.1`, sensor GND to `X2.2` (series R in line); open-collector NPN →
  +12V through the series R to `X2.1`, sensor output to `X2.2` (the sensor sinks the LED current).
  Called out as a manual user task in `tasks.md` (`GPIO_PINOUT_CUSTOM_BOARD_S3.md` §3/§8.9).
- Cross-change note: the pin and hardware assumptions above were corrected by
  `migrate-to-control-v0-board`, which re-pins the whole firmware onto `Control_v0`. Whichever
  change lands second must be re-read against the other. GPIO8 was previously earmarked for an
  S-BUS receiver input; S-BUS is dropped for good (no UART was available for it), so the opto input
  is free for this sensor.
