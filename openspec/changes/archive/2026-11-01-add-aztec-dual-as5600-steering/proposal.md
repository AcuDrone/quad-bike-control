# Change: Dual-AS5600 multi-turn steering feedback for the aztec buggy

## Why
The `aztec` branch targets a new vehicle, a buggy, built on the same `Control_v0` board and the same
Flipsky VESC steering drive. Its steering shaft turns ±1.7 turns from centre (3.4 turns
lock-to-lock). A single AS5600 on that shaft only reports an angle within one turn, so it can no
longer give an absolute steering position. Today's firmware assumes less than 360° lock-to-lock.

## What Changes
- **Two AS5600 sensors, both at address `0x36`, on different buses:**
  - **Fine sensor**: on the steering shaft. It sits on I2C1 (`Wire`, GPIO1/2, header X14), where the
    single sensor is today.
  - **Coarse sensor**: on a geared-down shaft that sweeps less than 360° from lock to lock. It sits
    on I2C2 (`Wire1`, GPIO48/47, header X15). That bus is shared with the PCA9557 opto-input `0x1A`,
    the mux `0x1C` and the relay board `0x1F`. `0x36` collides with none of them.
- **Working position**: the firmware tracks how far the fine sensor has turned, across turn
  boundaries ("unwrapped"), and measures the position in fine-sensor counts from the calibrated
  centre (int32, about ±6963 counts). These are the same units as today's counts, so the percent
  API, the P + boost loop, stall detection, the stall latch and the tolerances keep their units.
  Tolerances that are measured against the size of the travel are re-tuned.
- **Turn recovery at boot and after a sensor fault**: the firmware uses the coarse sensor to work
  out which turn the fine sensor is on, and accepts it only after several consecutive coarse polls
  agree. If the two sensors disagree by more than a quarter turn, the firmware treats it as a sensor
  fault (a mismatch).
- **Continuous consistency check** between the two sensors while running. A mismatch that persists
  is a sensor fault that latches until the calibration is completed again or the system reboots.
- **Boot centering fixed**: the move to centre is deferred until the VESC link is up (it was always
  rejected in `setup()` before), and dropped if that takes longer than a few seconds.
- **Fault policy (unchanged in spirit)**: any fault on either sensor stops the motor and rejects
  commands. The faults are an I2C failure, a lost magnet, or a mismatch. There is no degraded mode
  that runs on one sensor.
- **Calibration UI unchanged**: the same jog and nudge controls and the same three capture buttons.
  Each capture records both sensors. The firmware works out the coarse-to-fine gear ratio from the
  captures and checks it: the limits must be on opposite sides of centre, both sensors must move
  together in one direction, the ratio must give enough resolution, the coarse sensor must have an
  unused gap, and the centre capture must fall on the line through the two limits.
- **Calibration flow, BREAKING**: the centre, left and right points can be captured in any order;
  each button records only its own point and clears nothing else. The calibration is checked and
  saved automatically once all three points are set. All three must be captured within one
  power-on: a reboot or a sensor fault discards unfinished points. On a vehicle whose stored
  calibration and absolute turn are trusted, a single point can be re-captured.
- **`AS5600Sensor` becomes bus-agnostic** (it takes a `TwoWire&`). Only the fine sensor's driver
  may tear down and restart its own bus (`Wire`), which it has to itself. The coarse sensor never
  restarts `Wire1`. A **coordinated I2C2 recovery** owned by `main.cpp` restarts `Wire1` only when
  the coarse sensor and the opto-input expander are failing at the same time, which points to the
  bus rather than to one device. It runs at most once every few seconds.
- **NVS**: new calibration keys in the existing `"steering"` namespace. **BREAKING**: the old
  `c_ang`/`l_ang`/`r_ang` keys are ignored and removed, so **every vehicle needs recalibrating after
  flashing**. The `steer_sca_base` key in the same namespace is kept.
- **Telemetry and UI**:
  - These fields change meaning: `steer_position`, `steer_left` and `steer_right` are now
    fine-sensor counts measured from the centre (unwrapped), and `steer_center` is `0` or `null`.
    "Unset" is now `null` instead of `-1`, because positions can now be negative.
    While calibration points are pending they show the captured points (relative to the centre
    point, or to the first point captured when there is no centre yet).
  - New diagnostic fields: `steer_fine_raw`, `steer_coarse_raw`, `steer_turn`, `steer_ratio`,
    `steer_residual` and `steer_sensor_fault`; calibration progress fields `steer_cal_points`,
    `steer_cal_error` and `steer_cal_lost`.
  - MAVLink `STEER_POS` keeps its meaning (percent of calibrated travel). It is now valid only while
    both sensors are healthy and agree.
- **Docs and version**:
  - Update the pinout document and the `Constants.h` comments: X15 / I2C2 now also carries the
    coarse AS5600, and X14 now carries the fine sensor.
  - Bump the firmware version to 1.2.0 and add a changelog entry.

## Impact
- Affected specs:
  - `vehicle-systems`: Steering Control System modified. Dual-Sensor Multi-Turn Steering Position
    and Dual-Sensor Steering Calibration added.
  - `vehicle-actuators`: Hardware Configuration modified (bus contents of I2C1 and I2C2).
  - `gpio-expander`: Dual I2C Bus Topology and Address Map modified. Coordinated I2C2 Bus Recovery
    added.
  - `web-telemetry`: Steering Position Sensor Telemetry added.
  - `mavlink-interface`: no delta. `STEER_POS` is still percent of calibrated travel, and its
    validity gate ("sensor healthy AND calibrated") now reads the combined dual-sensor health
    through the existing `isSensorOk()`.
- Affected code:
  - `include/AS5600Sensor.h`, `src/AS5600Sensor.cpp`
  - `include/SteeringController.h`, `src/SteeringController.cpp`
  - `include/Constants.h`
  - `src/main.cpp`: begin the steering with both buses, coordinated `Wire1` recovery, boot scan
    comments
  - `src/TelemetryManager.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`
  - `src/VehicleController.cpp`: capture response text
  - `data/index.html`: calibration card, en/uk strings
  - `include/Changelog.h`
  - `GPIO_PINOUT_CUSTOM_BOARD_S3.md`
- Hardware: needs a second AS5600 with its magnet on the geared-down shaft, wired to X15. The
  gearing must give at least `STEER_COARSE_MIN_PER_TURN` coarse counts per steering-shaft turn and
  leave at least `STEER_COARSE_MIN_GAP` counts of unused coarse arc. In practice that means a
  coarse sweep of about 200° to 330° lock to lock.
