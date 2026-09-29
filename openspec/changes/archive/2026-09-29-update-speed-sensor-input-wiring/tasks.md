## 1. Specification (documentation only)

The firmware side of this change is already merged and deployed — commit `49ba19c` (noise
hardening, layer 1) and commit `30c28af` (confirmed wiring documented in the code). Nothing under
`src/`, `include/` or `data/` moves here; every task below is a spec or documentation task, and
every one of them is verification against code that already exists.

- [x] 1.1 Replace the `Hall-Effect Speed Sensor Hardware Interface` requirement (REMOVED) with
  `Opto-Isolated Hall Speed Sensor Input Stage` (ADDED), built around the confirmed PC817 opto
  stage: 12V toothed-ring pickup → 1 kΩ series resistor → PC817 LED, PC817
  collector → `PIN_SPEED_SENSOR` (GPIO 17), emitter → GND. Drop "reaches the ESP32-S3 directly",
  "there is no opto-isolated input stage" and "makes no assumption about signal inversion".
- [x] 1.2 State the polarity: the stage inverts, so the pin is **active LOW** and the counted edge
  is the falling one — the start of the active phase.
- [x] 1.3 State the mandatory external 1 kΩ pull-up from GPIO 17 to 3.3V, with the reason it is
  mandatory (ignition-coil pickup on a node held only by the ~45 kΩ internal pull-up produced
  phantom pulses and tripped the gear interlock) and the bench evidence (noise counters read 0
  with the engine idling once it was fitted).
- [x] 1.4 Remove the "wiring to be confirmed on the bench" escape clause and the
  "A 12V sensor must be level-shifted" scenario, both now answered by the fitted opto.
- [x] 1.5 Keep the two facts from the old requirement that are still true: GPIO 17 is the pin
  freed when the BTS7960 steering driver was removed, and GPIO 18 stays free/reserved.
- [x] 1.6 Update the `Hall Pulse Counting via PCNT` requirement: the counted edge is the falling
  one (documented, not arbitrary), the glitch filter is `SPEED_GLITCH_FILTER_NS` = 12.5 µs set to
  the ESP32-S3 hardware maximum, and a window above `SPEED_MAX_PULSES_PER_SAMPLE` (240 pulses per
  200 ms, the 80 km/h ceiling of ≈156 with a 1.5x margin) is discarded whole.
- [x] 1.7 Add a `Speed Sensor Noise Diagnostics` requirement covering the raw per-window count,
  the stray-pulse counter, the rejected windows/pulses counters, the three web telemetry fields
  and the rate-limited `[SPEED] window:` VEHICLE line — and the rule that all of them are counters
  only, never inputs to the speed or distance maths.

## 2. Verification against the merged code

- [x] 2.1 `include/Constants.h` — the `PIN_SPEED_SENSOR` block states the PC817 wiring, active-LOW
  polarity, the falling edge and the mandatory 1 kΩ pull-up; `SPEED_GLITCH_FILTER_NS` is 12500 and
  `SPEED_MAX_PULSES_PER_SAMPLE` is 240; `SPEED_NOISE_LOG_INTERVAL_MS` is 2000.
- [x] 2.2 `src/SpeedSensor.cpp` — `pcnt_channel_set_edge_action()` is `HOLD` on rising and
  `INCREASE` on falling; the window above the ceiling increments `rejectedWindows_` /
  `rejectedPulses_` and returns before any distance is added; `maybeLogWindow()` is rate limited by
  `SPEED_NOISE_LOG_INTERVAL_MS` and stays silent on a parked vehicle.
- [x] 2.3 `GPIO_PINOUT_S3.md` — the GPIO 17 row, the "Notes on the new speed sensor" section and
  the "Input electrical conventions" bullet all carry the confirmed wiring and the pull-up warning.
- [x] 2.4 `src/TelemetryManager.cpp` / `src/WebPortal.cpp` / `data/index.html` — the three fields
  are named `speed_raw_pulses`, `speed_stray_pulses` and `speed_rej_windows`.
- [x] 2.5 `openspec validate update-speed-sensor-input-wiring --strict` passes.

## 3. Build

- [x] 3.1 No build is required: this change touches no source file. The firmware it describes was
  built and flashed with commits `49ba19c` and `30c28af`.
