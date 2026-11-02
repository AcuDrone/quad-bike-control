# Design: Pulses-per-revolution measurement and the speed card split

## Context
- `SpeedSensor` counts the hall signal with the ESP32-S3 PCNT (`src/SpeedSensor.cpp`
  `begin()` / `update()`):
  - one channel on `PIN_SPEED_SENSOR`, no level/direction input (`level_gpio_num` unused);
  - edge actions: rising = HOLD, falling = INCREASE (the opto inverts the signal). Level actions
    KEEP. **The counter can only increase**;
  - limits `SPEED_PCNT_LOW_LIMIT` = −1 / `SPEED_PCNT_HIGH_LIMIT` = 10000, both registered as watch
    points, with `accum_count = 1`. The driver adds the overflow itself, so
    `pcnt_unit_get_count()` returns a running total that keeps rising past 10000;
  - glitch filter `SPEED_GLITCH_FILTER_NS` = 1000 ns. There is no minimum frequency, debounce or
    dead-band in firmware;
  - `update()` samples every `SPEED_SAMPLE_INTERVAL_MS` (200 ms) and writes
    `pulseTotal_ = (uint32_t)rawCount` **before** the per-sample glitch guard
    (`SPEED_MAX_PULSES_PER_SAMPLE`) and before any validity/decay logic. The total is therefore
    independent of `isValid()`, the stale timeout and the suspicious latch.
- Telemetry is broadcast every `TELEMETRY_INTERVAL` (200 ms) in a `StaticJsonDocument<4096>`
  (`src/WebPortal.cpp`, `createTelemetryJSON()`, capacity comment above it).
- `speed_cal_ppr` → `VehicleController::processSpeedCalPprCommand()` →
  `SpeedSensor::setPulsesPerRev()`: range `SPEED_PPR_MIN`..`SPEED_PPR_MAX` (1..1000), persisted to
  NVS `"speed"`/`ppr`, accepted regardless of the input source. The page sends it from
  `saveSpeedPpr()`.
- The speed settings inputs are not gated by `updateControlsAvailability()` today: they stay
  enabled, and `sendCommand()` drops the command when the WebSocket is down. Take-control is not
  needed for any of them.
- Approved mockup: `ui-mockups/ppr-mockup.html`, built from `data/index.html` by the session
  scratchpad scripts `apply_ppr.py` (measurement block, i18n, JS) and `split_cards.py` (card split,
  help split, status routing).

## Goals / Non-Goals
- Goals:
  - the operator measures ppr on the vehicle with no tools but a chalk mark, in under a minute;
  - the firmware stays stateless: one read-only telemetry value, no command, no NVS;
  - a wrong measurement (inexact turns, missed pulses, no signal) is visible before it is saved;
  - the speed settings read as two short cards: what calibrates the sensor, and what uses it.
- Non-Goals:
  - measuring the wheel circumference (it stays a typed value, measured with a tape);
  - a firmware-side measurement session, timeouts or persistence (variant B, rejected);
  - direction sensing. The single-channel sensor cannot tell direction (D2);
  - a JS test framework (D6).

## Decisions

### D1 — Variant A: page-side measurement over a running total
- The page snapshots `speed_pulse_total` at Start and computes `delta = (now − start) >>> 0` on
  every frame. Nothing is sent to the firmware until the operator saves the result, and then only
  the existing `speed_cal_ppr`.
- Why: the total already exists and is exact. A firmware session would add a command, state,
  a timeout and a "who owns the session" question for multiple clients, and would gain nothing.
  Two browsers can measure at the same time without interfering.
- Alternative (B, firmware start/stop commands holding the start count): rejected by the user on
  2026-11-02.

### D2 — PCNT behaviour at hand-turning speed and in reverse (confirmed by code reading)
- **Hand speed.** Counting is edge-driven with only a 1 µs glitch filter. A hand-turned wheel
  (≈ 0.2–2 rev/s × 70 teeth = 14–140 Hz, periods ≥ 7 ms) is far from any filter or limit. The
  firmware has no minimum frequency, and the stale/suspicious logic only affects `getSpeedMs()` /
  `isValid()`, not `pulseTotal_`. The remaining unknown is the sensor itself: a 3-pin digital
  hall gear-tooth sensor switches at zero speed, a VR pickup would not. That is confirmed by
  the bench check (task 9.1), not by code.
- **Reverse rotation counts up, not down.** There is no direction channel. Falling edges INCREASE
  whatever the direction the teeth pass. Turning the wheel backwards adds pulses. So does
  rocking it back to line the mark up after an overshoot. Consequences:
  - the hint and the help say to turn in one direction only; an overshoot means Cancel and repeat.
    They also say to lift the wheel and select neutral (user decision 2026-11-02): on the quad, a
    wheel turned by hand in gear drags the engine;
  - a "wrong" count from back-and-forth movement shows up as a large fractional part (warning) or
    a plausible-but-wrong integer. The bench check confirms the behaviour (task 9.3).
- **Overflow / wrap.** `accum_count` removes the 10000-count hardware wrap. The accumulated value
  is a C `int` in the driver, cast to `uint32_t`. It wraps modulo 2³² after ≈ 2.2·10⁹ pulses
  (≈ 26 days of continuous driving at 100 km/h). The page uses `(now − start) >>> 0`, which is
  correct across that wrap. JS numbers carry uint32 exactly. The JSON emits a plain unsigned
  integer.
- **Reboot.** A reboot resets the total to 0 and always drops the WebSocket. The disconnect
  returns the block to idle (D4), so a stale start snapshot is never subtracted from a fresh
  total. If it ever were, the unsigned delta would be enormous, and the result range check (D3)
  refuses it.
- **Side effect.** Hand-turned pulses are real pulses. They add to the odometer and the trip
  (≈ 2 m per turn) and briefly raise `vehicle_speed`, exactly as pushing the vehicle would. No
  suppression is added.

### D3 — Result rule (pure helper `pprMeasCompute(delta, turns)`)
- Input: unsigned delta, turns 1..20 (the turns input is clamped to 1..20 at Start and locked
  while measuring).
- `delta == 0` → kind `none` ("No pulses counted. Repeat."), no Save.
- `ppr = delta / turns`, `rounded = Math.round(ppr)`.
- `rounded < SPEED_PPR_MIN (1)` or `> SPEED_PPR_MAX (1000)` → kind `range`: the result line is
  shown with the existing `invalid_speed_ppr` text ("must be 1-1000") in the danger colour, no Save.
  This replaces the mockup's `Math.max(1, …)`, which would have offered to save 1 for a
  near-zero count.
- otherwise kind `ok`, Save shown. `warn = 5 × |delta − rounded × turns| ≥ turns`, which is
  `|ppr − rounded| ≥ 0.2` in exact integer arithmetic (no float edge at 70.2 / 69.8).
- The page constants for 1 / 1000 / 0.2 sit next to the helper with a comment naming
  `SPEED_PPR_MIN` / `SPEED_PPR_MAX` in `Constants.h`.
- The decimal shown uses one decimal, with a comma in Ukrainian (`pprFmt`, as in the mockup).

### D4 — Page state machine
- States: `idle` → (Start) `measuring` → (Stop) `done`; Cancel from `measuring` →
  `idle`; "Measure again" from `done` → `idle`.
- **Start** is enabled only while the WebSocket is connected and the last frame carried a numeric
  `speed_pulse_total`. A frame without the key, from an older firmware with a newer LittleFS image,
  keeps Start disabled. Start snapshots the last received total.
- **Stop freezes at once (user decision 2026-11-02).** The result uses the last
  `speed_pulse_total` received before the press. There is no settle delay. The total reaches the
  page through a 200 ms sample and a 200 ms broadcast, so a pulse just before Stop can be up to
  about 0.4 s late. The operator covers that: the on-page hint (`ppr_meas_hint`) says to wait
  until the live count stops changing before pressing Stop. A Stop pressed too early shows up as
  a short count and usually trips the fractional-part warning.
- **Disconnect.** `ws.onclose` resets the block to `idle` (the measured delta is lost by design).
  `updateConnectionStatus()` re-renders the block so Start follows the connection.
- **Reload.** The state is not persisted (no `localStorage`). A reload starts `idle`.
- **Language switch.** `setLanguage()` re-renders the block (as the mockup does via
  `pprMeasRender()`).
- **Take-control.** Not required, matching `speed_cal_ppr` (accepted regardless of the input
  source) and the other speed settings inputs. The block is not added to
  `updateControlsAvailability()`'s control-source gating. It follows only the WebSocket state.
- **Save.** Writes `rounded` into `#speed-ppr-input` and sends it through the same validation and
  `sendCommand('speed_cal_ppr', …)` path as `saveSpeedPpr()`, with `speedStatusTarget =
  'speed-status-msg'`. The firmware reply ("Pulses/rev …") appears in the card's status line. The
  mockup-only `#ppr-meas-status` element and `ppr_meas_saved` key are dropped. The block stays in
  `done`, so the operator can still press "Measure again".

### D5 — Card split and reply routing
- Taken from `split_cards.py`: the block from the limiter-floor input through the bench test
  speed moves, unchanged, into a new card `speed-limit-card` placed right after
  `speed-sensor-card` in the Calibration grid, with its own `#speed-limit-status-msg`.
- `speedStatusMsg()` writes to `speedStatusTarget`. Each save function sets it first:
  `saveSpeedPpr`, `saveSpeedCirc`, `pprMeasSave` → `speed-status-msg`; `saveSpeedLimitFloor`,
  `saveSteerScaBase`, `sendSteerTestSpeed` → `speed-limit-status-msg`.
- Found while reading the page: the reply-matching branch in `handleWebSocketMessage()` matches
  "Pulses/rev", "Wheel circumference", "Speed limit" and "Speed limiter" only. The firmware replies
  to `set_steer_sca_base` / `set_test_speed` ("Steering scale base …", "Test speed …") are therefore
  never shown; card 2 keeps "Saving…" after those saves. The branch gains those two prefixes so the
  replies land in card 2's status line. Nothing else in the routing changes.
- `data-bind` rows (speed limit, limiter row, steering scale, test speed) keep their names. The
  shared rendering path with the Diagnostics Speed card is untouched.
- The limiter floor stays a number input (user decision 2026-11-02).

### D6 — Testing
- Firmware: one getter feeding one JSON key; nothing host-testable. The 44 native tests stay
  unchanged and must still pass.
- Page: the repo has no JS test runner (no `package.json`), and none is added. The result rule
  lives in the pure `pprMeasCompute()` so it can be checked against a fixed table from the browser
  console (task 7.4). The state machine is checked on the bench.

## Risks / Trade-offs
- Back-and-forth wheel movement inflates the count → the hint says one direction; the fractional
  warning catches most cases; the operator compares with the expected value (70 on the quad).
- A hall sensor that does not switch at hand speed → the live count stays 0; the result says "No
  pulses counted", nothing is saved.
- Two cards make the Calibration section taller → each card is short. The mobile one-column
  layout is unchanged and checked on the bench (task 9.6).

## Migration Plan
- Flash firmware and LittleFS together (as every release). A newer page on older firmware keeps
  Start disabled. Older page on newer firmware ignores the extra key.
- Rollback: reflash 1.5.3; nothing is stored.

## Open Questions
- None. Both earlier questions were decided on 2026-11-02:
  - the hint and help say to lift the wheel and select neutral;
  - Stop freezes the result at once, and the hint says to wait for the count to stop changing.
