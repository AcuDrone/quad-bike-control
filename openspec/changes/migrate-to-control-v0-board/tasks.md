## 1. Pre-flight

- [ ] 1.1 Tag the last DevKit-compatible commit (e.g. `git tag hw-devkitc1-last`) as the rollback
  point, and note the tag in the PR description.
- [x] 1.2 **[USER/MANUAL]** Confirm board straps before first power-up: I2C2 level-shifter low side
  = 3V3 (`R16` populated, `R18` not — pinout §8.6), and the relay board's PCA9557 strapped to any
  address other than 0x1A (U10) or 0x1C (U2) — the as-shipped A2A1A0 = 111 → **0x1F** is kept, no
  rework (§8.8). DONE: strap kept at 0x1F and the relay board answers on **X15 / I2C2**. Header X15
  initially passed no signals (two failed scans) and was fixed by rework in the `FB3`/`FB4` ferrite
  area — check there first on any board copy with a silent X15.
- [ ] 1.3 **[USER/MANUAL]** Confirm the CH9121T / LAN section is unpopulated or held silent so the
  Prog header on GPIO43/44 remains usable (§8.1), and that X9 pin 1 (5V) is **not** wired to the
  Pixhawk TELEM 5V pin (§8.4).

## 2. Constants rewrite (`include/Constants.h`)

- [x] 2.1 Re-pin per `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §9: `PIN_THROTTLE_PWM` 3→15,
  `PIN_TRANS_SERVO` 9→16, `PIN_MAVLINK_TX` 15→17, `PIN_MAVLINK_RX` 8→18, `PIN_CAN_CS` 10→39,
  `PIN_CAN_SCK` 12→40, `PIN_CAN_MISO` 13→41, `PIN_CAN_MOSI` 11→42, `PIN_STEER_SDA` 41→1,
  `PIN_STEER_SCL` 42→2. Leave `PIN_VESC_TX/RX` (4/5) and `PIN_BRAKE_LPWM/RPWM` (6/7) unchanged.
- [x] 2.2 Add `PIN_CAN_INT` (GPIO38, defined only — the CAN driver keeps polling), `PIN_BOOST_EN`
  (GPIO46), `PIN_ADC_24V` (GPIO9), `PIN_ADC_IS_MUX` (GPIO3, defined only).
- [x] 2.3 **Delete** `PIN_STEER_RPWM`, `PIN_STEER_LPWM`, `LEDC_CH_STEER_RPWM`, `LEDC_CH_STEER_LPWM`
  and their "RESERVED/FREED" comment block — GPIO17/18 are now the MAVLink UART.
- [x] 2.4 **Delete** `PIN_BRAKE_SENSOR`, `PIN_GEAR_REVERSE`, `PIN_GEAR_NEUTRAL`, `PIN_GEAR_LOW`,
  `PIN_GEAR_HIGH`, `PIN_RELAY1`, `PIN_RELAY2`, `PIN_RELAY3`, `PIN_WHEEL_LOCK`.
- [x] 2.5 Add the I2C/expander block: `PIN_RELAY_SDA` (48), `PIN_RELAY_SCL` (47), `I2C_BUS_FREQ_HZ`
  (100000 — 100 kHz, long AS5600 cable run to X14), `PCA9557_ADDR_INPUTS` (0x1A), `PCA9557_ADDR_RELAYS` (0x1F), `PCA9557_ADDR_MUX_RESERVED`
  (0x1C, documented as never-addressed), and the opto-input bit indices In1-In5 →
  REVERSE/NEUTRAL/LOW/HIGH/BRAKE plus the relay port masks `RELAY_MASK_IGNITION` (`0b00110000`,
  Relay_1 IO4 + Relay_2 IO5), `RELAY_MASK_STARTER` (`0b01000000`, Relay_3 IO6),
  `RELAY_MASK_FRONT_LIGHT` (`0b10000000`, Relay_4 IO7) and `RELAY_MASK_WHEEL_LOCK` (`0b00001100`,
  Relay_5 IO3 + Relay_6 IO2). ⚠ The board's IO routing is NOT 1:1 — do not use per-relay bit indices.
- [x] 2.6 Add the timing/policy constants: `BOARD_INPUT_POLL_MS` (25), `BOARD_INPUT_STALE_MS` (250),
  `BOARD_INPUT_RECOVER_MS` (1000), `RELAY_WRITE_RETRIES` (3), `RAIL_SAMPLE_INTERVAL_MS` (500),
  `RAIL_24V_DIVIDER_RATIO` (9.33f), `RAIL_24V_LOW_THRESHOLD` (21.0f), `BOOST_STARTUP_GRACE_MS` (1000).
- [x] 2.7 Update the file's header comments so no stale ESP32-C6 / DevKit references remain, and add
  a pointer to `GPIO_PINOUT_CUSTOM_BOARD_S3.md` as the wiring authority.
- [x] 2.8 Build gate: `~/.platformio/penv/bin/pio run` — expect failures only in the files that still
  reference the deleted constants (they are fixed in §4-§6); no *unexpected* errors.

## 3. PCA9557 driver (`include/PCA9557Expander.h`, `src/PCA9557Expander.cpp`)

- [x] 3.1 Implement a minimal PCA9557 register driver bound at construction to one `TwoWire&` and
  one 7-bit address: `begin()`, `readInputs(uint8_t&)`, `writeOutputs(uint8_t)`,
  `readOutputRegister(uint8_t&)`, `setDirection(uint8_t)`, `setPolarityInversion(uint8_t)`,
  `isPresent()`. Registers: 0=input, 1=output, 2=polarity inversion, 3=configuration.
- [x] 3.2 Every transaction returns success/failure from `Wire::endTransmission()` /
  `requestFrom()`; no method blocks longer than one I2C transaction and none retries internally
  (retry policy belongs to the callers).
- [x] 3.3 Doxygen the public API per project convention; keep the implementation under ~120 lines.

## 4. Opto-input reader (`include/BoardInputs.h`, `src/BoardInputs.cpp`)

- [x] 4.1 Implement `BoardInputs` on `Wire` @ `PCA9557_ADDR_INPUTS`: `begin(TwoWire&)` configures all
  eight pins as inputs and takes a first read; `update()` polls every `BOARD_INPUT_POLL_MS`.
- [x] 4.2 Publish an immutable snapshot `{gearReverse, gearNeutral, gearLow, gearHigh, brakeReleased,
  valid, ageMs}` via `getSnapshot()`; consumers never issue I2C themselves.
- [x] 4.3 Implement the fault policy from `design.md` Decision 2: on read failure keep the last-known
  snapshot and increment a failure counter; when `ageMs > BOARD_INPUT_STALE_MS` set `valid = false`,
  raise the fault flag and log once per episode (rate-limited); while faulted, retry a bus/device
  re-init every `BOARD_INPUT_RECOVER_MS` and log recovery on the first good read.
- [x] 4.4 Expose `isFaulted()`, `getFailureCount()` and `getAgeMs()` for telemetry and diagnostics.
- [x] 4.5 Construct `boardInputs` in `src/main.cpp`, `begin()` it after `Wire.begin()`, and call
  `update()` once per `loop()` iteration alongside the other subsystems.

## 5. RelayController rewrite (`include/RelayController.h`, `src/RelayController.cpp`)

- [x] 5.1 Replace the four `pinMode`/`digitalWrite` GPIO paths with a PCA9557 on `Wire1` @
  `PCA9557_ADDR_RELAYS`; keep an 8-bit output shadow byte and always write the whole port.
  **The public API (`begin`, `setIgnitionState`, `requestCrank`, `setFrontLight`, `setWheelLock`,
  `update`, `allOff`, all getters) and the `IgnitionState` semantics must not change.**
- [x] 5.2 Build the shadow byte from the `RELAY_MASK_*` masks: ignition/ECU line = Relay_1 + Relay_2
  (IO4+IO5, paralleled pair), starter = Relay_3 (IO6), front light = Relay_4 (IO7), front-wheel lock
  = Relay_5 + Relay_6 (IO3+IO2, paralleled pair); spares Relay_7/Relay_8 (IO0/IO1) stay LOW.
  Preserve the existing truth table (OFF: ignition mask LOW / starter mask LOW; ACC and IGNITION:
  ignition mask HIGH / starter mask LOW; CRANKING: both HIGH) and the `ACC_PRECRANK_DWELL_MS` /
  `CRANKING_TIMEOUT` / RPM-exit crank logic untouched.
- [x] 5.3 In `begin()`, configure all eight pins as outputs and drive the port to 0x00 (all relays
  off) **before** returning; probe `PCA9557_ADDR_RELAYS` and, if absent, sweep the whole PCA9557
  block 0x18-0x1F on the driver's own bus and log which addresses ACKed ("none" if silent). Do NOT
  blame a mis-strapped relay board for an answer at 0x1C — that is the on-board mux U2. Return
  `false` on failure.
- [x] 5.4 Implement write verification: after every port write, read the output register back and
  compare; retry up to `RELAY_WRITE_RETRIES`; on persistent mismatch set `isFaulted()` and log.
- [x] 5.5 Implement the starter escalation from `design.md` Decision 3: `RELAY_MASK_STARTER` is only
  ever set in `CRANKING`; a crank aborts if the read-back does not confirm engagement; any relay fault
  while a starter-mask bit is set triggers an immediate repeated all-off write, latches ignition to
  `OFF`, and refuses new crank requests until the expander verifies clean again.
- [x] 5.6 When `begin()` failed or the driver is faulted, all setters become logging no-ops and
  `getIgnitionState()` reports `OFF`; the rest of the vehicle continues running degraded.
- [x] 5.7 In `src/main.cpp`, call `Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ)` before
  `relayController.begin()`.

## 6. Rewire the existing input consumers

- [x] 6.1 `src/TransmissionController.cpp`: give the controller a reference/pointer to `BoardInputs`;
  rewrite `initGearSensors()` (no more `gpio_set_direction`/`gpio_set_pull_mode`) and
  `getPhysicalGear()` (`:219-261`) to read the snapshot instead of `digitalRead()`. Keep the
  `TRANS_GEAR_READ_INTERVAL_MS` cache and the multi-active / none-active → `GEAR_UNKNOWN`
  resolution; replace the immediate re-read retry loop with "consume the next snapshot".
- [x] 6.2 When the snapshot is invalid (`valid == false`), `getPhysicalGear()` returns
  `GEAR_UNKNOWN` — verify this drives the existing `TRANS_UNKNOWN_GEAR_THROTTLE_MAX` (5%) cap and
  blocks gear-change confirmation with no additional interlock.
- [x] 6.3 `include/VehicleController.h:132`: rewrite `isBrakeReleased()` to read
  `boardInputs.getSnapshot().brakeReleased`, returning `false` ("not confirmed released") whenever
  the snapshot is invalid, so retraction stays bounded by `BRAKE_SENSOR_OVERRUN_TIME` /
  `BRAKE_FULL_TRAVEL_TIME`.
- [x] 6.4 `src/main.cpp`: delete the `pinMode(PIN_BRAKE_SENSOR, INPUT)` block and its
  `digitalRead` startup log (`:137-139`); replace with a snapshot-based startup log after
  `boardInputs.begin()`.
- [x] 6.5 Move/duplicate `Wire.begin(PIN_STEER_SDA, PIN_STEER_SCL, I2C_BUS_FREQ_HZ)` so the bus is up
  before both the AS5600 and the input expander, without depending on steering init succeeding
  (see `design.md` Open Questions).

## 7. 24V boost control, rail telemetry and UI

- [x] 7.1 `src/main.cpp`: as the very first action in `setup()`, `pinMode(PIN_BOOST_EN, OUTPUT)` and
  drive it HIGH; log the enable (accepting that the log may predate USB-CDC enumeration).
- [x] 7.2 Add rail sampling (in `VehicleController` or a small helper): read `PIN_ADC_24V` every
  `RAIL_SAMPLE_INTERVAL_MS`, average 8 samples, convert with `RAIL_24V_DIVIDER_RATIO`, expose
  `getRail24V()`, `isBoostOn()` and a `rail_low` flag that is suppressed for
  `BOOST_STARTUP_GRACE_MS` after enable and asserted below `RAIL_24V_LOW_THRESHOLD`.
- [x] 7.3 Add `rail_24v` (float), `boost_on` (bool) and `rail_low` (bool) to the `Telemetry` struct
  (`include/WebPortal.h`), populate them in `src/TelemetryManager.cpp`, and emit them in the
  WebSocket JSON (`src/WebPortal.cpp`).
- [x] 7.4 Add `io_input_fault` and `io_relay_fault` booleans to the same telemetry path, sourced from
  `BoardInputs::isFaulted()` and `RelayController::isFaulted()`.
- [x] 7.5 Add the `set_boost` web command: parse it in `src/WebPortal.cpp` like `set_light`, dispatch
  it in `VehicleController::processWebCommand` (`src/VehicleController.cpp:124-147`) to a
  `processBoostCommand(bool, WebPortal&)` that **refuses to disable the boost unless ignition is
  `OFF`** and replies `{"ok":false,"error":...}` on refusal; enabling is always allowed.
- [x] 7.6 `data/index.html`: add a 24V rail readout (value + low-rail colour state), a boost toggle
  wired to `set_boost`, and an I/O fault indicator; add matching `en` **and** `uk` i18n keys.
- [x] 7.7 Deploy the UI with `~/.platformio/penv/bin/pio run -t uploadfs` — a firmware flash alone
  does not update `data/index.html`. DONE over native USB; it only succeeds since the LittleFS
  partition was shrunk to 1.5 MB (`partitions_16mb.csv`).

## 8. Build system and console

- [x] 8.1 `platformio.ini`: add `build_flags = -DARDUINO_USB_CDC_ON_BOOT=1` to `[env]` (or the
  `esp32-s3-devkitc-1` env), and add a comment that UART0 / GPIO43-44 is reserved for the future
  CH9121T wired LAN and must not be used for the console.
- [x] 8.2 Verify no `while (!Serial)` or equivalent host-wait is introduced anywhere — the firmware
  must boot fully with no USB host attached.

## 9. Documentation and cross-change dependency

- [x] 9.1 Cross-check the finished `include/Constants.h` against `GPIO_PINOUT_CUSTOM_BOARD_S3.md`
  §2 and §9 pin by pin; correct whichever is wrong (the document is authoritative for wiring, the
  header for firmware) and record any divergence.
- [x] 9.2 Update `openspec/changes/add-hall-speed-sensor/proposal.md`: `PIN_SPEED_SENSOR` GPIO2
  (fallback GPIO1) → **GPIO8** (6N137 opto-isolated input, connector X2), and replace the
  `GPIO_PINOUT_S3.md` reference with `GPIO_PINOUT_CUSTOM_BOARD_S3.md`. Note that GPIO1/2 are now
  I2C1, and that the fitted sensor is a **12V** type — which is why it takes the opto path instead
  of a 5V-scaled Hall channel.
- [x] 9.3 Update `openspec/changes/add-hall-speed-sensor/tasks.md`: change task 2.1's pin to GPIO8
  and **delete/replace hardware tasks 1.1 and 1.2** — the external level shifter is obsolete because
  the X2 opto input (470Ω `R22` → 6N137 LED → open-collector output + pull-up → GPIO8) is
  current-driven and galvanically isolated. Replace them with (a) a wiring task covering both sensor
  types — push-pull 12V: signal → `X2.1`, GND → `X2.2`; open-collector NPN: +12V through the series
  resistor → `X2.1`, output → `X2.2` — and (b) a **required** series-resistor task: add 560Ω–1kΩ in
  the sensor cable (≈7–10 mA) or rework `R22` to 1kΩ, because 470Ω alone at 12V gives ≈22 mA, over
  the 6N137's 20 mA absolute maximum (pinout §3/§8.9).
- [x] 9.4 Update `openspec/changes/add-hall-speed-sensor/design.md` and its
  `specs/speed-sensor/spec.md` wherever GPIO2 / GPIO1 / Hall_1 / external level shifting are
  referenced, and note that the 6N137 output is **inverted** (LED on → GPIO8 low) — the PCNT config
  counts one chosen edge and records the inversion.
- [x] 9.5 Re-validate the touched change: `openspec validate add-hall-speed-sensor --strict`. Note it
  currently fails for a **pre-existing** reason unrelated to this migration — its `can-controller`
  MODIFIED block for "Speed-Based Gear Change Prevention" omits the existing
  "Timeout fallback when CAN data is unavailable" scenario. Fix that at the same time by copying the
  missing scenario into the MODIFIED block.
- [x] 9.6 Update `openspec/project.md` where it still says "ESP32-C6" and "S-bus" — the platform is
  ESP32-S3 on `Control_v0` and the command link is MAVLink.

## 10. Build gate

- [x] 10.1 Clean firmware build passes with no errors and no warnings about undefined pin macros:
  `~/.platformio/penv/bin/pio run`.
- [x] 10.2 Grep the tree for the deleted symbols (`PIN_RELAY`, `PIN_GEAR_`, `PIN_BRAKE_SENSOR`,
  `PIN_WHEEL_LOCK`, `PIN_STEER_RPWM`, `PIN_STEER_LPWM`, `LEDC_CH_STEER_`) and confirm zero hits
  outside `openspec/` and the archived pinout docs.

## 11. On-bench smoke tests (vehicle on stands, wheels clear)

- [ ] 11.1 **[USER/MANUAL]** Power-up: confirm the 24V rail comes up within ~1 s of boot, measure it
  at X10, and confirm the reported `rail_24v` matches the meter within ~0.5 V. Confirm GPIO46 shows
  no enable glitch through reset (pinout §8.2).
- [ ] 11.2 **[USER/MANUAL]** Console: confirm boot log and runtime output appear on the USB-C serial
  monitor, and that the board boots normally with USB unplugged.
- [x] 11.3 **[USER/MANUAL]** I2C scan on both buses. Bench-verified result: `Wire1` (I2C2) answers
  **0x1A (U10), 0x1C (U2) and 0x1F (relay board on X15)**; `Wire` (I2C1) is empty until the AS5600
  is plugged into X14, where it must then show 0x36 alone. Any other result stops bring-up. The
  debug-gated boot scan in `src/main.cpp` prints both buses on every boot. Both buses run at
  **100 kHz**
  (`I2C_BUS_FREQ_HZ`) by decision — the AS5600 is at the far end of a long cable run to the steering
  column (X14) and signal-integrity margin beats bus speed there; 400 kHz was tried and rejected.
- [ ] 11.3a **[USER/MANUAL] [OPTIONAL — only if faster steering reads are ever wanted]** Bench
  experiment, not part of bring-up: with a stronger pull-up fitted on the X14 SDA/SCL run, scope the
  edges at the AS5600 end and try `I2C_BUS_FREQ_HZ` at 400000. Keep 100 kHz unless the rise times
  and an extended error-free read soak clearly justify the change; the constant only moves after the
  hardware change is verified.
- [ ] 11.4 **[USER/MANUAL]** Opto inputs: move the gear selector through R / N / L / H and confirm
  the reported physical gear follows each position with exactly one input asserted; confirm the
  brake limit sensor state flips at the retracted endstop. Confirm the input bit polarity matches the
  firmware's "asserted" sense (`design.md` Open Questions).
- [ ] 11.5 **[USER/MANUAL]** Input fault behavior: unplug X16 (or hold SCL low) and confirm the
  gear goes `UNKNOWN` within `BOARD_INPUT_STALE_MS`, throttle is capped at 5%, `io_input_fault`
  appears in telemetry, and everything recovers automatically on reconnect.
- [x] 11.6 **[USER/MANUAL]** Relays, engine disabled (starter lead disconnected or fuel/ignition
  isolated): exercise each function from the web UI / MAVLink and confirm the correct relays click —
  ignition clicks **both** Relay_1 (X4) and Relay_2 (X5), the wheel lock clicks **both** Relay_5 (X8)
  and Relay_6 (X9), starter is Relay_3 (X6) alone, front light is Relay_4 (X7) alone — the read-back
  verifies, and Relay_7 (X10) / Relay_8 (X11) never move.
- [x] 11.7 **[USER/MANUAL]** Crank sequence with the starter lead still disconnected: confirm the
  `ACC_PRECRANK_DWELL_MS` dwell before Relay_3 (starter) engages, and that Relay_3 releases on
  `CRANKING_TIMEOUT`.
- [ ] 11.7a **[USER/MANUAL]** Relay-bus fault escalation: unplug the relay board's I2C connector
  (X15) mid-crank and confirm the all-off escalation, ignition latching to `OFF`, and
  `io_relay_fault` in telemetry.
- [ ] 11.8 **[USER/MANUAL]** Servos: confirm throttle (GPIO15) and transmission (GPIO16) move on the
  correct X8 pins with the correct LEDC channels and 50 Hz; confirm the brake BTS7960 (GPIO6/7 → X7)
  drives both directions.
- [ ] 11.9 **[USER/MANUAL]** Steering: AS5600 on the new GPIO1/2 reads a live angle, the VESC on
  GPIO4/5 (X6) responds to `COMM_GET_VALUES`, and a small closed-loop move completes. Confirm the
  saved NVS calibration survived the migration.
- [ ] 11.10 **[USER/MANUAL]** MAVLink on GPIO17/18 (X9): heartbeat and `SERVO_OUTPUT_RAW` exchange at
  115200 with the Pixhawk. **Do not raise the baud rate** (pinout §8.3).
- [ ] 11.11 **[USER/MANUAL]** CAN on GPIO39/40/41/42 with the 8 MHz crystal (`MCP_8MHZ`, unchanged):
  MCP2515 init succeeds and RPM is read from the ECU.
- [ ] 11.12 **[USER/MANUAL]** Boost web toggle: confirm disable is refused while ignition is not
  `OFF` and accepted when it is, and that re-enable always works and the rail recovers.
- [ ] 11.13 **[USER/MANUAL]** Failsafe: drop the MAVLink link and confirm `allOff()` still turns
  ignition and light off over the expander while the wheel lock holds its last state.

## 12. Validate

- [x] 12.1 `openspec validate migrate-to-control-v0-board --strict` passes with no errors.
- [x] 12.2 `openspec list` shows the change with its task count.
