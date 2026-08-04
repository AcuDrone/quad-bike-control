## Context

Target board is **ESP32-S3-DevKitC-1 N16R8** (`platformio.ini`,
`[env:esp32-s3-devkitc-1]`; `qio_qspi` memory type frees GPIO 33-37). The
steering actuator today is a brushed DC motor driven by a BTS7960 H-bridge
with closed-loop position feedback from an AS5600 magnetic angle sensor.

Ground truth verified in the current tree:

- **BTS7960 driver.** `include/BTS7960Controller.h`; `src/BTS7960Controller.cpp`.
  Shared LEDC timer, `MOTOR_PWM_FREQ` 10 kHz, 8-bit resolution
  (`BTS7960Controller.cpp:34-51`). `setSpeed(int16_t -255..+255)` →
  RPWM/LPWM, never both high (`BTS7960Controller.cpp:104-126`); `stop()` =
  both PWM 0 = coast (`BTS7960Controller.cpp:128-137`). Steering pins:
  GPIO17 RPWM ch6 / GPIO18 LPWM ch7 (`include/Constants.h:21-24`).
- **Steering P-loop.** `src/SteeringController.cpp`. Duty
  `= |error|*STEER_KP + dutyBoost_`, clamped `[STEER_PWM_MIN_DUTY 140,
  STEER_PWM_MAX_DUTY 255]` (`SteeringController.cpp:210-216`). Stall
  detection 5 counts / 500 ms (`SteeringController.cpp:154-166`), move
  timeout 7500 ms (`SteeringController.cpp:192-196`), jog
  (`:113-132`), nudge (`:134-146`), sensor-fault stop (`:68-102`). Tuning
  constants `include/Constants.h:148-171`.
- **The re-command reset bug.** `VehicleController::processMavlinkCommands()`
  calls `steering_.setSteeringPercent(mavlink_.getSteering())` unconditionally
  every loop (`src/VehicleController.cpp:258-260`).
  `SteeringController::setSteeringPercent()` unconditionally sets
  `isMoving_=true` and resets `moveStartTime_`, `lastStallPosition_`,
  `lastStallCheckTime_`, `dutyBoost_=0`, `lastProgressPos_/Time_` on every call
  (`src/SteeringController.cpp:249-258`). At the ~25 Hz command rate the
  500 ms stall window and 7500 ms move timeout never elapse and boost never
  accumulates — stall protection is permanently defeated in MAVLink mode.
- **MAVLink UART.** `src/MavlinkInterface.cpp:33-45` binds MAVLink to
  `Serial1` = `UART_NUM_1`; `include/Constants.h:11-14` sets RX GPIO8 / TX
  GPIO15 @ 115200. `Serial` (`UART_NUM_0`) is the debug console
  (`src/main.cpp:53`).
- **Steering telemetry.** `TelemetryData` struct `include/WebPortal.h:57-66`;
  populated `src/TelemetryManager.cpp:47-54`; serialized to JSON
  `src/WebPortal.cpp:587-593`. Steering init `src/main.cpp:96-112`.

## Goals / Non-Goals

- **Goals:** Replace the steering driver with a VESC over UART with the
  minimum change to the proven position P-loop; give stall protection a real
  hardware backstop; fix the MAVLink re-command bug; add motor telemetry;
  keep the vehicle safe when the VESC is absent/unresponsive.
- **Non-Goals:** No change to the AS5600 sensing, calibration, or P-loop
  math. No change to the brake actuator (keeps BTS7960). No change to the
  MAVLink interface or throttle/transmission. Not switching steering to
  current-mode control (see trade-off below).

## Decisions

### D1 — Motor-driver abstraction
Add `IMotorDriver` with `void setSpeed(int16_t)` and `void stop()`.
`BTS7960Controller` implements it (brake unchanged); a new `VescMotorDriver`
implements it for steering. `SteeringController` holds an `IMotorDriver&`
instead of an owned `BTS7960Controller`. This keeps the P-loop's
`motor_.setSpeed(±duty)` call sites (`SteeringController.cpp:131,145,216`)
byte-for-byte and isolates all VESC specifics behind the interface.
`SteeringController::begin()` signature changes: it no longer takes RPWM/LPWM
pins/channels; it takes a reference to the already-initialized driver plus the
AS5600 SDA/SCL pins.

### D2 — Duty command, not current command
`setSpeed(-255..+255)` maps to `COMM_SET_DUTY` with
`duty = constrain(speed/255.0f, -1.0f, +1.0f)`. `stop()` sends
`COMM_SET_DUTY 0` (coast, matching current BTS7960 coast semantics).
- **Why duty:** the existing P-loop already emits a duty-like `-255..+255`
  effort with an empirically tuned floor (140, static-friction breakaway) and
  ceiling (255). Duty preserves those dynamics exactly — no re-tuning, minimal
  change, lowest regression risk on a safety-critical loop.
- **Alternative — `COMM_SET_CURRENT`:** current-mode would give the cleanest
  torque/over-current behavior, but it redefines the P-loop output from
  "effort" to "amps," changing closed-loop gain and the meaning of
  `STEER_KP`/`STEER_PWM_MIN_DUTY`, and forcing a full bench re-tune.
  Rejected for this change; revisit if duty-mode dynamics prove inadequate.
- **Cost of duty mode:** duty does not inherently bound current the way
  current-mode does. Mitigated by two independent backstops: (1) the VESC
  motor-current limit configured in VESC Tool (hardware), and (2) the firmware
  over-current monitor from `COMM_GET_VALUES` (D5).

### D3 — Protocol layer: minimal in-repo, not an external library
Add `VescProtocol` (packet framing + CRC16-XMODEM) and `VescMotorDriver`
(command/telemetry) in-repo. Only two commands are needed: `COMM_SET_DUTY`
(id 5, payload = int32 duty×100000, big-endian) and `COMM_GET_VALUES` (id 4).
Frame: `0x02, len, payload…, crc_hi, crc_lo, 0x03` for payloads ≤ 255 bytes
(short packet); the `GET_VALUES` reply is a long packet (`0x03, len_hi,
len_lo, …`) and the parser must handle both.
- **Why in-repo:** the project convention is minimal dependencies
  (`openspec/project.md`, "Simplicity First"). A ~150-line module for two
  commands is smaller and more auditable than adopting a general library, and
  lets us pin the `COMM_GET_VALUES` field offsets to the VESC firmware version
  actually flashed (those offsets differ between FW versions and are the main
  parsing hazard).
- **Alternative — `SolidGeek/VescUart` via `lib_deps`:** well-known and small,
  but pulls its own `Stream`/datatype layer for functionality we do not need,
  and still needs offset auditing against the flashed FW. Rejected; if the
  in-repo parser proves fragile across FW versions, adopting it is a clean
  fallback.
- Only the fields the monitor/telemetry use are decoded from `GET_VALUES`:
  motor current (avg), FET temperature, input voltage, and the fault code.
  Field offsets are documented next to the parser and validated on the bench
  against VESC Tool readouts.

### D4 — UART port and pin selection (from pin audit)
- **Port:** `Serial2` / `UART_NUM_2`. `UART_NUM_1` is MAVLink; `UART_NUM_0` is
  the debug console. `UART_NUM_2` is free.
- **Pins:** `PIN_VESC_TX = GPIO4` (ESP TX → VESC RX), `PIN_VESC_RX = GPIO5`
  (ESP RX ← VESC TX), 3.3 V logic, direct wiring, 115200 baud
  (`VESC_UART_BAUD`, VESC app default).
- **Rationale:** GPIO4 and GPIO5 are the only completely unassigned, adjacent,
  low-number GPIOs (verified: zero references in `include/`+`src/`). They are
  not strapping pins, not the SPI-flash/PSRAM pins (26-32), not the CAN SPI
  pins (10-13), not I2C (41/42), not USB/JTAG. ESP32-S3 has no input-only pins
  and routes any UART through the GPIO matrix, so a clean adjacent pair is the
  only constraint. GPIO17/18 are freed by removing the steering PWM but are
  **not** reused for UART — they and LEDC ch6/7 are left documented/reserved so
  the wiring change stays localized and the freed PWM channels remain available.

Full audit — **assigned** GPIOs (`include/Constants.h`): 3 (throttle), 6/7
(brake PWM), 8/15 (MAVLink UART1), 9 (trans servo), 10-13 (CAN SPI), 14 (brake
sensor), 17/18 (steering PWM → freed by this change), 19/20/21/47 (gear
switches), 36-39 (relays/wheel-lock), 41/42 (AS5600 I2C). **Free** low pins:
1, 2, **4**, **5**, 16 (33/34/35/40/48 also free but higher/less convenient).

### D5 — Stall protection: re-command guard + stall latch + VESC over-current
Three layers, all converging on one stall-stop path.

**(a) Re-command no-op guard.** In `setSteeringPercent(percent)`, after mapping
`percent` to `target` counts: if a move is already in progress (`isMoving_`)
**and** `|target - targetRel_| <= STEER_RETARGET_TOLERANCE`, update nothing and
return (the in-flight move keeps its `moveStartTime_`/stall timers/boost). Only
when the target changes by more than `STEER_RETARGET_TOLERANCE` do the timers
reset. This makes the 500 ms stall window and 7500 ms move timeout actually
elapse under the ~25 Hz MAVLink stream, fixing the bug.
`STEER_RETARGET_TOLERANCE` (new, ≈10 counts ≈ `STEER_POSITION_TOLERANCE`) is
large enough to absorb per-loop rounding of a steady steering command.

**(b) Stall latch.** When `update()` detects a stall
(`SteeringController.cpp:154-166`) it now, in addition to `stop()`, records:
`stallLatched_=true`, `stallLatchDir_ = sign(error at stall)` (+1 = was pushing
right/toward larger rel, -1 = left), `stallLatchTime_ = millis()`.

Acceptance rule when a **new** target is requested (a change beyond
`STEER_RETARGET_TOLERANCE`, i.e. not a no-op) — and applied identically to
`jog()`:
- Let `d = sign(newTarget - currentRel)` (jog: `d = jog direction`).
- **Reject** (keep latch, do not command, `setSteeringPercent` returns false)
  iff `stallLatched_ && d == stallLatchDir_ &&
  (now - stallLatchTime_) < STEER_STALL_COOLDOWN_MS`.
- **Otherwise accept**, and clear the latch. Two accept paths:
  1. `d != stallLatchDir_` → moving away from / across the jam: accepted
     **immediately** (escape path).
  2. cooldown elapsed → same-direction retry allowed.

`STEER_STALL_COOLDOWN_MS` (new, ≈1500 ms). Must-not-brick guarantees, by
construction: opposite-direction moves are never blocked; same-direction moves
are blocked only during the ≤1.5 s cooldown and then permitted; any accepted
move clears the latch; a genuine no-op re-command (guard in (a)) neither
extends nor re-arms anything. The latch cannot outlive one cooldown without a
fresh stall.

**(c) VESC firmware over-current / fault monitor.** Poll `COMM_GET_VALUES`
every `STEER_VESC_TELEM_MS` (≈300 ms, ~3 Hz). If motor current stays above
`STEER_VESC_OVERCURRENT_A` (≈18 A, below the VESC-configured hardware limit)
for `STEER_VESC_OVERCURRENT_MS` (≈400 ms), invoke the same stall-stop + latch
path (latch direction = current drive direction). Any nonzero VESC fault code
→ immediate `stop()` + raise the fault flag. This is the real over-current
backstop the BTS7960 never had.

### D6 — VESC-unresponsive failsafe
Track the age of the last valid `COMM_GET_VALUES` reply. If it exceeds
`STEER_VESC_COMM_TIMEOUT_MS` (≈1000 ms, several poll cycles), set
`steerDriverOk_=false` and mirror the existing sensor-fault behavior
(`SteeringController.cpp:95-100`): stop the motor, cancel any move/jog, and
reject `setSteeringPercent()`/`jog()` until valid replies resume. With the VESC
disconnected the motor is simply never commanded (coasts); the wheel is held by
mechanical self-centering. Recovery clears the flag automatically. Startup:
`begin()` succeeds structurally even if the VESC is silent, but stays
`steerDriverOk_=false` until the first good reply, so a missing VESC never
blocks boot.

### D7 — Disposition of BTS7960 code
`BTS7960Controller.{h,cpp}` is **kept unchanged** — the brake actuator still
uses it (`src/main.cpp:124`, `VehicleController` brake path). Only
`SteeringController` stops depending on it. The freed GPIO17/18 and LEDC ch6/7
are left reserved in `Constants.h` with a comment, not repurposed.

## Risks / Trade-offs

- **Duty mode has no intrinsic current bound** → mitigated by VESC hardware
  current limit (D2) + firmware monitor (D5c). Both must be validated with the
  deliberate-stall bench test before vehicle use.
- **VESC comm timeout vs. hold.** VESC releases the motor (coast) after its own
  configured `timeout` if commands stop. The P-loop only commands while
  `isMoving_`; at target it sends `stop()` (coast). Steering is held
  mechanically, not electrically — acceptable (matches today's BTS7960 coast),
  but confirm the wheel does not drift under load on the bench.
- **`GET_VALUES` field offsets are FW-version specific** → decode only the four
  needed fields and bench-verify each against VESC Tool; pin the FW version in
  the config checklist.
- **Direction sign** (duty sign vs. wheel-right) must be verified on the bench;
  invert in `VescMotorDriver` or by swapping phase leads if reversed — same
  risk class as the current motor wiring.
- **VESC reset/brown-out mid-ride** → comm failsafe (D6) stops commanding and
  flags the driver down; ArduPilot loses steering authority. This is a genuine
  ride hazard and the reason on-bench + controlled first-drive validation is
  mandatory.
- **3.3 V UART levels** — Flipsky 75200 UART is 3.3 V logic; direct wiring is
  correct, but a miswire to a higher-voltage pad would damage the ESP32. Wiring
  task calls for a continuity/level check before power-up.

## Migration Plan

1. Configure the VESC in VESC Tool (DC mode, motor current limit, UART baud,
   app comm timeout); record the FW version.
2. Rewire: motor across two VESC phases; ESP32 GPIO4/5 ↔ VESC UART (3.3 V);
   verify levels before power-up.
3. Land firmware (abstraction, VESC driver, guard/latch fix, monitor,
   failsafe, telemetry) + web UI/i18n; flash + `uploadfs`.
4. Bench validation: sensor OK, calibration, jog/nudge, position moves,
   deliberate stall (verify latch + over-current stop + cooldown escape),
   VESC-disconnect failsafe.
5. Controlled first drive only after bench sign-off.

Rollback: revert firmware, re-wire the BTS7960 to GPIO17/18 (code retained),
re-flash. No NVS/calibration format change, so steering calibration is
preserved across rollback.

## Open Questions

- Exact `STEER_VESC_OVERCURRENT_A` vs. the VESC hardware limit — pick on the
  bench so the firmware trips slightly before the hardware limit.
- Whether to also surface VESC input voltage in telemetry (7s LiFePO4 sag could
  be a useful health signal) — included as an optional field; drop if the JSON
  budget (<1 KB) is tight.
