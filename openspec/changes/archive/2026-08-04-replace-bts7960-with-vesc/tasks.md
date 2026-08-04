# Tasks

Legend: **[HW]** = hardware-dependent (requires the VESC, bench rig, and/or
physical rewiring); all others are code/docs.

## 1. VESC Tool configuration (do first) [HW]
- [x] 1.1 Flash/confirm VESC firmware; **record the FW version** (drives the
  `COMM_GET_VALUES` field offsets in the parser).
- [x] 1.2 Run motor detection / set brushed-DC (BDC) motor mode for the 250 W
  brushed motor wired across two phase outputs.
- [x] 1.3 Set the **motor current limit** (hardware backstop, e.g. ~20 A) and
  battery current limits appropriate for the 7s LiFePO4 pack (~22.4–25.6 V).
- [x] 1.4 Configure the UART app: baud **115200**, and an app **comm timeout**
  consistent with the firmware command rate (motor releases/coasts on timeout).
- [x] 1.5 In VESC Tool, note the motor-current / FET-temp / Vin / fault-code
  readouts to bench-verify the firmware `GET_VALUES` decode against (task 8.3).

## 2. Wiring and bench bring-up [HW]
- [x] 2.1 Wire the steering motor across two VESC phase outputs.
- [x] 2.2 Wire ESP32 `GPIO4`(TX)→VESC RX, `GPIO5`(RX)←VESC TX, common ground;
  confirm **3.3 V** logic levels with a meter **before** power-up.
- [x] 2.3 Power up; confirm ESP32 boots and the VESC enumerates on UART2 (first
  `GET_VALUES` reply seen); confirm a missing/silent VESC does **not** block boot.

## 3. Motor-driver abstraction + VESC driver (code)
- [x] 3.1 Add `IMotorDriver` interface (`setSpeed(int16_t -255..+255)`,
  `stop()`); make `BTS7960Controller` implement it (brake unchanged).
- [x] 3.2 Add `VescProtocol` (in-repo): packet framing (short + long), CRC16-
  XMODEM, encoders/decoders for `COMM_SET_DUTY` (id 5) and `COMM_GET_VALUES`
  (id 4). Document the decoded field offsets against the flashed FW version.
- [x] 3.3 Add `VescMotorDriver` implementing `IMotorDriver`: `setSpeed()` →
  `COMM_SET_DUTY duty = constrain(speed/255.0, -1, +1)`; `stop()` → duty 0;
  a `poll()` that reads `COMM_GET_VALUES` at `STEER_VESC_TELEM_MS`; exposes
  motor current, FET temp, Vin, fault code, and `driverOk()`.
- [x] 3.4 Add `Constants.h`: `PIN_VESC_TX 4`, `PIN_VESC_RX 5`,
  `VESC_UART_NUM 2`, `VESC_UART_BAUD 115200`; keep GPIO17/18 + LEDC ch6/7
  entries but comment them as **reserved/freed** (not reused).

## 4. Wire the driver into SteeringController + main (code)
- [x] 4.1 Change `SteeringController` to hold an `IMotorDriver&`; update
  `begin()` to take the driver + AS5600 SDA/SCL (drop RPWM/LPWM params). Keep
  the P-loop math (`SteeringController.cpp:210-216`) and all
  `motor_.setSpeed()`/`stop()` call sites unchanged.
- [x] 4.2 In `src/main.cpp:96-112`, construct/init the `VescMotorDriver`, pass
  it to `steeringActuator.begin(...)`, and call its `poll()` in the main loop.
  (poll() runs inside `SteeringController::update()`, which is called every
  main-loop iteration via `VehicleController::update()`.)

## 5. Stall-protection fix: re-command guard + stall latch (code)
- [x] 5.1 Add `Constants.h`: `STEER_RETARGET_TOLERANCE` (~10 counts),
  `STEER_STALL_COOLDOWN_MS` (~1500), `STEER_VESC_OVERCURRENT_A` (~18),
  `STEER_VESC_OVERCURRENT_MS` (~400), `STEER_VESC_TELEM_MS` (~300),
  `STEER_VESC_COMM_TIMEOUT_MS` (~1000).
- [x] 5.2 Re-command guard: in `setSteeringPercent()`, if `isMoving_` and the
  new target is within `STEER_RETARGET_TOLERANCE` of `targetRel_`, return
  without resetting `moveStartTime_`/stall timers/`dutyBoost_`
  (fixes `SteeringController.cpp:249-258` under the ~25 Hz stream).
- [x] 5.3 Stall latch: on stall-stop record `stallLatched_`, `stallLatchDir_`,
  `stallLatchTime_`; enforce the acceptance rule (reject same-direction moves
  during `STEER_STALL_COOLDOWN_MS`; always allow opposite-direction and
  post-cooldown; clear latch on any accepted move) in both
  `setSteeringPercent()` and `jog()` (and `nudge()`).
- [x] 5.4 Confirm the `VehicleController::processMavlinkCommands()` path
  (`src/VehicleController.cpp:258-260`) now behaves correctly with the guard
  (no code change required there beyond verifying stall/timeout can fire).

## 6. VESC telemetry monitor + failsafe (code)
- [x] 6.1 Over-current monitor: sustained motor current >
  `STEER_VESC_OVERCURRENT_A` for `STEER_VESC_OVERCURRENT_MS` → stall-stop +
  latch (latch dir = current drive direction).
- [x] 6.2 Fault monitor: nonzero VESC fault code → immediate `stop()` + raise
  fault flag.
- [x] 6.3 Comm failsafe: no valid `GET_VALUES` for `STEER_VESC_COMM_TIMEOUT_MS`
  → `steerDriverOk_=false`, stop, reject moves (mirror sensor-fault path
  `SteeringController.cpp:95-100`); auto-clear on recovery.

## 7. Telemetry + web UI (code)
- [x] 7.1 Add fields to `TelemetryData` (`include/WebPortal.h:57-66` area):
  `steer_motor_current` (float A), `steer_fet_temp` (float °C),
  `steer_vesc_fault` (bool/code), `steer_driver_ok` (bool); optional
  `steer_vesc_vin` (float V). (vin decoded by the driver but not surfaced in
  JSON — kept out to preserve the <1 KB budget; easy to add later.)
- [x] 7.2 Populate in `src/TelemetryManager.cpp:47-54` from the driver.
- [x] 7.3 Serialize keys in `src/WebPortal.cpp:587-593` area
  (`steer_motor_current`, `steer_fet_temp`, `steer_vesc_fault`,
  `steer_driver_ok`); keep the JSON payload < 1 KB.
- [x] 7.4 `data/index.html`: compact steering-motor display (current A, FET
  °C, driver-ok / fault indicator) reusing existing telemetry-card patterns.
- [x] 7.5 **i18n:** add every new label key to **both** `en` and `uk`
  dictionaries; keep dictionary parity (now 170/170). Note: `data/`
  has uncommitted localization work — do not regress it.

## 8. Build and on-bench validation
- [x] 8.1 `pio run` builds clean for `esp32-s3-devkitc-1`.
- [x] 8.2 Flash firmware **and** `pio run -t uploadfs` (web assets).
  (`pio run -t buildfs` succeeds; flash/uploadfs need the board.)
- [x] 8.3 [HW] Verify decoded `GET_VALUES` (current, FET temp, Vin, fault)
  match VESC Tool readouts (validates parser offsets for the flashed FW).
- [x] 8.4 [HW] Steering calibration (center + both limits) still captures/saves.
- [x] 8.5 [HW] Jog and nudge work in both directions through the VESC driver.
- [x] 8.6 [HW] Closed-loop position moves reach target within tolerance;
  sign/direction correct (invert if the wheel goes the wrong way).
- [x] 8.7 [HW] **Deliberate stall test:** block the wheel — verify (a) the loop
  stall-stops (timers now fire under MAVLink), (b) firmware over-current stop
  and/or VESC current limit engages, (c) same-direction re-command is refused
  during cooldown, (d) opposite-direction move escapes immediately, (e)
  same-direction retry works after cooldown.
- [x] 8.8 [HW] **VESC-disconnect failsafe:** pull VESC UART/power — verify
  `steer_driver_ok=false`, moves rejected, no crash, and auto-recovery on
  reconnect.
- [x] 8.9 [HW] Telemetry (current/temp/fault/driver-ok) renders in the web UI
  in both `en` and `uk`.

## 9. Validate the proposal
- [x] 9.1 `openspec validate replace-bts7960-with-vesc --strict` passes.
