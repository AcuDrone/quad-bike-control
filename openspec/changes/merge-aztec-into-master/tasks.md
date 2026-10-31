## 0. Preconditions
- [x] 0.1 `master` working tree is clean except the untracked `openspec/changes/merge-aztec-into-master/` and `tree.txt` (`git status --short`).
- [x] 0.2 `aztec` is pushed (`git fetch && git status -sb aztec` shows no ahead/behind). Record its tip SHA (A). A = `e9fae18`.
- [x] 0.3 `git merge-base master aztec` = `b5a2153` = `git rev-parse master`, and `git log b5a2153..master` is empty. So the merge is a fast-forward (design D1). If master has moved, stop and re-plan as a true merge.
- [ ] 0.4 [USER] The Bosch LWS is mounted on the quad's steering shaft and wired to the MCP2515 CAN bus before 1.5.2 is flashed on the quad (design "Risks").

## 1. Fast-forward
- [x] 1.1 On master: `git merge --ff-only aztec`. Confirm `git rev-parse master` = A.
- [x] 1.2 `pio run`, `pio test -e native` and `pio run -t buildfs` pass on this exact aztec state. This is the baseline before any edit.

## 2. Commit G — vehicle-neutral merge fixes (built with the aztec `Constants.h`)
- [x] 2.1 `RelayController.h/.cpp` superset under `#if RELAY_WHEEL_LOCK_FITTED` (design D2):
  - `wheelLockOn_` and a real `getWheelLock()`;
  - fitted `setWheelLock()` (returns bool, clears test relays on lock);
  - `computeShadow()` wheel-lock term;
  - `allOff()` keeps the lock state;
  - `setRelayTest()` refuses while the lock is engaged;
  - vehicle-neutral static_asserts, plus the per-vehicle relay-lookup asserts under `#if`/`#else`;
  - Doxygen describes both assignments.
- [x] 2.2 `data/index.html`:
  - `relay_fn_q_1..8` en/uk keys, picked when `wheelLockFitted`;
  - `relayTestAllowed` also requires `!data.wheel_lock_on`;
  - `relay_test_help` / `relay_test_blocked` mention the wheel lock (en/uk);
  - gear-code card `id="card-gear-codes"`, hidden by default, shown only while telemetry carries `gear_codes` (design D3).
- [x] 2.3 `platformio.ini` and `src/main.cpp`: reword the UART0 / `CORE_DEBUG_LEVEL` / `usbOnlyLogVprintf` / `steeringVesc` comments for both vehicles (design D4). No code change.
- [x] 2.4 Docs wording for both vehicles:
  - `MAVLINK_SETUP.md`: `servo7_raw` row and `EFI_DIGITAL_FLAG_WHEEL_LOCK` row: "fitted on the quad (master); not fitted on aztec".
  - `WEB_PORTAL_SETUP.md`: version label example `1.5.2 · Control_v0`.
  - `openspec/project.md`, Purpose: add "On master (quad bike) the steering VESC stays on UART2 / X6 (GPIO4/5) and R63/R64 to the CH9121T stay fitted; the front-wheel lock is fitted".
- [x] 2.5 Build check on the aztec config: `pio run` with no new warnings, and `pio run -t buildfs`.
- [x] 2.6 Commit G (this change folder went in its own commit just before G, right after the fast-forward): "Merge fixes: wheel-lock-aware RelayController, telemetry-driven relay labels and gear-code card".

## 3. Commit Q — quad relay configuration
- [x] 3.1 `include/Constants.h`:
  - quad relay masks with `RELAY_MASK_WHEEL_LOCK 0b00001100`, `RELAY_MASK_RESERVED 0b00000011`, `RELAY_WHEEL_LOCK_FITTED 1` (design D2);
  - master's relay comment block, with aztec's relay-test paragraph;
  - `WHEEL_LOCK` / `EFI_DIGITAL_FLAG_WHEEL_LOCK` comments in master wording.
- [x] 3.2 `src/main.cpp`: the `relayController` declaration comment and the `digitalFlags` comment in master wording ("ignition, starter, lights and wheel lock").
- [x] 3.3 `pio run` passes (the static_asserts prove the masks).

## 4. Commit Q — quad gear sensing (design D3, option a)
- [x] 4.1 `git checkout b5a2153 -- include/BoardInputs.h src/BoardInputs.cpp include/TransmissionController.h src/TransmissionController.cpp`. This is a file restore, not a branch switch.
- [x] 4.2 `include/Constants.h`:
  - restore master's `BOARD_IN_BIT_GEAR_NEUTRAL/REVERSE/LOW/HIGH` block and its comments;
  - delete `BOARD_IN_BIT_GEAR_CODE_*`, `GEAR_CODE_*` and their static_asserts;
  - `TRANS_GEAR_READ_RETRY_COUNT` comment in master wording;
  - transmission servo comments in master wording ("HappyModel Super400 Plus @ 24V").
- [x] 4.3 `src/TelemetryManager.cpp`, `include/WebPortal.h`, `src/WebPortal.cpp`: delete the `gear_code` / `gear_sensed` / `gear_codes` / `gear_codes_custom` fields and emission, and update the JSON capacity comment.
- [x] 4.4 `include/VehicleController.h`, `src/VehicleController.cpp`: delete `processGearCodeCommand()`. `set_gear_code` / `reset_gear_codes` answer `false, "Gear codes not used on this vehicle"`.
- [x] 4.5 `grep -rn "GEAR_CODE\|gearCode\|gear_code" include src` returns nothing except the response string. `pio run` passes.

## 5. Commit Q — VESC link, docs, version
- [x] 5.1 `include/Constants.h` (design D4):
  - `PIN_VESC_TX GPIO_NUM_4`, `PIN_VESC_RX GPIO_NUM_5`, `VESC_UART_NUM 2`, with master's X6 / BSS138 comment;
  - the "over UART2" steering line;
  - the reserved-pin note (GPIO43/44 = UART0 → CH9121T, reserved; GPIO4/5 not free);
  - logging-section sentence in quad wording.
- [x] 5.2 Version and changelog:
  - `include/Constants.h`: `FIRMWARE_VERSION` "1.5.1" → "1.5.2".
  - `include/Changelog.h`: new first entry `{"version":"1.5.2","date":"<implementation date>"}`:
    - uk:
      - "Квадроцикл: кут керма тепер читається з датчика Bosch на шині CAN; AS5600 більше не використовується"
      - "Нова кнопка «Задати нуль датчика»; калібруються лише межі «ліво» та «право»"
      - "Без датчика кермо керується стіком як швидкістю (тільки MANUAL); перед рухом стік один раз у центр"
      - "Веб-панель — сервісна консоль за адресою 192.168.10.1 (було 192.168.4.1)"
      - "Перевірка реле у веб-панелі (лише при вимкненому запалюванні та відпущеному блокуванні колеса)"
      - "Калібрування газу приймає дзеркальний серво"
      - "Блокування переднього колеса, реле, датчики передач і VESC на X6 — без змін"
      - "Потрібне повторне калібрування керма: нуль датчика, потім «ліво» та «право»"
    - en:
      - "Quad: steering angle now comes from the Bosch sensor on the CAN bus; the AS5600 is no longer used"
      - "New \"Set sensor zero\" button; only the left and right limits are calibrated"
      - "Without the sensor the stick steers by rate (MANUAL only); centre the stick once before moving"
      - "Web portal is a service console at 192.168.10.1 (was 192.168.4.1)"
      - "Relay test in the web portal (ignition off and wheel lock released only)"
      - "Throttle calibration accepts a mirrored servo"
      - "Front-wheel lock, relays, gear sensors and the VESC on X6 are unchanged"
      - "Steering recalibration required: sensor zero, then left and right"
- [x] 5.3 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`: restore master's vehicle sections and keep aztec's LWS/CAN/I2C1 text:
  - vehicle line (§ header);
  - "Wired LAN" row;
  - GPIO 4/5/43/44/45 rows;
  - X1, X6, X8 connector rows and the transmission-servo supply note;
  - UART allocation table;
  - §5 gear table (one line per gear, bits 0/1/6/7);
  - §6 relay tables (paired ignition and wheel lock).
- [x] 5.4 Docs sweep: `grep -rn -i "aztec\|buggy\|UART0\|GPIO43\|X1\b\|not fitted\|3-bit\|gear code" *.md include src data platformio.ini`. Every hit must be either correct for both vehicles or explicitly labelled aztec. Fix the rest.
- [x] 5.5 Commit Q: "Quad configuration on master: paired relays with front-wheel lock, one-line-per-gear sensing, VESC on UART2/X6, firmware 1.5.2".

## 6. Build, test, validate (on master after Q)
- [x] 6.1 `pio run` builds without new warnings. Check the size against the 1.5.1 baseline from 1.2.
- [x] 6.2 `pio test -e native` passes (`test_lws305`, `test_steer_rate`, `test_steer_sensor_dir`).
- [x] 6.3 `pio run -t buildfs`.
- [x] 6.4 `openspec validate --strict` for every folder in `openspec/changes/` (the twelve aztec folders and `merge-aztec-into-master`), plus `openspec validate --specs --strict`.

## 7. Aztec bookkeeping [USER approval] (design D6)
- [ ] 7.1 On aztec: `git merge --ff-only <G>`. Then `pio run` with the aztec config (done 2026-10-31: G = `ca0d900`, aztec build passes; bench part pending), and on the bench: wheel-lock toggle disabled "not fitted", aztec relay labels, gear-code card visible.
- [x] 7.2 On aztec: `git merge -s ours <Q> -m "Record quad configuration (master) as merged; aztec keeps its vehicle config"`. Check `git diff <G> aztec` is empty.
- [ ] 7.3 Push master and aztec.

## 8. Bench verification on the quad [USER/MANUAL] (after the LWS is fitted)
- [ ] 8.1 [USER/MANUAL] Boot log: I2C2 scan shows 0x1A 0x1C 0x1F, no `Wire` start, and `[TRANS] Gear sensors bound to opto input bits N:0 R:1 L:6 H:7`. Version label reads `1.5.2 · Control_v0`, and "What's new" shows 1.5.2 first.
- [ ] 8.2 [USER/MANUAL] Web at `http://192.168.10.1` (SSID `QuadBike-Control`). The sidebar shows "Service console / QuadBike". The "Gear Sensor Codes" card is absent, and there is no sensor-code line under the current gear.
- [ ] 8.3 [USER/MANUAL] CAN: with the LWS unplugged and the ECU on, confirm that no 0x305 / 0x7C0 frames come from the quad ECU (`steer_lws_seen` stays false). Plug in the LWS: `steer_lws_fps` ≈ 100, and the OBD data still updates.
- [ ] 8.4 [USER/MANUAL] Relays:
  - ignition ACC/IGNITION energizes Relay_1 + Relay_2 together;
  - START cranks via Relay_3;
  - the light is Relay_4;
  - MAVLink channel 7 = 2000 → wheel lock LOCKED (Relay_5 + Relay_6, `wheel_lock_on` true, `EFI_STATUS` bit0 = 1), 1000 → UNLOCKED;
  - the web wheel-lock toggle works;
  - `allOff` / failsafe leaves the lock in its last state.
- [ ] 8.5 [USER/MANUAL] Relay test, with ignition OFF, light off and lock released:
  - each of Relay_1..8 toggles alone, with the quad labels;
  - Relay_3 auto-offs after 2 s, the others after 30 s;
  - with the wheel lock engaged, test ON is refused (UI disabled and firmware message);
  - an ignition, light or lock command clears active test relays.
- [ ] 8.6 [USER/MANUAL] Gears: sweep the lever R/N/L/H. Each position reads its gear, and between positions or on a lost opto feed the gear reads UNKNOWN. Servo gear changes confirm arrival as on 1.1.0. A `set_gear_code` sent from the browser console is answered "Gear codes not used on this vehicle".
- [ ] 8.7 [USER/MANUAL] VESC on X6 (UART2):
  - `steer_driver_ok` is true, and voltage, current and temperature update;
  - Calibration Jog and Nudge move the wheels both ways;
  - nothing on X1 / UART0;
  - no log text appears on the LAN chip side (if the CH9121T is used).
- [ ] 8.8 [USER/MANUAL] LWS calibration:
  - Set sensor zero with the wheels straight (confirmed, angle ≈ 0);
  - capture the left and right limits (the direction is inferred, and Jog ▶ increases the angle);
  - percent steering from the web slider and the stick tracks within tolerance;
  - boot centring works after a power cycle.

  Note the quad's lock-to-lock angle against `STEER_CAL_MIN_SPAN` (30°) and `STEER_LWS_SAFE_RANGE` (550°).
- [ ] 8.9 [USER/MANUAL] Rate mode: in MANUAL with the stick centred, unplug the LWS. `steer_mode` = rate, the "Увага" line is shown and `STEER_MODE` = 1. Centre the stick to arm it. Deflection = rotation, release = stop within 500 ms. In HOLD the mode is blocked. Reconnect → position.
- [ ] 8.10 [USER/MANUAL] MAVLink channels unchanged: steering, throttle, brake, transmission (ch 3), ignition (ch 4), light (ch 6), wheel lock (ch 7). Mission Planner shows the `STEER_*` named values and the `STEER_MODE` int.
- [ ] 8.11 [USER/MANUAL] Throttle: the existing idle/full calibration is still valid. Full throttle and idle are as on 1.1.0.
