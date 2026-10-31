## 0. Preconditions
- [ ] 0.1 `update-aztec-vesc-uart0`, `replace-aztec-steering-sensor-with-bosch-lws`, `redesign-web-ui-service-console` and `add-aztec-steering-rate-mode` are implemented on `aztec` (firmware 1.5.0). This change is archived AFTER all of them: its MODIFIED blocks quote their deltas.
- [ ] 0.2 **HARD GATE:** `move-can-rx-to-interrupt` (firmware 1.6.0) is implemented and is archived immediately before this change. It provides MCP2515 INT on GPIO38, the `can_rx` reader task, a receive queue drained by `update()`, one SPI mutex and the non-blocking `queueTx()` (design D6). Do not start sections 2–6 before it. In polled mode, 2 of 3 back-to-back VESC status frames are lost. This change's `CAN Communication Error Handling` delta is already re-based on that change's text. Re-check it if that change's delta was edited before archiving.
- [ ] 0.3 The dispatch hook from `move-can-rx-to-interrupt` is `CANController::routeRxFrame()`, the single consumer of the receive queue, called from `pumpRxQueue()` at the top of `update()` in every mode. Its first step is `dispatchSteeringFrame()`. Confirm the names in the implemented code, and adapt tasks 2.3–2.5 to it if they differ.

## 1. Pure logic (host-testable)
- [ ] 1.1 Make `include/VescProtocol.h` header-only and Arduino-free (`<stdint.h>`):
  - inline `crc16()` (CRC16-XMODEM, unchanged algorithm);
  - `CommId` with `COMM_GET_VALUES_SELECTIVE = 50` added;
  - big-endian get/put helpers.
  - Remove `frameShort()`, the UART `encodeSetDuty()`, `Values` and `decodeGetValues()`.
  - Delete `src/VescProtocol.cpp`.
- [ ] 1.2 Create `include/VescCan.h` (header-only, no Arduino and no `Constants.h`), following design D2–D4:
  - packet IDs (0, 5, 6, 7, 8, 9, 16, 27);
  - `makeEid` / `splitEid`;
  - `dutyE5FromSpeed` (integer, clamp ±255, 255 → 100000) and `encodeSetDuty`;
  - `decodeStatus1/4/5` with length checks;
  - `buildFaultPoll(senderId, out[7])` → `[sender, 0x00, 0x32, 00 00 80 00]`;
  - `parseSelectiveFault` (command byte 50, echoed mask `0x00008000`, length ≥ 6);
  - `RxReassembler`:
    - the single-frame `PROCESS_SHORT_BUFFER` reply (expected case);
    - `FILL_RX_BUFFER` index/data into a 64-byte buffer;
    - `PROCESS_RX_BUFFER` sender / len / CRC check;
    - reset on a new request;
  - `DutyTxScheduler`:
    - change → send if ≥ min gap;
    - duty 0 sent at once;
    - keep-alive of a non-zero duty only;
    - retry on failure;
    - one duty 0 queued at start.
- [ ] 1.3 Create `include/Mcp2515Filter.h` (pure). It models mask/filter acceptance from the `mcp_can` `init_Mask`/`init_Filt` argument encoding (`ext` 0/1), with EXIDE matching and data-byte filtering of standard frames (design D5).
- [ ] 1.4 Add `test/test_vesc_can/test_main.cpp` (Unity), mirroring the needed constants like `test_steer_rate` does. It covers:
  - eid compose/split: `0x0901`, `0x0802`, `0x00000001`;
  - duty: 255 → 100000, −255 → −100000, 140 → 54901, 180 → 70588, 0 → 0, ±300 clamped; byte order of the SET_DUTY payload;
  - STATUS_1/4/5 decoding of reference frames, including negative current and temperature; short frames rejected;
  - the fault-poll request bytes;
  - the single-frame reply `[01 01 32 00 00 80 00 07]` → fault 7; a wrong mask or command byte is rejected;
  - multi-frame reassembly: good CRC accepted; a bad CRC, `len` > received bytes, or a wrong sender rejected; out-of-order FILL indices still assemble;
  - scheduler:
    - first non-zero duty sent;
    - same duty within 200 ms not re-sent, and re-sent at 200 ms;
    - a change within 10 ms deferred and then sent with the newest value;
    - duty 0 sent immediately even within 10 ms, and never kept alive;
    - a failed send retried;
  - filter model, with the chosen values (`0x07F80000`, `0x07E80000`, ext `0x00000000`, `0x07FF0000`, `0x03050000`):
    - accept std `0x305` (any data), `0x7E8`–`0x7EF`, and ext `0x0901`/`0x1001`/`0x1B01`/`0x0802`/`0x0502`/`0x0702`;
    - reject std `0x301`/`0x7DF`/`0x7C0` and ext `0x18DAF110`;
    - RXB1 rejects every ext frame;
    - show that the rejected RXB1 mask `0x1FFF00FF` drops a `0x305` frame whose data byte 1 ≠ the filter's byte.
- [ ] 1.5 `platformio.ini` `[env:native]`: add `test_vesc_can` to the multi-line `test_filter` (one suite per indented line) and update the comment. `build_src_filter` stays `+<LwsSensor.cpp>`, because the new headers are header-only.

## 2. CAN controller (`CANController.h/.cpp`)
- [ ] 2.1 `applyRxFilters()`: set filter 1 to `init_Filt(1, 1, CAN_RX_VESC_FILTER_REG)` (ext ID 0). Leave mask 0, filter 0, mask 1 and filters 2–5 unchanged. Update the comment block (encoding, data-byte filtering, why RXB0 is used) and the debug line. Check that it is still re-applied after every reset.
- [ ] 2.2 Add `attachVescSink(VescCanDriver*)` / `hasVescSink()`.
- [ ] 2.3 Rename `dispatchSteeringFrame()` to `dispatchSharedFrame()`:
  - std `0x305` → LWS (as now);
  - ext frames (ID flag bit 31; remote frames ignored) with `eid` ≤ `0xFFFF` → `VescCanDriver::onFrame()` when the low byte is `STEER_VESC_CAN_ID` with cmd 9/16/27, or `CAN_CONTROLLER_VESC_SENDER_ID` with cmd 5/7/8;
  - other ext frames are counted and dropped, and not logged as unmatched OBD frames.
  - Call it first in `routeRxFrame()`, the single queue consumer from 0.3 (there are no other receive paths after `move-can-rx-to-interrupt`).
- [ ] 2.4 Add `bool sendVescFrame(uint8_t cmd, const uint8_t* data, uint8_t len)`:
  - whitelist cmd ∈ {`SET_DUTY`, `PROCESS_SHORT_BUFFER`};
  - ID `(cmd << 8) | STEER_VESC_CAN_ID`, `ext = 1`, `len ≤ 8`;
  - false when not initialized;
  - allowed in every OBD / probe / zero-set state;
  - failures go to its own counter, not to `consecutiveSendFailures_`;
  - concurrency-safe per the `move-can-rx-to-interrupt` SPI rule.
  - Document that it is the only extended-ID transmitter.
- [ ] 2.5 Update the class Doxygen header: the bus now carries the VESC, and there are three users of one bus.

## 3. VESC CAN driver
- [ ] 3.1 Add `include/VescCanDriver.h` and `src/VescCanDriver.cpp` (`VescCanDriver : IMotorDriver`), following design D8:
  - constructor `(vescId, ownId)`, `attachCan()`, `onFrame()`;
  - `setSpeed()` / `stop()` through `DutyTxScheduler`;
  - `poll()` (pending / keep-alive / fault poll every `STEER_VESC_FAULT_POLL_MS` while `driverOk`, reply timeout `STEER_VESC_FAULT_REPLY_MS` → stale, the 5 s `[VESC]` rate/counter debug line);
  - `driverOk()` = STATUS_1 within `STEER_VESC_COMM_TIMEOUT_MS`;
  - accessors, including `faultStale()`.
- [ ] 3.2 `IMotorDriver.h`: add an optional `virtual bool faultStale() const { return false; }` and update the comments (VESC over CAN).
- [ ] 3.3 Delete `include/VescMotorDriver.h` and `src/VescMotorDriver.cpp`. Grep that no reference remains.
- [ ] 3.4 `SteeringController.h/.cpp`: comment updates only (UART → CAN, "telemetry poll" → status frames). Add `bool isVescFaultStale() const`. Confirm by reading that the control logic is untouched.

## 4. Wiring (`main.cpp`, `VehicleController`)
- [ ] 4.1 `main.cpp`:
  - construct `VescCanDriver steeringVesc(STEER_VESC_CAN_ID, CAN_CONTROLLER_VESC_SENDER_ID)` before the steering controller;
  - remove `steeringVesc.begin(PIN_VESC_RX, PIN_VESC_TX, VESC_UART_BAUD)` and the UART comments;
  - keep `uartSetDebug(NULL)` / the `esp_log` redirect with updated comments (X1 kept quiet, no longer a VESC link);
  - call `vehicleController.attachSteeringVesc(&steeringVesc)` before `initCAN()`.
- [ ] 4.2 `VehicleController`: add `attachSteeringVesc(VescCanDriver*)`. In `initCAN()`, call `canController_.attachVescSink(vesc)` and `vesc->attachCan(&canController_)`, even if CAN init failed.

## 5. Constants (`include/Constants.h`)
- [ ] 5.1 Remove `PIN_VESC_TX`, `PIN_VESC_RX`, `VESC_UART_NUM`, `VESC_UART_BAUD` and `STEER_VESC_TELEM_MS`.
- [ ] 5.2 Add:
  - `STEER_VESC_CAN_ID 1`, with a comment: never 0; set in VESC Tool; 500K; CAN mode VESC;
  - `CAN_CONTROLLER_VESC_SENDER_ID 2`;
  - `STEER_VESC_CAN_KEEPALIVE_MS 200`;
  - `STEER_VESC_CAN_MIN_TX_MS 10`;
  - `STEER_VESC_FAULT_POLL_MS 5000`;
  - `STEER_VESC_FAULT_REPLY_MS 500`;
  - `STEER_VESC_RX_LOG_MS 5000`;
  - `CAN_RX_VESC_FILTER_REG 0x00000000`, marked as an ext-flag filter.
  - Re-comment `STEER_VESC_COMM_TIMEOUT_MS` (no STATUS_1 for this long).
- [ ] 5.3 Rewrite the comments:
  - the pin header (GPIO43/44 / X1 free, R63/R64 stay removed, CH9121T not restored);
  - the steering actuator block;
  - the serial-debug block;
  - the MCP2515 acceptance block (RXB0 = OBD + VESC ext, RXB1 = LWS).

## 6. Telemetry and web
- [ ] 6.1 `TelemetryManager.h/.cpp`: add `steer_vesc_fault_stale` (from `isVescFaultStale()`). The existing fields keep their names. `WebPortal.cpp`: emit `doc["steer_vesc_fault_stale"]`, and re-check the JSON capacity comment and size.
- [ ] 6.2 `data/index.html` "Кермо: стан":
  - the driver tile label becomes "VESC (CAN)" (`sd_driver`, uk + en);
  - while `steer_vesc_fault_stale` is true and the driver is OK, append a muted "(код не оновлено)" / "(fault code stale)" with new i18n keys in both dictionaries;
  - update the card help text (data source: VESC CAN status frames, 5 Hz; fault code every 5 s).
- [ ] 6.3 Confirm that "Увага", the `steer_vin` tile, `steer_vin_low` and the MAVLink named floats need no change. Update the comment in `MavlinkInterface` / `main.cpp` that says "VESC UART".

## 7. Docs
- [ ] 7.1 `openspec/project.md`: replace the UART0/X1 wiring paragraph. The steering VESC is on the MCP2515 CAN bus (ID 1, 500 kbit/s) with the ECU and the LWS. UART0/X1 is free, R63/R64 stay removed, and X6 is reserved for a 4th BTS7960.
- [ ] 7.2 `GPIO_PINOUT_CUSTOM_BOARD_S3.md`:
  - GPIO43/44 rows → free;
  - X1 → Prog header (free; UART flashing possible again only if nothing is attached);
  - UART table without the VESC;
  - the CAN section lists the VESC (ID 1, ext frames) and the termination check;
  - the warnings and the migration table: `PIN_VESC_*` removed.
- [ ] 7.3 `MAVLINK_SETUP.md`: in the steering-VESC section and the source tables, "VESC UART" → "VESC CAN (status frames)", and the refresh rate 3.3 Hz → 5 Hz (`STEER_VESC_TELEM_MS` reference removed). The fault-code note: polled every 5 s over CAN.
- [ ] 7.4 `WEB_PORTAL_SETUP.md`: the driver tile and the new stale-fault hint.
- [ ] 7.5 `platformio.ini`: rewrite the build-flag comment. UART0 is no longer the VESC link, and logging stays off it to keep X1 quiet.

## 8. Version and changelog
- [ ] 8.1 `FIRMWARE_VERSION` "1.6.0" → **"1.7.0"** in `include/Constants.h` (minor: wiring change).
- [ ] 8.2 `include/Changelog.h`: add the 1.7.0 entry (date of implementation), newest first:
  - uk: "VESC керма тепер по CAN (ID 1, 500 кбіт/с), роз'єм X1 (UART0) вільний"; "У VESC Tool виставити VESC ID = 1, CAN status 1/4/5 на 5 Гц"; "Код помилки VESC оновлюється раз на 5 с"; "Повторне калібрування не потрібне"
  - en: "Steering VESC now on CAN (ID 1, 500 kbit/s), connector X1 (UART0) is free"; "In VESC Tool set VESC ID = 1, CAN status 1/4/5 at 5 Hz"; "VESC fault code refreshes every 5 s"; "No recalibration required"

## 9. Build and test
- [ ] 9.1 `pio run` builds with no warnings from the new or changed files.
- [ ] 9.2 `pio test -e native`: `test_lws305`, `test_steer_rate` and `test_vesc_can` all pass.
- [ ] 9.3 `pio run -t buildfs` succeeds.
- [ ] 9.4 Grep: no `VescMotorDriver`, `PIN_VESC_`, `VESC_UART_` or `STEER_VESC_TELEM_MS` remains in `src/`, `include/` or `test/`. `sendMsgBuf(` with `ext = 1` appears only in `sendVescFrame()`.

## 10. Validation
- [ ] 10.1 `openspec validate move-aztec-vesc-to-can --strict` passes.

## 11. VESC and wiring setup [USER/MANUAL]
- [ ] 11.1 VESC Tool (USB) → App Settings → General:
  - *Controller ID* = 1;
  - *CAN Baud Rate* = 500K;
  - *CAN Mode* = VESC;
  - *CAN Status Rate 1* = 5 Hz with STATUS_1, STATUS_4 and STATUS_5 only; *Rate 2* = 0;
  - *Timeout* = 1000 ms, *Timeout Brake Current* = 0 A.
  - Write the app config.
- [ ] 11.2 Disconnect the VESC UART from X1. Wire the VESC CAN port's CAN_H/CAN_L/GND to the MCP2515 bus with a twisted pair. With power off, measure CAN_H–CAN_L ≈ 60 Ω (exactly two 120 Ω terminators on the whole bus).

## 12. Bench verification [USER/MANUAL]
- [ ] 12.1 Record the VESC firmware version (VESC Tool). It must be ≥ 6.0 for per-message status selection; otherwise see design "Risks".
- [ ] 12.2 After power-up, `steer_driver_ok` becomes true within 1 s of the VESC booting. `steer_motor_current`, `steer_fet_temp` and `steer_vin` are live.
- [ ] 12.3 Jog, nudge and percent moves (web slider, MAVLink) work as on 1.6.0. The minimum working duty is unchanged (~140; no movement at 70). Rate mode with the sensor unplugged still steers from the stick, and `STEER_MODE` reporting is unchanged.
- [ ] 12.4 With the ECU on and the LWS streaming, the `[VESC]` debug line shows:
  - STATUS_1, STATUS_4 and STATUS_5 at 5 Hz ± 0.5;
  - MCP2515 RX overflow = 0;
  - LWS checksum, counter and miss counters stay 0 (or no growth beyond the VESC-unplugged baseline), and no LWS `timeout`.
- [ ] 12.5 OBD-II polling still works: RPM, coolant and module voltage update, the ECU probe completes, and the LWS zero-set still works.
- [ ] 12.6 Force a VESC fault (e.g. disconnect the motor and command a jog, or lower the over-current limit). `steer_vesc_fault` shows the code within 5 s, the steering stops, and the stall-latch path is taken. Clear the fault and see the code return to 0 within 5 s. This confirms the selective mask bit 15 and the single-frame reply (cmd 8 to ID 2). Note in the debug line whether any multi-frame reply was seen.
- [ ] 12.7 `steer_vesc_fault_stale` stays false in normal operation.
- [ ] 12.8 `steer_vin` matches a multimeter on the VESC input within ±0.3 V. The low-voltage warning still triggers below `STEER_VIN_LOW_V`.
- [ ] 12.9 Keep-alive: hold a jog for > 5 s and check that the motor never drops out (no VESC timeout). Unplug the ESP32 CAN side mid-jog: the VESC stops by itself within ~1 s.
- [ ] 12.10 Steering-mode switching (position ↔ rate ↔ blocked) behaves as on 1.6.0.
- [ ] 12.11 Unplug the VESC CAN cable while idle: `steer_driver_ok` goes false within 1 s, jog/nudge/percent are refused, and the web "Увага" shows the VESC down. Re-plug: it recovers automatically.
- [ ] 12.12 X1 is unconnected and no log text appears on GPIO43 after boot (scope, optional).
