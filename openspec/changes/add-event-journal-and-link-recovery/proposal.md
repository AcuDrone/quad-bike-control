# Change: Persistent event journal and link recovery fixes (VESC UART, CAN LWS, I2C2)

## Why
Field faults on the aztec buggy (steering VESC "link lost" for minutes, the Bosch LWS stream going
silent while the ECU still answers OBD, the relay board on X15 dropping out) leave no trace once the
USB console is unplugged: every diagnostic today is a `Debug::printf` line that exists only while a
laptop is attached. Three of those faults also have a concrete firmware cause that never self-heals:

- **VESC UART parser** (`src/VescMotorDriver.cpp:65-131`): no inter-byte timeout. After one lost or
  corrupted byte the state machine keeps consuming the *next* reply's header as payload, and because
  the 0x03 stop byte is also the long-frame start byte and the replies repeat every 300 ms with almost
  identical content, the misalignment can repeat reply after reply ("phase lock") for minutes. The
  steering sees `driverOk() == false` (no valid reply for 1 s) and stops.
- **CAN**: the only re-init trigger is "no OBD reply for 15 s" (`src/CANController.cpp:236-252`). On
  aztec the Delphi MT05 ECU always answers OBD, so a silent LWS `0x305` path never triggers a re-init.
  `recoverFromBusOff()` (`:592-609`) ignores the results of `begin()` and `applyRxFilters()`.
- **I2C2** (`Wire1`, GPIO48/47): nothing ever clocks out a slave that holds SDA low, and no
  `setTimeOut()` is set, so a stuck bus both stays stuck and can stretch the loop by hundreds of ms
  (relay write retries × the core's 50 ms default timeout).

## What Changes
**A. Event journal (new capability `event-journal`, new module `EventJournal`)**
- Two plain-text files on LittleFS: `/events.log` (active) and `/events.1` (rotated), 128 KB each.
  When the active file reaches the limit: delete `.1`, rename active → `.1`, start a new active file.
  Survives application OTA; erased by `pio run -t uploadfs` and by a web filesystem (LittleFS) OTA.
- Boot counter in NVS (namespace `journal`, key `boot`), incremented once per boot.
- `RTC_NOINIT` ring of the last 16 not-yet-flushed events, replayed into the file at the next boot
  behind a marker line (survives panic / watchdog / software reset, not power loss).
- RAM write buffer flushed to flash at most once per second, and immediately before a controlled
  reboot (OTA). While the steering motor is being driven the flush is deferred (at most 10 s), because
  a flash sector erase stalls both cores for 30–50 ms. Dedupe: the same (source, event) pair is written at most once per 2 s; the number of
  suppressed repeats is noted (`sup=N`) on the next written line of that pair.
- Line format (ASCII, one event per line, ≤ 120 bytes):
  `<boot> <uptime_s.mmm> <SRC> <LVL> <event> k=v k=v ...`, levels `I`/`W`/`E`, sources
  `SYS SRC STEER VESC CAN I2C RELAY SNAP RTC`, plus a `# boot=N fw=... reset=...` comment header per boot.
- Events: boot (reset reason, fw, heap, previous-session min heap); RTC replay marker; VESC
  link_lost / link_up with a parser snapshot, fault-code change, `link_down_snap` every 10 s while
  down, parser resync; LWS health change (fault token, fps); CAN bus-off and re-init (reason, result,
  ms to first frame); steering mode change with reason; stall-stop; USB diag lock acquire/release;
  I2C relay write fault/recovery (error code, address), board-input read failure, bus unlock;
  input-source change; fail-safe entry/exit; relay board error; loop overrun > 50 ms (with phase);
  low heap (< 40 KB, 8 KB hysteresis); periodic `SNAP` every 60 s.
- Access: web card «Журнал подій» / "Event journal" in the System section (last 100 lines, source
  filter, Download, Clear); `GET /api/events?tail=N` (text/plain), `GET /api/events/download`
  (`.1` then active, one text stream), `POST /api/events/clear`.

**B. Recovery fixes (each logs its trigger to the journal)**
1. **VESC parser**: reset to `RX_START` when more than `VESC_RX_GAP_MS` (5 ms) passed since the last
   received byte while mid-frame; on the link-lost transition flush UART RX and reset the parser;
   counters (bytes, good, dropped-CRC, dropped-stop, resync). Parser extracted to a header-only
   `include/VescRxParser.h` with a native test that reproduces the stuck state and its recovery.
2. **CAN**: second watchdog — no LWS `0x305` frame for > 1 s while OBD is answering → MCP2515
   re-init, fixed spacing ≥ 5 s (no back-off). Runs only while the ignition is ACC or ON (the LWS is
   powered from the ACC line) and arms only after the first LWS frame since the ignition was switched on. `recoverFromBusOff()` checks `begin()`, `applyRxFilters()` and
   `setMode()`, retries a failed re-init (bounded, across `update()` calls) and logs the outcome.
3. **I2C2 bus unlock**: a single main-owned recovery routine (`I2cBusRecovery`) used by the relay
   recovery path and by the board-input reader after N consecutive failures: if SDA reads low,
   bit-bang up to 9 SCL clocks + STOP, then `Wire1.end()/begin()`, then the driver's own re-init.
   `Wire1.setTimeOut(I2C_TIMEOUT_MS)` (20 ms) at boot and after every re-init.
   **This reverses the current rule "no driver and no coordinator SHALL call `end()`/`begin()` on
   `Wire1` after `setup()`"** (gpio-expander), for this one coordinator only.
4. **Loop timing**: per-iteration duration and the slowest phase, max per 60 s snapshot window.
- **Web telemetry** gains `vesc_resync`, `can_reinit`, `i2c_unlock` (session totals) and
  `journal_bytes`.
- **Not changed**: steering control law, relay functions, throttle, gear logic, MAVLink (no new
  messages or fields).
- Firmware version 1.5.25 → 1.5.26, `include/Changelog.h` entry (uk + en). Docs:
  `GPIO_PINOUT_CUSTOM_BOARD_S3.md` (I2C2 unlock / timeout notes), `WEB_PORTAL_SETUP.md` (journal
  card, API, uploadfs caveat).

## Impact
- Affected specs: `event-journal` (new), `vehicle-actuators` (VESC driver lives here, not in
  `vehicle-systems`), `can-controller`, `gpio-expander`, `web-control`, `web-telemetry`,
  `vehicle-systems` (loop timing).
- Affected code: new `include/EventJournal.h`, `src/EventJournal.cpp`, `include/EventJournalLaw.h`
  (line format, dedupe, rotation decision — header-only, native-tested), `include/VescRxParser.h`,
  `include/I2cBusRecovery.h`, `src/I2cBusRecovery.cpp`; modified `src/main.cpp`,
  `src/VescMotorDriver.cpp`/`.h`, `include/IMotorDriver.h` (link stats), `src/SteeringController.cpp`,
  `src/VehicleController.cpp`, `src/CANController.cpp`/`.h`, `src/LwsSensor.cpp` (health-change hook
  only), `src/RelayController.cpp`, `src/BoardInputs.cpp`, `src/SteerDiag.cpp`, `src/WebPortal.cpp`,
  `include/WebPortal.h`, `src/TelemetryManager.cpp`, `data/index.html`, `include/Constants.h`,
  `include/Changelog.h`, `platformio.ini` (native test filters), two docs.
- Flash: ≤ 256 KB of the 1.5 MB LittleFS partition (≈ 1.2 MB free next to `index.html`); one NVS key.
- Overlap with pending changes: `move-can-rx-to-interrupt` and `move-aztec-vesc-to-can` both MODIFY
  `CAN Communication Error Handling` and the VESC requirements. To avoid three-way conflicts this
  change only ADDS requirements in `can-controller` and `vehicle-actuators`. If `move-aztec-vesc-to-can`
  lands first, B.1 shrinks to the journal hooks (the UART parser is removed by it) and the CAN
  re-init must also re-apply the VESC extended-ID filters. `add-tacho-rpm-input` MODIFIES
  `Service Console Section Layout`; this change adds its card through a separate ADDED requirement.
