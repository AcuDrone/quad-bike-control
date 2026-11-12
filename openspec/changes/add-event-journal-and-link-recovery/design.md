# Design: Persistent event journal and link recovery

## Context
- Every diagnostic today is a `Debug::printfFeature()` line on the USB-CDC console. It is gone the
  moment the laptop is unplugged, and the faults that matter happen in the field.
- Storage available: LittleFS `spiffs` partition 1.5 MB (`partitions_16mb.csv`, `0x810000`), holding
  only `data/index.html` (≈ 316 KB) → ≈ 1.2 MB free. NVS 20 KB (namespaces `transmission`, `debug`,
  `throttle`, `speed`, `boost`, `steering`, `relay`, `tacho`). Coredump 64 KB (unchanged).
- Single cooperative `loop()` on core 1 (`src/main.cpp:291-381`); the web server and WebSocket run in
  the AsyncTCP task. Phases in order: board inputs, speed sensor, MAVLink, web portal, input source,
  web command, `vehicleController.update()` (steering/CAN/relays/...), `steerDiag.update()`, MAVLink
  report, `telemetryManager.update()`.
- Native tests cover header-only "law" headers (`platformio.ini` `[env:native]`). New pure logic in
  this change follows that pattern: `include/EventJournalLaw.h`, `include/VescRxParser.h`.

## Goals / Non-Goals
- Goals: a post-mortem record of link, sensor, bus and control-source events that survives app OTA and
  most resets; three self-healing fixes for faults that currently never recover on their own; cheap
  enough to run always.
- Non-Goals: changing steering control law, relays, throttle, gears; MAVLink changes; remote upload of
  the journal; time-of-day timestamps (no RTC clock on the board — boot number + uptime only).

## D1 — Files, rotation, size
- `/events.log` (active) and `/events.1` (previous), `JOURNAL_FILE_MAX_BYTES` = 128 KB each.
- Before a flush that would push the active file past the limit: `remove("/events.1")`,
  `rename("/events.log", "/events.1")`, open a new `/events.log`. LittleFS `rename` is atomic: after a
  power cut either the old or the new name exists, never a half file.
- Why two files rather than an in-place ring: a ring in one file needs a head pointer that is updated
  on every flush (an extra metadata write), wrap-aware readers, and a rule for a torn pointer after
  power loss. On LittleFS an in-place overwrite is not cheaper than an append anyway (copy-on-write
  per block). Two files stay plain text, downloadable and greppable as-is, at the cost of the history
  shrinking from 256 KB to 128 KB right after a rotation.
- Size figures: lines are ≈ 70–90 B (SNAP ≈ 100 B) → ≈ 1500 lines per 128 KB. At the idle rate
  (60 SNAP/h ≈ 6 KB/h plus ≈ 1–2 KB/h of events) one file holds ≈ 17–20 h of powered time, i.e. about
  one rotation per day of operation and ≈ 1.5–2 days of history across both files.
- **Erasure**: the files live in the filesystem partition, so `pio run -t uploadfs` (and a web
  filesystem OTA, `/update?type=littlefs`) erases them. An application OTA does not touch them. To be
  documented in `WEB_PORTAL_SETUP.md`: download the journal before `uploadfs`.

## D2 — Line format
`<boot> <uptime_s.mmm> <SRC> <LVL> <event> k=v k=v ...`, ASCII, ≤ `JOURNAL_LINE_MAX` (120) bytes
including `\n`; a longer line is cut and ends with `~`. Values never contain spaces (tokens use `_`).
Levels `I`, `W`, `E`. A comment header opens every boot: `# boot=47 fw=1.5.26 reset=PANIC`.
```
47 0.412 SYS I boot reset=POWERON fw=1.5.26 heap=131560 prev_heap_min=118204
47 0.415 RTC I replay n=3 lost=0 from_boot=46
47 612.447 VESC W resync state=PAYLOAD idle=287 bytes_since_good=1412 drop=1
47 980.113 CAN W lws_silent ms=1200 obd_alive=1 -> reinit
47 2210.551 I2C W bus_stuck sda=0 clocks=9 released=1 reinit=1
47 60.001 SNAP I heap=126840 min=118204 loop_max=4 jflush_max=38 vin=24.0 fet=40.1
47 60.001 SNAP I lws_fps=100 vesc_good=198 drop=0 resync=0 can_reinit=0 i2c_unlock=0
```
The snapshot is written as **two** `SNAP` lines (all twelve fields on one line would be ≈ 150 B,
over `JOURNAL_LINE_MAX`). `vesc_good` / `drop` count the window; `resync`, `can_reinit`,
`i2c_unlock` are session totals.
Formatting, truncation, dedupe and the rotation decision are pure functions in
`include/EventJournalLaw.h` (native-tested).

## D3 — Event catalogue
| SRC | Event | LVL | Fields |
|-----|-------|-----|--------|
| SYS | `boot` | I | `reset fw heap prev_heap_min` (`?` after power-on) |
| SYS | `reboot` | I | `why=ota` — written and flushed before a controlled restart |
| SYS | `loop_overrun` | W | `ms phase phase_ms` (> 50 ms; not for phase `journal`, see D6) |
| SYS | `heap_low` / `heap_ok` | W / I | `free min` (threshold `JOURNAL_HEAP_WARN_BYTES`, hysteresis 8 KB) |
| SYS | `journal_cleared`, `journal_drop`, `suppressed` | I / W / I | `n`, `src ev n` |
| RTC | `replay` | I | `n lost from_boot` |
| SRC | `change` | I | `from to` |
| SRC | `failsafe_enter` / `failsafe_exit` | W / I | `from` / `to` |
| STEER | `mode` | I | `from to reason` (reason as `updateSteerMode()` builds it) |
| STEER | `stall` | W | `dir reason` (`triggerStallStop()`) |
| STEER | `diag_lock` | I | `on=1/0` (`SteeringController::setDiagLock()`) |
| STEER | `lws` | W / I | `fault=<faultToken> fps` on every health change |
| VESC | `link_lost` | E | `st bsg good crc stop rs vin fet fault` — compact keys for state, bytes_since_good, good, drop_crc, drop_stop, resync (the long names overflow 120 B); `st` is the parser state at the loss |
| VESC | `link_up` | I | same snapshot + `down_ms` |
| VESC | `link_down_snap` | W | same snapshot, every `VESC_LINK_DOWN_SNAP_MS` (10 s) while down |
| VESC | `fault` | W / I | `code` on change (`I code=0` when cleared) |
| VESC | `resync` | W | `state idle bytes_since_good drop` |
| CAN | `bus_off` | E | `tec rec eflg` |
| CAN | `lws_silent` / `obd_silent` | W | `ms obd_alive` (+ `-> reinit` when acted on) |
| CAN | `reinit` | I / E | `reason result attempt` (`result=fail` = E) |
| CAN | `first_frame` | I | `lws_ms obd_ms` after a re-init |
| I2C | `relay_fault` / `relay_ok` | E / I | `addr err step` |
| I2C | `input_fail` / `input_ok` | W / I | `addr err n` |
| I2C | `bus_stuck` | W | `sda scl clocks released reinit by err runs` (identical unsuccessful runs at most once per minute, `runs` = runs since the last line) |
| RELAY | `board_error` | E | `board addr why` (detection / init failure, starter escalation) |
| SNAP | — | I | two lines: `heap min loop_max jflush_max vin fet` / `lws_fps vesc_good drop resync can_reinit i2c_unlock` |

Wire error codes (`endTransmission()` 2 = addr NACK, 3 = data NACK, 4 = other, 5 = timeout) are
currently discarded by `PCA9557Expander`/`XL9535Expander` (`== 0`); they gain a `lastError()`.

## D4 — Write path, flush, failure handling
- `EventJournal::log(src, lvl, event, fmt, ...)` (static, like `Debug`): dedupe check → format into a
  stack buffer → append to the RAM buffer (`JOURNAL_RAM_BUF_BYTES` 4 KB) and to the RTC ring. Cost:
  one `snprintf` (tens of µs); no flash access.
- `EventJournal::service()` runs once per loop at the end of `loop()`: SNAP timer, and a flush when the
  buffer is non-empty and ≥ `JOURNAL_FLUSH_MS` (1 s) passed since the last flush, or the buffer is
  ≥ 75 % full. Flush = rotation check, `append` + `close` (sync), RTC ring marked flushed.
- Controlled reboot: the OTA success handler calls `EventJournal::flushNow("ota")` before
  `ESP.restart()`. During a filesystem OTA (`FILESYSTEM.end()` in the upload handler) the journal is
  suspended (no file I/O; RAM + RTC only); the device reboots afterwards anyway.
- Failure: a failed open/write/rename keeps the buffer and retries on the next flush. When the buffer
  is full the oldest lines are dropped and counted; the next written line is
  `SYS W journal_drop n=N`. After 3 consecutive failed flushes (or a failed mount) file output is
  disabled for the session: the journal keeps the RAM buffer and the RTC ring, `/api/events?tail`
  still serves the RAM lines, `journal_bytes` reports `-1`.
- Concurrency: one FreeRTOS mutex guards the RAM buffer and file operations. The loop only
  try-takes it (a busy mutex postpones the flush by a second, never blocks the loop); the HTTP
  handlers take it with a short timeout. Rotation is postponed while a download is streaming.
  `Clear` is a request flag executed by the loop (delete both files, empty the buffer, write
  `journal_cleared`).

## D5 — Flash wear
- LittleFS appends copy-on-write: after a sync, the next append copies the partially filled tail
  block into a freshly erased block. Model: every non-empty flush costs ≤ 1 data-block erase (4 KB) plus
  a metadata commit (a metadata-pair compaction, i.e. another erase, roughly every few dozen commits).
- Idle (SNAP + a few events): ≈ 60–90 flushes/h → ≈ 100 erases/h. Logical data ≈ 7–8 KB/h; physical
  programming ≈ 0.4 MB/h (write amplification from tail copies).
- LittleFS dynamic wear levelling spreads erases over the ≈ 290 free 4 KB blocks →
  ≈ 0.35 erases/block/h → 100 k-cycle NOR lasts ≈ 285 000 powered hours.
- Worst case, an event storm flushing every second without pause: ≈ 4000 erases/h → ≈ 14/block/h →
  ≈ 7000 h of *continuous* storm. Dedupe (D7) and the 1 s cap bound storms; they are not sustained in
  practice. Rotation adds one delete + rename per ≈ 1500 lines.

## D6 — Main-loop cost
- Logging: µs. HTTP reads run in the AsyncTCP task.
- Flush: page programs for a few hundred bytes (≈ 1–3 ms) plus one 4 KB sector erase, typically
  30–50 ms (datasheet maximum several hundred ms). During a flash erase the cache is disabled on both
  cores, so moving the flush into another task would not remove the stall; it would only add locking.
  This is the same cost profile as the existing NVS writes (distance, engine hours, calibration).
- Mitigations: at most one flush per second and only when there is data (idle ≈ one per minute); no
  flush while the steering motor is being driven (deferred up to 10 s, see Decisions); the
  flush is the last step of the loop iteration; its duration is measured (`jflush_max` in SNAP) and an
  overrun caused by the flush is attributed to phase `journal` and counted, never logged as a
  `loop_overrun` event (no feedback loop).
- Bench acceptance (tasks §9): over a 1 h run, no LWS `timeout`, no VESC `link_lost` and no MAVLink
  loss correlated with flushes; `jflush_max` below the LWS timeout (100 ms).

## D7 — Dedupe
- Key = (source, event token); parameters are not part of the key. Table of 24 keys, LRU.
- A line whose key was written less than `JOURNAL_DEDUPE_MS` (2 s) ago is not written; its slot's
  suppressed counter increments. The next written line with that key carries ` sup=N`.
- Suppressed counts still pending after 60 s are emitted at the next SNAP as
  `SYS I suppressed src=.. ev=.. n=N`.
- Exempt: `boot`, `reboot`, the `#` header, `RTC replay`, `SNAP`, `journal_*`.

## D8 — RTC_NOINIT ring and replay
- `RTC_NOINIT_ATTR` struct in RTC slow memory: magic, version, CRC, the boot number that wrote it,
  `prev_heap_min` (updated at every SNAP), and a ring of `JOURNAL_RTC_RING` (16) formatted lines
  (≈ 2 KB, `static_assert` ≤ 2304 B).
- Every accepted line is also copied into the ring; a successful flush marks the ring empty. So the
  ring holds exactly the lines not yet on flash (up to the newest 16; older unflushed lines are
  counted as `lost`).
- At boot, before the new boot's `boot` line: if magic and CRC are valid and the reset reason is not
  `POWERON`, the unflushed lines are written behind `RTC I replay n=K lost=M from_boot=B`, verbatim
  (original boot number and uptime), and the ring is cleared. Content after a power-on reset is
  undefined and is discarded (as is a bad CRC).
- Covers: panic, task/interrupt watchdog, `ESP.restart()`, brownout when RTC memory held. Does not
  cover: power loss (up to ≈ 1 s of events lost).

## D9 — VESC parser bug and fix
Mechanism (`src/VescMotorDriver.cpp:65-131`). A `COMM_GET_VALUES` reply is
`02 L 04 d1 … dn CH CL 03`, sent as one ≈ 7 ms burst every 300 ms. Suppose one data byte is lost:
```
reply k    02 L 04 d1 d2 d3 [d4 lost] d5 … dn CH CL 03     parser still needs 1 payload byte
reply k+1  02 | L 04 → CRC=L,04 | d1 ≠ 03 → drop, RX_START at d2
           d2 … dn CH CL       hunting 0x02/0x03; values at standstill rarely contain them
           03                  stop byte read as LONG-frame start → RX_LEN_HI
reply k+2  02 L                length 0x02LL ≥ 512 > 256 → reject, RX_START at 04
           04 d1 … CH CL 03    hunting again … stop byte → RX_LEN_HI → eats 02 L of reply k+3
```
Every reply's header is swallowed by the previous reply's stop byte. Because successive replies at
standstill carry nearly identical values (and CRCs), the pattern repeats until a value happens to
produce a 0x02/0x03 at a different offset — observed as minutes. After 1 s without a valid reply
`driverOk()` is false and the steering stops.

Fix:
1. `VescRxParser::feed(byte, nowMs)`: if the state is not `RX_START` and `nowMs − lastByteMs >
   VESC_RX_GAP_MS` (5 ms), reset to `RX_START` before handling the byte and count a `resync`. In the
   trace above the parser sits in `RX_LEN_HI` across the 300 ms gap, so reply k+2 is decoded. One lost
   byte now costs exactly one reply.
2. On the `driverOk()` true → false transition the driver flushes the UART RX buffer and resets the
   parser (fresh start for the next request).
3. Counters: `bytes`, `good`, `drop_crc`, `drop_stop`, `resync`, `bytes_since_good`; exposed through a
   `LinkStats` getter on `IMotorDriver` (BTS7960: all zero).
- Timestamps are taken when the loop reads the bytes, not when they arrive. A loop stall > 5 ms in the
  middle of a burst therefore resets a good frame: at most one reply is lost per stall and the next gap
  re-aligns. Accepted (link timeout is 1 s = 3 replies); visible as `resync` with a small `idle`.
- Alternative considered: UART RX-idle interrupt (`onReceive(..., onlyOnTimeout)`) — exact frame
  boundaries, but the boundary has to be correlated with the byte position the loop has consumed across
  two tasks. Not worth it with a 300 ms inter-reply gap.
- Native test `test_vesc_rx`: a reply with a dropped byte followed by 10 identical replies whose data
  and CRC contain no 0x02/0x03, timestamps 300 ms apart: with the gap disabled 0 of 10 decode (stuck);
  with the gap 10 of 10 decode. Plus: clean stream, CRC error, bad stop byte, mid-burst stall.

## D10 — CAN LWS watchdog and its interaction with the OBD watchdog
- Existing (unchanged): no OBD reply for `CAN_REINIT_NO_DATA_MS` (15 s) while polling → re-init,
  min. spacing `CAN_REINIT_MIN_INTERVAL_MS` (15 s). On aztec the Delphi MT05 always answers, so this
  never fires for a dead LWS path. (Since fw 1.5.28, change `gate-obd-watchdog-and-pull-up-vesc-rx`,
  it runs only while the ignition is `ACC` / `IGNITION`, on the same flag as the LWS watchdog, and its
  15 s window restarts at each switch-on.)
- New: `CANController` timestamps every dispatched `0x305` frame. The watchdog runs only while the
  ignition is `ACC` or `IGNITION` (the LWS is fed from the ACC line, Relay_1) and is armed after the
  first `0x305` seen since the ignition was switched on (a quad bike, an LWS that never powered up, or
  an ignition switched OFF never triggers it). Fires when
  - no `0x305` for `CAN_LWS_SILENT_MS` (1 s), and
  - an OBD reply arrived within `CAN_LWS_WD_OBD_ALIVE_MS` (2 s) — the bus and the chip demonstrably
    work, so the LWS path (RXB1 filters, a latched overflow, a chip that reset its masks) is suspect, and
  - no LWS zero-set is running, and
  - ≥ `CAN_LWS_REINIT_MIN_MS` (5 s) since the last re-init of any kind.
- Journal economy (implementation): a watchdog that keeps firing in one silence episode (dead LWS
  with the ECU on every 5 s; an ECU that is simply off every 15 s) journals its trigger line at the
  start of the episode and then every `CAN_WD_REPEAT_LOG_MS` (10 min) with `repeat=N`; the successful
  re-inits in between only increment `can_reinit`. The re-init cadence itself is unchanged. An
  episode ends with the next frame of that kind. Failed re-inits are always journaled.
- If OBD is silent too, the LWS watchdog does not act (`lws_silent obd_alive=0` is journaled once,
  deduped); the bus-wide case stays with the 15 s watchdog. All re-inits share `lastReinitTime_`, so the
  two watchdogs cannot fire back-to-back.
- The steering already treats > 100 ms silence as an LWS `timeout` fault (POSITION → RATE/BLOCKED); the
  watchdog only shortens how long that lasts. Recovery is logged as `first_frame lws_ms=..`.
- Re-init result checking: `recoverFromBusOff()` returns a result built from `begin() == CAN_OK`,
  `applyRxFilters()` and `setMode(MCP_NORMAL) == MCP2515_OK`. A failed attempt is retried on the next
  `update()` calls (no in-call loop), at most 3 attempts per trigger (the spec's existing "retry up to
  3 times"), then `reinit result=fail` is journaled and the next watchdog interval tries again.

## D11 — I2C2 bus unlock and timeout
- Why `Wire1.end()/begin()` alone is insufficient: it resets the ESP32 controller only. A slave that
  was interrupted mid-byte (bus glitch, master timeout, reset of the master during a read) keeps
  driving SDA low while it waits for the rest of its clock pulses. With SDA low the master cannot
  generate a START, so every transaction fails, forever. Whether the core clears the bus on its own
  depends on the arduino-esp32 / ESP-IDF driver version and is not observable from the firmware. The
  old firmware's AS5600 recovery did only `Wire.end()/begin()`. An explicit procedure is deterministic,
  version-independent and logged.
- Procedure (`I2cBusRecovery::recover(by, lastErr)`, main-owned, rate-limited to one per
  `I2C_RECOVER_MIN_MS` 1 s across all callers):
  1. Read SDA/SCL levels (GPIO input read works while routed to the controller).
  2. If SDA is low: `Wire1.end()`; SCL as open-drain output, SDA as input; up to 9 clock pulses
     (5 µs low / 5 µs high, ≈ 100 kHz), stopping as soon as SDA reads high; then a STOP (SDA low → SCL
     high → SDA high). If SCL itself reads low, no clocking is possible: logged `scl=0 released=0`.
  3. `Wire1.end()` (if not yet), `Wire1.begin(PIN_RELAY_SDA, PIN_RELAY_SCL, I2C_BUS_FREQ_HZ)`,
     `Wire1.setTimeOut(I2C_TIMEOUT_MS)`.
  4. The caller then runs its own device re-init (relay `initPort`, opto-input re-configuration).
  Implementation note: on the pinned core (arduino-esp32 3.3.7 / ESP-IDF 5.5, `i2c_master` driver)
  a transmit NACK is reported as Wire error 4, so only error 5 (timeout) counts as error evidence;
  the line levels cover a held bus.
  Cost ≈ 0.2 ms plus `begin()`. The BSS138 level shifter is open-drain on both sides, so bit-banging
  through it is valid.
- When it runs: in the relay recovery path before `initPort` (every `RELAY_RECOVER_MS` while faulted),
  and from `BoardInputs` after `BOARD_INPUT_UNLOCK_FAILS` (8, ≈ 200 ms) consecutive failed polls. Steps
  2–3 run only on bus evidence: SDA or SCL low, or the last error was a timeout (Wire 5; see the
  implementation note above for why 4 is not used).
  A plain address NACK (device unplugged, bus healthy) re-inits the device only; re-initializing
  `Wire1` every second for an unplugged relay board would cost the opto-input reader a poll each time.
- Timeout: the arduino-esp32 default is 50 ms. With `RELAY_WRITE_RETRIES` (write + read-back per
  attempt) a dead bus can hold the loop ≈ 300 ms; with 20 ms ≈ 120 ms, and only while faulted. A
  PCA9557/XL9535 transaction at 100 kHz takes ≈ 0.3–0.5 ms, and neither device stretches the clock.
- Spec impact: gpio-expander's "no driver and no coordinator SHALL call `end()` or `begin()` on
  `Wire1` after `setup()`" becomes "only `I2cBusRecovery` may".

## D12 — Loop timing
`main.cpp` stamps `micros()` between the phases listed in Context (≈ 10 stamps, < 10 µs). Per
iteration: total and the slowest phase. Kept: max total per SNAP window (`loop_max`), max flush time
(`jflush_max`). An iteration > `JOURNAL_LOOP_OVERRUN_MS` (50 ms) logs `loop_overrun` unless the
slowest phase was `journal`.

## D13 — Flash-write switch, default off
- NVS `journal`/`fs` (uint8 0/1, default 0), read once in `EventJournal::begin()`, written by the
  main loop when the operator changes it (`POST /api/events/config` only sets a request flag; the
  loop persists it, journals it and arranges the flush, so ordering is deterministic).
- **Why default off**: the flush's flash erase stalls both cores for 30–50 ms (D6). The aztec
  vehicle has not been bench-verified with the journal yet (tasks §9.4), so the first field runs
  must not risk an LWS timeout or a VESC link loss caused by the journal itself. The operator turns
  flash writing on from the web card once the bench numbers (`jflush_max`) are known.
- **What survives while off**: the RAM tail — the existing 4 KB buffer (`JOURNAL_RAM_BUF_BYTES`,
  ≈ 45–55 lines, ≈ 20 min of idle SNAPs) holding the newest lines, oldest whole lines dropped
  silently (counted, reported as `ram_dropped=` when switched on; no `journal_drop` spam); the RTC
  ring (last 16 lines across a panic / watchdog / software reset, replayed into the RAM tail at the
  next boot; `lost` then counts the RAM-tail lines beyond 16); dedupe, SNAP every 60 s (with
  `jflush_max=0`), counters, heap watch, loop timing, `link_down_snap`. Lost at power-off: all of it.
  `flushNow()` before an OTA / reboot only logs the `reboot` line (RAM + RTC ring).
- **No file access while off**: no flush, rotation, size read at boot (sizes are read lazily on the
  first tail / download or the first flush). Reading existing files for tail / download is allowed
  (read-only, no erase); the file part and the RAM tail are each preceded by a `# --- … ---` label
  line. `Clear` still deletes the files (explicit operator action).
- **On → off**: `SYS I journal_fs on=0`, one final flush (steering deferral applies), then file
  writes stop. **Off → on**: `SYS I journal_fs on=1 ram_dropped=N`, the RAM tail is flushed at once
  (steering deferral applies), then normal 1 s cadence. Both lines are dedupe-exempt (`journal_*`),
  so a gap in the flash history is always bracketed by them.
- `journal_bytes`: on → files + unflushed bytes (−1 when file output failed); off → RAM-tail bytes.
  Telemetry `journal_fs` carries the setting; the tail response carries `X-Journal-Fs` and
  `X-Journal-Ram-Lines` for the card's status row.

## Risks / Trade-offs
- Flash-erase stall in the loop (D6) → bench measurement; flushes deferred while the steering drives (Decisions).
- LittleFS free space shrinks if `index.html` grows a lot → 256 KB budget checked in docs; write errors
  degrade the journal, never the vehicle.
- CAN re-init storms with an unpowered LWS → ignition gating (ACC/ON only), arming only after a first frame since ignition on, 5 s spacing, dedupe.
- False VESC resync on a loop stall mid-burst → one reply lost, counted.
- Wire1 re-init interrupts the opto-input reader for one poll → bus evidence required, 1 s spacing.
- Pending changes (`move-aztec-vesc-to-can`, `move-can-rx-to-interrupt`) touch the same CAN/VESC code:
  whichever lands second rebases its hooks; the journal API is transport-agnostic.

## Alternatives considered
- Raw journal partition (no filesystem): best latency and wear control, but needs a repartition (full
  serial flash, and the partition file documents why large erases are avoided) and a custom reader.
- NVS blobs for events: small, not meant for logs, worse wear.
- Separate writer task: does not remove the flash stall (D6).
- One combined CAN watchdog: the 15 s OBD scale is far too slow for steering, and the two failure modes
  need different evidence.
- Reporting events over MAVLink `STATUSTEXT`: out of scope (no MAVLink changes).

## Migration Plan
No data migration. First boot of 1.5.26 creates the `journal` NVS namespace (boot = 1) and the files.
Rollback: flash 1.5.25; the files stay on LittleFS unused until the next `uploadfs`.

## Decisions (resolved after review)
- **LWS watchdog gating (was: back-off for an unpowered sensor).** The Bosch LWS is powered from the
  ACC line (ignition Relay_1). The LWS watchdog runs ONLY while the relay controller's ignition state
  is `ACC` or `IGNITION` (ON) — not `OFF`, not `CRANKING` — and arms only after the first `0x305`
  frame seen since the ignition was switched on (every transition into "powered" disarms it again).
  With the ignition OFF the sensor is legitimately silent: no `lws_silent` events, no re-inits.
  While armed the re-init spacing is a fixed `CAN_LWS_REINIT_MIN_MS` (5 s); there is NO exponential
  back-off. `VehicleController::update()` passes the powered flag to `CANController::setLwsPowered()`
  every iteration (the CAN controller stays free of relay knowledge).
- **Deferred flush while the steering drives.** While the steering motor is actively driven
  (closed-loop DRIVING, jog, nudge, rate drive, park pulse, or any nonzero effort such as a USB diag
  drive — `SteeringController::isDriving()`), `EventJournal::service()` defers its flash flush, for at
  most `JOURNAL_FLUSH_DEFER_MAX_MS` (10 s) counted from the moment a flush first became due; after
  10 s it flushes anyway. Rationale: a sector erase stalls both cores 30–50 ms and the LWS timeout
  is 100 ms. The deferral applies to every flush trigger including the 75 % fill level; an
  overflowing buffer drops its oldest lines (counted, `journal_drop`) rather than flushing early.
  `flushNow()` (controlled reboot) is never deferred.
- **Heap-low threshold.** `JOURNAL_HEAP_WARN_BYTES` = 40 KB (≈ 130 KB free today), hysteresis
  `JOURNAL_HEAP_HYST_BYTES` = 8 KB.
