## ADDED Requirements
### Requirement: Event Journal Storage and Rotation
The system SHALL keep a persistent event journal as two plain-text files on the LittleFS partition:
`/events.log` (active) and `/events.1` (previous), each limited to `JOURNAL_FILE_MAX_BYTES`
(128 KB). The journal SHALL survive an application OTA. It is erased by `pio run -t uploadfs` and by
a web filesystem OTA, and this SHALL be documented in `WEB_PORTAL_SETUP.md`.

#### Scenario: Rotation by rename
- **WHEN** a flush would make `/events.log` exceed `JOURNAL_FILE_MAX_BYTES`
- **THEN** `/events.1` SHALL be deleted, `/events.log` SHALL be renamed to `/events.1`, and the
  pending lines SHALL be written to a new `/events.log`
- **AND** no more than two journal files SHALL exist at any time

#### Scenario: Application OTA keeps the journal
- **WHEN** a new application image is installed by OTA and the device reboots
- **THEN** both journal files SHALL still be present and the new boot SHALL append to `/events.log`

#### Scenario: Filesystem OTA suspends the journal
- **WHEN** a filesystem (LittleFS) OTA starts and the filesystem is unmounted
- **THEN** the journal SHALL stop all file I/O and keep new lines in RAM and in the RTC ring only

### Requirement: Event Line Format
Each event SHALL be one ASCII line of at most `JOURNAL_LINE_MAX` (120) bytes in the form
`<boot> <uptime_s.mmm> <SRC> <LVL> <event> k=v k=v ...`, where `<boot>` is the boot counter, the
uptime is seconds with millisecond resolution, `<SRC>` is one of `SYS SRC STEER VESC CAN I2C RELAY
SNAP RTC` and `<LVL>` is `I`, `W` or `E`. Values SHALL contain no spaces. Each boot SHALL begin with
a comment header line `# boot=<N> fw=<version> reset=<reason>`.

#### Scenario: Line is formatted
- **WHEN** the VESC parser resynchronizes 612.447 s after boot 47
- **THEN** a line of the form `47 612.447 VESC W resync state=PAYLOAD idle=287 bytes_since_good=1412 drop=1`
  SHALL be journaled

#### Scenario: Over-long line is truncated
- **WHEN** a formatted line would exceed `JOURNAL_LINE_MAX` bytes
- **THEN** it SHALL be cut to fit and end with `~` before the newline

### Requirement: Boot Record and Boot Counter
The system SHALL keep a boot counter in NVS (namespace `journal`, key `boot`), incremented once per
boot, and SHALL journal a boot record with the reset reason, firmware version, free heap and the
previous session's minimum free heap.

#### Scenario: Boot after power-on
- **WHEN** the device boots from power-on
- **THEN** the boot counter SHALL be incremented and stored
- **AND** a `# boot=N fw=... reset=POWERON` header and a `SYS I boot reset=POWERON fw=... heap=...
  prev_heap_min=?` line SHALL be journaled

#### Scenario: Boot after a crash
- **WHEN** the device boots after a panic or watchdog reset
- **THEN** the boot line SHALL name that reset reason and SHALL carry the previous session's minimum
  free heap kept in RTC memory

### Requirement: Unflushed Event Replay After Reset
Every accepted line SHALL also be stored in an `RTC_NOINIT` ring of the last `JOURNAL_RTC_RING` (16)
lines not yet written to flash. A successful flush SHALL mark the ring empty. At the next boot, unless
the reset reason is power-on or the ring fails its magic/CRC check, the unflushed lines SHALL be
written to the journal verbatim behind a replay marker, and the ring SHALL be cleared.

#### Scenario: Replay after a panic
- **WHEN** the firmware panics with 3 journal lines not yet flushed
- **THEN** at the next boot a line `RTC I replay n=3 lost=0 from_boot=<previous boot>` SHALL be
  written followed by the 3 lines with their original boot number and uptime

#### Scenario: Power loss
- **WHEN** power is removed and restored
- **THEN** the RTC ring content SHALL be discarded and no replay SHALL be written

#### Scenario: More unflushed lines than the ring holds
- **WHEN** more than 16 lines were pending at the reset
- **THEN** the newest 16 SHALL be replayed and `lost` SHALL report the number missing

### Requirement: Buffered Journal Flush
Lines SHALL be appended to a RAM buffer (`JOURNAL_RAM_BUF_BYTES`, 4 KB) by a non-blocking call that
never accesses flash. The buffer SHALL be written to flash from the main loop at most once per
`JOURNAL_FLUSH_MS` (1 s) and only when it holds data (earlier when ≥ 75 % full), and immediately
before a controlled reboot. While the steering motor is actively driven (closed-loop move, jog,
nudge, rate drive, park pulse) a due flush SHALL be deferred, for at most
`JOURNAL_FLUSH_DEFER_MAX_MS` (10 s), because a flash sector erase stalls both cores for tens of
milliseconds. A flash write failure SHALL never affect vehicle control.

#### Scenario: Idle flush cadence
- **WHEN** no line was logged since the last flush
- **THEN** no flash write SHALL occur

#### Scenario: Controlled reboot
- **WHEN** an OTA update completes and the device is about to restart
- **THEN** `SYS I reboot why=ota` SHALL be logged and the buffer SHALL be flushed before the restart

#### Scenario: Flash write fails
- **WHEN** a flush fails
- **THEN** the lines SHALL be kept and retried on the next flush
- **AND** if the buffer overflows, the oldest lines SHALL be dropped and the next written line SHALL be
  `SYS W journal_drop n=<count>`
- **AND** after 3 consecutive failed flushes, or a failed filesystem mount, file output SHALL be
  disabled for the session while the RAM buffer and RTC ring keep working and `journal_bytes`
  reports −1

#### Scenario: Flush deferred while the steering drives
- **WHEN** a flush is due while the steering motor is being driven
- **THEN** no flash write SHALL occur until the drive ends
- **AND** if the drive lasts longer than 10 s from the moment the flush became due, the flush SHALL
  run anyway
- **AND** a flush before a controlled reboot SHALL never be deferred

#### Scenario: Loop never waits for the journal
- **WHEN** an HTTP handler holds the journal lock at the moment a flush is due
- **THEN** the main loop SHALL skip that flush and retry one second later instead of waiting

### Requirement: Event Deduplication
The journal SHALL write the same (source, event) pair at most once per `JOURNAL_DEDUPE_MS` (2 s),
independent of the key=value fields. Suppressed repeats SHALL be counted and reported.

#### Scenario: Burst of identical events
- **WHEN** `I2C W input_fail` is logged 10 times within 1 s
- **THEN** only the first SHALL be written
- **AND** the next written `I2C input_fail` line SHALL carry `sup=9`

#### Scenario: Suppressed count without a later repeat
- **WHEN** a suppressed count is still pending 60 s later
- **THEN** the next snapshot SHALL be preceded by `SYS I suppressed src=<SRC> ev=<event> n=<count>`

#### Scenario: Exempt lines
- **WHEN** a boot header, boot line, reboot line, RTC replay marker or SNAP line is logged
- **THEN** it SHALL never be suppressed

### Requirement: Journaled Event Catalogue
The system SHALL journal at least: boot and controlled reboot; RTC replay; input-source change;
fail-safe entry and exit; steering mode change with reason; stall-stop; USB diagnostics lock acquire
and release; LWS health change with fault token and frame rate; VESC link lost / link up with a parser
snapshot (parser state, bytes since the last good packet, good and dropped counters, input voltage, FET
temperature, fault code); `link_down_snap` every `VESC_LINK_DOWN_SNAP_MS` (10 s) while the link is
down; VESC fault-code change; VESC parser resync; CAN bus-off; CAN silence detections and re-inits with
reason, result and the time to the first frame after a re-init; relay write fault and recovery with the
I2C error code and address; opto-input read failure and recovery; I2C bus unlock attempts; relay board
errors; main-loop overruns above `JOURNAL_LOOP_OVERRUN_MS` (50 ms) with the slowest phase; free heap
below `JOURNAL_HEAP_WARN_BYTES` and its recovery.

#### Scenario: VESC link loss
- **WHEN** the steering VESC driver reports no valid reply for `STEER_VESC_COMM_TIMEOUT_MS`
- **THEN** `VESC E link_lost` SHALL be journaled with the parser snapshot
- **AND** while the link stays down a `VESC W link_down_snap` line SHALL follow every 10 s
- **AND** `VESC I link_up` with `down_ms` SHALL be journaled when replies resume

#### Scenario: Fail-safe entry
- **WHEN** the input source becomes FAILSAFE
- **THEN** `SRC I change` and `SRC W failsafe_enter` SHALL be journaled

#### Scenario: Steering mode change
- **WHEN** the steering mode changes from POSITION to RATE because the LWS timed out
- **THEN** `STEER I mode from=POSITION to=RATE reason=sensor_timeout` (or equivalent tokens) SHALL be
  journaled

#### Scenario: Loop overrun caused by the journal flush
- **WHEN** an iteration exceeds 50 ms and the slowest phase was the journal flush
- **THEN** no `loop_overrun` line SHALL be written; the time SHALL be reported as `jflush_max` in the
  next snapshot

### Requirement: Periodic Snapshot
Every `JOURNAL_SNAP_MS` (60 s) the journal SHALL write `SNAP I` lines with free heap, minimum free
heap, the longest loop iteration and longest journal flush since the previous snapshot, VESC input
voltage and FET temperature, LWS frame rate, VESC good and dropped packet counters, and the session
totals of VESC resyncs, CAN re-inits and I2C unlocks.

#### Scenario: Snapshot line
- **WHEN** 60 s have elapsed since the previous snapshot
- **THEN** two lines like `47 60.001 SNAP I heap=126840 min=118204 loop_max=4 jflush_max=38 vin=24.0
  fet=40.1` and `47 60.001 SNAP I lws_fps=100 vesc_good=198 drop=0 resync=0 can_reinit=0
  i2c_unlock=0` SHALL be written (split so that each stays within `JOURNAL_LINE_MAX`)
- **AND** the loop and flush maxima SHALL restart from zero for the next window

### Requirement: Event Journal HTTP API
The web server SHALL expose the journal over HTTP without blocking the main loop.

#### Scenario: Tail
- **WHEN** a client requests `GET /api/events?tail=N`
- **THEN** the last N lines (default 100, at most 500) of the journal, including lines not yet
  flushed, SHALL be returned as `text/plain`, oldest first

#### Scenario: Download
- **WHEN** a client requests `GET /api/events/download`
- **THEN** `/events.1` followed by `/events.log` SHALL be streamed as one `text/plain` attachment
- **AND** rotation SHALL be postponed until the stream ends

#### Scenario: Clear
- **WHEN** a client sends `POST /api/events/clear`
- **THEN** the main loop SHALL delete both files and the RAM buffer and journal `SYS I journal_cleared`
- **AND** the boot counter SHALL NOT be reset
