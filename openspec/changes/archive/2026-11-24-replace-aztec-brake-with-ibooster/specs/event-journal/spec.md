## MODIFIED Requirements
### Requirement: Event Line Format
Each event SHALL be one ASCII line of at most `JOURNAL_LINE_MAX` (120) bytes in the form
`<boot> <uptime_s.mmm> <SRC> <LVL> <event> k=v k=v ...`, where `<boot>` is the boot counter, the
uptime is seconds with millisecond resolution, `<SRC>` is one of `SYS SRC STEER VESC CAN I2C RELAY
SNAP RTC BRAKE` and `<LVL>` is `I`, `W` or `E`. Values SHALL contain no spaces. Each boot SHALL begin
with a comment header line `# boot=<N> fw=<version> reset=<reason>`.

#### Scenario: Line is formatted
- **WHEN** the VESC parser resynchronizes 612.447 s after boot 47
- **THEN** a line of the form `47 612.447 VESC W resync state=PAYLOAD idle=287 bytes_since_good=1412 drop=1`
  SHALL be journaled

#### Scenario: Brake source
- **WHEN** the service brake becomes `READY` 3.2 s after boot 47
- **THEN** a line of the form `47 3.200 BRAKE I state from=STARTING to=READY st=5 is=0 drv=1
  rod=0.02 ign=1 crank=0` SHALL be journaled

#### Scenario: Over-long line is truncated
- **WHEN** a formatted line would exceed `JOURNAL_LINE_MAX` bytes
- **THEN** it SHALL be cut to fit and end with `~` before the newline

### Requirement: Buffered Journal Flush
Lines SHALL be appended to a RAM buffer (`JOURNAL_RAM_BUF_BYTES`, 4 KB) by a non-blocking call that
never accesses flash. While flash writes are switched on (see Journal Flash-Write Switch) the buffer
SHALL be written to flash from the main loop at most once per `JOURNAL_FLUSH_MS` (1 s) and only when it holds data (earlier when ≥ 75 % full), and immediately
before a controlled reboot. While the steering motor is actively driven (closed-loop move, jog,
nudge, rate drive, park pulse), or a service-brake request is active (`BRAKING` or `RELEASING`), a due
flush SHALL be deferred, for at most `JOURNAL_FLUSH_DEFER_MAX_MS` (10 s), because a flash sector erase
stalls both cores, and with them the CAN task, for tens of milliseconds. A flash write failure SHALL
never affect vehicle control.

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

#### Scenario: Flush deferred while the service brake is requested
- **WHEN** a flush is due while a service-brake request is active
- **THEN** no flash write SHALL occur until the request ends, for at most 10 s from the moment the
  flush became due

#### Scenario: Loop never waits for the journal
- **WHEN** an HTTP handler holds the journal lock at the moment a flush is due
- **THEN** the main loop SHALL skip that flush and retry one second later instead of waiting

## ADDED Requirements
### Requirement: Service Brake Journal Events
The journal SHALL record the service brake's availability and its troubles under the source `BRAKE`.
Normal braking (the `BRAKING` / `RELEASING` transitions) SHALL NOT be journaled.

#### Scenario: Availability transitions
- **WHEN** the brake state changes between `ASLEEP`, `STARTING`, `READY`, `LOST` and `FAULT`
- **THEN** `BRAKE I state from=<s> to=<s> st=<n> is=<n> drv=<n> rod=<mm> ign=<0|1> crank=<0|1>`
  SHALL be journaled
- **AND** these lines SHALL NOT be suppressed by the 2 s dedupe (transitions come in quick pairs,
  e.g. `STARTING` then `READY` within 0.3 s); instead at most 8 are written in a burst and then one
  per second, and the number held back is appended to the next one as `sup=<n>`

#### Scenario: Fault entry
- **WHEN** the state becomes `FAULT`
- **THEN** `BRAKE E fault st= is= drv= rod= tgt= req= flow= vbat= crank=0` SHALL be journaled, at
  level `W` with `crank=1` inside the crank window, `vbat` being the X10 battery voltage or `-`
- **AND** it SHALL be followed by `BRAKE I fault_raw r39d=<hex> r38e=<hex> r38f=<hex>` with the last
  raw bytes of the three unit frames

#### Scenario: Unit lost
- **WHEN** the state becomes `LOST`
- **THEN** `BRAKE E lost age=<ms> tgt= req= vbat= crank=` (level `W` with `crank=1`) and a
  `fault_raw` line SHALL be journaled
- **AND** on the next `READY`, `BRAKE I recovered after_s=<s>` SHALL be journaled

#### Scenario: Long-lasting trouble
- **WHEN** `FAULT` lasts 20 s, `STARTING` lasts 20 s, or `LOST` lasts 20 s
- **THEN** `BRAKE E fault_latched`, `BRAKE E start_timeout` or `BRAKE E no_unit` SHALL be journaled
  once per episode

#### Scenario: Request troubles
- **WHEN** the unit drops an active request, a slot gap above 50 ms occurs, a stale-command episode
  ends, a release times out, or the rod does not move under an accepted request
- **THEN** `BRAKE W unit_drop`, `BRAKE W tx_gap ms=`, `BRAKE W cmd_stale ms=`,
  `BRAKE W release_slow` or `BRAKE W no_move` SHALL be journaled with the target and the rod (and
  the request bit / flow where they apply)
- **AND** a park-brake hand-over cancelled for "no move" SHALL be journaled as
  `BRAKE W pbrake_handover_cancel why=no_move`

#### Scenario: Statistics
- **WHEN** `JOURNAL_SNAP_MS` (60 s) elapses while the ignition is on
- **THEN** `BRAKE I stats tx=<n> miss=<n> gap_max=<ms> bad=<n> drops=<n> cmd_max=<ms>` SHALL be
  journaled with the window's values
