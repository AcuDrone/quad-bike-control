## ADDED Requirements
### Requirement: Main Loop Timing Instrumentation
The main loop SHALL measure the duration of every iteration and of each of its phases (board inputs,
speed sensor, MAVLink, web portal, input source and web command, vehicle update, steering
diagnostics, MAVLink report, telemetry, journal) with `micros()` stamps, keep the maximum iteration
time and the maximum journal-flush time per snapshot window, and report overruns to the event
journal. The instrumentation SHALL NOT change the order or behavior of the phases.

#### Scenario: Overrun
- **WHEN** an iteration takes longer than `JOURNAL_LOOP_OVERRUN_MS` (50 ms)
- **AND** the slowest phase was not the journal flush
- **THEN** `SYS W loop_overrun ms=<total> phase=<slowest> phase_ms=<its time>` SHALL be journaled

#### Scenario: Snapshot window
- **WHEN** the periodic snapshot is written
- **THEN** it SHALL report `loop_max` and `jflush_max` for the elapsed window and both SHALL restart
  from zero

#### Scenario: Low heap
- **WHEN** free heap falls below `JOURNAL_HEAP_WARN_BYTES` (40 KB)
- **THEN** `SYS W heap_low free=<bytes> min=<bytes>` SHALL be journaled once
- **AND** `SYS I heap_ok` SHALL be journaled when free heap rises 8 KB above the threshold
