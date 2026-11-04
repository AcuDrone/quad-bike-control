# Change: CAN bus sniffer for ECU frame discovery

## Why
On the Aztec buggy (ODES JUNGLECROSS 800 EFI) the engine ECU answers OBD-II PID 0x0C with
`04 41 0C 00 00` (RPM 0) while the engine idles. We suspect it broadcasts real engine data in its
own CAN frames. The MCP2515 accepts only OBD replies (RXB0, 0x7E8–0x7EF) and the Bosch LWS frame
(RXB1, 0x305), so any such broadcast is invisible today. We need a way to see every ID on the bus
from the web console, without a laptop or a separate CAN tool.

## What Changes
- `CANController` gets a one-shot **sniffer**: `startSniff(ms)`, `sniffActive()`,
  `getSniffResults()`. For a fixed window (10 s from the web) it:
  - pauses OBD-II polling, and refuses to run together with the ECU probe or the LWS zero-set
    (each refuses while the other runs);
  - opens both MCP2515 receive buffers to every standard and extended ID (masks 0; one standard
    and one extended filter per buffer; BUKT rollover kept);
  - drains up to 32 frames per `update()` call (5 in normal operation);
  - records a table of up to 32 distinct IDs: ID, extended flag, DLC, frame count, last 8 data
    bytes, and a per-byte "changed" bitmask (a bit is set once that byte differs from the
    previous frame of the same ID);
  - counts distinct IDs beyond the 32-row table, total frames, and RX overflow events
    (EFLG RX0OVR/RX1OVR);
  - keeps handing 0x305 frames to `LwsSensor` (they also appear in the table);
  - at the end restores the normal masks/filters with the existing `applyRxFilters()` routine
    (full chip re-init if that fails) and resumes polling.
- Web command `can_sniff` (same pattern as `can_probe`), refused while the probe or the LWS
  zero-set runs, or during a gear change.
- Telemetry gets a transient `sniff` object while the sniffer runs or its results are fresh
  (same TTL idea as `probe`). The static telemetry document grows from 4608 to 8192 bytes.
- Diagnostics page: a "CAN sniffer (10 s)" button next to "Probe ECU", a status line
  (running / done / refused), and a table ordered by ID: ID (hex, EXT marker), count, Hz,
  last data with the changed bytes highlighted, plus a summary line
  "IDs beyond 32 / frames / overflow". uk/en strings.
- Firmware 1.5.7 with a changelog line.

## Safety
The sniffer is a standstill diagnostic. During the window there are no OBD-II replies, so
`dataValid` goes false after 5 s (`CAN_DATA_STALE_TIMEOUT`): gear boost idles, engine-running
checks fail safe. The LWS frame keeps reaching `LwsSensor`, but with every ID accepted the two
hardware buffers can overflow on a busy bus and drop LWS frames; the steering loop then takes its
normal sensor-unhealthy path (motor stopped) and recovers by itself once frames arrive again,
without a reboot. The help text says to use it standing still.

## Impact
- Affected specs: `can-controller` (ADDED: CAN Bus Sniffer), `web-control` (ADDED: CAN Sniffer
  Command via Web Interface), `web-telemetry` (ADDED: CAN Sniffer Results Telemetry).
- Affected code: `include/CANController.h`, `src/CANController.cpp`, `include/Constants.h`,
  `include/VehicleController.h`, `src/VehicleController.cpp`, `include/WebPortal.h`,
  `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`, `include/Changelog.h`.
- Deployment: both firmware and the LittleFS image must be flashed.
- Pending changes `move-can-rx-to-interrupt` / `move-aztec-vesc-to-can` rewrite the receive path;
  whichever lands later re-bases the sniffer onto the queue-based router (the sniffer then reads
  from the software queue instead of the MCP2515 buffers).
