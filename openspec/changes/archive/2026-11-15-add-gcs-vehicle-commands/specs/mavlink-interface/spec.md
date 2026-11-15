## MODIFIED Requirements
### Requirement: Inbound Command Handling and Trip Reset
The system SHALL accept inbound `COMMAND_LONG` (#76) messages addressed to THIS component and
SHALL answer each one with a `COMMAND_ACK`. Because this component deliberately shares the
autopilot's system id (`MAVLINK_SYSTEM_ID`) and is distinguished only by `MAVLINK_COMPONENT_ID`,
the addressing check SHALL be strict: a command SHALL be handled only when BOTH
`target_system == MAVLINK_SYSTEM_ID` AND `target_component == MAVLINK_COMPONENT_ID`. A broadcast
(`target_component == 0`) or a command addressed to any other component SHALL be ignored silently,
with NO acknowledgement, so that this peripheral can never answer for the autopilot.

This system implements exactly two commands:
- a **trip reset**, carried as `MAV_CMD_USER_1` (31010) with a magic `param1`;
- the **vehicle commands**, carried as `MAV_CMD_USER_2` (31011) (see "GCS Vehicle Commands via
  MAV_CMD_USER_2").

The trip reset SHALL clear BOTH resettable trip counters — the trip DISTANCE and the trip ENGINE
HOURS — because they describe the same "since the operator last reset" interval in different units
and SHALL NOT be allowed to diverge. There SHALL be NO second trip command, NO additional magic
`param1` value and NO separate web control for either of them. The odometer and the TOTAL engine
hour meter SHALL NOT be resettable by any command.

The handler SHALL follow the existing transport policy: `include/MavlinkInterface.h` stays free of
mavlink headers, so the handler SHALL take decoded scalars (`param1` and `param2`; the
`handleServoOutputRaw` / `handleParamValue` pattern). For the trip reset the transport SHALL NOT
hold a reference to the speed sensor — it SHALL latch a pending request that the vehicle layer
consumes and performs. For the vehicle commands the transport SHALL hand the decoded function and
value to a vehicle-command sink registered by `main.cpp` (implemented by the vehicle layer, no
mavlink types in its interface) and SHALL acknowledge the outcome it returns.

#### Scenario: Accept a trip reset addressed to this component
- **WHEN** a `COMMAND_LONG` arrives with `target_system == MAVLINK_SYSTEM_ID`,
  `target_component == MAVLINK_COMPONENT_ID`, `command == MAV_CMD_USER_1` (31010) and `param1`
  equal to `MAVLINK_CMD_TRIP_RESET_MAGIC`
- **THEN** a trip-reset request SHALL be latched for the vehicle layer to perform
- **AND** the vehicle layer SHALL zero the trip DISTANCE counter and persist it on its next control
  iteration
- **AND** in the SAME operation the vehicle layer SHALL zero the trip ENGINE HOURS counter and
  persist it, so the two trip readings can never disagree about which interval they describe
- **AND** a `COMMAND_ACK` with `MAV_RESULT_ACCEPTED` SHALL be sent
- **AND** one line SHALL be logged on the serial console naming the requester's system and
  component (for example `[MAV] TRIP reset accepted from 255/190`)
- **AND** the odometer AND the TOTAL engine hour meter SHALL be left unchanged
- **AND** the inbound command handler SHALL NOT be changed to accommodate the second counter: it
  SHALL still latch exactly one request, carry no reference to the vehicle's counters, and know
  nothing of which counters the vehicle layer clears

#### Scenario: Compare the magic parameter with a tolerance
- **WHEN** `param1` is evaluated against `MAVLINK_CMD_TRIP_RESET_MAGIC`
- **THEN** the comparison SHALL use a tolerance rather than float equality, in line with the
  existing rule against comparing floats with `==`
- **AND** a `param1` of `NaN` SHALL fail the comparison and SHALL therefore be denied, not accepted

#### Scenario: Deny a trip reset with the wrong parameter
- **WHEN** a `COMMAND_LONG` for `MAV_CMD_USER_1` is addressed to this component with any `param1`
  other than the magic value
- **THEN** a `COMMAND_ACK` with `MAV_RESULT_DENIED` SHALL be sent
- **AND** the rejection SHALL be logged with the offending `param1` value and the requester
- **AND** BOTH trip counters SHALL be left unchanged, so a stray, replayed or mis-scripted
  `MAV_CMD_USER_1` cannot silently destroy the operator's readings

#### Scenario: Report any other command as unsupported
- **WHEN** a `COMMAND_LONG` carrying any command other than `MAV_CMD_USER_1` and `MAV_CMD_USER_2`
  is addressed specifically to this system AND this component
- **THEN** a `COMMAND_ACK` with `MAV_RESULT_UNSUPPORTED` SHALL be sent
- **AND** the command id and the requester SHALL be logged
- **AND** no vehicle state SHALL change

#### Scenario: Ignore broadcasts and commands for other components
- **WHEN** a `COMMAND_LONG` arrives with `target_component == 0` (broadcast), with a
  `target_component` that is not `MAVLINK_COMPONENT_ID`, or with a `target_system` that is not
  `MAVLINK_SYSTEM_ID`
- **THEN** the message SHALL be discarded with NO `COMMAND_ACK` and no log line
- **AND** this SHALL apply to `MAV_CMD_USER_2` exactly as to every other command
- **AND** a normal ground-station session against the autopilot (arming, mode changes, mission
  upload) SHALL therefore produce no acknowledgement from this component at all

#### Scenario: Acknowledge back to the requester, not to the autopilot
- **WHEN** a `COMMAND_ACK` is sent
- **THEN** its `target_system` and `target_component` SHALL be the `sysid` and `compid` of the
  message that requested the command, not the learned autopilot's
- **AND** the ACK SHALL be sent from this component's own `MAVLINK_SYSTEM_ID` /
  `MAVLINK_COMPONENT_ID`
- **AND** delivery SHALL rely on the autopilot's existing routing, which has already learned this
  component's address from its outbound `HEARTBEAT` and `EFI_STATUS` traffic — no additional
  routing configuration SHALL be required

#### Scenario: The odometer is not resettable over MAVLink
- **WHEN** any inbound command is processed
- **THEN** there SHALL be NO command, parameter or magic value that zeroes or decreases the
  odometer, and none that zeroes or decreases the TOTAL engine hour meter
- **AND** only the two trip counters — the trip distance and the trip engine hours — SHALL be
  clearable, and only together

### Requirement: Steering Control Mode Reporting
The system SHALL report the current steering mode (`position`, `rate` or `blocked`, see "Sensorless Steering Rate Mode") to the MAVLink network as a `NAMED_VALUE_INT` (id 252) named `STEER_MODE` sent from this component, and SHALL announce every change with a `STATUSTEXT`. This reporting SHALL NOT be a control input, and it SHALL NOT change the set of `NAMED_VALUE_FLOAT` names this component sends.

#### Scenario: Periodic and on-change STEER_MODE
- **WHEN** the slow steering telemetry interval elapses (`MAVLINK_STEER_SLOW_TX_MS`, default 1000 ms)
- **THEN** a `NAMED_VALUE_INT` named `STEER_MODE` SHALL be sent from the ESP32's own system and component id (system 1, component 25), with value 0 = `position`, 1 = `rate`, 2 = `blocked`
- **AND** its `time_boot_ms` SHALL carry the milliseconds-since-boot value of that tick
- **WHEN** the steering mode changes
- **THEN** one additional `STEER_MODE` SHALL be sent on the next report opportunity, without waiting for the slow interval
- **AND** `STEER_MODE` SHALL keep being sent whatever the sensor, VESC or link state, and SHALL never carry any value other than 0, 1 or 2

#### Scenario: Mode change status text
- **WHEN** the steering mode changes
- **THEN** a `STATUSTEXT` SHALL be sent: `Steer: RATE mode (no sensor)` with severity WARNING on entry into `rate`, `Steer: blocked (no sensor)` with severity WARNING on entry into `blocked`, and `Steer: position mode` with severity INFO on entry into `position`
- **AND** it SHALL obey the existing `STATUSTEXT` rate limit, and a change that is suppressed by the rate limit SHALL still be visible through `STEER_MODE`

#### Scenario: Separate from the NAMED_VALUE_FLOAT exception
- **WHEN** steering telemetry is reported
- **THEN** the six `NAMED_VALUE_FLOAT` names (`STEER_POS`, `STEER_A`, `VESC_V`, `VESC_TEMP`, `VESC_OK`, `STEER_SCA`) SHALL remain the only `NAMED_VALUE_FLOAT` this component sends, unchanged in meaning and gating
- **AND** `STEER_POS` SHALL remain `NaN` in `rate` and `blocked` mode, because the sensor is unhealthy
- **AND** `STEER_MODE`, `QB_CAPS` (see "Vehicle Capability Broadcast") and `IGN_STATE` (see "Ignition State Broadcast") SHALL be the only `NAMED_VALUE_INT` names this component sends

#### Scenario: Reporting only
- **WHEN** `STEER_MODE` or the mode status text is sent with any value
- **THEN** no steering stop, interlock or other control action SHALL be triggered by the act of reporting it

## ADDED Requirements
### Requirement: GCS Vehicle Commands via MAV_CMD_USER_2
The system SHALL accept vehicle commands from the ground station as `COMMAND_LONG`
`MAV_CMD_USER_2` (31011) addressed exactly to this component (control-protocol version 1).
`param1` SHALL be the function and `param2` the value; `param3`..`param7` SHALL be ignored. Both
SHALL be compared with `MAVLINK_CMD_PARAM_EPSILON`, never with `==`, and `NaN` SHALL never match.

| param1 | Function | param2 | Capability bit |
|---|---|---|---|
| 0 | Query | 0 caps, 1 firmware version, 2 hardware version | — |
| 1 | Gear | R −1, N 0, H 1, L 2 | bit0 R/N/L, bit1 H |
| 2 | Ignition | OFF 0, ACC 1, ON 2 | bit2 |
| 3 | Headlight | OFF 0, ON 1 | bit3 |
| 4 | Diff lock | OFF 0, ON 1 | bit4 |
| 5 | Handbrake | OFF 0 (release), ON 1 (apply) | bit5 |

Every such command SHALL be answered with exactly one `COMMAND_ACK` to the requester:
- `MAV_RESULT_ACCEPTED` — accepted for execution, also when the vehicle is already in the
  requested state;
- `MAV_RESULT_TEMPORARILY_REJECTED` — an interlock refused it, with the reason in `result_param2`:
  1 engine not running, 2 speed above `TRANS_SPEED_INTERLOCK_THRESHOLD_MS` (1.4 m/s), 3 fail-safe /
  not under MAVLink control, 4 speed above 3 km/h (handbrake apply);
- `MAV_RESULT_DENIED` — `param2` not finite, not an integer within the tolerance, or not in the
  function's table;
- `MAV_RESULT_UNSUPPORTED` — `param1` not finite or not an integer 0…5, or a function (or gear H)
  whose capability bit is clear on this build;
- `MAV_RESULT_FAILED` — the vehicle could not execute it (see "GCS Vehicle Command Interlocks" and
  "GCS Handbrake Command").

`progress` SHALL be 0 except for the capability query. `result_param2` SHALL be the reason code for
`TEMPORARILY_REJECTED`, the answer for a query, and 0 otherwise. The ACK SHALL be sent in the same
call that handled the command. Each command SHALL be logged once on the serial console with the
function, the value, the result, the reason and the requester. Without a registered vehicle-command
sink, every `MAV_CMD_USER_2` SHALL be answered `UNSUPPORTED`.

#### Scenario: Accepted command
- **WHEN** `MAV_CMD_USER_2` with `param1` 3 and `param2` 1 arrives addressed to 1/25
- **THEN** the headlight SHALL be switched on
- **AND** a `COMMAND_ACK` with `MAV_RESULT_ACCEPTED`, `progress` 0 and `result_param2` 0 SHALL be
  sent to the requester

#### Scenario: Unknown function
- **WHEN** `param1` is 6, −1, 2.5 or `NaN`
- **THEN** the ACK SHALL be `MAV_RESULT_UNSUPPORTED` and no vehicle state SHALL change

#### Scenario: Function not on this vehicle
- **WHEN** `param1` is 5 on a build whose capability bit5 is clear (the quad bike, or the buggy
  without the X6 driver)
- **THEN** the ACK SHALL be `MAV_RESULT_UNSUPPORTED`

#### Scenario: Invalid value
- **WHEN** `param1` is 2 (ignition) and `param2` is 3, 1.5 or `NaN`
- **THEN** the ACK SHALL be `MAV_RESULT_DENIED` and the ignition SHALL not change

#### Scenario: Tolerance
- **WHEN** `param2` is 1.005 for the headlight
- **THEN** it SHALL be taken as 1 (ON)

#### Scenario: Broadcast is not answered
- **WHEN** `MAV_CMD_USER_2` arrives with `target_component` 0
- **THEN** no `COMMAND_ACK` SHALL be sent and no vehicle state SHALL change

### Requirement: Vehicle Command Queries
`MAV_CMD_USER_2` with `param1` 0 SHALL be answered `MAV_RESULT_ACCEPTED` with the answer in
`result_param2`, without changing any vehicle state:
- `param2` 0 — capability bitmask: bit0 gears R/N/L, bit1 gear H, bit2 ignition, bit3 headlight,
  bit4 diff lock, bit5 handbrake, bits 6–15 reserved = 0, and `progress` = the control-protocol
  version (1). A bit SHALL be set only when the function works on this build: on the quad bike
  (`master`) the mask SHALL be `0x1F` and bit5 SHALL never be set; on the buggy (`aztec`) it SHALL
  be `0x3F` when the X6 parking-brake driver initialised at boot and `0x1F` otherwise. The mask SHALL
  be fixed at boot.
- `param2` 1 — firmware version `major*10000 + minor*100 + patch` parsed from `FIRMWARE_VERSION`
  (1.5.30 → 10530).
- `param2` 2 — hardware version: bits 0–7 board revision (0 = `Control_v0`), bits 8–15 vehicle type
  (1 quad bike, 2 buggy), bits 16–23 the relay board detected at boot (0 none, 1 PCA9557, 2 XL9535),
  bits 24–31 zero.

Any other `param2` SHALL be answered `MAV_RESULT_DENIED`. The firmware and hardware versions are for
display only.

#### Scenario: Caps query on the quad bike
- **WHEN** the quad bike receives `param1` 0, `param2` 0
- **THEN** the ACK SHALL be `ACCEPTED` with `result_param2` 31 (`0x1F`) and `progress` 1

#### Scenario: Caps query on the buggy
- **WHEN** the buggy with an initialised X6 driver receives the caps query
- **THEN** `result_param2` SHALL be 63 (`0x3F`)

#### Scenario: Firmware version
- **WHEN** firmware 1.5.30 receives `param1` 0, `param2` 1
- **THEN** `result_param2` SHALL be 10530 and `progress` 0

#### Scenario: Hardware version
- **WHEN** the buggy with an XL9535 relay board receives `param1` 0, `param2` 2
- **THEN** `result_param2` SHALL be `0x00020200`

#### Scenario: Unknown query
- **WHEN** `param1` is 0 and `param2` is 3
- **THEN** the ACK SHALL be `MAV_RESULT_DENIED`

### Requirement: Vehicle Capability Broadcast
The system SHALL send a `NAMED_VALUE_INT` named `QB_CAPS` from this component every 10 s
(`VCMD_CAPS_TX_MS`), the first one on the first report after boot, with value = capability bitmask
(bits 0–15) | control-protocol version << 24, bits 16–23 zero. It SHALL use the 10-byte zero-padded
name handling of `sendNamedInt()` and SHALL be sent whatever the link or vehicle state. It is a
safety net for a lost query answer.

#### Scenario: Quad bike
- **WHEN** 10 s have elapsed since the last `QB_CAPS` on the quad bike
- **THEN** a `QB_CAPS` with value 16777247 (`0x1F | 1 << 24`) SHALL be sent

#### Scenario: Buggy with the parking brake
- **WHEN** `QB_CAPS` is sent on the buggy with an initialised X6 driver
- **THEN** its value SHALL be 16777279 (`0x3F | 1 << 24`)

### Requirement: Ignition State Broadcast
The system SHALL send a `NAMED_VALUE_INT` named `IGN_STATE` from this component with the APPLIED
relay ignition state: 0 OFF, 1 ACC, 2 ON, 3 CRANKING. CRANKING SHALL cover both the pre-crank dwell
(state ACC with a crank armed) and the starter phase; after the engine catches or the starter times
out the value SHALL be 2. With the relay board unavailable the value SHALL be 0. It SHALL be sent on
the 1 Hz slow telemetry tick and once immediately on every change, whatever the input source.

#### Scenario: Auto-crank sequence
- **WHEN** an ignition ON command starts the auto-crank from OFF
- **THEN** `IGN_STATE` SHALL be 3 during the 2 s pre-crank dwell and while the starter runs
- **AND** it SHALL become 2 when the engine starts or the 5 s cranking timeout ends the attempt

#### Scenario: On-change send
- **WHEN** the applied ignition state changes from 2 to 0
- **THEN** an `IGN_STATE` with value 0 SHALL be sent on the next report, without waiting for the
  1 Hz tick

### Requirement: Vehicle Command Protocol Extension Rules
The control protocol SHALL be extended without breaking existing clients: a new function SHALL take
the next `param1` and the next capability bit (and, if it is an on/off function, the next
`EFI_STATUS.pt_compensation` state bit); a new query SHALL take the next `param2` under `param1` 0;
a second capability word, if ever needed, SHALL be a new query value. The protocol version (caps
query `progress`, `QB_CAPS` bits 24–31) SHALL change only on a breaking change. Unknown commands and
unknown functions SHALL keep answering `UNSUPPORTED`.

#### Scenario: Adding a function
- **WHEN** a seventh function is added in a later firmware
- **THEN** it SHALL use `param1` 6 and capability bit6
- **AND** the protocol version SHALL stay 1
