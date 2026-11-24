## ADDED Requirements
### Requirement: Headlight Washer State in the EFI_STATUS Digital-Output Bitmask
The `EFI_STATUS.pt_compensation` digital-output bitmask SHALL report a running headlight-washer
pulse in bit4 `EFI_DIGITAL_FLAG_WASHER` (`0x10`) on both vehicles, next to bit0
`EFI_DIGITAL_FLAG_WHEEL_LOCK` and bit1 `EFI_DIGITAL_FLAG_FRONT_LIGHT`; bits 2–3 SHALL stay the
buggy's parking-brake bits (always 0 on the quad bike) and SHALL NOT be reused. Bit4 SHALL be set
exactly while `RelayController` reports a pulse running (`isWasherOn()`), and SHALL be 0 after the
normal end of the pulse, after every abort (ignition OFF, crank, `allOff()`, relay fault, GCS stop,
washer fitted switch turned off), while the relay board is unavailable and whenever the washer is
not fitted. The other bits SHALL NOT be affected. The bit is assembled
with the other flags where the report is built (`src/main.cpp`) and sent with the normal
`EFI_STATUS` rate; current GCS clients decode the word bit by bit and ignore an unknown bit4.

#### Scenario: Bit set during a pulse
- **WHEN** a 3000 ms washer pulse runs with the front light on
- **THEN** `pt_compensation` SHALL have bit4 and bit1 set (`0x12`), and bit0, bit2 and bit3 SHALL be
  as they were before the pulse

#### Scenario: Bit cleared after the normal end
- **WHEN** the pulse ends by itself after 3000 ms
- **THEN** the next `EFI_STATUS` SHALL have bit4 clear and the other bits unchanged

#### Scenario: Bit cleared after an abort
- **WHEN** the ignition is switched OFF 1000 ms into a pulse
- **THEN** the next `EFI_STATUS` SHALL have bit4 clear

## MODIFIED Requirements
### Requirement: Relay Controller for Ignition and Lights
The system SHALL provide a `RelayController` class to manage relay outputs for ignition
states, front light control and the headlight-washer pulse, driven by commands decoded from the MAVLink
interface or the web portal. The relays SHALL be driven through the external relay board's expander
— the PCA9557 relay board at `0x1F` or the XL9535-K8V5 relay board at `0x20`..`0x27`, selected once
at boot (see `gpio-expander` "Relay Board Selection and Detection") — rather than through direct
ESP32 GPIO, with the public API unchanged for callers. The bus is supplied by the owner at construction: `Wire1` (I2C2), the relay board's home on
header X15.

The PCA9557 relay board's IO routing is NOT 1:1 with the relay numbers (`Relay_1=IO4,
Relay_2=IO5, Relay_3=IO6, Relay_4=IO7, Relay_5=IO3, Relay_6=IO2, Relay_7=IO0, Relay_8=IO1`,
per the `Relay.PrjPcb` netlist); the XL9535 board maps Relay_N to P0.(N-1). Each function is defined
in `Constants.h` as a relay-number set and addressed at runtime as a whole-port MASK built from the
active board's table. Aztec buggy assignment (no relay is paired; masks shown for the PCA9557 /
XL9535 board):

| Function | Set constant | PCA9557 mask | XL9535 mask | Relays |
|----------|--------------|--------------|-------------|--------|
| Ignition / ECU line | `RELAY_SET_IGNITION` | `0b00010000` | `0b00000001` | Relay_1 |
| Starter | `RELAY_SET_STARTER` | `0b00100000` | `0b00000010` | Relay_2 |
| Front light | `RELAY_SET_FRONT_LIGHT` | `0b01000000` | `0b00000100` | Relay_3 |
| Headlight washer | `RELAY_SET_WASHER` | `0b00000010` | `0b10000000` | Relay_8 — timed pulse (see `vehicle-systems` "Headlight Washer Pulse") |
| Reserved | `RELAY_SET_RESERVED` | `0b10001101` | `0b01111000` | Relay_4..Relay_6 — three locks specified in a later change; Relay_7 — spare, never used by a function; off except in the web relay test |

The front-wheel lock is not fitted on the aztec buggy. The `EFI_STATUS` digital-output bitmask
layout SHALL be unchanged except for the new bit4 `EFI_DIGITAL_FLAG_WASHER` (see "Headlight Washer
State in the EFI_STATUS Digital-Output Bitmask"). MAVLink servo channel 7 SHALL NOT drive the lock on any build (see
`vehicle-systems` "Lock Not Driven by an RC Channel"); the lock is requested only by the web
`set_wheel_lock` command and the GCS lock command.

#### Scenario: Initialize relay controller
- **WHEN** `RelayController.begin()` is called with the I2C bus the relay board is wired to
- **THEN** the relay board SHALL be detected once (PCA9557 at `0x1F` or XL9535 at `0x20`..`0x27`)
  and its eight relay pins configured as outputs by `initAllOff()` (output `0x00` before
  configuration)
- **AND** the pins SHALL read back `0x00` (all relays OFF)
- **AND** internal state tracking is initialized to OFF / light off
- **AND** true is returned if initialization succeeds, false if detection fails or is ambiguous, or the
  all-off state does not verify

#### Scenario: Set ignition state
- **WHEN** `setIgnitionState(OFF)` is called, the ignition mask (Relay_1) and the starter mask
  (Relay_2) are LOW
- **WHEN** `setIgnitionState(ACC)` is called, the ignition mask is HIGH and the starter mask
  is LOW
- **WHEN** `setIgnitionState(IGNITION)` is called, the ignition mask is HIGH and
  the starter mask is LOW
- **WHEN** `setIgnitionState(CRANKING)` is called, the ignition mask and the starter mask are
  both HIGH
- **AND** internal state is updated to match in each case
- **AND** each change is applied as a single whole-port write from the output shadow byte

#### Scenario: Control front light
- **WHEN** `setFrontLight(true)` is called, the front-light mask (Relay_3) is set HIGH and
  state is updated
- **WHEN** `setFrontLight(false)` is called, the front-light mask is set LOW and state is updated

#### Scenario: Control front-wheel lock
- **WHEN** a web or GCS lock request calls `setWheelLock(true)`
- **THEN** `setWheelLock()` SHALL return `false` and no port write SHALL be made
- **AND** the rejection SHALL be logged once per boot, not on every control loop
- **AND** `getWheelLock()` SHALL return `false`
- **AND** bit0 (`EFI_DIGITAL_FLAG_WHEEL_LOCK`) of the `EFI_STATUS` digital-output bitmask SHALL be 0

#### Scenario: Spare relays are never energized
- **WHEN** any relay state changes through a MAVLink- or web-decoded vehicle function
- **THEN** every reserved bit — Relay_4, Relay_5, Relay_6 and Relay_7, at the active board's
  port bits — SHALL be written LOW unless the web relay test (ignition OFF only)
  energizes that relay
- **AND** Relay_8 SHALL be written HIGH only while a headlight-washer pulse runs (or the web relay test
  energizes it)
- **AND** a MAVLink ignition command that leaves `OFF`, a front-light ON or a washer start SHALL clear
  every web test relay first

#### Scenario: Fail-safe all relays off
- **WHEN** `allOff()` is called (fail-safe trigger)
- **THEN** the ignition mask, the starter mask, the front-light mask and the washer mask are set LOW
- **AND** internal state is reset to safe defaults (OFF, lights off, no washer pulse)
- **AND** every web test relay is released, so all eight relays are LOW
- **AND** the fail-safe write is verified by pin read-back like any other relay write

#### Scenario: Relay commands are no-ops when the expander is unavailable
- **WHEN** the expander failed to initialize or the driver is faulted
- **AND** a MAVLink- or web-decoded relay command arrives
- **THEN** the command SHALL be logged and ignored rather than silently assumed applied
- **AND** the reported ignition state SHALL be `OFF`

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
| 6 | Headlight washer | 0 stop, 1 wash (timed pulse; restarts a running one) | bit6 |

Every such command SHALL be answered with exactly one `COMMAND_ACK` to the requester:
- `MAV_RESULT_ACCEPTED` — accepted for execution, also when the vehicle is already in the
  requested state;
- `MAV_RESULT_TEMPORARILY_REJECTED` — an interlock refused it, with the reason in `result_param2`:
  1 engine not running, 2 speed above `TRANS_SPEED_INTERLOCK_THRESHOLD_MS` (1.4 m/s), 3 fail-safe /
  not under MAVLink control, 4 speed above 3 km/h (handbrake apply), 5 ignition not on or engine
  start in progress (headlight washer);
- `MAV_RESULT_DENIED` — `param2` not finite, not an integer within the tolerance, or not in the
  function's table;
- `MAV_RESULT_UNSUPPORTED` — `param1` not finite or not an integer 0…6, or a function (or gear H)
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
- **WHEN** `param1` is 7, −1, 2.5 or `NaN`
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

#### Scenario: Headlight washer values
- **WHEN** `param1` is 6 and `param2` is 2, 0.5 or `NaN`
- **THEN** the ACK SHALL be `MAV_RESULT_DENIED` and Relay_8 SHALL not change
- **AND WHEN** `param1` is 6 and `param2` is 1 on a firmware before this change
- **THEN** the ACK SHALL be `MAV_RESULT_UNSUPPORTED` (capability bit6 clear), so a ground station can
  tell the function is missing

#### Scenario: Broadcast is not answered
- **WHEN** `MAV_CMD_USER_2` arrives with `target_component` 0
- **THEN** no `COMMAND_ACK` SHALL be sent and no vehicle state SHALL change

### Requirement: Vehicle Command Queries
`MAV_CMD_USER_2` with `param1` 0 SHALL be answered `MAV_RESULT_ACCEPTED` with the answer in
`result_param2`, without changing any vehicle state:
- `param2` 0 — capability bitmask: bit0 gears R/N/L, bit1 gear H, bit2 ignition, bit3 headlight,
  bit4 diff lock, bit5 handbrake, bit6 headlight washer, bits 7–15 reserved = 0, and `progress` =
  the control-protocol version (1). A bit SHALL be set only when the function works on this build:
  on the quad bike (`master`) the mask SHALL be `0x5F` and bit5 SHALL never be set; on the buggy
  (`aztec`) it SHALL be `0x7F` when the X6 parking-brake driver initialised at boot and `0x5F`
  otherwise, when the washer fitted switch is on. Bit6 SHALL follow the persisted washer fitted
  switch (see `vehicle-systems` "Headlight Washer Fitted Switch"): with it off — the default — the
  masks SHALL be quad `0x1F`, buggy `0x3F` / `0x1F`, exactly as before this change. Bits 0–5 SHALL be
  fixed at boot; bit6 SHALL change at runtime when the switch changes, and the query SHALL always
  answer the current mask. Relay availability is runtime and gives `FAILED`, it does not clear bit6.
- `param2` 1 — firmware version `major*10000 + minor*100 + patch` parsed from `FIRMWARE_VERSION`
  (1.5.30 → 10530).
- `param2` 2 — hardware version: bits 0–7 board revision (0 = `Control_v0`), bits 8–15 vehicle type
  (1 quad bike, 2 buggy), bits 16–23 the relay board detected at boot (0 none, 1 PCA9557, 2 XL9535),
  bits 24–31 zero.

Any other `param2` SHALL be answered `MAV_RESULT_DENIED`. The firmware and hardware versions are for
display only.

#### Scenario: Caps query on the quad bike
- **WHEN** the quad bike receives `param1` 0, `param2` 0
- **THEN** the ACK SHALL be `ACCEPTED` with `result_param2` 95 (`0x5F`) and `progress` 1 when the
  washer is fitted, and 31 (`0x1F`) when it is not

#### Scenario: Caps query on the buggy
- **WHEN** the buggy with an initialised X6 driver receives the caps query
- **THEN** `result_param2` SHALL be 127 (`0x7F`) with the washer fitted and 63 (`0x3F`) without

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
safety net for a lost query answer. When the capability mask changes at runtime (bit6, washer fitted
switch) the vehicle layer SHALL hand the new mask to `MavlinkInterface`, which SHALL send `QB_CAPS`
with it on the next report (within one report period) and then continue the 10 s period from there;
caps queries SHALL answer the new mask at once.

#### Scenario: Quad bike
- **WHEN** 10 s have elapsed since the last `QB_CAPS` on the quad bike
- **THEN** a `QB_CAPS` with value 16777311 (`0x5F | 1 << 24`) SHALL be sent with the washer fitted,
  and 16777247 (`0x1F | 1 << 24`) with it not fitted

#### Scenario: Buggy with the parking brake
- **WHEN** `QB_CAPS` is sent on the buggy with an initialised X6 driver
- **THEN** its value SHALL be 16777343 (`0x7F | 1 << 24`) with the washer fitted, and 16777279
  (`0x3F | 1 << 24`) with it not fitted

#### Scenario: Washer switch toggled with a GCS connected
- **WHEN** the washer fitted switch is turned on on the quad bike while a ground station is connected
- **THEN** a `QB_CAPS` with value 16777311 SHALL be sent on the next report, without waiting for the
  10 s period
- **AND WHEN** it is turned off again
- **THEN** the next report SHALL carry `QB_CAPS` 16777247 and `param1` 6 SHALL be answered
  `MAV_RESULT_UNSUPPORTED`

### Requirement: Vehicle Command Protocol Extension Rules
The control protocol SHALL be extended without breaking existing clients: a new function SHALL take
the next `param1` and the next capability bit (and, if it is an on/off function, the next
`EFI_STATUS.pt_compensation` state bit); a new query SHALL take the next `param2` under `param1` 0;
a second capability word, if ever needed, SHALL be a new query value. The protocol version (caps
query `progress`, `QB_CAPS` bits 24–31) SHALL change only on a breaking change. Unknown commands and
unknown functions SHALL keep answering `UNSUPPORTED`.

#### Scenario: Adding a function
- **WHEN** a seventh function is added in a later firmware
- **THEN** it SHALL use `param1` 6 and capability bit6 (taken by the headlight washer)
- **AND** the protocol version SHALL stay 1

#### Scenario: Adding a function after the headlight washer
- **WHEN** an eighth function is added in a later firmware
- **THEN** it SHALL use `param1` 7 and capability bit7
- **AND** the protocol version SHALL stay 1

#### Scenario: Timed-pulse function with a state bit
- **WHEN** a function is a timed pulse rather than an on/off state (the headlight washer)
- **THEN** it MAY take the next free `EFI_STATUS.pt_compensation` state bit meaning "running"; the
  headlight washer takes bit4 (`0x10`), bits 2–3 being reserved for the buggy's parking brake on
  every build

