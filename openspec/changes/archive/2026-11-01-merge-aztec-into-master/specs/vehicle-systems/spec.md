## ADDED Requirements
### Requirement: Quad Bike Relay Assignment With Fitted Front-Wheel Lock
On the quad bike (`master`) build the relay board SHALL use the quad assignment:
- ignition/ECU line = Relay_1 + Relay_2, paralleled (`RELAY_MASK_IGNITION` `0b00110000`);
- starter = Relay_3 (`0b01000000`);
- front light = Relay_4 (`0b10000000`);
- front-wheel lock = Relay_5 + Relay_6, paralleled (`RELAY_MASK_WHEEL_LOCK` `0b00001100`);
- Relay_7 / Relay_8 spare (`RELAY_MASK_RESERVED` `0b00000011`).

The front-wheel lock SHALL be fitted (`RELAY_WHEEL_LOCK_FITTED` = 1). It replaces, for this build
only, the aztec buggy's "Front-Wheel Lock Not Fitted" behaviour.

#### Scenario: Wheel lock from MAVLink channel 7
- **WHEN** MAVLink servo channel 7 is above 1520 µs (e.g. 2000)
- **THEN** Relay_5 and Relay_6 SHALL both be energized, `getWheelLock()` SHALL return `true`,
  telemetry `wheel_lock_on` SHALL be `true` and `EFI_DIGITAL_FLAG_WHEEL_LOCK` SHALL be set
- **AND WHEN** channel 7 returns to 1000 µs
- **THEN** both relays SHALL be released and the lock SHALL report unlocked

#### Scenario: Wheel lock reported as fitted
- **WHEN** telemetry is formatted on the quad build
- **THEN** `wheel_lock_fitted` SHALL be `true` and the web wheel-lock toggle SHALL be usable

#### Scenario: Fail-safe all-off keeps the wheel lock
- **WHEN** `RelayController::allOff()` runs
- **THEN** ignition, starter and light SHALL be switched off
- **AND** the wheel lock SHALL hold its last commanded state

#### Scenario: Relay test respects the wheel lock
- **WHEN** a web relay test asks to energize any relay while the wheel lock is engaged
- **THEN** the request SHALL be refused with a reason naming the wheel lock
- **AND WHEN** a lock command arrives while test relays are on
- **THEN** all test relays SHALL be switched off before the lock is applied
- **AND** the 2 s starter test cap SHALL apply to Relay_3

### Requirement: Quad Bike One-Line-Per-Gear Sensing
On the quad bike (`master`) build each gear SHALL be sensed on its own opto input line on the
input expander: NEUTRAL = port bit 0, REVERSE = bit 1, LOW = bit 6, HIGH = bit 7, engaged = HIGH.
Exactly one engaged line SHALL decode to its gear. No engaged line, or more than one, SHALL decode
to `GEAR_UNKNOWN`. The aztec 3-bit gear code table SHALL NOT be built on this build.

#### Scenario: Single line engaged
- **WHEN** only port bit 6 reads HIGH among bits 0, 1, 6 and 7
- **THEN** the physical gear SHALL be LOW

#### Scenario: Ambiguous or empty reading
- **WHEN** none of bits 0, 1, 6, 7 reads HIGH, or two or more do
- **THEN** the physical gear SHALL be `GEAR_UNKNOWN`

#### Scenario: No gear-code calibration on the quad
- **WHEN** telemetry is formatted on the quad build
- **THEN** `gear_code`, `gear_sensed`, `gear_codes` and `gear_codes_custom` SHALL be absent
- **AND** the web "Gear Sensor Codes" card SHALL be hidden
- **AND** a `set_gear_code` or `reset_gear_codes` command SHALL be answered with failure
  "Gear codes not used on this vehicle"

### Requirement: Quad Bike Steering VESC on UART2
On the quad bike (`master`) build the steering VESC SHALL be driven over UART2 on connector X6
(GPIO4 TX → VESC RX, GPIO5 RX ← VESC TX, via the BSS138 level shifters, 115200 baud). UART0
(GPIO43/44) SHALL stay connected to the CH9121T and SHALL carry no console and no runtime log
output.

#### Scenario: VESC link on X6
- **WHEN** the firmware starts on the quad build
- **THEN** the VESC driver SHALL open `UART_NUM_2` on GPIO5 (RX) / GPIO4 (TX)
- **AND** a valid `COMM_GET_VALUES` reply SHALL set the steering driver healthy

#### Scenario: No log output on UART0
- **WHEN** an ESP-IDF component or the Arduino core logs at runtime
- **THEN** the output SHALL go to the USB-CDC console only, never to UART0
