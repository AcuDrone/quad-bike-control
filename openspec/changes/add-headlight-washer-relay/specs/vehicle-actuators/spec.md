## MODIFIED Requirements
### Requirement: Bosch iBooster Gen2 Service Brake Unit
The aztec service brake SHALL be a Bosch iBooster Gen2 (Tesla Model 3/Y, P/N 1044671-00-x) on
the master cylinder, wired as follows: X150 pin 1 (power) directly to the 12 V battery with 4 mm²
wire, without a diode or converter; pin 9 to ground common with the control board; pin 20 (Wake) to
the ACC / ignition line switched by Relay_1; pin 18 CAN-H and pin 10 CAN-L to the board's CAN bus
(X13). The pedal-travel sensor connector (X158) SHALL stay connected. Pins 25 / 16 (the unit's second
bus) SHALL NOT be connected. No new relay SHALL be added for it; Relay_7 stays reserved (Relay_8 is
the headlight washer pump, see `vehicle-systems` "Headlight Washer Pulse").

#### Scenario: Power path can absorb regeneration
- **WHEN** the rod returns quickly and the unit feeds energy back into its supply
- **THEN** the battery SHALL absorb it, because pin 1 has no diode or converter between it and the
  battery

#### Scenario: Wake follows the ignition
- **WHEN** Relay_1 is on (ACC, IGNITION or CRANKING)
- **THEN** the unit SHALL be awake
- **AND** when Relay_1 is off the unit SHALL sleep and the firmware SHALL NOT be able to wake it

#### Scenario: Bus termination
- **WHEN** the bus is assembled with the board, the engine ECU, the LWS and the iBooster
- **THEN** the resistance between CAN-H and CAN-L with the power off SHALL be about 60 Ω (exactly two
  terminators), measured before the first power-up and recorded in `GPIO_PINOUT_CUSTOM_BOARD_S3.md`
