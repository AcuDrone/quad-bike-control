## ADDED Requirements
### Requirement: VESC UART RX Idle Pull-Up
The VESC steering driver SHALL enable the ESP32 internal pull-up (and no pull-down) on its UART RX
pin (`PIN_VESC_RX`; GPIO44 on the Prog header X1 on aztec, which has only a 510 Ω series resistor
and no external pull-up) so the line idles high between bytes and while the VESC is unpowered or
unplugged. The pull-up SHALL be applied after the UART driver has claimed the pin, because the core's
pin assignment does not enable one, and SHALL be verified by reading the pad configuration back.

#### Scenario: Pull-up applied at UART start
- **WHEN** `VescMotorDriver::begin()` opens the UART
- **THEN** the internal pull-up SHALL be enabled on the RX pin after `HardwareSerial::begin()`
- **AND** the USB console line `[VESC] UART… started` SHALL report `pull-up on`

#### Scenario: Pull-up did not take
- **WHEN** the read-back pad configuration shows no pull-up (or a pull-down)
- **THEN** `VESC W rx_pullup pin=<n> ok=0` SHALL be journaled
- **AND** the UART SHALL still be used (structural success, as before)

#### Scenario: Link recovery keeps the pull-up
- **WHEN** the VESC link is lost and recovered
- **THEN** the UART SHALL NOT be re-opened (only the RX buffer is flushed and the parser reset), so the
  pull-up set at start SHALL remain in effect
