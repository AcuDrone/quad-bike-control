## ADDED Requirements

### Requirement: USB-CDC Serial Console
The debug console SHALL be carried over the ESP32-S3 native USB CDC device on the board's USB-C
connector, not over UART0, because UART0 (GPIO43/44) is shared copper between the programming header
and the future CH9121T wired-LAN chip and is reserved.

#### Scenario: Console routed to native USB
- **WHEN** the firmware is built
- **THEN** `platformio.ini` SHALL define the build flag `ARDUINO_USB_CDC_ON_BOOT=1`
- **AND** `Serial` SHALL resolve to the native USB CDC device
- **AND** all `Debug::print` / `printf` / `println` output SHALL appear on the USB-C serial monitor

#### Scenario: Boot never waits for a USB host
- **WHEN** the board is powered with no USB host attached
- **THEN** the firmware SHALL complete `setup()` and enter `loop()` normally
- **AND** no `while (!Serial)` or equivalent host-wait SHALL be present anywhere in the firmware
- **AND** vehicle operation SHALL be unaffected by the absence of a console

#### Scenario: Early-boot output may be lost
- **WHEN** log output is produced before the host enumerates the CDC endpoint
- **THEN** that output MAY be lost and SHALL NOT be relied upon as a boot diagnostic
- **AND** no safety-relevant decision SHALL depend on early-boot console output being observed

#### Scenario: UART0 remains reserved
- **WHEN** the pin map is read
- **THEN** GPIO43 and GPIO44 SHALL have no firmware function assigned
- **AND** no `Serial0` / UART0 console SHALL be initialized by the firmware
- **AND** the reservation SHALL be documented in `platformio.ini` and `Constants.h` so UART0 is not
  reclaimed before the LAN section is populated
