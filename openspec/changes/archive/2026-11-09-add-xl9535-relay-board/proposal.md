# Change: Support the XL9535-K8V5 relay board alongside the PCA9557 relay board

## Why
The custom PCA9557 relay board (`0x1F`, scrambled IO routing) is being joined by an off-the-shelf
XL9535-K8V5 (8 relays, PCA9535-compatible expander at `0x20`..`0x27`). Today the relay driver, the
address and every `RELAY_MASK_*` are compile-time PCA9557 facts, so the new board cannot be used
without a fork, and the current output-latch read-back cannot see a silent expander reset (the
XL9535 has no RESET pin and loses its configuration on a VIN brown-out while the latch still reads
back fine).

## What Changes
- New abstract `IRelayPort` (same pattern as `IMotorDriver`); `PCA9557Expander` implements it and a
  new `XL9535Expander` implements it on port P0 only (P1 never written).
- Mandatory hardware-safe `initAllOff()`: Output = 0x00, then Polarity = 0x00, then Config = 0x00,
  then a pin read-back — never Config first (the XL9535 latch powers up 0xFF = all relays on).
- Write verification reads the **pins** (input register) instead of the output latch, for both
  chips; a mismatch takes the existing fault path and starter escalation, and a timed recovery
  (`RELAY_RECOVER_MS`, like `BOARD_INPUT_RECOVER_MS`) re-runs `initAllOff()` and re-writes the shadow.
- Runtime board selection: persisted NVS setting `relay_board` ∈ {`auto`, `pca9557`, `xl9535`},
  default `auto`. Detection runs once in `RelayController::begin()` before any relay write and is
  latched until reboot. Any ambiguity (both boards, none, more than one XL9535 address, manual
  selection contradicting the bus) is a relay fault with every relay de-energized. The failure scan
  widens to `0x18`..`0x27`.
- `RELAY_MASK_*` stop being compile-time constants: `Constants.h` keeps the per-vehicle function
  assignment as **relay-number sets**; `begin()` builds the port masks through the detected board's
  Relay_N → bit table (PCA9557: the `Relay.PrjPcb` table; XL9535: identity). The static_asserts
  become constexpr checks on both tables plus runtime checks in `begin()`.
- Web UI: "Relay board" selector (applies after reboot) in the System section; new web command
  `set_relay_board`; telemetry `relay_board`, `relay_board_addr`, `relay_board_cfg`,
  `relay_board_reboot`, `relay_board_error`.
- New pure header `include/RelayBoardLaw.h` + native suite `test/test_relay_board`.
- Unchanged: Relay_1..Relay_8 numbering seen by the web/MAVLink, web relay test, MAVLink
  wheel-lock plumbing, ignition state machine, starter escalation, `io_relay_fault`.
- Hardware (not firmware): on the XL9535-K8V5 remove J1, feed VIN from the X15 I2C high-side 3.3 V
  rail, feed the 5V relay terminal separately (~0.6 A); shared GND, no I2C isolation.

## Impact
- Affected specs: gpio-expander, mavlink-interface, vehicle-systems, web-control, web-telemetry
- Affected code: `include/IRelayPort.h` (new), `include/XL9535Expander.h` + `src/XL9535Expander.cpp`
  (new), `include/RelayBoardLaw.h` (new), `include/PCA9557Expander.h`, `src/PCA9557Expander.cpp`,
  `include/RelayController.h`, `src/RelayController.cpp`, `include/Constants.h`, `src/main.cpp`,
  `include/VehicleController.h`, `src/VehicleController.cpp`, `include/WebPortal.h`,
  `src/WebPortal.cpp`, `src/TelemetryManager.cpp`, `data/index.html`, `include/Changelog.h`,
  `platformio.ini` (`test_filter`), `test/test_relay_board/` (new), `WEB_PORTAL_SETUP.md`,
  `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6
- NVS: new namespace `relay`, key `board` (uint8); the `nvs` partition is outside the OTA app slots
  (`partitions_16mb.csv`), so the setting survives OTA. Older firmware ignores the key.
