## 1. Hardware and bench (before the XL9535 table is trusted)
- [ ] 1.1 XL9535-K8V5 wiring: remove jumper J1; VIN from the X15 I2C high-side 3.3 V rail; 5V relay terminal from a separate 5 V supply (≥ 0.6 A); shared GND; no I2C isolation. Record it in `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6
- [x] 1.2 Bench scan: I2C2 shows `0x1A 0x1C 0x20` (default pads) and nothing new in `0x18`..`0x1F` (bench 2026-11-07, XL9535-K8V5 @0x20)
  - Note: with SDA/SCL swapped the module ACKs on no address at all; swapping the two lines fixed it
- [ ] 1.3 Power-up observation (starter load disconnected): no relay clicks from power-on until `begin()` completes
- [x] 1.4 Mapping + polarity check: write Output P0 = `0x01`, then Config P0 = `0xFE`; confirm Relay_1 clicks (active-high); walk `0x02`..`0x80` and record Relay_N ↔ P0.n. Only then mark the identity table as bench-verified in `RelayBoardLaw.h` / `Constants.h` (bench 2026-11-07, XL9535-K8V5 @0x20); all eight relays confirmed via the web relay test (identity, active-high); `RelayBoardLaw.h` comment still says pending
- [x] 1.5 Confirm the Input P0 register reflects the driven output pins (pin read-back) with relays energized (bench 2026-11-07, XL9535-K8V5 @0x20)

## 2. Pure law (host-testable)
- [x] 2.1 `include/RelayBoardLaw.h`: `RELAY_N()`, both Relay_N → bit tables (PCA9557 `Relay.PrjPcb`, XL9535 identity), `relayPortMask(table, set)`, constexpr table-permutation and set-disjoint/union checks, `RelayBoardSetting` / `RelayBoardKind` / `RelayBoardError` enums, `relayBoardDecide(ackMask, setting)` and name/token helpers
- [x] 2.2 `test/test_relay_board/test_main.cpp`: masks for aztec and quad sets on both tables, disjoint/union invariants, every detection row (auto/pca9557/xl9535 × none / 0x1F / one XL / both / two XL), out-of-range setting
- [x] 2.3 Register `test_relay_board` in `[env:native]` `test_filter` (one indented line)

## 3. Drivers
- [x] 3.1 `include/IRelayPort.h` (interface per design a)
- [x] 3.2 `PCA9557Expander` implements `IRelayPort` (`readPins()` = reg `0x00`, `initAllOff()` = output 0, polarity 0, config 0, pins == 0)
- [x] 3.3 `include/XL9535Expander.h` / `src/XL9535Expander.cpp`: P0 only (`0x00`, `0x02`, `0x04`, `0x06`), P1 never written, same mandatory init order, header comment warning about the all-on hazard of config-first

## 4. RelayController
- [x] 4.1 `Constants.h`: replace `RELAY_MASK_*` with relay-number sets `RELAY_SET_*`; add `XL9535_ADDR_MIN/MAX`, `RELAY_RECOVER_MS`, `RELAY_BOARD_NVS_NS`/`KEY`; update the relay and I2C bus-map comments (expected scan `0x1A 0x1C` + `0x1F` or `0x20..0x27`)
- [x] 4.2 `begin()`: read the setting, probe `0x1F` and `0x20`..`0x27`, `relayBoardDecide()`, latch result; on fault log reason + widened `0x18`..`0x27` scan, write nothing; on success build runtime masks (runtime checks) and `initAllOff()`
- [x] 4.3 `writePort()` verifies with `readPins()`; fault path / starter escalation unchanged
- [x] 4.4 `update()` faulted path: single attempt per call kept; every `RELAY_RECOVER_MS` a re-init seeded with the current shadow (output = shadow → polarity 0 → config 0 → pins == shadow; starter bit never seeded — escalation first; approved amendment) + shadow re-write; recovery and detected-board init failure logged once per episode
- [x] 4.5 `relayIoMask()`, `isStarterEnergized()`, `getRelayTestBitsFromMask()` and the test-gate use the runtime table/masks; static_asserts moved to `RelayBoardLaw.h` constexpr checks
- [x] 4.6 Accessors: active board, address, setting at boot, detection error
- [x] 4.7 Update the `RelayController.h` doc block (two boards, relay-number sets, pin verification, recovery)

## 5. Web and telemetry
- [x] 5.1 `VehicleController`: `set_relay_board` command (`value` string; validate; persist first; "takes effect after reboot"); accessors for telemetry
- [x] 5.2 `WebPortal.h` / `TelemetryManager.cpp` / `WebPortal.cpp`: `relay_board`, `relay_board_addr`, `relay_board_cfg`, `relay_board_reboot`, `relay_board_error`; check the JSON document capacity comment
- [x] 5.3 `data/index.html`: "Relay board" card in the System section (auto / PCA9557 / XL9535 selector, reboot note, active-board line, error line, uk/en i18n, ⓘ help); active board shown in the Relay Test card
- [x] 5.4 `WEB_PORTAL_SETUP.md`: relay board setting and the new telemetry fields (it documents the relay test and per-card settings)

## 6. Docs, version
- [x] 6.1 `GPIO_PINOUT_CUSTOM_BOARD_S3.md` §6: XL9535-K8V5 table, wiring, address pads
- [x] 6.2 `FIRMWARE_VERSION` bump and `include/Changelog.h` entry (uk + en)

## 7. Verification
- [x] 7.1 `pio test -e native` — all suites pass, including `test_relay_board`
- [x] 7.2 `pio run` builds (`esp32-s3-devkitc-1`)
- [ ] 7.3 Bench PCA9557 board, `auto`: telemetry "PCA9557 @0x1F (auto)", relay test 1..8 identical to before, ignition/crank unchanged
- [x] 7.4 Bench XL9535 board, `auto` and `xl9535`: "XL9535 @0x20", relay test 1..8 clicks Relay_1..Relay_8 (bench 2026-11-07, XL9535-K8V5 @0x20); `auto` mode only — manual `xl9535` not run
- [ ] 7.5 Ambiguity checks: both boards → `both_found` fault, no relay energized; manual `pca9557` with only the XL9535 → `cfg_mismatch`; second XL9535 address → `multiple_xl9535`; no board → `none_found`; widened scan lists the ACKing addresses
- [ ] 7.6 Brown-out recovery: with a function relay on, interrupt XL9535 VIN briefly; `io_relay_fault` raises, relays stay/fall off, recovery restores the shadow within ~1 s after VIN returns; repeat with the starter test relay on → escalation, ignition latched OFF
- [ ] 7.7 Setting persists across reboot and across an OTA update
