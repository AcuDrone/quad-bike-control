## 1. VESC RX pull-up
- [x] 1.1 `src/VescMotorDriver.cpp` `begin()`: `gpio_set_pull_mode(rxPin, GPIO_PULLUP_ONLY)` after
  `serial_.begin()`; read back with `gpio_get_io_config()`; console line reports `pull-up on|FAILED`;
  `VESC W rx_pullup pin= ok=0` journaled on failure.
- [x] 1.2 Confirm no runtime path re-opens the VESC UART (link recovery only flushes + resets the
  parser) — no other call site needed.
- [x] 1.3 Verify against the installed core (Arduino-ESP32 3.3.7, IDF 5.5): `uart_set_pin()` RX path
  enables no pull-up (disassembly of `libesp_driver_uart.a`).

## 2. OBD-II watchdog ignition gating
- [x] 2.1 `include/CanWatchdogLaw.h`: `obdNoDataDue()` (ignition gate, window from the later of last
  reply / re-init and switch-on, unchanged min spacing, wrap-safe).
- [x] 2.2 `test/test_can_watchdog` + `platformio.ini` `[env:native]` `test_filter` line.
- [x] 2.3 `CANController`: `ignitionOnMs_` stamped and `obdEpisode_` cleared on the off → on edge in
  `setLwsPowered()`; the IDLE-state watchdog uses `obdNoDataDue()` with the same `lwsPowered_` flag.
- [x] 2.4 Amend `add-event-journal-and-link-recovery` (can-controller delta "The OBD-II watchdog SHALL
  stay unchanged", design D10 "Existing (unchanged)") to point at this change.

## 3. Release
- [x] 3.1 `FIRMWARE_VERSION` → `"1.5.28"`; Changelog 1.5.28 uk + en.
- [x] 3.2 `pio run -e esp32-s3-devkitc-1` builds without new warnings; `pio test -e native` passes;
  `openspec validate gate-obd-watchdog-and-pull-up-vesc-rx --strict`.

## 4. Bench verification (no board was attached; nothing here is tested on hardware)
- [ ] 4.1 USB console (SERVO debug on) at boot shows `[VESC] UART0 started: RX=44 (pull-up on)`; no
  `rx_pullup` line in the journal.
- [ ] 4.2 VESC unplugged from X1 / VESC unpowered: GPIO44 measures ≈ 3.3 V (was floating); no garbage
  bytes counted (`vesc_good`/`drop`/`resync` stay flat in SNAP).
- [ ] 4.3 VESC connected: steering works, `vesc_good` rate unchanged, scope on X1 RxD shows clean
  idle-high between frames; long run: `resync` rate compared with the 1-in-45-min baseline.
- [ ] 4.4 Ignition OFF for ≥ 2 min after boot: no `obd_silent`, no `CAN I reinit`, `can_reinit`
  in SNAP stays 0.
- [ ] 4.5 Switch to ACC with the ECU answering: no re-init; with the ECU disconnected: the first
  `obd_silent … -> reinit` appears ≈ 15 s after the switch, not at once.
- [ ] 4.6 ACC → OFF → ACC with the ECU disconnected: the `obd_silent` line is journaled again in the new
  power cycle.
- [ ] 4.7 LWS unplugged with the ECU on (ACC): `lws_silent … -> reinit` every ≥ 5 s as before.
