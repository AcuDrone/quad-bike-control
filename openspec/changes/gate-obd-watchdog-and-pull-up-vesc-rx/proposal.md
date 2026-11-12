# Change: Gate the OBD-II CAN watchdog on the ignition and pull up the VESC UART RX pin

## Why
Two findings from the first fw 1.5.26 event journals (backlog agreed 2026-11-11):

- **VESC UART RX floats.** On aztec the steering VESC runs on UART0 through the Prog header X1
  (GPIO43 TX / GPIO44 RX). Per the board schematic X1 has only the 510 Ω series resistors — no
  pull-up, no capacitor, no ESD — unlike every other interface. The first field journal showed a lost
  byte on this link (`VESC W resync`, 1 in ~45 min). Arduino-ESP32 3.3.7 / IDF 5.5 `uart_set_pin()`
  enables no pull-up on the RX pad (checked in the prebuilt `libesp_driver_uart.a`: its only
  `gpio_pullup_en()` call is on the CTS pin), so nothing defines the idle level between bytes.
- **OBD-II watchdog fires with the ignition OFF.** The 15 s no-data watchdog
  (`CAN_REINIT_NO_DATA_MS`) re-initialises the MCP2515 whenever OBD-II polling gets no reply. The ECU,
  like the Bosch LWS, is powered from the ACC line (Relay_1), so with the ignition OFF the bus is
  legitimately silent: a bench run with ACC off journaled `can_reinit=15` plus `obd_silent` lines
  that look like faults. The LWS watchdog is already gated on the ignition state for exactly this
  reason; the OBD-II one is not.

## What Changes
- `VescMotorDriver::begin()` enables the ESP32 internal pull-up on the VESC RX pin
  (`gpio_set_pull_mode(rx, GPIO_PULLUP_ONLY)`) right after `HardwareSerial::begin()` has claimed the
  pin, reads it back with `gpio_get_io_config()` and journals `VESC W rx_pullup pin=<n> ok=0` only if
  it did not take. The VESC link self-recovery never re-opens the UART (it flushes the RX buffer and
  resets the parser), so `begin()` is the only place. Applied to whichever pin is `PIN_VESC_RX` on
  the branch (GPIO44 on aztec): a weak pull-up on an idle-high UART RX is harmless where an external
  10 kΩ pull-up already exists.
- The OBD-II no-data watchdog runs only while the relay controller's ignition state is `ACC` or
  `IGNITION` — the same flag (`CANController::setLwsPowered()`, fed every loop from
  `RelayController::getIgnitionState()`) and the same rule as the LWS watchdog. The silence is counted
  from the later of the last OBD-II reply (or begin / last re-init) and the last ignition off → on
  edge, so ACC/ON always starts a fresh 15 s window; the 15 s minimum spacing from the last re-init
  of any kind is unchanged. The off → on edge also ends the previous `obd_silent` journal episode, so
  the first silence of a new power cycle is journaled. Decision logic in header-only
  `include/CanWatchdogLaw.h`, native-tested in `test/test_can_watchdog`.
- Unchanged: OBD-II polling itself (requests still go out with the ignition OFF), bus-off and
  send-failure recovery (not gated), the LWS watchdog, `can_reinit` / SNAP / web counters (they
  simply stop growing with the ignition OFF), MAVLink.
- Firmware 1.5.28, Changelog uk + en.

## Impact
- Affected specs: `can-controller` (ADDED "OBD-II No-Data Watchdog Ignition Gating"),
  `vehicle-actuators` (ADDED "VESC UART RX Idle Pull-Up").
- The LWS watchdog requirement lives in the not yet archived change
  `add-event-journal-and-link-recovery` (ADDED "Steering Angle Frame Silence Watchdog"), whose text
  says "The OBD-II watchdog SHALL stay unchanged"; that sentence and the matching line in its
  `design.md` (D10) are amended there to point at this change (same approach as fd6f4ea).
- Affected code: `src/CANController.cpp`, `include/CANController.h`, `include/CanWatchdogLaw.h`
  (new), `test/test_can_watchdog/` (new), `platformio.ini`, `src/VescMotorDriver.cpp`,
  `include/VescMotorDriver.h`, `include/Constants.h`, `include/Changelog.h`.
- No web, API or LittleFS change.
- Risk: if an ECU is ever powered with the board's ignition OFF (manual key / external feed), the
  OBD-II watchdog no longer heals a wedged chip in that state; bus-off recovery still runs, and the
  watchdog resumes 15 s after the board's ignition goes ACC/ON.
