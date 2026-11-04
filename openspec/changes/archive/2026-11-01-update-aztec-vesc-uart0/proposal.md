# Change: Aztec steering VESC on UART0 (Prog header X1)

## Why
X6 (GPIO4/5) is a BTS7960 port and is needed for a fourth BTS7960 driver. On the aztec board R63/R64
(the 100 Ω links from GPIO43/44 to the CH9121T LAN chip) have been removed, so UART0 now reaches only
the Prog header X1 (through 510 Ω R13/R14) and can carry the steering VESC instead.

## What Changes
- `PIN_VESC_TX` = GPIO43, `PIN_VESC_RX` = GPIO44, `VESC_UART_NUM` = 0 (was GPIO4/5, UART2). Baud stays
  115200. GPIO4/5 (X6) become free, reserved for a 4th BTS7960. MAVLink stays on UART1.
- Runtime logging is kept off UART0: `-DCORE_DEBUG_LEVEL=0` pinned in `platformio.ini` (the Arduino
  `log_x()` path prints through the ROM putc, UART0 by default), the ROM putc is closed in `setup()`
  (`uartSetDebug(NULL)`), and ESP-IDF `esp_log` output (IDF console is UART0 + USB-Serial-JTAG in the
  prebuilt sdkconfig) is redirected to the USB-CDC console with `esp_log_set_vprintf`.
- Docs: pinout (GPIO table, X1/X6, UART table, warnings, migration table), `platformio.ini` comment,
  `openspec/project.md`, Changelog 1.2.0 (uk + en, one line).
- **BREAKING (hardware):** the board must have R63/R64 removed and the VESC wired to X1 TxD/RxD/GND.
  X1 can no longer be used for UART flashing (USB-C only); the ROM bootloader still prints on GPIO43
  at reset, which the VESC ignores (no valid frame).

## Impact
- Affected specs: `vehicle-actuators` (Hardware Configuration, VESC Brushed-DC Steering Driver),
  `debug-logging` (USB-CDC Serial Console)
- Affected code: `include/Constants.h`, `src/main.cpp`, `src/MavlinkInterface.cpp` (comment),
  `platformio.ini`, `include/Changelog.h`, `GPIO_PINOUT_CUSTOM_BOARD_S3.md`, `openspec/project.md`
- The `Hardware Configuration` delta is based on the version in `add-aztec-dual-as5600-steering`;
  archive that change first.
