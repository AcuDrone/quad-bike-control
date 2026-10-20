## 1. Implementation
- [x] 1.1 `include/Constants.h`: VESC pins → GPIO43/44, `VESC_UART_NUM` = 0, comments (X1, R63/R64 removed, X6 free)
- [x] 1.2 `platformio.ini`: pin `-DCORE_DEBUG_LEVEL=0`, update the UART0 comment
- [x] 1.3 `src/main.cpp`: close the ROM putc on UART0 and redirect `esp_log` to USB-CDC before any subsystem starts
- [x] 1.4 Docs: `GPIO_PINOUT_CUSTOM_BOARD_S3.md`, `openspec/project.md`, `include/Changelog.h` (1.2.0, uk + en)

## 2. Verification
- [x] 2.1 `pio run -e esp32-s3-devkitc-1` builds with no new warnings in project code
- [x] 2.2 `openspec validate update-aztec-vesc-uart0 --strict` passes
- [ ] 2.3 Bench: R63/R64 removed; VESC on X1 TxD/RxD/GND; `VESC_OK` = 1, steering moves both ways, no CRC errors over a few minutes with WiFi active
