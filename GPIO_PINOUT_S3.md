# ESP32-S3 GPIO Pinout - Quad Bike Control System

**Hardware:** ESP32-S3-DevKitC-1 v1.1 N16R8 (OPI PSRAM not used — `qio_qspi` mode; GPIO 33-37 free)
**Reference:** [Official User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)
**Pin Definitions:** `include/Constants.h`

---

## Complete Pin Assignment — All GPIO

| GPIO | Header | Project Assignment | Notes |
|------|--------|--------------------|-------|
| **0** | J3-14 | FREE | ⚠ Strapping: HIGH = normal boot, LOW = download mode |
| **1** | J3-4 | FREE | (was documented as transmission encoder — never in code) |
| **2** | J3-5 | FREE | (was documented as transmission encoder — never in code) |
| **3** | J1-13 | `PIN_THROTTLE_PWM` — LEDC Ch1 | Throttle servo 50 Hz |
| **4** | J1-4 | `PIN_VESC_TX` — UART2 TX | VESC steering driver: ESP TX → VESC RX (115200) |
| **5** | J1-5 | `PIN_VESC_RX` — UART2 RX | VESC steering driver: ESP RX ← VESC TX (115200) |
| **6** | J1-6 | `PIN_BRAKE_LPWM` — LEDC Ch5 | Brake actuator reverse |
| **7** | J1-7 | `PIN_BRAKE_RPWM` — LEDC Ch4 | Brake actuator forward |
| **8** | J1-12 | `PIN_MAVLINK_RX` — UART1 RX | MAVLink ↔ Pixhawk TELEM2 (non-inverted) |
| **9** | J1-15 | `PIN_TRANS_SERVO` — LEDC Ch2 | Transmission servo (gear selector) |
| **10** | J1-16 | `PIN_CAN_CS` — SPI CS | MCP2515 chip select |
| **11** | J1-17 | `PIN_CAN_MOSI` — SPI MOSI | MCP2515 data out |
| **12** | J1-18 | `PIN_CAN_SCK` — SPI CLK | MCP2515 clock |
| **13** | J1-19 | `PIN_CAN_MISO` — SPI MISO | MCP2515 data in |
| **14** | J1-20 | `PIN_BRAKE_SENSOR` — Digital In | Brake position sensor (active-low) |
| **15** | J1-8 | `PIN_MAVLINK_TX` — UART1 TX | MAVLink ↔ Pixhawk TELEM2 (non-inverted) |
| **16** | J1-9 | FREE | ADC2, 32 kHz crystal capable |
| **17** | J1-10 | `PIN_STEER_RPWM` — RESERVED/FREED | Freed by VESC change (was steering RPWM, LEDC Ch6) — reserved, not reused |
| **18** | J1-11 | `PIN_STEER_LPWM` — RESERVED/FREED | Freed by VESC change (was steering LPWM, LEDC Ch7) — reserved, not reused |
| **19** | J3-20 | `PIN_GEAR_REVERSE` — Digital In | ⚠ USB-OTG D− — do not enable USB-OTG |
| **20** | J3-19 | `PIN_GEAR_NEUTRAL` — Digital In | ⚠ USB-OTG D+ — do not enable USB-OTG |
| **21** | J3-18 | `PIN_GEAR_LOW` — Digital In | Gear selector LOW (active-low) |
| **35** | J3-13 | FREE | Was OPI PSRAM SPIIO6 — free with qio_qspi |
| **36** | J3-12 | `PIN_RELAY1` — Digital Out | Relay 1 — main power |
| **37** | J3-11 | `PIN_RELAY2` — Digital Out | Relay 2 — starter |
| **38** | J3-10 | `PIN_RELAY3` — Digital Out | ⚠ Drives onboard WS2812B RGB LED too |
| **39** | J3-9 | `PIN_WHEEL_LOCK` — Digital Out | Relay 4 — front-wheel lock (⚠ JTAG TCK; JTAG unused) |
| **40** | J3-8 | FREE | JTAG TDO |
| **41** | J3-7 | `PIN_STEER_SDA` — I2C SDA | AS5600 steering angle sensor (0x36) — ⚠ JTAG TDI, disables hardware JTAG |
| **42** | J3-6 | `PIN_STEER_SCL` — I2C SCL | AS5600 steering angle sensor (0x36) — ⚠ JTAG TMS, disables hardware JTAG |
| **43** | J3-2 | UART0 TX — reserved | USB serial console — do not use |
| **44** | J3-3 | UART0 RX — reserved | USB serial console — do not use |
| **45** | J3-15 | FREE | ⚠ Strapping: VDD_SPI voltage level |
| **46** | J1-14 | FREE | ⚠ Strapping: ROM log verbosity |
| **47** | J3-17 | `PIN_GEAR_HIGH` — Digital In | Gear selector HIGH (active-low) |
| **48** | J3-16 | FREE | |

---

## Free Pins Summary

| GPIO | Header | Best Use |
|------|--------|----------|
| **1** | J3-4 | General I/O, ADC1 |
| **2** | J3-5 | General I/O, ADC1 |
| **16** | J1-9 | General I/O, ADC2, 32 kHz crystal |
| **35** | J3-13 | General I/O, SPI |
| **40** | J3-8 | General I/O (JTAG TDO if JTAG needed) |
| **45** | J3-15 | General I/O — set HIGH before use (strapping) |
| **46** | J1-14 | General I/O — set HIGH before use (strapping) |
| **48** | J3-16 | General I/O |

**Reserved (freed, not reused):** GPIO 17 (J1-10) and GPIO 18 (J1-11) — former BTS7960 steering PWM pins (LEDC Ch6/Ch7), freed when steering moved to the VESC over UART2. Left reserved so the wiring change stays localized.

---

## Power & Non-GPIO Pins

| Pin | Header | Type | Notes |
|-----|--------|------|-------|
| 3V3 | J1-1, J1-2 | Power out | 3.3 V |
| 5V | J1-21 | Power in/out | 5 V |
| RST/EN | J1-3 | Input | Active-low chip enable |
| GND | J1-22, J3-1, J3-21, J3-22 | Ground | |

---

## LEDC PWM Channels

| Channel | GPIO | Function | Frequency | Resolution |
|---------|------|----------|-----------|------------|
| 1 | 3 | Throttle Servo | 50 Hz | 14-bit |
| 2 | 9 | Transmission Servo (gear selector) | 50 Hz | 14-bit |
| 3 | — | FREE | — | — |
| 4 | 7 | Brake RPWM | 10 kHz | 8-bit |
| 5 | 6 | Brake LPWM | 10 kHz | 8-bit |
| 6 | — | RESERVED/FREED (was steering RPWM on GPIO17) | — | — |
| 7 | — | RESERVED/FREED (was steering LPWM on GPIO18) | — | — |

---

## UART Allocation

| UART | RX | TX | Baud | Purpose |
|------|----|----|------|---------|
| UART0 | 44 | 43 | 115200 | USB serial console — reserved |
| UART1 | 8 | 15 | 115200 | MAVLink 2 ↔ Pixhawk TELEM2 (non-inverted) |
| UART2 | 5 | 4 | 115200 | VESC steering driver (Flipsky 75200, brushed-DC mode) |

## I2C Allocation

| Bus | SDA | SCL | Purpose |
|-----|-----|-----|---------|
| Wire | 41 | 42 | AS5600 absolute steering angle sensor (addr 0x36) |

---

## ⚠ Warnings

| GPIO | Issue |
|------|-------|
| **0** | Strapping pin — must be HIGH at boot for normal operation |
| **19, 20** | USB-OTG D−/D+ — do not enable USB-OTG peripheral |
| **38** | Drives onboard WS2812B RGB LED — will flicker when relay3 toggles |
| **41, 42** | JTAG TDI/TMS — hardware JTAG unavailable while these are used as the AS5600 I2C bus |
| **43, 44** | UART0 console — must stay free for USB serial debugging |
| **45, 46** | Strapping pins — read at reset; safe after boot but initialize carefully |
