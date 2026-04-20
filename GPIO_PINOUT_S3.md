# ESP32-S3 GPIO Pinout - Quad Bike Control System

**Hardware:** ESP32-S3-DevKitC-1 v1.1
**Reference:** [Official User Guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html)
**Pin Definitions:** `include/Constants.h`

---

## Complete Pin Assignment — All GPIO

| GPIO | Header | Project Assignment | Notes |
|------|--------|--------------------|-------|
| **0** | J3-14 | FREE | ⚠ Strapping: HIGH = normal boot, LOW = download mode |
| **1** | J3-4 | `PIN_TRANS_ENCODER_B` — PCNT | Transmission hall sensor B |
| **2** | J3-5 | `PIN_TRANS_ENCODER_A` — PCNT | Transmission hall sensor A |
| **3** | J1-13 | `PIN_THROTTLE_PWM` — LEDC Ch1 | Throttle servo 50 Hz |
| **4** | J1-4 | `PIN_TRANS_LPWM` — LEDC Ch3 | Transmission actuator reverse |
| **5** | J1-5 | `PIN_TRANS_RPWM` — LEDC Ch2 | Transmission actuator forward |
| **6** | J1-6 | `PIN_BRAKE_LPWM` — LEDC Ch5 | Brake actuator reverse |
| **7** | J1-7 | `PIN_BRAKE_RPWM` — LEDC Ch4 | Brake actuator forward |
| **8** | J1-12 | `PIN_SBUS_RX` — UART1 RX | S-bus receiver (inverted) |
| **9** | J1-15 | FREE | ADC1 / touch capable |
| **10** | J1-16 | `PIN_CAN_CS` — SPI CS | MCP2515 chip select |
| **11** | J1-17 | `PIN_CAN_MOSI` — SPI MOSI | MCP2515 data out |
| **12** | J1-18 | `PIN_CAN_SCK` — SPI CLK | MCP2515 clock |
| **13** | J1-19 | `PIN_CAN_MISO` — SPI MISO | MCP2515 data in |
| **14** | J1-20 | `PIN_BRAKE_SENSOR` — Digital In | Brake position sensor (active-low) |
| **15** | J1-8 | FREE | ADC2, 32 kHz crystal capable |
| **16** | J1-9 | FREE | ADC2, 32 kHz crystal capable |
| **17** | J1-10 | `PIN_STEER_RPWM` — Digital Out | Steering right (BTS7960 full speed) |
| **18** | J1-11 | `PIN_STEER_LPWM` — Digital Out | Steering left (BTS7960 full speed) |
| **19** | J3-20 | `PIN_GEAR_REVERSE` — Digital In | ⚠ USB-OTG D− — do not enable USB-OTG |
| **20** | J3-19 | `PIN_GEAR_NEUTRAL` — Digital In | ⚠ USB-OTG D+ — do not enable USB-OTG |
| **21** | J3-18 | `PIN_GEAR_LOW` — Digital In | Gear selector LOW (active-low) |
| **35** | J3-13 | FREE | |
| **36** | J3-12 | `PIN_RELAY1` — Digital Out | Relay 1 — main power |
| **37** | J3-11 | `PIN_RELAY2` — Digital Out | Relay 2 — starter |
| **38** | J3-10 | `PIN_RELAY3` — Digital Out | ⚠ Drives onboard WS2812B RGB LED too |
| **39** | J3-9 | FREE | JTAG TCK |
| **40** | J3-8 | FREE | JTAG TDO |
| **41** | J3-7 | `PIN_STEER_ENCODER_B` — PCNT Unit 1 | ⚠ JTAG TDI — disables hardware JTAG |
| **42** | J3-6 | `PIN_STEER_ENCODER_A` — PCNT Unit 1 | ⚠ JTAG TMS — disables hardware JTAG |
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
| **9** | J1-15 | ADC1 / capacitive touch |
| **15** | J1-8 | General I/O, ADC2, 32 kHz crystal |
| **16** | J1-9 | General I/O, ADC2, 32 kHz crystal |
| **35** | J3-13 | General I/O, SPI |
| **39** | J3-9 | General I/O (JTAG TCK if JTAG needed) |
| **40** | J3-8 | General I/O (JTAG TDO if JTAG needed) |
| **45** | J3-15 | General I/O — set HIGH before use (strapping) |
| **46** | J1-14 | General I/O — set HIGH before use (strapping) |
| **48** | J3-16 | General I/O |

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
| 2 | 5 | Transmission RPWM | 10 kHz | 8-bit |
| 3 | 4 | Transmission LPWM | 10 kHz | 8-bit |
| 4 | 7 | Brake RPWM | 10 kHz | 8-bit |
| 5 | 6 | Brake LPWM | 10 kHz | 8-bit |

---

## UART Allocation

| UART | RX | TX | Baud | Purpose |
|------|----|----|------|---------|
| UART0 | 44 | 43 | 115200 | USB serial console — reserved |
| UART1 | 8 | — | 100000 | S-bus receiver (inverted signal) |

---

## ⚠ Warnings

| GPIO | Issue |
|------|-------|
| **0** | Strapping pin — must be HIGH at boot for normal operation |
| **19, 20** | USB-OTG D−/D+ — do not enable USB-OTG peripheral |
| **38** | Drives onboard WS2812B RGB LED — will flicker when relay3 toggles |
| **41, 42** | JTAG TDI/TMS — hardware JTAG unavailable while these are used as PCNT |
| **43, 44** | UART0 console — must stay free for USB serial debugging |
| **45, 46** | Strapping pins — read at reset; safe after boot but initialize carefully |
