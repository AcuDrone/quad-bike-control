# ESP32-C6 GPIO Pinout - Quad Bike Control System

**Hardware:** ESP32-C6-DevKitC-1
**Valid GPIO Pins:** 0-23 (excluding GPIO 14 which doesn't exist)
**Total Pins Used:** 21 of 23 (91% utilization)

---

## 📊 Pin Allocation Summary

| Category | Pins Used | Status |
|----------|-----------|--------|
| **Active (In Use)** | 11 | ✅ Implemented |
| **Reserved (Future)** | 12 | 🟡 Planned |
| **Available (UART0)** | 2 | ⚠️ Avoid for compatibility |
| **Invalid** | 1 | ❌ GPIO 14 doesn't exist |

---

## 🟢 ACTIVE PINS - Currently Implemented

### Control Inputs

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **20** | `PIN_SBUS_RX` | UART1 RX | S-bus receiver input (inverted signal) |

### Transmission System

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **2** | `PIN_TRANS_ENCODER_B` | PCNT | Hall sensor channel B (quadrature) |
| **3** | `PIN_TRANS_ENCODER_A` | PCNT | Hall sensor channel A (quadrature) |
| **4** | `PIN_TRANS_RPWM` | LEDC Ch2 | Transmission actuator forward (BTS7960) |
| **5** | `PIN_TRANS_LPWM` | LEDC Ch3 | Transmission actuator reverse (BTS7960) |

### Steering Control

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **13** | `PIN_STEERING_PWM` | LEDC Ch0 | Steering servo (0-180°, 50Hz) |

### Throttle Control

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **1** | `PIN_THROTTLE_PWM` | LEDC Ch1 | Throttle servo (0-180°, 50Hz) |

### Brake System

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **6** | `PIN_BRAKE_RPWM` | LEDC Ch4 | Brake actuator forward (BTS7960) |
| **7** | `PIN_BRAKE_LPWM` | LEDC Ch5 | Brake actuator reverse (BTS7960) |

### Relay Control

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **9** | `PIN_RELAY3` | Digital Out | Relay 3 (safety system) |

---

## 🟡 RESERVED PINS - Future Expansion

### SPI CAN Controller (4 pins)
*For vehicle CAN bus communication*

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **0** | `PIN_CAN_MISO` | SPI MISO | CAN controller SPI input ⚠️ Boot pin |
| **8** | `PIN_CAN_MOSI` | SPI MOSI | CAN controller SPI output 🎨 NeoPixel |
| **22** | `PIN_CAN_CS` | SPI CS | CAN controller chip select |
| **23** | `PIN_CAN_SCK` | SPI SCK | CAN controller SPI clock |

### Gear Selection Sensors (4 pins)
*Physical gear selector position feedback (active-low, pull-up)*

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **10** | `PIN_GEAR_REVERSE` | Digital In | LOW when gear selector in REVERSE (active-low) |
| **11** | `PIN_GEAR_NEUTRAL` | Digital In | LOW when gear selector in NEUTRAL (active-low) |
| **15** | `PIN_GEAR_LOW` | Digital In | LOW when gear selector in LOW (active-low) |
| **16** | `PIN_GEAR_HIGH` | Digital In | LOW when gear selector in HIGH (active-low) |

### Brake Position Sensor (1 pin)
*Brake actuator position feedback*

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **21** | `PIN_BRAKE_SENSOR` | ADC/Analog | Brake position feedback (ADC capable) |

### Relay Control (3 pins)
*Power switching for accessories and safety systems*

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **12** | `PIN_RELAY1` | Digital Out | Relay 1 (main power control) |
| **19** | `PIN_RELAY2` | Digital Out | Relay 2 (accessory power) |
| **9** | `PIN_RELAY3` | Digital Out | Relay 3 (safety system) ✅ Active |

---

## 🔴 AVAILABLE BUT AVOID

| GPIO | Function | Reason to Avoid |
|------|----------|-----------------|
| **17** | UART0 TX | Console/programming - breaks USB serial debugging |
| **18** | UART0 RX | Console/programming - breaks USB serial debugging |

**Recommendation:** Keep these free for USB serial console compatibility during development.

---

## ❌ INVALID PINS

| GPIO | Status | Notes |
|------|--------|-------|
| **14** | Does not exist | ESP32-C6 skips GPIO 14 in hardware |

---

## ⚠️ Special Pin Considerations

### Boot Pin (GPIO 0)
- **Function:** Boot mode control
  - LOW during reset = Download mode (firmware flashing)
  - HIGH during reset = Normal boot (run firmware)
- **Current Use:** CAN_MISO (SPI input)
- **Safety:** ✅ Safe to use during runtime, only affects boot sequence
- **Warning:** Do not pull LOW during power-up/reset or board won't boot

### NeoPixel Pin (GPIO 8)
- **Function:** Onboard RGB LED (WS2812)
- **Current Use:** CAN_MOSI (SPI output)
- **Safety:** ✅ Safe to use, LED may flash during SPI communication (cosmetic only)
- **Effect:** Using this pin for other purposes will drive the onboard LED

### Strapping Pin (GPIO 15)
- **Function:** Controls ROM boot message verbosity
- **Current Use:** GEAR_LOW sensor (reserved)
- **Safety:** ✅ Safe to use, no boot failure risk

---

## 🔌 LEDC PWM Channel Allocation

**Total Channels:** 6 (ESP32-C6 limitation)
**Channels Used:** 6 of 6 (100% allocated)

| Channel | GPIO | Function | Frequency | Resolution | Duty Range |
|---------|------|----------|-----------|------------|------------|
| **0** | 13 | Steering Servo | 50 Hz | 16-bit | 500-2500 µs |
| **1** | 1 | Throttle Servo | 50 Hz | 16-bit | 500-2500 µs |
| **2** | 4 | Transmission RPWM | 10 kHz | 8-bit | 0-255 |
| **3** | 5 | Transmission LPWM | 10 kHz | 8-bit | 0-255 |
| **4** | 6 | Brake RPWM | 10 kHz | 8-bit | 0-255 |
| **5** | 7 | Brake LPWM | 10 kHz | 8-bit | 0-255 |

**Note:** No additional LEDC channels available for expansion.

---

## 📡 UART Allocation

| UART | RX Pin | TX Pin | Baud Rate | Purpose | Status |
|------|--------|--------|-----------|---------|--------|
| **UART0** | 17* | 18* | 115200 | USB Serial Console | Reserved |
| **UART1** | 20 | N/A | 100000 | S-bus Receiver (inverted) | ✅ Active |

*Default pins - not used in project to maintain USB serial compatibility

---

## 📊 Pin Usage by GPIO Number

| GPIO | Function | Direction | Type | Notes |
|------|----------|-----------|------|-------|
| **0** | CAN_MISO | Input | SPI | 🔴 Boot pin - safe during runtime |
| **1** | THROTTLE_PWM | Output | LEDC Ch1 | Servo control |
| **2** | TRANS_ENCODER_B | Input | PCNT | Quadrature encoder |
| **3** | TRANS_ENCODER_A | Input | PCNT | Quadrature encoder |
| **4** | TRANS_RPWM | Output | LEDC Ch2 | BTS7960 forward |
| **5** | TRANS_LPWM | Output | LEDC Ch3 | BTS7960 reverse |
| **6** | BRAKE_RPWM | Output | LEDC Ch4 | BTS7960 forward |
| **7** | BRAKE_LPWM | Output | LEDC Ch5 | BTS7960 reverse |
| **8** | CAN_MOSI | Output | SPI | 🎨 NeoPixel pin |
| **9** | RELAY3 | Output | Digital | Safety relay |
| **10** | GEAR_REVERSE | Input | Digital | 🟡 Reserved |
| **11** | GEAR_NEUTRAL | Input | Digital | 🟡 Reserved |
| **12** | RELAY1 | Output | Digital | 🟡 Reserved |
| **13** | STEERING_PWM | Output | LEDC Ch0 | Servo control |
| **14** | — | — | — | ❌ Does not exist |
| **15** | GEAR_LOW | Input | Digital | 🟡 Reserved |
| **16** | GEAR_HIGH | Input | Digital | 🟡 Reserved |
| **17** | UART0 TX | — | UART | ⚠️ Avoid - console |
| **18** | UART0 RX | — | UART | ⚠️ Avoid - console |
| **19** | RELAY2 | Output | Digital | 🟡 Reserved |
| **20** | SBUS_RX | Input | UART1 | S-bus receiver |
| **21** | BRAKE_SENSOR | Input | ADC | 🟡 Reserved |
| **22** | CAN_CS | Output | SPI | 🟡 Reserved |
| **23** | CAN_SCK | Output | SPI | 🟡 Reserved |

**Legend:**
- ✅ Active - Currently implemented
- 🟡 Reserved - Planned for future use
- ⚠️ Avoid - Keep free for compatibility
- ❌ Invalid - Does not exist in hardware
- 🔴 Boot pin - Special boot function
- 🎨 NeoPixel - Onboard LED

---

## 🛡️ Pin Safety & Boot Compatibility

### ✅ Configuration Status: SAFE

- ✅ No boot pin conflicts (GPIO 0 used for CAN - safe during runtime)
- ✅ No NeoPixel interference (GPIO 8 used for CAN - LED may flash)
- ✅ UART0 pins reserved (GPIO 17, 18 - debugging available)
- ✅ All critical controls on safe, non-strapping pins
- ✅ ADC-capable pin used for analog sensor (GPIO 21)
- ✅ Proper PCNT pins for quadrature encoder (GPIO 2, 3)

### ⚠️ Important Warnings

1. **GPIO 0 (Boot Pin):** Do not pull LOW during power-up/reset
2. **GPIO 8 (NeoPixel):** Onboard LED will flash when CAN SPI is active
3. **GPIO 17, 18 (UART0):** Avoid using to maintain USB serial debugging
4. **GPIO 14:** Does not exist - code will fail if referenced

---

## 🔧 Hardware Configuration Notes

### BTS7960 Motor Drivers (H-Bridge)
- **R_EN and L_EN:** Hardwired to 5V (always enabled)
- **RPWM:** Forward direction PWM control
- **LPWM:** Reverse direction PWM control
- **PWM Frequency:** 10 kHz (prevents audible noise)

### Servo Motors
- **Type:** Standard hobby servos (0-180°)
- **PWM Frequency:** 50 Hz (20 ms period)
- **Pulse Width Range:** 500-2500 µs
- **Center Position:** 1500 µs

### Hall Sensor Encoder
- **Type:** Incremental quadrature encoder
- **Pulses Per Revolution:** 400 (configurable in Constants.h)
- **Interface:** PCNT (Pulse Counter) peripheral
- **Channels:** A and B for direction sensing

### S-bus Receiver
- **Protocol:** FrSky S-bus (inverted UART)
- **Baud Rate:** 100000 bps
- **Data Format:** 25 bytes per frame
- **Signal:** Inverted (requires hardware or software inversion)

---

## 📝 Reference Files

- **Pin Definitions:** `include/Constants.h`
- **Main Firmware:** `src/main.cpp`
- **Servo Controller:** `include/ServoController.h`, `src/ServoController.cpp`
- **Motor Controller:** `include/BTS7960Controller.h`, `src/BTS7960Controller.cpp`
- **Encoder:** `include/EncoderCounter.h`, `src/EncoderCounter.cpp`
- **Web Portal:** `include/WebPortal.h`, `src/WebPortal.cpp`

---

## 🔄 Version History

| Date | Version | Changes |
|------|---------|---------|
| 2026-03-05 | 1.0 | Initial GPIO assignment with boot pin safety fixes |

---

**Document Version:** 1.0
**Last Updated:** March 5, 2026
**Hardware:** ESP32-C6-DevKitC-1
**Framework:** Arduino (PlatformIO)
