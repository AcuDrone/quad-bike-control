# ESP32-C6 GPIO Pinout - Quad Bike Control System

**Hardware:** ESP32-C6-DevKitC-1
**Valid GPIO Pins:** 0-23 (excluding GPIO 14 which doesn't exist)
**Total ESP32 Pins Used:** 15 of 23 (65% utilization)
**MCP23017 Pins Used:** 8 of 16 (50% utilization)

---

## Pin Allocation Summary

| Category | Pins Used | Status |
|----------|-----------|--------|
| **Active ESP32** | 15 | Implemented |
| **Active MCP23017** | 8 | Implemented |
| **Freed ESP32** | 6 (GPIO 9-12, 15, 21) | Available for expansion |
| **Available MCP23017** | 8 (GPA3-7, GPB5-7) | Available for expansion |
| **UART0** | 2 | Avoid for compatibility |
| **Invalid** | 1 | GPIO 14 doesn't exist |

---

## ACTIVE ESP32 PINS

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
| **13** | `PIN_STEERING_PWM` | LEDC Ch0 | Steering servo (0-180, 50Hz) |

### Throttle Control

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **1** | `PIN_THROTTLE_PWM` | LEDC Ch1 | Throttle servo (0-180, 50Hz) |

### Brake System

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **6** | `PIN_BRAKE_RPWM` | LEDC Ch4 | Brake actuator forward (BTS7960) |
| **7** | `PIN_BRAKE_LPWM` | LEDC Ch5 | Brake actuator reverse (BTS7960) |

### I2C Bus (MCP23017 GPIO Expander)

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **18** | `PIN_I2C_SDA` | I2C SDA | MCP23017 data line (400 kHz) |
| **19** | `PIN_I2C_SCL` | I2C SCL | MCP23017 clock line (400 kHz) |

### SPI CAN Controller

| GPIO | Function | Type | Description |
|------|----------|------|-------------|
| **0** | `PIN_CAN_MISO` | SPI MISO | CAN controller SPI input (boot pin) |
| **8** | `PIN_CAN_MOSI` | SPI MOSI | CAN controller SPI output (NeoPixel) |
| **22** | `PIN_CAN_CS` | SPI CS | CAN controller chip select |
| **23** | `PIN_CAN_SCK` | SPI SCK | CAN controller SPI clock |

---

## MCP23017 I2C GPIO EXPANDER (Address 0x20)

### Port A - Outputs (Relays)

| MCP Pin | Function | Type | Description |
|---------|----------|------|-------------|
| **GPA0** (0) | `MCP_PIN_RELAY1` | Digital Out | Relay 1 (main power control) |
| **GPA1** (1) | `MCP_PIN_RELAY2` | Digital Out | Relay 2 (accessory power) |
| **GPA2** (2) | `MCP_PIN_RELAY3` | Digital Out | Relay 3 (front light / safety) |
| **GPA3-7** (3-7) | -- | -- | Available for future outputs |

### Port B - Inputs (Sensors, active-low with pull-ups)

| MCP Pin | Function | Type | Description |
|---------|----------|------|-------------|
| **GPB0** (8) | `MCP_PIN_GEAR_REVERSE` | Digital In | Gear selector REVERSE (active-low) |
| **GPB1** (9) | `MCP_PIN_GEAR_NEUTRAL` | Digital In | Gear selector NEUTRAL (active-low) |
| **GPB2** (10) | `MCP_PIN_GEAR_LOW` | Digital In | Gear selector LOW (active-low) |
| **GPB3** (11) | `MCP_PIN_GEAR_HIGH` | Digital In | Gear selector HIGH (active-low) |
| **GPB4** (12) | `MCP_PIN_BRAKE_SENSOR` | Digital In | Brake position sensor (active-low) |
| **GPB5-7** (13-15) | -- | -- | Available for future inputs |

---

## FREED ESP32 PINS (Available for expansion)

*These GPIOs were freed by migrating relays, gear sensors, and brake sensor to MCP23017.*

| GPIO | Previously | Notes |
|------|-----------|-------|
| **9** | RELAY3 | General purpose digital I/O |
| **10** | GEAR_REVERSE | General purpose digital I/O |
| **11** | GEAR_NEUTRAL | General purpose digital I/O |
| **12** | RELAY1 | General purpose digital I/O |
| **15** | GEAR_LOW | Strapping pin - use with care |
| **21** | BRAKE_SENSOR | ADC-capable, general purpose |

---

## AVAILABLE BUT AVOID

| GPIO | Function | Reason to Avoid |
|------|----------|-----------------|
| **16** | UART0 TX | Console/programming - breaks USB serial debugging |
| **17** | UART0 RX | Console/programming - breaks USB serial debugging |

**Recommendation:** Keep these free for USB serial console compatibility during development.

---

## Special Pin Considerations

### Boot Pin (GPIO 0)
- **Function:** Boot mode control
  - LOW during reset = Download mode (firmware flashing)
  - HIGH during reset = Normal boot (run firmware)
- **Current Use:** CAN_MISO (SPI input)
- **Safety:** Safe to use during runtime, only affects boot sequence
- **Warning:** Do not pull LOW during power-up/reset or board won't boot

### NeoPixel Pin (GPIO 8)
- **Function:** Onboard RGB LED (WS2812)
- **Current Use:** CAN_MOSI (SPI output)
- **Safety:** Safe to use, LED may flash during SPI communication (cosmetic only)

### Strapping Pin (GPIO 15)
- **Function:** Controls ROM boot message verbosity
- **Current Use:** Freed (was GEAR_LOW, now on MCP23017)
- **Safety:** If reused, requires proper initialization with ESP-IDF GPIO API

---

## LEDC PWM Channel Allocation

**Total Channels:** 6 (ESP32-C6 limitation)
**Channels Used:** 6 of 6 (100% allocated)

| Channel | GPIO | Function | Frequency | Resolution | Duty Range |
|---------|------|----------|-----------|------------|------------|
| **0** | 13 | Steering Servo | 50 Hz | 16-bit | 500-2500 us |
| **1** | 1 | Throttle Servo | 50 Hz | 16-bit | 500-2500 us |
| **2** | 4 | Transmission RPWM | 10 kHz | 8-bit | 0-255 |
| **3** | 5 | Transmission LPWM | 10 kHz | 8-bit | 0-255 |
| **4** | 6 | Brake RPWM | 10 kHz | 8-bit | 0-255 |
| **5** | 7 | Brake LPWM | 10 kHz | 8-bit | 0-255 |

**Note:** No additional LEDC channels available for expansion.

---

## UART Allocation

| UART | RX Pin | TX Pin | Baud Rate | Purpose | Status |
|------|--------|--------|-----------|---------|--------|
| **UART0** | 17* | 16* | 115200 | USB Serial Console | Reserved |
| **UART1** | 20 | N/A | 100000 | S-bus Receiver (inverted) | Active |

*Default pins - kept free to maintain USB serial compatibility for programming and debugging

---

## Pin Usage by GPIO Number

| GPIO | Function | Direction | Type | Notes |
|------|----------|-----------|------|-------|
| **0** | CAN_MISO | Input | SPI | Boot pin - safe during runtime |
| **1** | THROTTLE_PWM | Output | LEDC Ch1 | Servo control |
| **2** | TRANS_ENCODER_B | Input | PCNT | Quadrature encoder |
| **3** | TRANS_ENCODER_A | Input | PCNT | Quadrature encoder |
| **4** | TRANS_RPWM | Output | LEDC Ch2 | BTS7960 forward |
| **5** | TRANS_LPWM | Output | LEDC Ch3 | BTS7960 reverse |
| **6** | BRAKE_RPWM | Output | LEDC Ch4 | BTS7960 forward |
| **7** | BRAKE_LPWM | Output | LEDC Ch5 | BTS7960 reverse |
| **8** | CAN_MOSI | Output | SPI | NeoPixel pin |
| **9** | -- | -- | -- | Freed (was RELAY3) |
| **10** | -- | -- | -- | Freed (was GEAR_REVERSE) |
| **11** | -- | -- | -- | Freed (was GEAR_NEUTRAL) |
| **12** | -- | -- | -- | Freed (was RELAY1) |
| **13** | STEERING_PWM | Output | LEDC Ch0 | Servo control |
| **14** | -- | -- | -- | Does not exist |
| **15** | -- | -- | -- | Freed (was GEAR_LOW), strapping pin |
| **16** | UART0 TX | -- | UART | Avoid - console |
| **17** | UART0 RX | -- | UART | Avoid - console |
| **18** | I2C_SDA | I/O | I2C | MCP23017 data |
| **19** | I2C_SCL | Output | I2C | MCP23017 clock |
| **20** | SBUS_RX | Input | UART1 | S-bus receiver |
| **21** | -- | -- | -- | Freed (was BRAKE_SENSOR), ADC capable |
| **22** | CAN_CS | Output | SPI | CAN chip select |
| **23** | CAN_SCK | Output | SPI | CAN SPI clock |

---

## Pin Safety & Boot Compatibility

### Configuration Status: SAFE

- No boot pin conflicts (GPIO 0 used for CAN - safe during runtime)
- No NeoPixel interference (GPIO 8 used for CAN - LED may flash)
- UART0 pins reserved (GPIO 16, 17 - debugging available)
- GPIO 15 strapping pin freed (GEAR_LOW moved to MCP23017)
- All critical controls on safe, non-strapping pins
- MCP23017 on I2C (GPIO 18/19) handles relays, gear sensors, brake sensor
- Proper PCNT pins for quadrature encoder (GPIO 2, 3)

### Important Warnings

1. **GPIO 0 (Boot Pin):** Do not pull LOW during power-up/reset
2. **GPIO 8 (NeoPixel):** Onboard LED will flash when CAN SPI is active
3. **GPIO 15 (Strapping Pin):** Freed, but if reused requires proper ESP-IDF GPIO initialization
4. **GPIO 16, 17 (UART0):** Avoid using to maintain USB serial debugging (TX/RX for console)
5. **GPIO 14:** Does not exist - code will fail if referenced

---

## Hardware Configuration Notes

### MCP23017 I2C GPIO Expander
- **Address:** 0x20 (A0=A1=A2=GND)
- **I2C Speed:** 400 kHz (fast mode)
- **Port A (GPA0-7):** Configured as outputs (relays)
- **Port B (GPB0-7):** Configured as inputs with pull-ups (gear sensors, brake sensor)
- **Logic:** Active-low inputs (LOW = sensor active)
- **Bulk Read:** `readPort(1)` reads all gear + brake sensors in one I2C transaction
- **Code Reference:** `MCP23017Controller` in `include/MCP23017Controller.h`

### BTS7960 Motor Drivers (H-Bridge)
- **R_EN and L_EN:** Hardwired to 5V (always enabled)
- **RPWM:** Forward direction PWM control
- **LPWM:** Reverse direction PWM control
- **PWM Frequency:** 10 kHz (prevents audible noise)

### Servo Motors
- **Type:** Standard hobby servos (0-180)
- **PWM Frequency:** 50 Hz (20 ms period)
- **Pulse Width Range:** 500-2500 us
- **Center Position:** 1500 us

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

### Gear Selection Sensors (on MCP23017 Port B)
- **Type:** Digital switches (active-low)
- **Logic:** LOW = gear selected, HIGH = not selected
- **Pull-up Resistors:** MCP23017 internal pull-ups enabled
- **Code Reference:** `TransmissionController::getPhysicalGear()` in `src/TransmissionController.cpp`

---

## Reference Files

- **Pin Definitions:** `include/Constants.h`
- **MCP23017 Controller:** `include/MCP23017Controller.h`, `src/MCP23017Controller.cpp`
- **Main Firmware:** `src/main.cpp`
- **Servo Controller:** `include/ServoController.h`, `src/ServoController.cpp`
- **Motor Controller:** `include/BTS7960Controller.h`, `src/BTS7960Controller.cpp`
- **Encoder:** `include/EncoderCounter.h`, `src/EncoderCounter.cpp`
- **Web Portal:** `include/WebPortal.h`, `src/WebPortal.cpp`

---

**Document Version:** 1.2
**Last Updated:** March 29, 2026
**Hardware:** ESP32-C6-DevKitC-1
**Framework:** Arduino (PlatformIO)
