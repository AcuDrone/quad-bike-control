# Change: Add MCP23017 I2C GPIO Expander

## Why
All 6 LEDC PWM channels are used and available ESP32-C6 GPIOs are nearly exhausted. Moving relay outputs, gear selector inputs, and brake sensor input to an MCP23017 I2C GPIO expander frees up 8 ESP32 GPIOs for future use (e.g., additional BTS7960 motor driver).

## What Changes
- Add `MCP23017Controller` class wrapping the Adafruit MCP23017 library over I2C
- Provide `digitalWrite()` and `digitalRead()` equivalents for MCP23017 pins
- Migrate 3 relay outputs (currently GPIO 12, 19, 9) to MCP23017 Port A (GPA0–GPA2)
- Migrate 4 gear selector inputs (currently GPIO 10, 11, 15, 18) to MCP23017 Port B (GPB0–GPB3) with pull-ups
- Migrate 1 brake sensor input (currently GPIO 21) to MCP23017 Port B (GPB4) with pull-up
- Update `RelayController` to write via MCP23017 instead of direct GPIO
- Update `TransmissionController` gear reading to read via MCP23017 instead of direct GPIO
- Update brake sensor reading to read via MCP23017
- Update `Constants.h` with I2C pins and MCP23017 pin mapping
- Free ESP32 GPIOs: 9, 10, 11, 12, 15, 18, 19, 21

## Impact
- Affected specs: new `gpio-expander` capability
- Affected code: `Constants.h`, new `MCP23017Controller.h/.cpp`, `RelayController.cpp`, `TransmissionController.cpp`, `main.cpp`, `VehicleController.cpp`
- **No changes** to public APIs of RelayController or TransmissionController
- New I2C dependency: Adafruit MCP23017 library (PlatformIO)
