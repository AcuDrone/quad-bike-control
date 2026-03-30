## 1. Setup
- [x] 1.1 Add Adafruit MCP23017 library to `platformio.ini`
- [x] 1.2 Define I2C pin constants (SDA, SCL) and MCP23017 pin mapping in `Constants.h`

## 2. MCP23017Controller
- [x] 2.1 Create `MCP23017Controller.h` with class interface (begin, pinMode, digitalWrite, digitalRead, readPort, writePort)
- [x] 2.2 Implement `MCP23017Controller.cpp` wrapping Adafruit MCP23017 library
- [x] 2.3 Instantiate in `main.cpp`, call `begin()` during setup

## 3. Migrate Relays
- [x] 3.1 Update `RelayController::begin()` to accept MCP23017Controller reference instead of GPIO pins
- [x] 3.2 Replace `digitalWrite(relayPin, ...)` calls with `mcp.digitalWrite(mcpPin, ...)` in RelayController
- [x] 3.3 Update relay pin constants in `Constants.h` to MCP23017 pin numbers (GPA0–GPA2)

## 4. Migrate Gear Selector
- [x] 4.1 Update `TransmissionController` gear reading to use `mcp.readPort()` bulk read
- [x] 4.2 Update gear pin constants in `Constants.h` to MCP23017 pin numbers (GPB0–GPB3)

## 5. Migrate Brake Sensor
- [x] 5.1 Update brake sensor reading in `VehicleController`/`main.cpp` to use `mcp.digitalRead()`
- [x] 5.2 Update brake sensor pin constant in `Constants.h` to MCP23017 pin number (GPB4)

## 6. Cleanup
- [x] 6.1 Remove freed ESP32 GPIO pin definitions from `Constants.h` (GPIO 9, 10, 11, 12, 15, 18, 19, 21)
- [x] 6.2 Update `GPIO_PINOUT.md` to reflect MCP23017 pin allocation

## 7. Validation
- [x] 7.1 Build compiles without errors
- [ ] 7.2 Relays switch correctly via MCP23017
- [ ] 7.3 Gear selector reads correctly via MCP23017
- [ ] 7.4 Brake sensor reads correctly via MCP23017
- [ ] 7.5 I2C failure handled gracefully (safe state)
