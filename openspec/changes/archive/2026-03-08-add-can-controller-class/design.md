# Design: CAN Controller Integration

## Overview
This design document covers the integration of MCP2515 CAN controller for reading vehicle OBD-II data and using it for safe transmission control.

## Architecture

### Class Hierarchy
```
CANController
├── Manages MCP2515 hardware communication
├── Implements OBD-II protocol (Mode 01 requests)
└── Provides VehicleData structure

VehicleController (existing)
├── Creates and owns CANController instance
├── Calls canController.update() in control loop
└── Passes CAN data to subsystems

TransmissionController (modified)
├── Receives VehicleData from VehicleController
├── Enforces speed-based gear change safety
└── Requests throttle boost during shifts

TelemetryManager (modified)
├── Receives VehicleData from VehicleController
└── Broadcasts to web clients
```

### Component Responsibilities

#### CANController
**Primary Responsibility**: Hardware abstraction for MCP2515 and OBD-II communication

**Public Interface**:
```cpp
class CANController {
public:
    struct VehicleData {
        uint16_t engineRPM;
        uint8_t vehicleSpeed;
        int8_t coolantTemp;
        int8_t oilTemp;
        uint8_t throttlePosition;
        uint32_t lastUpdateTime;
        bool dataValid;
    };

    bool begin(gpio_num_t csPin, gpio_num_t sckPin, gpio_num_t mosiPin, gpio_num_t misoPin);
    void update();  // Call every loop iteration
    VehicleData getVehicleData() const;
    bool isConnected() const;
    String getStatusString() const;
};
```

**Internal Responsibilities**:
- Initialize MCP2515 via SPI
- Send OBD-II requests (Mode 01)
- Parse OBD-II responses
- Handle CAN errors and timeouts
- Update VehicleData structure

**Polling Strategy**:
- RPM: Every 100ms (critical for gear shifting)
- Speed: Every 100ms (critical for safety)
- Temperature: Every 1000ms (slow-changing)
- Round-robin polling to balance bus load

#### TransmissionController Integration
**Modifications**:
1. Add `setVehicleData(const VehicleData& data)` method
2. Check speed before allowing `setGear()`
3. Request throttle boost during transitions

**Speed Interlock Logic**:
```cpp
bool TransmissionController::canChangeGear(Gear targetGear) const {
    // Allow changes to NEUTRAL always
    if (targetGear == Gear::GEAR_NEUTRAL) {
        return true;
    }

    // Check speed only for non-NEUTRAL changes
    if (vehicleData_.dataValid) {
        if (vehicleData_.vehicleSpeed > TRANS_SPEED_INTERLOCK_THRESHOLD) {
            Serial.println("[TRANS] Gear change blocked: vehicle moving");
            return false;
        }
    } else {
        // CAN data unavailable - check timeout
        if ((millis() - vehicleData_.lastUpdateTime) > TRANS_CAN_TIMEOUT) {
            Serial.println("[TRANS] WARNING: CAN timeout, allowing gear change");
            return true;  // Failsafe: allow after timeout
        }
    }

    return true;
}
```

**Throttle Boost Logic**:
```cpp
void TransmissionController::update() {
    if (isChangingGear_ && throttleBoostEnabled_) {
        // Request VehicleController to increase throttle
        throttleBoostActive_ = true;
    } else {
        throttleBoostActive_ = false;
    }

    // Existing position control...
}
```

#### VehicleController Integration
**Modifications**:
1. Add `CANController` member
2. Call `canController_.update()` in `update()` method
3. Pass vehicle data to `TransmissionController`
4. Handle throttle boost requests

**Data Flow**:
```cpp
void VehicleController::update() {
    // Update CAN data
    canController_.update();
    CANController::VehicleData vehicleData = canController_.getVehicleData();

    // Pass to transmission for safety checks
    transmission_.setVehicleData(vehicleData);

    // Handle throttle boost during gear changes
    if (transmission_.needsThrottleBoost()) {
        float boostThrottle = TRANS_THROTTLE_BOOST_PERCENT;
        applyThrottleBoost(boostThrottle);
    }

    // Existing control logic...
}
```

#### TelemetryManager Integration
**Modifications**:
1. Add vehicle data fields to JSON telemetry
2. Add CAN status indicator

**Telemetry Format**:
```json
{
    "engineRPM": 2500,
    "vehicleSpeed": 15,
    "coolantTemp": 85,
    "oilTemp": 90,
    "throttlePosition": 25,
    "canStatus": "connected",
    "canDataAge": 150
}
```

## OBD-II Protocol Implementation

### Mode 01 (Current Data) PIDs
| PID | Description | Bytes | Formula | Update Rate |
|-----|-------------|-------|---------|-------------|
| 0x0C | Engine RPM | 2 | ((A*256)+B)/4 | 100ms |
| 0x0D | Vehicle Speed | 1 | A | 100ms |
| 0x05 | Coolant Temp | 1 | A - 40 | 1000ms |
| 0x5C | Oil Temp | 1 | A - 40 | 1000ms |
| 0x11 | Throttle Position | 1 | A * 100/255 | 100ms |

### Request/Response Format
**Request** (Mode 01, PID 0x0C - RPM):
```
[0x02] [0x01] [0x0C] [0x00] [0x00] [0x00] [0x00] [0x00]
 ^^^    ^^^    ^^^
 len    mode   PID
```

**Response**:
```
[0x04] [0x41] [0x0C] [0x1A] [0xF8] [0x00] [0x00] [0x00]
 ^^^    ^^^    ^^^    ^^^    ^^^
 len    resp   PID    data_A data_B

RPM = ((0x1A * 256) + 0xF8) / 4 = (6656 + 248) / 4 = 1726 RPM
```

### Error Handling
1. **No response timeout**: 1000ms, set `dataValid = false`
2. **CAN bus errors**: Reset MCP2515, retry 3 times
3. **Invalid data**: Range check, discard outliers
4. **Stale data**: Mark invalid after 5 seconds

## Constants and Configuration

### New Constants (Constants.h)
```cpp
// CAN Controller Configuration
#define CAN_SPEED               CAN_500KBPS  // 500 kbps
#define CAN_POLL_INTERVAL_RPM   100          // ms - RPM/Speed polling
#define CAN_POLL_INTERVAL_TEMP  1000         // ms - Temperature polling
#define CAN_RESPONSE_TIMEOUT    1000         // ms - OBD-II response timeout
#define CAN_DATA_STALE_TIMEOUT  5000         // ms - Mark data invalid

// Transmission Safety
#define TRANS_SPEED_INTERLOCK_THRESHOLD  5   // km/h - Block gear changes above
#define TRANS_CAN_TIMEOUT                5000 // ms - Allow gear change if CAN fails
#define TRANS_THROTTLE_BOOST_PERCENT     20   // % - Throttle during gear change
#define TRANS_THROTTLE_BOOST_DURATION    500  // ms - Boost duration
```

## SPI Bus Sharing
MCP2515 shares SPI bus with potential future devices:
- **CS Pins**: Each device has separate CS (chip select)
- **MOSI/MISO/SCK**: Shared bus lines
- **MCP2515 CS**: GPIO 22 (PIN_CAN_CS)

**SPI Configuration**:
- Frequency: 10 MHz (MCP2515 max)
- Mode: SPI_MODE0
- Bit order: MSBFIRST

## Safety Considerations

### Fail-Safe Behavior
| Failure Mode | System Response | Timeout |
|--------------|-----------------|---------|
| CAN init fails | Log error, continue without CAN | N/A |
| No OBD-II response | Set `dataValid = false` | 1000ms |
| Stale CAN data | Allow gear changes (fail-open) | 5000ms |
| MCP2515 hardware error | Reset controller, retry | 100ms |

### Speed Interlock Safety
- **Default**: Block gear changes if speed > 5 km/h
- **Timeout Fallback**: Allow after 5s if CAN fails
- **Emergency Override**: Manual override via web portal (future)

### Throttle Boost Safety
- **Duration Limit**: 500ms maximum
- **Magnitude Limit**: 20% throttle increase
- **Conflict Resolution**: SBUS throttle overrides boost
- **Emergency Stop**: Disable boost if brake applied

## Performance Considerations

### CPU Load
- SPI transaction: ~1ms per request
- OBD-II round-trip: ~100ms (waiting for response)
- Polling rate: 10 Hz (RPM/Speed) + 1 Hz (Temp) = 11 requests/sec
- **Estimated CPU load**: <5%

### Memory Usage
- CANController class: ~200 bytes
- MCP_CAN library: ~2 KB
- CAN message buffers: ~256 bytes
- **Total**: ~2.5 KB

### CAN Bus Load
- OBD-II request: 8 bytes @ 500 kbps = 160 μs
- Response: 8 bytes = 160 μs
- Per-request overhead: ~320 μs
- 11 requests/sec = 3520 μs/sec = 0.35% bus load
- **Impact**: Negligible

## Testing Plan

### Phase 1: Hardware Verification
1. MCP2515 SPI communication test
2. CAN bus voltage levels (CANH/CANL)
3. Loopback test (MCP2515 internal)

### Phase 2: OBD-II Communication
1. Connect to vehicle OBD-II port
2. Verify PID responses (RPM, speed, temp)
3. Measure response times and reliability

### Phase 3: Transmission Integration
1. Test speed interlock with vehicle moving
2. Verify timeout fallback (disconnect CAN)
3. Test throttle boost during gear changes

### Phase 4: Telemetry Integration
1. Verify data appears in web portal
2. Check update rates and accuracy
3. Test CAN status indicators

## Future Enhancements
1. **CAN Write Support**: Send commands to vehicle (future)
2. **Diagnostic Trouble Codes (DTCs)**: Read error codes (Mode 03)
3. **Freeze Frame Data**: Capture data during errors (Mode 02)
4. **Multiple ECUs**: Support multi-ECU vehicles
5. **Custom CAN Messages**: Beyond OBD-II standard
6. **CAN Sniffing Mode**: Monitor all bus traffic for debugging
