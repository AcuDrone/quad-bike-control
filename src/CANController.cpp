#include "CANController.h"
#include <SPI.h>

CANController::CANController()
    : mcp_can_(nullptr),
      initialized_(false),
      lastRPMPoll_(0),
      lastTempPoll_(0),
      retryCount_(0) {
    // Initialize vehicle data to safe defaults
    vehicleData_.engineRPM = 0;
    vehicleData_.vehicleSpeed = 0;
    vehicleData_.coolantTemp = 0;
    vehicleData_.oilTemp = 0;
    vehicleData_.throttlePosition = 0;
    vehicleData_.lastUpdateTime = 0;
    vehicleData_.dataValid = false;
}

CANController::~CANController() {
    if (mcp_can_ != nullptr) {
        delete mcp_can_;
        mcp_can_ = nullptr;
    }
}

bool CANController::begin(gpio_num_t csPin, gpio_num_t sckPin, gpio_num_t mosiPin, gpio_num_t misoPin) {
    Serial.println("[CAN] Initializing MCP2515 on SPI...");

    // Initialize SPI pins before MCP_CAN tries to use them
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);  // CS idle high
    pinMode(sckPin, OUTPUT);
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);

    // Explicitly initialize SPI with custom pins to prevent auto-configuration of default pins
    // This prevents SPI library from trying to use GPIO 19 (which is used for RELAY2)
    SPI.begin(sckPin, misoPin, mosiPin, csPin);
    Serial.printf("[CAN] SPI initialized: SCK=%d, MISO=%d, MOSI=%d, CS=%d\n", sckPin, misoPin, mosiPin, csPin);

    // Create MCP_CAN object with CS pin
    mcp_can_ = new MCP_CAN(csPin);

    // Initialize MCP2515 at 500 kbps
    if (mcp_can_->begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("[CAN] ERROR: MCP2515 initialization failed");
        delete mcp_can_;
        mcp_can_ = nullptr;
        return false;
    }

    // Set to normal mode (not loopback)
    mcp_can_->setMode(MCP_NORMAL);

    initialized_ = true;
    Serial.println("[CAN] MCP2515 initialized at 500 kbps");
    Serial.println("[CAN] Ready to read vehicle data");

    return true;
}

void CANController::update() {
    if (!initialized_) {
        return;  // Not initialized, nothing to do
    }

    uint32_t now = millis();

    // Poll RPM and speed at higher frequency (100ms)
    if (now - lastRPMPoll_ >= CAN_POLL_INTERVAL_RPM) {
        lastRPMPoll_ = now;

        // Try to read RPM
        if (readEngineRPM()) {
            vehicleData_.lastUpdateTime = now;
            vehicleData_.dataValid = true;
            retryCount_ = 0;  // Reset retry count on success
        } else {
            retryCount_++;
            if (retryCount_ >= CAN_RETRY_ATTEMPTS) {
                vehicleData_.dataValid = false;
                Serial.println("[CAN] WARNING: RPM read failed after retries");
            }
        }

        // Try to read speed
        if (readVehicleSpeed()) {
            vehicleData_.lastUpdateTime = now;
            vehicleData_.dataValid = true;
        }
    }

    // Poll temperatures at lower frequency (1000ms)
    if (now - lastTempPoll_ >= CAN_POLL_INTERVAL_TEMP) {
        lastTempPoll_ = now;

        // Try to read coolant temperature
        readCoolantTemp();

        // Try to read oil temperature
        readOilTemp();

        // Try to read throttle position
        readThrottlePosition();
    }

    // Check if data has become stale
    if (isDataStale()) {
        vehicleData_.dataValid = false;
    }
}

String CANController::getStatusString() const {
    if (!initialized_) {
        return "Error - MCP2515 not responding";
    }

    if (!vehicleData_.dataValid) {
        if (vehicleData_.lastUpdateTime == 0) {
            return "Disconnected - no data";
        } else {
            return "Disconnected - timeout";
        }
    }

    // Calculate update rate
    uint32_t dataAge = millis() - vehicleData_.lastUpdateTime;
    if (dataAge < 200) {
        return "Connected - 10 Hz";
    } else if (dataAge < 1000) {
        return "Connected - slow";
    } else {
        return "Disconnected - stale";
    }
}

bool CANController::sendOBDRequest(uint8_t pid) {
    if (!initialized_ || mcp_can_ == nullptr) {
        return false;
    }

    // OBD-II request format: [length] [mode] [PID] [unused...]
    // Mode 01 = Current Data
    uint8_t requestData[8] = {0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00};

    // Send request to OBD-II standard ID (0x7DF = broadcast)
    byte result = mcp_can_->sendMsgBuf(0x7DF, 0, 8, requestData);

    if (result != CAN_OK) {
        Serial.printf("[CAN] ERROR: Failed to send request for PID 0x%02X\n", pid);
        return false;
    }

    return true;
}

bool CANController::receiveOBDResponse(uint8_t pid, uint8_t* data, uint8_t& dataLen) {
    if (!initialized_ || mcp_can_ == nullptr) {
        return false;
    }

    uint32_t startTime = millis();
    unsigned long canId;
    uint8_t len = 0;
    uint8_t rxBuf[8];

    // Wait for response with timeout
    while (millis() - startTime < CAN_RESPONSE_TIMEOUT) {
        if (mcp_can_->checkReceive() == CAN_MSGAVAIL) {
            mcp_can_->readMsgBuf(&canId, &len, rxBuf);

            // OBD-II responses are from 0x7E8 to 0x7EF (ECU response IDs)
            if (canId >= 0x7E8 && canId <= 0x7EF) {
                // Verify this is a Mode 01 response (0x41) for our PID
                if (len >= 3 && rxBuf[1] == 0x41 && rxBuf[2] == pid) {
                    // Copy data payload (skip length, mode, and PID bytes)
                    dataLen = len - 3;
                    for (uint8_t i = 0; i < dataLen && i < 5; i++) {
                        data[i] = rxBuf[3 + i];
                    }
                    return true;
                }
            }
        }
        delay(10);  // Small delay to avoid tight loop
    }

    // Timeout
    Serial.printf("[CAN] WARNING: Timeout waiting for PID 0x%02X response\n", pid);
    return false;
}

bool CANController::readEngineRPM() {
    if (!sendOBDRequest(PID_ENGINE_RPM)) {
        return false;
    }

    uint8_t data[5];
    uint8_t dataLen;

    if (receiveOBDResponse(PID_ENGINE_RPM, data, dataLen)) {
        if (dataLen >= 2) {
            // RPM formula: ((A*256)+B)/4
            uint16_t rpm = ((data[0] * 256) + data[1]) / 4;
            vehicleData_.engineRPM = rpm;
            return true;
        }
    }

    return false;
}

bool CANController::readVehicleSpeed() {
    if (!sendOBDRequest(PID_VEHICLE_SPEED)) {
        return false;
    }

    uint8_t data[5];
    uint8_t dataLen;

    if (receiveOBDResponse(PID_VEHICLE_SPEED, data, dataLen)) {
        if (dataLen >= 1) {
            // Speed formula: A (direct value in km/h)
            vehicleData_.vehicleSpeed = data[0];
            return true;
        }
    }

    return false;
}

bool CANController::readCoolantTemp() {
    if (!sendOBDRequest(PID_COOLANT_TEMP)) {
        return false;
    }

    uint8_t data[5];
    uint8_t dataLen;

    if (receiveOBDResponse(PID_COOLANT_TEMP, data, dataLen)) {
        if (dataLen >= 1) {
            // Temperature formula: A - 40 (°C)
            vehicleData_.coolantTemp = data[0] - 40;
            return true;
        }
    }

    return false;
}

bool CANController::readOilTemp() {
    if (!sendOBDRequest(PID_OIL_TEMP)) {
        return false;
    }

    uint8_t data[5];
    uint8_t dataLen;

    if (receiveOBDResponse(PID_OIL_TEMP, data, dataLen)) {
        if (dataLen >= 1) {
            // Temperature formula: A - 40 (°C)
            vehicleData_.oilTemp = data[0] - 40;
            return true;
        }
    }

    return false;
}

bool CANController::readThrottlePosition() {
    if (!sendOBDRequest(PID_THROTTLE_POS)) {
        return false;
    }

    uint8_t data[5];
    uint8_t dataLen;

    if (receiveOBDResponse(PID_THROTTLE_POS, data, dataLen)) {
        if (dataLen >= 1) {
            // Throttle position formula: A * 100/255 (percentage)
            vehicleData_.throttlePosition = (data[0] * 100) / 255;
            return true;
        }
    }

    return false;
}

bool CANController::isDataStale() const {
    if (vehicleData_.lastUpdateTime == 0) {
        return true;  // Never received data
    }

    uint32_t age = millis() - vehicleData_.lastUpdateTime;
    return (age > CAN_DATA_STALE_TIMEOUT);
}
