#include "CANController.h"
#include "Debug.h"
#include <SPI.h>

CANController::CANController()
    : mcp_can_(nullptr),
      initialized_(false),
      state_(OBDState::IDLE),
      activePIDIndex_(0),
      requestSentTime_(0),
      responseLen_(0) {
    // Initialize vehicle data to safe defaults
    vehicleData_.engineRPM = 0;
    vehicleData_.vehicleSpeed = 0;
    vehicleData_.coolantTemp = 0;
    vehicleData_.oilTemp = 0;
    vehicleData_.throttlePosition = 0;
    vehicleData_.fuelLevel = 0;
    vehicleData_.lastUpdateTime = 0;
    vehicleData_.dataValid = false;

    // Initialize PID scheduling table
    pidTable_[0] = {PID_ENGINE_RPM,    CAN_POLL_INTERVAL_RPM,  0, 0};
    pidTable_[1] = {PID_VEHICLE_SPEED, CAN_POLL_INTERVAL_RPM,  0, 0};
    pidTable_[2] = {PID_COOLANT_TEMP,  CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[3] = {PID_OIL_TEMP,      CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[4] = {PID_THROTTLE_POS,  CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[5] = {PID_FUEL_LEVEL,    CAN_POLL_INTERVAL_TEMP, 0, 0};
}

CANController::~CANController() {
    if (mcp_can_ != nullptr) {
        delete mcp_can_;
        mcp_can_ = nullptr;
    }
}

bool CANController::begin(gpio_num_t csPin, gpio_num_t sckPin, gpio_num_t mosiPin, gpio_num_t misoPin) {
    Debug::printlnFeature(DebugFeature::CAN,"[CAN] Initializing MCP2515 on SPI...");

    // Initialize SPI pins before MCP_CAN tries to use them
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);  // CS idle high
    pinMode(sckPin, OUTPUT);
    pinMode(mosiPin, OUTPUT);
    pinMode(misoPin, INPUT);

    // Explicitly initialize SPI with custom pins to prevent auto-configuration of default pins
    // This prevents SPI library from trying to use GPIO 19 (which is used for RELAY2)
    SPI.begin(sckPin, misoPin, mosiPin, csPin);
    Debug::printfFeature(DebugFeature::CAN,"[CAN] SPI initialized: SCK=%d, MISO=%d, MOSI=%d, CS=%d\n", sckPin, misoPin, mosiPin, csPin);

    // Create MCP_CAN object with CS pin
    mcp_can_ = new MCP_CAN(csPin);

    // Initialize MCP2515 at 500 kbps
    if (mcp_can_->begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Debug::printlnFeature(DebugFeature::CAN,"[CAN] ERROR: MCP2515 initialization failed");
        delete mcp_can_;
        mcp_can_ = nullptr;
        return false;
    }

    // Set to normal mode (not loopback)
    mcp_can_->setMode(MCP_NORMAL);

    initialized_ = true;
    Debug::printlnFeature(DebugFeature::CAN,"[CAN] MCP2515 initialized at 500 kbps");
    Debug::printlnFeature(DebugFeature::CAN,"[CAN] Ready to read vehicle data");

    return true;
}

void CANController::update() {
    if (!initialized_) {
        return;
    }

    switch (state_) {
        case OBDState::IDLE: {
            int8_t index = selectNextPID();
            if (index < 0) {
                break;  // Nothing due yet
            }

            if (sendOBDRequest(pidTable_[index].pid)) {
                activePIDIndex_ = index;
                requestSentTime_ = millis();
                state_ = OBDState::WAITING_RESPONSE;
            } else {
                // Send failed, skip and reschedule
                pidTable_[index].nextPollTime = millis() + pidTable_[index].interval;
                pidTable_[index].retryCount++;
            }
            break;
        }

        case OBDState::WAITING_RESPONSE: {
            if (tryReceiveResponse()) {
                // Got valid response
                parseAndStore(activePIDIndex_, responseData_, responseLen_);
                pidTable_[activePIDIndex_].nextPollTime = millis() + pidTable_[activePIDIndex_].interval;
                pidTable_[activePIDIndex_].retryCount = 0;
                vehicleData_.lastUpdateTime = millis();
                vehicleData_.dataValid = true;
                state_ = OBDState::IDLE;
            } else if (millis() - requestSentTime_ >= CAN_RESPONSE_TIMEOUT) {
                // Abort pending TX and wait for bus-off recovery before next send
                mcp_can_->abortTX();
                if (mcp_can_->getError() & MCP_EFLG_TXBO) {
                    Debug::printlnFeature(DebugFeature::CAN, "[CAN] Bus-off detected, recovering...");
                    mcp_can_->begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
                    mcp_can_->setMode(MCP_NORMAL);
                }
                pidTable_[activePIDIndex_].retryCount++;
                // Add a short delay before the next send to let abortTX settle
                pidTable_[activePIDIndex_].nextPollTime = millis() + 10 + pidTable_[activePIDIndex_].interval;

                if (pidTable_[activePIDIndex_].retryCount >= CAN_RETRY_ATTEMPTS) {
                    Debug::printfFeature(DebugFeature::CAN,
                        "[CAN] WARNING: Timeout for PID 0x%02X after %d retries\n",
                        pidTable_[activePIDIndex_].pid, CAN_RETRY_ATTEMPTS);
                }

                // Check if all PIDs have failed
                bool allFailed = true;
                for (uint8_t i = 0; i < PID_COUNT; i++) {
                    if (pidTable_[i].retryCount < CAN_RETRY_ATTEMPTS) {
                        allFailed = false;
                        break;
                    }
                }
                if (allFailed) {
                    vehicleData_.dataValid = false;
                }

                state_ = OBDState::IDLE;
            }
            // else: no response yet, return immediately — try next loop iteration
            break;
        }
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
        Debug::printfFeature(DebugFeature::CAN,"[CAN] ERROR: Failed to send request for PID 0x%02X\n", pid);
        return false;
    }

    return true;
}

bool CANController::tryReceiveResponse() {
    if (mcp_can_ == nullptr) {
        return false;
    }

    uint8_t expectedPid = pidTable_[activePIDIndex_].pid;
    unsigned long canId;
    uint8_t len = 0;
    uint8_t rxBuf[8];

    // Drain up to 5 messages to prevent MCP2515 RX buffer overflow
    for (uint8_t i = 0; i < 5; i++) {
        if (mcp_can_->checkReceive() != CAN_MSGAVAIL) {
            break;  // No more messages
        }

        mcp_can_->readMsgBuf(&canId, &len, rxBuf);

        // OBD-II responses are from 0x7E8 to 0x7EF (ECU response IDs)
        if (canId >= 0x7E8 && canId <= 0x7EF) {
            // Verify this is a Mode 01 response (0x41) for our PID
            if (len >= 3 && rxBuf[1] == 0x41 && rxBuf[2] == expectedPid) {
                // Copy data payload (skip length, mode, and PID bytes)
                responseLen_ = len - 3;
                for (uint8_t j = 0; j < responseLen_ && j < 5; j++) {
                    responseData_[j] = rxBuf[3 + j];
                }
                return true;
            }
        }
        // Non-matching message — discard and continue draining
    }

    return false;
}

int8_t CANController::selectNextPID() {
    uint32_t now = millis();
    int8_t bestIndex = -1;
    uint32_t mostOverdue = 0;

    for (uint8_t i = 0; i < PID_COUNT; i++) {
        if (now >= pidTable_[i].nextPollTime) {
            uint32_t overdue = now - pidTable_[i].nextPollTime;
            if (overdue >= mostOverdue) {
                mostOverdue = overdue;
                bestIndex = i;
            }
        }
    }
    return bestIndex;
}

void CANController::parseAndStore(uint8_t index, const uint8_t* data, uint8_t len) {
    switch (pidTable_[index].pid) {
        case PID_ENGINE_RPM:
            if (len >= 2) {
                vehicleData_.engineRPM = ((data[0] * 256) + data[1]) / 4;
            }
            break;
        case PID_VEHICLE_SPEED:
            if (len >= 1) {
                vehicleData_.vehicleSpeed = data[0];
            }
            break;
        case PID_COOLANT_TEMP:
            if (len >= 1) {
                vehicleData_.coolantTemp = data[0] - 40;
            }
            break;
        case PID_OIL_TEMP:
            if (len >= 1) {
                vehicleData_.oilTemp = data[0] - 40;
            }
            break;
        case PID_THROTTLE_POS:
            if (len >= 1) {
                vehicleData_.throttlePosition = (data[0] * 100) / 255;
            }
            break;
        case PID_FUEL_LEVEL:
            if (len >= 1) {
                vehicleData_.fuelLevel = (data[0] * 100) / 255;
            }
            break;
    }
}

bool CANController::isDataStale() const {
    if (vehicleData_.lastUpdateTime == 0) {
        return true;  // Never received data
    }

    uint32_t age = millis() - vehicleData_.lastUpdateTime;
    return (age > CAN_DATA_STALE_TIMEOUT);
}
