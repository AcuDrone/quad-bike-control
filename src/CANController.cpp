#include "CANController.h"
#include "Debug.h"
#include <SPI.h>

// Candidate PIDs test-requested during a probe (order matters for reporting)
static const uint8_t kProbeCandidates[CANController::PROBE_CANDIDATE_COUNT] = {
    0x04,  // calculated engine load
    0x0B,  // manifold absolute pressure
    0x0E,  // timing advance
    0x0F,  // intake air temperature
    0x42,  // control module voltage
    0x0D,  // vehicle speed
    0x2F,  // fuel tank level
    0x5C,  // engine oil temperature
    0x14   // O2 sensor 1 (voltage + trim)
};

// Supported-PID bitmap group request PIDs (Mode 01)
static const uint8_t kBitmapGroups[4] = {0x00, 0x20, 0x40, 0x60};

CANController::CANController()
    : mcp_can_(nullptr),
      initialized_(false),
      csPin_(PIN_CAN_CS),
      consecutiveSendFailures_(0),
      lastRxLogTime_(0),
      lastHealthLogTime_(0),
      lastOverflowLogTime_(0),
      rxOverflowSuppressed_(0),
      state_(OBDState::IDLE),
      activePIDIndex_(0),
      requestSentTime_(0),
      responseLen_(0),
      mode_(Mode::NORMAL),
      probePhase_(ProbePhase::DONE),
      probeBitmapGroup_(0),
      probeMaxBitmapGroups_(1),
      probeCandidateIdx_(0),
      probeRequestInFlight_(false),
      probeRequestSentTime_(0),
      probeRetryCount_(0) {
    // Initialize vehicle data to safe defaults
    vehicleData_.engineRPM = 0;
    vehicleData_.vehicleSpeed = 0;
    vehicleData_.coolantTemp = 0;
    vehicleData_.oilTemp = 0;
    vehicleData_.throttlePosition = 0;
    vehicleData_.fuelLevel = 0;
    vehicleData_.mapKpa = 0;
    vehicleData_.moduleVoltageMv = 0;
    vehicleData_.intakeTemp = 0;
    vehicleData_.engineLoad = 0;
    vehicleData_.lastUpdateTime = 0;
    vehicleData_.dataValid = false;

    // Initialize probe results to an empty/idle snapshot
    probeResults_ = {};
    for (uint8_t i = 0; i < PROBE_CANDIDATE_COUNT; i++) {
        probeResults_.pids[i].pid = kProbeCandidates[i];
    }
    strncpy(probeResults_.status, "idle", sizeof(probeResults_.status) - 1);

    // Initialize PID scheduling table (all PID_COUNT entries must be initialized)
    pidTable_[0] = {PID_ENGINE_RPM,   CAN_POLL_INTERVAL_RPM,  0, 0};
    pidTable_[1] = {PID_COOLANT_TEMP, CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[2] = {PID_THROTTLE_POS, CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[3] = {PID_MAP,          CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[4] = {PID_MODULE_VOLTAGE, CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[5] = {PID_INTAKE_TEMP,    CAN_POLL_INTERVAL_TEMP, 0, 0};
    pidTable_[6] = {PID_ENGINE_LOAD,    CAN_POLL_INTERVAL_TEMP, 0, 0};
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

    // Remember CS for the raw register reads used by the diagnostics helpers
    csPin_ = csPin;

    // Create MCP_CAN object with CS pin
    mcp_can_ = new MCP_CAN(csPin);

    // Initialize MCP2515 at 500 kbps.
    // MCP_STDEXT (not MCP_ANY) leaves RXB0CTRL/RXB1CTRL RXM = 00 so the acceptance
    // masks/filters are actually applied; MCP_ANY sets RXM = 11 and disables them.
    if (mcp_can_->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Debug::printlnFeature(DebugFeature::CAN,"[CAN] ERROR: MCP2515 initialization failed");
        delete mcp_can_;
        mcp_can_ = nullptr;
        return false;
    }

    // Program the acceptance window BEFORE going NORMAL (library convention: init_Mask()/
    // init_Filt() switch to CONFIG mode internally and restore the previously cached mode).
    if (!applyRxFilters()) {
        Debug::printlnFeature(DebugFeature::CAN,
            "[CAN] ERROR: failed to program RX acceptance masks/filters");
        delete mcp_can_;
        mcp_can_ = nullptr;
        return false;
    }

    // Set to normal mode (not loopback). setMode() returns MCP2515_FAIL if CANSTAT
    // never reflects the requested mode — silently ignoring it leaves the chip in
    // config/loopback mode, which looks like a healthy init but never touches the bus.
    byte modeResult = mcp_can_->setMode(MCP_NORMAL);
    if (modeResult != MCP2515_OK) {
        Debug::printlnFeature(DebugFeature::CAN, "[CAN] WARNING: setMode(NORMAL) failed, retrying once...");
        modeResult = mcp_can_->setMode(MCP_NORMAL);
    }

    // Read CANSTAT back directly (the library exposes no getMode()) so the actual
    // operating mode is visible on the monitor regardless of what setMode() claimed.
    uint8_t canstat = readRegister(MCP_CANSTAT);
    uint8_t opmode = canstat & MODE_MASK;
    const char* modeName = "UNKNOWN";
    switch (opmode) {
        case MCP_NORMAL:     modeName = "NORMAL";     break;
        case MCP_SLEEP:      modeName = "SLEEP";      break;
        case MCP_LOOPBACK:   modeName = "LOOPBACK";   break;
        case MCP_LISTENONLY: modeName = "LISTENONLY"; break;
        case MODE_CONFIG:    modeName = "CONFIG";     break;
        default: break;
    }
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] CANSTAT=0x%02X -> mode %s (0x%02X)\n", canstat, modeName, opmode);

    if (modeResult != MCP2515_OK || opmode != MCP_NORMAL) {
        Debug::printlnFeature(DebugFeature::CAN, "[CAN] ERROR: failed to enter NORMAL mode (stuck in loopback?)");
        initialized_ = false;
        delete mcp_can_;
        mcp_can_ = nullptr;
        return false;
    }

    initialized_ = true;

    // Stagger the three added slow PIDs. At power-on every entry is due at once and
    // selectNextPID() breaks exact ties by highest index (`overdue >= mostOverdue`),
    // so RPM (index 0) is served last; offsetting the new entries keeps them out of
    // that burst. Done here, not in the constructor: the constructor runs before boot
    // time is meaningful, so any offset baked in there is already in the past by the
    // time begin() completes.
    uint32_t staggerBase = millis();
    pidTable_[4].nextPollTime = staggerBase + 150;
    pidTable_[5].nextPollTime = staggerBase + 300;
    pidTable_[6].nextPollTime = staggerBase + 450;

    Debug::printlnFeature(DebugFeature::CAN,"[CAN] MCP2515 initialized at 500 kbps");
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] health: TEC=%u REC=%u EFLG=0x%02X\n",
        mcp_can_->errorCountTX(), mcp_can_->errorCountRX(), mcp_can_->getError());
    Debug::printlnFeature(DebugFeature::CAN,"[CAN] Ready to read vehicle data");

    return true;
}

void CANController::update() {
    if (!initialized_) {
        return;
    }

    // While probing, run the capability sweep instead of the normal scheduler.
    // VehicleData is left untouched so normal polling resumes where it left off.
    if (mode_ == Mode::PROBING) {
        updateProbe();
        if (isDataStale()) {
            vehicleData_.dataValid = false;
        }
        return;
    }

    switch (state_) {
        case OBDState::IDLE: {
            // Keep the MCP2515 RX buffers empty between polls: a frame left sitting in
            // RXB0/RXB1 blocks the next reception and eventually raises RXnOVR.
            drainRxBuffers();
            checkRxOverflow();
            logHealth();

            int8_t index = selectNextPID();
            if (index < 0) {
                break;  // Nothing due yet
            }

            if (sendOBDRequest(pidTable_[index].pid)) {
                activePIDIndex_ = index;
                requestSentTime_ = millis();
                state_ = OBDState::WAITING_RESPONSE;
            } else {
                // Send failed (logged by sendOBDRequest) — surface why and try to recover
                uint8_t eflg = mcp_can_->getError();
                Debug::printfFeature(DebugFeature::CAN,
                    "[CAN] Send failure: TEC=%u REC=%u EFLG=0x%02X\n",
                    mcp_can_->errorCountTX(), mcp_can_->errorCountRX(), eflg);

                if (eflg & MCP_EFLG_TXBO) {
                    recoverFromBusOff();
                }

                // A wedged TX buffer never clears itself: abort queued transmissions
                // once the failures stack up, so the next poll starts from a clean slate.
                if (++consecutiveSendFailures_ >= CAN_MAX_CONSEC_SEND_FAILURES) {
                    mcp_can_->abortTX();
                    Debug::printfFeature(DebugFeature::CAN,
                        "[CAN] Aborted stuck TX buffers after %d consecutive send failures\n",
                        consecutiveSendFailures_);
                    consecutiveSendFailures_ = 0;
                }

                // Skip and reschedule
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

                static uint32_t lastDataLog = 0;
                if (millis() - lastDataLog >= 1000) {
                    lastDataLog = millis();
                    Debug::printfFeature(DebugFeature::CAN,
                        "[CAN] RPM=%u coolant=%dC throttle=%u%% volt=%u.%03uV IAT=%dC load=%u%%\n",
                        vehicleData_.engineRPM, vehicleData_.coolantTemp,
                        vehicleData_.throttlePosition,
                        vehicleData_.moduleVoltageMv / 1000,
                        vehicleData_.moduleVoltageMv % 1000,
                        vehicleData_.intakeTemp, vehicleData_.engineLoad);
                }
            } else if (millis() - requestSentTime_ >= CAN_RESPONSE_TIMEOUT) {
                // Abort pending TX and wait for bus-off recovery before next send
                // mcp_can_->abortTX();
                if (mcp_can_->getError() & MCP_EFLG_TXBO) {
                    recoverFromBusOff();
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

void CANController::setRPMPollInterval(uint32_t ms) {
    pidTable_[0].interval = ms;
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

    consecutiveSendFailures_ = 0;
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
        // Non-matching message — report it (rate-limited) so a wrong response ID or a
        // negative response (0x7F) is visible instead of being silently dropped
        logUnmatchedFrame(canId, len, rxBuf);
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
        // case PID_VEHICLE_SPEED:
        //     if (len >= 1) {
        //         vehicleData_.vehicleSpeed = data[0];
        //     }
        //     break;
        case PID_COOLANT_TEMP:
            if (len >= 1) {
                vehicleData_.coolantTemp = data[0] - 40;
            }
            break;
        // case PID_OIL_TEMP:
        //     if (len >= 1) {
        //         vehicleData_.oilTemp = data[0] - 40;
        //     }
        //     break;
        case PID_THROTTLE_POS:
            if (len >= 1) {
                vehicleData_.throttlePosition = (data[0] * 100) / 255;
            }
            break;
        case PID_MAP:
            if (len >= 1) {
                vehicleData_.mapKpa = data[0];  // kPa absolute (0-255)
            }
            break;
        case PID_MODULE_VOLTAGE:
            if (len >= 2) {
                // Raw (A*256)+B is already millivolts; kept as-is and divided by 1000
                // only at the JSON/MAVLink presentation edge (no floats on this path).
                vehicleData_.moduleVoltageMv = (data[0] * 256) + data[1];
            }
            break;
        case PID_INTAKE_TEMP:
            if (len >= 1) {
                vehicleData_.intakeTemp = data[0] - 40;
            }
            break;
        case PID_ENGINE_LOAD:
            if (len >= 1) {
                vehicleData_.engineLoad = (data[0] * 100) / 255;
            }
            break;
        // case PID_FUEL_LEVEL:
        //     if (len >= 1) {
        //         vehicleData_.fuelLevel = (data[0] * 100) / 255;
        //     }
        //     break;
    }
}

bool CANController::isDataStale() const {
    if (vehicleData_.lastUpdateTime == 0) {
        return true;  // Never received data
    }

    uint32_t age = millis() - vehicleData_.lastUpdateTime;
    return (age > CAN_DATA_STALE_TIMEOUT);
}

// ============================================================================
// DIAGNOSTICS / RECOVERY
// ============================================================================

// The vendored coryjfowler MCP_CAN library keeps mcp2515_readRegister() and
// mcp2515_modifyRegister() private and exposes no getMode()/clearRXnOVR(), so the
// two registers the diagnostics need (CANSTAT for mode readback, EFLG for clearing
// the RX overflow flags) are accessed here over the same SPI bus the library uses.
// Same SPISettings as the library (10 MHz, MSBFIRST, SPI_MODE0) and the same CS pin,
// so the transfers interleave safely with the library's own transactions.
uint8_t CANController::readRegister(uint8_t address) const {
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    SPI.transfer(MCP_READ);
    SPI.transfer(address);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(csPin_, HIGH);
    SPI.endTransaction();
    return value;
}

void CANController::modifyRegister(uint8_t address, uint8_t mask, uint8_t data) const {
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    SPI.transfer(MCP_BITMOD);
    SPI.transfer(address);
    SPI.transfer(mask);
    SPI.transfer(data);
    digitalWrite(csPin_, HIGH);
    SPI.endTransaction();
}

// Restrict the MCP2515 RX buffers to the OBD-II response window 0x7E8-0x7EF.
//
// Encoding (mcp_can.cpp, mcp2515_write_mf(), lines 722-748): for ext == 0 the library
// does `canid = (uint16_t)(id >> 16)` and then writes
//     SIDL = (canid & 0x07) << 5;  SIDH = canid >> 3;
// i.e. the 11-bit standard ID must sit in bits 26:16 of the argument -> id << 16.
// The low 16 bits land in EIDn (extended-ID / first-two-data-bytes filtering) and are
// left at 0 = don't care. This matches the library's own OBD2_PID_Request example,
// which passes init_Mask(0, 0x7F00000) / init_Filt(0, 0x7DF0000).
//
// mask 0x7F8 << 16 keeps ID bits 10:3 as "care" and bits 2:0 as "don't care";
// filter 0x7E8 << 16 therefore accepts exactly 0x7E8..0x7EF.
// Mask 0 gates RXB0 (filters 0-1), mask 1 gates RXB1 (filters 2-5), so both masks and
// all six filters must be set or an unfiltered buffer keeps letting broadcasts through.
// ext = 0 on every filter also clears EXIDE, so extended frames match nothing and are
// rejected outright.
bool CANController::applyRxFilters() {
    if (mcp_can_ == nullptr) {
        return false;
    }

    bool ok = true;
    ok &= (mcp_can_->init_Mask(0, 0, CAN_RX_ID_MASK_REG) == MCP2515_OK);
    ok &= (mcp_can_->init_Mask(1, 0, CAN_RX_ID_MASK_REG) == MCP2515_OK);
    for (uint8_t n = 0; n < 6; n++) {
        ok &= (mcp_can_->init_Filt(n, 0, CAN_RX_ID_FILTER_REG) == MCP2515_OK);
    }

    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] RX acceptance filter: standard IDs 0x%03X-0x%03X only "
        "(mask=0x%03X filter=0x%03X, regs 0x%08lX/0x%08lX)%s\n",
        (unsigned)CAN_RX_ID_FILTER,
        (unsigned)(CAN_RX_ID_FILTER | (~CAN_RX_ID_MASK & 0x7FF)),
        (unsigned)CAN_RX_ID_MASK, (unsigned)CAN_RX_ID_FILTER,
        (unsigned long)CAN_RX_ID_MASK_REG, (unsigned long)CAN_RX_ID_FILTER_REG,
        ok ? "" : " [FAILED]");

    return ok;
}

void CANController::recoverFromBusOff() {
    if (mcp_can_ == nullptr) {
        return;
    }

    Debug::printlnFeature(DebugFeature::CAN, "[CAN] Bus-off detected, recovering...");
    // begin() resets the chip, which wipes the masks/filters — reprogram them before
    // returning to NORMAL or the broadcast flood comes straight back.
    mcp_can_->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ);
    applyRxFilters();
    if (mcp_can_->setMode(MCP_NORMAL) != MCP2515_OK) {
        Debug::printlnFeature(DebugFeature::CAN, "[CAN] ERROR: failed to enter NORMAL mode (stuck in loopback?)");
    }
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] Recovery: CANSTAT=0x%02X EFLG=0x%02X\n",
        readRegister(MCP_CANSTAT), mcp_can_->getError());
    consecutiveSendFailures_ = 0;
}

void CANController::logUnmatchedFrame(unsigned long canId, uint8_t len, const uint8_t* buf) {
    uint32_t now = millis();
    if (now - lastRxLogTime_ < CAN_RX_LOG_INTERVAL) {
        return;  // rate-limited to ~1 line per second
    }
    lastRxLogTime_ = now;

    uint8_t d[4] = {0, 0, 0, 0};
    for (uint8_t i = 0; i < 4 && i < len; i++) {
        d[i] = buf[i];
    }
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] RX unmatched: id=0x%03lX len=%d data=%02X %02X %02X %02X\n",
        canId, len, d[0], d[1], d[2], d[3]);
}

void CANController::logHealth() {
    if (mcp_can_ == nullptr || vehicleData_.dataValid) {
        return;  // only interesting while no data is coming back
    }

    uint32_t now = millis();
    if (now - lastHealthLogTime_ < CAN_HEALTH_LOG_INTERVAL) {
        return;
    }
    lastHealthLogTime_ = now;

    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] health: TEC=%u REC=%u EFLG=0x%02X\n",
        mcp_can_->errorCountTX(), mcp_can_->errorCountRX(), mcp_can_->getError());
}

void CANController::checkRxOverflow() {
    if (mcp_can_ == nullptr) {
        return;
    }

    uint8_t eflg = mcp_can_->getError();
    if ((eflg & (MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR)) == 0) {
        return;
    }

    // RXnOVR is sticky and must be cleared by software, otherwise the flags stay
    // latched forever and mask any later overflow.
    modifyRegister(MCP_EFLG, MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR, 0x00);

    // Hardware filtering should make overflows rare, but a burst still floods the log
    // one line per loop iteration — report at most once per CAN_OVERFLOW_LOG_INTERVAL
    // and carry the suppressed count so the real rate stays visible.
    uint32_t now = millis();
    if (lastOverflowLogTime_ != 0 && (now - lastOverflowLogTime_) < CAN_OVERFLOW_LOG_INTERVAL) {
        rxOverflowSuppressed_++;
        return;
    }
    lastOverflowLogTime_ = now;

    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] RX overflow (EFLG=0x%02X), cleared (x%lu since last report)\n",
        eflg, (unsigned long)rxOverflowSuppressed_);
    rxOverflowSuppressed_ = 0;
}

void CANController::drainRxBuffers() {
    if (mcp_can_ == nullptr) {
        return;
    }

    unsigned long canId;
    uint8_t len = 0;
    uint8_t rxBuf[8];

    // Bounded drain (same cap as the receive paths) — keeps the loop non-blocking
    for (uint8_t i = 0; i < 5; i++) {
        if (mcp_can_->checkReceive() != CAN_MSGAVAIL) {
            break;
        }
        mcp_can_->readMsgBuf(&canId, &len, rxBuf);
        // Nothing is expected while IDLE, so every frame here is "unmatched"
        logUnmatchedFrame(canId, len, rxBuf);
    }
}

// ============================================================================
// ECU CAPABILITY PROBE
// ============================================================================

// Format a raw 2-byte OBD-II DTC into the standard P/C/B/U code string.
static void formatDtc(uint16_t code, char* out, size_t outLen) {
    const char letters[4] = {'P', 'C', 'B', 'U'};
    uint8_t hi = (code >> 8) & 0xFF;
    uint8_t lo = code & 0xFF;
    snprintf(out, outLen, "%c%X%X%X%X",
             letters[(hi >> 6) & 0x03],
             (hi >> 4) & 0x03,
             hi & 0x0F,
             (lo >> 4) & 0x0F,
             lo & 0x0F);
}

bool CANController::sendMode03Request() {
    if (!initialized_ || mcp_can_ == nullptr) {
        return false;
    }

    // Mode 03 = request stored DTCs (no PID byte)
    uint8_t requestData[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    byte result = mcp_can_->sendMsgBuf(0x7DF, 0, 8, requestData);

    if (result != CAN_OK) {
        Debug::printlnFeature(DebugFeature::CAN, "[CAN] ERROR: Failed to send Mode 03 request");
        return false;
    }

    consecutiveSendFailures_ = 0;
    return true;
}

bool CANController::tryReceiveProbeResponse(uint8_t expectedMode, uint8_t expectedPid,
                                            uint8_t* outBuf, uint8_t& outLen) {
    if (mcp_can_ == nullptr) {
        return false;
    }

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
        if (canId >= 0x7E8 && canId <= 0x7EF && len >= 2) {
            bool match = false;
            if (expectedMode == 0x01) {
                // Mode 01 positive response (0x41) for the expected PID
                match = (len >= 3 && rxBuf[1] == 0x41 && rxBuf[2] == expectedPid);
            } else if (expectedMode == 0x03) {
                // Mode 03 positive response (0x43)
                match = (rxBuf[1] == 0x43);
            }
            if (match) {
                outLen = len;
                for (uint8_t j = 0; j < len && j < 8; j++) {
                    outBuf[j] = rxBuf[j];
                }
                return true;
            }
        }
        // Non-matching message — report it (rate-limited) and continue draining
        logUnmatchedFrame(canId, len, rxBuf);
    }

    return false;
}

CANController::ProbeStart CANController::startProbe(bool gearChangeActive) {
    // Reject if a probe is already running
    if (mode_ == Mode::PROBING) {
        return ProbeStart::BUSY;
    }

    // Defer while a gear change is active (protects the high-rate RPM feedback the shift relies on)
    if (gearChangeActive) {
        return ProbeStart::BUSY;
    }

    // Reset the results snapshot for a fresh sweep
    probeResults_ = {};
    for (uint8_t i = 0; i < PROBE_CANDIDATE_COUNT; i++) {
        probeResults_.pids[i].pid = kProbeCandidates[i];
    }

    // Short-circuit if the ECU is not answering (no bus traffic generated)
    if (!vehicleData_.dataValid) {
        probeResults_.running = false;
        probeResults_.complete = true;
        probeResults_.completedAtMs = millis();
        strncpy(probeResults_.status, "no ECU", sizeof(probeResults_.status) - 1);
        Debug::printlnFeature(DebugFeature::CAN, "[CAN][PROBE] Aborted: no ECU / disconnected");
        return ProbeStart::NO_ECU;
    }

    // Enter probe mode and initialize the sequencer cursor
    probeResults_.running = true;
    probeResults_.complete = false;
    strncpy(probeResults_.status, "probing", sizeof(probeResults_.status) - 1);
    probePhase_ = ProbePhase::BITMAP;
    probeBitmapGroup_ = 0;
    probeMaxBitmapGroups_ = 1;
    probeCandidateIdx_ = 0;
    probeRequestInFlight_ = false;
    probeRetryCount_ = 0;
    mode_ = Mode::PROBING;

    Debug::printlnFeature(DebugFeature::CAN, "[CAN][PROBE] Started ECU capability probe");
    return ProbeStart::STARTED;
}

bool CANController::probeCurrentRequest(uint8_t& mode, uint8_t& pid) const {
    switch (probePhase_) {
        case ProbePhase::BITMAP:
            mode = 0x01;
            pid = kBitmapGroups[probeBitmapGroup_];
            return true;
        case ProbePhase::CANDIDATE:
            mode = 0x01;
            pid = kProbeCandidates[probeCandidateIdx_];
            return true;
        case ProbePhase::DTC:
            mode = 0x03;
            pid = 0x00;
            return true;
        case ProbePhase::DONE:
        default:
            return false;
    }
}

void CANController::probeAdvanceCursor() {
    probeRetryCount_ = 0;
    switch (probePhase_) {
        case ProbePhase::BITMAP:
            probeBitmapGroup_++;
            if (probeBitmapGroup_ >= probeMaxBitmapGroups_ || probeBitmapGroup_ >= 4) {
                probePhase_ = ProbePhase::CANDIDATE;
                probeCandidateIdx_ = 0;
            }
            break;
        case ProbePhase::CANDIDATE:
            probeCandidateIdx_++;
            if (probeCandidateIdx_ >= PROBE_CANDIDATE_COUNT) {
                probePhase_ = ProbePhase::DTC;
            }
            break;
        case ProbePhase::DTC:
            probePhase_ = ProbePhase::DONE;
            break;
        case ProbePhase::DONE:
        default:
            break;
    }
}

void CANController::recordProbeResponse(uint8_t mode, uint8_t pid, const uint8_t* buf, uint8_t len) {
    if (mode == 0x01 && pid <= 0x60 && (pid % 0x20) == 0) {
        // Supported-PID bitmap group response: buf = [len][0x41][pid][A][B][C][D]
        ProbeBitmap& bm = probeResults_.bitmaps[probeBitmapGroup_];
        bm.queried = true;
        bm.answered = true;
        for (uint8_t j = 0; j < 4; j++) {
            bm.raw[j] = (len >= (uint8_t)(3 + j + 1)) ? buf[3 + j] : 0;
        }

        // Mark which candidate PIDs this group reports as supported.
        uint8_t base = pid;  // group covers PIDs base+1 .. base+0x20
        for (uint8_t c = 0; c < PROBE_CANDIDATE_COUNT; c++) {
            uint8_t cand = probeResults_.pids[c].pid;
            if (cand > base && cand <= (uint8_t)(base + 0x20)) {
                uint8_t offset = cand - base;              // 1..32
                uint8_t byteIdx = (offset - 1) / 8;        // 0..3
                uint8_t bit = 7 - ((offset - 1) % 8);      // MSB first
                if (bm.raw[byteIdx] & (1 << bit)) {
                    probeResults_.pids[c].supportedByBitmap = true;
                }
            }
        }

        // Continuation bit (bit0 of D) indicates the next group is supported.
        if ((bm.raw[3] & 0x01) && (probeBitmapGroup_ + 1) < 4) {
            probeMaxBitmapGroups_ = probeBitmapGroup_ + 2;
        }

        Debug::printfFeature(DebugFeature::CAN,
            "[CAN][PROBE] Bitmap 0x%02X = %02X %02X %02X %02X\n",
            pid, bm.raw[0], bm.raw[1], bm.raw[2], bm.raw[3]);
        return;
    }

    if (mode == 0x01) {
        // Candidate PID positive response: buf = [len][0x41][pid][payload...]
        ProbePidResult& pr = probeResults_.pids[probeCandidateIdx_];
        pr.answered = true;
        pr.rawLen = (len > 3) ? (len - 3) : 0;
        if (pr.rawLen > 4) pr.rawLen = 4;
        for (uint8_t j = 0; j < pr.rawLen; j++) {
            pr.raw[j] = buf[3 + j];
        }

        // Decode using standard OBD-II formulas for the PIDs we know.
        float A = (pr.rawLen >= 1) ? pr.raw[0] : 0.0f;
        float B = (pr.rawLen >= 2) ? pr.raw[1] : 0.0f;
        pr.hasDecoded = true;
        switch (pid) {
            case 0x04: pr.decoded = A * 100.0f / 255.0f; break;   // calc load %
            case 0x0B: pr.decoded = A; break;                     // MAP kPa
            case 0x0E: pr.decoded = A / 2.0f - 64.0f; break;      // timing advance °
            case 0x0F: pr.decoded = A - 40.0f; break;             // IAT °C
            case 0x42: pr.decoded = ((A * 256.0f) + B) / 1000.0f; break; // module V
            case 0x0D: pr.decoded = A; break;                     // speed km/h
            case 0x2F: pr.decoded = A * 100.0f / 255.0f; break;   // fuel level %
            case 0x5C: pr.decoded = A - 40.0f; break;             // oil temp °C
            case 0x14: pr.decoded = A / 200.0f; break;            // O2 S1 voltage V
            default:   pr.hasDecoded = false; break;
        }

        Debug::printfFeature(DebugFeature::CAN,
            "[CAN][PROBE] PID 0x%02X answered (%u bytes)%s\n",
            pid, pr.rawLen,
            pr.hasDecoded ? "" : " [raw only]");
        return;
    }

    if (mode == 0x03) {
        // Stored-DTC response: buf = [len][0x43][count][DTC1_hi][DTC1_lo][DTC2_hi][DTC2_lo]...
        ProbeDtc& d = probeResults_.dtc;
        d.queried = true;
        d.answered = true;
        d.count = (len >= 3) ? buf[2] : 0;

        uint8_t availPairs = (len > 3) ? ((len - 3) / 2) : 0;   // DTC pairs in this frame
        uint8_t capture = d.count;
        if (capture > availPairs) capture = availPairs;
        if (capture > PROBE_MAX_DTC) capture = PROBE_MAX_DTC;
        d.codeCount = capture;
        for (uint8_t k = 0; k < capture; k++) {
            d.codes[k] = ((uint16_t)buf[3 + 2 * k] << 8) | buf[3 + 2 * k + 1];
        }
        if (d.count > d.codeCount) {
            probeResults_.multiFrameTruncated = true;
        }

        Debug::printfFeature(DebugFeature::CAN,
            "[CAN][PROBE] DTC count=%u captured=%u%s\n",
            d.count, d.codeCount, probeResults_.multiFrameTruncated ? " (truncated)" : "");
        return;
    }
}

void CANController::recordProbeTimeout(uint8_t mode, uint8_t pid) {
    if (mode == 0x01 && pid <= 0x60 && (pid % 0x20) == 0) {
        probeResults_.bitmaps[probeBitmapGroup_].queried = true;
        probeResults_.bitmaps[probeBitmapGroup_].answered = false;
        Debug::printfFeature(DebugFeature::CAN, "[CAN][PROBE] Bitmap 0x%02X timeout\n", pid);
    } else if (mode == 0x01) {
        probeResults_.pids[probeCandidateIdx_].answered = false;
        Debug::printfFeature(DebugFeature::CAN, "[CAN][PROBE] PID 0x%02X no response\n", pid);
    } else if (mode == 0x03) {
        probeResults_.dtc.queried = true;
        probeResults_.dtc.answered = false;
        Debug::printlnFeature(DebugFeature::CAN, "[CAN][PROBE] Mode 03 (DTC) no response");
    }
}

void CANController::finishProbe() {
    probeResults_.running = false;
    probeResults_.complete = true;
    probeResults_.completedAtMs = millis();
    strncpy(probeResults_.status, "complete", sizeof(probeResults_.status) - 1);
    mode_ = Mode::NORMAL;
    state_ = OBDState::IDLE;

    // Mirror the final results table to the serial log
    Debug::printlnFeature(DebugFeature::CAN, "[CAN][PROBE] ===== Probe results =====");
    for (uint8_t g = 0; g < 4; g++) {
        const ProbeBitmap& bm = probeResults_.bitmaps[g];
        if (bm.queried) {
            Debug::printfFeature(DebugFeature::CAN,
                "[CAN][PROBE] bitmap 0x%02X: %s %02X %02X %02X %02X\n",
                kBitmapGroups[g], bm.answered ? "ok " : "---",
                bm.raw[0], bm.raw[1], bm.raw[2], bm.raw[3]);
        }
    }
    for (uint8_t i = 0; i < PROBE_CANDIDATE_COUNT; i++) {
        const ProbePidResult& pr = probeResults_.pids[i];
        if (pr.hasDecoded) {
            Debug::printfFeature(DebugFeature::CAN,
                "[CAN][PROBE] PID 0x%02X sup=%d ans=%d decoded=%.2f\n",
                pr.pid, pr.supportedByBitmap, pr.answered, pr.decoded);
        } else {
            Debug::printfFeature(DebugFeature::CAN,
                "[CAN][PROBE] PID 0x%02X sup=%d ans=%d raw=%u bytes\n",
                pr.pid, pr.supportedByBitmap, pr.answered, pr.rawLen);
        }
    }
    if (probeResults_.dtc.queried) {
        Debug::printfFeature(DebugFeature::CAN, "[CAN][PROBE] DTC count=%u:\n", probeResults_.dtc.count);
        char code[8];
        for (uint8_t k = 0; k < probeResults_.dtc.codeCount; k++) {
            formatDtc(probeResults_.dtc.codes[k], code, sizeof(code));
            Debug::printfFeature(DebugFeature::CAN, "[CAN][PROBE]   %s\n", code);
        }
        if (probeResults_.multiFrameTruncated) {
            Debug::printlnFeature(DebugFeature::CAN, "[CAN][PROBE]   (additional DTCs truncated - multi-frame)");
        }
    }
    Debug::printlnFeature(DebugFeature::CAN, "[CAN][PROBE] =========================");
}

void CANController::updateProbe() {
    uint8_t mode = 0x01;
    uint8_t pid = 0x00;

    if (probePhase_ == ProbePhase::DONE) {
        finishProbe();
        return;
    }

    if (!probeCurrentRequest(mode, pid)) {
        finishProbe();
        return;
    }

    if (!probeRequestInFlight_) {
        // Send the current request (retry on send failure like the normal machine)
        bool ok = (mode == 0x03) ? sendMode03Request() : sendOBDRequest(pid);
        if (ok) {
            probeRequestInFlight_ = true;
            probeRequestSentTime_ = millis();
        } else {
            if (probeRetryCount_ >= CAN_PROBE_RETRY_ATTEMPTS) {
                recordProbeTimeout(mode, pid);
                probeAdvanceCursor();
            } else {
                probeRetryCount_++;
            }
        }
        return;
    }

    // Waiting for a response
    uint8_t rxBuf[8];
    uint8_t rxLen = 0;
    if (tryReceiveProbeResponse(mode, pid, rxBuf, rxLen)) {
        recordProbeResponse(mode, pid, rxBuf, rxLen);
        probeRequestInFlight_ = false;
        probeAdvanceCursor();
    } else if (millis() - probeRequestSentTime_ >= CAN_RESPONSE_TIMEOUT) {
        probeRequestInFlight_ = false;
        if (probeRetryCount_ >= CAN_PROBE_RETRY_ATTEMPTS) {
            recordProbeTimeout(mode, pid);
            probeAdvanceCursor();
        } else {
            probeRetryCount_++;  // re-send the same request next iteration
        }
    }
    // else: no response yet — return and poll again next loop
}
