#include "CANController.h"
#include "CanWatchdogLaw.h"
#include "Debug.h"
#include "EventJournal.h"
#include "Lws305.h"
#include "LwsSensor.h"
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
    0x14,  // O2 sensor 1 (voltage + trim)
    0x0C,  // engine RPM (raw reply shown for RPM diagnostics)
    0x05   // coolant temperature (known-good reference)
};

// Supported-PID bitmap group request PIDs (Mode 01)
static const uint8_t kBitmapGroups[4] = {0x00, 0x20, 0x40, 0x60};

// TXB0CTRL value written with every frame: TXP = 1 (TXREQ 0). The follow-up's periodic frames
// get TXB1 / TXB2 at a higher TXP (design D5).
static constexpr uint8_t CAN_TXB0_CTRL_TXP = 0x01;

static uint8_t popCount8(uint8_t v) {
    uint8_t n = 0;
    for (; v; v &= (uint8_t)(v - 1)) n++;
    return n;
}

CANController::CANController()
    : mcp_can_(nullptr),
      initialized_(false),
      csPin_(PIN_CAN_CS),
      lastRxLogTime_(0),
      lastHealthLogTime_(0),
      lastOverflowLogTime_(0),
      rxOverflowSuppressed_(0),
      lastReinitTime_(0),
      reinitPending_(false),
      reinitReason_("bus_off"),
      reinitAttempt_(0),
      reinitAttemptMs_(0),
      reinitCount_(0),
      lwsPowered_(false),
      ignitionOnMs_(0),
      lwsSeenSinceOn_(false),
      lastLwsFrameMs_(0),
      lwsSilentObdDeadLogged_(false),
      firstFrameWait_(false),
      reinitDoneMs_(0),
      firstLwsMs_(0),
      firstObdMs_(0),
      reinitQuiet_(false),
      lwsEpisode_(false),
      obdEpisode_(false),
      lwsEpLogMs_(0),
      obdEpLogMs_(0),
      lwsEpRepeats_(0),
      obdEpRepeats_(0),
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
      probeRetryCount_(0),
      sniffExtraCount_(0),
      lwsSink_(nullptr),
      lwsZeroPhase_(LwsZeroPhase::IDLE),
      lwsZeroPhaseMs_(0),
      lwsZeroResetSent_(false),
      lwsZeroZeroSent_(false),
      lwsZeroSentMs_(0),
      lwsZeroAbort_(nullptr) {
    // CAN task channels and counters (assigned here: the member list above stays in order)
    task_ = nullptr;
    portMUX_INITIALIZE(&rxMux_);
    portMUX_INITIALIZE(&txMux_);
    portMUX_INITIALIZE(&evMux_);
    cmdSlot_ = 0;
    cmdInFlight_ = CanCmd::NONE;
    txSeq_ = 0;
    hasIntPin_ = false;
    isrCount_ = 0;
    chipUp_ = false;
    initAttempted_ = false;
    initAttempts_ = 0;
    firstInitMs_ = 0;
    lastInitTryMs_ = 0;
    lastEflgMs_ = 0;
    lastIsrSeen_ = 0;
    busOffLatched_ = false;
    txBusy_ = false;
    txAborting_ = false;
    txReported_ = true;
    txLoadMs_ = 0;
    txAbortMs_ = 0;
    txCur_ = {};
    txCancelSeq_ = 0;
    statsResetAck_ = 0;
    cRx_ = 0; cRxDrop_ = 0; cRxHwm_ = 0; cWinHwm_ = 0; cOvr_ = 0; cOvrEflg_ = 0;
    cIntWakes_ = 0; cPollFrames_ = 0; cCapHits_ = 0; cTx_ = 0; cTxFail_ = 0;
    cBusyMaxUs_ = 0; cWinBusyMaxUs_ = 0; cCmdMaxUs_ = 0; cBusyUs_ = 0; cInitFails_ = 0; cEvDrops_ = 0; cHealth_ = 0;
    txQueueFull_ = 0;
    statsResetReq_ = 0;
    lastStatsMs_ = 0;
    stRx_ = stDrop_ = stOvr_ = stInt_ = stPoll_ = stTx_ = stTxf_ = stMiss_ = 0;
    lastIntWinMs_ = 0;
    iwInt_ = iwRx_ = 0;
    intState_ = CanIoLaw::IntState::NONE;
    ovrSeen_ = 0;
    obdReplyLatched_ = false;
    obdReply_ = {};
    probeReplyLatched_ = false;
    probeReply_ = {};
    sniffPhase_ = SniffPhase::OPENING;
    sniffOvrBase_ = 0;
    sniffDropBase_ = 0;
    lwsZeroTxSeq_ = 0;
    lwsZeroTxResult_ = 0;

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
    sniffResults_ = {};

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

    // Remember CS for the raw register access of the CAN task
    csPin_ = csPin;

    // Create MCP_CAN object with CS pin. From here on only the CAN task uses it.
    mcp_can_ = new MCP_CAN(csPin);

    // The task exists before the first init and is never deleted (design D2, D7): it runs the
    // init below and, if that fails, retries it every CAN_INIT_RETRY_MS for the whole power cycle.
    if (xTaskCreatePinnedToCore(canIoTask, "can_io", CAN_TASK_STACK, this, CAN_TASK_PRIORITY,
                                &task_, CAN_TASK_CORE) != pdPASS) {
        task_ = nullptr;
        Debug::printlnFeature(DebugFeature::CAN, "[CAN] ERROR: cannot create the CAN task (out of memory) - CAN stays down");
        EventJournal::log(JSrc::CAN, JLvl::E, "init", "result=fail task=0");
        return false;
    }

#if defined(PIN_CAN_INT) && !defined(CAN_TEST_NO_INT)
    // MCP2515 INT (active low while RX0IF / RX1IF is set) wakes the task; it also polls every
    // CAN_POLL_MS, so a dead or missing INT line only costs latency (design D2).
    pinMode(PIN_CAN_INT, INPUT_PULLUP);
    attachInterruptArg(PIN_CAN_INT, canIntIsr, this, FALLING);
    hasIntPin_ = true;
    intState_ = CanIoLaw::IntState::OK;   // unproven until frames arrive
    Debug::printfFeature(DebugFeature::CAN, "[CAN] CAN task started (prio %d, core %d): INT GPIO%d + %d ms poll\n",
                         CAN_TASK_PRIORITY, CAN_TASK_CORE, (int)PIN_CAN_INT, CAN_POLL_MS);
#else
    Debug::printfFeature(DebugFeature::CAN, "[CAN] CAN task started (prio %d, core %d): no INT line, %d ms poll\n",
                         CAN_TASK_PRIORITY, CAN_TASK_CORE, CAN_POLL_MS);
#endif

    // Initialize MCP2515 at 500 kbps (in the task) and wait for the result here, in setup().
    // MCP_STDEXT (not MCP_ANY) leaves RXB0CTRL/RXB1CTRL RXM = 00 so the acceptance
    // masks/filters are actually applied; MCP_ANY sets RXM = 11 and disables them.
    postCommand(CanCmd::INIT);
    const uint32_t t0 = millis();
    while (cmdInFlight_ != CanCmd::NONE && (millis() - t0) < CAN_BOOT_INIT_WAIT_MS) {
        delay(1);                 // the task runs the init at the loop's priority meanwhile
        serviceTaskEvents();      // the INIT result completes the init here (onInitResult())
    }
    if (cmdInFlight_ != CanCmd::NONE) {
        Debug::printfFeature(DebugFeature::CAN,
            "[CAN] No init result after %u ms - handled when it arrives\n", (unsigned)CAN_BOOT_INIT_WAIT_MS);
    }
    return initialized_;
}

// ============================================================================
// CAN TASK `can_io` — the only code that touches the MCP2515 (design D1, D2)
// ============================================================================

void CANController::canIoTask(void* arg) {
    static_cast<CANController*>(arg)->taskLoop();
}

// FALLING edge on PIN_CAN_INT: count and wake the task. No SPI, no library, no queue.
void IRAM_ATTR CANController::canIntIsr(void* arg) {
    CANController* self = static_cast<CANController*>(arg);
    self->isrCount_ = self->isrCount_ + 1;
    BaseType_t woken = pdFALSE;
    if (self->task_ != nullptr) {
        vTaskNotifyGiveFromISR(self->task_, &woken);
    }
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void CANController::taskLoop() {
    for (;;) {
        // INT, a loop request (TX / command), or the CAN_POLL_MS poll. Every cycle ends here.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_POLL_MS));
        const uint32_t isr = isrCount_;
        const bool intWake = (isr != lastIsrSeen_);
        lastIsrSeen_ = isr;
        if (statsResetReq_ != statsResetAck_) {      // the loop wrote a stats line: new window
            statsResetAck_ = statsResetReq_;
            cWinHwm_ = 0;
            cWinBusyMaxUs_ = 0;
        }

        // Configuration work (command or init retry) is timed on its own: it runs at the loop's
        // priority and takes milliseconds (two 5 ms resets); the service-pass maximum below is the
        // per-cycle cost the brake follow-up's 10 ms budget relies on (design D2, D9).
        const uint32_t tc = micros();
        bool configured = false;
        const uint8_t cmd = cmdSlot_;
        if (cmd != 0) {
            cmdSlot_ = 0;   // taken; the loop posts nothing until it has read the result
            taskRunCommand(cmd);
            configured = true;
        } else if (CanIoLaw::initRetryDue(chipUp_, initAttempted_, false, millis(), lastInitTryMs_,
                                          CAN_INIT_RETRY_MS)) {
            taskInitRetry();
            configured = true;
        }
        if (configured) {
            const uint32_t cmdUs = micros() - tc;
            if (cmdUs > cCmdMaxUs_) cCmdMaxUs_ = cmdUs;
        }

        const uint32_t t0 = micros();
        uint16_t frames = 0;
        if (!chipUp_) {
            taskFailQueuedTx();
        } else {
            uint8_t st = 0;
            frames = taskDrain(st);
            taskServiceTx(st);
            if ((millis() - lastEflgMs_) >= CAN_EFLG_POLL_MS) {
                taskServiceErrors();
            }
        }

        if (intWake) {
            cIntWakes_ = cIntWakes_ + 1;
        } else {
            cPollFrames_ = cPollFrames_ + frames;
        }
        const uint32_t busy = micros() - t0;
        cBusyUs_ = cBusyUs_ + busy;
        if (busy > cBusyMaxUs_) cBusyMaxUs_ = busy;
        if (busy > cWinBusyMaxUs_) cWinBusyMaxUs_ = busy;
    }
}

void CANController::taskPostEvent(const CanEvent& ev) {
    portENTER_CRITICAL(&evMux_);
    const bool ok = evRing_.pushReject(ev);
    portEXIT_CRITICAL(&evMux_);
    if (!ok) {
        cEvDrops_ = cEvDrops_ + 1;   // TX_DONE / BUS_OFF only; command results use their own slot
    }
}

// One chip init attempt (design D6, steps 2-5). Called at CAN_TASK_CONFIG_PRIORITY.
CANController::InitResult CANController::taskInitAttempt() {
    InitResult r = {};
    // Liveness gate: SPI RESET, then CANSTAT must show CONFIG and CANCTRL its reset value 0x87.
    // No chip / floating MISO fails here in ~6 ms, without any library mode wait (audit a, g).
    spiCommand(MCP_RESET);
    delay(5);
    r.canstat = readRegister(MCP_CANSTAT);
    const uint8_t canctrl = readRegister(MCP_CANCTRL);
    if (CanIoLaw::mcp2515AliveAfterReset(r.canstat, canctrl)) {
        // Library init: resets again, 500 kbit/s, CANINTE = RX0IE | RX1IE, ends in LOOPBACK.
        // Each failing library mode switch costs up to 200 ms; stop at the first failure.
        r.begin = (mcp_can_->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK);
        // Program the acceptance window BEFORE going NORMAL (library convention: init_Mask()/
        // init_Filt() switch to CONFIG mode internally and restore the previously cached mode).
        r.filt = r.begin && applyRxFilters();
        if (r.filt) {
            // setMode() returns MCP2515_FAIL if CANSTAT never reflects the requested mode —
            // ignoring it would leave the chip in config/loopback mode, which looks like a healthy
            // init but never touches the bus. One retry, as the boot path always had.
            byte modeResult = mcp_can_->setMode(MCP_NORMAL);
            if (modeResult != MCP2515_OK) {
                modeResult = mcp_can_->setMode(MCP_NORMAL);
            }
            // Read back directly (the library exposes no getMode()): NORMAL mode and the CANINTE
            // the library wrote (RX0IE | RX1IE, which the INT wake-up relies on).
            r.canstat = readRegister(MCP_CANSTAT);
            const uint8_t inte = readRegister(MCP_CANINTE);
            r.mode = (modeResult == MCP2515_OK) && ((r.canstat & MODE_MASK) == MCP_NORMAL) &&
                     (inte == (MCP_RX0IF | MCP_RX1IF));
        } else {
            r.canstat = readRegister(MCP_CANSTAT);
        }
    }
    r.eflg = readRegister(MCP_EFLG);
    uint8_t tr[2] = {0, 0};
    readRegisters(MCP_TEC, tr, 2);
    r.tec = tr[0];
    r.rec = tr[1];
    return r;
}

// Init / re-init / filter switch requested by the loop (design D6). Runs at the loop's priority
// so the library's busy mode waits are time-sliced with the loop and never starve it.
void CANController::taskRunCommand(uint8_t cmd) {
    vTaskPrioritySet(nullptr, CAN_TASK_CONFIG_PRIORITY);

    // A frame pending in TXB0 would hold any mode change until it leaves (the MCP2515 changes
    // mode only after pending transmissions): abort it alone and report what became of it.
    taskAbortTxb0();

    CanEvent ev = {};
    ev.type = CanEvType::CMD_DONE;
    ev.cmd = cmd;
    switch ((CanCmd)cmd) {
        case CanCmd::INIT:
        case CanCmd::REINIT: {
            const bool wasUp = chipUp_;
            chipUp_ = false;
            const InitResult r = taskInitAttempt();
            const bool ok = r.begin && r.filt && r.mode;
            chipUp_ = ok;
            busOffLatched_ = false;
            // While the chip is down after this (boot or re-init), the task itself retries every
            // CAN_INIT_RETRY_MS from the last attempt of any kind (design D7, extended).
            initAttempted_ = true;
            lastInitTryMs_ = millis();
            if ((CanCmd)cmd == CanCmd::INIT) {
                initAttempts_ = 1;
                firstInitMs_ = lastInitTryMs_;
            } else if (!ok && wasUp) {
                initAttempts_ = 0;          // went down now: count the task's own retries
                firstInitMs_ = lastInitTryMs_;
            }
            ev.flags = (ok ? EV_OK : 0) | (r.begin ? EV_BEGIN : 0) | (r.filt ? EV_FILT : 0) |
                       (r.mode ? EV_MODE : 0);
            ev.canstat = r.canstat;
            ev.eflg = r.eflg;
            ev.tec = r.tec;
            ev.rec = r.rec;
            ev.attempt = initAttempts_;
            break;
        }
        case CanCmd::FILTERS_SNIFF: {
            const bool open = applySniffFilters();
            // Half-programmed filters: put the normal set back (the loop re-inits if that fails).
            const bool restored = open ? true : applyRxFilters();
            if (open) {
                // Stale overflow flags from before the window must not be counted.
                modifyRegister(MCP_EFLG, MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR, 0x00);
            }
            ev.flags = (open ? EV_OK : 0) | (restored ? EV_FILT : 0);
            ev.canstat = readRegister(MCP_CANSTAT);
            ev.eflg = readRegister(MCP_EFLG);
            break;
        }
        case CanCmd::FILTERS_NORMAL: {
            const bool ok = applyRxFilters();
            ev.flags = ok ? (EV_OK | EV_FILT) : 0;
            ev.canstat = readRegister(MCP_CANSTAT);
            ev.eflg = readRegister(MCP_EFLG);
            break;
        }
        case CanCmd::NONE:
        default:
            break;
    }
    lastEflgMs_ = millis();

    vTaskPrioritySet(nullptr, CAN_TASK_PRIORITY);
    xTaskNotifyGive(task_);   // frames latched during the command are read at once
    // Own single-entry slot: a command result can never be lost to a full event ring
    portENTER_CRITICAL(&evMux_);
    cmdResult_.post(ev);
    portEXIT_CRITICAL(&evMux_);
}

// Chip down (failed boot init, or a failed re-init sequence): one retry, gate first (~6 ms
// without a chip). Failures are only counted; a success is reported once (design D7, extended).
void CANController::taskInitRetry() {
    vTaskPrioritySet(nullptr, CAN_TASK_CONFIG_PRIORITY);
    initAttempts_++;
    const InitResult r = taskInitAttempt();
    lastInitTryMs_ = millis();
    vTaskPrioritySet(nullptr, CAN_TASK_PRIORITY);
    if (!(r.begin && r.filt && r.mode)) {
        cInitFails_ = cInitFails_ + 1;
        return;
    }
    chipUp_ = true;
    busOffLatched_ = false;
    lastEflgMs_ = millis();
    CanEvent ev = {};
    ev.type = CanEvType::INIT_OK;
    ev.flags = EV_OK | EV_BEGIN | EV_FILT | EV_MODE;
    ev.canstat = r.canstat;
    ev.eflg = r.eflg;
    ev.tec = r.tec;
    ev.rec = r.rec;
    ev.attempt = initAttempts_;
    ev.afterMs = millis() - firstInitMs_;
    xTaskNotifyGive(task_);
    portENTER_CRITICAL(&evMux_);
    recovered_.post(ev);
    portEXIT_CRITICAL(&evMux_);
}

// Read frames while READ STATUS reports one, at most CAN_RX_DRAIN_CAP (design D2). READ STATUS,
// not the INT pin, is the authority, so a dead INT line only costs latency.
uint16_t CANController::taskDrain(uint8_t& st) {
    st = readStatus();
    uint16_t n = 0;
    while (CanIoLaw::rxPending(st) && n < CAN_RX_DRAIN_CAP) {
        CanFrame f = {};
        INT32U id = 0;
        INT8U len = 0;
        // readMsgBuf() clears RXnIF (BIT MODIFY), which releases INT
        if (mcp_can_->readMsgBuf(&id, &len, f.data) != CAN_OK) {
            break;
        }
        f.id = (uint32_t)id;
        f.len = len;
        portENTER_CRITICAL(&rxMux_);
        const bool dropped = rxRing_.pushDropOldest(f);   // full: the oldest goes (design D3)
        const uint8_t cnt = rxRing_.count();
        portEXIT_CRITICAL(&rxMux_);
        if (dropped) cRxDrop_ = cRxDrop_ + 1;
        if (cnt > cRxHwm_) cRxHwm_ = cnt;
        if (cnt > cWinHwm_) cWinHwm_ = cnt;
        n++;
        st = readStatus();
    }
    if (n >= CAN_RX_DRAIN_CAP && CanIoLaw::rxPending(st)) {
        cCapHits_ = cCapHits_ + 1;   // the next wake (<= CAN_POLL_MS) continues
    }
    cRx_ = cRx_ + n;
    return n;
}

// TXB0 only, FIFO, one frame at a time; abort of this buffer only after CAN_TX_TIMEOUT_MS
// (design D4). CANCTRL.ABAT is never used.
void CANController::taskServiceTx(uint8_t st) {
    if (txBusy_) {
        if (!CanIoLaw::txPending(st, 0)) {
            // TXREQ clear. Without our abort: sent. After our abort the chip still finishes an
            // attempt that was on the wire, so ABTF tells aborted from sent (CanIoLaw::txOutcome).
            const uint8_t ctrl = txAborting_ ? readRegister(MCP_TXB0CTRL) : 0;
            if (!txReported_) {
                taskFinishTx(CanIoLaw::txOutcome(ctrl, txAborting_) == CanIoLaw::TxOutcome::SENT);
            }
            txBusy_ = false;
            txAborting_ = false;
        } else if (!txAborting_ && CanIoLaw::txTimedOut(millis(), txLoadMs_, CAN_TX_TIMEOUT_MS)) {
            // Typically no ACK (every other node unpowered): abort THIS buffer only. The result
            // is reported once TXREQ reads clear (ABTF decides).
            modifyRegister(MCP_TXB0CTRL, MCP_TXB_TXREQ_M, 0x00);
            txAborting_ = true;
            txAbortMs_ = millis();
        } else if (txAborting_ && !txReported_ &&
                   CanIoLaw::txTimedOut(millis(), txAbortMs_, CAN_TX_TIMEOUT_MS)) {
            // TXREQ still set long after the abort (wedged controller): report the frame failed,
            // keep the buffer blocked until it clears; bus-off / the watchdogs re-init the chip.
            taskFinishTx(false);
        }
        if (txBusy_) {
            return;
        }
    }
    if (CanIoLaw::txPending(st, 0)) {
        return;   // not ours (cannot happen) — never write a pending buffer
    }
    for (;;) {
        CanTxItem it;
        portENTER_CRITICAL(&txMux_);
        const bool got = txRing_.pop(it);
        portEXIT_CRITICAL(&txMux_);
        if (!got) {
            return;
        }
        if (CanIoLaw::txCancelled(it.wantResult, it.seq, txCancelSeq_)) {
            txCur_ = it;          // its requester gave up: never sent late
            txReported_ = false;
            taskFinishTx(false);
            continue;
        }
        loadTxb0(it.frame);
        txCur_ = it;
        txBusy_ = true;
        txAborting_ = false;
        txReported_ = false;
        txLoadMs_ = millis();
        return;
    }
}

// Before a command: abort a frame pending in TXB0 and wait briefly (the attempt on the wire
// ends within one frame time, ~0.25 ms) so ABTF can say whether it went out; the command's reset
// would erase that evidence. Still pending after 2 ms: reported failed (wedged).
void CANController::taskAbortTxb0() {
    if (!txBusy_) {
        return;
    }
    if (!txAborting_) {
        modifyRegister(MCP_TXB0CTRL, MCP_TXB_TXREQ_M, 0x00);
        txAborting_ = true;
        txAbortMs_ = millis();
    }
    const uint32_t t0 = micros();
    uint8_t ctrl = readRegister(MCP_TXB0CTRL);
    while ((ctrl & MCP_TXB_TXREQ_M) && (micros() - t0) < 2000) {
        ctrl = readRegister(MCP_TXB0CTRL);
    }
    const CanIoLaw::TxOutcome o = CanIoLaw::txOutcome(ctrl, true);
    if (!txReported_) {
        taskFinishTx(o == CanIoLaw::TxOutcome::SENT);
    }
    if (o != CanIoLaw::TxOutcome::PENDING) {
        txBusy_ = false;
        txAborting_ = false;
    }
}

void CANController::taskFinishTx(bool ok) {
    txReported_ = true;
    if (ok) {
        cTx_ = cTx_ + 1;
    } else {
        cTxFail_ = cTxFail_ + 1;
    }
    if (txCur_.wantResult) {
        CanEvent ev = {};
        ev.type = CanEvType::TX_DONE;
        ev.cmd = txCur_.tag;
        ev.seq = txCur_.seq;
        ev.flags = ok ? EV_OK : 0;
        taskPostEvent(ev);
    }
}

// Chip down (boot init failed, or a failed re-init): nothing can be sent. Queued frames are
// dropped as failures; a sender that asked for a result hears it at once (design D4).
void CANController::taskFailQueuedTx() {
    CanTxItem it;
    for (;;) {
        portENTER_CRITICAL(&txMux_);
        const bool got = txRing_.pop(it);
        portEXIT_CRITICAL(&txMux_);
        if (!got) {
            return;
        }
        txCur_ = it;
        taskFinishTx(false);
    }
}

// Every CAN_EFLG_POLL_MS in every mode (design D2): RXnOVR counted and cleared, TXBO reported
// once per bus-off episode, TEC / REC / EFLG snapshot for the loop's health line.
void CANController::taskServiceErrors() {
    lastEflgMs_ = millis();
    uint8_t tr[2] = {0, 0};
    readRegisters(MCP_TEC, tr, 2);
    const uint8_t eflg = readRegister(MCP_EFLG);
    cHealth_ = (uint32_t)tr[0] | ((uint32_t)tr[1] << 8) | ((uint32_t)eflg << 16) | (1UL << 24);
    const uint8_t ovr = eflg & (MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR);
    if (ovr) {
        // RXnOVR is sticky and must be cleared by software, otherwise the flags stay latched
        // forever and mask any later overflow.
        modifyRegister(MCP_EFLG, ovr, 0x00);
        cOvrEflg_ = eflg;
        cOvr_ = cOvr_ + popCount8(ovr);
    }
    if (eflg & MCP_EFLG_TXBO) {
        if (!busOffLatched_) {
            busOffLatched_ = true;
            CanEvent ev = {};
            ev.type = CanEvType::BUS_OFF;
            ev.eflg = eflg;
            ev.tec = tr[0];
            ev.rec = tr[1];
            taskPostEvent(ev);
        }
    } else {
        busOffLatched_ = false;
    }
}

// ============================================================================
// LOOP SIDE OF THE CHANNELS
// ============================================================================

bool CANController::postCommand(CanCmd cmd) {
    if (task_ == nullptr || cmdInFlight_ != CanCmd::NONE) {
        return false;   // one command at a time
    }
    cmdInFlight_ = cmd;
    cmdSlot_ = (uint8_t)cmd;
    xTaskNotifyGive(task_);
    return true;
}

bool CANController::queueTx(const CanFrame& f, CanTxTag tag, bool wantResult) {
    if (task_ == nullptr) {
        return false;
    }
    CanTxItem it;
    it.frame = f;
    it.tag = (uint8_t)tag;
    it.seq = CanIoLaw::nextTxSeq(txSeq_);
    it.wantResult = wantResult;
    portENTER_CRITICAL(&txMux_);
    const bool ok = txRing_.pushReject(it);
    portEXIT_CRITICAL(&txMux_);
    if (!ok) {
        txQueueFull_++;
        return false;
    }
    txSeq_ = it.seq;
    xTaskNotifyGive(task_);
    return true;
}

CANController::DriverStats CANController::getDriverStats() const {
    DriverStats d;
    d.taskRunning = task_ != nullptr;
    d.intState = CanIoLaw::intStateToken(intState_);
    d.rxDrop = cRxDrop_;
    d.rxOvr = cOvr_;
    d.rxHwm = (uint8_t)cRxHwm_;
    d.txFail = cTxFail_;
    d.busyMaxUs = cBusyMaxUs_;
    return d;
}

// Results from the task: command results, TX results, bus-off (design D1). Loop only.
void CANController::serviceTaskEvents() {
    CanEvent ev;
    // Single-entry slots first: the command result and the task's own init-retry success
    portENTER_CRITICAL(&evMux_);
    const bool gotCmd = cmdResult_.take(ev);
    portEXIT_CRITICAL(&evMux_);
    if (gotCmd) {
        cmdInFlight_ = CanCmd::NONE;
        switch ((CanCmd)ev.cmd) {
            case CanCmd::INIT:           onInitResult(ev); break;
            case CanCmd::REINIT:         onReinitResult(ev); break;
            case CanCmd::FILTERS_SNIFF:  onSniffOpened(ev); break;
            case CanCmd::FILTERS_NORMAL: finishSniff((ev.flags & EV_OK) != 0); break;
            case CanCmd::NONE:
            default: break;
        }
    }
    portENTER_CRITICAL(&evMux_);
    const bool gotRec = recovered_.take(ev);
    portEXIT_CRITICAL(&evMux_);
    if (gotRec) {
        onInitResult(ev);
    }
    for (;;) {
        portENTER_CRITICAL(&evMux_);
        const bool got = evRing_.pop(ev);
        portEXIT_CRITICAL(&evMux_);
        if (!got) {
            return;
        }
        switch (ev.type) {
            case CanEvType::CMD_DONE:     // carried by cmdResult_, never by the ring
            case CanEvType::INIT_OK:      // carried by recovered_
                break;
            case CanEvType::TX_DONE:
                if ((lwsZeroPhase_ == LwsZeroPhase::WAIT_RESET_TX || lwsZeroPhase_ == LwsZeroPhase::WAIT_ZERO_TX) &&
                    ev.cmd == (uint8_t)CanTxTag::LWS_CFG && ev.seq == lwsZeroTxSeq_ && lwsZeroTxResult_ == 0) {
                    lwsZeroTxResult_ = (ev.flags & EV_OK) ? 1 : 2;
                }
                break;
            case CanEvType::BUS_OFF:
                if (initialized_) {
                    handleBusOff(ev.tec, ev.rec, ev.eflg);
                }
                break;
        }
    }
}

// Take every queued frame (at most CAN_RX_RING per call) through the router (design D3).
void CANController::drainRxQueue() {
    CanFrame f;
    for (uint8_t i = 0; i < CAN_RX_RING; i++) {
        portENTER_CRITICAL(&rxMux_);
        const bool got = rxRing_.pop(f);
        portEXIT_CRITICAL(&rxMux_);
        if (!got) {
            return;
        }
        routeFrame(f);
    }
}

// One router for every received frame (design D3): LWS first, then the sniffer table, the
// OBD-II / probe reply latch, else "unmatched" (rate-limited).
void CANController::routeFrame(const CanFrame& f) {
    const bool lws = dispatchSteeringFrame(f.id, f.len, f.data);
    if (mode_ == Mode::SNIFFING) {
        if (sniffPhase_ == SniffPhase::WINDOW) {
            recordSniffFrame(f.id, f.len, f.data);   // 0x305 too, as before
        }
        return;
    }
    if (lws) {
        return;   // steering angle frame: handed to the LWS decoder
    }
    // OBD-II responses are from 0x7E8 to 0x7EF (ECU response IDs; flag bits make an extended
    // or remote frame compare larger)
    const bool obdId = (f.id >= 0x7E8 && f.id <= 0x7EF);
    if (obdId && mode_ == Mode::NORMAL && state_ == OBDState::WAITING_RESPONSE && !obdReplyLatched_) {
        // Mode 01 positive response (0x41) for the PID in flight
        const uint8_t expectedPid = pidTable_[activePIDIndex_].pid;
        if (f.len >= 3 && f.data[1] == 0x41 && f.data[2] == expectedPid) {
            obdReply_ = f;
            obdReplyLatched_ = true;
            return;
        }
    }
    if (obdId && mode_ == Mode::PROBING && probeRequestInFlight_ && !probeReplyLatched_ && f.len >= 2) {
        uint8_t expectedMode = 0;
        uint8_t expectedPid = 0;
        if (probeCurrentRequest(expectedMode, expectedPid)) {
            bool match = false;
            if (expectedMode == 0x01) {
                // Mode 01 positive response (0x41) for the expected PID
                match = (f.len >= 3 && f.data[1] == 0x41 && f.data[2] == expectedPid);
            } else if (expectedMode == 0x03) {
                // Mode 03 positive response (0x43)
                match = (f.data[1] == 0x43);
            }
            if (match) {
                probeReply_ = f;
                probeReplyLatched_ = true;
                return;
            }
        }
    }
    // Non-matching message — report it (rate-limited) so a wrong response ID or a negative
    // response (0x7F) is visible instead of being silently dropped
    logUnmatchedFrame(f.id, f.len, f.data);
}

void CANController::printFiltersLine(bool ok) const {
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] RX filters: RXB0 OBD 0x%03X-0x%03X (mask=0x%03X filt=0x%03X), "
        "RXB1 LWS 0x%03X (mask=0x%03X filt=0x%03X), std IDs only%s\n",
        (unsigned)CAN_RX_OBD_FILTER,
        (unsigned)(CAN_RX_OBD_FILTER | (~CAN_RX_OBD_MASK & 0x7FF)),
        (unsigned)CAN_RX_OBD_MASK, (unsigned)CAN_RX_OBD_FILTER,
        (unsigned)STEER_LWS_CAN_ID, (unsigned)CAN_RX_LWS_MASK, (unsigned)STEER_LWS_CAN_ID,
        ok ? "" : " [FAILED]");
}

// What a successful begin() always did; also on a later boot-retry success (design D7).
void CANController::becomeInitialized() {
    initialized_ = true;

    // Stagger the three added slow PIDs. At power-on every entry is due at once and
    // selectNextPID() breaks exact ties by highest index (`overdue >= mostOverdue`),
    // so RPM (index 0) is served last; offsetting the new entries keeps them out of
    // that burst. Done here, not in the constructor: the constructor runs before boot
    // time is meaningful, so any offset baked in there is already in the past by the
    // time begin() completes.
    uint32_t staggerBase = millis();
    // No response has arrived yet, so the no-data watchdog measures from here.
    lastReinitTime_ = staggerBase;
    pidTable_[4].nextPollTime = staggerBase + 150;
    pidTable_[5].nextPollTime = staggerBase + 300;
    pidTable_[6].nextPollTime = staggerBase + 450;
}

// Result of the boot INIT command, or of a later boot retry (INIT_OK).
void CANController::onInitResult(const CanEvent& ev) {
    const bool ok = (ev.flags & EV_OK) != 0;
    if (ev.type == CanEvType::INIT_OK && initialized_) {
        // The task's own retry healed a chip left down by a failed re-init sequence (design D7,
        // extended): journaled once, counted as a verified re-init, first frames timed again.
        printFiltersLine(true);
        Debug::printfFeature(DebugFeature::CAN,
            "[CAN] Controller back after %lu s (task retry %lu)\n",
            (unsigned long)(ev.afterMs / 1000), (unsigned long)ev.attempt);
        reinitCount_++;
        lastReinitTime_ = millis();
        EventJournal::log(JSrc::CAN, JLvl::I, "reinit", "reason=task_retry result=ok attempt=%lu after_s=%lu",
                          (unsigned long)ev.attempt, (unsigned long)(ev.afterMs / 1000));
        firstFrameWait_ = true;
        reinitDoneMs_ = millis();
        firstLwsMs_ = 0;
        firstObdMs_ = 0;
        return;
    }
    const uint8_t opmode = ev.canstat & MODE_MASK;
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
        "[CAN] CANSTAT=0x%02X -> mode %s (0x%02X)\n", ev.canstat, modeName, opmode);

    if (!ok) {
        if (!(ev.flags & EV_BEGIN)) {
            Debug::printlnFeature(DebugFeature::CAN,"[CAN] ERROR: MCP2515 initialization failed");
        } else if (!(ev.flags & EV_FILT)) {
            Debug::printlnFeature(DebugFeature::CAN,
                "[CAN] ERROR: failed to program RX acceptance masks/filters");
        } else {
            Debug::printlnFeature(DebugFeature::CAN, "[CAN] ERROR: failed to enter NORMAL mode (stuck in loopback?)");
        }
        Debug::printfFeature(DebugFeature::CAN, "[CAN] The CAN task retries the init every %u s\n",
                             (unsigned)(CAN_INIT_RETRY_MS / 1000));
        // Journaled once; later retries are counted, not journaled (design D7)
        EventJournal::log(JSrc::CAN, JLvl::E, "init", "result=fail begin=%d filt=%d mode=%d",
                          (ev.flags & EV_BEGIN) ? 1 : 0, (ev.flags & EV_FILT) ? 1 : 0,
                          (ev.flags & EV_MODE) ? 1 : 0);
        return;
    }

    printFiltersLine(true);
    becomeInitialized();
    Debug::printlnFeature(DebugFeature::CAN,"[CAN] MCP2515 initialized at 500 kbps");
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] health: TEC=%u REC=%u EFLG=0x%02X\n", ev.tec, ev.rec, ev.eflg);
    Debug::printlnFeature(DebugFeature::CAN,"[CAN] Ready to read vehicle data");
    if (ev.type == CanEvType::INIT_OK) {
        EventJournal::log(JSrc::CAN, JLvl::I, "init", "result=ok attempt=%lu after_s=%lu",
                          (unsigned long)ev.attempt, (unsigned long)(ev.afterMs / 1000));
    } else {
        // Boot line with the task's stack high-water mark (bytes), so CAN_TASK_STACK can be cut
        // later on evidence (design D2)
        const unsigned stk = task_ ? (unsigned)uxTaskGetStackHighWaterMark(task_) : 0;
        Debug::printfFeature(DebugFeature::CAN, "[CAN] CAN task stack free (high-water mark): %u B\n", stk);
        EventJournal::log(JSrc::CAN, JLvl::I, "init", "result=ok attempt=%lu after_s=0 stk=%u",
                          (unsigned long)ev.attempt, stk);
    }
}

// Window counters every JOURNAL_SNAP_MS while the task exists (design D9)
void CANController::serviceStats(uint32_t now) {
    if ((now - lastStatsMs_) < JOURNAL_SNAP_MS) {
        return;
    }
    lastStatsMs_ = now;
    const uint32_t rx = cRx_, drop = cRxDrop_, ovr = cOvr_, in = cIntWakes_, poll = cPollFrames_;
    const uint32_t tx = cTx_, txf = cTxFail_;
    const uint32_t hwm = cWinHwm_, bmax = cWinBusyMaxUs_;
    statsResetReq_ = statsResetReq_ + 1;   // the task restarts the window maxima
    char miss[12];
    if (lwsSink_ != nullptr) {
        const uint32_t m = lwsSink_->missedFrames();
        snprintf(miss, sizeof(miss), "%lu", (unsigned long)(m - stMiss_));
        stMiss_ = m;
    } else {
        strcpy(miss, "-");
    }
    EventJournal::log(JSrc::CAN, JLvl::I, "stats",
                      "rx=%lu drop=%lu hwm=%lu ovr=%lu int=%lu poll=%lu tx=%lu txf=%lu bmax=%lu miss=%s",
                      (unsigned long)(rx - stRx_), (unsigned long)(drop - stDrop_), (unsigned long)hwm,
                      (unsigned long)(ovr - stOvr_), (unsigned long)(in - stInt_),
                      (unsigned long)(poll - stPoll_), (unsigned long)(tx - stTx_),
                      (unsigned long)(txf - stTxf_), (unsigned long)bmax, miss);
    // Console only (not in the journal line, whose fields the spec fixes): session counters that
    // show a spinning task or a starved queue, and the task's live stack high-water mark (D2, D9)
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] task: cap_hits=%lu busy_ms=%lu cmd_max_us=%lu tx_qfull=%lu init_retry_fail=%lu ev_drop=%lu stk=%u B int=%s\n",
        (unsigned long)cCapHits_, (unsigned long)(cBusyUs_ / 1000), (unsigned long)cCmdMaxUs_,
        (unsigned long)txQueueFull_,
        (unsigned long)cInitFails_, (unsigned long)cEvDrops_,
        (unsigned)uxTaskGetStackHighWaterMark(task_), CanIoLaw::intStateToken(intState_));
    stRx_ = rx;
    stDrop_ = drop;
    stOvr_ = ovr;
    stInt_ = in;
    stPoll_ = poll;
    stTx_ = tx;
    stTxf_ = txf;
}

// INT wake-up state per CAN_INT_WINDOW_MS (design D9); transitions journaled once
void CANController::serviceIntState(uint32_t now) {
    if ((now - lastIntWinMs_) < CAN_INT_WINDOW_MS) {
        return;
    }
    lastIntWinMs_ = now;
    const uint32_t in = cIntWakes_;
    const uint32_t rx = cRx_;
    const CanIoLaw::IntState next = CanIoLaw::canIntState(intState_, hasIntPin_, in - iwInt_, rx - iwRx_);
    iwInt_ = in;
    iwRx_ = rx;
    if (next == intState_) {
        return;
    }
    if (next == CanIoLaw::IntState::DEAD) {
        EventJournal::log(JSrc::CAN, JLvl::W, "int", "state=dead");
    } else if (intState_ == CanIoLaw::IntState::DEAD && next == CanIoLaw::IntState::OK) {
        EventJournal::log(JSrc::CAN, JLvl::I, "int", "state=ok");
    }
    Debug::printfFeature(DebugFeature::CAN, "[CAN] RX wake-up: INT %s\n", CanIoLaw::intStateToken(next));
    intState_ = next;
}

void CANController::update() {
    if (task_ == nullptr) {
        return;   // no task (out of memory at boot): CAN permanently down
    }
    // Task results first (command / TX results, bus-off), also before the first init succeeded
    serviceTaskEvents();
    const uint32_t nowDiag = millis();
    serviceStats(nowDiag);
    serviceIntState(nowDiag);

    if (!initialized_) {
        return;
    }

    // Every queued frame, in every mode, before any state machine step (design D3)
    drainRxQueue();
    checkRxOverflow();
    logHealth();

    // A failed re-init attempt is retried here, on a later call (never looped inside one call).
    if (reinitPending_ && !reinitInFlight() && (millis() - reinitAttemptMs_) >= CAN_REINIT_RETRY_MS) {
        attemptReinit();
    }
    serviceFirstFrame(millis());

    // While sniffing, only the sniffer runs (no OBD-II requests go out).
    if (mode_ == Mode::SNIFFING) {
        updateSniff();
        if (isDataStale()) {
            vehicleData_.dataValid = false;
        }
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

    // LWS silence watchdog (normal mode only: the sniffer opens the filters, the probe pauses OBD)
    serviceLwsWatchdog(millis());

    switch (state_) {
        case OBDState::IDLE: {
            // LWS zero-set: OBD-II polling is paused (no new request) while it runs.
            // Reaching IDLE means any in-flight request has completed or timed out.
            if (lwsZeroPhase_ != LwsZeroPhase::IDLE) {
                serviceLwsZero();
                break;
            }

            int8_t index = selectNextPID();
            if (index < 0) {
                break;  // Nothing due yet
            }

            // Requests are going out but nothing is coming back: re-initialize the chip.
            // Covers a wedged TX that never leaves the chip, and the boot case where the ECU was
            // off and no response ever arrived —
            // measured from begin()/the last re-init until the first response lands.
            // Rate-limited so a genuinely absent ECU costs one re-init per interval.
            // The ECU is powered from the ACC line like the LWS: the watchdog runs only while the
            // ignition is ACC / IGNITION (same flag as the LWS watchdog) and counts the silence from
            // the later of the last reply / re-init and the ignition switch-on, so a bus silent with
            // the ignition OFF neither re-inits nor journals, and ACC/ON starts a fresh 15 s window
            // (change gate-obd-watchdog-and-pull-up-vesc-rx; CanWatchdogLaw.h).
            uint32_t now = millis();
            uint32_t noDataFor = 0;
            if (!reinitInFlight() &&
                CanWatchdogLaw::obdNoDataDue(lwsPowered_, now, vehicleData_.lastUpdateTime, lastReinitTime_,
                                             ignitionOnMs_, CAN_REINIT_NO_DATA_MS,
                                             CAN_REINIT_MIN_INTERVAL_MS, &noDataFor)) {
                Debug::printfFeature(DebugFeature::CAN,
                    "[CAN] No data for %lus, re-initializing controller\n",
                    (unsigned long)(noDataFor / 1000));
                // Journaled at the start of a silence episode, then every CAN_WD_REPEAT_LOG_MS with
                // the number of re-inits in between (an ECU that is simply off must not flood it).
                bool quiet = obdEpisode_ && (now - obdEpLogMs_) < CAN_WD_REPEAT_LOG_MS;
                if (quiet) {
                    obdEpRepeats_++;
                } else {
                    EventJournal::log(JSrc::CAN, JLvl::W, "obd_silent", "ms=%lu obd_alive=0 repeat=%lu -> reinit",
                                      (unsigned long)noDataFor, (unsigned long)obdEpRepeats_);
                    obdEpLogMs_ = now;
                    obdEpRepeats_ = 0;
                }
                obdEpisode_ = true;
                recoverFromBusOff("obd_silent", quiet);
                lastReinitTime_ = millis();
                state_ = OBDState::IDLE;
                break;  // resume polling on the next iteration, from a freshly configured chip
            }

            if (sendOBDRequest(pidTable_[index].pid)) {
                activePIDIndex_ = index;
                requestSentTime_ = millis();
                state_ = OBDState::WAITING_RESPONSE;
            } else {
                // Not queued (transmit ring full, logged by sendOBDRequest) — show the chip's
                // error state from the task's snapshot. Bus-off arrives as a task event.
                const uint32_t h = cHealth_;
                Debug::printfFeature(DebugFeature::CAN,
                    "[CAN] Send failure: TEC=%u REC=%u EFLG=0x%02X\n",
                    (unsigned)(h & 0xFF), (unsigned)((h >> 8) & 0xFF), (unsigned)((h >> 16) & 0xFF));

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
                if (firstFrameWait_ && firstObdMs_ == 0) {
                    firstObdMs_ = (millis() - reinitDoneMs_) + 1;
                }
                obdEpisode_ = false;   // the OBD silence episode is over

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
                // A frame that never left was aborted by the CAN task after CAN_TX_TIMEOUT_MS;
                // bus-off is reported by the task as an event (design D4).
                pidTable_[activePIDIndex_].retryCount++;
                // Add a short delay before the next send
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
    if (!initialized_) {
        return false;
    }

    // OBD-II request format: [length] [mode] [PID] [unused...]
    // Mode 01 = Current Data, to the OBD-II standard ID (0x7DF = broadcast).
    // Queued for the CAN task: "sent" means "queued"; a frame that never leaves the chip shows
    // up as the 200 ms reply timeout (design D4).
    CanFrame f = {};
    f.id = 0x7DF;
    f.len = 8;
    f.data[0] = 0x02;
    f.data[1] = 0x01;
    f.data[2] = pid;
    obdReplyLatched_ = false;     // a new request: any older reply is stale
    probeReplyLatched_ = false;

    if (!queueTx(f, CanTxTag::OBD, false)) {
        Debug::printfFeature(DebugFeature::CAN,"[CAN] ERROR: Failed to send request for PID 0x%02X (transmit queue full)\n", pid);
        return false;
    }
    return true;
}

// The router latched the first matching reply (design D3); take it.
bool CANController::tryReceiveResponse() {
    if (!obdReplyLatched_) {
        return false;
    }
    obdReplyLatched_ = false;

    const uint8_t expectedPid = pidTable_[activePIDIndex_].pid;
    const CanFrame& rx = obdReply_;
    const uint8_t len = rx.len;
    // Copy data payload (skip length, mode, and PID bytes)
    responseLen_ = len - 3;
    for (uint8_t j = 0; j < responseLen_ && j < 5; j++) {
        responseData_[j] = rx.data[3 + j];
    }
    // RPM diagnostics: raw reply frame, rate-limited to 1 line per second
    if (expectedPid == PID_ENGINE_RPM) {
        static uint32_t lastRpmRawLog = 0;
        if (millis() - lastRpmRawLog >= 1000) {
            lastRpmRawLog = millis();
            uint8_t d[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            for (uint8_t j = 0; j < len && j < 8; j++) {
                d[j] = rx.data[j];
            }
            Debug::printfFeature(DebugFeature::CAN,
                "[CAN] RPM raw: id=0x%03lX len=%d data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                (unsigned long)rx.id, len, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);
        }
    }
    return true;
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

// Raw MCP2515 access — CAN TASK ONLY (design D1). The coryjfowler MCP_CAN library keeps
// mcp2515_readRegister()/mcp2515_modifyRegister()/readStatus() private and has no getMode(),
// raw TX load or single-buffer abort, so these are done here over the same SPI bus the library
// uses: same SPISettings (10 MHz, MSBFIRST, SPI_MODE0) and the same CS pin. Only the task calls
// the library or these helpers, so their transactions never interleave with another context's.
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

void CANController::readRegisters(uint8_t address, uint8_t* out, uint8_t n) const {
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    SPI.transfer(MCP_READ);
    SPI.transfer(address);
    for (uint8_t i = 0; i < n; i++) {
        out[i] = SPI.transfer(0x00);   // the MCP2515 auto-increments the address
    }
    digitalWrite(csPin_, HIGH);
    SPI.endTransaction();
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

// READ STATUS: RX0IF, RX1IF and the three TXREQ bits in one transaction
uint8_t CANController::readStatus() const {
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    SPI.transfer(MCP_READ_STATUS);
    uint8_t value = SPI.transfer(0x00);
    digitalWrite(csPin_, HIGH);
    SPI.endTransaction();
    return value;
}

// One-byte instruction (RESET 0xC0, RTS TX0 0x81)
void CANController::spiCommand(uint8_t instruction) const {
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    SPI.transfer(instruction);
    digitalWrite(csPin_, HIGH);
    SPI.endTransaction();
}

// Load TXB0 with one WRITE burst (TXB0CTRL with TXP = 1, SIDH, SIDL, EID8, EID0, DLC, data) and
// request transmission with RTS TX0 (design D4). Only while TXREQ reads 0.
void CANController::loadTxb0(const CanFrame& f) {
    uint8_t buf[6 + 8];
    buf[0] = CAN_TXB0_CTRL_TXP;
    CanIoLaw::canTxHeader(f.id, (f.id & 0x80000000UL) != 0, f.len, &buf[1]);
    const uint8_t dlc = buf[5];
    memcpy(&buf[6], f.data, dlc);
    SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(csPin_, LOW);
    SPI.transfer(MCP_WRITE);
    SPI.transfer(MCP_TXB0CTRL);
    for (uint8_t i = 0; i < 6 + dlc; i++) {
        SPI.transfer(buf[i]);
    }
    digitalWrite(csPin_, HIGH);
    SPI.endTransaction();
    spiCommand(MCP_RTS_TX0);
}

// Two acceptance paths on the one shared bus (design D3):
//   RXB0 = MASK0 0x7F8 + RXF0/RXF1 0x7E8  -> OBD-II responses 0x7E8-0x7EF
//   RXB1 = MASK1 0x7FF + RXF2..RXF5 0x305 -> Bosch LWS steering angle only
// so the 100 Hz angle stream and the OBD replies never evict each other. BUKT (RXB0 ->
// RXB1 rollover) is left enabled as the library sets it in begin(): an OBD reply that
// arrives while RXB0 is full rolls into RXB1 (the receive paths accept it from either
// buffer); the only loss case is RX1OVR dropping one LWS frame, which is harmless.
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
// filter 0x7E8 << 16 therefore accepts exactly 0x7E8..0x7EF. mask 0x7FF << 16 with
// filter 0x305 << 16 accepts exactly 0x305.
// Mask 0 gates RXB0 (filters 0-1), mask 1 gates RXB1 (filters 2-5), so both masks and
// all six filters must be set or an unfiltered buffer keeps letting broadcasts through.
// ext = 0 on every mask and filter also clears EXIDE, so extended frames match nothing
// and are rejected outright. Re-applied after every chip reset (task re-init).
// CAN task only. Stops at the first failing call: each failing call waits up to 2 x 200 ms in
// the library's mode switches (design D6). Never logs (the loop prints printFiltersLine()).
bool CANController::applyRxFilters() {
    if (mcp_can_ == nullptr) {
        return false;
    }
    // RXB0: OBD-II response window
    if (mcp_can_->init_Mask(0, 0, CAN_RX_OBD_MASK_REG) != MCP2515_OK) return false;
    if (mcp_can_->init_Filt(0, 0, CAN_RX_OBD_FILTER_REG) != MCP2515_OK) return false;
    if (mcp_can_->init_Filt(1, 0, CAN_RX_OBD_FILTER_REG) != MCP2515_OK) return false;
    // RXB1: steering-angle frame only
    if (mcp_can_->init_Mask(1, 0, CAN_RX_LWS_MASK_REG) != MCP2515_OK) return false;
    for (uint8_t n = 2; n < 6; n++) {
        if (mcp_can_->init_Filt(n, 0, CAN_RX_LWS_FILTER_REG) != MCP2515_OK) return false;
    }
    return true;
}

bool CANController::recoverFromBusOff(const char* reason, bool quiet) {
    if (task_ == nullptr) {
        return false;
    }
    if (reinitInFlight()) {
        return true;   // the attempt in flight resets the chip and re-applies everything anyway
    }
    if (!reinitPending_) {
        // A new trigger starts a new sequence; a trigger arriving while a failed attempt waits
        // for its retry simply makes that retry happen now (same sequence, same budget).
        reinitReason_ = reason ? reason : "?";
        reinitAttempt_ = 0;
        reinitQuiet_ = quiet;
    }
    return attemptReinit();
}

// Post one attempt to the CAN task (design D6); onReinitResult() completes it on a later
// update(). The loop never waits for the chip.
bool CANController::attemptReinit() {
    if (!postCommand(CanCmd::REINIT)) {
        reinitPending_ = true;    // another command is in the task: posted on a later update()
        return false;
    }
    reinitPending_ = false;
    reinitAttempt_++;
    reinitAttemptMs_ = millis();
    Debug::printlnFeature(DebugFeature::CAN, "[CAN] Bus-off detected, recovering...");
    return true;
}

void CANController::onReinitResult(const CanEvent& ev) {
    // begin() resets the chip, which wipes the masks/filters — the task reprograms them before
    // returning to NORMAL or the broadcast flood comes straight back.
    const bool okBegin = (ev.flags & EV_BEGIN) != 0;
    const bool okFilt = (ev.flags & EV_FILT) != 0;
    const bool okMode = (ev.flags & EV_MODE) != 0;
    if (okBegin) {
        printFiltersLine(okFilt);
    }
    if (okFilt && !okMode) {
        Debug::printlnFeature(DebugFeature::CAN, "[CAN] ERROR: failed to enter NORMAL mode (stuck in loopback?)");
    }
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] Recovery: CANSTAT=0x%02X EFLG=0x%02X\n", ev.canstat, ev.eflg);

    const bool ok = (ev.flags & EV_OK) != 0;
    if (ok) {
        reinitPending_ = false;
        reinitCount_++;
        if (!reinitQuiet_) {
            EventJournal::log(JSrc::CAN, JLvl::I, "reinit", "reason=%s result=ok attempt=%u",
                              reinitReason_, (unsigned)reinitAttempt_);
            // Time to the first LWS frame / OBD reply after this re-init
            firstFrameWait_ = true;
            reinitDoneMs_ = millis();
            firstLwsMs_ = 0;
            firstObdMs_ = 0;
        }
    } else if (reinitAttempt_ >= CAN_REINIT_MAX_ATTEMPTS) {
        reinitPending_ = false;   // the next watchdog interval / bus-off event tries again
        EventJournal::log(JSrc::CAN, JLvl::E, "reinit", "reason=%s result=fail attempt=%u begin=%d filt=%d mode=%d",
                          reinitReason_, (unsigned)reinitAttempt_, okBegin ? 1 : 0, okFilt ? 1 : 0, okMode ? 1 : 0);
    } else {
        reinitPending_ = true;    // retried on a later update() call
        Debug::printfFeature(DebugFeature::CAN,
            "[CAN] Re-init attempt %u failed (begin=%d filters=%d mode=%d) — retrying\n",
            (unsigned)reinitAttempt_, okBegin ? 1 : 0, okFilt ? 1 : 0, okMode ? 1 : 0);
    }
}

void CANController::handleBusOff(uint8_t tec, uint8_t rec, uint8_t eflg) {
    EventJournal::log(JSrc::CAN, JLvl::E, "bus_off", "tec=%u rec=%u eflg=0x%02X",
                      (unsigned)tec, (unsigned)rec, (unsigned)eflg);
    recoverFromBusOff("bus_off");
}

void CANController::setLwsPowered(bool on) {
    if (!(on && lwsPowered_)) {
        lwsSeenSinceOn_ = false;   // off, or switched on just now: disarmed until the next 0x305 frame
    }
    if (on && !lwsPowered_) {
        // The ECU hangs on the same ACC line: the OBD-II no-data window starts fresh here, and the
        // first silence of this power cycle is journaled again (change
        // gate-obd-watchdog-and-pull-up-vesc-rx).
        ignitionOnMs_ = millis();
        obdEpisode_ = false;
    }
    lwsPowered_ = on;
}

void CANController::serviceLwsWatchdog(uint32_t now) {
    // Decision 1: only while the LWS is powered (ignition ACC / IGNITION) and armed by a frame
    // seen since the ignition was switched on. Ignition OFF = legitimately silent: no event.
    if (!lwsPowered_ || !lwsSeenSinceOn_ || lwsSink_ == nullptr) return;
    if (lwsZeroPhase_ != LwsZeroPhase::IDLE) return;   // a zero-set is running
    const uint32_t silentMs = now - lastLwsFrameMs_;
    if (silentMs <= CAN_LWS_SILENT_MS) return;

    const bool obdAlive = vehicleData_.lastUpdateTime != 0 &&
                          (now - vehicleData_.lastUpdateTime) <= CAN_LWS_WD_OBD_ALIVE_MS;
    if (!obdAlive) {
        // The whole bus is quiet: left to the OBD-II no-data watchdog. Journaled once per episode.
        if (!lwsSilentObdDeadLogged_) {
            lwsSilentObdDeadLogged_ = true;
            EventJournal::log(JSrc::CAN, JLvl::W, "lws_silent", "ms=%lu obd_alive=0", (unsigned long)silentMs);
        }
        return;
    }
    // Fixed spacing from the last re-init of any kind; no back-off (decision 1).
    if ((now - lastReinitTime_) < CAN_LWS_REINIT_MIN_MS || reinitPending_ || reinitInFlight()) return;

    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] No LWS 0x305 frame for %lu ms while the ECU answers — re-initializing controller\n",
        (unsigned long)silentMs);
    // Same journal economy as the OBD watchdog: a dead sensor with the ECU on is re-initialized
    // every 5 s (decision 1), journaled at the start of the episode and every 10 min after.
    const bool quiet = lwsEpisode_ && (now - lwsEpLogMs_) < CAN_WD_REPEAT_LOG_MS;
    if (quiet) {
        lwsEpRepeats_++;
    } else {
        EventJournal::log(JSrc::CAN, JLvl::W, "lws_silent", "ms=%lu obd_alive=1 repeat=%lu -> reinit",
                          (unsigned long)silentMs, (unsigned long)lwsEpRepeats_);
        lwsEpLogMs_ = now;
        lwsEpRepeats_ = 0;
    }
    lwsEpisode_ = true;
    recoverFromBusOff("lws_silent", quiet);
    lastReinitTime_ = millis();
    state_ = OBDState::IDLE;
}

void CANController::serviceFirstFrame(uint32_t now) {
    if (!firstFrameWait_) return;
    const bool done = (firstLwsMs_ != 0 && firstObdMs_ != 0) ||
                      (now - reinitDoneMs_) >= CAN_FIRST_FRAME_WAIT_MS;
    if (!done) return;
    firstFrameWait_ = false;
    char lws[12];
    char obd[12];
    if (firstLwsMs_) snprintf(lws, sizeof(lws), "%lu", (unsigned long)(firstLwsMs_ - 1)); else strcpy(lws, "-");
    if (firstObdMs_) snprintf(obd, sizeof(obd), "%lu", (unsigned long)(firstObdMs_ - 1)); else strcpy(obd, "-");
    EventJournal::log(JSrc::CAN, JLvl::I, "first_frame", "lws_ms=%s obd_ms=%s", lws, obd);
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
    if (vehicleData_.dataValid) {
        return;  // only interesting while no data is coming back
    }

    uint32_t now = millis();
    if (now - lastHealthLogTime_ < CAN_HEALTH_LOG_INTERVAL) {
        return;
    }
    lastHealthLogTime_ = now;

    const uint32_t h = cHealth_;   // the task's TEC / REC / EFLG snapshot (one 32-bit write)
    if ((h >> 24) == 0) {
        return;   // no snapshot yet
    }
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] health: TEC=%u REC=%u EFLG=0x%02X\n",
        (unsigned)(h & 0xFF), (unsigned)((h >> 8) & 0xFF), (unsigned)((h >> 16) & 0xFF));
}

void CANController::checkRxOverflow() {
    // The task counts and clears RX0OVR / RX1OVR every CAN_EFLG_POLL_MS (design D2).
    const uint32_t ovr = cOvr_;
    if (ovr == ovrSeen_) {
        return;
    }
    const uint32_t fresh = ovr - ovrSeen_;
    ovrSeen_ = ovr;

    // Hardware filtering should make overflows rare, but a burst still floods the log
    // one line per loop iteration — report at most once per CAN_OVERFLOW_LOG_INTERVAL
    // and carry the suppressed count so the real rate stays visible.
    uint32_t now = millis();
    if (lastOverflowLogTime_ != 0 && (now - lastOverflowLogTime_) < CAN_OVERFLOW_LOG_INTERVAL) {
        rxOverflowSuppressed_ += fresh;
        return;
    }
    lastOverflowLogTime_ = now;

    Debug::printfFeature(DebugFeature::CAN,
        "[CAN] RX overflow (EFLG=0x%02X), cleared (x%lu since last report)\n",
        (unsigned)cOvrEflg_, (unsigned long)(rxOverflowSuppressed_ + fresh - 1));
    rxOverflowSuppressed_ = 0;
}

// ============================================================================
// BOSCH LWS STEERING-ANGLE SENSOR (shared bus)
// ============================================================================

bool CANController::dispatchSteeringFrame(unsigned long canId, uint8_t len, const uint8_t* buf) {
    // readMsgBuf() ORs 0x80000000 into the ID of an extended frame and 0x40000000 into a
    // remote request, so only a standard data frame compares equal to 0x305.
    if (canId != (unsigned long)STEER_LWS_CAN_ID) {
        return false;
    }
    lastLwsFrameMs_ = millis();
    lwsSilentObdDeadLogged_ = false;
    lwsEpisode_ = false;   // the LWS silence episode is over
    if (lwsPowered_) {
        lwsSeenSinceOn_ = true;   // arms the LWS silence watchdog
    }
    if (firstFrameWait_ && firstLwsMs_ == 0) {
        firstLwsMs_ = (lastLwsFrameMs_ - reinitDoneMs_) + 1;
    }
    if (lwsSink_ != nullptr) {
        lwsSink_->onFrame(buf, len, millis());
    }
    return true;
}

// The ONLY transmitter on 0x7C0 in the firmware. The payload is chosen here from the two
// fixed constants in Lws305.h — no caller can supply one, so [03 00] (which would move
// the sensor's output to ID 0x000) can never be sent. Queued with a TX result request
// (design D4): serviceLwsZero() waits for it.
bool CANController::sendLwsCfg(bool zeroFrame) {
    if (!initialized_) {
        return false;
    }
    const uint8_t* src = zeroFrame ? LWS_CFG_ZERO_ID305 : LWS_CFG_RESET;
    CanFrame f = {};
    f.id = STEER_LWS_CFG_ID;
    f.len = 2;
    f.data[0] = src[0];
    f.data[1] = src[1];
    const bool queued = queueTx(f, CanTxTag::LWS_CFG, true);
    lwsZeroTxSeq_ = txSeq_;
    lwsZeroTxResult_ = 0;
    Debug::printfFeature(DebugFeature::CAN, "[CAN] LWS 0x%03X [%02X %02X] %s\n",
                         (unsigned)STEER_LWS_CFG_ID, f.data[0], f.data[1],
                         queued ? "queued" : "SEND FAILED (queue full)");
    return queued;
}

void CANController::abortLwsZero() {
    lwsZeroAbort_ = "tx_failed";
    lwsZeroPhase_ = LwsZeroPhase::IDLE;
    Debug::printlnFeature(DebugFeature::CAN, "[CAN] LWS zero-set aborted (tx_failed) — OBD-II resumed");
}

CANController::LwsZeroStart CANController::startLwsZero() {
    if (!initialized_) {
        return LwsZeroStart::NOT_INITIALIZED;
    }
    if (mode_ == Mode::PROBING) {
        return LwsZeroStart::BUSY_PROBE;   // short and bounded: the operator presses again
    }
    if (mode_ == Mode::SNIFFING) {
        return LwsZeroStart::BUSY;         // filters are open; zero-set waits for the window to end
    }
    if (lwsZeroPhase_ != LwsZeroPhase::IDLE) {
        return LwsZeroStart::BUSY;
    }
    lwsZeroResetSent_ = false;
    lwsZeroZeroSent_ = false;
    lwsZeroSentMs_ = 0;
    lwsZeroAbort_ = nullptr;
    lwsZeroPhase_ = LwsZeroPhase::WAIT_IDLE;
    lwsZeroPhaseMs_ = millis();
    Debug::printlnFeature(DebugFeature::CAN, "[CAN] LWS zero-set started — OBD-II polling paused");
    return LwsZeroStart::STARTED;
}

bool CANController::isLwsZeroRunning() const {
    return lwsZeroPhase_ != LwsZeroPhase::IDLE;
}

void CANController::serviceLwsZero() {
    // Called only while state_ == IDLE in NORMAL mode: no OBD-II request is in flight.
    // TX results were taken from the task's events at the start of this update().
    uint32_t now = millis();
    switch (lwsZeroPhase_) {
        case LwsZeroPhase::IDLE:
            return;

        case LwsZeroPhase::WAIT_IDLE:
            lwsZeroPhase_ = LwsZeroPhase::SEND_RESET;
            lwsZeroPhaseMs_ = now;
            // fall through: the bus is ours now
            [[fallthrough]];

        case LwsZeroPhase::SEND_RESET:
            if (!sendLwsCfg(false)) {
                abortLwsZero();   // never queued: the sensor's zero is untouched
                return;
            }
            // Queued = it may reach the sensor whatever the TX result says later: from here on
            // the old zero is not guaranteed and the steering erases its limits (err toward
            // "calibration lost").
            lwsZeroResetSent_ = true;
            lwsZeroPhase_ = LwsZeroPhase::WAIT_RESET_TX;
            lwsZeroPhaseMs_ = millis();
            return;

        case LwsZeroPhase::WAIT_RESET_TX:
            if (lwsZeroTxResult_ == 0 && (now - lwsZeroPhaseMs_) < CAN_TX_RESULT_WAIT_MS) {
                return;   // non-blocking wait for the task's TX result (it always answers)
            }
            if (lwsZeroTxResult_ == 0) {
                txCancelSeq_ = lwsZeroTxSeq_;   // giving up: the task must not send it later
            }
            if (lwsZeroTxResult_ != 1) {
                Debug::printfFeature(DebugFeature::CAN, "[CAN] LWS 0x%03X [05 00] SEND FAILED\n",
                                     (unsigned)STEER_LWS_CFG_ID);
                abortLwsZero();
                return;
            }
            Debug::printfFeature(DebugFeature::CAN, "[CAN] LWS 0x%03X [05 00] sent\n", (unsigned)STEER_LWS_CFG_ID);
            lwsZeroPhase_ = LwsZeroPhase::GAP;
            lwsZeroPhaseMs_ = millis();   // the gap starts at the confirmed send
            return;

        case LwsZeroPhase::GAP:
            if (now - lwsZeroPhaseMs_ < STEER_LWS_ZERO_GAP_MS) {
                return;   // non-blocking wait
            }
            lwsZeroPhase_ = LwsZeroPhase::SEND_ZERO;
            [[fallthrough]];

        case LwsZeroPhase::SEND_ZERO:
            if (!sendLwsCfg(true)) {
                abortLwsZero();
                return;
            }
            lwsZeroPhase_ = LwsZeroPhase::WAIT_ZERO_TX;
            lwsZeroPhaseMs_ = millis();
            return;

        case LwsZeroPhase::WAIT_ZERO_TX:
            if (lwsZeroTxResult_ == 0 && (now - lwsZeroPhaseMs_) < CAN_TX_RESULT_WAIT_MS) {
                return;
            }
            if (lwsZeroTxResult_ == 0) {
                txCancelSeq_ = lwsZeroTxSeq_;   // giving up: the task must not send it later
            }
            if (lwsZeroTxResult_ != 1) {
                Debug::printfFeature(DebugFeature::CAN, "[CAN] LWS 0x%03X [53 30] SEND FAILED\n",
                                     (unsigned)STEER_LWS_CFG_ID);
                abortLwsZero();
                return;
            }
            lwsZeroZeroSent_ = true;
            lwsZeroSentMs_ = millis();
            lwsZeroPhase_ = LwsZeroPhase::IDLE;   // DONE: polling resumes where the scheduler left off
            Debug::printlnFeature(DebugFeature::CAN, "[CAN] LWS zero-set frames sent — OBD-II resumed");
            return;
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
    if (!initialized_) {
        return false;
    }

    // Mode 03 = request stored DTCs (no PID byte)
    CanFrame f = {};
    f.id = 0x7DF;
    f.len = 8;
    f.data[0] = 0x01;
    f.data[1] = 0x03;
    obdReplyLatched_ = false;
    probeReplyLatched_ = false;

    if (!queueTx(f, CanTxTag::OBD, false)) {
        Debug::printlnFeature(DebugFeature::CAN, "[CAN] ERROR: Failed to send Mode 03 request (transmit queue full)");
        return false;
    }
    return true;
}

// The router latched the first Mode 01 (0x41 + pid) / Mode 03 (0x43) reply (design D3); take it.
bool CANController::tryReceiveProbeResponse(uint8_t expectedMode, uint8_t expectedPid,
                                            uint8_t* outBuf, uint8_t& outLen, uint16_t& outCanId) {
    (void)expectedMode;   // matched by routeFrame() against the same probeCurrentRequest()
    (void)expectedPid;
    if (!probeReplyLatched_) {
        return false;
    }
    probeReplyLatched_ = false;
    outCanId = (uint16_t)probeReply_.id;
    outLen = probeReply_.len;
    for (uint8_t j = 0; j < probeReply_.len && j < 8; j++) {
        outBuf[j] = probeReply_.data[j];
    }
    return true;
}

CANController::ProbeStart CANController::startProbe(bool gearChangeActive) {
    // Reject if a probe or the sniffer is already running, or an LWS zero-set holds the bus
    if (mode_ != Mode::NORMAL || lwsZeroPhase_ != LwsZeroPhase::IDLE) {
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

void CANController::recordProbeResponse(uint8_t mode, uint8_t pid, uint16_t canId, const uint8_t* buf, uint8_t len) {
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
        // Whole reply frame (ID, DLC, all 8 bytes) for diagnostics
        pr.rxId = canId;
        pr.rxDlc = len;
        for (uint8_t j = 0; j < 8; j++) {
            pr.rxFrame[j] = (j < len) ? buf[j] : 0;
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
            case 0x0C: pr.decoded = ((A * 256.0f) + B) / 4.0f; break; // engine RPM
            case 0x05: pr.decoded = A - 40.0f; break;             // coolant °C
            default:   pr.hasDecoded = false; break;
        }

        Debug::printfFeature(DebugFeature::CAN,
            "[CAN][PROBE] PID 0x%02X answered (%u bytes)%s id=0x%03X len=%u data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
            pid, pr.rawLen,
            pr.hasDecoded ? "" : " [raw only]",
            pr.rxId, pr.rxDlc,
            pr.rxFrame[0], pr.rxFrame[1], pr.rxFrame[2], pr.rxFrame[3],
            pr.rxFrame[4], pr.rxFrame[5], pr.rxFrame[6], pr.rxFrame[7]);
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
    uint16_t rxId = 0;
    if (tryReceiveProbeResponse(mode, pid, rxBuf, rxLen, rxId)) {
        recordProbeResponse(mode, pid, rxId, rxBuf, rxLen);
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

// ============================================================================
// CAN BUS SNIFFER
// ============================================================================

// Open both receive buffers to everything. The filter's EXIDE bit decides whether it matches
// standard or extended frames (the masks have no such bit), so each buffer gets one standard
// and one extended filter, all ID bits "don't care" through mask 0. BUKT stays as begin() set it.
// CAN task only; stops at the first failing call; never logs.
bool CANController::applySniffFilters() {
    if (mcp_can_ == nullptr) {
        return false;
    }
    if (mcp_can_->init_Mask(0, 0, 0) != MCP2515_OK) return false;
    if (mcp_can_->init_Filt(0, 0, 0) != MCP2515_OK) return false;   // RXB0: standard
    if (mcp_can_->init_Filt(1, 1, 0) != MCP2515_OK) return false;   // RXB0: extended
    if (mcp_can_->init_Mask(1, 0, 0) != MCP2515_OK) return false;
    if (mcp_can_->init_Filt(2, 0, 0) != MCP2515_OK) return false;   // RXB1: standard
    if (mcp_can_->init_Filt(3, 1, 0) != MCP2515_OK) return false;   // RXB1: extended
    if (mcp_can_->init_Filt(4, 0, 0) != MCP2515_OK) return false;
    if (mcp_can_->init_Filt(5, 1, 0) != MCP2515_OK) return false;
    return true;
}

CANController::SniffStart CANController::startSniff(uint32_t ms, bool gearChangeActive) {
    if (!initialized_) {
        return SniffStart::NOT_INITIALIZED;
    }
    if (mode_ == Mode::PROBING) {
        return SniffStart::BUSY_PROBE;
    }
    if (lwsZeroPhase_ != LwsZeroPhase::IDLE) {
        return SniffStart::BUSY_LWS_ZERO;
    }
    if (mode_ == Mode::SNIFFING || gearChangeActive) {
        return SniffStart::BUSY;
    }
    if (cmdInFlight_ != CanCmd::NONE) {
        return SniffStart::BUSY;   // a re-init is running in the CAN task
    }
    if (ms == 0) ms = 1;
    if (ms > CAN_SNIFF_MAX_WINDOW_MS) ms = CAN_SNIFF_MAX_WINDOW_MS;

    sniffResults_ = {};
    sniffExtraCount_ = 0;
    sniffResults_.windowMs = ms;

    // The CAN task opens the filters (design D6); the window starts on its result
    // (onSniffOpened()). OBD-II polling pauses now: an in-flight request is abandoned.
    if (!postCommand(CanCmd::FILTERS_SNIFF)) {
        return SniffStart::BUSY;
    }
    sniffPhase_ = SniffPhase::OPENING;
    mode_ = Mode::SNIFFING;
    Debug::printfFeature(DebugFeature::CAN,
        "[CAN][SNIFF] Opening the RX filters (%lu ms window) — OBD-II polling paused\n", (unsigned long)ms);
    return SniffStart::STARTED;
}

void CANController::onSniffOpened(const CanEvent& ev) {
    if (mode_ != Mode::SNIFFING || sniffPhase_ != SniffPhase::OPENING) {
        return;
    }
    if (ev.flags & EV_OK) {
        Debug::printlnFeature(DebugFeature::CAN, "[CAN][SNIFF] RX filters opened: all standard + extended IDs");
        sniffResults_.running = true;
        sniffResults_.startedAtMs = millis();
        sniffOvrBase_ = cOvr_;
        sniffDropBase_ = cRxDrop_;
        sniffPhase_ = SniffPhase::WINDOW;
        Debug::printfFeature(DebugFeature::CAN,
            "[CAN][SNIFF] Started (%lu ms) — OBD-II polling paused\n", (unsigned long)sniffResults_.windowMs);
        return;
    }
    // Half-programmed filters: the task put the normal set back (EV_FILT) or could not.
    Debug::printlnFeature(DebugFeature::CAN, "[CAN][SNIFF] RX filters opened: [FAILED] — sniff ends without frames");
    sniffResults_.openFailed = true;
    finishSniff((ev.flags & EV_FILT) != 0);
}

void CANController::recordSniffFrame(unsigned long canId, uint8_t len, const uint8_t* buf) {
    // readMsgBuf(): bit 31 = extended, bit 30 = remote request
    bool ext = (canId & 0x80000000UL) != 0;
    uint32_t id = (uint32_t)(canId & (ext ? 0x1FFFFFFFUL : 0x7FFUL));
    uint8_t dlc = len > 8 ? 8 : len;
    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (!(canId & 0x40000000UL)) {   // a remote frame carries no data
        for (uint8_t j = 0; j < dlc; j++) data[j] = buf[j];
    }

    sniffResults_.frames++;

    for (uint8_t i = 0; i < sniffResults_.idCount; i++) {
        SniffEntry& e = sniffResults_.entries[i];
        if (e.id == id && e.ext == ext) {
            for (uint8_t j = 0; j < 8; j++) {
                if (e.data[j] != data[j]) e.changed |= (uint8_t)(1u << j);
                e.data[j] = data[j];
            }
            e.dlc = dlc;
            e.count++;
            return;
        }
    }

    if (sniffResults_.idCount < CAN_SNIFF_MAX_IDS) {
        SniffEntry& e = sniffResults_.entries[sniffResults_.idCount++];
        e.id = id;
        e.ext = ext;
        e.dlc = dlc;
        e.count = 1;
        e.changed = 0;
        for (uint8_t j = 0; j < 8; j++) e.data[j] = data[j];
        return;
    }

    // Table full: count distinct extra IDs (ext flag folded into bit 31 of the key)
    uint32_t key = id | (ext ? 0x80000000UL : 0);
    for (uint8_t i = 0; i < sniffExtraCount_; i++) {
        if (sniffExtraIds_[i] == key) return;
    }
    if (sniffExtraCount_ < CAN_SNIFF_MAX_IDS) {
        sniffExtraIds_[sniffExtraCount_++] = key;
        sniffResults_.overflowIds++;
    } else {
        sniffResults_.overflowIdsSaturated = true;   // further extra IDs can no longer be told apart
    }
}

// Window end, then the filter restore in the task (design D6)
void CANController::updateSniff() {
    if (sniffPhase_ == SniffPhase::WINDOW &&
        (millis() - sniffResults_.startedAtMs) >= sniffResults_.windowMs) {
        closeSniffWindow();
    }
    // A re-init may hold the command slot: the restore is posted as soon as it is free
    if (sniffPhase_ == SniffPhase::CLOSE_PENDING && postCommand(CanCmd::FILTERS_NORMAL)) {
        sniffPhase_ = SniffPhase::CLOSING;
    }
}

// Stop recording; the window's overflow / queue-drop counts are the task's counter deltas
void CANController::closeSniffWindow() {
    sniffResults_.overflows = cOvr_ - sniffOvrBase_;
    sniffResults_.queueDrops = cRxDrop_ - sniffDropBase_;
    sniffPhase_ = SniffPhase::CLOSE_PENDING;
}

// Normal filters back (restoreOk) or not; called on the FILTERS_NORMAL result, or at once
// when the filters could not be opened.
void CANController::finishSniff(bool restoreOk) {
    if (mode_ != Mode::SNIFFING) {
        return;
    }
    sniffResults_.running = false;
    sniffResults_.complete = true;
    sniffResults_.restoreFailed = !restoreOk;
    sniffResults_.completedAtMs = millis();
    mode_ = Mode::NORMAL;
    state_ = OBDState::IDLE;
    // No replies came back during the window: keep the no-data watchdog from forcing a chip
    // re-init on the first poll after it (it is rate-limited from lastReinitTime_).
    lastReinitTime_ = millis();
    if (!restoreOk) {
        Debug::printlnFeature(DebugFeature::CAN,
            "[CAN][SNIFF] Filter restore failed — re-initialising the controller");
        recoverFromBusOff("sniff_restore");   // full reset + applyRxFilters() + NORMAL, in the task
    }

    Debug::printfFeature(DebugFeature::CAN,
        "[CAN][SNIFF] Done: %u IDs (+%u beyond table%s), %lu frames, %lu overflows, %lu queue drops%s — OBD-II resumed\n",
        sniffResults_.idCount, sniffResults_.overflowIds,
        sniffResults_.overflowIdsSaturated ? "+" : "",
        (unsigned long)sniffResults_.frames, (unsigned long)sniffResults_.overflows,
        (unsigned long)sniffResults_.queueDrops, sniffResults_.openFailed ? " (open failed)" : "");
    for (uint8_t i = 0; i < sniffResults_.idCount; i++) {
        const SniffEntry& e = sniffResults_.entries[i];
        Debug::printfFeature(DebugFeature::CAN,
            "[CAN][SNIFF] %s%0*lX [%u] n=%lu chg=%02X data=%02X %02X %02X %02X %02X %02X %02X %02X\n",
            e.ext ? "x" : "", e.ext ? 8 : 3, (unsigned long)e.id, e.dlc,
            (unsigned long)e.count, e.changed,
            e.data[0], e.data[1], e.data[2], e.data[3],
            e.data[4], e.data[5], e.data[6], e.data[7]);
    }
}
