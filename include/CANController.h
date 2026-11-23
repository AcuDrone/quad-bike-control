#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <Arduino.h>
#include <mcp_can.h>
#include "CanIoLaw.h"
#include "Constants.h"

class LwsSensor;

/**
 * @brief CAN bus controller for reading vehicle OBD-II data via MCP2515
 *
 * Interfaces with MCP2515 SPI CAN controller to read standard OBD-II diagnostic data
 * from vehicle ECU (engine RPM, speed, coolant temperature, etc.).
 * Uses a non-blocking state machine to avoid stalling the main loop.
 *
 * The same bus carries the Bosch LWS steering-angle sensor (0x305, 100 Hz): it has its
 * own receive buffer (RXB1, see applyRxFilters()), every received 0x305 frame goes to the
 * attached LwsSensor in any CAN state, and the LWS zero-set sequence (startLwsZero()) is the
 * only code that transmits on 0x7C0.
 *
 * Threading (change move-can-rx-to-interrupt): the MCP2515 has ONE owner, the FreeRTOS task
 * `can_io` (core 1, priority CAN_TASK_PRIORITY), woken by the INT line (GPIO38, FALLING) or
 * every CAN_POLL_MS. Every SPI access — init, re-init, filters, receive, transmit, EFLG — runs
 * in that task (the task* methods and the helpers they call). loop() (update() and every
 * public method) never touches SPI and never waits on the chip; it talks to the task only
 * through:
 *   - rxRing_  (task -> loop, CAN_RX_RING frames, full = drop the oldest), rxMux_;
 *   - txRing_  (loop -> task, CAN_TX_RING frames, full = reject the newest), txMux_;
 *   - evRing_  (task -> loop, command / TX results and bus-off), evMux_;
 *   - cmdSlot_ (loop -> task, one command at a time: written by the loop only while no
 *     command is in flight, cleared by the task before it posts the result);
 *   - counters / snapshots written by the task only (aligned 32-bit, read by the loop).
 * Each ring's portMUX is held only for one element copy. The consumers (LWS decoder, OBD-II
 * and probe state machines, sniffer table, watchdogs, journal) stay loop-only.
 */
class CANController {
public:
    /**
     * @brief Vehicle data structure from CAN bus
     */
    struct VehicleData {
        uint16_t engineRPM;         // RPM (0-16383)
        uint8_t vehicleSpeed;       // km/h (0-255)
        int8_t coolantTemp;         // °C (-40 to +215)
        int8_t oilTemp;             // °C (-40 to +215)
        uint8_t throttlePosition;   // % (0-100)
        uint8_t fuelLevel;          // % (0-100)
        uint8_t mapKpa;             // Manifold absolute pressure, kPa absolute (0-255)
        int8_t intakeTemp;          // Intake air temperature °C (-40 to +215)
        uint8_t engineLoad;         // Calculated engine load % (0-100)
        uint32_t lastUpdateTime;    // millis() timestamp of last successful update
        bool dataValid;             // true if CAN communication is healthy
    };

    // ---- ECU capability probe (on-demand diagnostic sweep) ------------------
    static constexpr uint8_t PROBE_CANDIDATE_COUNT = 11; // candidate PIDs test-requested
    static constexpr uint8_t PROBE_MAX_DTC = 2;          // DTCs that fit one classic 8-byte frame

    // Supported-PID bitmap group (Mode 01 PID 0x00 / 0x20 / 0x40 / 0x60)
    struct ProbeBitmap {
        bool queried;      // request was sent for this group
        bool answered;     // ECU returned a bitmap
        uint8_t raw[4];    // 4-byte supported-PID bitmap
    };

    // Per-candidate-PID probe result
    struct ProbePidResult {
        uint8_t pid;               // OBD-II PID tested
        bool supportedByBitmap;    // reported supported by a Mode 01 bitmap
        bool answered;             // ECU returned a positive (0x41) response
        uint8_t rawLen;            // number of data bytes stored in raw[]
        uint8_t raw[4];            // raw data payload (after mode+PID)
        float decoded;             // decoded engineering value (valid iff hasDecoded)
        bool hasDecoded;           // true when a known formula produced `decoded`
        uint16_t rxId;             // CAN ID of the matched reply frame (valid iff answered)
        uint8_t rxDlc;             // DLC of the matched reply frame
        uint8_t rxFrame[8];        // all 8 data bytes of the reply frame, as received
    };

    // Stored-DTC summary (Mode 03, single classic frame)
    struct ProbeDtc {
        bool queried;                    // Mode 03 request was sent
        bool answered;                   // ECU returned a 0x43 response
        uint8_t count;                   // DTC count reported by the ECU
        uint16_t codes[PROBE_MAX_DTC];   // raw 2-byte DTC codes captured from the frame
        uint8_t codeCount;               // number of codes actually captured
    };

    // Full probe snapshot (fixed-size, no heap)
    struct ProbeResults {
        bool running;                 // probe sweep in progress
        bool complete;                // a probe has completed at least once
        uint32_t completedAtMs;       // millis() when the last probe completed
        bool multiFrameTruncated;     // DTC count exceeded a single frame (codes truncated)
        char status[24];              // short human-readable status
        ProbeBitmap bitmaps[4];       // groups 0x00 / 0x20 / 0x40 / 0x60
        ProbePidResult pids[PROBE_CANDIDATE_COUNT];
        ProbeDtc dtc;
    };

    // Outcome of a startProbe() request
    enum class ProbeStart : uint8_t { STARTED, BUSY, NO_ECU };

    // ---- CAN bus sniffer (on-demand, all IDs for a fixed window) ---------------
    // One row per distinct ID seen during the window
    struct SniffEntry {
        uint32_t id;        // 11- or 29-bit identifier (flags stripped)
        bool ext;           // extended (29-bit) frame
        uint8_t dlc;        // DLC of the last frame
        uint32_t count;     // frames received with this ID
        uint8_t data[8];    // last data bytes (zero-padded past dlc)
        uint8_t changed;    // bit n set once byte n differed from the previous frame of this ID
    };

    // Sniffer snapshot (fixed-size, no heap)
    struct SniffResults {
        bool running;               // window in progress
        bool complete;              // a sniff has completed at least once
        bool restoreFailed;         // applyRxFilters() failed at the end (chip was re-initialised)
        uint32_t startedAtMs;       // millis() at start
        uint32_t completedAtMs;     // millis() at the end of the last window
        uint32_t windowMs;          // requested window length
        bool openFailed;            // the task could not open the filters: completed at once, no frames
        uint32_t frames;            // all frames read during the window
        uint32_t overflows;         // RX0OVR/RX1OVR flags counted by the CAN task in the window
        uint32_t queueDrops;        // frames dropped from the full receive queue in the window
        uint16_t overflowIds;       // distinct IDs that did not fit the table
        bool overflowIdsSaturated;  // the dedup list for overflowIds filled up (count is a minimum)
        uint8_t idCount;            // rows used in entries[]
        SniffEntry entries[CAN_SNIFF_MAX_IDS];
    };

    // Outcome of a startSniff() request
    enum class SniffStart : uint8_t { STARTED, NOT_INITIALIZED, BUSY_PROBE, BUSY_LWS_ZERO, BUSY };

    // Outcome of a startLwsZero() request
    enum class LwsZeroStart : uint8_t { STARTED, NOT_INITIALIZED, BUSY_PROBE, BUSY };

    // Sender of a queued frame (design D4). Follow-ups add their own tags (e.g. VESC).
    enum class CanTxTag : uint8_t { OBD, LWS_CFG };

    // CAN task health (design D9): session values for the telemetry
    struct DriverStats {
        bool taskRunning;          // the can_io task exists
        const char* intState;      // "ok" / "dead" / "none" (static token)
        uint32_t rxDrop;           // frames dropped from the full receive queue
        uint32_t rxOvr;            // RX0OVR / RX1OVR flags counted
        uint8_t rxHwm;             // receive-queue high-water mark
        uint32_t txFail;           // frames not sent (timeout abort, chip down)
        uint32_t busyMaxUs;        // longest RX / TX / EFLG service pass, µs (commands excluded)
    };

    CANController();
    ~CANController();

    /**
     * @brief Initialize CAN controller
     *
     * @param csPin Chip Select pin for MCP2515
     * @param sckPin SPI Clock pin
     * @param mosiPin SPI MOSI pin
     * @param misoPin SPI MISO pin
     * @return true if initialization successful
     */
    bool begin(gpio_num_t csPin = PIN_CAN_CS,
               gpio_num_t sckPin = PIN_CAN_SCK,
               gpio_num_t mosiPin = PIN_CAN_MOSI,
               gpio_num_t misoPin = PIN_CAN_MISO);

    /**
     * @brief Update CAN controller - call every loop iteration
     * Non-blocking state machine: sends one request or checks one response per call
     */
    void update();

    /**
     * @brief Get current vehicle data
     * @return VehicleData structure with latest values
     */
    VehicleData getVehicleData() const { return vehicleData_; }

    /**
     * @brief Clear the stored engine RPM to 0 (VehicleController calls it when the applied
     * relay ignition state goes into OFF — change add-precrank-dwell-setting, design D4).
     *
     * With the ECU unpowered the engine cannot be running, so 0 is the true value; the next
     * PID 0x0C reply overwrites it. Only engineRPM is touched: dataValid stays as it is
     * (clearing it would turn coolant, MAP, voltage and EFI_STATUS NaN five seconds early)
     * and lastUpdateTime stays (the OBD silence watchdog depends on it). Without this the
     * last RPM before OFF stays "valid" for up to CAN_DATA_STALE_TIMEOUT and suppresses the
     * next auto-crank ("engine already running").
     */
    void clearEngineRpm() { vehicleData_.engineRPM = 0; }

    /**
     * @brief Set RPM PID poll interval at runtime
     * @param ms Poll interval in milliseconds (e.g. CAN_POLL_INTERVAL_RPM_BOOST during gear change)
     */
    void setRPMPollInterval(uint32_t ms);

    /**
     * @brief Start an on-demand ECU capability probe.
     * Suspends normal polling, walks the supported-PID bitmaps and a fixed candidate list,
     * and reads stored DTCs, then resumes normal polling. Non-blocking; results become
     * available via getProbeResults() once complete.
     * @param gearChangeActive true if a gear change is in progress (probe is deferred)
     * @return STARTED if the probe began, BUSY if deferred, NO_ECU if CAN data is invalid
     */
    ProbeStart startProbe(bool gearChangeActive);

    /**
     * @brief Get the latest ECU capability probe snapshot
     */
    ProbeResults getProbeResults() const { return probeResults_; }

    /**
     * @brief Start the CAN bus sniffer: pause OBD-II polling, accept every standard and
     * extended ID for `ms` milliseconds, record a per-ID table, then restore the normal
     * filters (applyRxFilters()) and resume polling. 0x305 frames still reach the LWS sink.
     * Standstill diagnostic: with every ID accepted the two RX buffers can overflow and drop
     * LWS frames; the steering loop then takes its sensor-unhealthy path until frames return.
     * @param ms window length (clamped to 1..CAN_SNIFF_MAX_WINDOW_MS)
     * @param gearChangeActive true if a gear change is in progress (refused as BUSY)
     */
    SniffStart startSniff(uint32_t ms, bool gearChangeActive = false);

    /** @brief True while the sniffer window is running */
    bool sniffActive() const { return mode_ == Mode::SNIFFING; }

    /** @brief Latest sniffer snapshot (reference: no copy of the ~800-byte table) */
    const SniffResults& getSniffResults() const { return sniffResults_; }

    /** @brief True once the MCP2515 is initialised and on the bus */
    bool isInitialized() const { return initialized_; }

    /** @brief Completed (verified) MCP2515 re-initializations of any cause since boot */
    uint32_t getReinitCount() const { return reinitCount_; }

    /**
     * @brief LWS power state, fed every loop by the vehicle layer: true while the ignition is ACC
     * or IGNITION (the Bosch LWS hangs on the ACC line, Relay_1). The LWS silence watchdog runs
     * only while powered and arms on the first 0x305 frame after each off -> on transition.
     * The ECU hangs on the same ACC line, so the OBD-II no-data watchdog is gated on the same flag
     * and restarts its 15 s window at each off -> on transition
     * (change gate-obd-watchdog-and-pull-up-vesc-rx).
     */
    void setLwsPowered(bool on);

    /**
     * @brief Queue one frame for the CAN task (loop only). Returns at once: false when the
     * transmit ring is full (counted). The task loads it into TXB0 (FIFO) and aborts it alone
     * if it has not left after CAN_TX_TIMEOUT_MS. With wantResult a TX_DONE result comes back
     * on a later update() (sent / failed; failed at once while the chip is down).
     */
    bool queueTx(const CanFrame& f, CanTxTag tag, bool wantResult);

    /** @brief CAN task health counters (session values) */
    DriverStats getDriverStats() const;

    // ---- Bosch LWS steering-angle sensor (shared bus) ------------------------

    /**
     * @brief Attach the steering-angle frame sink (owned by the steering controller).
     * Every standard-ID 0x305 frame read on any receive path goes to sink->onFrame().
     */
    void attachLwsSink(LwsSensor* sink) { lwsSink_ = sink; }
    bool hasLwsSink() const { return lwsSink_ != nullptr; }

    /**
     * @brief Start the LWS zero-set sequence: pause OBD-II polling (an in-flight request
     * completes or times out first), send 0x7C0 [05 00], wait STEER_LWS_ZERO_GAP_MS
     * without blocking, send 0x7C0 [53 30], resume polling. The two payloads are fixed;
     * no API takes a 0x7C0 payload.
     * @return STARTED, or why it was refused (nothing is sent then)
     */
    LwsZeroStart startLwsZero();

    /** @brief True while the zero-set sequence is running (OBD-II polling paused) */
    bool isLwsZeroRunning() const;
    /**
     * @brief The reset frame [05 00] of the last sequence MAY have reached the sensor: true from
     * the moment it is queued (the old zero is then no longer guaranteed, so the steering erases
     * its limits), whatever the TX result.
     */
    bool lwsZeroResetSent() const { return lwsZeroResetSent_; }
    /** @brief The zero frame [53 30] of the last sequence was sent (at lwsZeroSentMs()) */
    bool lwsZeroZeroSent() const { return lwsZeroZeroSent_; }
    uint32_t lwsZeroSentMs() const { return lwsZeroSentMs_; }
    /** @brief Abort reason of the last sequence ("tx_failed"), nullptr if none */
    const char* lwsZeroAbortReason() const { return lwsZeroAbort_; }

private:
    // State machine states
    enum class OBDState : uint8_t {
        IDLE,               // Ready to send next PID request
        WAITING_RESPONSE    // Request sent, polling for response
    };

    // Coarse controller mode: normal polling vs. one-shot capability probe
    enum class Mode : uint8_t {
        NORMAL,
        PROBING,
        SNIFFING
    };

    // LWS zero-set sub-state machine (runs next to the OBD-II one, polling paused)
    enum class LwsZeroPhase : uint8_t {
        IDLE,        // not running (last result in the lwsZero* fields)
        WAIT_IDLE,   // let an in-flight OBD-II request complete / time out
        SEND_RESET,  // queue 0x7C0 [05 00]
        WAIT_RESET_TX, // wait for its TX result (at most CAN_TX_RESULT_WAIT_MS)
        GAP,         // wait STEER_LWS_ZERO_GAP_MS from the confirmed send (non-blocking)
        SEND_ZERO,   // queue 0x7C0 [53 30]
        WAIT_ZERO_TX // wait for its TX result, then back to IDLE (OBD-II resumes)
    };

    // Sniffer window phases while mode_ == SNIFFING (the filter switches run in the task)
    enum class SniffPhase : uint8_t { OPENING, WINDOW, CLOSE_PENDING, CLOSING };

    // Commands from the loop to the task (cmdSlot_; 0 = none)
    enum class CanCmd : uint8_t { NONE = 0, INIT, REINIT, FILTERS_SNIFF, FILTERS_NORMAL };

    // Task -> loop events
    enum class CanEvType : uint8_t { CMD_DONE, INIT_OK, TX_DONE, BUS_OFF };
    struct CanEvent {
        CanEvType type;
        uint8_t cmd;         // CMD_DONE: CanCmd;  TX_DONE: CanTxTag
        uint8_t flags;       // EV_* bits
        uint8_t seq;         // TX_DONE: sequence number of the frame
        uint8_t canstat;
        uint8_t eflg;
        uint8_t tec;
        uint8_t rec;
        uint32_t attempt;    // INIT / INIT_OK: init attempt number
        uint32_t afterMs;    // INIT_OK: ms since the first init attempt
    };
    static constexpr uint8_t EV_OK = 0x01;         // command / frame succeeded
    static constexpr uint8_t EV_BEGIN = 0x02;      // reset gate + begin() passed
    static constexpr uint8_t EV_FILT = 0x04;       // filter programming passed (FILTERS_SNIFF failure: restore passed)
    static constexpr uint8_t EV_MODE = 0x08;       // setMode(NORMAL) + CANSTAT / CANINTE readback passed

    // One queued frame
    struct CanTxItem {
        CanFrame frame;
        uint8_t tag;         // CanTxTag
        uint8_t seq;
        bool wantResult;
    };

    // Result of one task-side chip init attempt (design D6)
    struct InitResult {
        bool begin;
        bool filt;
        bool mode;
        uint8_t canstat;
        uint8_t eflg;
        uint8_t tec;
        uint8_t rec;
    };

    // Probe sequencer phases
    enum class ProbePhase : uint8_t {
        BITMAP,     // walking supported-PID bitmap groups
        CANDIDATE,  // one test request per candidate PID
        DTC,        // single Mode 03 stored-DTC read
        DONE
    };

    // PID scheduling entry
    struct PIDEntry {
        uint8_t pid;
        uint32_t interval;       // ms between polls
        uint32_t nextPollTime;   // millis() when next poll is due
        uint8_t retryCount;
    };

    static constexpr uint8_t PID_COUNT = 6;

    // OBD-II PIDs (Mode 01 - Current Data)
    static constexpr uint8_t PID_ENGINE_RPM = 0x0C;
    // static constexpr uint8_t PID_VEHICLE_SPEED = 0x0D;
    static constexpr uint8_t PID_COOLANT_TEMP = 0x05;
    // static constexpr uint8_t PID_OIL_TEMP = 0x5C;
    static constexpr uint8_t PID_THROTTLE_POS = 0x11;
    static constexpr uint8_t PID_MAP = 0x0B;
    // PID 0x42 (control module voltage) is NOT polled: the Delphi MT05 ECU never answers it.
    // The 12 V battery is measured on X10/GPIO9 instead (VehicleController battery sampler);
    // 0x42 stays in the on-demand probe list.
    static constexpr uint8_t PID_INTAKE_TEMP = 0x0F;
    static constexpr uint8_t PID_ENGINE_LOAD = 0x04;
    // static constexpr uint8_t PID_FUEL_LEVEL = 0x2F;

    MCP_CAN* mcp_can_;              // MCP2515 CAN controller object — used by the CAN task ONLY
    VehicleData vehicleData_;       // Current vehicle data
    bool initialized_;              // Initialization status (loop-side: first init succeeded)
    gpio_num_t csPin_;              // MCP2515 chip-select (raw register access, see readRegister())

    // ---- CAN task and its channels (move-can-rx-to-interrupt) ---------------------
    TaskHandle_t task_;
    CanFrameRing<CAN_RX_RING> rxRing_;                  // task -> loop
    CanRing<CanTxItem, CAN_TX_RING> txRing_;            // loop -> task
    CanRing<CanEvent, 8> evRing_;                       // task -> loop
    portMUX_TYPE rxMux_;
    portMUX_TYPE txMux_;
    portMUX_TYPE evMux_;
    volatile uint8_t cmdSlot_;      // CanCmd posted by the loop, cleared by the task when taken
    CanCmd cmdInFlight_;            // loop-only: command posted, result not yet received
    uint8_t txSeq_;                 // loop-only: sequence number of the last queued frame
    bool hasIntPin_;                // an INT interrupt is attached
    volatile uint32_t isrCount_;    // INT edges (ISR only)

    // Task-only state
    bool chipUp_;                   // last init / re-init succeeded
    bool initAttempted_;            // an init ran: while the chip is down, retry every CAN_INIT_RETRY_MS
    uint32_t initAttempts_;         // attempts since the chip went down (boot INIT = 1)
    uint32_t firstInitMs_;          // millis() when the chip went down (boot: first INIT)
    uint32_t lastInitTryMs_;
    uint32_t lastEflgMs_;
    uint32_t lastIsrSeen_;
    bool busOffLatched_;            // TXBO reported for this episode
    bool txBusy_;                   // TXB0 holds our frame (or an abort is completing)
    bool txAborting_;               // TXREQ cleared by us, waiting for it to read clear
    bool txReported_;               // the frame in TXB0 has had its result counted / reported
    uint32_t txLoadMs_;
    uint32_t txAbortMs_;
    CanTxItem txCur_;
    uint32_t statsResetAck_;

    // Task-written counters (single writer, aligned 32-bit, read by the loop) — design D9
    volatile uint32_t cRx_;         // frames read
    volatile uint32_t cRxDrop_;     // receive-ring drops (oldest discarded)
    volatile uint32_t cRxHwm_;      // receive-ring high-water mark (session)
    volatile uint32_t cWinHwm_;     // receive-ring high-water mark since the last stats line
    volatile uint32_t cOvr_;        // RX0OVR + RX1OVR flags seen
    volatile uint32_t cOvrEflg_;    // EFLG of the last overflow
    volatile uint32_t cIntWakes_;   // cycles woken by the INT line
    volatile uint32_t cPollFrames_; // frames found on a wake that was not an INT
    volatile uint32_t cCapHits_;    // drain cap reached with frames still pending
    volatile uint32_t cTx_;         // frames sent
    volatile uint32_t cTxFail_;     // frames aborted on timeout / failed while the chip is down
    volatile uint32_t cBusyMaxUs_;  // longest RX / TX / EFLG service pass, commands excluded (session)
    volatile uint32_t cWinBusyMaxUs_; // same, since the last stats line
    volatile uint32_t cCmdMaxUs_;   // longest init / re-init / filter command or init retry (session)
    volatile uint32_t cBusyUs_;     // sum of cycle times
    volatile uint32_t cInitFails_;  // failed boot init retries
    volatile uint32_t cEvDrops_;    // event ring full (never expected)
    volatile uint32_t cHealth_;     // TEC | REC << 8 | EFLG << 16 | valid << 24 (one write)

    // Results that must never be lost: single-entry slots, guarded by evMux_ (finding: an event
    // ring full of bus-off / TX events must not swallow a command result)
    CanSlot<CanEvent> cmdResult_;   // the result of the one command in flight
    CanSlot<CanEvent> recovered_;   // the task's own init retry succeeded (INIT_OK)

    // Loop-written
    uint32_t txQueueFull_;          // queueTx() refused (ring full)
    volatile uint8_t txCancelSeq_;  // a requester gave up on this frame: the task must not load it
    volatile uint32_t statsResetReq_; // loop asks the task to restart the window maxima

    // Loop-side diagnostics bookkeeping
    uint32_t lastStatsMs_;
    uint32_t stRx_, stDrop_, stOvr_, stInt_, stPoll_, stTx_, stTxf_, stMiss_;
    uint32_t lastIntWinMs_;
    uint32_t iwInt_, iwRx_;
    CanIoLaw::IntState intState_;
    uint32_t ovrSeen_;              // cOvr_ already reported by checkRxOverflow()

    // Replies latched by the frame router (design D3); sending a request clears the latch
    bool obdReplyLatched_;
    CanFrame obdReply_;
    bool probeReplyLatched_;
    CanFrame probeReply_;

    // ---- Diagnostics / recovery state ---------------------------------------
    uint32_t lastRxLogTime_;          // millis() of the last unmatched-frame log (rate limit)
    uint32_t lastHealthLogTime_;      // millis() of the last TEC/REC/EFLG health log (rate limit)
    uint32_t lastOverflowLogTime_;    // millis() of the last RXnOVR log (rate limit)
    uint32_t rxOverflowSuppressed_;   // RXnOVR clears since the last overflow log line
    uint32_t lastReinitTime_;         // millis() of the last forced re-init (also the boot reference before any response)

    // ---- Re-init verification + LWS silence watchdog (add-event-journal-and-link-recovery) ----
    bool reinitPending_;              // a failed re-init attempt waits for its retry
    const char* reinitReason_;        // trigger token of the running re-init sequence
    uint8_t reinitAttempt_;           // attempts made in the running sequence
    uint32_t reinitAttemptMs_;        // millis() of the last attempt
    uint32_t reinitCount_;            // verified re-inits since boot (telemetry can_reinit)
    bool lwsPowered_;                 // ignition ACC / IGNITION (setLwsPowered()); gates both CAN watchdogs
    uint32_t ignitionOnMs_;           // millis() of the last ignition off -> on edge (OBD-II window start)
    bool lwsSeenSinceOn_;             // a 0x305 frame arrived since the ignition was switched on (armed)
    uint32_t lastLwsFrameMs_;         // millis() of the last dispatched 0x305 frame
    bool lwsSilentObdDeadLogged_;     // "lws_silent obd_alive=0" journaled for this silence episode
    bool firstFrameWait_;             // measuring the time to the first frames after a re-init
    uint32_t reinitDoneMs_;
    uint32_t firstLwsMs_;             // ms after the re-init (0 = not yet; stored as +1)
    uint32_t firstObdMs_;
    // Journal economy for watchdog repeats (the re-init cadence itself is unchanged): within one
    // silence episode (no frame of that kind since the episode's first re-init) the trigger line
    // is journaled at the start and then every CAN_WD_REPEAT_LOG_MS with a repeat count; the
    // successful re-inits in between are counted only. Failures are always journaled.
    bool reinitQuiet_;                // the running sequence is such a repeat
    bool lwsEpisode_;
    bool obdEpisode_;
    uint32_t lwsEpLogMs_;
    uint32_t obdEpLogMs_;
    uint32_t lwsEpRepeats_;
    uint32_t obdEpRepeats_;

    // State machine
    OBDState state_;
    uint8_t activePIDIndex_;        // Index into pidTable_ of current request
    uint32_t requestSentTime_;      // millis() when current request was sent

    // PID scheduling table
    PIDEntry pidTable_[PID_COUNT];

    // Response buffer
    uint8_t responseData_[5];
    uint8_t responseLen_;

    // ---- Probe state ---------------------------------------------------------
    Mode mode_;                     // NORMAL polling vs. PROBING
    ProbeResults probeResults_;     // latest probe snapshot
    ProbePhase probePhase_;         // current sequencer phase
    uint8_t probeBitmapGroup_;      // index into bitmap groups (0..3)
    uint8_t probeMaxBitmapGroups_;  // number of bitmap groups to query (grows on continuation bit)
    uint8_t probeCandidateIdx_;     // index into candidate PID list
    bool probeRequestInFlight_;     // a probe request is awaiting a response
    uint32_t probeRequestSentTime_; // millis() when current probe request was sent
    uint8_t probeRetryCount_;       // retries used on the current probe request

    // ---- Sniffer state ------------------------------------------------------------
    SniffResults sniffResults_;
    uint32_t sniffExtraIds_[CAN_SNIFF_MAX_IDS];   // dedup list for IDs beyond the table
    uint8_t sniffExtraCount_;
    SniffPhase sniffPhase_;
    uint32_t sniffOvrBase_;         // cOvr_ at the window start
    uint32_t sniffDropBase_;        // cRxDrop_ at the window start

    // ---- LWS steering sensor state --------------------------------------------
    LwsSensor* lwsSink_;            // steering-angle frame sink (nullptr = none attached)
    LwsZeroPhase lwsZeroPhase_;
    uint32_t lwsZeroPhaseMs_;       // millis() when the current phase started
    bool lwsZeroResetSent_;
    bool lwsZeroZeroSent_;
    uint32_t lwsZeroSentMs_;        // millis() when [53 30] was sent
    const char* lwsZeroAbort_;      // "tx_failed" or nullptr
    uint8_t lwsZeroTxSeq_;          // sequence number of the frame whose result is awaited
    uint8_t lwsZeroTxResult_;       // 0 = pending, 1 = sent, 2 = not sent (aborted / failed)

    bool sendOBDRequest(uint8_t pid);
    bool sendMode03Request();
    bool tryReceiveResponse();
    // Generalized probe receive: matches a Mode 01 (0x41+pid) or Mode 03 (0x43) response.
    bool tryReceiveProbeResponse(uint8_t expectedMode, uint8_t expectedPid,
                                 uint8_t* outBuf, uint8_t& outLen, uint16_t& outCanId);
    int8_t selectNextPID();
    void parseAndStore(uint8_t index, const uint8_t* data, uint8_t len);
    bool isDataStale() const;

    // ---- CAN task (the ONLY code that touches the MCP2515) ---------------------
    static void canIoTask(void* arg);
    static void canIntIsr(void* arg);   // IRAM; counts + notifies only
    void taskLoop();
    void taskRunCommand(uint8_t cmd);
    void taskInitRetry();
    void taskAbortTxb0();
    InitResult taskInitAttempt();
    uint16_t taskDrain(uint8_t& st);
    void taskServiceTx(uint8_t st);
    void taskFinishTx(bool ok);
    void taskFailQueuedTx();
    void taskServiceErrors();
    void taskPostEvent(const CanEvent& ev);
    // Raw MCP2515 access over the library's SPI bus and settings (10 MHz, MODE0, same CS).
    // The library keeps mcp2515_readRegister()/modifyRegister() private and has no getMode(),
    // READ STATUS, raw TX load or single-buffer abort, so those are done here — task only.
    uint8_t readRegister(uint8_t address) const;
    void readRegisters(uint8_t address, uint8_t* out, uint8_t n) const;
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t data) const;
    uint8_t readStatus() const;
    void spiCommand(uint8_t instruction) const;
    void loadTxb0(const CanFrame& f);

    // ---- Loop side of the channels ---------------------------------------------
    bool postCommand(CanCmd cmd);
    void serviceTaskEvents();
    void drainRxQueue();
    void routeFrame(const CanFrame& f);
    void onInitResult(const CanEvent& ev);
    void onReinitResult(const CanEvent& ev);
    void becomeInitialized();
    void printFiltersLine(bool ok) const;
    void serviceStats(uint32_t now);
    void serviceIntState(uint32_t now);

    // Re-initialize the MCP2515 (bus-off, no-data watchdog, LWS silence watchdog, sniffer filter
    // restore). The loop decides and journals; each attempt is posted to the CAN task, which
    // checks the reset gate, begin(), applyRxFilters() and setMode(MCP_NORMAL). A failed attempt
    // is retried on later update() calls (CAN_REINIT_RETRY_MS apart), at most
    // CAN_REINIT_MAX_ATTEMPTS per trigger. Every outcome is journaled when the result arrives.
    // Returns true when an attempt was posted (or one is already in flight).
    bool recoverFromBusOff(const char* reason = "bus_off", bool quiet = false);
    bool attemptReinit();
    bool reinitInFlight() const { return cmdInFlight_ == CanCmd::REINIT; }
    // Journal `CAN E bus_off tec rec eflg`, then recoverFromBusOff("bus_off")
    void handleBusOff(uint8_t tec, uint8_t rec, uint8_t eflg);
    void serviceLwsWatchdog(uint32_t now);
    void serviceFirstFrame(uint32_t now);

    // Rate-limited (~1/s) dump of a received frame that did not match the expected response
    void logUnmatchedFrame(unsigned long canId, uint8_t len, const uint8_t* buf);

    // Rate-limited (~5 s, only while dataValid == false) TEC/REC/EFLG health line, from the
    // task's snapshot
    void logHealth();

    // Report new RX0OVR/RX1OVR counts from the task, at most once per CAN_OVERFLOW_LOG_INTERVAL
    void checkRxOverflow();

    // Program the MCP2515 masks/filters: RXB0 = OBD-II replies 0x7E8-0x7EF, RXB1 = LWS
    // 0x305 only. Must run after begin() and before setMode(MCP_NORMAL) — init_Mask()/
    // init_Filt() drop the chip into CONFIG mode and restore the library's cached mode.
    // Task only; stops at the first failing call; never logs.
    bool applyRxFilters();

    // Sniffer: accept every standard and extended ID in both buffers (masks 0, one std and
    // one ext filter per buffer). Same CONFIG-mode round trip as applyRxFilters(). Task only.
    bool applySniffFilters();
    void updateSniff();
    void recordSniffFrame(unsigned long canId, uint8_t len, const uint8_t* buf);
    void closeSniffWindow();
    void onSniffOpened(const CanEvent& ev);
    void finishSniff(bool restoreOk);

    // Called FIRST for every frame taken from the receive queue: a standard-ID 0x305 frame
    // goes to the LWS sink (if attached) and returns true — it is then neither an
    // OBD-II candidate nor logged as unmatched.
    bool dispatchSteeringFrame(unsigned long canId, uint8_t len, const uint8_t* buf);

    // Zero-set sequencer step (called from update() while IDLE in NORMAL mode)
    void serviceLwsZero();
    // Queue one of the two fixed 0x7C0 payloads: zeroFrame=false -> [05 00], true -> [53 30]
    // (with a TX result request; lwsZeroTxSeq_ / lwsZeroTxResult_ track it)
    bool sendLwsCfg(bool zeroFrame);
    void abortLwsZero();

    // Probe sequencer helpers
    void updateProbe();
    bool probeCurrentRequest(uint8_t& mode, uint8_t& pid) const;
    void probeAdvanceCursor();
    void recordProbeResponse(uint8_t mode, uint8_t pid, uint16_t canId, const uint8_t* buf, uint8_t len);
    void recordProbeTimeout(uint8_t mode, uint8_t pid);
    void finishProbe();
};

#endif // CAN_CONTROLLER_H
