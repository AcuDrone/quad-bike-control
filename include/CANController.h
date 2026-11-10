#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <Arduino.h>
#include <mcp_can.h>
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
 * own receive buffer (RXB1, see applyRxFilters()), every receive path hands 0x305 frames
 * to the attached LwsSensor in any CAN state, and the LWS zero-set sequence
 * (startLwsZero()) is the only code that transmits on 0x7C0.
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
        uint32_t frames;            // all frames read during the window
        uint32_t overflows;         // RX0OVR/RX1OVR flags seen (each = at least one frame lost)
        uint16_t overflowIds;       // distinct IDs that did not fit the table
        bool overflowIdsSaturated;  // the dedup list for overflowIds filled up (count is a minimum)
        uint8_t idCount;            // rows used in entries[]
        SniffEntry entries[CAN_SNIFF_MAX_IDS];
    };

    // Outcome of a startSniff() request
    enum class SniffStart : uint8_t { STARTED, NOT_INITIALIZED, BUSY_PROBE, BUSY_LWS_ZERO, BUSY };

    // Outcome of a startLwsZero() request
    enum class LwsZeroStart : uint8_t { STARTED, NOT_INITIALIZED, BUSY_PROBE, BUSY };

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
     */
    void setLwsPowered(bool on);

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
    /** @brief The reset frame [05 00] of the last sequence was sent */
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
        SEND_RESET,  // send 0x7C0 [05 00]
        GAP,         // wait STEER_LWS_ZERO_GAP_MS (non-blocking)
        SEND_ZERO    // send 0x7C0 [53 30], then back to IDLE (OBD-II resumes)
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

    MCP_CAN* mcp_can_;              // MCP2515 CAN controller object
    VehicleData vehicleData_;       // Current vehicle data
    bool initialized_;              // Initialization status
    gpio_num_t csPin_;              // MCP2515 chip-select (raw register access, see readRegister())

    // ---- Diagnostics / recovery state ---------------------------------------
    uint8_t consecutiveSendFailures_; // sendMsgBuf() failures since the last successful send
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
    bool lwsPowered_;                 // ignition ACC / IGNITION (setLwsPowered())
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

    // ---- LWS steering sensor state --------------------------------------------
    LwsSensor* lwsSink_;            // steering-angle frame sink (nullptr = none attached)
    LwsZeroPhase lwsZeroPhase_;
    uint32_t lwsZeroPhaseMs_;       // millis() when the current phase started
    bool lwsZeroResetSent_;
    bool lwsZeroZeroSent_;
    uint32_t lwsZeroSentMs_;        // millis() when [53 30] was sent
    const char* lwsZeroAbort_;      // "tx_failed" or nullptr

    bool sendOBDRequest(uint8_t pid);
    bool sendMode03Request();
    bool tryReceiveResponse();
    // Generalized probe receive: matches a Mode 01 (0x41+pid) or Mode 03 (0x43) response.
    bool tryReceiveProbeResponse(uint8_t expectedMode, uint8_t expectedPid,
                                 uint8_t* outBuf, uint8_t& outLen, uint16_t& outCanId);
    int8_t selectNextPID();
    void parseAndStore(uint8_t index, const uint8_t* data, uint8_t len);
    bool isDataStale() const;

    // ---- Diagnostics / recovery helpers --------------------------------------
    // Raw MCP2515 register access. The vendored coryjfowler library keeps
    // mcp2515_readRegister()/mcp2515_modifyRegister() private and exposes no
    // getMode()/clearRXnOVR(), so mode readback and EFLG overflow clearing are
    // done here over the same SPI bus/settings the library uses (10 MHz, MODE0).
    uint8_t readRegister(uint8_t address) const;
    void modifyRegister(uint8_t address, uint8_t mask, uint8_t data) const;

    // Re-initialize the MCP2515 (bus-off, send failure, no-data watchdog, LWS silence watchdog,
    // sniffer filter restore). Checks begin(), applyRxFilters() and setMode(MCP_NORMAL); a failed
    // attempt is retried on later update() calls (CAN_REINIT_RETRY_MS apart, no in-call loop),
    // at most CAN_REINIT_MAX_ATTEMPTS per trigger. Every outcome is journaled.
    // Non-blocking apart from the library's own SPI waits. Returns this attempt's result.
    bool recoverFromBusOff(const char* reason = "bus_off", bool quiet = false);
    bool attemptReinit();
    // Journal `CAN E bus_off tec rec eflg`, then recoverFromBusOff("bus_off")
    void handleBusOff(uint8_t eflg);
    void serviceLwsWatchdog(uint32_t now);
    void serviceFirstFrame(uint32_t now);

    // Rate-limited (~1/s) dump of a received frame that did not match the expected response
    void logUnmatchedFrame(unsigned long canId, uint8_t len, const uint8_t* buf);

    // Rate-limited (~5 s, only while dataValid == false) TEC/REC/EFLG health line
    void logHealth();

    // Clear MCP_EFLG_RX0OVR/RX1OVR if set, logging at most once per CAN_OVERFLOW_LOG_INTERVAL
    void checkRxOverflow();

    // Program the MCP2515 masks/filters: RXB0 = OBD-II replies 0x7E8-0x7EF, RXB1 = LWS
    // 0x305 only. Must run after begin() and before setMode(MCP_NORMAL) — init_Mask()/
    // init_Filt() drop the chip into CONFIG mode and restore the library's cached mode.
    bool applyRxFilters();

    // Sniffer: accept every standard and extended ID in both buffers (masks 0, one std and
    // one ext filter per buffer). Same CONFIG-mode round trip as applyRxFilters().
    bool applySniffFilters();
    void updateSniff();
    void recordSniffFrame(unsigned long canId, uint8_t len, const uint8_t* buf);
    void finishSniff();

    // While IDLE: pull any queued frames out of the MCP2515 RX buffers so they
    // never sit full between polls; unmatched frames go through logUnmatchedFrame()
    void drainRxBuffers();

    // Called FIRST for every frame read on any receive path: a standard-ID 0x305 frame
    // goes to the LWS sink (if attached) and returns true — it is then neither an
    // OBD-II candidate nor logged as unmatched.
    bool dispatchSteeringFrame(unsigned long canId, uint8_t len, const uint8_t* buf);

    // Zero-set sequencer step (called from update() while IDLE in NORMAL mode)
    void serviceLwsZero();
    // Send one of the two fixed 0x7C0 payloads: zeroFrame=false -> [05 00], true -> [53 30]
    bool sendLwsCfg(bool zeroFrame);

    // Probe sequencer helpers
    void updateProbe();
    bool probeCurrentRequest(uint8_t& mode, uint8_t& pid) const;
    void probeAdvanceCursor();
    void recordProbeResponse(uint8_t mode, uint8_t pid, uint16_t canId, const uint8_t* buf, uint8_t len);
    void recordProbeTimeout(uint8_t mode, uint8_t pid);
    void finishProbe();
};

#endif // CAN_CONTROLLER_H
