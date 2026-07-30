#ifndef CAN_CONTROLLER_H
#define CAN_CONTROLLER_H

#include <Arduino.h>
#include <mcp_can.h>
#include "Constants.h"

/**
 * @brief CAN bus controller for reading vehicle OBD-II data via MCP2515
 *
 * Interfaces with MCP2515 SPI CAN controller to read standard OBD-II diagnostic data
 * from vehicle ECU (engine RPM, speed, coolant temperature, etc.).
 * Uses a non-blocking state machine to avoid stalling the main loop.
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
        uint32_t lastUpdateTime;    // millis() timestamp of last successful update
        bool dataValid;             // true if CAN communication is healthy
    };

    // ---- ECU capability probe (on-demand diagnostic sweep) ------------------
    static constexpr uint8_t PROBE_CANDIDATE_COUNT = 9;  // candidate PIDs test-requested
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

private:
    // State machine states
    enum class OBDState : uint8_t {
        IDLE,               // Ready to send next PID request
        WAITING_RESPONSE    // Request sent, polling for response
    };

    // Coarse controller mode: normal polling vs. one-shot capability probe
    enum class Mode : uint8_t {
        NORMAL,
        PROBING
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

    static constexpr uint8_t PID_COUNT = 4;

    // OBD-II PIDs (Mode 01 - Current Data)
    static constexpr uint8_t PID_ENGINE_RPM = 0x0C;
    // static constexpr uint8_t PID_VEHICLE_SPEED = 0x0D;
    static constexpr uint8_t PID_COOLANT_TEMP = 0x05;
    // static constexpr uint8_t PID_OIL_TEMP = 0x5C;
    static constexpr uint8_t PID_THROTTLE_POS = 0x11;
    static constexpr uint8_t PID_MAP = 0x0B;
    // static constexpr uint8_t PID_FUEL_LEVEL = 0x2F;

    MCP_CAN* mcp_can_;              // MCP2515 CAN controller object
    VehicleData vehicleData_;       // Current vehicle data
    bool initialized_;              // Initialization status

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

    bool sendOBDRequest(uint8_t pid);
    bool sendMode03Request();
    bool tryReceiveResponse();
    // Generalized probe receive: matches a Mode 01 (0x41+pid) or Mode 03 (0x43) response.
    bool tryReceiveProbeResponse(uint8_t expectedMode, uint8_t expectedPid,
                                 uint8_t* outBuf, uint8_t& outLen);
    int8_t selectNextPID();
    void parseAndStore(uint8_t index, const uint8_t* data, uint8_t len);
    bool isDataStale() const;

    // Probe sequencer helpers
    void updateProbe();
    bool probeCurrentRequest(uint8_t& mode, uint8_t& pid) const;
    void probeAdvanceCursor();
    void recordProbeResponse(uint8_t mode, uint8_t pid, const uint8_t* buf, uint8_t len);
    void recordProbeTimeout(uint8_t mode, uint8_t pid);
    void finishProbe();
};

#endif // CAN_CONTROLLER_H
