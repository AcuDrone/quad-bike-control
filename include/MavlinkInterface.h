#ifndef MAVLINKINTERFACE_H
#define MAVLINKINTERFACE_H

#include <Arduino.h>
#include "Constants.h"
#include "TransmissionController.h"

/**
 * @brief MAVLink 2 vehicle interface to a Pixhawk autopilot (TELEM2).
 *
 * Replaces the former S-bus transport. Decodes per-channel actuator commands
 * from the MAVLink SERVO_OUTPUT_RAW message (servo PWM microseconds) and exposes
 * the same typed command API the vehicle layer consumes
 * (steering/throttle/gear/brake/ignition/light). Also reports vehicle state back
 * to the MAVLink network: HEARTBEAT plus a single EFI_STATUS from this component,
 * a VFR_HUD carrying hall-sensor ground speed, the steering VESC's telemetry as
 * exactly five NAMED_VALUE_FLOAT values (STEER_POS, STEER_A, VESC_V at 5 Hz;
 * VESC_TEMP, VESC_OK at 1 Hz), plus STATUSTEXT for state transitions.
 *
 * Those five names are the project's ONE scoped exception to the "everything in the
 * field that names it, in a single EFI_STATUS" rule: the two standard ESC telemetry
 * messages (ids 290 and 291) are absent from the dialect Mission Planner decodes
 * with, so that pair never reached a consumer at all. They carry the MEASURED
 * steering position in percent of the calibrated lock-to-lock range, the VESC's
 * MOTOR current, its input voltage (the 24 V boost rail, distinct from
 * EFI_STATUS.ignition_voltage on the 12 V side) and its FET temperature, each
 * NaN while its own source is unhealthy, plus
 * VESC_OK (1.0/0.0, never NaN) as the link flag. All five keep being sent while the
 * VESC is silent, so "VESC down" stays distinguishable from "peripheral down".
 * See the .cpp.
 *
 * EFI_STATUS carries every value in the field that NAMES it (RPM, coolant, intake
 * air temp, manifold pressure, ECU load, measured TPS, commanded throttle, module
 * voltage), with documented exceptions in the permanently-free fields: the fuel
 * pair carries the two GEAR values — fuel_consumed = ASSUMED (commanded) gear,
 * fuel_flow = PHYSICAL (opto-sensed) gear, NaN while unknown — pt_compensation
 * carries the digital-output bitmask, and barometric_pressure / fuel_pressure carry
 * the total ODOMETER / TRIP distance in km. See the field-mapping block in the .cpp.
 *
 * Accepts exactly one inbound command: a TRIP RESET as COMMAND_LONG /
 * MAV_CMD_USER_1 with a magic param1, answered with a COMMAND_ACK. Addressing is
 * strict (this component shares the autopilot's system id), and the reset itself is
 * performed by the vehicle layer, which owns the counters.
 *
 * Also feeds the hall wheel speed to the autopilot's EKF3 as body-frame wheel
 * odometry (VISION_POSITION_DELTA). That message carries a BODY-FRAME distance
 * increment, so no heading is needed and nothing inbound is subscribed — EKF3
 * rotates it with its own attitude. The interface stays silent whenever the speed
 * validity or the direction sign is in doubt (a wrong measurement is worse than no
 * measurement).
 *
 * Subscribes READ-ONLY to a single autopilot parameter, `SPEED_MAX` (m/s) — the ONLY
 * source of the vehicle layer's max-speed throttle limiter ceiling. Polled with
 * PARAM_REQUEST_READ and also accepted unsolicited; never written back (no PARAM_SET),
 * and held in RAM only.
 *
 * The MAVLink C library headers are included only in the .cpp to keep this
 * header lightweight.
 */
class MavlinkInterface {
public:
    /**
     * @brief Ignition state enum for relay control (same semantics as before).
     */
    enum class IgnitionState {
        OFF,        // All power off
        ACC,        // Accessory power only
        IGNITION    // Full ignition (triggers auto-cranking)
    };

    /**
     * @brief MAVLink link quality / liveness metrics.
     */
    struct LinkQuality {
        float    commandRate;     // Command frame rate (Hz, rolling)
        uint32_t signalAge;       // ms since last command frame
        uint32_t heartbeatAge;    // ms since last autopilot HEARTBEAT
    };

    /**
     * @brief Plain snapshot of vehicle state to report back over MAVLink.
     * Kept dependency-free so MavlinkInterface need not include CAN/relay headers.
     */
    struct StateReport {
        bool        canValid;       // CAN data fresh/valid
        uint16_t    engineRpm;      // RPM
        int8_t      coolantTemp;    // °C
        const char* gearFrom;       // gear the current step is leaving "R"/"N"/"L"/"H"
        const char* gearTo;         // current step / assumed gear "R"/"N"/"L"/"H"
        bool        gearMoving;     // true while the servo is actively moving (a phase is active)
        const char* ignition;       // "OFF"/"ACC"/"IGNITION"/"CRANKING"
        bool        failsafe;       // true when in fail-safe
        uint8_t     digitalFlags;   // digital output bitmask (EFI_DIGITAL_FLAG_* in Constants.h)
        bool        speedValid;     // hall speed sensor health (independent of canValid)
        float       speedMs;        // hall-sensor ground speed, m/s (reported as VFR_HUD groundspeed)
        int8_t      intakeTemp;     // intake air temperature °C (ECU PID 0x0F)
        uint16_t    moduleVoltageMv; // control module supply voltage, mV (ECU PID 0x42)
        uint8_t     throttlePosition; // MEASURED throttle position, % (ECU PID 0x11)
        uint8_t     throttleCmdPct;   // COMMANDED (arbitrated) throttle, % 0-100 — servo output
        const char* gearPhysical;     // PHYSICALLY sensed gear "R"/"N"/"L"/"H" (opto switches),
                                      // "?" = UNKNOWN (mid-shift / ambiguous / expander fault)
        uint8_t     mapKpa;           // manifold absolute pressure, kPa (ECU PID 0x0B)
        uint8_t     engineLoad;       // ECU calculated engine load, % (ECU PID 0x04)
        int8_t      travelDirection;  // +1 forward gear, -1 reverse, 0 neutral/unknown — signs the
                                      // (unsigned) wheel speed for VISION_POSITION_DELTA. The sign
                                      // policy lives in the vehicle layer; this stays a transport.
        float       odoKm;            // TOTAL odometer, km — ALWAYS valid, never NaN: distance
                                      // already driven depends on neither CAN health nor the
                                      // current speed reading's validity
        float       tripKm;           // resettable TRIP distance, km — always valid, never NaN
        float       engineHours;      // total ENGINE HOUR METER, h — ALWAYS valid, never NaN:
                                      // accumulated running time is history. While CAN is invalid
                                      // it STOPS GROWING rather than going unknown.
        float       engineTripHours;  // resettable TRIP engine hours, h — always valid, never NaN;
                                      // 0 is a GENUINE zero after a trip reset, not "unknown"

        // --- Steering VESC driver telemetry (STEER_A / VESC_V / VESC_TEMP / VESC_OK) ----
        // Gated by the STEERING DRIVER's own link health (steerDriverOk), which is entirely
        // independent of canValid: a silent VESC does not make CAN data stale, and vice versa.
        bool        steerDriverOk;        // a valid VESC reply arrived within STEER_VESC_COMM_TIMEOUT_MS
        float       steerMotorCurrentA;   // VESC average MOTOR current, A (load/stall diagnostic —
                                          // NOT the input current drawn from the rail)
        float       steerFetTempC;        // VESC power-stage (FET) temperature, °C
        float       steerInputVoltageV;   // VESC-measured input voltage, V — the 24 V BOOST RAIL at
                                          // the load. NOT the ECU module voltage in
                                          // EFI_STATUS.ignition_voltage (12 V side, PID 0x42)

        // --- Steering POSITION (AS5600) -------------------------------------------------
        // A DIFFERENT validity gate from the driver fields above: this is the shaft sensor's,
        // not the VESC's. A dead VESC does not blind the AS5600, and an uncalibrated steering
        // does not make the VESC's electrical readings stale.
        float       steerPercent;         // MEASURED steering position: -100 (left limit) .. 0
                                          // (centre) .. +100 (right limit), percent of the
                                          // CALIBRATED lock-to-lock range — NOT degrees
        bool        steerSensorOk;        // AS5600 reading healthy
        bool        steerCalibrated;      // centre/limits stored — steerPercent is meaningful
                                          // only while BOTH flags are true
        // Note: oil temperature is not available from the ECU and is not reported.
        // Ground speed is NOT carried in EFI_STATUS — it comes from the hall speed
        // sensor and is reported separately via VFR_HUD.
    };

    MavlinkInterface();

    /**
     * @brief Initialize the MAVLink UART link.
     * @return true if initialization succeeds
     */
    bool begin(uint8_t rxPin = PIN_MAVLINK_RX, uint8_t txPin = PIN_MAVLINK_TX,
               uint8_t uartNum = MAVLINK_UART_NUM, uint32_t baud = MAVLINK_BAUD_RATE);

    /**
     * @brief Parse inbound bytes, refresh link timers, request the command stream.
     * Call every loop iteration. Non-blocking.
     */
    void update();

    /**
     * @brief Send scheduled outbound telemetry (rate-limited internally).
     * @param state Current vehicle state snapshot
     */
    void report(const StateReport& state);

    // ---- Raw channel access -------------------------------------------------

    /** @brief Channel value in microseconds (1-16), 0 if invalid. */
    uint16_t getChannel(uint8_t channel) const;

    /** @brief Copy all 16 decoded channel values (µs) into a size-16 array. */
    void getRawChannels(uint16_t* channels) const;

    // ---- Mapped vehicle commands -------------------------------------------

    float getSteering() const;                       // -100..+100 (0 = center)
    float getThrottle() const;                        // 0..100 (0 = idle)
    TransmissionController::Gear getGear() const;     // REVERSE/NEUTRAL/LOW
    float getBrake() const;                           // 0..100 (0 = released)
    IgnitionState getIgnitionState() const;           // OFF/ACC/IGNITION
    bool getFrontLight() const;                       // true = ON
    bool getWheelLock() const;                        // true = front wheels LOCKED

    // ---- Link monitoring ----------------------------------------------------

    /** @brief True if a command frame arrived within MAVLINK_CMD_TIMEOUT_MS. */
    bool isSignalValid() const;

    /** @brief ms since the last command frame. */
    uint32_t getSignalAge() const;

    /** @brief True if an autopilot heartbeat arrived within the heartbeat timeout. */
    bool isLinkUp() const;

    /** @brief Aggregate link metrics. */
    LinkQuality getLinkQuality() const;

    // ---- External navigation odometry ---------------------------------------

    /** @brief Count of VISION_POSITION_DELTA messages actually sent (gates passed). */
    uint32_t getOdomTxCount() const { return odomTxCount_; }

    // ---- Autopilot parameter subscription (SPEED_MAX) -----------------------
    //
    // READ-ONLY: this interface polls PARAM_REQUEST_READ and accepts PARAM_VALUE, and never
    // sends PARAM_SET. The value is RAM-only — the vehicle layer must not persist it.

    /**
     * @brief True when a SPEED_MAX value can be trusted right now.
     * Received AND greater than zero (0 means "no limit") AND the link is up
     * AND the reading is younger than MAVLINK_PARAM_STALE_MS. The two staleness gates are
     * deliberate: an unplugged cable falls back within the heartbeat timeout, an autopilot
     * that is alive but has stopped answering falls back within the parameter timeout.
     */
    bool hasSpeedMaxParam() const;

    /** @brief Last accepted SPEED_MAX in m/s (0 if never received). */
    float getSpeedMaxMs() const;

    /** @brief ms since the last accepted SPEED_MAX (MAVLINK_PARAM_STALE_MS if never received). */
    uint32_t getSpeedMaxAgeMs() const;

    // ---- Inbound commands (COMMAND_LONG) ------------------------------------

    /**
     * @brief Take a pending trip-reset request, clearing it.
     *
     * The transport validates and acknowledges the COMMAND_LONG and latches the request here;
     * the VEHICLE layer consumes it and performs the reset, because this transport must not
     * hold a SpeedSensor& (see the layering rule in openspec/project.md).
     *
     * @return true exactly once per accepted MAV_CMD_USER_1 trip reset
     */
    bool consumeTripResetRequest();

private:
    HardwareSerial* serial_;

    // Decoded command channels (microseconds)
    uint16_t channels_[16];

    // Link tracking
    uint32_t lastCmdTime_;
    uint32_t totalCommands_;
    uint32_t lastHeartbeatTime_;
    bool      heartbeatSeen_;

    // Command-rate rolling estimate
    uint32_t rateWindowStart_;
    uint32_t rateWindowCount_;
    float    lastCmdRate_;

    // Body-frame wheel odometry (VISION_POSITION_DELTA)
    int64_t  lastOdomTimeUs_;      // esp_timer_get_time() of the last send; 0 = baseline invalid
    uint32_t lastOdomDtMs_;        // last transmitted integration interval (ms) — debug only
    uint32_t odomTxCount_;         // VISION_POSITION_DELTA messages sent

    // Autopilot addressing (learned from HEARTBEAT)
    uint8_t targetSystem_;
    uint8_t targetComponent_;
    bool     targetKnown_;
    uint32_t lastStreamRequestTime_;   // millis() of last SET_MESSAGE_INTERVAL request (0 = none)
    uint32_t targetLearnedMs_;         // millis() the autopilot was learned (0 = not yet)

    // Autopilot SPEED_MAX subscription (RAM-only, never persisted)
    float    speedMaxMs_;              // last accepted value, m/s (NAN = never received)
    uint32_t speedMaxRxMs_;            // millis() of the last accepted value (0 = none)
    uint32_t lastParamRequestMs_;      // millis() of the last PARAM_REQUEST_READ (0 = none)

    // Outbound scheduling
    uint32_t lastHeartbeatTx_;
    uint32_t lastReportTx_;
    uint32_t lastSteerSlowTx_;         // VESC_TEMP / VESC_OK ride their own 1 Hz timer, not the
                                       // report tick
    uint32_t lastStatustextTx_;

    // Inbound trip reset, latched here and performed by the vehicle layer
    bool     tripResetPending_;        // set by an accepted MAV_CMD_USER_1, cleared on consumption

    // Change detection for STATUSTEXT
    char     lastGear_[4];
    char     lastIgnition_[12];
    bool     lastFailsafe_;
    bool     stateInitialized_;

    // Message handlers
    void handleHeartbeat(uint8_t sysid, uint8_t compid);
    void handleServoOutputRaw(const uint16_t* servoUs);
    void requestServoOutputStream();
    void sendStatusText(uint8_t severity, const char* text);

    // Send one NAMED_VALUE_FLOAT (251) from this component. `name` is a plain C string and is
    // ZERO-PADDED into a local 10-byte buffer before packing — the library's pack helper copies
    // exactly 10 bytes and a shorter literal would over-read. Takes DECODED scalars so this
    // header stays free of the mavlink C library headers.
    void sendNamedFloat(uint32_t nowMs, const char* name, float value);

    // Autopilot parameter subscription. Takes DECODED scalars (like handleServoOutputRaw) so
    // this header stays free of the mavlink C library headers.
    void handleParamValue(uint8_t sysid, uint8_t compid, const char* paramId, float value);
    void requestSpeedMaxParam();

    // Inbound COMMAND_LONG. Takes DECODED scalars for the same reason as handleParamValue, so
    // this header stays free of the mavlink C library headers. Addressing is strict: this
    // component shares the autopilot's system id, so only an exact target_system/target_component
    // match is handled and a broadcast is ignored WITHOUT an ACK.
    void handleCommandLong(uint8_t sysid, uint8_t compid, uint8_t targetSys, uint8_t targetComp,
                           uint16_t command, float param1);

    // Wheel speed as an EKF-fusable body-frame distance increment (gated — see the .cpp)
    void sendVisionPositionDelta(const StateReport& state);

    // Map a gear name ("R"/"N"/"H"/"L") to its physical-sequence value [-1,0,1,2].
    // unknownValue is returned for a null or unrecognised name: the ASSUMED-gear path
    // keeps the 0.0f (NEUTRAL) default, the PHYSICAL-gear path passes NAN so "cannot
    // tell" is never reported as a real gear.
    static float encodeGear(const char* gear, float unknownValue = 0.0f);

    // Mapping helpers (identical math to the former S-bus path)
    uint16_t clampUs(uint16_t valueUs) const;
    float applyDeadband(float value, float center, float deadband) const;
    float mapToPercentage(uint16_t valueUs, uint16_t minUs, uint16_t maxUs) const;
    float mapToPercentageBidirectional(uint16_t valueUs, uint16_t minUs,
                                       uint16_t centerUs, uint16_t maxUs) const;
};

#endif // MAVLINKINTERFACE_H
