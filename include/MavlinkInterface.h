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
 * a VFR_HUD carrying hall-sensor ground speed, plus STATUSTEXT for state
 * transitions.
 *
 * EFI_STATUS carries every value in the field that NAMES it (RPM, coolant, intake
 * air temp, manifold pressure, ECU load, measured TPS, commanded throttle, module
 * voltage), with two documented exceptions: the permanently-free fuel pair carries
 * the two GEAR values — fuel_consumed = ASSUMED (commanded) gear, fuel_flow =
 * PHYSICAL (opto-sensed) gear, NaN while unknown — and pt_compensation carries the
 * digital-output bitmask. See the field-mapping block in the .cpp.
 *
 * Also feeds the hall wheel speed to the autopilot's EKF3 as body-frame wheel
 * odometry (VISION_POSITION_DELTA). That message carries a BODY-FRAME distance
 * increment, so no heading is needed and nothing inbound is subscribed — EKF3
 * rotates it with its own attitude. The interface stays silent whenever the speed
 * validity or the direction sign is in doubt (a wrong measurement is worse than no
 * measurement).
 *
 * Subscribes READ-ONLY to a single autopilot parameter, `SPEED_MAX`, which the vehicle
 * layer uses as the ceiling of its own max-speed throttle limiter. Polled with
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
        float       speedKmh;       // hall-sensor ground speed, km/h (reported as VFR_HUD groundspeed)
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
     * Received AND greater than zero (ArduPilot reads 0 as "not set") AND the link is up
     * AND the reading is younger than MAVLINK_PARAM_STALE_MS. The two staleness gates are
     * deliberate: an unplugged cable falls back within the heartbeat timeout, an autopilot
     * that is alive but has stopped answering falls back within the parameter timeout.
     */
    bool hasSpeedMaxParam() const;

    /** @brief Last accepted SPEED_MAX in km/h (0 if never received). */
    float getSpeedMaxKmh() const;

    /** @brief Last accepted SPEED_MAX as received, in m/s (NAN if never received). */
    float getSpeedMaxRawMs() const;

    /** @brief ms since the last accepted SPEED_MAX (MAVLINK_PARAM_STALE_MS if never received). */
    uint32_t getSpeedMaxAgeMs() const;

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
    uint32_t lastStatustextTx_;

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

    // Autopilot parameter subscription. Takes DECODED scalars (like handleServoOutputRaw) so
    // this header stays free of the mavlink C library headers.
    void handleParamValue(uint8_t sysid, uint8_t compid, const char* paramId, float value);
    void requestSpeedMaxParam();

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
