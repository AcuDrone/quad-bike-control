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
 * to the MAVLink network using standard messages (HEARTBEAT / EFI_STATUS /
 * VFR_HUD) plus STATUSTEXT for state transitions.
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
        uint32_t totalCommands;   // SERVO_OUTPUT_RAW frames received since init
        float    commandRate;     // Command frame rate (Hz, rolling)
        uint32_t signalAge;       // ms since last command frame
        uint32_t heartbeatAge;    // ms since last autopilot HEARTBEAT
        bool     isValid;         // Command stream within timeout
        bool     linkUp;          // Heartbeat within timeout
    };

    /**
     * @brief Plain snapshot of vehicle state to report back over MAVLink.
     * Kept dependency-free so MavlinkInterface need not include CAN/relay headers.
     */
    struct StateReport {
        bool        canValid;       // CAN data fresh/valid
        uint16_t    engineRpm;      // RPM
        int8_t      coolantTemp;    // °C
        uint8_t     throttlePct;    // %
        const char* gearFrom;       // gear the current step is leaving "R"/"N"/"L"/"H"
        const char* gearTo;         // current step / assumed gear "R"/"N"/"L"/"H"
        bool        gearMoving;     // true while the servo is actively moving (a phase is active)
        const char* ignition;       // "OFF"/"ACC"/"IGNITION"/"CRANKING"
        bool        failsafe;       // true when in fail-safe
        // Note: vehicle speed and oil temperature are not available from the ECU
        // and are therefore not reported over MAVLink.
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

    // ---- Link monitoring ----------------------------------------------------

    /** @brief True if a command frame arrived within MAVLINK_CMD_TIMEOUT_MS. */
    bool isSignalValid() const;

    /** @brief ms since the last command frame. */
    uint32_t getSignalAge() const;

    /** @brief True if an autopilot heartbeat arrived within the heartbeat timeout. */
    bool isLinkUp() const;

    /** @brief Aggregate link metrics. */
    LinkQuality getLinkQuality() const;

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

    // Autopilot addressing (learned from HEARTBEAT)
    uint8_t targetSystem_;
    uint8_t targetComponent_;
    bool     targetKnown_;
    uint32_t lastStreamRequestTime_;   // millis() of last SET_MESSAGE_INTERVAL request (0 = none)

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
    void sendNamedValueFloat(const char* name, float value);

    // Map a gear name ("R"/"N"/"H"/"L") to its physical-sequence value [-1,0,1,2]
    static float encodeGear(const char* gear);

    // Mapping helpers (identical math to the former S-bus path)
    uint16_t clampUs(uint16_t valueUs) const;
    float applyDeadband(float value, float center, float deadband) const;
    float mapToPercentage(uint16_t valueUs, uint16_t minUs, uint16_t maxUs) const;
    float mapToPercentageBidirectional(uint16_t valueUs, uint16_t minUs,
                                       uint16_t centerUs, uint16_t maxUs) const;
};

#endif // MAVLINKINTERFACE_H
