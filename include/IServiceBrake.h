#ifndef I_SERVICE_BRAKE_H
#define I_SERVICE_BRAKE_H

#include <stdint.h>

/**
 * @file IServiceBrake.h
 * @brief The one way the firmware talks to the service brake (change
 *        replace-aztec-brake-with-ibooster, design D2), like IMotorDriver / IRelayPort.
 *
 * Loop-only. VehicleController sets a percent target (the applyBrake() value: 0 or 5..100) and
 * reads the commanded target, the measured applied percentage, `reached` (park-brake hand-over),
 * `available`, `requestActive` and the state. The aztec implementation is IboosterBrake (Bosch
 * iBooster Gen2 on CAN); no motor driver is held for the service brake.
 */

/** Service brake state (the iBooster unit state machine, design D4). */
enum class BrakeState : uint8_t { ASLEEP, STARTING, READY, BRAKING, RELEASING, LOST, FAULT };

/** What update() needs from the vehicle each loop. */
struct BrakeLoopInputs {
    uint32_t nowMs;
    bool ignOn;          // ignition state not OFF (ACC / IGNITION / CRANKING)
    bool cranking;       // ignition state CRANKING
    float speedMs;       // hall wheel speed, m/s
    bool speedValid;
    float batteryV;      // X10 battery sense, NaN when invalid (journal only)
};

class IServiceBrake {
public:
    virtual ~IServiceBrake() {}

    /** @brief The applyBrake() value: 0 (released) or 5..100 %. */
    virtual void setTargetPct(float pct) = 0;
    /** @brief Commanded target, % (what the throttle paths read as "brake commanded"). */
    virtual float targetPct() const = 0;
    /** @brief Measured brake as % of the maximum stroke, 0..100; 0 while not available. */
    virtual float appliedPct() const = 0;
    /** @brief BRAKING and the unit holds the CURRENT target (tolerance, stall or stroke stop). */
    virtual bool reached() const = 0;
    /**
     * @brief BRAKING, the unit accepted, positive flow, but the rod has not moved 0.3 mm for 3 s
     * ("no move"): ready but producing no force. The park-brake hand-over is then cancelled.
     */
    virtual bool stuck() const = 0;
    /** @brief READY / BRAKING / RELEASING. */
    virtual bool available() const = 0;
    /** @brief BRAKING / RELEASING: a brake request is on the bus. */
    virtual bool requestActive() const = 0;
    virtual BrakeState state() const = 0;

    /** @brief Once per loop: publish the command, read the unit state, journal its events. */
    virtual void update(const BrakeLoopInputs& in) = 0;
};

#endif  // I_SERVICE_BRAKE_H
