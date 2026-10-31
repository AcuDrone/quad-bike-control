#ifndef BOARD_INPUTS_H
#define BOARD_INPUTS_H

#include <Arduino.h>
#include <Wire.h>
#include "Constants.h"
#include "PCA9557Expander.h"

/**
 * @brief Polled reader for the eight 5V opto-isolated (PC817) board inputs.
 *
 * The inputs live on the on-board PCA9557 U10 @ `PCA9557_ADDR_INPUTS` (0x1A) on
 * I2C2 (`Wire1`), shared with the on-board CD4051 mux expander U2 @0x1C. (Live
 * bus scans put U10 on I2C2; the schematic's port names suggested I2C1 — the
 * copper wins.) Consumers
 * (`TransmissionController::getPhysicalGear()`, `VehicleController::isBrakeReleased()`)
 * read the published snapshot and never issue I2C themselves, so a control-loop
 * read can never turn into a burst of blocking transactions.
 *
 * Input map (bench-verified 2026-08-21, see Constants.h): port bits 0/1/6/7 =
 * gear NEUTRAL/REVERSE/LOW/HIGH, engaged-HIGH; bit 4 = brake "released" endstop,
 * polarity from `BOARD_INPUT_ACTIVE_LOW`. The remaining bits are spare.
 *
 * Fault policy (see the change's design.md, Decision 2):
 *  - a single failed read keeps the last-known snapshot and retries next poll;
 *  - no good read for `BOARD_INPUT_STALE_MS` invalidates the snapshot and raises
 *    the fault flag (logged once per episode);
 *  - while faulted, the device is re-initialized every `BOARD_INPUT_RECOVER_MS`
 *    and recovery is automatic on the first good read.
 */
class BoardInputs {
public:
    /** @brief Immutable view of the input state published to consumers */
    struct Snapshot {
        bool gearReverse;    // bit 1 HIGH
        bool gearNeutral;    // bit 0 HIGH
        bool gearLow;        // bit 6 HIGH
        bool gearHigh;       // bit 7 HIGH
        bool brakeReleased;  // bit 4 asserted — brake at its retracted endstop
        bool valid;          // false once the data is stale (do not trust the bools)
        uint32_t ageMs;      // ms since the last successful read
    };

    /**
     * @brief Bind to the expander on the given bus (bound at construction; the
     * bus itself is opened by `main.cpp` before `begin()` is called).
     */
    explicit BoardInputs(TwoWire& bus = Wire1);

    /**
     * @brief Configure all eight pins as inputs and take a first read.
     * @return true only if the device answers and the seed read succeeds
     */
    bool begin();

    /**
     * @brief Poll the input port at most every `BOARD_INPUT_POLL_MS`.
     * Never blocks beyond a single I2C transaction; safe to call every loop().
     */
    void update();

    /** @brief Latest published snapshot (age computed at call time) */
    Snapshot getSnapshot() const;

    /** @brief True while the reader is in a fault episode (stale/unreachable) */
    bool isFaulted() const { return faulted_; }

    /** @brief Consecutive failed reads since the last good one */
    uint32_t getFailureCount() const { return failureCount_; }

    /** @brief ms since the last successful read (UINT32_MAX if there never was one) */
    uint32_t getAgeMs() const;

    /** @brief Raw input port byte of the last successful read (diagnostics) */
    uint8_t getRawInputs() const { return rawInputs_; }

private:
    /** @brief Decode the raw port byte into the snapshot bools */
    void applyRaw(uint8_t raw);

    /** @brief True when the given bit index is asserted in `raw` (active-low inputs) */
    static bool bitAsserted(uint8_t raw, uint8_t bit);

    /** @brief Raw level of the given bit index in `raw` (engaged-HIGH gear inputs) */
    static bool bitHigh(uint8_t raw, uint8_t bit);

    /** @brief (Re)configure the device as all-inputs; used by begin() and recovery */
    bool configureDevice();

    PCA9557Expander expander_;

    Snapshot snapshot_;
    uint8_t  rawInputs_;
    uint16_t lastLoggedRaw_;       // 0xFFFF until the first read is logged (never a valid byte)

    bool     initialized_;
    bool     faulted_;
    uint32_t failureCount_;
    uint32_t lastPollMs_;
    uint32_t lastGoodReadMs_;
    bool     hasGoodRead_;
    uint32_t lastRecoverAttemptMs_;
    uint32_t lastFailLogMs_;
};

#endif // BOARD_INPUTS_H
