#ifndef IBOOSTER_BRAKE_H
#define IBOOSTER_BRAKE_H

#include <Arduino.h>
#include "CANController.h"
#include "CanIoLaw.h"
#include "IServiceBrake.h"
#include "IboosterLaw.h"

/**
 * @brief Aztec service brake: Bosch iBooster Gen2 (Tesla Model 3/Y) on the shared CAN bus
 * (change replace-aztec-brake-with-ibooster, design D2-D10).
 *
 * Two sides, one writer per shared record:
 *  - LOOP side (IServiceBrake): settings (NVS "brake"), percent -> rod target, publishes one
 *    IbCommand per loop, copies the task's IbSnapshot, drains the task's brake events into the
 *    journal, telemetry getters.
 *  - TASK side (ICanPeriodicClient, `can_io` task only): onFrame() checks and decodes 0x38E /
 *    0x38F / 0x39D; onSlot() steps the pure law (IboosterLaw.h ibStep()) once per 10 ms slot and
 *    builds 0x38D / 0x38C / 0x38B (IboosterProto.h). It never logs, allocates, blocks or touches NVS.
 *
 * Crossing the boundary (each copied whole under its own portMUX, held for one copy only):
 *    cmd_   loop -> task  (IbCommand, 16 B, written every loop)
 *    snap_  task -> loop  (IbSnapshot, written every slot)
 *    ev_    task -> loop  (8 IbEvent, full = the oldest is dropped and counted)
 *    holdBus_, winResetReq_  single aligned words (task-written / loop-written).
 */
class IboosterBrake : public IServiceBrake, public ICanPeriodicClient {
public:
    /** @brief Setter outcome (web replies). */
    enum class SetResult : uint8_t { OK, OUT_OF_RANGE, BUSY, SAVE_FAILED };

    /** @brief Task -> loop state (copied whole). */
    struct IbSnapshot {
        IbState state;
        bool have39D;
        uint8_t st, is, drv;
        int16_t rodCmm;
        bool have38E;
        uint8_t extStatus;
        int16_t aCmm, bCmm;
        bool have38F;
        uint8_t runoutBar;
        int8_t partner;
        uint32_t age39DMs;      // at publication
        uint32_t age38FMs;
        bool req;
        uint16_t flowWord;
        uint16_t targetCmm;     // target the task used in this slot
        bool reached;
        bool noMove;            // ready, accepted, flow > 0, rod not moving (3 s)
        bool cmdStale;
        bool crank;
        uint8_t raw39D[5], raw38E[6], raw38F[6];
        uint32_t slots;         // slots sent (session)
        uint32_t txMiss;        // periodic frames missed (session, from the CAN task)
        uint32_t badCsum;       // unit frames failing their checksum (session)
        uint32_t lateFrames;    // unit frames drained after a task stall, not used (session)
        uint32_t drops;         // unit drops (session)
        uint32_t faults;        // FAULT entries (session)
        uint32_t gapMaxMs;      // longest gap between two sent slots (session)
        uint32_t winGapMaxMs;   // ... since the last stats line
        uint32_t cmdMaxMs;      // oldest command used (session)
        uint32_t winCmdMaxMs;   // ... since the last stats line
        uint32_t evDrops;       // brake events dropped from the full ring
        uint32_t publishedMs;
    };

    IboosterBrake();

    /**
     * @brief setup(), before the CAN bus starts: load the settings (out-of-range -> default, logged,
     * not rewritten) and remove the old X7 keys "travel_ms" / "hold_pwm" once.
     */
    void begin();

    // ---- IServiceBrake (loop) ----
    void setTargetPct(float pct) override;
    float targetPct() const override { return targetPct_; }
    float appliedPct() const override;
    bool reached() const override;
    bool stuck() const override { return snapL_.noMove && snapL_.state == IbState::BRAKING; }
    bool available() const override { return ibAvailable(snapL_.state); }
    bool requestActive() const override { return ibRequestState(snapL_.state); }
    BrakeState state() const override { return (BrakeState)snapL_.state; }
    void update(const BrakeLoopInputs& in) override;

    // ---- ICanPeriodicClient (CAN task; holdsBus() loop) ----
    void onSlot(const CanSlotInfo& info, CanSlotTx& out) override;
    bool onFrame(const CanFrame& f, uint32_t nowMs) override;
    bool holdsBus() const override { return holdBus_; }

    // ---- Settings (web card, persisted first, refused while a request is active) ----
    SetResult setMaxMm10(int32_t mm10);
    SetResult setApplyRate(int32_t mlps);
    SetResult setReleaseRate(int32_t mlps);
    uint8_t getMaxMm10() const { return maxMm10_; }
    uint8_t getApplyRate() const { return applyMlps_; }
    uint8_t getReleaseRate() const { return releaseMlps_; }

    // ---- Telemetry ----
    const IbSnapshot& snapshot() const { return snapL_; }
    const char* stateToken() const { return ibStateToken(snapL_.state); }
    /** @brief Rod position, mm; NAN when no fresh 0x39D. */
    float rodMm() const;
    /** @brief STARTING for longer than IB_START_TIMEOUT_MS (attention item). */
    bool startingTooLong() const;

private:
    // Loop side
    float targetPct_;
    uint8_t maxMm10_, applyMlps_, releaseMlps_;
    uint16_t lastTargetCmm_;       // target published in the last command
    IbSnapshot snapL_;             // loop copy of the snapshot
    uint32_t lastStatsMs_;
    uint32_t stSlots_, stMiss_, stBad_, stDrops_;
    bool troubleActive_;           // a LOST / FAULT episode is open (for "recovered")
    uint32_t troubleSinceMs_;
    uint32_t startingSinceMs_;
    bool lastIgnOn_;
    float lastVbat_;
    // State-line flood guard (the journal dedupe exempts BRAKE state lines): burst / refill
    uint8_t stateTokens_;
    uint32_t stateRefillMs_;
    uint32_t stateSuppressed_;

    // Shared
    portMUX_TYPE cmdMux_;
    portMUX_TYPE snapMux_;
    portMUX_TYPE evMux_;
    IbCommand cmd_;                // loop writes, task reads
    IbSnapshot snap_;              // task writes, loop reads

    struct IbEvent {
        uint32_t ms;
        uint8_t kind;              // EvKind
        uint8_t from, to;          // IbState
        uint8_t st, is, drv;
        bool req, crank, ign;
        int16_t rodCmm;
        uint16_t tgtCmm;
        uint16_t flowWord;
        uint32_t valMs;            // gap / age / drop / stale length
        uint8_t raw39D[5], raw38E[6], raw38F[6];
        // EV_EPISODE only (valMs = duration): the `BRAKE I episode` summary
        bool epRodValid, epRunValid, epStop;
        uint8_t epSpdMin, epRunMin, epRunMax;
        int16_t epRodMaxCmm;
    };
    enum EvKind : uint8_t { EV_STATE, EV_UNIT_DROP, EV_RELEASE_SLOW, EV_FAULT_LATCHED, EV_START_TIMEOUT,
                            EV_NO_UNIT, EV_CMD_STALE, EV_TX_GAP, EV_NO_MOVE, EV_EPISODE };
    CanRing<IbEvent, 8> ev_;       // task pushes, loop pops (evMux_)
    volatile bool holdBus_;        // task-written: request active
    volatile uint32_t winResetReq_; // loop-written: restart the window maxima
    uint32_t winResetAck_;         // task-only

    // Task side (written only in the CAN task)
    IbLaw law_;
    IbUnitView unit_;
    uint8_t counter_;
    bool have38E_;
    uint8_t extStatus_;
    int16_t aCmm_, bCmm_;
    uint8_t raw39D_[5], raw38E_[6], raw38F_[6];
    uint32_t lastSlotMs_;
    bool haveLastSent_;
    uint32_t lastSentMs_;
    bool prevReq_;
    uint16_t prevFlowWord_;
    uint32_t tSlots_, tBad_, tLate_, tDrops_, tFaults_, tGapMax_, tWinGapMax_, tCmdMax_, tWinCmdMax_, tEvDrops_;
    // Brake episode summary (change add-ibooster-speed-floor, design D2)
    IbEpisode episode_;
    bool epHave38F_;               // a 0x38F arrival time was seen by a previous slot ...
    uint32_t epT38F_;              // ... and this was it ("new 0x38F this slot" = it changed)

    void taskPushEvent(uint8_t kind, const IbStepOut& o, const IbCommand& c, uint32_t valMs,
                       const IbEpisodeSummary* ep = nullptr);
    void journalEvent(const IbEvent& e);
    void journalStats(uint32_t now);
    bool takeStateToken(uint32_t now);
    SetResult saveSetting(const char* key, uint8_t value);
};

#endif  // IBOOSTER_BRAKE_H
