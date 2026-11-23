#include "IboosterBrake.h"
#include "Debug.h"
#include "EventJournal.h"
#include "IboosterProto.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

// IboosterLaw.h cannot include Constants.h (native tests): keep its literals in sync.
static_assert(IB_LAW_SILENT_MS == IB_SILENT_MS, "IboosterLaw.h silent out of sync");
static_assert(IB_LAW_STATUS_STALE_MS == IB_STATUS_STALE_MS, "IboosterLaw.h status stale out of sync");
static_assert(IB_LAW_ROD_HOLD_MS == IB_ROD_HOLD_MS, "IboosterLaw.h rod hold out of sync");
static_assert(IB_LAW_START_TIMEOUT_MS == IB_START_TIMEOUT_MS, "IboosterLaw.h start timeout out of sync");
static_assert(IB_LAW_NO_UNIT_MS == IB_NO_UNIT_MS, "IboosterLaw.h no-unit timeout out of sync");
static_assert(IB_LAW_FAULT_PERSIST_MS == IB_FAULT_PERSIST_MS, "IboosterLaw.h fault persist out of sync");
static_assert(IB_LAW_ACCEPT_TIMEOUT_MS == IB_ACCEPT_TIMEOUT_MS, "IboosterLaw.h accept timeout out of sync");
static_assert(IB_LAW_CRANK_GRACE_MS == IB_CRANK_GRACE_MS, "IboosterLaw.h crank grace out of sync");
static_assert(IB_LAW_CMD_STALE_MS == IB_CMD_STALE_MS, "IboosterLaw.h stale command out of sync");
static_assert(IB_LAW_STATUSTEXT_MIN_MS == IB_STATUSTEXT_MIN_MS, "IboosterLaw.h STATUSTEXT hold-off out of sync");
static_assert(IB_LAW_ROD_KP == IB_ROD_KP, "IboosterLaw.h kp out of sync");
static_assert(IB_LAW_DEADBAND_CMM == (int)(IB_ROD_DEADBAND_MM * 100.0f + 0.5f), "IboosterLaw.h deadband out of sync");
static_assert(IB_LAW_REACHED_TOL_CMM == (int)(IB_REACHED_TOL_MM * 100.0f + 0.5f), "IboosterLaw.h reached tolerance out of sync");
static_assert(IB_LAW_STALL_CMM == (int)(IB_STALL_MM * 100.0f + 0.5f), "IboosterLaw.h stall distance out of sync");
static_assert(IB_LAW_STALL_MS == IB_STALL_MS, "IboosterLaw.h stall time out of sync");
static_assert(IB_LAW_RELEASE_HANDOFF_CMM == (int)(IB_RELEASE_HANDOFF_MM * 100.0f + 0.5f), "IboosterLaw.h hand-off out of sync");
static_assert(IB_LAW_RELEASE_TIMEOUT_MS == IB_RELEASE_TIMEOUT_MS && IB_LAW_RELEASE_MIN_MS == IB_RELEASE_TIMEOUT_MS,
              "IboosterLaw.h release timeout out of sync");
static_assert(IB_LAW_RELEASE_MAX_MS == IB_RELEASE_MAX_MS, "IboosterLaw.h release cap out of sync");
static_assert(IB_LAW_ARM_CMM == (int)(IB_ARM_MM * 100.0f + 0.5f), "IboosterLaw.h arming distance out of sync");
static_assert(IB_LAW_NO_MOVE_MS == IB_NO_MOVE_MS, "IboosterLaw.h no-move time out of sync");
static_assert(IB_LAW_ROD_CEIL_CMM == (int)(IB_ROD_CEIL_MM * 100.0f + 0.5f), "IboosterLaw.h ceiling out of sync");
static_assert(IB_LAW_MAX_MM10_MIN == BRAKE_MAX_MM10_MIN && IB_LAW_MAX_MM10_MAX == BRAKE_MAX_MM10_MAX &&
              IB_LAW_MAX_MM10_DEFAULT == BRAKE_MAX_MM10_DEFAULT, "IboosterLaw.h max stroke out of sync");
static_assert(IB_LAW_RATE_MIN_MLPS == BRAKE_RATE_MIN_MLPS && IB_LAW_RATE_MAX_MLPS == BRAKE_RATE_MAX_MLPS &&
              IB_LAW_RATE_DEFAULT_MLPS == BRAKE_RATE_DEFAULT_MLPS, "IboosterLaw.h rates out of sync");
static_assert(IB_LAW_FLOW_ZERO == IB_FLOW_ZERO && IB_LAW_FLOW_MIN == IB_FLOW_MIN && IB_LAW_FLOW_MAX == IB_FLOW_MAX,
              "IboosterLaw.h flow range out of sync with IboosterProto.h");
static_assert(IB_LAW_UNITS_PER_MLPS == IB_FLOW_UNITS_PER_MLPS, "IboosterLaw.h flow scale out of sync");
static_assert(sizeof(IbCommand) == 16, "IbCommand is the 16-byte loop -> task record");
// BrakeState (IServiceBrake.h) mirrors IbState (IboosterLaw.h) value for value.
static_assert((int)BrakeState::ASLEEP == (int)IbState::ASLEEP && (int)BrakeState::STARTING == (int)IbState::STARTING &&
              (int)BrakeState::READY == (int)IbState::READY && (int)BrakeState::BRAKING == (int)IbState::BRAKING &&
              (int)BrakeState::RELEASING == (int)IbState::RELEASING && (int)BrakeState::LOST == (int)IbState::LOST &&
              (int)BrakeState::FAULT == (int)IbState::FAULT, "BrakeState / IbState out of sync");

// State-line flood guard (BRAKE state lines are exempt from the journal's 2 s dedupe because
// availability transitions come in quick pairs): a burst of 8, then one line per second.
static constexpr uint8_t STATE_LINE_BURST = 8;
static constexpr uint32_t STATE_LINE_REFILL_MS = 1000;

IboosterBrake::IboosterBrake()
    : targetPct_(0.0f),
      maxMm10_(BRAKE_MAX_MM10_DEFAULT),
      applyMlps_(BRAKE_RATE_DEFAULT_MLPS),
      releaseMlps_(BRAKE_RATE_DEFAULT_MLPS),
      lastTargetCmm_(0),
      snapL_(),
      lastStatsMs_(0),
      stSlots_(0), stMiss_(0), stBad_(0), stDrops_(0),
      troubleActive_(false),
      troubleSinceMs_(0),
      startingSinceMs_(0),
      lastIgnOn_(false),
      lastVbat_(NAN),
      stateTokens_(STATE_LINE_BURST),
      stateRefillMs_(0),
      stateSuppressed_(0),
      cmd_(),
      snap_(),
      ev_(),
      holdBus_(false),
      winResetReq_(0),
      winResetAck_(0),
      law_(ibLawInitial(0)),
      unit_(),
      counter_(0),
      have38E_(false),
      extStatus_(0),
      aCmm_(0),
      bCmm_(0),
      lastSlotMs_(0),
      haveLastSent_(false),
      lastSentMs_(0),
      prevReq_(false),
      prevFlowWord_(IB_FLOW_ZERO),
      tSlots_(0), tBad_(0), tLate_(0), tDrops_(0), tFaults_(0), tGapMax_(0), tWinGapMax_(0),
      tCmdMax_(0), tWinCmdMax_(0), tEvDrops_(0),
      episode_(ibEpisodeInitial()),
      epHave38F_(false),
      epT38F_(0) {
    portMUX_INITIALIZE(&cmdMux_);
    portMUX_INITIALIZE(&snapMux_);
    portMUX_INITIALIZE(&evMux_);
    memset(raw39D_, 0, sizeof(raw39D_));
    memset(raw38E_, 0, sizeof(raw38E_));
    memset(raw38F_, 0, sizeof(raw38F_));
    snapL_.state = IbState::ASLEEP;
    snapL_.flowWord = IB_FLOW_ZERO;
    snap_ = snapL_;
}

// ============================================================================
// SETUP / SETTINGS (loop)
// ============================================================================

void IboosterBrake::begin() {
    int32_t mx = BRAKE_MAX_MM10_DEFAULT;
    int32_t ap = BRAKE_RATE_DEFAULT_MLPS;
    int32_t rl = BRAKE_RATE_DEFAULT_MLPS;
    Preferences prefs;
    if (prefs.begin(BRAKE_NVS_NAMESPACE, false)) {
        // The X7 actuator's travel time / hold drive mean nothing for a rod: removed once, no
        // migration (design D6). A rollback then runs on its own defaults.
        if (prefs.isKey(BRAKE_OLD_TRAVEL_NVS_KEY)) {
            prefs.remove(BRAKE_OLD_TRAVEL_NVS_KEY);
            Debug::printlnFeature(DebugFeature::BRAKE, "[BRAKE] Old key brake/travel_ms removed");
        }
        if (prefs.isKey(BRAKE_OLD_HOLD_NVS_KEY)) {
            prefs.remove(BRAKE_OLD_HOLD_NVS_KEY);
            Debug::printlnFeature(DebugFeature::BRAKE, "[BRAKE] Old key brake/hold_pwm removed");
        }
        mx = prefs.getUChar(BRAKE_MAX_MM10_NVS_KEY, BRAKE_MAX_MM10_DEFAULT);
        ap = prefs.getUChar(BRAKE_APPLY_NVS_KEY, BRAKE_RATE_DEFAULT_MLPS);
        rl = prefs.getUChar(BRAKE_RELEASE_NVS_KEY, BRAKE_RATE_DEFAULT_MLPS);
        prefs.end();
    }
    // Out of range: ignored and logged, the default used, the stored value not rewritten.
    if (!ibMaxMm10Valid(mx)) {
        Debug::printfFeature(DebugFeature::BRAKE, "[BRAKE] Stored max stroke %ld (0.1 mm) out of range — default used\n", (long)mx);
    }
    if (!ibRateValid(ap)) {
        Debug::printfFeature(DebugFeature::BRAKE, "[BRAKE] Stored apply rate %ld ml/s out of range — default used\n", (long)ap);
    }
    if (!ibRateValid(rl)) {
        Debug::printfFeature(DebugFeature::BRAKE, "[BRAKE] Stored release rate %ld ml/s out of range — default used\n", (long)rl);
    }
    maxMm10_ = (uint8_t)ibMaxMm10OrDefault(mx);
    applyMlps_ = (uint8_t)ibRateOrDefault(ap);
    releaseMlps_ = (uint8_t)ibRateOrDefault(rl);
    Debug::printfFeature(DebugFeature::BRAKE,
        "[BRAKE] iBooster service brake: max stroke %.1f mm, apply %u ml/s, release %u ml/s\n",
        maxMm10_ / 10.0, (unsigned)applyMlps_, (unsigned)releaseMlps_);
}

IboosterBrake::SetResult IboosterBrake::saveSetting(const char* key, uint8_t value) {
    bool ok = false;
    Preferences prefs;
    if (prefs.begin(BRAKE_NVS_NAMESPACE, false)) {
        ok = prefs.putUChar(key, value) == sizeof(uint8_t);
        prefs.end();
    }
    return ok ? SetResult::OK : SetResult::SAVE_FAILED;
}

// Persisted first, then used from the next command / slot; refused while a request is active
// (no flash write during a request, no stroke change mid-brake — design D6, D9).
IboosterBrake::SetResult IboosterBrake::setMaxMm10(int32_t mm10) {
    if (!ibMaxMm10OnGrid(mm10)) return SetResult::OUT_OF_RANGE;   // 2.0-16.0 mm in 0.5 mm steps
    if (requestActive()) return SetResult::BUSY;
    const SetResult r = saveSetting(BRAKE_MAX_MM10_NVS_KEY, (uint8_t)mm10);
    if (r == SetResult::OK) maxMm10_ = (uint8_t)mm10;
    return r;
}

IboosterBrake::SetResult IboosterBrake::setApplyRate(int32_t mlps) {
    if (!ibRateValid(mlps)) return SetResult::OUT_OF_RANGE;
    if (requestActive()) return SetResult::BUSY;
    const SetResult r = saveSetting(BRAKE_APPLY_NVS_KEY, (uint8_t)mlps);
    if (r == SetResult::OK) applyMlps_ = (uint8_t)mlps;
    return r;
}

IboosterBrake::SetResult IboosterBrake::setReleaseRate(int32_t mlps) {
    if (!ibRateValid(mlps)) return SetResult::OUT_OF_RANGE;
    if (requestActive()) return SetResult::BUSY;
    const SetResult r = saveSetting(BRAKE_RELEASE_NVS_KEY, (uint8_t)mlps);
    if (r == SetResult::OK) releaseMlps_ = (uint8_t)mlps;
    return r;
}

// ============================================================================
// LOOP SIDE
// ============================================================================

void IboosterBrake::setTargetPct(float pct) {
    // applyBrake() already clamps (0 below 5 %, 100 above 100 %); garbage is released here too.
    if (!(pct > 0.0f)) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    targetPct_ = pct;
}

float IboosterBrake::rodMm() const {
    if (!snapL_.have39D || snapL_.age39DMs > IB_STATUS_STALE_MS) return NAN;
    return snapL_.rodCmm / 100.0f;
}

float IboosterBrake::appliedPct() const {
    const float rod = rodMm();
    if (!available() || isnan(rod)) return 0.0f;
    float pct = rod * 1000.0f / (float)maxMm10_;   // rod / (max mm) x 100
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

// Only for the CURRENT target: the snapshot must come from a slot that used the target this
// loop published (a stale "reached" of a previous target never fires the park-brake release).
bool IboosterBrake::reached() const {
    return snapL_.reached && lastTargetCmm_ > 0 && snapL_.targetCmm == lastTargetCmm_;
}

bool IboosterBrake::startingTooLong() const {
    return snapL_.state == IbState::STARTING && startingSinceMs_ != 0 &&
           (millis() - startingSinceMs_) > IB_START_TIMEOUT_MS;
}

void IboosterBrake::update(const BrakeLoopInputs& in) {
    lastVbat_ = in.batteryV;
    lastIgnOn_ = in.ignOn;

    IbCommand c = {};
    c.stampMs = in.nowMs ? in.nowMs : 1;
    c.targetCmm = ibTargetCmm(targetPct_, maxMm10_);
    c.applyUnits = ibRateUnits(applyMlps_);
    c.releaseUnits = ibRateUnits(releaseMlps_);
    c.speedByte = ibSpeedByte(in.speedMs, in.speedValid);
    c.ignOn = in.ignOn;
    c.cranking = in.cranking;
    portENTER_CRITICAL(&cmdMux_);
    cmd_ = c;
    portEXIT_CRITICAL(&cmdMux_);
    lastTargetCmm_ = c.targetCmm;

    portENTER_CRITICAL(&snapMux_);
    snapL_ = snap_;
    portEXIT_CRITICAL(&snapMux_);

    if (snapL_.state == IbState::STARTING) {
        if (startingSinceMs_ == 0) startingSinceMs_ = in.nowMs | 1u;
    } else {
        startingSinceMs_ = 0;
    }

    for (uint8_t i = 0; i < 8; i++) {
        IbEvent e;
        portENTER_CRITICAL(&evMux_);
        const bool got = ev_.pop(e);
        portEXIT_CRITICAL(&evMux_);
        if (!got) break;
        journalEvent(e);
    }
    journalStats(in.nowMs);
}

bool IboosterBrake::takeStateToken(uint32_t now) {
    while (stateTokens_ < STATE_LINE_BURST && (now - stateRefillMs_) >= STATE_LINE_REFILL_MS) {
        stateTokens_++;
        stateRefillMs_ += STATE_LINE_REFILL_MS;
    }
    if (stateTokens_ == STATE_LINE_BURST) stateRefillMs_ = now;
    if (stateTokens_ == 0) {
        stateSuppressed_++;
        return false;
    }
    stateTokens_--;
    return true;
}

static void fmtMm(char* out, size_t cap, bool have, int32_t cmm) {
    if (!have) {
        snprintf(out, cap, "-");
    } else {
        snprintf(out, cap, "%s%ld.%02ld", cmm < 0 ? "-" : "", (long)(labs(cmm) / 100), (long)(labs(cmm) % 100));
    }
}

static void fmtHex(char* out, const uint8_t* d, uint8_t n) {
    static const char hexd[] = "0123456789ABCDEF";
    for (uint8_t i = 0; i < n; i++) {
        out[2 * i] = hexd[d[i] >> 4];
        out[2 * i + 1] = hexd[d[i] & 0x0F];
    }
    out[2 * n] = '\0';
}

void IboosterBrake::journalEvent(const IbEvent& e) {
    char rod[12];
    char tgt[12];
    char vbat[8];
    fmtMm(rod, sizeof(rod), e.st != 0xFF, e.rodCmm);
    fmtMm(tgt, sizeof(tgt), true, e.tgtCmm);
    if (isnan(lastVbat_)) {
        snprintf(vbat, sizeof(vbat), "-");
    } else {
        snprintf(vbat, sizeof(vbat), "%.1f", lastVbat_);
    }
    const unsigned st = e.st == 0xFF ? 0 : e.st;
    switch ((EvKind)e.kind) {
        case EV_STATE: {
            const IbState from = (IbState)e.from;
            const IbState to = (IbState)e.to;
            if (ibAvailable(from) && ibAvailable(to)) {
                return;   // normal braking is not journaled
            }
            Debug::printfFeature(DebugFeature::BRAKE, "[BRAKE] %s -> %s (st=%u is=%u drv=%u rod=%s)\n",
                                 ibStateName(from), ibStateName(to), st, (unsigned)e.is, (unsigned)e.drv, rod);
            if (takeStateToken(e.ms)) {
                char sup[16] = "";
                if (stateSuppressed_) {
                    snprintf(sup, sizeof(sup), " sup=%lu", (unsigned long)stateSuppressed_);
                    stateSuppressed_ = 0;
                }
                EventJournal::log(JSrc::BRAKE, JLvl::I, "state", "from=%s to=%s st=%u is=%u drv=%u rod=%s ign=%d crank=%d%s",
                                  ibStateName(from), ibStateName(to), st, (unsigned)e.is, (unsigned)e.drv, rod,
                                  e.ign ? 1 : 0, e.crank ? 1 : 0, sup);
            }
            const JLvl trouble = e.crank ? JLvl::W : JLvl::E;
            if (to == IbState::FAULT || to == IbState::LOST) {
                if (to == IbState::FAULT) {
                    EventJournal::log(JSrc::BRAKE, trouble, "fault",
                                      "st=%u is=%u drv=%u rod=%s tgt=%s req=%d flow=0x%04X vbat=%s crank=%d",
                                      st, (unsigned)e.is, (unsigned)e.drv, rod, tgt, e.req ? 1 : 0,
                                      (unsigned)e.flowWord, vbat, e.crank ? 1 : 0);
                } else {
                    EventJournal::log(JSrc::BRAKE, trouble, "lost", "age=%lu tgt=%s req=%d vbat=%s crank=%d",
                                      (unsigned long)e.valMs, tgt, e.req ? 1 : 0, vbat, e.crank ? 1 : 0);
                }
                char r39[11], r8e[13], r8f[13];
                fmtHex(r39, e.raw39D, 5);
                fmtHex(r8e, e.raw38E, 6);
                fmtHex(r8f, e.raw38F, 6);
                EventJournal::log(JSrc::BRAKE, JLvl::I, "fault_raw", "r39d=%s r38e=%s r38f=%s", r39, r8e, r8f);
                if (!troubleActive_) {
                    troubleActive_ = true;
                    troubleSinceMs_ = e.ms;
                }
            } else if (to == IbState::READY && troubleActive_) {
                EventJournal::log(JSrc::BRAKE, JLvl::I, "recovered", "after_s=%lu",
                                  (unsigned long)((e.ms - troubleSinceMs_) / 1000));
                troubleActive_ = false;
            } else if (to == IbState::ASLEEP) {
                troubleActive_ = false;   // the episode ended with the ignition: nothing to recover
            }
            break;
        }
        case EV_UNIT_DROP:
            EventJournal::log(JSrc::BRAKE, JLvl::W, "unit_drop", "gap_ms=%lu rod=%s tgt=%s",
                              (unsigned long)e.valMs, rod, tgt);
            break;
        case EV_TX_GAP:
            EventJournal::log(JSrc::BRAKE, JLvl::W, "tx_gap", "ms=%lu req=%d", (unsigned long)e.valMs, e.req ? 1 : 0);
            break;
        case EV_CMD_STALE:
            EventJournal::log(JSrc::BRAKE, JLvl::W, "cmd_stale", "ms=%lu tgt=%s req=%d",
                              (unsigned long)e.valMs, tgt, e.req ? 1 : 0);
            break;
        case EV_RELEASE_SLOW:
            EventJournal::log(JSrc::BRAKE, JLvl::W, "release_slow", "rod=%s", rod);
            break;
        case EV_FAULT_LATCHED:
            EventJournal::log(JSrc::BRAKE, JLvl::E, "fault_latched", "s=%lu",
                              (unsigned long)(IB_FAULT_PERSIST_MS / 1000));
            break;
        case EV_NO_MOVE:
            // Ready, accepted and pushed, but the rod has not moved 0.3 mm for 3 s: no brake force.
            // The park-brake hand-over is cancelled (no release).
            EventJournal::log(JSrc::BRAKE, JLvl::W, "no_move", "ms=%lu rod=%s tgt=%s flow=0x%04X",
                              (unsigned long)IB_NO_MOVE_MS, rod, tgt, (unsigned)e.flowWord);
            break;
        case EV_EPISODE: {
            // One line per brake request episode (change add-ibooster-speed-floor, design D2); the
            // normal 2 s (source, event) dedupe applies (held-back episodes -> sup=N).
            char rmax[12];
            char run[12];
            fmtMm(rmax, sizeof(rmax), e.epRodValid, e.epRodMaxCmm);
            if (e.epRunValid) {
                snprintf(run, sizeof(run), "%u..%u", (unsigned)e.epRunMin, (unsigned)e.epRunMax);
            } else {
                snprintf(run, sizeof(run), "-");
            }
            EventJournal::log(JSrc::BRAKE, JLvl::I, "episode", "ms=%lu rod_max=%s spd_min=%u runout=%s stop=%d",
                              (unsigned long)e.valMs, rmax, (unsigned)e.epSpdMin, run, e.epStop ? 1 : 0);
            break;
        }
        case EV_START_TIMEOUT:
            EventJournal::log(JSrc::BRAKE, JLvl::E, "start_timeout", nullptr);
            break;
        case EV_NO_UNIT:
            EventJournal::log(JSrc::BRAKE, JLvl::E, "no_unit", nullptr);
            break;
    }
}

// Window statistics every JOURNAL_SNAP_MS while the ignition is on (design D10)
void IboosterBrake::journalStats(uint32_t now) {
    if ((now - lastStatsMs_) < JOURNAL_SNAP_MS) {
        return;
    }
    lastStatsMs_ = now;
    const IbSnapshot& s = snapL_;
    if (lastIgnOn_) {
        EventJournal::log(JSrc::BRAKE, JLvl::I, "stats", "tx=%lu miss=%lu gap_max=%lu bad=%lu drops=%lu cmd_max=%lu",
                          (unsigned long)(s.slots - stSlots_), (unsigned long)(s.txMiss - stMiss_),
                          (unsigned long)s.winGapMaxMs, (unsigned long)(s.badCsum - stBad_),
                          (unsigned long)(s.drops - stDrops_), (unsigned long)s.winCmdMaxMs);
    }
    stSlots_ = s.slots;
    stMiss_ = s.txMiss;
    stBad_ = s.badCsum;
    stDrops_ = s.drops;
    winResetReq_ = winResetReq_ + 1;   // the task restarts the window maxima
}

// ============================================================================
// TASK SIDE (`can_io` only: no logging, no heap, no NVS, no blocking)
// ============================================================================

bool IboosterBrake::onFrame(const CanFrame& f, uint32_t nowMs) {
    // readMsgBuf() sets bit 31 (extended) / bit 30 (remote): only standard data frames match.
    const uint32_t id = f.id;
    if (id != IB_ID_EXT_STATUS && id != IB_ID_RUNOUT && id != IB_ID_STATUS) {
        return false;
    }
    const uint8_t len = f.len > 8 ? 8 : f.len;
    if (!ibChecksumOk((uint16_t)id, f.data, len)) {
        tBad_++;
        return true;   // counted and discarded, never decoded
    }
    // A frame drained long after the last slot may have sat in the chip through a stall (flash
    // erase): its arrival time is unknown, so it never counts as fresh (kept for the journal only).
    const bool trusted = lastSlotMs_ != 0 && (uint32_t)(nowMs - lastSlotMs_) <= IB_RX_TRUST_MS;
    if (id == IB_ID_STATUS) {
        memcpy(raw39D_, f.data, len < 5 ? len : 5);
    } else if (id == IB_ID_RUNOUT) {
        memcpy(raw38F_, f.data, len < 6 ? len : 6);
    } else {
        memcpy(raw38E_, f.data, len < 6 ? len : 6);
    }
    if (!trusted) {
        tLate_++;
        return true;
    }
    if (id == IB_ID_STATUS) {
        IbStatus s;
        if (!ibDecode39D(f.data, len, s)) return true;   // short frame
        if (!ibRodPlausible(ibRodCentiMm(s.rodRaw))) {   // checksum-valid but impossible: discarded
            tBad_++;
            return true;
        }
        unit_.have39D = true;
        unit_.t39D = nowMs;
        unit_.st = s.status;
        unit_.is = s.istate;
        unit_.drv = s.drv;
        unit_.rodCmm = ibRodCentiMm(s.rodRaw);
    } else if (id == IB_ID_RUNOUT) {
        IbRunout r;
        if (!ibDecode38F(f.data, len, r)) return true;
        unit_.have38F = true;
        unit_.t38F = nowMs;
        unit_.runoutBar = r.runoutBar;
        unit_.partner = r.partner;
    } else {
        IbExtStatus e;
        if (!ibDecode38E(f.data, len, e)) return true;
        have38E_ = true;
        extStatus_ = e.extStatus;
        aCmm_ = e.aCmm;
        bCmm_ = e.bCmm;
    }
    unit_.heard = true;
    unit_.tAny = nowMs;
    return true;
}

void IboosterBrake::taskPushEvent(uint8_t kind, const IbStepOut& o, const IbCommand& c, uint32_t valMs,
                                  const IbEpisodeSummary* ep) {
    IbEvent e;
    e.ms = lastSlotMs_;
    e.kind = kind;
    e.from = (uint8_t)o.evFrom;
    e.to = (uint8_t)o.evTo;
    e.st = unit_.have39D ? unit_.st : 0xFF;   // 0xFF = never heard (rod printed "-")
    e.is = unit_.is;
    e.drv = unit_.drv;
    // What we were commanding: the slot before a state change, this slot otherwise
    e.req = (kind == EV_STATE) ? prevReq_ : o.req;
    e.flowWord = (kind == EV_STATE) ? prevFlowWord_ : o.flowWord;
    e.crank = o.crank;
    e.ign = c.ignOn;
    e.rodCmm = unit_.rodCmm;
    e.tgtCmm = o.targetCmm;
    e.valMs = valMs;
    memcpy(e.raw39D, raw39D_, sizeof(e.raw39D));
    memcpy(e.raw38E, raw38E_, sizeof(e.raw38E));
    memcpy(e.raw38F, raw38F_, sizeof(e.raw38F));
    e.epRodValid = ep != nullptr && ep->rodValid;
    e.epRunValid = ep != nullptr && ep->runoutValid;
    e.epStop = ep != nullptr && ep->stop;
    e.epSpdMin = ep != nullptr ? ep->spdMin : 0;
    e.epRunMin = ep != nullptr ? ep->runoutMin : 0;
    e.epRunMax = ep != nullptr ? ep->runoutMax : 0;
    e.epRodMaxCmm = ep != nullptr ? ep->rodMaxCmm : 0;
    portENTER_CRITICAL(&evMux_);
    const bool dropped = ev_.pushDropOldest(e);
    portEXIT_CRITICAL(&evMux_);
    if (dropped) tEvDrops_++;
}

void IboosterBrake::onSlot(const CanSlotInfo& info, CanSlotTx& out) {
    out.n = 0;
    const uint32_t now = info.nowMs;
    lastSlotMs_ = now ? now : 1;
    if (winResetReq_ != winResetAck_) {
        winResetAck_ = winResetReq_;
        tWinGapMax_ = 0;
        tWinCmdMax_ = 0;
    }

    IbCommand c;
    portENTER_CRITICAL(&cmdMux_);
    c = cmd_;
    portEXIT_CRITICAL(&cmdMux_);

    const IbState prevState = law_.state;
    const IbStepOut o = ibStep(law_, c, unit_, now);
    if (o.cmdAgeMs > tCmdMax_) tCmdMax_ = o.cmdAgeMs;
    if (o.cmdAgeMs > tWinCmdMax_) tWinCmdMax_ = o.cmdAgeMs;

    if (o.ev & IB_EV_STATE) {
        if (o.evTo == IbState::FAULT) tFaults_++;
        taskPushEvent(EV_STATE, o, c, o.evTo == IbState::LOST ? o.evMs : 0);
    }
    if (o.ev & IB_EV_UNIT_DROP) {
        tDrops_++;
        taskPushEvent(EV_UNIT_DROP, o, c, o.evMs);
    }
    if (o.ev & IB_EV_RELEASE_SLOW) taskPushEvent(EV_RELEASE_SLOW, o, c, 0);
    if (o.ev & IB_EV_FAULT_LATCHED) taskPushEvent(EV_FAULT_LATCHED, o, c, 0);
    if (o.ev & IB_EV_START_TIMEOUT) taskPushEvent(EV_START_TIMEOUT, o, c, 0);
    if (o.ev & IB_EV_NO_UNIT) taskPushEvent(EV_NO_UNIT, o, c, 0);
    if (o.ev & IB_EV_CMD_STALE) taskPushEvent(EV_CMD_STALE, o, c, o.evStaleMs);
    if (o.ev & IB_EV_NO_MOVE) taskPushEvent(EV_NO_MOVE, o, c, 0);

    // Brake episode summary (design D2): after ibStep() and after this slot's state event, so an
    // episode ended by LOST / FAULT / STARTING / ASLEEP is journaled after its state line.
    {
        const bool new38F = unit_.have38F && (!epHave38F_ || unit_.t38F != epT38F_);
        epHave38F_ = unit_.have38F;
        epT38F_ = unit_.t38F;
        const uint32_t age39 = unit_.have39D ? (now - unit_.t39D) : 0xFFFFFFFFUL;
        const bool fresh39 = unit_.have39D && age39 <= IB_LAW_STATUS_STALE_MS;
        const bool rodFresh = unit_.have39D && age39 <= IB_LAW_ROD_HOLD_MS;
        IbEpisodeSummary sum;
        if (ibEpisodeStep(episode_, prevState, o.state, o.req, fresh39, unit_.is, new38F, unit_.runoutBar,
                          c.speedByte, rodFresh, unit_.rodCmm, now, sum)) {
            taskPushEvent(EV_EPISODE, o, c, sum.ms, &sum);
        }
    }

    if (o.txEnabled && info.canSend) {
        // Gap between two sent slots (flash erase, task starvation, a CAN command). A late slot is
        // sent once (the CAN task restarted its clock): nothing is made up.
        if (haveLastSent_) {
            const uint32_t gap = now - lastSentMs_;
            if (gap > tGapMax_) tGapMax_ = gap;
            if (gap > tWinGapMax_) tWinGapMax_ = gap;
            if (gap > IB_GAP_WARN_MS) taskPushEvent(EV_TX_GAP, o, c, gap);
        }
        haveLastSent_ = true;
        lastSentMs_ = now;

        // Bench-exact frames: request bits 0x40 only in BRAKING / RELEASING, never 0x80; flow
        // 0x7E00 without a request, clamped to 0x6A00..0x9200; pMC1 117 / qualifier 1.
        CanFrame& d = out.frame[0];
        CanFrame& b = out.frame[2];
        CanFrame& f = out.frame[1];
        d = {};
        b = {};
        f = {};
        ibBuild(counter_, o.req, o.flowWord, c.speedByte, b.data, f.data, d.data);
        d.id = IB_ID_VEHICLE_ALIVE;   // -> TXB2 (TXP 3): watched by the unit even at idle
        d.len = 7;
        f.id = IB_ID_BRAKE_REQUEST;   // -> TXB1 (TXP 2): the flow
        f.len = 4;
        b.id = IB_ID_VEHICLE_STATUS;  // -> TXB1 once free: the most tolerant one
        b.len = 4;
        out.n = 3;
        counter_ = (uint8_t)((counter_ + 1) & 0x0F);
        tSlots_++;
    } else if (!o.txEnabled) {
        haveLastSent_ = false;   // frames stopped on purpose (asleep): no gap to measure
    }
    prevReq_ = o.req && o.txEnabled;
    prevFlowWord_ = o.flowWord;
    holdBus_ = ibRequestState(o.state);

    IbSnapshot s;
    s.state = o.state;
    s.have39D = unit_.have39D;
    s.st = unit_.st;
    s.is = unit_.is;
    s.drv = unit_.drv;
    s.rodCmm = unit_.rodCmm;
    s.have38E = have38E_;
    s.extStatus = extStatus_;
    s.aCmm = aCmm_;
    s.bCmm = bCmm_;
    s.have38F = unit_.have38F;
    s.runoutBar = unit_.runoutBar;
    s.partner = unit_.partner;
    s.age39DMs = unit_.have39D ? now - unit_.t39D : 0xFFFFFFFFUL;
    s.age38FMs = unit_.have38F ? now - unit_.t38F : 0xFFFFFFFFUL;
    s.req = o.req;
    s.flowWord = o.flowWord;
    s.targetCmm = o.targetCmm;
    s.reached = o.reached;
    s.noMove = o.noMove;
    s.cmdStale = o.cmdStale;
    s.crank = o.crank;
    memcpy(s.raw39D, raw39D_, sizeof(s.raw39D));
    memcpy(s.raw38E, raw38E_, sizeof(s.raw38E));
    memcpy(s.raw38F, raw38F_, sizeof(s.raw38F));
    s.slots = tSlots_;
    s.txMiss = info.txMiss;
    s.badCsum = tBad_;
    s.lateFrames = tLate_;
    s.drops = tDrops_;
    s.faults = tFaults_;
    s.gapMaxMs = tGapMax_;
    s.winGapMaxMs = tWinGapMax_;
    s.cmdMaxMs = tCmdMax_;
    s.winCmdMaxMs = tWinCmdMax_;
    s.evDrops = tEvDrops_;
    s.publishedMs = now;
    portENTER_CRITICAL(&snapMux_);
    snap_ = s;
    portEXIT_CRITICAL(&snapMux_);
}
