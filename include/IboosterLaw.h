#ifndef IBOOSTER_LAW_H
#define IBOOSTER_LAW_H

/**
 * @file IboosterLaw.h
 * @brief Pure logic of the aztec iBooster service brake (header-only, <stdint.h> only, no Arduino,
 *        no Constants.h — the IB_LAW_* literals are mirrored by static_assert against Constants.h in
 *        src/IboosterBrake.cpp). Native-tested in test/test_ibooster_law.
 *        Change replace-aztec-brake-with-ibooster, design D4 (state machine), D5 (rod law),
 *        D6 (settings), D7 (stale command), D8 (reached), D10 (STATUSTEXT decision).
 *
 * Units: rod and targets in 0.01 mm ("cmm", int), flow in 0x38C units (128 per ml/s, signed,
 * relative to 0x7E00), times in ms from millis() (all differences unsigned, wrap-safe).
 *
 * ibStep() is called by the CAN task once per 10 ms slot with the newest command from the loop
 * and the newest decoded unit frames; it returns the state, whether frames are sent, the request
 * bit, the flow word and the events of this slot. One state transition per step at most.
 */

#include <stdint.h>

// ---- Constants (mirrored in Constants.h as IB_*, checked by static_assert) -----------------------
#define IB_LAW_SILENT_MS            500     // ignition OFF + no unit frame this long = asleep, frames stop
#define IB_LAW_STATUS_STALE_MS      250     // 0x39D older than this = not fresh (six 40 ms frames)
#define IB_LAW_ROD_HOLD_MS          100     // rod sample older than this: flow 0 (hold) instead of P on an old rod
#define IB_LAW_START_TIMEOUT_MS     20000   // STARTING this long = start_timeout (bench reboot -> ready ~12 s)
#define IB_LAW_NO_UNIT_MS           20000   // LOST this long = no_unit
#define IB_LAW_FAULT_PERSIST_MS     20000   // FAULT this long = fault_latched (longer than the 12 s self-reboot)
#define IB_LAW_ACCEPT_TIMEOUT_MS    1000    // request set, internal state != 3 this long = unit_drop (bench 260-290 ms)
#define IB_LAW_CRANK_GRACE_MS       3000    // after CRANKING: LOST / FAULT journaled W crank=1
#define IB_LAW_CMD_STALE_MS         500     // loop command older than this = stale (held, flagged)
#define IB_LAW_ROD_KP               250     // flow units per mm of rod error (bench, ~1.95 ml/s per mm)
#define IB_LAW_DEADBAND_CMM         5       // |error| <= 0.05 mm -> flow 0 (bench deadband 3 raw)
#define IB_LAW_REACHED_TOL_CMM      20      // reached: |rod - target| <= 0.2 mm
#define IB_LAW_STALL_CMM            5       // reached by stall: < 0.05 mm progress ...
#define IB_LAW_STALL_MS             300     // ... over 300 ms with a positive flow
#define IB_LAW_RELEASE_HANDOFF_CMM  100     // RELEASING: clear the request at rod <= 1.0 mm
#define IB_LAW_RELEASE_TIMEOUT_MS   3000    // = IB_LAW_RELEASE_MIN_MS (kept for the Constants.h mirror)
#define IB_LAW_RELEASE_MIN_MS       3000    // release timeout floor
#define IB_LAW_RELEASE_MAX_MS       30000   // release hard cap (also: rod unknown at RELEASING entry)
#define IB_LAW_RELEASE_SLACK_MS     1000    // timeout = slack + 2 x nominal return time
#define IB_LAW_ROD_CMM_PER_S_PER_MLPS 190   // rod speed per ml/s of flow (bench 1.9-2.0 mm/s)
#define IB_LAW_RELEASE_PROG_CMM     10      // RELEASING "still responding": >= 0.1 mm down ...
#define IB_LAW_RELEASE_PROG_MS      1000    // ... within the last 1 s
#define IB_LAW_ARM_CMM              30      // stall / pRunout "reached" only after the rod moved 0.3 mm
#define IB_LAW_NO_MOVE_MS           3000    // accepted, positive flow, rod never moved 0.3 mm: no_move
#define IB_LAW_ROD_MIN_CMM          -100    // plausible 0x39D rod: -1.00 mm ...
#define IB_LAW_ROD_MAX_CMM          4000    // ... 40.00 mm (mechanical full stroke ~38 mm, bench)
#define IB_LAW_ROD_CEIL_CMM         1600    // hard ceiling of the rod target, 16.00 mm (stop 16.62-16.64 mm dry)
#define IB_LAW_MAX_MM10_MIN         20      // max stroke setting 2.0 mm
#define IB_LAW_MAX_MM10_MAX         160     // ... 16.0 mm
#define IB_LAW_MAX_MM10_DEFAULT     160     // 16.0 mm = full external-request stroke (owner 2026-11-23)
#define IB_LAW_RATE_MIN_MLPS        1       // apply / release rate setting, ml/s
#define IB_LAW_RATE_MAX_MLPS        40      // = the unit's usable flow 0x9200 / 0x6A00
#define IB_LAW_RATE_DEFAULT_MLPS    8       // owner 2026-11-23 (~15-16 mm/s)
#define IB_LAW_UNITS_PER_MLPS       128     // 0x38C: 0.0078125 ml/s per unit
#define IB_LAW_FLOW_ZERO            0x7E00
#define IB_LAW_FLOW_MIN             0x6A00
#define IB_LAW_FLOW_MAX             0x9200
#define IB_LAW_STATUSTEXT_MIN_MS    5000    // at most one brake STATUSTEXT per 5 s
#define IB_LAW_SPEED_KMH_PER_BIT    0.4f    // 0x38B byte 2 (Rivian DBC scale, unverified)
#define IB_LAW_SPEED_FLOOR_KMH      2.0f    // 0x38B byte 2 never below 2 km/h = 5 counts (bench: pRunout 103 from 3 counts)
                                            // (change add-ibooster-speed-floor, design D1)

static_assert(IB_LAW_RATE_MAX_MLPS * IB_LAW_UNITS_PER_MLPS == IB_LAW_FLOW_MAX - IB_LAW_FLOW_ZERO,
              "40 ml/s must be the top of the flow range");
static_assert(IB_LAW_FLOW_ZERO - IB_LAW_RATE_MAX_MLPS * IB_LAW_UNITS_PER_MLPS == IB_LAW_FLOW_MIN,
              "-40 ml/s must be the bottom of the flow range");
static_assert(IB_LAW_MAX_MM10_MAX * 10 == IB_LAW_ROD_CEIL_CMM, "max stroke range ends at the ceiling");

enum class IbState : uint8_t { ASLEEP, STARTING, READY, BRAKING, RELEASING, LOST, FAULT };

/** Journal token (upper case, as in `BRAKE I state from=STARTING to=READY`). */
inline const char* ibStateName(IbState s) {
  switch (s) {
    case IbState::ASLEEP:    return "ASLEEP";
    case IbState::STARTING:  return "STARTING";
    case IbState::READY:     return "READY";
    case IbState::BRAKING:   return "BRAKING";
    case IbState::RELEASING: return "RELEASING";
    case IbState::LOST:      return "LOST";
    case IbState::FAULT:     return "FAULT";
  }
  return "?";
}

/** Telemetry token (brake_state). */
inline const char* ibStateToken(IbState s) {
  switch (s) {
    case IbState::ASLEEP:    return "asleep";
    case IbState::STARTING:  return "starting";
    case IbState::READY:     return "ready";
    case IbState::BRAKING:   return "braking";
    case IbState::RELEASING: return "releasing";
    case IbState::LOST:      return "lost";
    case IbState::FAULT:     return "fault";
  }
  return "?";
}

/** READY / BRAKING / RELEASING. */
inline bool ibAvailable(IbState s) {
  return s == IbState::READY || s == IbState::BRAKING || s == IbState::RELEASING;
}
/** BRAKING / RELEASING: the external request is set. */
inline bool ibRequestState(IbState s) {
  return s == IbState::BRAKING || s == IbState::RELEASING;
}

// ---- Settings and conversions (design D5, D6) ----------------------------------------------------

inline bool ibMaxMm10Valid(int32_t v) { return v >= IB_LAW_MAX_MM10_MIN && v <= IB_LAW_MAX_MM10_MAX; }
/** The web slider step is 0.5 mm: a value must also sit on that grid. */
inline bool ibMaxMm10OnGrid(int32_t v) { return ibMaxMm10Valid(v) && (v % 5) == 0; }
inline bool ibRateValid(int32_t v) { return v >= IB_LAW_RATE_MIN_MLPS && v <= IB_LAW_RATE_MAX_MLPS; }
inline int32_t ibMaxMm10OrDefault(int32_t v) { return ibMaxMm10Valid(v) ? v : IB_LAW_MAX_MM10_DEFAULT; }
inline int32_t ibRateOrDefault(int32_t v) { return ibRateValid(v) ? v : IB_LAW_RATE_DEFAULT_MLPS; }

/** A checksum-valid 0x39D rod the unit can actually report (-1..40 mm); anything else is discarded. */
inline bool ibRodPlausible(int32_t rodCmm) {
  return rodCmm >= IB_LAW_ROD_MIN_CMM && rodCmm <= IB_LAW_ROD_MAX_CMM;
}

/** Flow cap in 0x38C units for a rate in ml/s, clamped to 1..40 ml/s (128..5120 units). */
inline uint16_t ibRateUnits(int32_t mlps) {
  if (mlps < IB_LAW_RATE_MIN_MLPS) mlps = IB_LAW_RATE_MIN_MLPS;
  if (mlps > IB_LAW_RATE_MAX_MLPS) mlps = IB_LAW_RATE_MAX_MLPS;
  return (uint16_t)(mlps * IB_LAW_UNITS_PER_MLPS);
}

/** Any flow cap (e.g. from a command record) into 128..5120 units. */
inline int32_t ibClampCapUnits(int32_t units) {
  const int32_t lo = IB_LAW_RATE_MIN_MLPS * IB_LAW_UNITS_PER_MLPS;
  const int32_t hi = IB_LAW_RATE_MAX_MLPS * IB_LAW_UNITS_PER_MLPS;
  return units < lo ? lo : (units > hi ? hi : units);
}

/**
 * Rod target for a brake percentage: pct / 100 x max stroke, in 0.01 mm. NaN, negative -> 0;
 * above 100 % -> 100 %; the stroke is clamped to 2.0..16.0 mm whatever the stored value, so the
 * result never exceeds IB_LAW_ROD_CEIL_CMM.
 */
inline uint16_t ibTargetCmm(float pct, int32_t maxMm10) {
  if (!(pct > 0.0f)) return 0;   // also NaN
  if (pct > 100.0f) pct = 100.0f;
  if (maxMm10 < IB_LAW_MAX_MM10_MIN) maxMm10 = IB_LAW_MAX_MM10_MIN;
  if (maxMm10 > IB_LAW_MAX_MM10_MAX) maxMm10 = IB_LAW_MAX_MM10_MAX;
  int32_t cmm = (int32_t)(pct * (float)(maxMm10 * 10) / 100.0f + 0.5f);
  if (cmm < 0) cmm = 0;
  if (cmm > IB_LAW_ROD_CEIL_CMM) cmm = IB_LAW_ROD_CEIL_CMM;
  return (uint16_t)cmm;
}

/**
 * P law on the rod: flow units = IB_LAW_ROD_KP x error (mm), 0 inside the deadband, limited to
 * +apply cap / -release cap (each re-clamped to 1..40 ml/s). The target is re-clamped to the
 * ceiling here too.
 */
inline int32_t ibFlowUnits(int32_t targetCmm, int32_t rodCmm, int32_t applyUnits, int32_t releaseUnits) {
  if (targetCmm < 0) targetCmm = 0;
  if (targetCmm > IB_LAW_ROD_CEIL_CMM) targetCmm = IB_LAW_ROD_CEIL_CMM;
  const int32_t err = targetCmm - rodCmm;
  if (err <= IB_LAW_DEADBAND_CMM && err >= -IB_LAW_DEADBAND_CMM) return 0;
  int32_t f = (IB_LAW_ROD_KP * err) / 100;
  const int32_t up = ibClampCapUnits(applyUnits);
  const int32_t down = ibClampCapUnits(releaseUnits);
  if (f > up) f = up;
  if (f < -down) f = -down;
  return f;
}

/**
 * RELEASING timeout for a release that starts at rodCmm (negative = unknown) with this release cap:
 * slack + 2 x the nominal return time to the 1 mm hand-off at ~1.9 mm/s per ml/s, clamped to
 * IB_LAW_RELEASE_MIN_MS..IB_LAW_RELEASE_MAX_MS (unknown rod: the maximum).
 */
inline uint32_t ibReleaseTimeoutMs(int32_t rodCmm, int32_t releaseUnits) {
  if (rodCmm < 0) return IB_LAW_RELEASE_MAX_MS;
  int32_t dist = rodCmm - IB_LAW_RELEASE_HANDOFF_CMM;
  if (dist < 0) dist = 0;
  const int32_t units = ibClampCapUnits(releaseUnits);
  // ms = dist[cmm] / (units / 128 x 190 cmm/s) x 1000
  const uint32_t nominal = (uint32_t)(((int64_t)dist * 1000 * IB_LAW_UNITS_PER_MLPS) /
                                      ((int64_t)units * IB_LAW_ROD_CMM_PER_S_PER_MLPS));
  uint32_t t = IB_LAW_RELEASE_SLACK_MS + 2 * nominal;
  if (t < IB_LAW_RELEASE_MIN_MS) t = IB_LAW_RELEASE_MIN_MS;
  if (t > IB_LAW_RELEASE_MAX_MS) t = IB_LAW_RELEASE_MAX_MS;
  return t;
}

/** 0x38C flow word for a signed flow, always within 0x6A00..0x9200. */
inline uint16_t ibFlowWord(int32_t flowUnits) {
  int32_t w = IB_LAW_FLOW_ZERO + flowUnits;
  if (w < IB_LAW_FLOW_MIN) w = IB_LAW_FLOW_MIN;
  if (w > IB_LAW_FLOW_MAX) w = IB_LAW_FLOW_MAX;
  return (uint16_t)w;
}

/**
 * 0x38B byte 2: round(max(km/h, IB_LAW_SPEED_FLOOR_KMH) / 0.4), clamped to 0..255 (change
 * add-ibooster-speed-floor, design D1). An invalid, NaN or non-positive wheel speed gives the floor
 * (5 counts), never 0 and never "faulty". The floored value exists only in this byte (IbCommand
 * speedByte and the frames): the rest of the firmware keeps the true speed.
 */
inline uint8_t ibSpeedByte(float speedMs, bool valid) {
  float kmh = (valid && speedMs > 0.0f) ? speedMs * 3.6f : 0.0f;   // NaN -> 0
  if (!(kmh >= IB_LAW_SPEED_FLOOR_KMH)) kmh = IB_LAW_SPEED_FLOOR_KMH;
  const float counts = kmh / IB_LAW_SPEED_KMH_PER_BIT + 0.5f;
  if (counts >= 255.0f) return 255;
  return (uint8_t)counts;
}

/** Frames go out while the ignition is on or a valid unit frame arrived < 500 ms ago. */
inline bool ibTxEnabled(bool ignOn, bool heardEver, uint32_t msSinceUnitFrame) {
  return ignOn || (heardEver && msSinceUnitFrame < IB_LAW_SILENT_MS);
}

// ---- Records ---------------------------------------------------------------------------------------

/** Loop -> task, every loop (16 bytes). stampMs 0 = no command yet. */
struct IbCommand {
  uint32_t stampMs;       // millis() of publication (never 0 once published)
  uint16_t targetCmm;     // rod target, 0.01 mm (0 = released)
  uint16_t applyUnits;    // apply flow cap, 0x38C units
  uint16_t releaseUnits;  // release flow cap, 0x38C units
  uint8_t speedByte;      // 0x38B byte 2
  bool ignOn;             // ignition not OFF
  bool cranking;          // ignition CRANKING
  uint8_t pad_[3];
};

/** The task's newest decoded unit frames (written by the task only). Times = arrival millis(). */
struct IbUnitView {
  bool have39D;
  uint32_t t39D;
  uint8_t st, is, drv;
  int16_t rodCmm;
  bool have38F;
  uint32_t t38F;
  uint8_t runoutBar;
  int8_t partner;          // 1 sees us, 0 does not, -1 other
  bool heard;              // any valid unit frame ever
  uint32_t tAny;           // last valid unit frame of any of the three IDs
};

/** Law state kept between steps (task-owned). */
struct IbLaw {
  IbState state;
  uint32_t enteredMs;      // entry time of `state`
  bool timeoutLogged;      // start_timeout / no_unit / fault_latched given for this episode
  bool acceptTracking;     // request set, waiting for internal state 3
  uint32_t acceptSinceMs;
  bool dropReported;       // unit_drop given for this non-accepted episode
  bool stallTracking;
  int16_t stallRefCmm;
  uint32_t stallRefMs;
  bool crankSeen;
  uint32_t lastCrankMs;
  bool staleActive;
  uint32_t staleMaxMs;
  // BRAKING episode: has the rod moved IB_LAW_ARM_CMM since entry (take-up over)?
  bool armRefValid;
  int16_t armRefCmm;
  bool armed;
  bool noMoveTracking;
  uint32_t noMoveSinceMs;
  bool noMoveLogged;
  // RELEASING: timeout from the entry rod and rate, progress watch
  uint32_t relTimeoutMs;
  bool relProgValid;
  int16_t relRefCmm;
  uint32_t relRefMs;
  // Ignition edge (OFF -> ON re-enters STARTING)
  bool ignSeen;
  bool prevIgnOn;
  bool wakeValid;          // an ignition OFF -> ON edge was seen at wakeMs:
  uint32_t wakeMs;         // the READY condition then needs unit frames newer than the edge
};

inline IbLaw ibLawInitial(uint32_t nowMs) {
  IbLaw s = {};
  s.state = IbState::ASLEEP;
  s.enteredMs = nowMs;
  return s;
}

// Event bits of one step
#define IB_EV_STATE          0x01   // evFrom -> evTo
#define IB_EV_UNIT_DROP      0x02   // evMs = time the request has not been accepted
#define IB_EV_RELEASE_SLOW   0x04
#define IB_EV_FAULT_LATCHED  0x08
#define IB_EV_START_TIMEOUT  0x10
#define IB_EV_NO_UNIT        0x20
#define IB_EV_CMD_STALE      0x40   // a stale-command episode ended; evStaleMs = longest command age
#define IB_EV_NO_MOVE        0x80   // accepted, positive flow, rod not moved 0.3 mm for 3 s (once per episode)

struct IbStepOut {
  IbState state;
  bool txEnabled;      // send the three frames this slot
  bool req;            // request bits (0x40) set
  int32_t flow;        // signed flow units sent (0 without a request)
  uint16_t flowWord;   // 0x38C flow word, 0x6A00..0x9200 (0x7E00 without a request)
  uint16_t targetCmm;  // rod target in use (clamped)
  bool reached;
  bool noMove;         // BRAKING, accepted, positive flow, rod has not moved 0.3 mm for IB_LAW_NO_MOVE_MS
  bool crank;          // inside the crank window (CRANKING or < 3 s after)
  bool cmdStale;
  uint32_t cmdAgeMs;
  uint8_t ev;          // IB_EV_* bits
  IbState evFrom, evTo;
  uint32_t evMs;       // unit_drop: ms not accepted; state -> LOST: 0x39D age
  uint32_t evStaleMs;
};

// ---- Reached (design D8) ----------------------------------------------------------------------------
/**
 * BRAKING, the unit accepted (internal state 3, fresh 0x39D), and the rod is within 0.2 mm of the
 * target, or — only once the rod has moved IB_LAW_ARM_CMM in this BRAKING episode (`armed`: the
 * ~2 mm free-travel take-up after acceptance moves the actuator, not the output rod) — it stalled
 * (positive flow, < 0.05 mm progress over 300 ms) or 0x38F byte 3 is 0 (stroke stop, fresh 0x38F).
 * False in every other state.
 */
inline bool ibReached(IbState state, bool fresh39D, uint8_t is, int32_t rodCmm, int32_t targetCmm,
                      bool stalled, bool fresh38F, uint8_t runoutBar, bool armed = true) {
  if (state != IbState::BRAKING || !fresh39D || is != 3) return false;
  const int32_t err = rodCmm - targetCmm;
  if (err <= IB_LAW_REACHED_TOL_CMM && err >= -IB_LAW_REACHED_TOL_CMM) return true;
  if (!armed) return false;
  if (stalled) return true;
  return fresh38F && runoutBar == 0;
}

// ---- The step -----------------------------------------------------------------------------------------
inline void ibEnter(IbLaw& s, IbStepOut& o, IbState to, uint32_t nowMs) {
  if (s.state == to) return;
  o.ev |= IB_EV_STATE;
  o.evFrom = s.state;
  o.evTo = to;
  s.state = to;
  s.enteredMs = nowMs;
  s.timeoutLogged = false;
}

/**
 * One 10 ms slot (design D4 / D5 / D7). Rules, first match wins, one transition per step:
 *  - frames disabled (ignition OFF and unit silent 500 ms) -> ASLEEP from anywhere;
 *  - ASLEEP -> STARTING;
 *  - fresh 0x39D with status 2 or drv 3 -> FAULT (from STARTING / READY / BRAKING / RELEASING);
 *  - READY / BRAKING / RELEASING: 0x39D older than 250 ms with the ignition on -> LOST (with the
 *    ignition reported OFF the silence is the unit going to sleep: no LOST, ASLEEP follows);
 *  - READY: READY condition gone (not fault) -> STARTING; target > 0 -> BRAKING;
 *  - BRAKING / RELEASING: status other than 5 / 6 (not fault) -> STARTING;
 *  - BRAKING: target 0 -> RELEASING;
 *  - ignition OFF -> ON edge: READY / BRAKING / RELEASING / LOST -> STARTING (the unit lost its Wake);
 *  - RELEASING: target > 0 (fresh 0x39D) -> BRAKING; rod <= 1.0 mm -> READY; past the release
 *    timeout (from the entry rod and the release rate) and no 0.1 mm progress for 1 s, or past
 *    IB_LAW_RELEASE_MAX_MS -> READY + release_slow;
 *  - STARTING / FAULT: READY condition -> READY (FAULT leaves only on the unit's own evidence);
 *  - LOST: fresh 0x39D -> STARTING.
 * READY condition: fresh 0x39D, status 5 or 6, drv != 3, fresh 0x38F with byte 4 = 0x40.
 * A stale command (> 500 ms) is still used as it is (target and speed held) and flagged.
 */
inline IbStepOut ibStep(IbLaw& s, const IbCommand& cmd, const IbUnitView& u, uint32_t nowMs) {
  IbStepOut o = {};
  o.flowWord = IB_LAW_FLOW_ZERO;

  // Command age (stamp 0 = nothing published yet: not stale, all-zero = idle, ignition OFF)
  if (cmd.stampMs != 0) {
    o.cmdAgeMs = nowMs - cmd.stampMs;
    if (o.cmdAgeMs > 0x7FFFFFFFUL) o.cmdAgeMs = 0;   // stamped after this slot's time base
  }
  o.cmdStale = o.cmdAgeMs > IB_LAW_CMD_STALE_MS;
  if (o.cmdStale) {
    s.staleActive = true;
    if (o.cmdAgeMs > s.staleMaxMs) s.staleMaxMs = o.cmdAgeMs;
  } else if (s.staleActive) {
    o.ev |= IB_EV_CMD_STALE;
    o.evStaleMs = s.staleMaxMs;
    s.staleActive = false;
    s.staleMaxMs = 0;
  }

  // Crank window
  if (cmd.cranking) {
    s.crankSeen = true;
    s.lastCrankMs = nowMs;
  }
  o.crank = cmd.cranking || (s.crankSeen && (nowMs - s.lastCrankMs) < IB_LAW_CRANK_GRACE_MS);

  int32_t target = cmd.targetCmm;
  if (target > IB_LAW_ROD_CEIL_CMM) target = IB_LAW_ROD_CEIL_CMM;
  o.targetCmm = (uint16_t)target;

  const uint32_t age39 = u.have39D ? (nowMs - u.t39D) : 0xFFFFFFFFUL;
  const uint32_t age38F = u.have38F ? (nowMs - u.t38F) : 0xFFFFFFFFUL;
  const bool fresh39 = u.have39D && age39 <= IB_LAW_STATUS_STALE_MS;
  const bool fresh38F = u.have38F && age38F <= IB_LAW_STATUS_STALE_MS;
  const bool rodFresh = u.have39D && age39 <= IB_LAW_ROD_HOLD_MS;
  const bool fault = fresh39 && (u.st == 2 || u.drv == 3);
  const bool statusReady = fresh39 && (u.st == 5 || u.st == 6) && u.drv != 3;
  o.txEnabled = ibTxEnabled(cmd.ignOn, u.heard, u.heard ? (nowMs - u.tAny) : 0xFFFFFFFFUL);
  // Ignition OFF -> ON: the unit lost its Wake meanwhile (asleep after a short blip, or rebooting)
  const bool ignRise = s.ignSeen && cmd.ignOn && !s.prevIgnOn;
  s.ignSeen = true;
  s.prevIgnOn = cmd.ignOn;
  if (ignRise) {
    s.wakeValid = true;
    s.wakeMs = nowMs;
  }
  // After the edge, frames from before it say nothing about the unit now
  const bool sinceWake = !s.wakeValid || ((int32_t)(u.t39D - s.wakeMs) >= 0 && (int32_t)(u.t38F - s.wakeMs) >= 0);
  const bool readyCond = statusReady && fresh38F && u.partner == 1 && sinceWake;

  // RELEASING progress watch (rod going down)
  if (s.state == IbState::RELEASING && rodFresh) {
    if (!s.relProgValid || (int32_t)s.relRefCmm - (int32_t)u.rodCmm >= IB_LAW_RELEASE_PROG_CMM) {
      s.relProgValid = true;
      s.relRefCmm = u.rodCmm;
      s.relRefMs = nowMs;
    }
  }

  const IbState prev = s.state;
  if (!o.txEnabled) {
    ibEnter(s, o, IbState::ASLEEP, nowMs);
  } else {
    switch (prev) {
      case IbState::ASLEEP:
        ibEnter(s, o, IbState::STARTING, nowMs);
        break;
      case IbState::STARTING:
        if (fault) ibEnter(s, o, IbState::FAULT, nowMs);
        else if (readyCond) ibEnter(s, o, IbState::READY, nowMs);
        break;
      case IbState::READY:
        if (fault) ibEnter(s, o, IbState::FAULT, nowMs);
        else if (ignRise) ibEnter(s, o, IbState::STARTING, nowMs);
        else if (!fresh39 && cmd.ignOn) ibEnter(s, o, IbState::LOST, nowMs);
        else if (fresh39 && !readyCond) ibEnter(s, o, IbState::STARTING, nowMs);
        else if (fresh39 && target > 0) ibEnter(s, o, IbState::BRAKING, nowMs);
        break;
      case IbState::BRAKING:
      case IbState::RELEASING:
        if (fault) ibEnter(s, o, IbState::FAULT, nowMs);
        else if (ignRise) ibEnter(s, o, IbState::STARTING, nowMs);
        else if (!fresh39 && cmd.ignOn) ibEnter(s, o, IbState::LOST, nowMs);
        else if (fresh39 && !statusReady) ibEnter(s, o, IbState::STARTING, nowMs);
        else if (prev == IbState::BRAKING) {
          if (target == 0) {
            ibEnter(s, o, IbState::RELEASING, nowMs);
            s.relTimeoutMs = ibReleaseTimeoutMs(rodFresh ? u.rodCmm : -1, cmd.releaseUnits);
            s.relProgValid = false;
          }
        } else {
          const uint32_t in = nowMs - s.enteredMs;
          // Still responding = the rod went down 0.1 mm within the last second
          const bool progressing = s.relProgValid && (nowMs - s.relRefMs) < IB_LAW_RELEASE_PROG_MS;
          if (target > 0 && fresh39) {
            ibEnter(s, o, IbState::BRAKING, nowMs);   // a re-press needs a fresh status
          } else if (fresh39 && u.rodCmm <= IB_LAW_RELEASE_HANDOFF_CMM) {
            ibEnter(s, o, IbState::READY, nowMs);
          } else if ((in >= s.relTimeoutMs && !progressing) || in >= IB_LAW_RELEASE_MAX_MS) {
            // The rod stopped responding (or the hard cap): clear the request; the unit returns
            // the rod by itself. Never while it is still coming down within its time budget.
            o.ev |= IB_EV_RELEASE_SLOW;
            ibEnter(s, o, IbState::READY, nowMs);
          }
        }
        break;
      case IbState::LOST:
        if (fresh39 || ignRise) ibEnter(s, o, IbState::STARTING, nowMs);
        break;
      case IbState::FAULT:
        if (readyCond) ibEnter(s, o, IbState::READY, nowMs);
        break;
    }
  }
  if ((o.ev & IB_EV_STATE) && o.evTo == IbState::LOST) {
    o.evMs = age39;
  }

  // Long-lasting trouble, once per episode
  if (!s.timeoutLogged) {
    const uint32_t in = nowMs - s.enteredMs;
    if (s.state == IbState::STARTING && in >= IB_LAW_START_TIMEOUT_MS) {
      o.ev |= IB_EV_START_TIMEOUT;
      s.timeoutLogged = true;
    } else if (s.state == IbState::LOST && in >= IB_LAW_NO_UNIT_MS) {
      o.ev |= IB_EV_NO_UNIT;
      s.timeoutLogged = true;
    } else if (s.state == IbState::FAULT && in >= IB_LAW_FAULT_PERSIST_MS) {
      o.ev |= IB_EV_FAULT_LATCHED;
      s.timeoutLogged = true;
    }
  }

  o.state = s.state;
  o.req = o.txEnabled && ibRequestState(s.state);
  if (o.req) {
    // RELEASING returns the rod toward 0 under P with the release cap until the hand-off at 1 mm.
    const int32_t tgt = (s.state == IbState::BRAKING) ? target : 0;
    // P only on a recent rod sample; an older one (frames lost, < 250 ms) holds the rod (flow 0).
    o.flow = rodFresh ? ibFlowUnits(tgt, u.rodCmm, cmd.applyUnits, cmd.releaseUnits) : 0;
    o.flowWord = ibFlowWord(o.flow);
  }

  // Accept watch: request set but internal state != 3 for 1 s -> unit_drop (once per episode),
  // the request is kept (the unit re-accepts by itself after a gap).
  if (o.req) {
    if (fresh39 && u.is == 3) {
      s.acceptTracking = false;
      s.dropReported = false;
    } else {
      if (!s.acceptTracking) {
        s.acceptTracking = true;
        s.acceptSinceMs = nowMs;
      }
      const uint32_t waited = nowMs - s.acceptSinceMs;
      if (waited >= IB_LAW_ACCEPT_TIMEOUT_MS && !s.dropReported) {
        o.ev |= IB_EV_UNIT_DROP;
        o.evMs = waited;
        s.dropReported = true;
      }
    }
  } else {
    s.acceptTracking = false;
    s.dropReported = false;
  }

  // Arming (take-up over): the rod moved IB_LAW_ARM_CMM since the first fresh sample of this
  // BRAKING episode. Before that neither a stall nor pRunout 0 counts as reached.
  if (s.state != IbState::BRAKING || ((o.ev & IB_EV_STATE) && o.evTo == IbState::BRAKING)) {
    s.armRefValid = false;
    s.armed = false;
    s.noMoveTracking = false;
    s.noMoveLogged = false;
  }
  if (s.state == IbState::BRAKING && rodFresh) {
    if (!s.armRefValid) {
      s.armRefValid = true;
      s.armRefCmm = u.rodCmm;
    } else if ((int32_t)u.rodCmm - (int32_t)s.armRefCmm >= IB_LAW_ARM_CMM) {
      s.armed = true;
    }
  }
  // No move: accepted with a positive flow and the rod never armed for IB_LAW_NO_MOVE_MS (take-up at
  // 1 ml/s is ~1 s). Reported once per episode; `noMove` holds while it lasts. The request stays.
  if (s.state == IbState::BRAKING && fresh39 && u.is == 3 && o.flow > 0 && !s.armed) {
    if (!s.noMoveTracking) {
      s.noMoveTracking = true;
      s.noMoveSinceMs = nowMs;
    }
    o.noMove = (nowMs - s.noMoveSinceMs) >= IB_LAW_NO_MOVE_MS;
    if (o.noMove && !s.noMoveLogged) {
      o.ev |= IB_EV_NO_MOVE;
      s.noMoveLogged = true;
    }
  } else {
    s.noMoveTracking = false;
  }

  // Stall watch (reached by stall): BRAKING, accepted, positive flow, recent rod
  bool stalled = false;
  if (s.state == IbState::BRAKING && rodFresh && fresh39 && u.is == 3 && o.flow > 0) {
    if (!s.stallTracking || (int32_t)u.rodCmm - (int32_t)s.stallRefCmm >= IB_LAW_STALL_CMM) {
      s.stallTracking = true;
      s.stallRefCmm = u.rodCmm;
      s.stallRefMs = nowMs;
    }
    stalled = (nowMs - s.stallRefMs) >= IB_LAW_STALL_MS;
  } else {
    s.stallTracking = false;
  }
  o.reached = ibReached(s.state, fresh39, u.is, u.rodCmm, target, stalled, fresh38F, u.runoutBar, s.armed);
  return o;
}

// ---- Brake episode summary (change add-ibooster-speed-floor, design D2) ---------------------------
/**
 * One brake request episode: from the slot whose state enters BRAKING from READY to the first slot
 * whose state is neither BRAKING nor RELEASING (RELEASING -> BRAKING continues it). Task-owned.
 */
struct IbEpisode {
  bool active;
  uint32_t startMs;        // slot time of the READY -> BRAKING entry
  bool rodValid;           // a fresh rod was seen
  int16_t rodMaxCmm;       // highest fresh rod, 0.01 mm
  uint8_t spdMin;          // lowest 0x38B byte 2 sent
  bool anyRunout;          // a non-zero pRunout sample was taken
  uint8_t runoutMin, runoutMax;   // over the non-zero samples (bar)
  bool stop;               // a sample was 0 (stroke stop)
};

/** The `BRAKE I episode` line's fields, produced once, on the episode's last slot. */
struct IbEpisodeSummary {
  uint32_t ms;             // episode duration
  bool rodValid;
  int16_t rodMaxCmm;
  uint8_t spdMin;
  bool runoutValid;        // false = `runout=-`
  uint8_t runoutMin, runoutMax;
  bool stop;
};

inline IbEpisode ibEpisodeInitial() {
  IbEpisode e = {};
  return e;
}

/**
 * Called once per slot after ibStep() with the step's previous and new state. Samples: the byte 3
 * of a 0x38F frame newly decoded since the previous slot (`new38F`: checksum-valid, trusted), only
 * while the request is sent (`req`) and a fresh 0x39D reports internal state 3 (accepted) — so the
 * idle 50 / settling 51 before acceptance and the decay after the request is cleared are never
 * sampled. Returns true with `out` filled on the episode's last slot (whatever state ends it).
 */
inline bool ibEpisodeStep(IbEpisode& e, IbState prev, IbState now, bool req, bool fresh39D, uint8_t is,
                          bool new38F, uint8_t runoutBar, uint8_t speedByte, bool rodFresh, int16_t rodCmm,
                          uint32_t nowMs, IbEpisodeSummary& out) {
  if (!e.active) {
    if (prev != IbState::READY || now != IbState::BRAKING) return false;
    e = ibEpisodeInitial();
    e.active = true;
    e.startMs = nowMs;
    e.spdMin = speedByte;
  } else if (!ibRequestState(now)) {
    out.ms = nowMs - e.startMs;   // unsigned: wrap-safe
    out.rodValid = e.rodValid;
    out.rodMaxCmm = e.rodMaxCmm;
    out.spdMin = e.spdMin;
    out.runoutValid = e.anyRunout;
    out.runoutMin = e.runoutMin;
    out.runoutMax = e.runoutMax;
    out.stop = e.stop;
    e.active = false;
    return true;
  }
  if (speedByte < e.spdMin) e.spdMin = speedByte;
  if (rodFresh && (!e.rodValid || rodCmm > e.rodMaxCmm)) {
    e.rodValid = true;
    e.rodMaxCmm = rodCmm;
  }
  if (req && fresh39D && is == 3 && new38F) {
    if (runoutBar == 0) {
      e.stop = true;
    } else if (!e.anyRunout) {
      e.anyRunout = true;
      e.runoutMin = runoutBar;
      e.runoutMax = runoutBar;
    } else {
      if (runoutBar < e.runoutMin) e.runoutMin = runoutBar;
      if (runoutBar > e.runoutMax) e.runoutMax = runoutBar;
    }
  }
  return false;
}

// ---- Brake STATUSTEXT decision (design D10, owner Q4 (a)) -----------------------------------------
enum class IbText : uint8_t { NONE, LOST, FAULT, OK };

struct IbTextState {
  IbText announced;    // last text sent (NONE = none yet)
  IbText pending;      // text waiting for the hold-off / the shared limiter
  IbText lastClass;    // class seen on the previous step
  bool sentOnce;
  uint32_t lastSentMs;
};

inline IbTextState ibTextInitial() { return {IbText::NONE, IbText::NONE, IbText::NONE, false, 0}; }

/** LOST -> LOST, FAULT -> FAULT, READY / BRAKING / RELEASING -> OK, ASLEEP / STARTING -> NONE (silent). */
inline IbText ibTextClass(IbState st) {
  if (st == IbState::LOST) return IbText::LOST;
  if (st == IbState::FAULT) return IbText::FAULT;
  if (ibAvailable(st)) return IbText::OK;
  return IbText::NONE;
}

/**
 * One report: returns the text to send now (NONE = nothing). A class change replaces the pending
 * text (latest state wins); a change back to what was last announced cancels it; OK is only ever
 * pending after an announced LOST / FAULT. At most one text per IB_LAW_STATUSTEXT_MIN_MS, and only
 * when the shared limiter allows (`sharedReady`); a text held back stays pending.
 */
inline IbText ibStatustextStep(IbTextState& s, IbState st, uint32_t nowMs, bool sharedReady) {
  const IbText cls = ibTextClass(st);
  if (cls != IbText::NONE && cls != s.lastClass) {
    IbText want = cls;
    if (want == IbText::OK && s.announced != IbText::LOST && s.announced != IbText::FAULT) {
      want = IbText::NONE;   // recovery without an announced warning: nothing to say
    }
    if (want == s.announced) want = IbText::NONE;   // back to what the GCS already shows
    s.pending = want;
  }
  if (cls != IbText::NONE) s.lastClass = cls;
  if (s.pending == IbText::NONE || !sharedReady) return IbText::NONE;
  if (s.sentOnce && (nowMs - s.lastSentMs) < IB_LAW_STATUSTEXT_MIN_MS) return IbText::NONE;
  const IbText out = s.pending;
  s.announced = out;
  s.pending = IbText::NONE;
  s.sentOnce = true;
  s.lastSentMs = nowMs;
  return out;
}

#endif  // IBOOSTER_LAW_H
