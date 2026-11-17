// ParkBrakeLaw.h — aztec parking brake: state decode, command transitions, run timing,
// apply speed check, drive-away latch and throttle hold (change add-aztec-parking-brake).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit tests
// (test/test_park_brake_law). ParkBrakeController owns the bridge, the run timer and NVS
// "pbrake"; VehicleController owns the policy (speed interlock, drive-away, throttle hold).
// This file is only the arithmetic and the decisions.
// The literals below MUST match include/Constants.h (static_assert in src/ParkBrakeController.cpp):
//   PARK_BRAKE_TRAVEL_MS_MIN 1000, PARK_BRAKE_TRAVEL_MS_MAX 5000, PARK_BRAKE_TRAVEL_MS_DEFAULT 2000,
//   PARK_BRAKE_TRAVEL_MARGIN_MS 500, PARK_BRAKE_REVERSE_DWELL_MS 200,
//   PARK_BRAKE_THR_PASS_PCT_MIN 20, PARK_BRAKE_THR_PASS_PCT_MAX 100, PARK_BRAKE_THR_PASS_PCT_DEFAULT 50,
//   PARK_BRAKE_APPLY_MAX_SPEED_MS 0.833, PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT 10,
//   BRAKE_TOLERANCE 1 (service-brake release hand-over, change add-park-brake-release-by-service-brake).
#pragma once

#include <stdint.h>

static const int32_t PB_LAW_TRAVEL_MS_MIN       = 1000;
static const int32_t PB_LAW_TRAVEL_MS_MAX       = 5000;
static const int32_t PB_LAW_TRAVEL_MS_DEFAULT   = 2000;
static const int32_t PB_LAW_TRAVEL_MARGIN_MS    = 500;
static const int32_t PB_LAW_REVERSE_DWELL_MS    = 200;
static const int32_t PB_LAW_THR_PASS_PCT_MIN    = 20;
static const int32_t PB_LAW_THR_PASS_PCT_MAX    = 100;
static const int32_t PB_LAW_THR_PASS_PCT_DEFAULT = 50;
static constexpr float PB_LAW_APPLY_MAX_SPEED_MS = 0.833f;   // 3 km/h
static constexpr float PB_LAW_DRIVE_AWAY_PCT     = 10.0f;
static constexpr float PB_LAW_SVC_TOL_PCT        = 1.0f;    // service-brake "at target" (BRAKE_TOLERANCE)

// State; the numeric values are the NVS "pbrake"/state codes.
enum class ParkBrakeState : uint8_t {
  UNKNOWN   = 0,
  RELEASED  = 1,
  APPLIED   = 2,
  APPLYING  = 3,
  RELEASING = 4
};

enum class ParkBrakeRequest : uint8_t { APPLY, RELEASE };

// Who issued a command: the web card (WEB) or the ground station via MAV_CMD_USER_2 param1 5 (GCS).
enum class ParkBrakeSource : uint8_t { WEB, GCS };

// commandParkBrake() result codes (design D4). The web layer maps each one to a reply text;
// a future MAV_CMD_USER_2 handler maps them to MAV_RESULT_*.
enum class ParkBrakeResult : uint8_t {
  STARTED,                // a run started
  STARTED_SPEED_UNKNOWN,  // APPLY run started, speed sensor never pulsed since boot
  ALREADY_RUNNING,        // same direction already running — ignored, timer not restarted
  REVERSING,              // opposite direction was running — stop, dwell, full run the new way
  REFUSED_MOVING,         // APPLY above PARK_BRAKE_APPLY_MAX_SPEED_MS
  REFUSED_DRIVER,         // X6 driver failed to initialise
  REFUSED_NVS             // APPLY whose moving state could not be saved
};

// What the controller must do for an accepted command.
enum class ParkBrakeAction : uint8_t {
  NONE,     // nothing (ignored or refused)
  START,    // drive the new direction at once
  REVERSE   // stop, PARK_BRAKE_REVERSE_DWELL_MS at zero duty, then drive the new direction
};

enum class ParkBrakeSpeedCheck : uint8_t { OK, REFUSED, UNKNOWN_SPEED };

// Commanded transmission gear as the law sees it (the controller maps TransmissionController::Gear).
enum class ParkBrakeGear : uint8_t { Unknown, Neutral, Low, High, Reverse };

// Why a throttle press did not release (telemetry pbrake_da_wait).
enum class ParkBrakeDaWait : uint8_t { NONE, IGN_OFF, REARM };

struct ParkBrakeTransition {
  ParkBrakeResult result;
  ParkBrakeAction action;
};

struct ParkBrakeLatchStep {
  bool armed;   // latch state after this sample
  bool fire;    // start a drive-away RELEASE now
};

// ---- State decode / tokens ----

// Stored NVS code → state at boot: 1 RELEASED, 2 APPLIED, anything else (0, a moving value 3/4,
// out of range, or a missing key passed as 0) UNKNOWN.
inline ParkBrakeState parkBrakeDecodeStored(int32_t code) {
  if (code == 1) return ParkBrakeState::RELEASED;
  if (code == 2) return ParkBrakeState::APPLIED;
  return ParkBrakeState::UNKNOWN;
}

inline const char* parkBrakeStateToken(ParkBrakeState s) {
  switch (s) {
    case ParkBrakeState::RELEASED:  return "released";
    case ParkBrakeState::APPLIED:   return "applied";
    case ParkBrakeState::APPLYING:  return "applying";
    case ParkBrakeState::RELEASING: return "releasing";
    default:                        return "unknown";
  }
}

inline const char* parkBrakeDaWaitToken(ParkBrakeDaWait w) {
  switch (w) {
    case ParkBrakeDaWait::IGN_OFF: return "ign_off";
    case ParkBrakeDaWait::REARM:   return "rearm";
    default:                       return "none";
  }
}

inline bool parkBrakeIsMoving(ParkBrakeState s) {
  return s == ParkBrakeState::APPLYING || s == ParkBrakeState::RELEASING;
}

// APPLIED or UNKNOWN: possibly applied, eligible for the drive-away release.
inline bool parkBrakeMaybeApplied(ParkBrakeState s) {
  return s == ParkBrakeState::APPLIED || s == ParkBrakeState::UNKNOWN;
}

inline bool parkBrakeGearEngaged(ParkBrakeGear g) {
  return g == ParkBrakeGear::Low || g == ParkBrakeGear::High || g == ParkBrakeGear::Reverse;
}

// ---- Travel time / run length / throttle release point ----

inline bool parkBrakeTravelMsValid(int32_t ms) {
  return ms >= PB_LAW_TRAVEL_MS_MIN && ms <= PB_LAW_TRAVEL_MS_MAX;
}

inline int32_t parkBrakeTravelMsOrDefault(int32_t stored) {
  return parkBrakeTravelMsValid(stored) ? stored : PB_LAW_TRAVEL_MS_DEFAULT;
}

// One run: travel time plus the fixed margin (reverse dwell not included).
inline uint32_t parkBrakeRunMs(int32_t travelMs) {
  return (uint32_t)(travelMs + PB_LAW_TRAVEL_MARGIN_MS);
}

inline bool parkBrakeThrPassPctValid(int32_t pct) {
  return pct >= PB_LAW_THR_PASS_PCT_MIN && pct <= PB_LAW_THR_PASS_PCT_MAX;
}

inline int32_t parkBrakeThrPassPctOrDefault(int32_t stored) {
  return parkBrakeThrPassPctValid(stored) ? stored : PB_LAW_THR_PASS_PCT_DEFAULT;
}

// ms into the release DRIVE (after any reverse dwell) from which the operator throttle passes.
// pct < 100 → travel × pct / 100; 100 (or more) → the whole run, margin included.
inline uint32_t parkBrakeThrPassMs(int32_t travelMs, int32_t pct) {
  if (pct >= PB_LAW_THR_PASS_PCT_MAX) return parkBrakeRunMs(travelMs);
  if (pct < 0) pct = 0;
  return (uint32_t)(((int64_t)travelMs * pct) / 100);
}

// ---- Apply speed interlock ----

// Same trust rule as the gear-change interlock: the hall reading counts when valid or latched
// suspicious. Never pulsed since boot (neither) → no evidence of motion → UNKNOWN_SPEED (allowed).
inline ParkBrakeSpeedCheck parkBrakeApplySpeedOk(float speedMs, bool valid, bool suspicious) {
  if (!valid && !suspicious) return ParkBrakeSpeedCheck::UNKNOWN_SPEED;
  return (speedMs > PB_LAW_APPLY_MAX_SPEED_MS) ? ParkBrakeSpeedCheck::REFUSED : ParkBrakeSpeedCheck::OK;
}

// ---- Command transitions (design D4) ----
// `speed` is only consulted for APPLY (pass OK for RELEASE). `driverOk` false refuses everything.
inline ParkBrakeTransition parkBrakeTransition(ParkBrakeState state, ParkBrakeRequest req,
                                               ParkBrakeSpeedCheck speed, bool driverOk) {
  if (!driverOk) return {ParkBrakeResult::REFUSED_DRIVER, ParkBrakeAction::NONE};
  if (req == ParkBrakeRequest::APPLY) {
    if (state == ParkBrakeState::APPLYING) return {ParkBrakeResult::ALREADY_RUNNING, ParkBrakeAction::NONE};
    if (speed == ParkBrakeSpeedCheck::REFUSED) return {ParkBrakeResult::REFUSED_MOVING, ParkBrakeAction::NONE};
    if (state == ParkBrakeState::RELEASING) return {ParkBrakeResult::REVERSING, ParkBrakeAction::REVERSE};
    return {speed == ParkBrakeSpeedCheck::UNKNOWN_SPEED ? ParkBrakeResult::STARTED_SPEED_UNKNOWN
                                                        : ParkBrakeResult::STARTED,
            ParkBrakeAction::START};
  }
  if (state == ParkBrakeState::RELEASING) return {ParkBrakeResult::ALREADY_RUNNING, ParkBrakeAction::NONE};
  if (state == ParkBrakeState::APPLYING) return {ParkBrakeResult::REVERSING, ParkBrakeAction::REVERSE};
  return {ParkBrakeResult::STARTED, ParkBrakeAction::START};
}

// ---- Direction mapping (design D2) ----
// +1 = setSpeed(+duty) = RPWM (GPIO5), -1 = setSpeed(-duty) = LPWM (GPIO4).
// invert 0: APPLY → +1, RELEASE → -1; invert 1 swaps them.
inline int8_t parkBrakeDriveSign(ParkBrakeRequest req, bool invert) {
  const int8_t s = (req == ParkBrakeRequest::APPLY) ? 1 : -1;
  return invert ? (int8_t)-s : s;
}

// ---- Throttle hold (design D5, D12-A) ----
// In L/H/R: APPLIED / APPLYING / UNKNOWN hold; RELEASING holds while releaseElapsedMs (ms into the
// release DRIVE, 0 during the reverse dwell) is below thrPassMs. N and UNKNOWN gear never hold.
inline bool parkBrakeHoldsThrottle(ParkBrakeState state, ParkBrakeGear gear,
                                   uint32_t releaseElapsedMs, uint32_t thrPassMs) {
  if (!parkBrakeGearEngaged(gear)) return false;
  switch (state) {
    case ParkBrakeState::APPLIED:
    case ParkBrakeState::APPLYING:
    case ParkBrakeState::UNKNOWN:
      return true;
    case ParkBrakeState::RELEASING:
      return releaseElapsedMs < thrPassMs;
    default:
      return false;
  }
}

// ---- Drive-away latch (design D5, D12-B) ----
// One sample of the operator demand (after the gear clip). Ignition OFF disarms and never fires.
// Arms while APPLIED/UNKNOWN with the ignition on and demand ≤ threshold (in any gear — in N the
// demand is clipped to 8.5 %). Fires, with no debounce, on demand > threshold while armed,
// APPLIED/UNKNOWN, gear L/H/R and no gear change / shift throttle lock; firing disarms.
inline ParkBrakeLatchStep parkBrakeLatchStep(bool armed, ParkBrakeState state, bool ignitionOn,
                                             ParkBrakeGear gear, bool shiftBusy, float demandPct) {
  if (!ignitionOn) return {false, false};
  const bool maybeApplied = parkBrakeMaybeApplied(state);
  if (maybeApplied && demandPct <= PB_LAW_DRIVE_AWAY_PCT) armed = true;
  const bool fire = maybeApplied && armed && parkBrakeGearEngaged(gear) && !shiftBusy &&
                    demandPct > PB_LAW_DRIVE_AWAY_PCT;
  return {fire ? false : armed, fire};
}

// Why a throttle press is not releasing the brake (telemetry pbrake_da_wait).
inline ParkBrakeDaWait parkBrakeDaWait(ParkBrakeState state, ParkBrakeGear gear, bool ignitionOn,
                                       float demandPct, bool armed) {
  if (!parkBrakeMaybeApplied(state) || !parkBrakeGearEngaged(gear)) return ParkBrakeDaWait::NONE;
  if (!ignitionOn) return ParkBrakeDaWait::IGN_OFF;
  if (demandPct > PB_LAW_DRIVE_AWAY_PCT && !armed) return ParkBrakeDaWait::REARM;
  return ParkBrakeDaWait::NONE;
}

// ---- Release by a service-brake press (change add-park-brake-release-by-service-brake) ----
// Owner decision 2026-11-17 "no protection": ANY new service-brake press releases an APPLIED or
// UNKNOWN parking brake — including an automatic brake on channel 2 (BlueOS Copilot on RC loss).
//
// Input: the commanded service-brake target as applyBrake() sets it (0, or 5..100 %), the
// open-loop service-brake position estimate, the input source and the park-brake state, sampled
// on EVERY control loop in every park-brake state (design D1–D3):
//  - press = target 0 → > 0 under MAVLINK or WEB; non-zero → other non-zero is not a press;
//  - source NONE (the firmware FAILSAFE) never detects a press but keeps the sample current, so a
//    frozen target is never a press, also when a live source returns with the same value;
//  - any source change re-seeds the sample and cancels a pending hand-over (no edge by itself);
//  - hand-over (assumption A1): the release fires on the first step whose estimate is at or above
//    target − PB_LAW_SVC_TOL_PCT. Target back to 0, a source change or a state other than
//    APPLIED/UNKNOWN cancels the wait; a new press is then needed.
// The step has NO apply output: a service-brake press can only ever request a RELEASE.
enum class ParkBrakeSvcSource : uint8_t { NONE, MAVLINK, WEB };   // NONE = firmware FAILSAFE

struct ParkBrakeSvcRelease {
  float prevTargetPct;            // target seen on the previous step
  bool pending;                   // a press is waiting for the hand-over
  ParkBrakeSvcSource prevSource;  // source seen on the previous step
};

struct ParkBrakeSvcStep {
  ParkBrakeSvcRelease next;       // law state after this step
  bool fire;                      // request a RELEASE run now
};

inline ParkBrakeSvcRelease parkBrakeSvcInitial() {
  return {0.0f, false, ParkBrakeSvcSource::NONE};
}

inline ParkBrakeSvcStep parkBrakeSvcStep(const ParkBrakeSvcRelease& s, ParkBrakeSvcSource src,
                                         float targetPct, float estPct, ParkBrakeState st) {
  ParkBrakeSvcStep r{{targetPct, false, src}, false};
  if (src != s.prevSource) return r;               // re-seed, cancel, no fire
  if (src == ParkBrakeSvcSource::NONE) return r;   // FAILSAFE: sample only
  if (!parkBrakeMaybeApplied(st)) return r;        // RELEASED / APPLYING / RELEASING: cancel

  bool pending = s.pending;
  if (s.prevTargetPct <= 0.0f && targetPct > 0.0f) pending = true;   // rising edge
  if (targetPct <= 0.0f) pending = false;                              // dropped before hand-over
  if (pending && estPct >= targetPct - PB_LAW_SVC_TOL_PCT) {
    r.fire = true;
    pending = false;
  }
  r.next.pending = pending;
  return r;
}
