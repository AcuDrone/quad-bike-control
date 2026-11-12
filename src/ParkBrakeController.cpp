#include "ParkBrakeController.h"
#include "Debug.h"
#include <Preferences.h>

// ParkBrakeLaw.h cannot include Constants.h (native tests): keep its literals in sync.
static_assert(PB_LAW_TRAVEL_MS_MIN == PARK_BRAKE_TRAVEL_MS_MIN, "ParkBrakeLaw.h travel min out of sync");
static_assert(PB_LAW_TRAVEL_MS_MAX == PARK_BRAKE_TRAVEL_MS_MAX, "ParkBrakeLaw.h travel max out of sync");
static_assert(PB_LAW_TRAVEL_MS_DEFAULT == PARK_BRAKE_TRAVEL_MS_DEFAULT, "ParkBrakeLaw.h travel default out of sync");
static_assert(PB_LAW_TRAVEL_MARGIN_MS == PARK_BRAKE_TRAVEL_MARGIN_MS, "ParkBrakeLaw.h margin out of sync");
static_assert(PB_LAW_REVERSE_DWELL_MS == PARK_BRAKE_REVERSE_DWELL_MS, "ParkBrakeLaw.h dwell out of sync");
static_assert(PB_LAW_THR_PASS_PCT_MIN == PARK_BRAKE_THR_PASS_PCT_MIN, "ParkBrakeLaw.h release point min out of sync");
static_assert(PB_LAW_THR_PASS_PCT_MAX == PARK_BRAKE_THR_PASS_PCT_MAX, "ParkBrakeLaw.h release point max out of sync");
static_assert(PB_LAW_THR_PASS_PCT_DEFAULT == PARK_BRAKE_THR_PASS_PCT_DEFAULT, "ParkBrakeLaw.h release point default out of sync");
static_assert(PB_LAW_APPLY_MAX_SPEED_MS == PARK_BRAKE_APPLY_MAX_SPEED_MS, "ParkBrakeLaw.h apply speed out of sync");
static_assert(PB_LAW_DRIVE_AWAY_PCT == PARK_BRAKE_DRIVE_AWAY_THROTTLE_PCT, "ParkBrakeLaw.h drive-away threshold out of sync");

static const char* dirName(ParkBrakeRequest r) {
    return (r == ParkBrakeRequest::APPLY) ? "apply" : "release";
}

ParkBrakeController::ParkBrakeController(BTS7960Controller& driver)
    : driver_(driver),
      state_(ParkBrakeState::UNKNOWN),
      phase_(Phase::IDLE),
      runDir_(ParkBrakeRequest::RELEASE),
      phaseStartMs_(0),
      runMs_(0),
      runThrPassMs_(0),
      travelMs_(PARK_BRAKE_TRAVEL_MS_DEFAULT),
      thrPassPct_(PARK_BRAKE_THR_PASS_PCT_DEFAULT),
      invert_(false) {
}

void ParkBrakeController::begin() {
    // No movement at boot: the bridge stays at the zero duty begin() left it at.
    int32_t stored = 0;          // missing key / missing namespace → 0 → UNKNOWN
    int32_t travel = PARK_BRAKE_TRAVEL_MS_DEFAULT;
    int32_t pass = PARK_BRAKE_THR_PASS_PCT_DEFAULT;
    uint8_t inv = 0;
    bool hasState = false;
    Preferences prefs;
    if (prefs.begin(PARK_BRAKE_NVS_NAMESPACE, true)) {
        hasState = prefs.isKey(PARK_BRAKE_STATE_NVS_KEY);
        stored = prefs.getUChar(PARK_BRAKE_STATE_NVS_KEY, 0);
        travel = prefs.getUShort(PARK_BRAKE_TRAVEL_NVS_KEY, PARK_BRAKE_TRAVEL_MS_DEFAULT);
        pass = prefs.getUChar(PARK_BRAKE_THR_PASS_NVS_KEY, PARK_BRAKE_THR_PASS_PCT_DEFAULT);
        inv = prefs.getUChar(PARK_BRAKE_INVERT_NVS_KEY, 0);
        prefs.end();
    }

    state_ = parkBrakeDecodeStored(stored);
    if (state_ == ParkBrakeState::UNKNOWN) {
        // Logged, never rewritten: a moving value (3/4) means power was lost mid-run.
        Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Stored state %s (%ld) -> UNKNOWN\n",
                             hasState ? "invalid or mid-run" : "missing", (long)stored);
    }
    if (!parkBrakeTravelMsValid(travel)) {
        Debug::printfFeature(DebugFeature::BRAKE,
            "[PBRAKE] Stored travel time %ld ms out of range (%d-%d ms) — ignored\n",
            (long)travel, PARK_BRAKE_TRAVEL_MS_MIN, PARK_BRAKE_TRAVEL_MS_MAX);
    }
    if (!parkBrakeThrPassPctValid(pass)) {
        Debug::printfFeature(DebugFeature::BRAKE,
            "[PBRAKE] Stored throttle release point %ld %% out of range (%d-%d %%) — ignored\n",
            (long)pass, PARK_BRAKE_THR_PASS_PCT_MIN, PARK_BRAKE_THR_PASS_PCT_MAX);
    }
    travelMs_ = (uint16_t)parkBrakeTravelMsOrDefault(travel);
    thrPassPct_ = (uint8_t)parkBrakeThrPassPctOrDefault(pass);
    invert_ = (inv != 0);
    phase_ = Phase::IDLE;

    Debug::printfFeature(DebugFeature::BRAKE,
        "[PBRAKE] State %s, travel %u ms, release point %u %%, direction %s, driver %s\n",
        parkBrakeStateToken(state_), (unsigned)travelMs_, (unsigned)thrPassPct_,
        invert_ ? "inverted" : "normal", isDriverOk() ? "OK" : "NOT AVAILABLE");
}

bool ParkBrakeController::persistState(ParkBrakeState s) {
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(PARK_BRAKE_NVS_NAMESPACE, false)) {
        ok = prefs.putUChar(PARK_BRAKE_STATE_NVS_KEY, (uint8_t)s) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) {
        Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] NVS write of state %s FAILED\n",
                             parkBrakeStateToken(s));
    }
    return ok;
}

void ParkBrakeController::startDrive(uint32_t now) {
    const int8_t sign = parkBrakeDriveSign(runDir_, invert_);
    driver_.setSpeed((int16_t)(sign * PARK_BRAKE_DRIVE_DUTY));
    phase_ = Phase::DRIVE;
    phaseStartMs_ = now;
    Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Drive %s (%s, %s) for %lu ms\n",
                         dirName(runDir_), sign > 0 ? "RPWM GPIO5" : "LPWM GPIO4",
                         invert_ ? "inverted" : "normal", (unsigned long)runMs_);
}

ParkBrakeController::Outcome ParkBrakeController::request(ParkBrakeRequest req, ParkBrakeSpeedCheck speed) {
    const ParkBrakeTransition tr = parkBrakeTransition(state_, req, speed, isDriverOk());
    Outcome out{tr.result, false};
    if (tr.action == ParkBrakeAction::NONE) {
        if (tr.result == ParkBrakeResult::ALREADY_RUNNING) {
            Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Already %s — run continues\n",
                                 req == ParkBrakeRequest::APPLY ? "applying" : "releasing");
        } else if (tr.result == ParkBrakeResult::REFUSED_DRIVER) {
            Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] %s refused: driver not available\n", dirName(req));
        }
        return out;
    }

    const ParkBrakeState moving = (req == ParkBrakeRequest::APPLY) ? ParkBrakeState::APPLYING
                                                                   : ParkBrakeState::RELEASING;
    // The moving state is written BEFORE the bridge is driven. An APPLY whose start is not on
    // record could come back after a reboot as a confident RELEASED while the brake is on: refused.
    if (!persistState(moving)) {
        if (req == ParkBrakeRequest::APPLY) {
            Debug::printlnFeature(DebugFeature::BRAKE, "[PBRAKE] Apply refused: state not saved");
            out.result = ParkBrakeResult::REFUSED_NVS;
            return out;
        }
        out.stateNotSaved = true;   // a stale APPLIED/UNKNOWN only holds the throttle until the next release
    }

    const uint32_t now = millis();
    runDir_ = req;
    runMs_ = parkBrakeRunMs(travelMs_);   // a new travel time only affects later runs
    runThrPassMs_ = parkBrakeThrPassMs(travelMs_, thrPassPct_);
    state_ = moving;
    if (tr.action == ParkBrakeAction::REVERSE) {
        driver_.stop();
        phase_ = Phase::DWELL;
        phaseStartMs_ = now;
        Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Reversing to %s: stop, %d ms dwell\n",
                             dirName(req), PARK_BRAKE_REVERSE_DWELL_MS);
    } else {
        Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Start %s%s\n", dirName(req),
                             out.stateNotSaved ? " (state not saved)" : "");
        startDrive(now);
    }
    return out;
}

void ParkBrakeController::update() {
    if (phase_ == Phase::IDLE) return;
    const uint32_t now = millis();
    const uint32_t elapsed = now - phaseStartMs_;   // millis() deltas: wrap-safe
    if (phase_ == Phase::DWELL) {
        if (elapsed >= (uint32_t)PARK_BRAKE_REVERSE_DWELL_MS) startDrive(now);
        return;
    }
    if (elapsed >= runMs_) {
        driver_.stop();
        phase_ = Phase::IDLE;
        state_ = (runDir_ == ParkBrakeRequest::APPLY) ? ParkBrakeState::APPLIED : ParkBrakeState::RELEASED;
        // After stop(). A failed write leaves the moving value stored → UNKNOWN after a reboot (safe).
        persistState(state_);
        Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Run end after %lu ms: %s\n",
                             (unsigned long)elapsed, parkBrakeStateToken(state_));
    }
}

uint32_t ParkBrakeController::getRemainingMs() const {
    if (phase_ == Phase::IDLE) return 0;
    const uint32_t elapsed = millis() - phaseStartMs_;
    if (phase_ == Phase::DWELL) {
        const uint32_t dwellLeft = (elapsed < (uint32_t)PARK_BRAKE_REVERSE_DWELL_MS)
                                       ? (uint32_t)PARK_BRAKE_REVERSE_DWELL_MS - elapsed : 0;
        return dwellLeft + runMs_;
    }
    return (elapsed < runMs_) ? runMs_ - elapsed : 0;
}

uint32_t ParkBrakeController::getReleaseElapsedMs() const {
    if (state_ != ParkBrakeState::RELEASING || phase_ != Phase::DRIVE) return 0;
    return millis() - phaseStartMs_;
}

uint32_t ParkBrakeController::getThrPassMs() const {
    // During a release run: the value captured at its start; otherwise what the next run will use.
    if (phase_ != Phase::IDLE) return runThrPassMs_;
    return parkBrakeThrPassMs(travelMs_, thrPassPct_);
}

bool ParkBrakeController::setTravelMs(uint16_t ms) {
    if (!parkBrakeTravelMsValid(ms)) return false;
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(PARK_BRAKE_NVS_NAMESPACE, false)) {
        ok = prefs.putUShort(PARK_BRAKE_TRAVEL_NVS_KEY, ms) == sizeof(uint16_t);
        prefs.end();
    }
    if (!ok) {
        Debug::printlnFeature(DebugFeature::BRAKE, "[PBRAKE] NVS write of travel_ms FAILED");
        return false;
    }
    travelMs_ = ms;   // a run in progress keeps its captured duration
    Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Travel time set to %u ms\n", (unsigned)ms);
    return true;
}

bool ParkBrakeController::setThrPassPct(uint8_t pct) {
    if (!parkBrakeThrPassPctValid(pct)) return false;
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(PARK_BRAKE_NVS_NAMESPACE, false)) {
        ok = prefs.putUChar(PARK_BRAKE_THR_PASS_NVS_KEY, pct) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) {
        Debug::printlnFeature(DebugFeature::BRAKE, "[PBRAKE] NVS write of thr_pass FAILED");
        return false;
    }
    thrPassPct_ = pct;   // from the next release run
    Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Throttle release point set to %u %%\n", (unsigned)pct);
    return true;
}

ParkBrakeController::InvertResult ParkBrakeController::setInvert(bool invert) {
    if (phase_ != Phase::IDLE || parkBrakeIsMoving(state_)) {
        Debug::printlnFeature(DebugFeature::BRAKE, "[PBRAKE] Direction change refused: actuator moving");
        return InvertResult::REFUSED_MOVING;
    }
    Preferences prefs;
    bool ok = false;
    if (prefs.begin(PARK_BRAKE_NVS_NAMESPACE, false)) {
        ok = prefs.putUChar(PARK_BRAKE_INVERT_NVS_KEY, invert ? 1 : 0) == sizeof(uint8_t);
        prefs.end();
    }
    if (!ok) {
        Debug::printlnFeature(DebugFeature::BRAKE, "[PBRAKE] NVS write of invert FAILED");
        return InvertResult::NVS_FAILED;
    }
    invert_ = invert;
    // The old state was measured with the old polarity. A failed write is logged by persistState();
    // the stored APPLIED/RELEASED would then come back after a reboot — RAM is UNKNOWN regardless.
    state_ = ParkBrakeState::UNKNOWN;
    persistState(state_);
    Debug::printfFeature(DebugFeature::BRAKE, "[PBRAKE] Direction %s — state UNKNOWN\n",
                         invert ? "inverted" : "normal");
    return InvertResult::OK;
}
