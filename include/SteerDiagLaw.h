// SteerDiagLaw.h — pure logic of the USB steering diagnostics console (change
// add-steering-serial-diagnostics): the command-line parser, the experiment state machine
// (ramp / pulse / hold / goto) with its summary values, and the safety predicates.
//
// NO Arduino / Constants.h dependency: compiled by the native unit tests
// (test/test_steer_diag). The Arduino side (include/SteerDiag.h) feeds samples in, applies
// the returned duty and prints the lines. Callers pass the constants through SteerDiagParams.
//
// Units: positions 0.1° (firmware sign, + = right), times ms, duties 0..255 (signed when
// applied: + = right), rate °/s, current A.
//
// Experiments (one at a time):
//  RAMP  dir: duty = startDuty, +step every stepMs up to maxDuty. Movement = (pos − start)·dir ≥
//        movePos, or rate·dir ≥ moveRate. On movement: break-away duty / current / time recorded,
//        duty cut to 0 → SETTLE (settleMs) → DONE. No movement: ends after maxHoldMs at maxDuty
//        or after rampTimeoutMs (moved = false).
//  PULSE dir duty ms: duty for ms → SETTLE → DONE.
//  HOLD  dir duty ms: duty for ms; the second half averages the current and the rate (LWS and
//        position slope) → SETTLE → DONE.
//  GOTO  target: the closed loop drives (the caller starts it); DRIVE while the loop drives,
//        SETTLE while it is parked / idle (back to DRIVE if it restarts), DONE after settleMs
//        of SETTLE or at gotoTimeoutMs. Overshoot = max (pos − target)·travelDir.
#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Command parser
// ---------------------------------------------------------------------------

enum class SteerDiagCmdType : uint8_t {
  NONE,      // empty line
  INVALID,   // see SteerDiagCmd::error
  DIAG_ON, DIAG_OFF, STATUS, STOP, HELP,
  RAMP, PULSE, HOLD, GOTO,
};

struct SteerDiagCmd {
  SteerDiagCmdType type = SteerDiagCmdType::NONE;
  int8_t dir = 0;            // −1 = L, +1 = R (ramp / pulse / hold)
  int16_t duty = 0;          // pulse / hold
  uint32_t ms = 0;           // pulse / hold
  int16_t maxDuty = 220;     // ramp
  int16_t step = 2;          // ramp
  uint32_t stepMs = 150;     // ramp
  int16_t startDuty = 40;    // ramp
  int32_t target = 0;        // goto, 0.1°
  const char* error = nullptr;  // INVALID: unknown_cmd, bad_dir, bad_arg, range, missing_arg
};

struct SteerDiagLimits {
  int16_t maxDuty;           // absolute duty cap (255)
  uint32_t pulseMaxMs;
  uint32_t holdMaxMs;
  float gotoMaxDeg;
};

static inline char steerDiagLower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c; }

static inline bool steerDiagSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

// Strict integer parse of a whole token
static inline bool steerDiagParseInt(const char* s, long& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  out = v;
  return true;
}

static inline bool steerDiagParseFloat(const char* s, float& out) {
  if (!s || !*s) return false;
  char* end = nullptr;
  float v = strtof(s, &end);
  if (end == s || *end != '\0' || !isfinite(v)) return false;
  out = v;
  return true;
}

static inline int8_t steerDiagParseDir(const char* s) {
  if (!s) return 0;
  if (strcmp(s, "l") == 0 || strcmp(s, "left") == 0 || strcmp(s, "-") == 0) return -1;
  if (strcmp(s, "r") == 0 || strcmp(s, "right") == 0 || strcmp(s, "+") == 0) return 1;
  return 0;
}

static inline SteerDiagCmd steerDiagInvalid(const char* why) {
  SteerDiagCmd c;
  c.type = SteerDiagCmdType::INVALID;
  c.error = why;
  return c;
}

/**
 * @brief Parse one command line (case-insensitive, surrounding whitespace / CR / LF ignored).
 *   diag on | diag off | status | stop | s | help | ?
 *   ramp L|R [max=220] [step=2] [stepms=150] [start=40]   (also startduty=)
 *   pulse L|R <duty> <ms>        hold L|R <duty> <ms>        goto <deg>
 */
static inline SteerDiagCmd steerDiagParse(const char* line, const SteerDiagLimits& lim) {
  char buf[128];
  size_t n = 0;
  for (const char* p = line; p && *p && n < sizeof(buf) - 1; ++p) buf[n++] = steerDiagLower(*p);
  buf[n] = '\0';

  // Tokenise in place
  char* tok[10];
  int nt = 0;
  char* p = buf;
  while (*p && nt < 10) {
    while (*p && steerDiagSpace(*p)) *p++ = '\0';
    if (!*p) break;
    tok[nt++] = p;
    while (*p && !steerDiagSpace(*p)) ++p;
  }
  if (*p) return steerDiagInvalid("bad_arg");   // too many tokens
  SteerDiagCmd c;
  if (nt == 0) return c;   // NONE

  const char* cmd = tok[0];
  if (strcmp(cmd, "diag") == 0) {
    if (nt != 2) return steerDiagInvalid("missing_arg");
    if (strcmp(tok[1], "on") == 0) { c.type = SteerDiagCmdType::DIAG_ON; return c; }
    if (strcmp(tok[1], "off") == 0) { c.type = SteerDiagCmdType::DIAG_OFF; return c; }
    return steerDiagInvalid("bad_arg");
  }
  if (strcmp(cmd, "status") == 0) { if (nt != 1) return steerDiagInvalid("bad_arg"); c.type = SteerDiagCmdType::STATUS; return c; }
  if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "s") == 0) { c.type = SteerDiagCmdType::STOP; return c; }
  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) { c.type = SteerDiagCmdType::HELP; return c; }

  if (strcmp(cmd, "ramp") == 0) {
    if (nt < 2) return steerDiagInvalid("missing_arg");
    c.dir = steerDiagParseDir(tok[1]);
    if (c.dir == 0) return steerDiagInvalid("bad_dir");
    for (int i = 2; i < nt; ++i) {
      char* eq = strchr(tok[i], '=');
      if (!eq) return steerDiagInvalid("bad_arg");
      *eq = '\0';
      long v;
      if (!steerDiagParseInt(eq + 1, v)) return steerDiagInvalid("bad_arg");
      const char* k = tok[i];
      if (strcmp(k, "max") == 0) {
        if (v < 1 || v > lim.maxDuty) return steerDiagInvalid("range");
        c.maxDuty = (int16_t)v;
      } else if (strcmp(k, "step") == 0) {
        if (v < 1 || v > 50) return steerDiagInvalid("range");
        c.step = (int16_t)v;
      } else if (strcmp(k, "stepms") == 0) {
        if (v < 20 || v > 2000) return steerDiagInvalid("range");
        c.stepMs = (uint32_t)v;
      } else if (strcmp(k, "start") == 0 || strcmp(k, "startduty") == 0) {
        if (v < 0 || v > lim.maxDuty) return steerDiagInvalid("range");
        c.startDuty = (int16_t)v;
      } else {
        return steerDiagInvalid("bad_arg");
      }
    }
    if (c.maxDuty > lim.maxDuty) c.maxDuty = lim.maxDuty;
    if (c.startDuty > c.maxDuty) return steerDiagInvalid("range");
    c.type = SteerDiagCmdType::RAMP;
    return c;
  }

  if (strcmp(cmd, "pulse") == 0 || strcmp(cmd, "hold") == 0) {
    const bool pulse = (cmd[0] == 'p');
    if (nt != 4) return steerDiagInvalid("missing_arg");
    c.dir = steerDiagParseDir(tok[1]);
    if (c.dir == 0) return steerDiagInvalid("bad_dir");
    long d, ms;
    if (!steerDiagParseInt(tok[2], d) || !steerDiagParseInt(tok[3], ms)) return steerDiagInvalid("bad_arg");
    if (d < 1 || d > lim.maxDuty) return steerDiagInvalid("range");
    const long maxMs = (long)(pulse ? lim.pulseMaxMs : lim.holdMaxMs);
    if (ms < (pulse ? 5 : 50) || ms > maxMs) return steerDiagInvalid("range");
    c.duty = (int16_t)d;
    c.ms = (uint32_t)ms;
    c.type = pulse ? SteerDiagCmdType::PULSE : SteerDiagCmdType::HOLD;
    return c;
  }

  if (strcmp(cmd, "goto") == 0) {
    if (nt != 2) return steerDiagInvalid("missing_arg");
    float deg;
    if (!steerDiagParseFloat(tok[1], deg)) return steerDiagInvalid("bad_arg");
    if (fabsf(deg) > lim.gotoMaxDeg) return steerDiagInvalid("range");
    c.target = (int32_t)lroundf(deg * 10.0f);
    c.type = SteerDiagCmdType::GOTO;
    return c;
  }

  return steerDiagInvalid("unknown_cmd");
}

// ---------------------------------------------------------------------------
// Safety predicates
// ---------------------------------------------------------------------------

/** @brief Vehicle moving: a trusted speed reading above maxKmh (an untrusted one reads as standstill) */
static inline bool steerDiagMoving(bool speedValid, float speedMs, float maxKmh) {
  return speedValid && speedMs * 3.6f > maxKmh;
}

/**
 * @brief Driving with sign dutySign at pos would come within margin of a captured limit
 * (unset = the POS_UNSET sentinel: that side is not checked).
 */
static inline bool steerDiagNearLimit(int32_t pos, int dutySign, int32_t limL, int32_t limR,
                                      int32_t margin, int32_t unset) {
  if (dutySign > 0 && limR != unset && pos >= limR - margin) return true;
  if (dutySign < 0 && limL != unset && pos <= limL + margin) return true;
  return false;
}

/**
 * @brief USB host-lost debounce. `(bool)Serial` on the ESP32-S3 USB-Serial/JTAG (HWCDC) can read
 * false transiently, so the check arms only after it has read true once since `diag on`, and
 * trips only after it has read false continuously for lostMs.
 */
struct SteerDiagHostWatch {
  bool armed = false;
  bool lowSeen = false;
  uint32_t lowSinceMs = 0;
};

/** @brief Feed one `(bool)Serial` reading; true = host lost (debounced) */
static inline bool steerDiagHostLost(SteerDiagHostWatch& w, bool connected, uint32_t now, uint32_t lostMs) {
  if (connected) {
    w.armed = true;
    w.lowSeen = false;
    return false;
  }
  if (!w.armed) return false;
  if (!w.lowSeen) {
    w.lowSeen = true;
    w.lowSinceMs = now;
    return false;
  }
  return now - w.lowSinceMs >= lostMs;
}

/**
 * @brief USB TX keepalive due now? HWCDC (core 3.3.x) can leave a short reply in its TX ring
 * buffer until the next write() re-arms the TX interrupt; a periodic tiny write through the
 * core's own write() path pushes it out. Window: while diag is on, or for windowMs after the
 * last received command line (lineSeen = at least one line since boot). Never while D-lines
 * flow (logging — they already keep TX moving), never outside the window (a closed host port
 * accumulates nothing). Rate: one per periodMs.
 */
static inline bool steerDiagKeepaliveDue(bool active, bool logging, bool lineSeen, uint32_t lastLineMs,
                                         uint32_t lastKeepaliveMs, uint32_t now,
                                         uint32_t windowMs, uint32_t periodMs) {
  if (logging) return false;
  const bool open = active || (lineSeen && now - lastLineMs < windowMs);
  if (!open) return false;
  return now - lastKeepaliveMs >= periodMs;
}

/** @brief A goto target is inside the captured limits with the margin to spare */
static inline bool steerDiagTargetOk(int32_t target, int32_t limL, int32_t limR, int32_t margin) {
  return target > limL + margin && target < limR - margin;
}

// ---------------------------------------------------------------------------
// Experiment state machine
// ---------------------------------------------------------------------------

struct SteerDiagParams {
  int32_t movePos;           // STEER_DIAG_MOVE_POS
  int32_t moveRate;          // STEER_DIAG_MOVE_RATE
  uint32_t settleMs;         // STEER_DIAG_SETTLE_MS
  uint32_t rampTimeoutMs;    // STEER_DIAG_RAMP_TIMEOUT_MS
  uint32_t rampMaxHoldMs;    // STEER_DIAG_RAMP_MAXHOLD_MS
  uint32_t gotoTimeoutMs;    // STEER_DIAG_GOTO_TIMEOUT_MS
};

enum class SteerDiagKind : uint8_t { NONE, RAMP, PULSE, HOLD, GOTO };
enum class SteerDiagPhase : uint8_t { IDLE, DRIVE, SETTLE, DONE };
/** @brief Closed-loop state as seen by goto (mirror of SteerPosState) */
enum class SteerDiagLoop : uint8_t { IDLE, DRIVE, PARK };

struct SteerDiagSample {
  uint32_t now = 0;
  int32_t pos = 0;           // 0.1°
  int32_t rate = 0;          // °/s
  bool rateValid = false;
  float currentA = 0.0f;     // latest VESC motor current
  SteerDiagLoop loop = SteerDiagLoop::IDLE;   // goto only
};

struct SteerDiagOut {
  int16_t duty = 0;          // signed open-loop duty to apply (ignored for goto)
  bool phaseChanged = false; // log a line now
  bool done = false;         // summary ready (fires once)
};

struct SteerDiagExp {
  SteerDiagKind kind = SteerDiagKind::NONE;
  SteerDiagPhase phase = SteerDiagPhase::IDLE;
  int8_t dir = 0;            // drive direction (goto: travel direction, 0 = already there)

  // Parameters
  int16_t startDuty = 0, maxDuty = 0, step = 0, duty = 0;
  uint32_t stepMs = 0, ms = 0;
  int32_t target = 0;

  // Running state
  uint32_t t0 = 0;
  uint32_t lastStepMs = 0;
  bool maxReached = false;
  uint32_t maxReachedMs = 0;
  uint32_t settleStartMs = 0;
  int32_t startPos = 0;
  int16_t curDuty = 0;       // unsigned duty in force (open loop)
  int32_t lastPos = 0;

  // Results
  bool moved = false;
  const char* endReason = nullptr;   // ramp: moved / no_move / timeout; goto: park / idle / timeout
  int16_t breakDuty = 0;
  float breakCurrentA = 0.0f;
  uint32_t breakMs = 0;      // since t0
  int32_t cutPos = 0;
  uint32_t cutMs = 0;        // since t0 (actual drive length)
  int32_t endPos = 0;
  uint32_t endMs = 0;        // since t0
  float peakCurrentA = 0.0f;
  // hold (second half)
  bool halfSeen = false;
  int32_t halfPos = 0;
  uint32_t halfMs = 0;
  double curSum = 0.0;
  uint32_t curN = 0;
  double rateSum = 0.0;
  uint32_t rateN = 0;
  // goto
  int32_t maxPast = 0;       // max (pos − target)·dir, 0.1° (≥ 0 when it overshot)
  uint32_t firstStopMs = 0;  // since t0, first time the loop stopped driving (0 = never)
  uint16_t restarts = 0;     // park → drive transitions

  bool running() const { return phase == SteerDiagPhase::DRIVE || phase == SteerDiagPhase::SETTLE; }
};

static inline void steerDiagReset(SteerDiagExp& e, SteerDiagKind kind, int32_t pos, uint32_t now) {
  e = SteerDiagExp();
  e.kind = kind;
  e.phase = SteerDiagPhase::DRIVE;
  e.t0 = now;
  e.lastStepMs = now;
  e.startPos = pos;
  e.lastPos = pos;
  e.cutPos = pos;
  e.endPos = pos;
}

static inline void steerDiagBeginRamp(SteerDiagExp& e, int8_t dir, int16_t startDuty, int16_t maxDuty,
                                      int16_t step, uint32_t stepMs, int32_t pos, uint32_t now) {
  steerDiagReset(e, SteerDiagKind::RAMP, pos, now);
  e.dir = dir;
  e.maxDuty = maxDuty;
  e.startDuty = startDuty > maxDuty ? maxDuty : startDuty;
  e.step = step;
  e.stepMs = stepMs;
  e.curDuty = e.startDuty;
  if (e.curDuty >= e.maxDuty) { e.maxReached = true; e.maxReachedMs = now; }
}

static inline void steerDiagBeginTimed(SteerDiagExp& e, SteerDiagKind kind, int8_t dir, int16_t duty,
                                       uint32_t ms, int32_t pos, uint32_t now) {
  steerDiagReset(e, kind, pos, now);
  e.dir = dir;
  e.duty = duty;
  e.ms = ms;
  e.curDuty = duty;
}

static inline void steerDiagBeginGoto(SteerDiagExp& e, int32_t target, int32_t pos, uint32_t now) {
  steerDiagReset(e, SteerDiagKind::GOTO, pos, now);
  e.target = target;
  e.dir = target > pos ? 1 : (target < pos ? -1 : 0);
  e.maxPast = INT32_MIN;
}

static inline void steerDiagCut(SteerDiagExp& e, const SteerDiagSample& s) {
  e.cutPos = s.pos;
  e.cutMs = s.now - e.t0;
  e.curDuty = 0;
  e.phase = SteerDiagPhase::SETTLE;
  e.settleStartMs = s.now;
}

static inline void steerDiagFinish(SteerDiagExp& e, const SteerDiagSample& s) {
  e.endPos = s.pos;
  e.endMs = s.now - e.t0;
  e.curDuty = 0;
  e.phase = SteerDiagPhase::DONE;
}

/** @brief Advance a running experiment by one sample (call every loop iteration) */
static inline SteerDiagOut steerDiagStep(SteerDiagExp& e, const SteerDiagParams& p, const SteerDiagSample& s) {
  SteerDiagOut out;
  if (!e.running()) return out;
  const SteerDiagPhase before = e.phase;
  const uint32_t el = s.now - e.t0;
  const float absCur = fabsf(s.currentA);
  e.lastPos = s.pos;

  if (e.kind == SteerDiagKind::GOTO) {
    if (absCur > e.peakCurrentA) e.peakCurrentA = absCur;
    const int32_t past = e.dir != 0 ? (s.pos - e.target) * e.dir
                                    : (s.pos > e.target ? s.pos - e.target : e.target - s.pos);
    if (past > e.maxPast) e.maxPast = past;
    if (s.loop == SteerDiagLoop::DRIVE) {
      if (e.phase == SteerDiagPhase::SETTLE) e.restarts++;
      e.phase = SteerDiagPhase::DRIVE;
    } else if (e.phase == SteerDiagPhase::DRIVE) {
      e.phase = SteerDiagPhase::SETTLE;
      e.settleStartMs = s.now;
      if (e.firstStopMs == 0) e.firstStopMs = el ? el : 1;
      e.endReason = s.loop == SteerDiagLoop::PARK ? "park" : "idle";
    } else {
      e.endReason = s.loop == SteerDiagLoop::PARK ? "park" : "idle";
    }
    if (el >= p.gotoTimeoutMs) {
      e.endReason = "timeout";
      steerDiagFinish(e, s);
    } else if (e.phase == SteerDiagPhase::SETTLE && s.now - e.settleStartMs >= p.settleMs) {
      steerDiagFinish(e, s);
    }
    out.phaseChanged = e.phase != before;
    out.done = e.phase == SteerDiagPhase::DONE;
    return out;
  }

  if (e.phase == SteerDiagPhase::DRIVE) {
    if (absCur > e.peakCurrentA) e.peakCurrentA = absCur;
    if (e.kind == SteerDiagKind::RAMP) {
      const bool posMoved = (s.pos - e.startPos) * e.dir >= p.movePos;
      const bool rateMoved = s.rateValid && s.rate * e.dir >= p.moveRate;
      if (posMoved || rateMoved) {
        e.moved = true;
        e.endReason = "moved";
        e.breakDuty = e.curDuty;
        e.breakCurrentA = s.currentA;
        e.breakMs = el;
        steerDiagCut(e, s);
      } else if (el >= p.rampTimeoutMs) {
        e.endReason = "timeout";
        steerDiagCut(e, s);
      } else if (e.maxReached && s.now - e.maxReachedMs >= p.rampMaxHoldMs) {
        e.endReason = "no_move";
        steerDiagCut(e, s);
      } else if (e.curDuty < e.maxDuty && s.now - e.lastStepMs >= e.stepMs) {
        e.lastStepMs = s.now;
        e.curDuty = (int16_t)(e.curDuty + e.step > e.maxDuty ? e.maxDuty : e.curDuty + e.step);
        if (e.curDuty >= e.maxDuty) { e.maxReached = true; e.maxReachedMs = s.now; }
        out.phaseChanged = true;   // log every duty step
      }
    } else {   // PULSE / HOLD
      if (e.kind == SteerDiagKind::HOLD && el >= e.ms / 2 && el < e.ms) {
        if (!e.halfSeen) {
          e.halfSeen = true;
          e.halfPos = s.pos;
          e.halfMs = el;
        }
        e.curSum += absCur;
        e.curN++;
        if (s.rateValid) {
          e.rateSum += s.rate;
          e.rateN++;
        }
      }
      if (el >= e.ms) steerDiagCut(e, s);
    }
  } else if (e.phase == SteerDiagPhase::SETTLE) {
    if (s.now - e.settleStartMs >= p.settleMs) steerDiagFinish(e, s);
  }

  out.duty = e.phase == SteerDiagPhase::DRIVE ? (int16_t)(e.curDuty * e.dir) : 0;
  out.phaseChanged = out.phaseChanged || e.phase != before;
  out.done = e.phase == SteerDiagPhase::DONE;
  return out;
}

// ---------------------------------------------------------------------------
// Summary values (degrees, firmware sign unless noted)
// ---------------------------------------------------------------------------

/** @brief Displacement from the start at the duty cut, ° (signed) */
static inline float steerDiagMovedAtCutDeg(const SteerDiagExp& e) { return (float)(e.cutPos - e.startPos) / 10.0f; }
/** @brief Displacement from the start after settling, ° (signed) */
static inline float steerDiagMovedEndDeg(const SteerDiagExp& e) { return (float)(e.endPos - e.startPos) / 10.0f; }
/** @brief Coast after the cut in the drive direction, ° (≥ 0 = kept going) */
static inline float steerDiagCoastDeg(const SteerDiagExp& e) { return (float)((e.endPos - e.cutPos) * e.dir) / 10.0f; }

/** @brief Hold: rate from the position slope over the second half, °/s (signed) */
static inline float steerDiagHoldPosRate(const SteerDiagExp& e) {
  if (!e.halfSeen || e.cutMs <= e.halfMs) return 0.0f;
  return ((float)(e.cutPos - e.halfPos) / 10.0f) / ((float)(e.cutMs - e.halfMs) / 1000.0f);
}
/** @brief Hold: mean LWS rate over the second half, °/s (signed) */
static inline float steerDiagHoldLwsRate(const SteerDiagExp& e) { return e.rateN ? (float)(e.rateSum / e.rateN) : 0.0f; }
/** @brief Hold: mean |current| over the second half, A */
static inline float steerDiagHoldMeanCurrent(const SteerDiagExp& e) { return e.curN ? (float)(e.curSum / e.curN) : 0.0f; }

/** @brief Goto: overshoot past the target in the travel direction, ° (0 if it never passed it) */
static inline float steerDiagOvershootDeg(const SteerDiagExp& e) {
  return (e.maxPast == INT32_MIN || e.maxPast < 0) ? 0.0f : (float)e.maxPast / 10.0f;
}
/** @brief Goto: residual error target − pos at the end, ° (signed) */
static inline float steerDiagResidualDeg(const SteerDiagExp& e) { return (float)(e.target - e.endPos) / 10.0f; }

/** @brief Token for a phase/kind in the D lines (open loop) */
static inline const char* steerDiagPhaseToken(const SteerDiagExp& e) {
  if (e.phase == SteerDiagPhase::SETTLE) return "coast";
  if (e.phase == SteerDiagPhase::DONE) return "done";
  switch (e.kind) {
    case SteerDiagKind::RAMP:  return "ramp";
    case SteerDiagKind::PULSE: return "pulse";
    case SteerDiagKind::HOLD:  return "hold";
    case SteerDiagKind::GOTO:  return "goto";
    default: return "idle";
  }
}
