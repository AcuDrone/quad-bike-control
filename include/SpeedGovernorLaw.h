// SpeedGovernorLaw.h — speed governor law: the throttle input is a speed setpoint, a PI loop on
// the wheel-speed error computes the throttle (change add-speed-governor, design D2–D8, D13–D15).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit tests
// (test/test_speed_governor). Callers pass the constants in through SpeedGovernorParams.
//
// Units: speeds m/s, gains % per m/s (Kp) and %/s per m/s (Ki), throttle 0–100 % (0 = idle).
//
//  stop    : stick ≤ dead-band or stop requested → setpoint 0, integrator 0, output 0 at once
//  target  : v* = vMin + (s − db)/(100 − db) × (L − vMin),  vMin = min(minSetpoint, 0.5 × L)
//  ramp    : vr → v* by at most acc × dt upward, dec × dt downward
//  PI      : e = vr − v;  u = clamp(Kp × e + I, 0, uMax);  uMax = start cap while v < standstill
//  windup  : then I += Ki × e × dt only if not (P + I ≥ uMax, e > 0) and not (P + I ≤ 0, e < 0);
//            I clamped to [0, uMax] (u uses the integrator as it stood before this step)
//  output  : moves toward u by at most outSlew × dt
//  prime   : vr = measured speed, output = last commanded throttle, I = min(that, start cap)
// The speed is used as read (no filter, no error dead-band, design D15).
#pragma once

#include <stdint.h>
#include <math.h>

struct SpeedGovernorParams {
  float kpPctPerMs;        // proportional gain, % per m/s
  float kiPctSPerMs;       // integral gain, %/s per m/s
  float accelMs2;          // setpoint ramp upward, m/s²
  float decelMs2;          // setpoint ramp downward, m/s²
  float startCapPct;       // start throttle limit: uMax while the wheels do not turn, %
  float stickDeadbandPct;  // stick at or below this = stop, %
  float minSetpointMs;     // lowest non-zero setpoint (halved for very low limits), m/s
  float standstillMs;      // reading below this = the wheels do not turn, m/s
  float outSlewPctS;       // output rate limit (the limiter's thr_slew), %/s
};

struct SpeedGovernorState {
  float spMs;      // ramped setpoint, m/s
  float integPct;  // integrator, %
  float outPct;    // last output, %
  bool primed;     // false = the next evaluation must prime (bumpless entry)

  SpeedGovernorState() { reset(); }
  void reset() {
    spMs = 0.0f;
    integPct = 0.0f;
    outPct = 0.0f;
    primed = false;
  }
};

enum class GovMode : uint8_t { OFF, NOLIMIT, BYPASS, NOSPEED, ACTIVE };

static inline float govClamp(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/** @brief Mode selection, first match wins (design D3). */
static inline GovMode govModeSelect(bool enabled, float limitMs, bool bypass, bool speedValid) {
  if (!enabled) return GovMode::OFF;
  if (!(limitMs > 0.0f)) return GovMode::NOLIMIT;
  if (bypass) return GovMode::BYPASS;
  if (!speedValid) return GovMode::NOSPEED;
  return GovMode::ACTIVE;
}

/**
 * @brief True when the caller must take today's throttle path untouched, evaluating nothing of
 * the governor: the switch is off, the last mode was OFF and no hand-over blend is pending. This
 * is what makes "switch OFF" bit-for-bit the firmware without the governor.
 */
static inline bool govPassThrough(bool enabled, GovMode lastMode, bool blendActive) {
  return !enabled && lastMode == GovMode::OFF && !blendActive;
}

/** @brief Static telemetry token of a mode. */
static inline const char* govModeToken(GovMode m) {
  switch (m) {
    case GovMode::OFF:     return "off";
    case GovMode::NOLIMIT: return "nolimit";
    case GovMode::BYPASS:  return "bypass";
    case GovMode::NOSPEED: return "nospeed";
    case GovMode::ACTIVE:  return "active";
  }
  return "off";
}

/** @brief Target speed (m/s) for a stick position (%) under limit L (m/s); 0 in the dead-band. */
static inline float govTargetMs(float stickPct, float limitMs, const SpeedGovernorParams& p) {
  if (!(limitMs > 0.0f) || !(stickPct > p.stickDeadbandPct)) return 0.0f;
  const float s = govClamp(stickPct, p.stickDeadbandPct, 100.0f);
  float vMin = p.minSetpointMs;
  if (vMin > 0.5f * limitMs) vMin = 0.5f * limitMs;
  const float span = 100.0f - p.stickDeadbandPct;
  if (span <= 0.0f) return limitMs;
  return vMin + (s - p.stickDeadbandPct) / span * (limitMs - vMin);
}

/**
 * @brief Bumpless entry (design D8): setpoint = measured speed, output = the throttle last
 * commanded, integrator = that throttle but not above the start throttle limit.
 */
static inline void govPrime(SpeedGovernorState& s, float lastThrottlePct, float speedMs,
                            float startCapPct) {
  const float u0 = govClamp(isnan(lastThrottlePct) ? 0.0f : lastThrottlePct, 0.0f, 100.0f);
  s.spMs = (speedMs > 0.0f) ? speedMs : 0.0f;
  s.outPct = u0;
  s.integPct = (u0 < startCapPct) ? u0 : startCapPct;
  if (s.integPct < 0.0f) s.integPct = 0.0f;
  s.primed = true;
}

/**
 * @brief One governor evaluation (design D2). Primes from idle if not primed (callers prime
 * explicitly with the real throttle first).
 * @param dtS seconds since the previous evaluation, clamped to 0..1
 * @return the throttle output, %
 */
static inline float govStep(SpeedGovernorState& s, const SpeedGovernorParams& p, float stickPct,
                            bool stopRequested, float speedMs, float limitMs, float dtS) {
  if (!s.primed) govPrime(s, 0.0f, speedMs, p.startCapPct);
  const float dt = govClamp(isnan(dtS) ? 0.0f : dtS, 0.0f, 1.0f);

  // 1. Stop: idle at once, ramp and integrator reset.
  if (stopRequested || !(stickPct > p.stickDeadbandPct)) {
    s.spMs = 0.0f;
    s.integPct = 0.0f;
    s.outPct = 0.0f;
    return 0.0f;
  }

  // 2–3. Target and setpoint ramp.
  const float target = govTargetMs(stickPct, limitMs, p);
  if (s.spMs < target) {
    s.spMs += p.accelMs2 * dt;
    if (s.spMs > target) s.spMs = target;
  } else if (s.spMs > target) {
    s.spMs -= p.decelMs2 * dt;
    if (s.spMs < target) s.spMs = target;
  }

  // 4–5. PI with the start throttle limit and conditional integration.
  const float e = s.spMs - speedMs;
  const float P = p.kpPctPerMs * e;
  const float uMax = (speedMs < p.standstillMs) ? govClamp(p.startCapPct, 0.0f, 100.0f) : 100.0f;
  // Output from the integrator as it stood (design D2 step 4), then integrate (step 5).
  const float sum = P + s.integPct;
  const float u = govClamp(sum, 0.0f, uMax);
  if (!((sum >= uMax && e > 0.0f) || (sum <= 0.0f && e < 0.0f))) {
    s.integPct += p.kiPctSPerMs * e * dt;
  }
  s.integPct = govClamp(s.integPct, 0.0f, uMax);

  // 6. Output rate.
  const float maxStep = p.outSlewPctS * dt;
  s.outPct += govClamp(u - s.outPct, -maxStep, maxStep);
  s.outPct = govClamp(s.outPct, 0.0f, 100.0f);
  return s.outPct;
}

/** @brief Hand-over blend out of ACTIVE (design D8): move toward the target at ≤ slew × dt. */
static inline float govBlendStep(float blendPct, float targetPct, float slewPctS, float dtS) {
  const float dt = govClamp(isnan(dtS) ? 0.0f : dtS, 0.0f, 1.0f);
  const float maxStep = slewPctS * dt;
  return blendPct + govClamp(targetPct - blendPct, -maxStep, maxStep);
}

/** @brief The blend has reached today's value (within 0.5 %). */
static inline bool govBlendDone(float blendPct, float targetPct) {
  return fabsf(targetPct - blendPct) < 0.5f;
}

/** @brief Raw at-idle saturation condition (design D6), before the hold timers. */
static inline bool govSatRaw(float outPct, float speedMs, float spMs, float marginMs) {
  return (outPct <= 0.5f) && (speedMs > spMs + marginMs);
}

// ── Settings validation (ranges common to every vehicle, design D5 / D12) ─────
// Mirrored literals: Constants.h static_asserts that GOV_*_MIN / _MAX match these.
#define GOV_LAW_KP_MIN      0.2f
#define GOV_LAW_KP_MAX      10.0f
#define GOV_LAW_KI_MIN      0.0f
#define GOV_LAW_KI_MAX      5.0f
#define GOV_LAW_ACC_MIN     0.3f
#define GOV_LAW_ACC_MAX     3.0f
#define GOV_LAW_DEC_MIN     0.5f
#define GOV_LAW_DEC_MAX     5.0f
#define GOV_LAW_LAUNCH_MIN  1.0f
#define GOV_LAW_LAUNCH_MAX  40.0f

static inline bool govInRange(float v, float lo, float hi) {
  return !isnan(v) && v >= lo && v <= hi;
}
static inline bool govKpValid(float v) { return govInRange(v, GOV_LAW_KP_MIN, GOV_LAW_KP_MAX); }
static inline bool govKiValid(float v) { return govInRange(v, GOV_LAW_KI_MIN, GOV_LAW_KI_MAX); }
static inline bool govAccValid(float v) { return govInRange(v, GOV_LAW_ACC_MIN, GOV_LAW_ACC_MAX); }
static inline bool govDecValid(float v) { return govInRange(v, GOV_LAW_DEC_MIN, GOV_LAW_DEC_MAX); }
static inline bool govLaunchValid(float v) {
  return govInRange(v, GOV_LAW_LAUNCH_MIN, GOV_LAW_LAUNCH_MAX);
}

/** @brief A stored value if valid, else the build's default. */
static inline float govOrDefault(float value, bool (*valid)(float), float def) {
  return valid(value) ? value : def;
}
