// ============================================================================
// TransmissionLaw.h — pure logic of the transmission gear sync (change
// fix-transmission-gear-sync): the choice of initialisation source (physical sensor, else a
// plausible NVS state, else an initialisation step into NEUTRAL), the overshoot-direction hint
// for that step, and the idle resync consecutive-mismatch counter; plus (change
// speed-up-gear-change-with-sensor) the sensor confirmation that ends the gear-change waits.
//
// NO Arduino / Constants.h dependency: compiled by the native unit tests
// (test/test_transmission_sync). TransmissionController feeds the sensor reading, the gear
// positions and the NVS record in and applies the decision.
//
// Units: servo positions in percent (0.0–100.0). Gear values mirror
// TransmissionController::Gear (static_assert in src/TransmissionController.cpp).
// ============================================================================
#pragma once

#include <stdint.h>

enum class TransGear : int {
  GEAR_HIGH = 0,
  GEAR_LOW = 1,
  GEAR_NEUTRAL = 2,
  GEAR_REVERSE = 3,
  GEAR_UNKNOWN = 4,
};

enum class TransInitSource : uint8_t {
  SENSOR,           // the physical sensor reads exactly one gear
  NVS,              // sensor UNKNOWN, NVS state valid and plausible
  STEP_TO_NEUTRAL,  // neither: real shift step into NEUTRAL
};

struct TransInitDecision {
  TransInitSource source;
  TransGear gear;   // expected gear after initialisation (NEUTRAL for the step)
  float pct;        // servo position to command (SENSOR / NVS) or the NEUTRAL position (step)
  float direction;  // STEP_TO_NEUTRAL: overshoot direction, +1 (toward H) or −1 (toward R)
};

// A saved position equal to the NEUTRAL position within this tolerance counts as "at N". The
// saved state_pct is read back from servo microseconds (1 µs ≈ 0.07 %), so an exact float
// compare against the configured N position would miss.
static constexpr float TRANS_LAW_PCT_EQ_TOL = 0.1f;

inline bool transGearIsReal(TransGear g) {
  int v = (int)g;
  return v >= (int)TransGear::GEAR_HIGH && v <= (int)TransGear::GEAR_REVERSE;
}

// 0.0–100.0 and a number (NaN fails both comparisons, ±inf fails the range).
inline bool transPctIsPlausible(float pct) {
  return pct >= 0.0f && pct <= 100.0f;
}

// Overshoot direction of the initialisation step from the (invalid) NVS record. The interrupted
// step saved its target as state_gear and the servo position at its start as state_pct, so the
// lever lies between them: travel toward N from state_pct, or, when state_pct is N (the step
// started at N), toward N from state_gear's position. Anything unusable → +1 (toward H).
inline float transInitStepDirection(const float gearPositions[4], bool nvsFound, TransGear nvsGear,
                                    float nvsPct) {
  if (!nvsFound || !transPctIsPlausible(nvsPct)) return 1.0f;
  const float nPct = gearPositions[(int)TransGear::GEAR_NEUTRAL];
  float diff = nPct - nvsPct;
  if (diff > TRANS_LAW_PCT_EQ_TOL) return 1.0f;
  if (diff < -TRANS_LAW_PCT_EQ_TOL) return -1.0f;
  // state_pct == N position: use the interrupted step's target gear.
  if (!transGearIsReal(nvsGear)) return 1.0f;
  float gearDiff = nPct - gearPositions[(int)nvsGear];
  if (gearDiff > TRANS_LAW_PCT_EQ_TOL) return 1.0f;
  if (gearDiff < -TRANS_LAW_PCT_EQ_TOL) return -1.0f;
  return 1.0f;
}

inline TransInitDecision chooseTransmissionInit(TransGear sensorGear, const float gearPositions[4],
                                                bool nvsFound, bool nvsValid, TransGear nvsGear,
                                                float nvsPct) {
  TransInitDecision d;
  d.direction = 1.0f;
  if (transGearIsReal(sensorGear)) {
    d.source = TransInitSource::SENSOR;
    d.gear = sensorGear;
    d.pct = gearPositions[(int)sensorGear];
    return d;
  }
  if (nvsFound && nvsValid && transGearIsReal(nvsGear) && transPctIsPlausible(nvsPct)) {
    d.source = TransInitSource::NVS;
    d.gear = nvsGear;
    d.pct = nvsPct;
    return d;
  }
  d.source = TransInitSource::STEP_TO_NEUTRAL;
  d.gear = TransGear::GEAR_NEUTRAL;
  d.pct = gearPositions[(int)TransGear::GEAR_NEUTRAL];
  d.direction = transInitStepDirection(gearPositions, nvsFound, nvsGear, nvsPct);
  return d;
}

// Idle resync: the sensor must read the same single gear, different from the expected gear, on
// `needed` consecutive checks. A check reading UNKNOWN or the expected gear resets the count; a
// different single gear than the previous mismatching check restarts it with that gear.
struct TransmissionResync {
  TransGear candidate = TransGear::GEAR_UNKNOWN;
  uint8_t count = 0;

  void reset() {
    candidate = TransGear::GEAR_UNKNOWN;
    count = 0;
  }

  // Returns true when the expected gear should be replaced by `sensorGear` (count is reset).
  bool step(TransGear sensorGear, TransGear targetGear, uint8_t needed) {
    if (!transGearIsReal(sensorGear) || sensorGear == targetGear) {
      reset();
      return false;
    }
    if (sensorGear != candidate) {
      candidate = sensorGear;
      count = 1;
    } else if (count < 255) {
      count++;
    }
    if (count >= needed) {
      reset();
      return true;
    }
    return false;
  }
};

// Sensor-confirmed gear-change wait (overshoot dwell and inter-step pause). The wait ends
// CONFIRMED once the sensor has matched the expected gear on every step for `confirmMs`,
// measured from the first matching step; any non-matching step (another gear or UNKNOWN)
// restarts the window. Otherwise it ends TIMEOUT `timeoutMs` after `startMs` (the fixed wait
// before this change). Times in ms (millis(), wrap-safe unsigned differences).
enum class ShiftConfirmResult : uint8_t {
  WAIT,
  CONFIRMED,
  TIMEOUT,
};

struct ShiftConfirm {
  bool armed = false;     // a confirmation window is running
  uint32_t sinceMs = 0;   // time of the first matching read of the running window

  void reset() {
    armed = false;
    sinceMs = 0;
  }

  ShiftConfirmResult step(uint32_t now, uint32_t startMs, bool sensorMatches, uint32_t confirmMs,
                          uint32_t timeoutMs) {
    if (!sensorMatches) {
      armed = false;
    } else if (!armed) {
      armed = true;
      sinceMs = now;
    }
    if (armed && now - sinceMs >= confirmMs) return ShiftConfirmResult::CONFIRMED;
    if (now - startMs >= timeoutMs) return ShiftConfirmResult::TIMEOUT;
    return ShiftConfirmResult::WAIT;
  }
};
