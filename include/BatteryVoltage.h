// BatteryVoltage.h — 12 V battery sense on X10 / GPIO9 (aztec: no Buck-Boost board, the
// battery + is wired to X10 pin 3 through the on-board 200K/24K divider)
// (change repurpose-x10-battery-sense).
//
// NO Arduino / Constants.h dependency: this header is compiled by the native unit
// tests (test/test_battery_voltage). Callers pass the constants in through BatParams.
//
//  raw V  = averaged ADC mV / 1000 × dividerRatio          (before user calibration)
//  V      = raw V × cal                                     (cal = user correction factor)
//  present: raw V > presentMinV (below that X10 is open / nothing is wired)
//  low    : hysteresis — set when V < lowOnV, cleared only when V > lowOffV
//  cal    : cal = measured V (multimeter) / raw V, accepted only when measured is in
//           [measuredMinV, measuredMaxV], the battery is present and the factor lands
//           in [calMin, calMax]. Anything else is refused and the old factor is kept.
#pragma once

#include <math.h>
#include <stdint.h>

/** @brief Tunables (volts unless noted) */
struct BatParams {
  float dividerRatio;  // 224/24 = 9.33 (BAT12_DIVIDER_RATIO)
  float presentMinV;   // raw below this = nothing connected (BAT12_PRESENT_MIN_V)
  float lowOnV;        // low set below this (BAT12_LOW_V)
  float lowOffV;       // low cleared above this (BAT12_LOW_CLEAR_V)
  float calMin;        // accepted factor range (BAT12_CAL_MIN / BAT12_CAL_MAX)
  float calMax;
  float measuredMinV;  // accepted multimeter reading range (BAT12_CAL_MEASURED_MIN_V / _MAX_V)
  float measuredMaxV;
};

/** @brief Outcome of batComputeCal() */
enum class BatCalResult : uint8_t {
  OK,
  MEASURED_OUT_OF_RANGE,  // multimeter value NaN or outside [measuredMinV, measuredMaxV]
  NOT_CONNECTED,          // raw reading ≤ presentMinV: nothing on X10, nothing to calibrate
  FACTOR_OUT_OF_RANGE     // measured / raw outside [calMin, calMax]: wrong value typed or wiring fault
};

/** @brief Divider-scaled voltage before calibration */
inline float batRawVolts(float adcMv, const BatParams& p) {
  return (adcMv / 1000.0f) * p.dividerRatio;
}

/** @brief Calibrated battery voltage */
inline float batVolts(float rawV, float cal) { return rawV * cal; }

/** @brief True when something is wired to X10 (raw reading above presentMinV) */
inline bool batPresent(float rawV, const BatParams& p) { return rawV > p.presentMinV; }

/** @brief True when a stored / proposed factor may be used */
inline bool batCalValid(float cal, const BatParams& p) {
  return !isnan(cal) && cal >= p.calMin && cal <= p.calMax;
}

/** @brief Low-voltage flag with hysteresis: returns the new flag state */
inline bool batLowStep(bool wasLow, float v, const BatParams& p) {
  if (isnan(v)) return wasLow;
  if (wasLow) return !(v > p.lowOffV);
  return v < p.lowOnV;
}

/**
 * @brief Compute the calibration factor from a multimeter reading and the CURRENT raw value.
 * @param calOut written only on OK
 */
inline BatCalResult batComputeCal(float measuredV, float rawV, const BatParams& p, float& calOut) {
  if (isnan(measuredV) || measuredV < p.measuredMinV || measuredV > p.measuredMaxV) {
    return BatCalResult::MEASURED_OUT_OF_RANGE;
  }
  if (isnan(rawV) || !batPresent(rawV, p)) {
    return BatCalResult::NOT_CONNECTED;
  }
  float cal = measuredV / rawV;
  if (!batCalValid(cal, p)) {
    return BatCalResult::FACTOR_OUT_OF_RANGE;
  }
  calOut = cal;
  return BatCalResult::OK;
}
