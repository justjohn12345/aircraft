// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's alpha floor detection (generated FacComputer.cpp 722-831): the filtered angle of attack, the alpha floor
// threshold for Mach and configuration, the speed-rate anticipation, the flight phase chart that inhibits alpha floor
// on the ground and below 100 ft in approach, and the alpha floor latch sent to the FMGCs (FAC discrete word 5 bit 29).
//
//   set     not inhibited, Mach < 0.6, an ELAC in normal law, and the filtered alpha above the threshold (lowered by
//           half the deceleration in kt/s, at most 3 deg, in CONF 3 and FULL only)
//   reset   inhibited, or no ELAC high angle of attack protection active, or no ELAC in normal law
//   (set is evaluated first: while the set condition holds the latch stays on)

#pragma once

#include <cstdint>

#include "FacComputer_types.h"
#include "fac_blocks.h"
#include "sensors.h"

namespace fac::readable {

/** Filtered alpha (rad/s), speed-rate filter (rad/s), threshold rate (deg/s). */
constexpr double kAlphaFilterC1 = 0.5;
constexpr double kSpeedRateFilterC1 = 0.4;
constexpr double kAlphaThresholdRateDegS = 1.0;
/** Inhibited above this Mach; radio height of the approach inhibition (ft), and when the own FMGC's RA failed. */
constexpr double kAlphaFloorMaxMach = 0.6;
constexpr double kAlphaFloorInhibitRaFt = 100.0;
constexpr double kRaFailedDefaultFt = 200.0;
/** Speed-rate anticipation in CONF 3 and FULL (config index >= 4): at most this much lower threshold (deg). */
constexpr double kMaxAnticipationDeg = 3.0;

/** The flight phase chart (Stateflow chart c15): alpha floor is inhibited in Landed and Landing100ft. */
enum class AlphaFloorPhase : uint8_t { None = 0, Flying = 1, Landed = 2, Landing100ft = 3, Takeoff100ft = 4 };

class AlphaFloor {
 public:
  void reset();

  /** One frame. Returns the filtered alpha (flight_envelope.alpha_filtered_deg, used by the speeds too). */
  double update(const fac_inputs& in, const Sensors& s);

  bool active() const { return latched_; }
  double alphaFilteredDeg() const { return alphaFiltered_; }

  // state access for the checker
  const blocks::LagFilter<double>& alphaFilter() const { return alphaFilter_; }
  const blocks::RateLimiter& threshold() const { return threshold_; }
  const blocks::LagFilter<double>& speedRateFilter() const { return speedRate_; }
  double previousHalfCas() const { return previousHalfCas_; }
  AlphaFloorPhase phase() const { return phase_; }
  bool chartActive() const { return chartActive_; }

  /** Restart (Runtime_MODE): filters, chart and latch reset; the previous CAS sample to 0. */
  void restart();

 private:
  blocks::LagFilter<double> alphaFilter_;  // sf_LagFilter_c
  blocks::RateLimiter threshold_;          // sf_RateLimiter
  blocks::LagFilter<double> speedRate_;    // sf_LagFilter (derivative of 0.5 x CAS)
  double previousHalfCas_ = 0.0;           // Delay_DSTATE
  bool chartActive_ = false;               // is_active_c15_FacComputer
  AlphaFloorPhase phase_ = AlphaFloorPhase::None;
  bool latched_ = false;                   // sAlphaFloor
  double alphaFiltered_ = 0.0;
};

}  // namespace fac::readable
