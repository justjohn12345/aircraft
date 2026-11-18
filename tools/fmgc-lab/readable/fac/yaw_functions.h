// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's three rudder functions: yaw damper, rudder trim and rudder travel limiter (generated FacComputer.cpp
// 670-689 engagement, 1102-1238 laws).
//
// Engagement, the same for each function: it "can engage" with the FAC pushbutton on and its actuator available (the
// yaw damper: hydraulic pressure, green for FAC 1, yellow for FAC 2; the trim and limiter actuators are always healthy
// in the sim), and it engages when it can and FAC 1 is this unit or FAC 1's function is not engaged (FAC 1 has
// priority; there is no re-engagement delay or monitoring).
//
// Laws (ELAC words: discrete status word 1 bit 20 = yaw damper command valid? (used with bit 26), bit 26 = ELAC in
// normal law; see the .cpp):
//   yaw damper      the ELAC's yaw damper order when an ELAC in normal law provides one, else the FAC's own
//                   (alternate) law: washed-out yaw rate, +-5 deg; then +-20 deg and 40 deg/s; held at the measured
//                   position (0 in the sim) while not engaged
//   rudder trim     an integrator of the trim switches (1 deg/s) or, with an AP engaged above 200 ft radio height, of
//                   the ELAC yaw damper order x 1/15 (the trim takes over a steady yaw damper order, at most 1.7
//                   deg/s), +-20 deg, then 2 deg/s; the RESET button (no AP) zeroes it; not engaged: follows the
//                   measured trim position
//   travel limiter  the rudder travel limit for the indicated speed (25 deg up to 160 kt, 3.4 deg from 380 kt), 1 deg/s;
//                   not engaged: follows the measured limiter position

#pragma once

#include "FacComputer_types.h"
#include "fac_blocks.h"
#include "sensors.h"

namespace fac::readable {

/** Rudder trim: switch rate (deg/s), auto-trim gain on the ELAC yaw damper order (1/s) and its limit, range, rate. */
constexpr double kTrimSwitchRateDegS = 1.0;
constexpr double kAutoTrimGain = 0.066666666666666666;  // 1/15
constexpr double kAutoTrimRateLimitDegS = 1.7;
constexpr double kAutoTrimMinRadioHeightFt = 200.0;
constexpr double kTrimRangeDeg = 20.0;
constexpr double kTrimOrderRateDegS = 2.0;
/** Yaw damper: washout of the yaw rate (rad/s), alternate law limit, overall limit and rate. */
constexpr double kYawDamperWashoutC1 = 20.0;
constexpr double kAlternateYawDamperLimitDeg = 5.0;
constexpr double kYawDamperLimitDeg = 20.0;
constexpr double kYawDamperRateDegS = 40.0;
/** Rudder travel limiter: rate of the limit (deg/s), range. */
constexpr double kTravelLimitRateDegS = 1.0;
constexpr double kTravelLimitMaxDeg = 30.0;

/** Engagement of the three functions (logic bus fields). */
struct YawEngagement {
  bool yawDamperCanEngage, yawDamperHasPriority, yawDamperEngaged;
  bool rudderTrimCanEngage, rudderTrimHasPriority, rudderTrimEngaged;
  bool travelLimCanEngage, travelLimHasPriority, travelLimEngaged;

  static YawEngagement compute(const base_fac_discrete_inputs& d);
};

/** The ELAC words the yaw functions read. */
struct ElacYawCommand {
  bool available;     // an ELAC in normal law with bit 20 (FAC yaw damper source), healthy
  double commandDeg;  // its yaw damper order
};

class RudderTrim {
 public:
  void reset();

  /** Generated 1102-1171. Returns the rudder trim order (laws.rudder_trim_command_deg). */
  double update(const fac_inputs& in, const Sensors& s, bool engaged);

  // state access for the checker
  bool resetLatch() const { return resetLatch_.q; }
  const blocks::EdgeDetector& resetButtonEdge() const { return resetButton_; }
  double integrator() const { return integrator_; }
  bool reload() const { return reload_; }
  const blocks::RateLimiterFrom& orderRate() const { return order_; }

  /** Restart (Runtime_MODE): the latch cleared and the integrator reloaded on the next frame; the integrator value
   *  itself is not reset. */
  void restart();

 private:
  blocks::SrLatch resetLatch_;          // Memory_PreviousInput: RESET requested, until a trim switch or an AP
  blocks::EdgeDetector resetButton_;    // previousInput / previousInput_not_empty
  double integrator_ = 0.0;             // Delay_DSTATE_d
  bool reload_ = true;                  // icLoad
  blocks::RateLimiterFrom order_;       // sf_RateLimiter_l
};

class YawDamper {
 public:
  void reset();

  /** Generated 1172-1209. Returns the yaw damper order (laws.yaw_damper_command_deg). */
  double update(const fac_inputs& in, const Sensors& s, bool engaged);

  const blocks::WashoutFilter& washout() const { return washout_; }
  const blocks::RateLimiterFrom& orderRate() const { return order_; }

 private:
  blocks::WashoutFilter washout_;   // sf_WashoutFilter_i
  blocks::RateLimiterFrom order_;   // sf_RateLimiter_f
};

class TravelLimiter {
 public:
  void reset();

  /** Generated 1210-1222. Returns the rudder travel limit order (laws.rudder_travel_lim_command_deg). */
  double update(const fac_inputs& in, const Sensors& s, bool engaged);

  const blocks::RateLimiterFrom& orderRate() const { return order_; }

 private:
  blocks::RateLimiterFrom order_;  // sf_RateLimiter_p
};

}  // namespace fac::readable
