// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable LateralNormalLaw (generated fbw-a32nx/src/wasm/fbw_a320/src/model/LateralNormalLaw.cpp, 498 lines), the
// ELAC's roll and yaw normal law. Bit-exact (src/check_lateral_laws.cpp, and in situ by src/check_elac.cpp).
//
// Roll: the sidestick commands a roll rate (15 deg/s at full deflection) that a bank angle integrator turns into a
// bank angle command, held when the stick is released (rate command / attitude hold). Above 33 deg of bank the
// protection feeds back a rate towards 33 deg (to 0 deg with high speed protection, 45 with high AoA protection), and
// the integrator is limited to +-67 deg. The aileron order follows from a roll attitude law (gains from a simple roll
// model at the current dynamic pressure, scheduled with dt). On the ground the order is the direct law (-25 deg x
// stick); the two are blended over 0.5 s by the flight/ground mode. With an AP engaged the bank command is the AP's.
// Yaw: turn coordination and yaw damping in flight (yaw rate minus the turn rate of the commanded bank), a yaw rate
// feedback on the ground above 40 kt, plus the AP's sideslip order.

#pragma once

#include <cstdint>

#include "efcs_blocks.h"
#include "lateral_laws.h"

namespace efcs::readable {

class LateralNormalLaw {
 public:
  /** The flight/ground mode chart (is_c5_LateralNormalLaw). */
  enum class Mode : uint8_t { None = 0, Flight = 1, Ground = 2 };

  /** init() (175-180): called once by ElacComputer::initialize(); the rest of the state is zero from construction. */
  void init();
  /** reset() (182-194): every ELAC restart. The bank integrator keeps its value (it is reloaded instead). */
  void reset();
  LateralLawOutputs step(const LateralNormalLawInputs& in);

  // state, for the differential checker
  bool modeChartActive() const { return modeChartActive_; }
  Mode mode() const { return mode_; }
  const blocks::RateLimiterInit& inFlightRate() const { return inFlightRate_; }
  const blocks::RateLimiterInit& pedalFeedForwardRate() const { return pedalFeedForwardRate_; }
  const blocks::RateLimiterInit& apEngagedRate() const { return apEngagedRate_; }
  const blocks::RateLimiterInit& xiOrderRate() const { return xiOrderRate_; }
  const blocks::RateLimiterInit& airborneRate() const { return airborneRate_; }
  double previousRollRateDemand() const { return previousRollRateDemand_; }
  double previousXiCommand() const { return previousXiCommand_; }
  double bankCommand() const { return bankCommand_; }
  bool reloadBankCommand() const { return reloadBankCommand_; }

 private:
  /** The flight/ground mode chart (230-246): 1 in flight, 0 on the ground. */
  int updateMode(const LateralNormalLawInputs& in);
  /** Roll rate demand -> bank angle command (356-412). `blend`: the in-flight blend (0 ground .. 1 flight). */
  double bankAngleCommand(const LateralNormalLawInputs& in, double inFlightBlend, double omega0);

  bool modeChartActive_ = false;                    // is_active_c5_LateralNormalLaw
  Mode mode_ = Mode::None;                          // is_c5_LateralNormalLaw
  blocks::RateLimiterInit inFlightRate_;            // sf_RateLimiter: flight mode 0/1, 2/s
  blocks::RateLimiterInit pedalFeedForwardRate_;    // sf_RateLimiter_d: rudder pedal feed-forward, output unused
  blocks::RateLimiterInit apEngagedRate_;           // pY / pY_not_empty: AP engaged 0/1, 2/s
  blocks::RateLimiterInit xiOrderRate_;             // sf_RateLimiter_j: aileron order, 50 deg/s
  blocks::RateLimiterInit airborneRate_;            // pY_b / pY_not_empty_j: !on_ground 0/1, 2/s
  double previousRollRateDemand_ = 0.0;             // Delay_DSTATE_o (between steps)
  double previousXiCommand_ = 0.0;                  // Delay_DSTATE: unlimited aileron command
  double bankCommand_ = 0.0;                        // Delay_DSTATE_d: the bank angle integrator
  bool reloadBankCommand_ = false;                  // icLoad
};

}  // namespace efcs::readable
