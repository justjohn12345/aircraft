// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable LateralDirectLaw (generated LateralDirectLaw.cpp, 45 lines): aileron order = -25 deg x roll stick at no more
// than 50 deg/s, no yaw order. Stepped every frame by the ELAC (roll direct law) and by every SEC (on the speed brake
// command, its output unused there).

#pragma once

#include "efcs_blocks.h"
#include "lateral_laws.h"

namespace efcs::readable {

class LateralDirectLaw {
 public:
  void reset() { xiRate_.reset(); }

  LateralLawOutputs step(const LateralDirectLawInputs& in) {
    LateralLawOutputs out;
    out.zeta_deg = 0.0;
    out.xi_deg = xiRate_.update(-25.0 * in.delta_xi_pos, 50.0, -50.0, in.time_dt, 0.0);
    return out;
  }

  const blocks::RateLimiterInit& xiRate() const { return xiRate_; }

 private:
  blocks::RateLimiterInit xiRate_;  // pY / pY_not_empty, starts at 0
};

}  // namespace efcs::readable
