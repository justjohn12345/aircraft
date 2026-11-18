// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchDirectLaw (generated PitchDirectLaw.cpp 23-54). See pitch_direct_law.h.

#include "pitch_direct_law.h"

namespace efcs::readable {

void PitchDirectLaw::reset() {
  eta_.reset();  // 23-26
}

PitchLawOutputs PitchDirectLaw::step(const PitchDirectLawInputs& in) {
  // 28-54
  PitchLawOutputs out;
  out.eta_trim_dot_deg_s = 0.0;
  out.eta_trim_limit_up = 3.5;
  out.eta_trim_limit_lo = -11.0;
  out.eta_deg = eta_.update(blocks::limit(-30.0 * in.delta_eta_pos, -30.0, 17.0), 45.0, -45.0, in.time_dt, 0.0);
  return out;
}

}  // namespace efcs::readable
