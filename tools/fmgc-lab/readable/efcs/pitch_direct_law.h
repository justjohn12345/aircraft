// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchDirectLaw (generated PitchDirectLaw.cpp), stepped by the ELAC (LawMDLOBJ4) and the SECs
// (LawMDLOBJ3). Bit-exact with the generated law (src/check_pitch_laws.cpp).
//
// The elevator follows the stick: -30 deg x stick, limited to -30 .. +17 deg, rate limited 45 deg/s (starting at 0
// after a reset). No auto trim (rate 0); THS limits -11 / +3.5 deg. The generated law has no init().

#pragma once

#include "efcs_blocks.h"
#include "pitch_laws.h"

namespace efcs::readable {

class PitchDirectLaw {
 public:
  void reset();
  PitchLawOutputs step(const PitchDirectLawInputs& in);

  /** The elevator rate limiter (generated pY / pY_not_empty). */
  const blocks::RateLimiterInit& eta() const { return eta_; }

 private:
  blocks::RateLimiterInit eta_;
};

}  // namespace efcs::readable
