// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchAlternateLaw (generated PitchAlternateLaw.cpp), stepped by the ELAC (alternate law 1 and 2,
// LawMDLOBJ3) and by SECs 1 and 2 (LawMDLOBJ2). Bit-exact with the generated law (src/check_pitch_laws.cpp).
//
// The alternate law is the normal law's in-flight part without protections, autopilot, flare and ground law: the
// elevator integrates the median of three C* channels (pitch_law_blocks.h): nz max, the stick load factor demand with
// the low and high speed stabilities (when available), nz min; times a gain by flap lever position. On the ground (and
// in tracking mode) the integrator follows the elevator position (in flight) or the stick (on ground). Auto trim: the
// integrator / 13 deg/s, frozen outside 0.5..1.25 g or beyond 30 deg of bank; the THS limits are fixed.
//
// Inputs from the ELAC (ElacComputer.cpp 1471-1478): eta_deg and both spoiler positions are constant 0.

#pragma once

#include "pitch_law_blocks.h"
#include "pitch_laws.h"

namespace efcs::readable {

/** Every memory of the alternate law, named; generated names in the comments. */
struct PitchAlternateLawState {
  pitch_blocks::TrimFreezeChart trimFreeze;      // c9
  pitch_blocks::FlightPhaseChart flightPhase;    // c7
  pitch_blocks::AutoTrimModeChart autoTrimMode;  // c8 (unused)
  pitch_blocks::RateLimiterInit nzMax;           // sf_RateLimiter (starts at 2 g)
  pitch_blocks::RateLimiterInit nzMin;           // sf_RateLimiter_n
  pitch_blocks::RateLimiterInit stick;           // pY / pY_not_empty
  pitch_blocks::CStarFilters cstar;              // 3 generated copies each
  pitch_blocks::CStarChannel nzMaxChannel;       // Delay_DSTATE_k
  pitch_blocks::CStarChannel stickChannel;       // Delay_DSTATE_j
  pitch_blocks::CStarChannel nzMinChannel;       // Delay_DSTATE_g
  pitch_blocks::Integrator integrator;           // Delay_DSTATE_o, icLoad
  pitch_blocks::RateLimiterInit eta;             // sf_RateLimiter_b
};

class PitchAlternateLaw {
 public:
  void init();
  void reset();
  PitchLawOutputs step(const PitchAlternateLawInputs& in);

  const PitchAlternateLawState& state() const { return s_; }

 private:
  PitchAlternateLawState s_;
};

}  // namespace efcs::readable
