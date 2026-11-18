// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchDirectLaw (generated PitchDirectLaw.cpp), stepped by the ELAC and the SECs. Skeleton: the public API is
// fixed; the implementation is to be written.

#pragma once

#include "pitch_laws.h"

namespace efcs::readable {

class PitchDirectLaw {
 public:
  void reset();
  PitchLawOutputs step(const PitchDirectLawInputs& in);
};

}  // namespace efcs::readable
