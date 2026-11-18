// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchAlternateLaw (generated PitchAlternateLaw.cpp), stepped by the ELAC (alternate law 1 and 2) and by
// SECs 1 and 2. Skeleton: the public API is fixed; the implementation is to be written.

#pragma once

#include "pitch_laws.h"

namespace efcs::readable {

class PitchAlternateLaw {
 public:
  void init();
  void reset();
  PitchLawOutputs step(const PitchAlternateLawInputs& in);
};

}  // namespace efcs::readable
