// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchNormalLaw (generated fbw-a32nx/src/wasm/fbw_a320/src/model/PitchNormalLaw.cpp). Skeleton: the public API
// is fixed (the readable ELAC calls it); the implementation is to be written.

#pragma once

#include "pitch_laws.h"

namespace efcs::readable {

class PitchNormalLaw {
 public:
  /** PitchNormalLaw::init(): called once by ElacComputer::initialize(). */
  void init();
  /** PitchNormalLaw::reset(): called on every ELAC restart (first running frame). */
  void reset();
  PitchLawOutputs step(const PitchNormalLawInputs& in);
};

}  // namespace efcs::readable
