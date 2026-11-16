// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ALT window (generated FcuComputer.cpp 934-954). See windows.h.

#include <cmath>

#include "windows.h"

namespace fcu::readable {

void AltWindow::reset() {
  memoryPrimed_ = false;
}

float AltWindow::update(const fcu_inputs& in, const FmgcView& fmgc) {
  const auto& afs = in.discrete_inputs.afs_inputs;
  const float simValue = in.sim_input.alt;
  // knob increment (ft): 100, or 1000 with the 100/1000 selector at 1000
  const float increment = afs.alt_increment_1000 ? 1000.0F : 100.0F;
  // the turn is worth half an increment plus 1 ft, so that the rounding below lands on the next multiple
  const float clickFt = increment / 2.0F + 1.0F;

  if (!memoryPrimed_) {
    memory_ = std::round(fmgc.data.alt_ft / increment) * increment;
    memoryPrimed_ = true;
  }
  if (simValue != kNoSimInput) {
    memory_ = simValue;
  }
  if (simValue != kNoSimInput || afs.alt_knob.turns != 0) {
    memory_ = std::round((clickFt * static_cast<float>(afs.alt_knob.turns) + memory_) / increment) * increment;
  }
  memory_ = std::fmax(std::fmin(memory_, kMaxAltFt), kMinAltFt);
  return memory_;
}

}  // namespace fcu::readable
