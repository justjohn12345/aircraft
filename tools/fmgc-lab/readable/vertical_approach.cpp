// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Not rewritten yet: the checker uses the generated values for this group's signals.

#include "vertical.h"

namespace fmgc::readable {

void VerticalModeLogic::resetApproach(Signals&) {}

bool VerticalModeLogic::updateApproach(Signal, const Frame&, Signals&) {
  return false;
}

}  // namespace fmgc::readable
