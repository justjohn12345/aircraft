// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// G/S, VPATH (FINAL and DES on path) and FLARE laws. Not rewritten yet: commands stay NaN and the checker skips them.

#include "laws.h"

namespace fmgc::readable::laws {

void VerticalLaws::resetPath() {}

void VerticalLaws::computePath(const ap_laws_output&, LawResults&) {}

}  // namespace fmgc::readable::laws
