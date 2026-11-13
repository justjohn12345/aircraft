// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SPD_MACH (speed on pitch: CLB, DES, OP CLB/DES, EXP) and SRS laws. Not rewritten yet: commands stay NaN and the checker skips them.

#include "laws.h"

namespace fmgc::readable::laws {

void VerticalLaws::resetSpeed() {}

void VerticalLaws::computeSpeed(const ap_laws_output&, LawResults&) {}

}  // namespace fmgc::readable::laws
