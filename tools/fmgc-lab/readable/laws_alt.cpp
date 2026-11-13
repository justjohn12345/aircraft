// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ALT_ACQ (ALT*) and ALT_HOLD laws. Not rewritten yet: commands stay NaN and the checker skips them.

#include "laws.h"

namespace fmgc::readable::laws {

void VerticalLaws::resetAlt() {}

void VerticalLaws::computeAlt(const ap_laws_output&, LawResults&) {}

}  // namespace fmgc::readable::laws
