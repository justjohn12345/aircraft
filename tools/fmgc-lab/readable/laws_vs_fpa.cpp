// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// V/S and FPA laws. Not rewritten yet: commands stay NaN and the checker skips them.

#include "laws.h"

namespace fmgc::readable::laws {

void VerticalLaws::resetVsFpa() {}

void VerticalLaws::computeVsFpa(const ap_laws_output&, LawResults&) {}

}  // namespace fmgc::readable::laws
