// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// V/S, FPA, V/S reversion, TCAS, ALT soft mode (vertical_vs_tcas.cpp): block memories, and inputs from the parts of the FMGC not rewritten yet.

#pragma once

#include "blocks.h"

namespace fmgc::readable {

struct VsTcasState {};

/** Not-yet-rewritten FMGC signals this group needs (filled in src/external_vs_tcas.h). */
struct VsTcasExternal {};

}  // namespace fmgc::readable
