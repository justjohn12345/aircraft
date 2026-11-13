// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SRS, LAND, FLARE, G/S, FINAL (vertical_approach.cpp): block memories, and inputs from the parts of the FMGC not rewritten yet.

#pragma once

#include "blocks.h"

namespace fmgc::readable {

struct ApproachState {};

/** Not-yet-rewritten FMGC signals this group needs (filled in src/external_approach.h). */
struct ApproachExternal {};

}  // namespace fmgc::readable
