// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// CLB, DES, OP CLB, OP DES, EXP CLB, EXP DES and their armed modes (vertical_climb_descent.cpp): block memories, and inputs from the parts of the FMGC not rewritten yet.

#pragma once

#include "blocks.h"

namespace fmgc::readable {

struct ClimbDescentState {};

/** Not-yet-rewritten FMGC signals this group needs (filled in src/external_climb_descent.h). */
struct ClimbDescentExternal {};

}  // namespace fmgc::readable
