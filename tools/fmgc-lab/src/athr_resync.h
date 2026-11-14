// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Resync of the readable autothrust state from the generated model's state at the start of a step (check_athr resync
// mode): each readable latch/memory takes the value the generated one had before the step, so a mismatch points at the
// step where the logic differs. Filled in alongside the readable state (readable/athr.h ModesState, N1State); state
// that has no single generated counterpart is left free-running.

#pragma once

#include "FmgcComputer.h"
#include "athr.h"

inline void resyncAthrModes(fmgc::readable::athr::ModesState& /*s*/, const FmgcComputer::D_Work_FmgcComputer_T& /*pre*/) {}

inline void resyncAthrN1(fmgc::readable::athr::N1State& /*s*/, const FmgcComputer::D_Work_FmgcComputer_T& /*pre*/) {}
