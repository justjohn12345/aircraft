// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SPD_MACH (speed on pitch: CLB, DES, OP CLB/DES, EXP) and SRS laws (laws_speed.cpp): block memories.

#pragma once

#include "blocks.h"
#include "laws_common.h"

namespace fmgc::readable::laws {

struct SpeedLawState {
  SpeedOnPitchFilters spdMach;  // generated sf_WashoutFilter_h, sf_LeadLagFilter_c, sf_LeadLagFilter_oer
  SpeedOnPitchFilters srs;      // generated sf_WashoutFilter_eq, sf_LeadLagFilter_k, sf_LeadLagFilter_ay
};

}  // namespace fmgc::readable::laws
