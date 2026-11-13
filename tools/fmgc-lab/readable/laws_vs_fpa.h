// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// V/S and FPA laws (laws_vs_fpa.cpp): block memories.

#pragma once

#include "blocks.h"
#include "laws_common.h"

namespace fmgc::readable::laws {

/**
 * Level-off detection of the V/S and FPA laws: true while the target is 0 after the law was entered with a 0 target,
 * or after the target went to 0 from a climb target above the threshold. Selects the higher pitch rate limit.
 */
struct LevelOffDetector {
  double previousLaw = 0;     // vertical_law last frame
  double previousTarget = 0;  // target (fpm or deg) last frame
  bool primed = false;        // previousLaw/previousTarget valid (generated: prevVerticalLaw/prevTarget_not_empty)
  bool active = false;        // generated: islevelOffActive

  void reset() {
    primed = false;
    active = false;
  }
};

struct VsFpaLawState {
  SpeedOnPitchFilters fpaVls, fpaVmax;
  LevelOffDetector fpaLevelOff;
  SpeedOnPitchFilters vsVls, vsVmax;
  LevelOffDetector vsLevelOff;
};

}  // namespace fmgc::readable::laws
