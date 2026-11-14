// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Lateral modes (lateral_modes.cpp): RWY, RWY TRK, ROLL OUT, ALIGN, GA TRK, HDG/TRK, NAV armed, NAV, LOC armed, LOC*,
// LOC, the HDG/TRK preset, law selection: block memories, and inputs from the parts of the FMGC not rewritten yet.

#pragma once

#include "FmgcComputer_types.h"
#include "blocks.h"
#include "targets.h"

namespace fmgc::readable {

struct LateralModesState {
  // RWY (LOC submode)
  blocks::ConfirmNode rwyTakeoff09s;      // SRS takeoff active for 0.9 s
  blocks::ConfirmNode rwyOnGround30s;     // on ground for 30 s
  blocks::ConfirmNode rwyLocSetRecent;    // RWY set condition true within the last 0.72 s

  // RWY TRK
  blocks::EdgeDetector rwyTrk30ftEdge;    // radio height reaches 30 ft
  blocks::Monostable rwyTrk30ftPulse;     // ... held for 0.9 s
  blocks::ConfirmNode rwyTrkSetRecent;    // RWY TRK set condition true within the last 0.72 s

  // ROLL OUT
  blocks::ConfirmNode rollOutCondition036s;  // touchdown condition true for 0.36 s

  // GA TRK
  blocks::Monostable gaTrkSrsGaPulse;     // 0.72 s after SRS go-around engages
  blocks::ConfirmNode gaTrkSetRecent;     // GA TRK set condition true within the last 0.72 s

  // HDG/TRK (and the lateral mode reversion)
  blocks::EdgeDetector revLandLost;       // falling edge of LAND armed or active
  blocks::Monostable revLocPush072s;      // 0.72 s after LOC pushed
  blocks::EdgeDetector revFinalLost;      // falling edge of FINAL armed or active
  blocks::EdgeDetector revLocPushLoc;     // LOC pushed (with LOC captured)
  blocks::EdgeDetector revLocPushFinal;   // LOC pushed (with NAV and FINAL)
  blocks::EdgeDetector hdgKnobPull;       // HDG/TRK knob pulled
  blocks::EdgeDetector hdgApFdEngage;     // first AP or FD engaged
  blocks::ConfirmNode hdgSetRecent;       // HDG/TRK set condition true within the last 0.72 s

  // NAV armed
  blocks::EdgeDetector navArmKnobPush;    // HDG/TRK knob pushed
  blocks::EdgeDetector navArmDirTo;       // FMS DIR TO NAV engage request
  blocks::EdgeDetector navArmFlightPlanValid;  // lateral flight plan becomes valid
  blocks::EdgeDetector navArmFinalArmed;  // FINAL armed (last frame)
  blocks::Monostable navArmSrsGaPulse;    // 0.72 s after SRS go-around engages
  blocks::EdgeDetector navArmPresetEdge;  // HDG/TRK preset becomes available
  blocks::EdgeDetector navArmKnobPull;    // HDG/TRK knob pulled
  blocks::EdgeDetector navArmLocEdge;     // LOC armed, LOC*, LOC or LAND engaged
  blocks::EdgeDetector navArmFinalLost;   // falling edge of FINAL armed (this frame)

  // NAV
  blocks::Monostable navDirToPulse;       // 0.72 s after a DIR TO NAV engage request
  blocks::ConfirmNode navSetRecent;       // NAV set condition true within the last 0.72 s

  // LOC armed
  blocks::EdgeDetector locArmLandEdge;    // LAND armed or engaged
  blocks::EdgeDetector locArmLocPush;     // LOC pushed
  blocks::ConfirmNode locArmPushRecent;   // LOC push armed LOC within the last 0.72 s
  blocks::EdgeDetector locArmLandDisarmed;  // falling edge of LAND armed
  blocks::EdgeDetector locArmNavArmed;    // NAV armed

  // LOC*
  blocks::ConfirmNode locCaptureSetRecent;  // LOC* set condition true within the last 0.72 s

  // LOC
  blocks::ConfirmNode locTrackDev10s;     // localizer deviation small for 10 s
  blocks::ConfirmNode locTrackSetRecent;  // LOC set condition true within the last 0.72 s

  // Roll FD bars flashing
  blocks::EdgeDetector rollBarsApFdEngage;  // first AP or FD engaged
  blocks::Monostable rollBarsFlash10s;      // 10 s of flashing

  // HDG/TRK preset (logic shared with the targets block, readable/targets.cpp)
  targets::HdgTrkPresetState preset;
};

/** Not-yet-rewritten FMGC signals this group needs (filled in src/external_lateral.h). */
struct LateralExternal {
  // Outer loops (FmgcOuterLoops) LOC capture roll command of the PREVIOUS step (Delay2_DSTATE.Phi_loc_c, deg): the
  // mode logic runs before this step's outer loops.
  double lastPhiLocC = 0;
};

}  // namespace fmgc::readable
