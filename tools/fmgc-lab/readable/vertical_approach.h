// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SRS, LAND, FLARE, G/S, FINAL (vertical_approach.cpp): block memories, and inputs from the parts of the FMGC not
// rewritten yet.

#pragma once

#include "blocks.h"

namespace fmgc::readable {

struct ApproachState {
  // SRS takeoff
  blocks::ConfirmNode toOnGround30s;          // on ground for 30 s: a thrust lever edge means takeoff, not go-around
  blocks::EdgeDetector toTakeoffThrustEdge;   // a thrust lever reaches FLX (with a FLEX temperature) or TOGA
  blocks::ConfirmNode toSetRecent;            // SRS T/O set condition true within the last 0.72 s

  // LAND armed
  blocks::EdgeDetector landArmApprPush;       // APPR pushed
  blocks::ConfirmNode landArmedFor045s;       // LAND armed for 0.45 s (a second APPR push disarms instead)
  blocks::EdgeDetector landArmSelectedVertical;  // V/S, FPA, OP CLB/DES or EXPED engaged
  blocks::EdgeDetector landArmSelectedLateral;   // HDG, TRK or NAV engaged
  blocks::EdgeDetector landArmLocPush;        // LOC pushed

  // LAND
  blocks::ConfirmNode landNotActive072s;      // LAND not active for 0.72 s
  blocks::ConfirmNode landBelow400ft;         // RA < 400 ft for 1.2 s
  blocks::ConfirmNode landSetRecent;          // LAND set condition true within the last 0.72 s
  blocks::EdgeDetector landApprPush;          // APPR pushed
  blocks::ConfirmNode landOnGround10s;        // on ground for 10 s

  // FLARE
  blocks::ConfirmNode flareCondition036s;     // flare condition true for 0.36 s

  // FINAL armed
  blocks::ConfirmNode finalArmedRecent;       // FINAL armed within the last 0.72 s
  blocks::EdgeDetector finalArmApprPush;      // APPR pushed
  blocks::EdgeDetector finalArmNavLost;       // falling edge of NAV armed or active
  blocks::EdgeDetector finalArmLocPush;       // LOC pushed

  // FINAL
  blocks::ConfirmNode finalSetRecent;         // FINAL set condition true within the last 0.72 s

  // SRS go-around
  blocks::ConfirmNode gaOnGround30s;          // on ground for 30 s (then a TOGA edge is a takeoff, not a go-around)
  blocks::EdgeDetector gaTogaEdge;            // a thrust lever reaches TOGA
  blocks::ConfirmNode gaSetRecent;            // SRS G/A set condition true within the last 0.72 s

  // G/S armed
  blocks::EdgeDetector glideArmLandArmedOrActive;  // rising edge of LAND armed or active
  blocks::EdgeDetector glideArmLandDisarmed;       // falling edge of LAND armed

  // G/S*
  blocks::LagFilter<float> gsDeviationLag;    // G/S deviation, 1 s lag
  float gsDeviationLagLast = 0;               // its value of the previous frame (DetectDecrease)
  blocks::ConfirmNode gsCaptureSetRecent;     // G/S* set condition true within the last 0.72 s

  // G/S
  blocks::ConfirmNode gsCaptureFor15s;        // G/S* active for 15 s
  blocks::ConfirmNode gsTrackSetRecent;       // G/S set condition true within the last 0.72 s
};

/** Not-yet-rewritten FMGC signals this group needs (filled in src/external_approach.h). */
struct ApproachExternal {
  // Outer loops (FmgcOuterLoops) flare trigger of the PREVIOUS step: the mode logic runs before this step's outer
  // loops and reads their delayed output (Delay2_DSTATE.flare_law.condition_Flare).
  bool lastFlareCondition = false;
};

}  // namespace fmgc::readable
