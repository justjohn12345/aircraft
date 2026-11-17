// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// V/S, FPA, V/S reversion, TCAS, ALT soft mode (vertical_vs_tcas.cpp): block memories, and inputs from the parts of the
// FMGC not rewritten yet.

#pragma once

#include "blocks.h"
#include "targets.h"

namespace fmgc::readable {

struct VsTcasState {
  // V/S reversion
  blocks::EdgeDetector landLost;         // LAND armed or active went false
  blocks::EdgeDetector navLost;          // NAV went false
  blocks::EdgeDetector apprPushed;       // FCU APPR pushed
  double altCaptureTargetFt = 0;         // ALT* target on the frame ALT* engaged (sample and hold)
  double previousNextCstrFt = 0;         // fix: next_alt_cstr_ft of the previous frame

  // V/S-FPA
  blocks::EdgeDetector vsKnobPushedOrPulled;
  blocks::EdgeDetector apFdEngaged;      // first AP or FD engaged
  blocks::ConfirmNode airborne5s;        // in the air for 5 s
  blocks::ConfirmNode vsFpaCondRecent;   // V/S-FPA set condition true within the last 0.72 s

  // TCAS
  blocks::EdgeDetector raOnset;          // TCAS RA started
  blocks::ConfirmNode tcasCondRecent;    // TCAS set condition true within the last 0.72 s
  double tcasAltAcqTargetFt = 0;         // ALT* target on the frame the TCAS submode became ALT_ACQ (sample and hold)

  // ALT soft mode (logic shared with the targets block, readable/targets.cpp)
  targets::AltSoftState altSoft;
};

/** Not-yet-rewritten FMGC signals this group needs (filled in src/external_vs_tcas.h). */
struct VsTcasExternal {
  bool altCaptureCond = false;  // tcas_alt_acq_cond: ALT* geometric capture condition (ALT group, captureCond)
  bool altHoldCond = false;     // tcas_alt_hold_cond: |target − H| < 40 ft for 0.8 s (ALT group, withinAltHoldBand)
  double speedTargetKt = 0;     // spd_target_kts: the speed target (selected or managed, kt)
  bool machMode = false;        // mach_control_active: the speed target is a Mach number
};

}  // namespace fmgc::readable
