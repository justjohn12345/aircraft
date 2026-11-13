// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// CLB, DES, OP CLB, OP DES, EXP CLB, EXP DES and their armed modes (vertical_climb_descent.cpp): block memories, and
// inputs from the parts of the FMGC not rewritten yet.

#pragma once

#include "blocks.h"

namespace fmgc::readable {

struct ClimbDescentState {
  // CLB armed
  blocks::EdgeDetector clbArmAltPushed;  // ALT knob pushed (feeds a redundant term only, but updated every frame)

  // CLB
  blocks::EdgeDetector clbAccAltCrossedUp;   // indicated altitude rises above ACC ALT
  blocks::EdgeDetector clbAltPushed;         // ALT knob pushed
  blocks::ConfirmNode clbSetRecently;        // CLB set condition true within the last 0.72 s
  double clbPreviousNextCstrFt = 0;          // next_alt_cstr_ft of the previous frame (DetectChange)

  // DES armed
  blocks::EdgeDetector desArmAltPushed;  // ALT knob pushed (redundant term, see CLB armed)

  // DES
  blocks::EdgeDetector desAltPushed;
  blocks::ConfirmNode desSetRecently;
  double desPreviousNextCstrFt = 0;  // second DetectChange copy, same value as clbPreviousNextCstrFt

  // OP CLB
  blocks::EdgeDetector opClbAltPulled;          // ALT knob pulled
  blocks::ConfirmNode manualSpdInExpClbOrSrs;   // selected speed in EXP CLB, SRS or SRS GA for 0.72 s
  blocks::EdgeDetector opClbAccAltCrossedUp;    // indicated altitude rises above ACC ALT (own copy)
  blocks::ConfirmNode opClbSetRecently;
  bool opClbReversion = false;                  // this frame's OP CLB reversion condition, for ReversionOpClb

  // OP DES
  blocks::EdgeDetector opDesAltPulled;
  blocks::ConfirmNode manualSpdInExpDes;  // selected speed in EXP DES for 0.72 s
  blocks::ConfirmNode opDesSetRecently;

  // EXP CLB
  blocks::EdgeDetector expClbExpedPushed;
  blocks::ConfirmNode expClbSetRecently;

  // EXP DES
  blocks::EdgeDetector expDesExpedPushed;
  blocks::ConfirmNode expDesSetRecently;
};

/** Not-yet-rewritten FMGC signals this group needs (filled in src/external_climb_descent.h). */
struct ClimbDescentExternal {
  // Input conditioning: "not on ground" confirmed for 5 s (ConfirmNode on !logic.on_ground, FmgcComputer.cpp 1276)
  bool inFlightFor5s = false;
};

}  // namespace fmgc::readable
