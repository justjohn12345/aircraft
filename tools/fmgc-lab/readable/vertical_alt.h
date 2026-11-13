// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ALT*, ALT, ALT CRZ (vertical_alt.cpp): block memories, and inputs from the parts of the FMGC not rewritten yet.

#pragma once

#include "blocks.h"

namespace fmgc::readable {

struct AltState {
  blocks::Monostable fcuAltChangedRecently;  // 3 s after an FCU altitude change: no capture
  blocks::ConfirmNode captureCondRecent;     // ALT* set condition true within the last 0.72 s
  blocks::ConfirmNode withinAltHoldBand;     // |target − H| < 40 ft for 0.8 s
  blocks::ConfirmNode altHoldCondRecent;     // ALT set condition true within the last 0.72 s
};

}  // namespace fmgc::readable
