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

namespace athr_resync_detail {
inline void confirm(fmgc::blocks::ConfirmNode& c, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_k_T& g) {
  c.output = g.output;
  c.timeSinceCondition = g.timeSinceCondition;
}
inline void edge(fmgc::blocks::EdgeDetector& e, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_c_T& g) {
  e.previousInput = g.previousInput;
  e.primed = g.previousInput_not_empty;
}
}  // namespace athr_resync_detail

// Every mode-stage memory has a single generated counterpart.
inline void resyncAthrModes(fmgc::readable::athr::ModesState& s, const FmgcComputer::D_Work_FmgcComputer_T& pre) {
  using namespace athr_resync_detail;
  confirm(s.tcasActiveFor5s, pre.sf_MATLABFunction_a);
  edge(s.tcasEngagePulse, pre.sf_MATLABFunction_g);
  s.tcasLatch.q = pre.Memory_PreviousInput_jb;
  edge(s.alphaFloorRequestPulse, pre.sf_MATLABFunction_k);
  s.alphaFloor.q = pre.Memory_PreviousInput_hu;
  edge(s.thrustLawPulse, pre.sf_MATLABFunction_j);
  confirm(s.thrustSetRecent, pre.sf_MATLABFunction_ay);
  s.thrustMode.q = pre.Memory_PreviousInput_bh;
  edge(s.speedLawPulse, pre.sf_MATLABFunction_ge);
  confirm(s.speedSetRecent, pre.sf_MATLABFunction_ig);
  s.speedMode.q = pre.Memory_PreviousInput_cm;
  confirm(s.retardSetRecent, pre.sf_MATLABFunction_kh);
  s.retardMode.q = pre.Memory_PreviousInput_ol;
  s.lvrClbArmed.q = pre.Memory_PreviousInput_kr;
  s.lvrMctArmed.q = pre.Memory_PreviousInput_km;
}

inline void resyncAthrN1(fmgc::readable::athr::N1State& /*s*/, const FmgcComputer::D_Work_FmgcComputer_T& /*pre*/) {}
