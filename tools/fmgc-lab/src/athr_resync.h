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

inline void resyncAthrN1(fmgc::readable::athr::N1State& s, const FmgcComputer::D_Work_FmgcComputer_T& pre) {
  auto leadLag = [](fmgc::blocks::LeadLagFilter& f, const auto& g) {
    f.pY = g.pY;
    f.pU = g.pU;
    f.primed = g.pY_not_empty && g.pU_not_empty;
  };
  s.groundSpeedRate.pY = pre.pY_k;
  s.groundSpeedRate.pU = pre.pU_l;
  s.groundSpeedRate.primed = pre.pY_not_empty_c && pre.pU_not_empty_e;
  leadLag(s.inertialAccelHighPass, pre.sf_LeadLagFilter);
  leadLag(s.airspeedWashout, pre.sf_LeadLagFilter_b);
  s.lastPredictedSpeedErrorKn = pre.Delay_DSTATE_i;
  s.errorRateLag.pY = pre.pY_e;
  s.errorRateLag.pU = pre.pU;
  s.errorRateLag.primed = pre.pY_not_empty_j && pre.pU_not_empty_n;
  s.normalAccelLag.pY = pre.sf_LagFilter_g.pY;
  s.normalAccelLag.pU = pre.sf_LagFilter_g.pU;
  s.normalAccelLag.primed = pre.sf_LagFilter_g.pY_not_empty && pre.sf_LagFilter_g.pU_not_empty;
  s.loadFactorWashout.pY = pre.pY_h;
  s.loadFactorWashout.pU = pre.pU_p;
  s.loadFactorWashout.primed = pre.pY_not_empty && pre.pU_not_empty;
  s.altSoftBlend.pY = pre.pY;
  s.altSoftBlend.primed = pre.pY_not_empty_e;
  // Delay_DSTATE_f (integrator state) equals Delay_DSTATE_l (limited output) at the start of every step after the first.
  s.n1CommandPercent = pre.Delay_DSTATE_l;
  s.loadIntegrator = pre.icLoad;
}
