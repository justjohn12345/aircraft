// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Resync of the readable engagement state (readable/engagement.h State) from the generated model's state at the start
// of a step (check_engagement resync mode): each readable memory takes the value its generated counterpart had before
// the step, so a mismatch points at the step where the logic differs. Every memory has a single generated counterpart.

#pragma once

#include "FmgcComputer.h"
#include "engagement.h"

namespace engagement_resync_detail {
inline void confirm(fmgc::blocks::ConfirmNode& c, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_k_T& g) {
  c.output = g.output;
  c.timeSinceCondition = g.timeSinceCondition;
}
inline void edge(fmgc::blocks::EdgeDetector& e, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_c_T& g) {
  e.previousInput = g.previousInput;
  e.primed = g.previousInput_not_empty;
}
inline void mono(fmgc::blocks::Monostable& m, const FmgcComputer::rtDW_MATLABFunction_FmgcComputer_f_T& g) {
  m.previousInput = g.previousInput;
  m.remainingTriggerTime = g.remainingTriggerTime;
}
}  // namespace engagement_resync_detail

inline void resyncEngagement(fmgc::readable::engagement::State& s, const FmgcComputer::D_Work_FmgcComputer_T& pre) {
  using namespace engagement_resync_detail;
  confirm(s.apFdAthrCommon, pre.sf_MATLABFunction_kv);
  mono(s.rolloutHeadingDev, pre.sf_MATLABFunction_m3);
  confirm(s.apFdCommon, pre.sf_MATLABFunction_bv);
  edge(s.apPbPush, pre.sf_MATLABFunction_il);
  edge(s.apEngineStart, pre.sf_MATLABFunction_jp);
  edge(s.apOppEngage, pre.sf_MATLABFunction_cb);
  edge(s.apLandGaLost, pre.sf_MATLABFunction_hp);
  s.apEngageRequest.q = pre.Memory_PreviousInput;
  confirm(s.apRequestHeld, pre.sf_MATLABFunction_it);
  // The AP engaged latch's Q is Memory_PreviousInput_g (a copy of Delay_DSTATE_p made at the end of the step); the
  // engage request reads Delay_DSTATE_p itself. Both hold the previous frame's value.
  s.apEngaged.q = pre.Memory_PreviousInput_g;
  confirm(s.athrInstinctive15s, pre.sf_MATLABFunction_kz);
  s.athrInhibited.q = pre.Memory_PreviousInput_g1;
  edge(s.athrPbPush, pre.sf_MATLABFunction_o1);
  confirm(s.athrSrs09s, pre.sf_MATLABFunction_mn);
  edge(s.athrSrsEdge, pre.sf_MATLABFunction_jt);
  edge(s.athrLeversIdle, pre.sf_MATLABFunction_d4);
  edge(s.athrAlphaFloor, pre.sf_MATLABFunction_ga);
  edge(s.athrOppSyncLost, pre.sf_MATLABFunction_e5);
  edge(s.athrTcas, pre.sf_MATLABFunction_ag);
  confirm(s.athrEcuNotEngaged, pre.sf_MATLABFunction_dx);
  s.athrEngaged.q = pre.Memory_PreviousInput_p;  // copy of Delay_DSTATE_k, as for the AP
  edge(s.resetEngineStart, pre.sf_MATLABFunction_p2o);
  mono(s.modeReversion1s, pre.sf_MATLABFunction_hdx);
  edge(s.pitchBarsApFdEngage, pre.sf_MATLABFunction_h0);
  mono(s.pitchBarsFlash10s, pre.sf_MATLABFunction_ppo);
}
