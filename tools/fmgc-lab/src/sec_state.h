// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Hidden memories of the generated SecComputer paired with the readable SEC's (and of its three laws), for the
// differential checker (src/check_sec.cpp): add(name, generated, readable) per memory.

#pragma once

#include <string>

#include "SecComputer.h"  // included with private -> public by the includer (efcs_drive.h)
#include "lateral_law_state.h"
#include "pitch_law_state.h"
#include "sec.h"

template <class Add>
void secPairs(const SecComputer& g, const sec::readable::SecComputer& r, Add&& add) {
  namespace B = efcs::blocks;
  const auto& d = g.SecComputer_DWork;
  const auto& s = r.state();
  auto name = [](const char* a, const char* b) { return std::string(a) + b; };
  auto confirm = [&](const char* n, const auto& gc, const B::ConfirmNode& rc) {
    add(name(n, ".time").c_str(), gc.timeSinceCondition, rc.timeSinceCondition);
    add(name(n, ".output").c_str(), gc.output, rc.output);
  };
  auto edge = [&](const char* n, const auto& ge, const B::EdgeDetector& re) {
    add(name(n, ".previous").c_str(), ge.previousInput, re.previousInput);
    add(name(n, ".primed").c_str(), ge.previousInput_not_empty, re.primed);
  };
  auto rate = [&](const char* n, const auto& gr, const auto& rr) {
    add(name(n, ".pY").c_str(), gr.pY, rr.pY);
    add(name(n, ".primed").c_str(), gr.pY_not_empty, rr.primed);
  };

  add("running(Runtime_MODE)", d.Runtime_MODE, r.running());
  confirm("lgciu2UplockConfirm(jk)", d.sf_MATLABFunction_jk, s.lgciu2UplockConfirm);
  confirm("lgciu1UplockConfirm(dw)", d.sf_MATLABFunction_dw, s.lgciu1UplockConfirm);
  add("groundChartActive(is_active_c8)", d.is_active_c8_SecComputer, s.groundChartActive);
  add("groundPhase(is_c8)", d.is_c8_SecComputer, static_cast<double>(s.groundPhase));
  add("flightChartActive(is_active_c30)", d.is_active_c30_SecComputer, s.flightChartActive);
  add("flightPhase(is_c30)", d.is_c30_SecComputer, static_cast<double>(s.flightPhase));
  add("inFlight(B.in_flight)", g.SecComputer_B.in_flight, s.inFlight);
  add("onGroundTime(on_ground_time)", d.on_ground_time, s.onGroundTime);
  confirm("yellowConfirm(ndv)", d.sf_MATLABFunction_ndv, s.yellowConfirm);
  confirm("blueConfirm(gf)", d.sf_MATLABFunction_gf, s.blueConfirm);
  confirm("greenConfirm(h)", d.sf_MATLABFunction_h, s.greenConfirm);
  edge("captTakeover(g4b)", d.sf_MATLABFunction_g4b, s.captTakeover);
  edge("foTakeover(nu)", d.sf_MATLABFunction_nu, s.foTakeover);
  add("leftStickDisabled(pLeftStickDisabled)", d.pLeftStickDisabled, s.leftStickDisabled);
  add("rightStickDisabled(pRightStickDisabled)", d.pRightStickDisabled, s.rightStickDisabled);
  confirm("leftLockConfirm(j2)", d.sf_MATLABFunction_j2, s.leftLockConfirm);
  confirm("rightLockConfirm(g24)", d.sf_MATLABFunction_g24, s.rightLockConfirm);
  add("leftLockedPrev(Delay_DSTATE_c)", d.Delay_DSTATE_c, s.leftLockedPrev);
  add("rightLockedPrev(Delay1_DSTATE)", d.Delay1_DSTATE, s.rightLockedPrev);
  confirm("slewConfirm(k4)", d.sf_MATLABFunction_k4, s.slewConfirm);
  add("abnormalWasActive(abnormalConditionWasActive)", d.abnormalConditionWasActive, s.abnormalWasActive);
  edge("landing(b4)", d.sf_MATLABFunction_b4, s.landing);
  add("thsGroundLatch(Memory_PreviousInput)", d.Memory_PreviousInput, s.thsGroundLatch);
  confirm("leverRetracted(fh)", d.sf_MATLABFunction_fh, s.leverRetracted);
  add("speedBrakeInhibitLatch(Memory_PreviousInput_f)", d.Memory_PreviousInput_f, s.speedBrakeInhibitLatch);
  edge("touchdown(nd)", d.sf_MATLABFunction_nd, s.touchdown);
  edge("liftOff(n)", d.sf_MATLABFunction_n, s.liftOff);
  add("wheelsSlowLatch(Memory_PreviousInput_n)", d.Memory_PreviousInput_n, s.wheelsSlowLatch);
  edge("bothMainGears(a)", d.sf_MATLABFunction_a, s.bothMainGears);
  add("groundSpoilersOut(Delay1_DSTATE_i)", d.Delay1_DSTATE_i, s.groundSpoilersOut);
  edge("oneMainGear(e3)", d.sf_MATLABFunction_e3, s.oneMainGear);
  add("partialLiftDumpingOneGear(Delay_DSTATE_n)", d.Delay_DSTATE_n, s.partialLiftDumpingOneGear);
  add("partialLiftDumpingArmed(Delay2_DSTATE)", d.Delay2_DSTATE, s.partialLiftDumpingArmed);
  rate("groundSpoilerRate(RateLimiter_c)", d.sf_RateLimiter_c, s.groundSpoilerRate);
  rate("speedBrakeRate(RateLimiter)", d.sf_RateLimiter, s.speedBrakeRate);
  rate("leftSpoiler1(RateLimiter_b)", d.sf_RateLimiter_b, s.leftSpoiler1);
  rate("rightSpoiler1(RateLimiter_a)", d.sf_RateLimiter_a, s.rightSpoiler1);
  rate("leftSpoiler2(RateLimiter_k)", d.sf_RateLimiter_k, s.leftSpoiler2);
  rate("rightSpoiler2(RateLimiter_b4)", d.sf_RateLimiter_b4, s.rightSpoiler2);
  add("thsOrder(Delay_DSTATE)", d.Delay_DSTATE, s.thsOrder);
  add("thsIntegrator(Delay_DSTATE_l)", d.Delay_DSTATE_l, s.thsIntegrator);
  add("reloadThs(icLoad)", d.icLoad, s.reloadThs);
  confirm("battPower(i)", d.sf_MATLABFunction_i, s.battPower);

  auto prefixed = [&](const char* prefix) {
    return [&add, prefix](const char* n, double gv, double rv) { add((std::string(prefix) + n).c_str(), gv, rv); };
  };
  lateralDirectLawPairs(g.LawMDLOBJ1, r.lateralDirectLaw(), prefixed("lateralDirect."));
  pitchAlternateLawPairs(g.LawMDLOBJ2, r.pitchAlternateLaw(), prefixed("pitchAlternate."));
  pitchDirectLawPairs(g.LawMDLOBJ3, r.pitchDirectLaw(), prefixed("pitchDirect."));
}
