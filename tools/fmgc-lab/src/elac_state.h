// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Hidden memories of the generated ElacComputer paired with the readable ELAC's (and of their five laws), for the
// differential checker (src/check_elac.cpp): add(name, generated, readable) per memory, as doubles (exact for floats,
// bools, enums). Every DWork field, the block IO in_flight and the laws' DWork.

#pragma once

#include <string>

#include "ElacComputer.h"  // included with private -> public by the includer (efcs_drive.h)
#include "elac.h"
#include "lateral_law_state.h"
#include "pitch_law_state.h"

template <class Add>
void elacPairs(const ElacComputer& g, const elac::readable::ElacComputer& r, Add&& add) {
  namespace B = efcs::blocks;
  const auto& d = g.ElacComputer_DWork;
  auto name = [](const char* a, const char* b) { return std::string(a) + b; };
  auto confirm = [&](const char* n, const auto& gc, const B::ConfirmNode& rc) {
    add(name(n, ".time").c_str(), gc.timeSinceCondition, rc.timeSinceCondition);
    add(name(n, ".output").c_str(), gc.output, rc.output);
  };
  auto edge = [&](const char* n, const auto& ge, const B::EdgeDetector& re) {
    add(name(n, ".previous").c_str(), ge.previousInput, re.previousInput);
    add(name(n, ".primed").c_str(), ge.previousInput_not_empty, re.primed);
  };
  auto lag = [&](const char* n, const auto& gl, const B::LagFilter<double>& rl) {
    add(name(n, ".pY").c_str(), gl.pY, rl.pY);
    add(name(n, ".pU").c_str(), gl.pU, rl.pU);
    add(name(n, ".pY_not_empty").c_str(), gl.pY_not_empty, rl.primed);
    add(name(n, ".pU_not_empty").c_str(), gl.pU_not_empty, rl.primed);
  };
  auto rate = [&](const char* n, const auto& gr, const auto& rr) {
    add(name(n, ".pY").c_str(), gr.pY, rr.pY);
    add(name(n, ".primed").c_str(), gr.pY_not_empty, rr.primed);
  };

  add("running(Runtime_MODE)", d.Runtime_MODE, r.running());
  const auto& s = r.sensorState();
  lag("alphaFilter(LagFilter_a)", d.sf_LagFilter_a, s.alphaFilter);
  confirm("raDisagree(jz)", d.sf_MATLABFunction_jz, s.raDisagree);
  confirm("ra1Implausible(lf)", d.sf_MATLABFunction_lf, s.ra1Implausible);
  confirm("ra2Implausible(jl)", d.sf_MATLABFunction_jl, s.ra2Implausible);
  add("ra1Rejected(ra1CoherenceRejected)", d.ra1CoherenceRejected, s.ra1Rejected);
  add("ra2Rejected(ra2CoherenceRejected)", d.ra2CoherenceRejected, s.ra2Rejected);
  add("flapsFullTime(configFullEventTime)", d.configFullEventTime, s.flapsFullTime);
  add("flapsFullTimeSet(configFullEventTime_not_empty)", d.configFullEventTime_not_empty, s.flapsFullTimeSet);
  add("flightChartActive(is_active_c30)", d.is_active_c30_ElacComputer, s.flightChartActive);
  add("flightPhase(is_c30)", d.is_c30_ElacComputer, static_cast<double>(s.flightPhase));
  add("inFlight(B.in_flight)", g.ElacComputer_B.in_flight, s.inFlight);
  add("onGroundTime(on_ground_time)", d.on_ground_time, s.onGroundTime);
  add("yellowPressure(jg)", d.sf_MATLABFunction_jg.output, s.yellowPressure.output);
  add("bluePressure(mi)", d.sf_MATLABFunction_mi.output, s.bluePressure.output);
  add("greenPressure(br)", d.sf_MATLABFunction_br.output, s.greenPressure.output);
  confirm("yellowConfirm(cj)", d.sf_MATLABFunction_cj, s.yellowConfirm);
  confirm("blueConfirm(g2)", d.sf_MATLABFunction_g2, s.blueConfirm);
  confirm("greenConfirm(gfx)", d.sf_MATLABFunction_gfx, s.greenConfirm);

  const auto& e = r.engagementState();
  edge("captTakeover(g4)", d.sf_MATLABFunction_g4, e.captTakeover);
  edge("foTakeover(nu)", d.sf_MATLABFunction_nu, e.foTakeover);
  add("leftStickDisabled(pLeftStickDisabled)", d.pLeftStickDisabled, e.leftStickDisabled);
  add("rightStickDisabled(pRightStickDisabled)", d.pRightStickDisabled, e.rightStickDisabled);
  confirm("leftLockConfirm(j2)", d.sf_MATLABFunction_j2, e.leftLockConfirm);
  confirm("rightLockConfirm(g24)", d.sf_MATLABFunction_g24, e.rightLockConfirm);
  add("leftLockedPrev(Delay_DSTATE_cc)", d.Delay_DSTATE_cc, e.leftLockedPrev);
  add("rightLockedPrev(Delay1_DSTATE)", d.Delay1_DSTATE, e.rightLockedPrev);
  confirm("slewConfirm(nb)", d.sf_MATLABFunction_nb, e.slewConfirm);
  add("abnormalWasActive(abnormalConditionWasActive)", d.abnormalConditionWasActive, e.abnormalWasActive);
  edge("landing(l0)", d.sf_MATLABFunction_l0, e.landing);
  add("thsGroundLatch(Memory_PreviousInput)", d.Memory_PreviousInput, e.thsGroundLatch);

  const auto& p = r.protectionState();
  rate("alphaMaxRate(RateLimiter_n)", d.sf_RateLimiter_n, p.alphaMaxRate);
  rate("alphaProtRate(RateLimiter_m)", d.sf_RateLimiter_m, p.alphaProtRate);
  add("liftOffTime(eventTime_p)", d.eventTime_p, p.liftOffTime);
  add("liftOffTimeSet(eventTime_not_empty_i)", d.eventTime_not_empty_i, p.liftOffTimeSet);
  add("highSpeedProt(sProtActive)", d.sProtActive, p.highSpeedProt);
  add("stickNotPulledTime(resetEventTime)", d.resetEventTime, p.stickNotPulledTime);
  add("stickNotPulledTimeSet(resetEventTime_not_empty)", d.resetEventTime_not_empty, p.stickNotPulledTimeSet);
  add("alphaProt(sProtActive_f)", d.sProtActive_f, p.alphaProt);
  add("apChartActive(is_active_c28)", d.is_active_c28_ElacComputer, p.apChartActive);
  add("apPhase(is_c28)", d.is_c28_ElacComputer, static_cast<double>(p.apPhase));
  add("speedBelowLimitTime(eventTime)", d.eventTime, p.speedBelowLimitTime);
  add("speedBelowLimitTimeSet(eventTime_not_empty)", d.eventTime_not_empty, p.speedBelowLimitTimeSet);

  const auto& o = r.orderState();
  rate("droop(RateLimiter)", d.sf_RateLimiter, o.droop);
  rate("antiDroop(RateLimiter_b)", d.sf_RateLimiter_b, o.antiDroop);
  rate("leftAileron(RateLimiter_p)", d.sf_RateLimiter_p, o.leftAileron);
  rate("rightAileron(RateLimiter_a)", d.sf_RateLimiter_a, o.rightAileron);
  lag("pitchAcceleration(LagFilter)", d.sf_LagFilter, o.pitchAcceleration);
  add("previousThetaDot(Delay_DSTATE)", d.Delay_DSTATE, o.previousThetaDot);
  add("thsOrder(Delay_DSTATE_b)", d.Delay_DSTATE_b, o.thsOrder);
  add("thsIntegrator(Delay_DSTATE_c)", d.Delay_DSTATE_c, o.thsIntegrator);
  add("reloadThs(icLoad)", d.icLoad, o.reloadThs);
  confirm("battPower(fb)", d.sf_MATLABFunction_fb, o.battPower);

  auto prefixed = [&](const char* prefix) {
    return [&add, prefix](const char* n, double gv, double rv) { add((std::string(prefix) + n).c_str(), gv, rv); };
  };
  lateralNormalLawPairs(g.LawMDLOBJ2, r.lateralNormalLaw(), prefixed("lateralNormal."));
  lateralDirectLawPairs(g.LawMDLOBJ1, r.lateralDirectLaw(), prefixed("lateralDirect."));
  pitchNormalLawPairs(g.LawMDLOBJ5, r.pitchNormalLaw(), prefixed("pitchNormal."));
  pitchAlternateLawPairs(g.LawMDLOBJ3, r.pitchAlternateLaw(), prefixed("pitchAlternate."));
  pitchDirectLawPairs(g.LawMDLOBJ4, r.pitchDirectLaw(), prefixed("pitchDirect."));
}
