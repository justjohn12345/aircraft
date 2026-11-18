// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Hidden memories of the generated lateral laws paired with the readable ones', for the differential checkers
// (src/check_lateral_laws.cpp, src/check_elac.cpp, src/check_sec.cpp): add(name, generated, readable) per memory, as
// doubles (exact for floats, bools, enums).

#pragma once

#define private public
#include "LateralDirectLaw.h"
#include "LateralNormalLaw.h"
#undef private

#include "lateral_direct_law.h"
#include "lateral_normal_law.h"

template <class Add>
void lateralNormalLawPairs(const LateralNormalLaw& g, const efcs::readable::LateralNormalLaw& r, Add&& add) {
  const auto& d = g.LateralNormalLaw_DWork;
  auto rate = [&](const char* name, double gY, bool gPrimed, const efcs::blocks::RateLimiterInit& rr) {
    add((std::string(name) + ".pY").c_str(), gY, rr.pY);
    add((std::string(name) + ".primed").c_str(), gPrimed, rr.primed);
  };
  add("modeChartActive(is_active_c5)", d.is_active_c5_LateralNormalLaw, r.modeChartActive());
  add("mode(is_c5)", d.is_c5_LateralNormalLaw, static_cast<double>(r.mode()));
  rate("inFlightRate(sf_RateLimiter)", d.sf_RateLimiter.pY, d.sf_RateLimiter.pY_not_empty, r.inFlightRate());
  rate("pedalFeedForwardRate(sf_RateLimiter_d)", d.sf_RateLimiter_d.pY, d.sf_RateLimiter_d.pY_not_empty,
       r.pedalFeedForwardRate());
  rate("apEngagedRate(pY)", d.pY, d.pY_not_empty, r.apEngagedRate());
  rate("xiOrderRate(sf_RateLimiter_j)", d.sf_RateLimiter_j.pY, d.sf_RateLimiter_j.pY_not_empty, r.xiOrderRate());
  rate("airborneRate(pY_b)", d.pY_b, d.pY_not_empty_j, r.airborneRate());
  add("previousRollRateDemand(Delay_DSTATE_o)", d.Delay_DSTATE_o, r.previousRollRateDemand());
  add("previousXiCommand(Delay_DSTATE)", d.Delay_DSTATE, r.previousXiCommand());
  add("bankCommand(Delay_DSTATE_d)", d.Delay_DSTATE_d, r.bankCommand());
  add("reloadBankCommand(icLoad)", d.icLoad, r.reloadBankCommand());
}

template <class Add>
void lateralDirectLawPairs(const LateralDirectLaw& g, const efcs::readable::LateralDirectLaw& r, Add&& add) {
  add("xiRate.pY", g.LateralDirectLaw_DWork.pY, r.xiRate().pY);
  add("xiRate.primed", g.LateralDirectLaw_DWork.pY_not_empty, r.xiRate().primed);
}
