// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Generated <-> readable hidden state of the pitch laws (PitchNormalLaw, PitchAlternateLaw, PitchDirectLaw), for the
// differential checkers (src/check_pitch_laws.cpp, and the ELAC / SEC checkers). Every D_Work member (and the normal
// law's block outputs held in PitchNormalLaw_B) is paired with the readable value it must equal:
//
//   pitchNormalLawPairs(g, r, add), pitchAlternateLawPairs(g, r, add), pitchDirectLawPairs(g, r, add)
//
// call add("readableName(GeneratedName)", double generated, double readable) once per memory. The names are string
// literals (no allocation), so the pairs can be walked every step. Booleans and chart states are compared as numbers
// (chart states in the generated numbering; "active" flags as 0/1). Where the generated model keeps several identical
// copies of a block (the C* law's shared filters, the two V_ias rate filters), each copy is paired with the one
// readable block.
//
// Include this header before any other include of the generated pitch law headers (it opens their private members).

#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

#define private public
#include "PitchAlternateLaw.h"
#include "PitchDirectLaw.h"
#include "PitchNormalLaw.h"
#undef private

#include "pitch_alternate_law.h"
#include "pitch_direct_law.h"
#include "pitch_normal_law.h"

// G: generated D_Work, R: readable state
#define PLS_PAIR(name, gv, rv) add(name, static_cast<double>(gv), static_cast<double>(rv))
#define PLS_LAG(rn, gn)                                                                     \
  PLS_PAIR(#rn ".pY(" #gn ".pY)", G.gn.pY, R.rn.pY);                                        \
  PLS_PAIR(#rn ".pU(" #gn ".pU)", G.gn.pU, R.rn.pU);                                        \
  PLS_PAIR(#rn ".primed(" #gn ".pY_not_empty)", G.gn.pY_not_empty, R.rn.primed);           \
  PLS_PAIR(#rn ".primed(" #gn ".pU_not_empty)", G.gn.pU_not_empty, R.rn.primed)
#define PLS_RATE(rn, gn)                                \
  PLS_PAIR(#rn ".pY(" #gn ".pY)", G.gn.pY, R.rn.pY); \
  PLS_PAIR(#rn ".primed(" #gn ".pY_not_empty)", G.gn.pY_not_empty, R.rn.primed)
#define PLS_DELAY(rn, gn) PLS_PAIR(#rn "(" #gn ")", G.gn, R.rn)
#define PLS_CHART(rn, gn, none)                                                                          \
  PLS_PAIR(#rn " active(is_active_" #gn ")", G.is_active_##gn, R.rn != none ? 1 : 0);                    \
  PLS_PAIR(#rn "(is_" #gn ")", G.is_##gn, static_cast<int>(R.rn))

template <class Add>
void pitchNormalLawPairs(const ::PitchNormalLaw& g, const efcs::readable::PitchNormalLaw& r, Add&& add) {
  using S = efcs::readable::PitchNormalLawState;
  const auto& G = g.PitchNormalLaw_DWork;
  const S& R = r.state();

  // flare mode
  PLS_LAG(thetaFilter, sf_LagFilter);
  PLS_CHART(flareMode, c2_PitchNormalLaw, S::FlareMode::None);
  PLS_PAIR("flareThetaCmdDeg(PitchNormalLaw_B.flare_Theta_c_deg)", g.PitchNormalLaw_B.flare_Theta_c_deg,
           R.flareThetaCmdDeg);
  PLS_PAIR("flareThetaCmdRateDegS(PitchNormalLaw_B.flare_Theta_c_rate_deg_s)",
           g.PitchNormalLaw_B.flare_Theta_c_rate_deg_s, R.flareThetaCmdRateDegS);
  PLS_DELAY(flareThetaCmd.y, Delay_DSTATE_e);

  // auto trim, flight phase, blends
  PLS_CHART(trimFreeze.state, c9_PitchNormalLaw, efcs::readable::pitch_blocks::TrimFreezeChart::State::None);
  PLS_CHART(autoTrimMode.mode, c8_PitchNormalLaw, efcs::readable::pitch_blocks::AutoTrimModeChart::Mode::None);
  PLS_CHART(flightPhase.phase, c7_PitchNormalLaw, efcs::readable::pitch_blocks::FlightPhaseChart::Phase::None);
  PLS_RATE(inFlightBlend, sf_RateLimiter);
  PLS_CHART(rotation, c6_PitchNormalLaw, S::Rotation::None);
  PLS_RATE(rotationBlend, sf_RateLimiter_p);
  PLS_RATE(nzMax, sf_RateLimiter_c);
  PLS_RATE(nzMin, sf_RateLimiter_n);
  PLS_PAIR("trimLimitLo.value(sf_eta_trim_limit_lofreeze.frozen_eta_trim)",
           G.sf_eta_trim_limit_lofreeze.frozen_eta_trim, R.trimLimitLo.value);
  PLS_PAIR("trimLimitLo.valid(sf_eta_trim_limit_lofreeze.frozen_eta_trim_not_empty)",
           G.sf_eta_trim_limit_lofreeze.frozen_eta_trim_not_empty, R.trimLimitLo.valid);
  PLS_PAIR("trimLimitUp.value(sf_eta_trim_limit_upfreeze.frozen_eta_trim)",
           G.sf_eta_trim_limit_upfreeze.frozen_eta_trim, R.trimLimitUp.value);
  PLS_PAIR("trimLimitUp.valid(sf_eta_trim_limit_upfreeze.frozen_eta_trim_not_empty)",
           G.sf_eta_trim_limit_upfreeze.frozen_eta_trim_not_empty, R.trimLimitUp.valid);

  // C*: the shared filters (7 copies) and each channel's D term
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_l);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_b);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_bg);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_kd);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_e5);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_h);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_c);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_d);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_i);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_lf);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_dy);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_lf1);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_jt);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_k);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_g3);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_m);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_gr);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_i);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_g);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_n);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_k);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_c);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_k4);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_ca);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_l);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_d);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter);
  PLS_DELAY(thetaMaxChannel.dTerm.pU, Delay_DSTATE_n);
  PLS_DELAY(nzMaxChannel.dTerm.pU, Delay_DSTATE_k);
  PLS_DELAY(autopilotChannel.dTerm.pU, Delay_DSTATE_en);
  PLS_DELAY(flareChannel.dTerm.pU, Delay_DSTATE_jv);
  PLS_DELAY(normalChannel.dTerm.pU, Delay_DSTATE_jh);
  PLS_DELAY(nzMinChannel.dTerm.pU, Delay_DSTATE_gz);
  PLS_DELAY(thetaMinChannel.dTerm.pU, Delay_DSTATE_ds);

  // pitch attitude protection, autopilot
  PLS_RATE(thetaMax, sf_RateLimiter_o);
  PLS_RATE(apThetaCmd, sf_RateLimiter_i);
  PLS_DELAY(apBlend.y, Delay_DSTATE_g);

  // high angle of attack protection
  PLS_RATE(aoaStick, sf_RateLimiter_nx);
  PLS_LAG(alphaFilter, sf_LagFilter_mf);
  PLS_LAG(attitudeWashout, sf_WashoutFilter_h);
  PLS_DELAY(alphaErrorRate.derivative.pU, Delay_DSTATE_f);
  PLS_DELAY(alphaErrorRate.lag.pU, Delay_DSTATE_g5);
  PLS_DELAY(alphaErrorRate.lag.y, Delay1_DSTATE);
  PLS_DELAY(iasRate.derivative.pU, Delay_DSTATE_j);
  PLS_DELAY(iasRate.lag.pU, Delay_DSTATE_ca);
  PLS_DELAY(iasRate.lag.y, Delay1_DSTATE_i);
  PLS_DELAY(iasRate.derivative.pU, Delay_DSTATE_m);
  PLS_DELAY(iasRate.lag.pU, Delay_DSTATE_k2);
  PLS_DELAY(iasRate.lag.y, Delay1_DSTATE_ns);
  PLS_DELAY(aoaBlend.y, Delay_DSTATE_e1);

  // flare law
  PLS_RATE(flareThetaLimit, sf_RateLimiter_ck);
  PLS_DELAY(flareThetaLag.pU, Delay_DSTATE_o);
  PLS_DELAY(flareThetaLag.y, Delay1_DSTATE_n);
  PLS_RATE(flareThetaRate, sf_RateLimiter_e);
  PLS_DELAY(flareBlend.y, Delay_DSTATE_dv);

  // normal law, high speed protection
  PLS_RATE(normalStick, sf_RateLimiter_c2);
  PLS_DELAY(hspStick.y, Delay_DSTATE_b5);
  PLS_DELAY(hspTargetRate.derivative.pU, Delay_DSTATE_ku);
  PLS_DELAY(hspTargetRate.lag.pU, Delay_DSTATE_gl);
  PLS_DELAY(hspTargetRate.lag.y, Delay1_DSTATE_l);
  PLS_DELAY(hspBlend.y, Delay_DSTATE_mz);

  // integrators, ground law, output
  PLS_DELAY(flightIntegrator.x, Delay_DSTATE_o3);
  PLS_DELAY(flightIntegrator.icLoad, icLoad);
  PLS_RATE(rotationStick, sf_RateLimiter_ct);
  PLS_DELAY(qDemandRate.pU, Delay_DSTATE_ej);
  PLS_DELAY(qErrorRate.pU, Delay_DSTATE_e4);
  PLS_LAG(qPredicted, sf_LagFilter_f);
  PLS_DELAY(rotationIntegrator.x, Delay_DSTATE_cl);
  PLS_DELAY(rotationIntegrator.icLoad, icLoad_p);
  PLS_RATE(eta, sf_RateLimiter_b);
}

template <class Add>
void pitchAlternateLawPairs(const ::PitchAlternateLaw& g, const efcs::readable::PitchAlternateLaw& r, Add&& add) {
  namespace pb = efcs::readable::pitch_blocks;
  const auto& G = g.PitchAlternateLaw_DWork;
  const efcs::readable::PitchAlternateLawState& R = r.state();

  PLS_CHART(trimFreeze.state, c9_PitchAlternateLaw, pb::TrimFreezeChart::State::None);
  PLS_CHART(flightPhase.phase, c7_PitchAlternateLaw, pb::FlightPhaseChart::Phase::None);
  PLS_CHART(autoTrimMode.mode, c8_PitchAlternateLaw, pb::AutoTrimModeChart::Mode::None);
  PLS_RATE(nzMax, sf_RateLimiter);
  PLS_RATE(nzMin, sf_RateLimiter_n);
  PLS_PAIR("stick.pY(pY)", G.pY, R.stick.pY);
  PLS_PAIR("stick.primed(pY_not_empty)", G.pY_not_empty, R.stick.primed);

  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_kd);
  PLS_DELAY(cstar.qkRate.pU, Delay_DSTATE_e);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_d);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_dy);
  PLS_DELAY(cstar.vTasRate.pU, Delay_DSTATE_l);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_g3);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter);
  PLS_LAG(cstar.vTasRateLag, sf_LagFilter_g);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_c);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter);
  PLS_LAG(cstar.spoilerWashout, sf_WashoutFilter_d);
  PLS_DELAY(nzMaxChannel.dTerm.pU, Delay_DSTATE_k);
  PLS_DELAY(stickChannel.dTerm.pU, Delay_DSTATE_j);
  PLS_DELAY(nzMinChannel.dTerm.pU, Delay_DSTATE_g);

  PLS_DELAY(integrator.x, Delay_DSTATE_o);
  PLS_DELAY(integrator.icLoad, icLoad);
  PLS_RATE(eta, sf_RateLimiter_b);
}

template <class Add>
void pitchDirectLawPairs(const ::PitchDirectLaw& g, const efcs::readable::PitchDirectLaw& r, Add&& add) {
  PLS_PAIR("eta.pY(pY)", g.PitchDirectLaw_DWork.pY, r.eta().pY);
  PLS_PAIR("eta.primed(pY_not_empty)", g.PitchDirectLaw_DWork.pY_not_empty, r.eta().primed);
}

#undef PLS_PAIR
#undef PLS_LAG
#undef PLS_RATE
#undef PLS_DELAY
#undef PLS_CHART
