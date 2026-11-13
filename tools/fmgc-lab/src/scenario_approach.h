// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic approach episodes injected on top of a replay, for differential checks: the recordings contain no ILS
// approaches, so G/S, LAND and FLARE would otherwise never be exercised. Each 40–260 s episode overrides the ILS
// localizer/glideslope deviations (decaying, oscillating), radio height (descending), the approach type, APPR pushes
// (FCU word 1 bit 23, sometimes repeated), TOGA thrust lever positions, and sometimes no AP/FD engaged. Physically
// inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>

#include "FmgcComputer_types.h"

struct ApproachScenario {
  std::mt19937_64 rng;
  double t = 0, epLen = 0, te = 1e9;
  double L0 = 0, tauL = 1, G0 = 0, tauG = 1, w = 0, R0 = 0, rate = 0, pushAt = -1, push2At = -1, togaAt = -1;
  int approachType = 1;
  double pushLeft = 0, nextPushIn = 0;
  bool noApFd = false;
  explicit ApproachScenario(uint64_t seed) : rng(seed) {}
  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  void apply(fmgc_inputs& in, double dt) {
    if (te >= epLen) {
      te = 0; epLen = u(40, 260);
      L0 = u(-2.5, 2.5); tauL = u(3, 40); G0 = u(-1.5, 1.5); tauG = u(3, 60); w = u(0, 0.3);
      R0 = u(200, 3000); rate = u(2, 25);
      pushAt = u(0, 8); push2At = u(0, 1) < 0.2 ? u(0, epLen) : -1; togaAt = u(0, 1) < 0.35 ? u(5, epLen) : -1;
      approachType = u(0, 1) < 0.8 ? 1 : 2;
      noApFd = u(0, 1) < 0.5;
    }
    te += dt; t += dt;
    auto word = [](base_arinc_429& w, float v) { w.SSM = 3; w.Data = v; };
    float hdg = in.bus_inputs.ir_own_bus.heading_magnetic_deg.Data;
    float loc = static_cast<float>(L0 * std::exp(-te / tauL));
    float gs = static_cast<float>(G0 * std::exp(-te / tauG) * std::cos(w * te));
    for (auto* ils : {&in.bus_inputs.ils_own_bus, &in.bus_inputs.ils_opp_bus}) {
      word(ils->runway_heading_deg, hdg); word(ils->ils_frequency_mhz, 110.3F);
      word(ils->localizer_deviation_deg, loc); word(ils->glideslope_deviation_deg, gs);
    }
    float ra = static_cast<float>(std::fmax(0.0, R0 - rate * te));
    word(in.bus_inputs.ra_own_bus.radio_height_ft, ra); word(in.bus_inputs.ra_opp_bus.radio_height_ft, ra);
    in.fms_inputs.selected_approach_type = static_cast<fmgc_approach_type>(approachType);
    auto setBit = [](base_arinc_429& w, int b) { uint32_t v = static_cast<uint32_t>(w.Data); v |= 1u << (b - 1); w.Data = static_cast<float>(v); };
    if (pushLeft <= 0 && u(0, 1) < dt / 15.0) { pushLeft = u(0.1, 0.35); }
    if (pushLeft > 0) {
      pushLeft -= dt;
      if (pushLeft <= 0 && u(0, 1) < 0.4) { nextPushIn = u(0.2, 1.5); }
    }
    if (nextPushIn > 0) { nextPushIn -= dt; if (nextPushIn <= 0) pushLeft = u(0.1, 0.35); }
    if ((te >= pushAt && te < pushAt + 0.3) || (push2At >= 0 && te >= push2At && te < push2At + 0.3) || pushLeft > 0) {
      setBit(in.bus_inputs.fcu_bus.fcu_discrete_word_1, 23);
    }
    if (togaAt >= 0 && te >= togaAt && te < togaAt + 3) {
      word(in.bus_inputs.fadec_own_bus.selected_tla_deg, 45.0F); word(in.bus_inputs.fadec_opp_bus.selected_tla_deg, 45.0F);
    }
    if (noApFd && togaAt >= 0 && te >= togaAt - 3 && te < togaAt + 2) {
      setBit(in.bus_inputs.fcu_bus.fcu_discrete_word_2, 26); setBit(in.bus_inputs.fcu_bus.fcu_discrete_word_2, 27);
      in.discrete_inputs.fd_opp_engaged = false; in.discrete_inputs.ap_opp_engaged = false;
      in.discrete_inputs.ap_instinctive_disc = true;
    }
  }
};
