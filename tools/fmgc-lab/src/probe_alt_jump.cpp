// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Experiment: a sudden change of indicated altitude during an ALT* capture (e.g. an MSFS live weather update moving
// the pressure field: the aircraft does not move, the pressure altitude does; the IR inertial V/S is unaffected).
//
// For each ALT* capture in a recording, replays it through the generated model with a step of dH ft added to every
// ADR altitude word (standard and both baro-corrected, own/opp/3) from `delay` s after ALT* engaged, and compares the
// AP and FD pitch commands with the unperturbed replay over the next 20 s. Open loop: the recorded aircraft does not
// respond, so this shows the FMGC's immediate command, not the resulting flight path.
// Usage: fmgc-probe-alt-jump <file.fdr> [delay s, default 3] [max captures, default 4]

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include "FmgcComputer.h"
#include "replay_core.h"

namespace {

struct Sample {
  double t;
  bool altAcq, altHold, vs, apEngaged;
  double apTheta, fdTheta, theta, vsFpm, hInd, hTarget;
};

// Replays the file; from time tJump on adds dH to the ADR altitudes. Stops at tStop (or the end when tStop < 0).
std::vector<Sample> run(const char* path, double tJump, double dH, double tStop) {
  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    std::exit(2);
  }
  double tNow = 0;
  replayer.beforeStep = [&]() {
    if (dH == 0 || tNow < tJump) {
      return;
    }
    auto& b = replayer.mutableInputs().bus_inputs;
    for (base_adr_bus* adr : {&b.adr_own_bus, &b.adr_opp_bus, &b.adr_3_bus}) {
      adr->altitude_standard_ft.Data += static_cast<float>(dH);
      adr->altitude_corrected_1_ft.Data += static_cast<float>(dH);
      adr->altitude_corrected_2_ft.Data += static_cast<float>(dH);
    }
  };
  std::vector<Sample> out;
  Replayer::Frame f;
  while (replayer.next(f)) {
    tNow = f.fdr->base.simulation_time_s;
    const fmgc_outputs& o = computer->getExternalOutputs().out;
    const auto& m = o.ap_fd_logic.longitudinal_modes;
    out.push_back({tNow, m.alt_acq_active, m.alt_hold_active, m.vs_active, o.logic.ap_own_engaged,
                   o.ap_fd_outer_loops.autopilot.Theta_c_deg, o.ap_fd_outer_loops.flight_director.Theta_c_deg,
                   o.data.bus_inputs.ir_own_bus.pitch_angle_deg.Data,
                   o.data.bus_inputs.ir_own_bus.inertial_vertical_speed_ft_s.Data, o.logic.altitude_indicated_ft.Data,
                   o.ap_fd_logic.alt_sel_or_cstr});
    if (tStop >= 0 && tNow > tStop) {
      break;
    }
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: fmgc-probe-alt-jump <file.fdr> [delay s] [max captures]\n");
    return 2;
  }
  const double delay = argc > 2 ? std::atof(argv[2]) : 3.0;
  const int maxCaptures = argc > 3 ? std::atoi(argv[3]) : 4;
  auto base = run(argv[1], 1e30, 0, -1);

  int captures = 0;
  for (size_t i = 1; i < base.size() && captures < maxCaptures; i++) {
    if (!(base[i].altAcq && !base[i - 1].altAcq)) {
      continue;
    }
    const double tEntry = base[i].t;
    const double tJump = tEntry + delay;
    // only captures still in ALT* at the jump, with the AP engaged
    size_t j = i;
    while (j < base.size() && base[j].t < tJump) {
      j++;
    }
    if (j >= base.size() || !base[j].altAcq || !base[j].apEngaged) {
      continue;
    }
    captures++;
    std::printf("capture at t %.1f: target %.0f ft, H %.0f ft, V/S at entry %.0f fpm; at jump (t %.1f): H %.0f, V/S %.0f, "
                "theta %.1f, AP theta_c %.2f\n",
                tEntry, base[i].hTarget, base[i].hInd, base[i].vsFpm, tJump, base[j].hInd, base[j].vsFpm, base[j].theta,
                base[j].apTheta);
    for (double dH : {-1000.0, -500.0, -200.0, 200.0, 500.0}) {
      auto pert = run(argv[1], tJump, dH, tJump + 20);
      double maxUp = 0, maxDown = 0, maxUpT = 0, maxFdUp = 0;
      bool reverted = false, altHold = false;
      for (size_t k = j; k < pert.size() && k < base.size() && pert[k].t <= tJump + 20; k++) {
        double d = pert[k].apTheta - base[k].apTheta;
        if (d > maxUp) {
          maxUp = d;
          maxUpT = pert[k].t - tJump;
        }
        maxDown = std::min(maxDown, d);
        maxFdUp = std::max(maxFdUp, pert[k].fdTheta - base[k].fdTheta);
        reverted |= pert[k].vs;
        altHold |= pert[k].altHold;
      }
      std::printf("  dH %+6.0f ft: AP theta_c vs unperturbed: up to %+5.2f deg (at +%.1f s), down to %+5.2f deg; FD up to "
                  "%+5.2f deg; %s%s\n",
                  dH, maxUp, maxUpT, maxDown, maxFdUp, reverted ? "V/S engaged " : "", altHold ? "ALT engaged" : "");
    }
  }
  if (captures == 0) {
    std::printf("no ALT* capture with the AP engaged\n");
  }
  return 0;
}
