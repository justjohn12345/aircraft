// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Probe: replays a recording and reports every ALT* capture law initialisation (FmgcOuterLoops, ALT_ACQ law entry),
// reading the law's persistent state (k, dH_offset, maxH_dot) from the model.
//
// The law (an embedded MATLAB function in FmgcOuterLoops::step) is, every frame:
//   r = dH + sign(dH)·dH_offset                       dH = H_c − H_ind (ft)
//   on law entry:  k = Hdot / r;  dH_offset = |500/|k| − 100|;  r += sign(r)·dH_offset;  k = Hdot / r;  maxH_dot = |Hdot|
//   Hdot_c = clamp(k·r, ±maxH_dot)
// On entry, r already contains the dH_offset of the PREVIOUS capture (state is only reset at power-up), so from the
// second capture of a flight on, k and the new offset are computed from a too-large distance. This probe shows the
// effect: V/S command right after entry and the V/S the law arrives with at the target, actual vs without the stale
// offset.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define private public
#include "FmgcComputer.h"
#include "FmgcOuterLoops.h"
#undef private

#include "replay_core.h"

namespace {
double sgn(double x) {
  return x < 0 ? -1.0 : (x > 0 ? 1.0 : 0.0);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: fmgc-probe-alt-capture <file.fdr>\n");
    return 2;
  }
  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(argv[1], {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    return 2;
  }
  auto& dw = computer->LawMDLOBJ1.FmgcOuterLoops_DWork;
  double prevOffset = dw.dH_offset, prevK = dw.k, prevMax = dw.maxH_dot;
  bool prevActive = false;
  int captures = 0;
  std::printf("%9s %4s %8s %8s %10s %10s %7s | %8s %8s | %8s %8s | %4s %6s %6s %6s %6s %6s\n", "t (s)", "#", "dH ft",
              "Hdot", "oldOff", "newOff", "k", "VScmd+1", "intended", "VS@tgt", "intended", "gnd", "RA", "VS+5s", "VS+10s",
              "VS+20s", "dur s");
  struct Pending {
    double t0;
    double vs[3];
    bool done[3];
    bool open;
  } pend{};
  Replayer::Frame f;
  while (replayer.next(f)) {
    bool active = dw.wasActive_a;
    double t = f.fdr->base.simulation_time_s;
    double vsNow = f.fdr->fmgc1.bus_inputs.ir_own_bus.inertial_vertical_speed_ft_s.Data;
    if (pend.open) {
      const double at[3]{5, 10, 20};
      for (int i = 0; i < 3; i++) {
        if (!pend.done[i] && t - pend.t0 >= at[i]) {
          pend.vs[i] = active ? vsNow : NAN;
          pend.done[i] = true;
        }
      }
      if (!active) {
        std::printf(" %6.0f %6.0f %6.0f %6.1f\n", pend.vs[0], pend.vs[1], pend.vs[2], t - pend.t0);
        pend.open = false;
      }
    }
    if (active && !prevActive) {
      captures++;
      double oldOff = prevOffset;
      double newOff = dw.dH_offset;
      double k = dw.k;
      const auto& o = computer->getExternalOutputs().out;
      double dH = o.ap_fd_logic.alt_sel_or_cstr - o.logic.altitude_indicated_ft.Data;  // the law's H_c − H_ind
      double s = sgn(dH);
      double hdot = dw.maxH_dot * (k * s < 0 ? -1.0 : 1.0);
      double vsNext = std::fmax(-dw.maxH_dot, std::fmin(dw.maxH_dot, k * (dH + s * newOff)));
      double vsTarget = k * s * newOff;
      // without the stale offset
      double k1 = hdot / dH;
      double newOffI = std::abs(500.0 / std::abs(k1) - 100.0);
      double kI = hdot / (dH + s * newOffI);
      double vsNextI = std::fmax(-dw.maxH_dot, std::fmin(dw.maxH_dot, kI * (dH + s * newOffI)));
      double vsTargetI = kI * s * newOffI;
      std::printf("%9.1f %4d %8.0f %8.0f %10.3g %10.3g %7.3f | %8.0f %8.0f | %8.0f %8.0f | %4d %6.0f", t, captures, dH,
                  hdot, oldOff, newOff, k, vsNext, vsNextI, vsTarget, vsTargetI,
                  static_cast<int>(computer->getExternalOutputs().out.logic.on_ground),
                  f.fdr->fmgc1.bus_inputs.ra_own_bus.radio_height_ft.Data);
      pend = {t, {NAN, NAN, NAN}, {false, false, false}, true};
    }
    prevActive = active;
    prevOffset = dw.dH_offset;
    prevK = dw.k;
    prevMax = dw.maxH_dot;
  }
  (void)prevK;
  (void)prevMax;
  return 0;
}
