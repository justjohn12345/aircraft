// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Experiment: what does the stock FMGC do if ALT* engages on the ground with the V/S exactly 0?
// (docs/laws/alt-capture.md Finding 2: k = 0, dH_offset = inf, then k·r = 0·inf = NaN for every later capture.)
//
// Replays a recording twice through the generated model: as recorded, and with the IR inertial V/S (own, opposite,
// 3) forced to exactly 0.0 while on the ground. Reports the capture law state, the first non-finite AP/FD pitch
// command, and how the AP and FD pitch commands and the ELAC pitch word differ from the as-recorded replay.
// Usage: fmgc-probe-zero-vs <file.fdr>

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#define private public
#include "FmgcComputer.h"
#include "FmgcOuterLoops.h"
#undef private

#include "replay_core.h"

namespace {

struct Sample {
  double t;
  bool running, onGround, apEngaged, altAcq;
  double apTheta, fdTheta, elacPitch, offset, k;
  int law;
};

std::vector<Sample> run(const char* path, bool zeroVsOnGround) {
  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    std::exit(2);
  }
  bool lastOnGround = true;
  replayer.beforeStep = [&]() {
    if (!zeroVsOnGround || !lastOnGround) {
      return;
    }
    auto& b = replayer.mutableInputs().bus_inputs;
    b.ir_own_bus.inertial_vertical_speed_ft_s.Data = 0.0F;
    b.ir_opp_bus.inertial_vertical_speed_ft_s.Data = 0.0F;
    b.ir_3_bus.inertial_vertical_speed_ft_s.Data = 0.0F;
  };
  std::vector<Sample> out;
  Replayer::Frame f;
  auto& dw = computer->LawMDLOBJ1.FmgcOuterLoops_DWork;
  while (replayer.next(f)) {
    const fmgc_outputs& o = computer->getExternalOutputs().out;
    lastOnGround = o.logic.on_ground;
    Sample s{};
    s.t = f.fdr->base.simulation_time_s;
    s.running = f.running;
    s.onGround = o.logic.on_ground;
    s.apEngaged = o.logic.ap_own_engaged;
    s.altAcq = o.ap_fd_logic.longitudinal_modes.alt_acq_active;
    s.apTheta = o.ap_fd_outer_loops.autopilot.Theta_c_deg;
    s.fdTheta = o.ap_fd_outer_loops.flight_director.Theta_c_deg;
    s.elacPitch = o.bus_outputs.fmgc_b_bus.delta_q_cmd_deg.Data;
    s.offset = dw.dH_offset;
    s.k = dw.k;
    s.law = static_cast<int>(o.ap_fd_logic.active_longitudinal_law);
    out.push_back(s);
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: fmgc-probe-zero-vs <file.fdr>\n");
    return 2;
  }
  auto base = run(argv[1], false);
  auto zero = run(argv[1], true);
  size_t n = std::min(base.size(), zero.size());

  // Capture law state: report every change of dH_offset in the zero-V/S run.
  double lastOff = NAN;
  int changes = 0;
  for (size_t i = 0; i < n; i++) {
    const Sample& z = zero[i];
    bool same = (std::isnan(z.offset) && std::isnan(lastOff)) || z.offset == lastOff;
    if (!same && changes < 12) {
      std::printf("t %8.1f  zero-V/S run: dH_offset %-12g k %-10g (as recorded: offset %-12g k %g)  gnd %d ALT* %d\n",
                  z.t, z.offset, z.k, base[i].offset, base[i].k, z.onGround, z.altAcq);
      changes++;
    }
    lastOff = z.offset;
  }

  long nonFiniteAp = 0, nonFiniteFd = 0, nonFiniteElac = 0, apFrames = 0, apDiffFrames = 0;
  double firstNonFinite = NAN, maxApDiff = 0, maxApDiffT = 0, maxFdDiff = 0;
  for (size_t i = 0; i < n; i++) {
    const Sample& b = base[i];
    const Sample& z = zero[i];
    if (!z.running) {
      continue;
    }
    bool apBad = !std::isfinite(z.apTheta), fdBad = !std::isfinite(z.fdTheta), elacBad = !std::isfinite(z.elacPitch);
    nonFiniteAp += apBad;
    nonFiniteFd += fdBad;
    nonFiniteElac += elacBad;
    if ((apBad || fdBad || elacBad) && std::isnan(firstNonFinite)) {
      firstNonFinite = z.t;
      std::printf("first non-finite command at t %.1f: AP theta %g, FD theta %g, ELAC pitch word %g, law %d, ALT* %d\n",
                  z.t, z.apTheta, z.fdTheta, z.elacPitch, z.law, z.altAcq);
    }
    if (z.apEngaged) {
      apFrames++;
      double d = std::fabs(z.apTheta - b.apTheta);
      if (!(d <= 1e-9)) {
        apDiffFrames++;
      }
      if (d > maxApDiff || std::isnan(d)) {
        maxApDiff = d;
        maxApDiffT = z.t;
      }
    }
    double fd = std::fabs(z.fdTheta - b.fdTheta);
    if (fd > maxFdDiff) {
      maxFdDiff = fd;
    }
  }
  std::printf("frames %zu; non-finite: AP theta %ld, FD theta %ld, ELAC pitch word %ld\n", n, nonFiniteAp, nonFiniteFd,
              nonFiniteElac);
  std::printf("AP engaged frames %ld, AP theta differs from as-recorded in %ld; max |diff| %g deg at t %.1f; max FD diff %g\n",
              apFrames, apDiffFrames, maxApDiff, maxApDiffT, maxFdDiff);
  return 0;
}
