// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable vertical control laws (readable/laws*.cpp) against the generated FmgcOuterLoops,
// using an instrumented copy of the stock model (scripts/gen_hooked_model.py): every call of FmgcOuterLoops::step()
// is captured with its inputs and outputs, and probes report each law's FD/AP command (src/probes/*.txt).
//
// Checked every step:
//   - each rewritten law's FD and AP command against its probes (whether or not the law is active);
//   - the output stage (FD and AP theta command) against the generated outputs, fed with the generated law commands
//     (default) or, with --free, with the readable ones (laws not rewritten yet still use the generated commands);
//   - the flare law outputs, once rewritten;
//   - each lateral law's roll, sideslip and nosewheel command (readable/laws_lateral.cpp, probes in
//     src/probes/lateral.txt), Phi_loc_c, and the lateral output stage (FD/AP roll, FD/AP sideslip, nosewheel), fed
//     like the vertical one. The lateral laws run first; with --free the vertical laws read the readable AP sideslip
//     command (the flare law uses it).
// Exact comparison by default (--tol X for an absolute tolerance).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "FmgcComputer.h"
#include "input_fuzzer.h"
#include "lab_hooks.h"
#include "laws.h"
#include "laws_lateral.h"
#include "outer_io.gen.h"
#include "replay_core.h"
#include "scenario_approach.h"

namespace L = fmgc::readable::laws;

namespace {

struct Stats {
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0;
  double firstReadable = 0;
  double maxDiff = 0;
  long checked = 0;
};

struct Checker {
  double tol = 0;
  long step = 0;
  double time = 0;
  std::map<std::string, Stats> stats;

  void compare(const std::string& name, double generated, double readable) {
    Stats& s = stats[name];
    s.checked++;
    bool same;
    if (std::isnan(generated) || std::isnan(readable)) {
      same = std::isnan(generated) && std::isnan(readable);
    } else if (tol == 0) {
      same = std::memcmp(&generated, &readable, sizeof(double)) == 0 || generated == readable;
    } else {
      same = std::fabs(generated - readable) <= tol;
    }
    if (same) {
      return;
    }
    if (s.firstStep < 0) {
      s.firstStep = step;
      s.firstTime = time;
      s.firstGenerated = generated;
      s.firstReadable = readable;
    }
    s.mismatches++;
    if (!std::isnan(generated) && !std::isnan(readable)) {
      s.maxDiff = std::fmax(s.maxDiff, std::fabs(generated - readable));
    }
  }
};

double probe(const char* name) {
  auto& p = labHooks().probes;
  auto it = p.find(name);
  return it == p.end() ? std::nan("") : it->second;
}

}  // namespace

int main(int argc, char** argv) {
  bool freeRunning = false;
  long fuzzSeed = -1, approachSeed = -1;
  double tol = 0;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--free") {
      freeRunning = true;
    } else if (a == "--fuzz" && i + 1 < argc) {
      fuzzSeed = std::atol(argv[++i]);
    } else if (a == "--approach" && i + 1 < argc) {
      approachSeed = std::atol(argv[++i]);
    } else if (a == "--tol" && i + 1 < argc) {
      tol = std::atof(argv[++i]);
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: fmgc-check-laws [--free] [--fuzz SEED] [--approach SEED] [--tol X] <file.fdr>\n");
    return 2;
  }

  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    return 2;
  }
  std::unique_ptr<InputFuzzer> fuzzer;
  if (fuzzSeed >= 0) {
    fuzzer = std::make_unique<InputFuzzer>(static_cast<uint64_t>(fuzzSeed));
  }
  std::unique_ptr<ApproachScenario> scenario;
  if (approachSeed >= 0) {
    scenario = std::make_unique<ApproachScenario>(static_cast<uint64_t>(approachSeed));
  }
  replayer.beforeStep = [&]() {
    if (scenario) {
      scenario->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
  };

  L::VerticalLaws laws;
  laws.reset();
  Checker check;
  check.tol = tol;
  long lawSteps = 0;
  const char* lawProbe[L::kLawCount] = {nullptr, "alt_hold", "alt_acq", "spd_mach", "vs",
                                        "fpa",   "gs",       "flare",   "srs",      "vpath"};

  L::LateralLaws lateral;
  lateral.reset();
  long lateralSteps[L::kLateralLawCount + 1]{};  // steps per active lateral law (last: out of range)
  long lateralApSteps[L::kLateralLawCount + 1]{};

  labHooks().onOuterReset = [&]() {
    laws.reset();
    lateral.reset();
  };
  labHooks().onOuterStep = [&]() {
    lawSteps++;
    check.step = lawSteps;
    ap_laws_output io{};
    unpackOuterInputs(labHooks().outerIn, io);
    ap_laws_output gen = io;
    unpackOuterOutputs(labHooks().outerOut, gen);
    check.time = io.time.simulation_time;

    // ---- lateral laws (computed before the vertical ones in the generated step)
    int latLaw = static_cast<int>(io.input.lateral_law);
    latLaw = latLaw >= 0 && latLaw < L::kLateralLawCount ? latLaw : L::kLateralLawCount;
    lateralSteps[latLaw]++;
    lateralApSteps[latLaw] += io.input.ap_engaged;
    L::LateralResults latReadable;
    lateral.computeLaws(io, latReadable);
    L::LateralCommand latGenerated[L::kLateralLawCount];
    for (int k = 0; k < L::kLateralLawCount; k++) {
      std::string n = L::lateralLawName(k);
      latGenerated[k].phi = probe(("lat.phi." + n).c_str());
      latGenerated[k].beta = probe(("lat.beta." + n).c_str());
      latGenerated[k].nosewheel = probe(("lat.nose." + n).c_str());
      if (latReadable.rewritten[k]) {
        check.compare("lat." + n + ".phi", latGenerated[k].phi, latReadable.cmd[k].phi);
        check.compare("lat." + n + ".beta", latGenerated[k].beta, latReadable.cmd[k].beta);
        check.compare("lat." + n + ".nose", latGenerated[k].nosewheel, latReadable.cmd[k].nosewheel);
      }
    }
    check.compare("lat.phi_loc_c", gen.output.Phi_loc_c, latReadable.phiLocDeg);
    L::LateralCommand latFeed[L::kLateralLawCount];
    for (int k = 0; k < L::kLateralLawCount; k++) {
      latFeed[k] = freeRunning && latReadable.rewritten[k] ? latReadable.cmd[k] : latGenerated[k];
    }
    L::LateralOutputs lo{};
    bool latOutputs = lateral.computeOutputs(io, latFeed, lo);
    if (latOutputs) {
      check.compare("output.fd_phi", gen.output.flight_director.Phi_c_deg, lo.fdPhiDeg);
      check.compare("output.ap_phi", gen.output.autopilot.Phi_c_deg, lo.apPhiDeg);
      check.compare("output.fd_beta", gen.output.flight_director.Beta_c_deg, lo.fdBetaDeg);
      check.compare("output.ap_beta", gen.output.autopilot.Beta_c_deg, lo.apBetaDeg);
      check.compare("output.nosewheel", gen.output.Nosewheel_c, lo.nosewheel);
    }

    // The vertical laws read the lateral AP sideslip command (flare law): the generated one, or with --free the
    // readable one.
    io.output.autopilot.Beta_c_deg = freeRunning && latOutputs ? lo.apBetaDeg : gen.output.autopilot.Beta_c_deg;

    L::LawResults readable;
    laws.computeLaws(io, readable);

    L::LawCommand generated[L::kLawCount];
    generated[L::None] = {0.0, 0.0};
    for (int k = 1; k < L::kLawCount; k++) {
      generated[k].fd = probe((std::string("fd.") + lawProbe[k]).c_str());
      generated[k].ap = probe((std::string("ap.") + lawProbe[k]).c_str());
      if (readable.rewritten[k]) {
        check.compare(std::string("law.") + lawProbe[k] + ".fd", generated[k].fd, readable.cmd[k].fd);
        check.compare(std::string("law.") + lawProbe[k] + ".ap", generated[k].ap, readable.cmd[k].ap);
      }
    }
    if (readable.flareValid) {
      const auto& g = gen.output.flare_law;
      const auto& r = readable.flare;
      check.compare("flare.condition_Flare", g.condition_Flare, r.condition_Flare);
      check.compare("flare.H_dot_radio_fpm", g.H_dot_radio_fpm, r.H_dot_radio_fpm);
      check.compare("flare.H_dot_c_fpm", g.H_dot_c_fpm, r.H_dot_c_fpm);
      check.compare("flare.delta_Theta_H_dot_deg", g.delta_Theta_H_dot_deg, r.delta_Theta_H_dot_deg);
      check.compare("flare.delta_Theta_bz_deg", g.delta_Theta_bz_deg, r.delta_Theta_bz_deg);
      check.compare("flare.delta_Theta_bx_deg", g.delta_Theta_bx_deg, r.delta_Theta_bx_deg);
      check.compare("flare.delta_Theta_beta_c_deg", g.delta_Theta_beta_c_deg, r.delta_Theta_beta_c_deg);
    }

    // output stage
    L::LawCommand feed[L::kLawCount];
    for (int k = 0; k < L::kLawCount; k++) {
      bool useReadable = freeRunning && readable.rewritten[k];
      feed[k] = useReadable ? readable.cmd[k] : generated[k];
    }
    L::PitchOutputs po{};
    if (laws.computeOutputs(io, feed, po)) {
      check.compare("output.fd_theta", gen.output.flight_director.Theta_c_deg, po.fdThetaDeg);
      check.compare("output.ap_theta", gen.output.autopilot.Theta_c_deg, po.apThetaDeg);
    }
  };

  Replayer::Frame fr;
  while (replayer.next(fr)) {
  }

  std::string mode = freeRunning ? "free-running" : "generated law commands into the output stage";
  if (fuzzer) {
    mode += ", fuzzed (seed " + std::to_string(fuzzSeed) + ", " + std::to_string(fuzzer->episodes()) + " episodes)";
  }
  if (scenario) {
    mode += ", approach scenarios (seed " + std::to_string(approachSeed) + ")";
  }
  std::printf("file     %s\nmode     %s\nsteps    %ld outer-loop steps\n", path.c_str(), mode.c_str(), lawSteps);
  std::printf("lateral  active steps (AP engaged):");
  for (int k = 0; k <= L::kLateralLawCount; k++) {
    std::printf(" %s %ld (%ld)", k < L::kLateralLawCount ? L::lateralLawName(k) : "other", lateralSteps[k],
                lateralApSteps[k]);
  }
  std::printf("\n");
  std::printf("\n%-30s %9s %9s %10s %10s %16s %16s %10s\n", "channel", "checked", "mismatch", "first", "t_first",
              "generated", "readable", "max|diff|");
  long failing = 0;
  for (auto& [name, s] : check.stats) {
    if (s.mismatches) {
      failing++;
    }
    std::printf("%-30s %9ld %9ld %10ld %10.2f %16.9g %16.9g %10.3g\n", name.c_str(), s.checked, s.mismatches,
                s.firstStep, s.firstTime, s.firstGenerated, s.firstReadable, s.maxDiff);
  }
  std::printf("\n%zu channels checked, %ld with mismatches\n", check.stats.size(), failing);
  return failing == 0 ? 0 : 1;
}
