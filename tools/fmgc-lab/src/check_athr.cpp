// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable autothrust (readable/athr*.cpp) against the generated FmgcComputer.
//
//   resync (default): the readable state starts each step from the generated state (src/athr_resync.h), the previous
//     outputs are the generated ones, and the N1 stage gets the generated mode outputs, so a mismatch points at the
//     step and the stage where the logic differs;
//   --free: the readable autothrust runs on its own state and its own previous outputs, and the N1 stage gets the
//     readable mode outputs (fields not rewritten yet still come from the generated code).
//
// Inputs are this frame's generated out.data / out.logic / out.ap_fd_logic (the autothrust reads those buses and the
// generated code does not change them afterwards). Every rewritten field of out.athr is compared exactly every step.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>

// The generated model keeps its state and parameters private; the checker reads them (test-only access).
#define private public
#include "FmgcComputer.h"
#undef private

#include "athr.h"
#include "athr_resync.h"
#include "input_fuzzer.h"
#include "replay_core.h"
#include "scenario_approach.h"

namespace A = fmgc::readable::athr;
using DWork = FmgcComputer::D_Work_FmgcComputer_T;

namespace {

double fieldValue(const base_fmgc_athr_outputs& o, int f) {
  switch (f) {
    case A::AthrActive: return o.athr_active;
    case A::AthrLimited: return o.athr_limited;
    case A::AlphaFloorMode: return o.alpha_floor_mode_active;
    case A::ThrustMode: return o.thrust_mode_active;
    case A::ThrustTargetIdle: return o.thrust_target_idle;
    case A::SpeedMachMode: return o.speed_mach_mode_active;
    case A::RetardMode: return o.retard_mode_active;
    case A::FmaMode: return static_cast<double>(static_cast<int>(o.fma_mode));
    case A::FmaMessage: return static_cast<double>(static_cast<int>(o.fma_message));
    case A::N1Command: return o.n1_c_percent;
    default: return std::nan("");
  }
}

void copyField(base_fmgc_athr_outputs& to, const base_fmgc_athr_outputs& from, int f) {
  switch (f) {
    case A::AthrActive: to.athr_active = from.athr_active; break;
    case A::AthrLimited: to.athr_limited = from.athr_limited; break;
    case A::AlphaFloorMode: to.alpha_floor_mode_active = from.alpha_floor_mode_active; break;
    case A::ThrustMode: to.thrust_mode_active = from.thrust_mode_active; break;
    case A::ThrustTargetIdle: to.thrust_target_idle = from.thrust_target_idle; break;
    case A::SpeedMachMode: to.speed_mach_mode_active = from.speed_mach_mode_active; break;
    case A::RetardMode: to.retard_mode_active = from.retard_mode_active; break;
    case A::FmaMode: to.fma_mode = from.fma_mode; break;
    case A::FmaMessage: to.fma_message = from.fma_message; break;
    case A::N1Command: to.n1_c_percent = from.n1_c_percent; break;
    default: break;
  }
}

bool same(double a, double b) {
  if (std::isnan(a) || std::isnan(b)) {
    return std::isnan(a) && std::isnan(b);
  }
  return std::memcmp(&a, &b, sizeof(double)) == 0 || a == b;
}

struct Stats {
  bool implemented = false;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0;
  double firstReadable = 0;
  double maxDiff = 0;
  long trueSteps = 0;  // coverage for flags: steps where the generated value was non-zero
  long rises = 0;      // coverage: zero -> non-zero transitions
  double prev = 0;
};

}  // namespace

int main(int argc, char** argv) {
  bool freeRunning = false;
  long fuzzSeed = -1, approachSeed = -1;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--free") {
      freeRunning = true;
    } else if (a == "--fuzz" && i + 1 < argc) {
      fuzzSeed = std::atol(argv[++i]);
    } else if (a == "--approach" && i + 1 < argc) {
      approachSeed = std::atol(argv[++i]);
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: fmgc-check-athr [--free] [--fuzz SEED] [--approach SEED] <file.fdr>\n");
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

  auto pre = std::make_unique<DWork>();
  A::Autothrust athr;
  athr.reset();
  base_fmgc_athr_outputs lastReadable{};
  bool haveLastReadable = false;
  Stats stats[A::kFieldCount];
  std::map<int, long> fmaModeSteps, fmaMessageSteps;  // coverage of the generated FMA values
  long steps = 0, checkedSteps = 0;

  replayer.beforeStep = [&]() {
    if (scenario) {
      scenario->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    *pre = computer->FmgcComputer_DWork;
  };
  replayer.afterStep = [&]() {
    steps++;
    if (!replayer.inputs().sim_data.computer_running) {
      haveLastReadable = false;
      return;
    }
    const fmgc_outputs& out = computer->getExternalOutputs().out;
    const base_fmgc_athr_outputs& gen = out.athr;
    bool restarted = !pre->Runtime_MODE;

    base_fmgc_athr_outputs last = restarted ? FmgcComputer::FmgcComputer_P.Delay1_InitialCondition : pre->Delay1_DSTATE;
    if (restarted) {
      athr.reset();
    } else if (!freeRunning || !haveLastReadable) {
      resyncAthrModes(athr.modesState(), *pre);
      resyncAthrN1(athr.n1State(), *pre);
    }
    if (freeRunning && haveLastReadable && !restarted) {
      for (int f = 0; f < A::kFieldCount; f++) {
        if (stats[f].implemented) {
          copyField(last, lastReadable, f);
        }
      }
    }

    A::Inputs in{out.data.time.dt, out.data, out.logic, out.ap_fd_logic, last};
    A::Result r;
    double t = out.data.time.simulation_time;
    auto check = [&](int f) {
      Stats& st = stats[f];
      double g = fieldValue(gen, f);
      st.trueSteps += g != 0.0;
      st.rises += g != 0.0 && st.prev == 0.0;
      st.prev = g;
      if (!r.rewritten[f]) {
        return;
      }
      st.implemented = true;
      double v = fieldValue(r.out, f);
      if (same(g, v)) {
        return;
      }
      if (st.firstStep < 0) {
        st.firstStep = steps;
        st.firstTime = t;
        st.firstGenerated = g;
        st.firstReadable = v;
      }
      st.mismatches++;
      if (!std::isnan(g) && !std::isnan(v)) {
        st.maxDiff = std::fmax(st.maxDiff, std::fabs(g - v));
      }
    };

    athr.updateModes(in, r);
    for (int f = 0; f < A::N1Command; f++) {
      check(f);
      if (!r.rewritten[f] || !freeRunning) {
        copyField(r.out, gen, f);  // the N1 stage sees generated mode outputs (resync, or not rewritten)
      }
    }
    athr.updateN1(in, r);
    check(A::N1Command);
    if (!r.rewritten[A::N1Command]) {
      copyField(r.out, gen, A::N1Command);
    }
    fmaModeSteps[static_cast<int>(gen.fma_mode)]++;
    fmaMessageSteps[static_cast<int>(gen.fma_message)]++;
    lastReadable = r.out;
    haveLastReadable = true;
    checkedSteps++;
  };

  Replayer::Frame fr;
  while (replayer.next(fr)) {
  }

  std::string mode = freeRunning ? "free-running" : "resync";
  if (fuzzer) {
    mode += ", fuzzed (seed " + std::to_string(fuzzSeed) + ", " + std::to_string(fuzzer->episodes()) + " episodes)";
  }
  if (scenario) {
    mode += ", approach scenarios (seed " + std::to_string(approachSeed) + ")";
  }
  std::printf("file     %s\nmode     %s\nsteps    %ld (%ld checked, computer running)\n", path.c_str(), mode.c_str(),
              steps, checkedSteps);
  std::printf("\n%-24s %6s %9s %10s %10s %14s %14s %10s %10s %7s\n", "field", "impl", "mismatch", "first", "t_first",
              "generated", "readable", "max|diff|", "nonzero", "rises");
  long implemented = 0, failing = 0;
  for (int f = 0; f < A::kFieldCount; f++) {
    const Stats& st = stats[f];
    implemented += st.implemented;
    failing += st.mismatches != 0;
    std::printf("%-24s %6s %9ld %10ld %10.2f %14.9g %14.9g %10.3g %10ld %7ld\n", A::fieldName(f),
                st.implemented ? "yes" : "no", st.mismatches, st.firstStep, st.firstTime, st.firstGenerated,
                st.firstReadable, st.maxDiff, st.trueSteps, st.rises);
  }
  std::printf("\nfma_mode steps:");
  for (auto& [v, n] : fmaModeSteps) {
    std::printf(" %d:%ld", v, n);
  }
  std::printf("\nfma_message steps:");
  for (auto& [v, n] : fmaMessageSteps) {
    std::printf(" %d:%ld", v, n);
  }
  std::printf("\n\n%ld fields rewritten, %ld with mismatches\n", implemented, failing);
  return failing == 0 ? 0 : 1;
}
