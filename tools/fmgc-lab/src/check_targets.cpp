// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable targets block (readable/targets.cpp) against the generated FmgcComputer.
//
//   resync (default): the readable state and values start each step from the generated memories (src/targets_resync.h)
//     and every field is compared and then reset to the generated value right after its readable update, so a mismatch
//     points at one field's logic in one step;
//   --free: the readable block keeps its own state and its own previous outputs (`last`), as it would in the sim;
//   --fuzz SEED, --approach SEED, --targets SEED: input perturbations (src/input_fuzzer.h, scenario_approach.h,
//     scenario_targets.h), combinable;
//   --events: also prints a timeline of the speed-mode events (generated values): selected/managed/Mach changes, SPD
//     knob and SPD/MACH pb edges, FMS Mach/speed requests and presets, with the FCU and FMS speeds.
//
// Inputs are this frame's generated out.logic and out.ap_fd_logic mode flags (computed before the block, not changed
// by it) and the previous frame's ap_fd_logic (Delay_DSTATE). Every field is compared exactly (doubles bit for bit).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

// The generated model keeps its state and parameters private; the checker reads them (test-only access).
#define private public
#include "FmgcComputer.h"
#undef private

#include "input_fuzzer.h"
#include "replay_core.h"
#include "scenario_approach.h"
#include "scenario_targets.h"
#include "targets.h"
#include "targets_resync.h"

namespace T = fmgc::readable::targets;
using DWork = FmgcComputer::D_Work_FmgcComputer_T;
using BlockIO = FmgcComputer::BlockIO_FmgcComputer_T;

namespace {

bool same(double a, double b) {
  if (std::isnan(a) || std::isnan(b)) {
    return std::isnan(a) && std::isnan(b);
  }
  return std::memcmp(&a, &b, sizeof(double)) == 0 || a == b;
}

struct Stats {
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0;
  double firstReadable = 0;
  double maxDiff = 0;
  long nonzeroSteps = 0;  // coverage: steps where the generated value was non-zero (flags: true)
  long changes = 0;       // coverage: steps where the generated value changed
  double prev = 0;
  bool havePrev = false;
};

}  // namespace

int main(int argc, char** argv) {
  bool freeRunning = false, events = false;
  long fuzzSeed = -1, approachSeed = -1, targetsSeed = -1;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--free") {
      freeRunning = true;
    } else if (a == "--events") {
      events = true;
    } else if (a == "--fuzz" && i + 1 < argc) {
      fuzzSeed = std::atol(argv[++i]);
    } else if (a == "--approach" && i + 1 < argc) {
      approachSeed = std::atol(argv[++i]);
    } else if (a == "--targets" && i + 1 < argc) {
      targetsSeed = std::atol(argv[++i]);
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr,
                 "usage: fmgc-check-targets [--free] [--fuzz SEED] [--approach SEED] [--targets SEED] [--events] "
                 "<file.fdr>\n");
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
  std::unique_ptr<ApproachScenario> approach;
  if (approachSeed >= 0) {
    approach = std::make_unique<ApproachScenario>(static_cast<uint64_t>(approachSeed));
  }
  std::unique_ptr<TargetsScenario> scenario;
  if (targetsSeed >= 0) {
    scenario = std::make_unique<TargetsScenario>(static_cast<uint64_t>(targetsSeed));
  }

  auto pre = std::make_unique<DWork>();
  auto preB = std::make_unique<BlockIO>();
  T::Targets targets;
  T::Values v;
  targets.initialize(v);
  T::Values lastReadable;
  bool haveLastReadable = false;
  Stats stats[T::kFieldCount];
  long condSteps[T::kCondCount] = {};
  long steps = 0, checkedSteps = 0;
  base_fmgc_ap_fd_logic_outputs prevOut{};
  uint32_t prevWord1 = 0;
  bool prevMachReq = false, prevSpdReq = false, prevPreset = false;

  replayer.beforeStep = [&]() {
    if (approach) {
      approach->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    if (scenario) {
      scenario->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    *pre = computer->FmgcComputer_DWork;
    *preB = computer->FmgcComputer_B;
  };
  replayer.afterStep = [&]() {
    steps++;
    const fmgc_inputs& in = replayer.inputs();
    if (!in.sim_data.computer_running) {
      haveLastReadable = false;
      return;
    }
    const DWork& post = computer->FmgcComputer_DWork;
    const BlockIO& postB = computer->FmgcComputer_B;
    const fmgc_outputs& out = computer->getExternalOutputs().out;
    bool restarted = !pre->Runtime_MODE;

    base_fmgc_ap_fd_logic_outputs last = restarted ? FmgcComputer::FmgcComputer_P.Delay_InitialCondition
                                                   : pre->Delay_DSTATE;
    if (restarted) {
      targets.reset(v);
    } else if (!freeRunning || !haveLastReadable) {
      resyncTargets(targets.state(), v, *pre, *preB);
    }
    if (freeRunning && haveLastReadable && !restarted) {
      for (int f = 0; f < T::kFieldCount; f++) {
        putTargetOutput(last, f, lastReadable);
      }
    }
    // Delay_DSTATE_fe: 1 from the initialisation until the end of the first running step
    bool firstFrame = restarted || pre->Delay_DSTATE_fe;

    T::Inputs ti{in.time.dt, in, out.logic, out.ap_fd_logic, last, firstFrame};
    T::Trace trace;
    double t = in.time.simulation_time;
    for (int f = 0; f < T::kFieldCount; f++) {
      targets.update(static_cast<T::Field>(f), ti, v, trace);
      double g = generatedTargetValue(f, post, postB, out);
      Stats& st = stats[f];
      st.nonzeroSteps += g != 0.0;
      st.changes += st.havePrev && !same(g, st.prev);
      st.prev = g;
      st.havePrev = true;
      // SpdTarget's update computes both targets: PfdSpdTarget is compared after its own (empty) update
      double r = v.get(f);
      if (!same(g, r)) {
        if (st.firstStep < 0) {
          st.firstStep = steps;
          st.firstTime = t;
          st.firstGenerated = g;
          st.firstReadable = r;
        }
        st.mismatches++;
        if (!std::isnan(g) && !std::isnan(r)) {
          st.maxDiff = std::fmax(st.maxDiff, std::fabs(g - r));
        }
      }
      if (!freeRunning) {
        v.set(f, g);
      }
    }
    for (int c = 0; c < T::kCondCount; c++) {
      condSteps[c] += trace.c[c];
    }
    if (events) {
      const auto& a = out.ap_fd_logic;
      const auto& fms = in.fms_inputs;
      const auto& fcu = in.bus_inputs.fcu_bus;
      uint32_t w1 = fcu.fcu_discrete_word_1.Data >= 0 && fcu.fcu_discrete_word_1.Data < 4294967296.0F
                        ? static_cast<uint32_t>(fcu.fcu_discrete_word_1.Data)
                        : 0u;
      auto rise = [&](int bit) { return ((w1 >> (bit - 1)) & 1u) && !((prevWord1 >> (bit - 1)) & 1u); };
      std::string ev;
      if (restarted) ev += " RESTART";
      if (a.manual_spd_control_active != prevOut.manual_spd_control_active)
        ev += a.manual_spd_control_active ? " +SELECTED" : " -SELECTED";
      if (a.auto_spd_control_active != prevOut.auto_spd_control_active)
        ev += a.auto_spd_control_active ? " +MANAGED" : " -MANAGED";
      if (a.mach_control_active != prevOut.mach_control_active) ev += a.mach_control_active ? " +MACH" : " -MACH";
      if (a.alt_soft_mode_active != prevOut.alt_soft_mode_active) ev += a.alt_soft_mode_active ? " +SOFT" : " -SOFT";
      if (rise(11)) ev += " spd-knob-push";
      if (rise(12)) ev += " spd-knob-pull";
      if (rise(21)) ev += " spd/mach-pb";
      if (fms.fms_mach_mode_activate && !prevMachReq) ev += " fms-mach-req";
      if (fms.fms_spd_mode_activate && !prevSpdReq) ev += " fms-spd-req";
      if (fms.preset_spd_mach_activate && !prevPreset) ev += " preset";
      if (!ev.empty()) {
        std::printf("event t=%10.2f%s | fcu spd %.1f (ssm %u) mach %.3f (ssm %u) | v_managed %.2f phase %d | V_c %.2f "
                    "pfd %.2f | CAS %.2f\n",
                    t, ev.c_str(), fcu.selected_spd_kts.Data, fcu.selected_spd_kts.SSM, fcu.selected_mach.Data,
                    fcu.selected_mach.SSM, fms.v_managed_kts, static_cast<int>(fms.fms_flight_phase), a.spd_target_kts,
                    a.pfd_spd_target_kts, out.logic.adr_computation_data.airspeed_computed_kn.Data);
      }
      prevOut = a;
      prevWord1 = w1;
      prevMachReq = fms.fms_mach_mode_activate;
      prevSpdReq = fms.fms_spd_mode_activate;
      prevPreset = fms.preset_spd_mach_activate;
    }
    lastReadable = v;
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
  if (approach) {
    mode += ", approach scenarios (seed " + std::to_string(approachSeed) + ")";
  }
  if (scenario) {
    mode += ", targets scenarios (seed " + std::to_string(targetsSeed) + ")";
  }
  std::printf("file     %s\nmode     %s\nsteps    %ld (%ld checked, computer running)\n", path.c_str(), mode.c_str(),
              steps, checkedSteps);
  std::printf("\n%-16s %6s %9s %10s %10s %14s %14s %10s %10s %8s\n", "field", "impl", "mismatch", "first", "t_first",
              "generated", "readable", "max|diff|", "nonzero", "changes");
  long failing = 0;
  for (int f = 0; f < T::kFieldCount; f++) {
    const Stats& st = stats[f];
    failing += st.mismatches != 0;
    std::printf("%-16s %6s %9ld %10ld %10.2f %14.9g %14.9g %10.3g %10ld %8ld\n", T::fieldName(f), "yes", st.mismatches,
                st.firstStep, st.firstTime, st.firstGenerated, st.firstReadable, st.maxDiff, st.nonzeroSteps,
                st.changes);
  }
  std::printf("\nconditions:");
  for (int c = 0; c < T::kCondCount; c++) {
    std::printf(" %s:%ld", T::condName(c), condSteps[c]);
  }
  std::printf("\n\n%d fields rewritten, %ld with mismatches\n", static_cast<int>(T::kFieldCount), failing);
  return failing == 0 ? 0 : 1;
}
