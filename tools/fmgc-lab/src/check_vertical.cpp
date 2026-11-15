// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable vertical and lateral mode logic (readable/vertical*.cpp, lateral_modes.cpp)
// against the generated FmgcComputer, over a flight data recording. The generated model is the oracle (not the recording), so use the stock variant.
//
// Modes:
//   resync (default): every signal starts the frame from the generated value of the previous frame, and after each
//     readable update the signal is compared and then reset to the generated value, so a mismatch points at exactly
//     one signal's logic.
//   --free: the readable logic keeps its own state (its own previous outputs feed `last`), as it would in the sim.
//   --fuzz SEED, --approach SEED, --lateral SEED: input perturbations (src/input_fuzzer.h, scenario_approach.h,
//     scenario_lateral.h), combinable.
// Signals that are not rewritten yet always take the generated value.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define private public
#include "FmgcComputer.h"
#undef private

#include "input_fuzzer.h"
#include "scenario_approach.h"
#include "scenario_lateral.h"
#include "replay_core.h"
#include "vertical.h"
#include "vertical_latches.h"

#include "external_approach.h"
#include "external_climb_descent.h"
#include "external_lateral.h"
#include "external_vs_tcas.h"

using fmgc::readable::External;
using fmgc::readable::Frame;
using fmgc::readable::kSignalCount;
using fmgc::readable::Signal;
using fmgc::readable::Signals;
using fmgc::readable::VerticalModeLogic;

namespace {

using DWork = FmgcComputer::D_Work_FmgcComputer_T;

/** Generated value of a signal after this step. */
bool generatedValue(const DWork& post, const fmgc_outputs& out, Signal s) {
  if (const boolean_T* l = latchOf(post, s)) {
    return *l;
  }
  if (s == Signal::ActiveLateralLaw) {
    return out.ap_fd_logic.active_lateral_law != lateral_law::NONE;
  }
  if (s == Signal::AltSelOrCstr) {
    return out.ap_fd_logic.alt_sel_or_cstr != 0.0;
  }
  auto o = out.ap_fd_logic;
  boolean_T* field = outputOf(o, s);
  return field ? *field : false;
}

/** Fills the not-yet-rewritten parts of the FMGC from the generated model's outputs of this step. */
void fillExternal(External& e, const fmgc_outputs& out, const DWork& pre, const DWork& post) {
  e.logic = &out.logic;
  e.oppModeSync = out.ap_fd_logic.fmgc_opp_mode_sync;
  e.anyApFdEngaged = out.ap_fd_logic.any_ap_fd_engaged;
  e.modeReset = out.ap_fd_logic.longitudinal_mode_reset;
  e.tcasVsTarget = out.ap_fd_logic.tcas_vs_target;
  e.tcasRaCorrective = out.ap_fd_logic.tcas_ra_corrective;
  e.tcasModeAvailable = out.logic.tcas_mode_available;
  fillVsTcasExternal(e.vsTcas, out, pre, post);
  fillClimbDescentExternal(e.climbDescent, out, pre, post);
  fillApproachExternal(e.approach, out, pre, post);
  fillLateralExternal(e.lateral, out, pre, post);
}

struct Stats {
  bool implemented = false;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  bool firstGenerated = false;
  long lastStep = -1;
  long trueSteps = 0;  // coverage: steps where the generated signal was true
  long rises = 0;      // coverage: false -> true transitions
  bool prevGenerated = false;
};

}  // namespace

int main(int argc, char** argv) {
  bool freeRunning = false;
  long fuzzSeed = -1;
  long approachSeed = -1;
  long lateralSeed = -1;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--free") {
      freeRunning = true;
    } else if (a == "--fuzz" && i + 1 < argc) {
      fuzzSeed = std::atol(argv[++i]);
    } else if (a == "--approach" && i + 1 < argc) {
      approachSeed = std::atol(argv[++i]);
    } else if (a == "--lateral" && i + 1 < argc) {
      lateralSeed = std::atol(argv[++i]);
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      std::fprintf(stderr, "usage: fmgc-check-vertical [--free] [--fuzz SEED] [--approach SEED] [--lateral SEED] <file.fdr>\n");
      return 2;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr, "usage: fmgc-check-vertical [--free] [--fuzz SEED] [--approach SEED] [--lateral SEED] <file.fdr>\n");
    return 2;
  }

  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    return 2;
  }

  auto pre = std::make_unique<DWork>();
  VerticalModeLogic logic;
  Signals s;
  Signals lastReadable;  // free mode: readable outputs of the previous step
  bool haveLastReadable = false;
  std::vector<Stats> stats(kSignalCount);
  long steps = 0, checkedSteps = 0;
  constexpr size_t kLawCount = static_cast<size_t>(lateral_law::ROLL_OUT) + 1;
  long lawSteps[kLawCount] = {}, lawRises[kLawCount] = {};  // coverage of each active_lateral_law value
  lateral_law prevLaw = lateral_law::NONE;
  double simTime = 0;

  std::unique_ptr<InputFuzzer> fuzzer;
  if (fuzzSeed >= 0) {
    fuzzer = std::make_unique<InputFuzzer>(static_cast<uint64_t>(fuzzSeed));
  }
  std::unique_ptr<ApproachScenario> scenario;
  if (approachSeed >= 0) {
    scenario = std::make_unique<ApproachScenario>(static_cast<uint64_t>(approachSeed));
  }
  std::unique_ptr<LateralScenario> lateralScenario;
  if (lateralSeed >= 0) {
    lateralScenario = std::make_unique<LateralScenario>(static_cast<uint64_t>(lateralSeed));
  }
  replayer.beforeStep = [&]() {
    if (scenario) {
      scenario->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    if (lateralScenario) {
      lateralScenario->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    *pre = computer->FmgcComputer_DWork;
  };
  replayer.afterStep = [&]() {
    steps++;
    const fmgc_inputs& in = replayer.inputs();
    simTime = in.time.simulation_time;
    if (!in.sim_data.computer_running) {
      haveLastReadable = false;
      return;
    }
    const DWork& post = computer->FmgcComputer_DWork;
    const fmgc_outputs& out = computer->getExternalOutputs().out;
    bool restarted = !pre->Runtime_MODE;

    // previous frame's outputs as the logic sees them
    base_fmgc_ap_fd_logic_outputs last = restarted ? FmgcComputer::FmgcComputer_P.Delay_InitialCondition
                                                   : pre->Delay_DSTATE;
    if (restarted) {
      logic.reset(s);
    } else if (!freeRunning || !haveLastReadable) {
      for (size_t i = 0; i < kSignalCount; i++) {
        if (const boolean_T* l = latchOf(*pre, static_cast<Signal>(i))) {
          s.v[i] = *l;
        }
      }
    }
    if (freeRunning && haveLastReadable && !restarted) {
      for (size_t i = 0; i < kSignalCount; i++) {
        if (stats[i].implemented) {
          if (boolean_T* field = outputOf(last, static_cast<Signal>(i))) {
            *field = lastReadable.v[i];
          }
        }
      }
      if (stats[static_cast<size_t>(Signal::ActiveLateralLaw)].implemented) {
        last.active_lateral_law = lastReadable.activeLateralLaw;
      }
      if (stats[static_cast<size_t>(Signal::AltSelOrCstr)].implemented) {
        last.alt_sel_or_cstr = lastReadable.altSelOrCstr;
      }
    }

    External ext;
    fillExternal(ext, out, *pre, post);
    Frame f{in.time.dt, in, last, ext};
    for (size_t i = 0; i < kSignalCount; i++) {
      Signal id = static_cast<Signal>(i);
      bool generated = generatedValue(post, out, id);
      if (!logic.update(id, f, s)) {
        s.v[i] = generated;
        continue;
      }
      Stats& st = stats[i];
      st.implemented = true;
      st.trueSteps += generated;
      st.rises += generated && !st.prevGenerated;
      st.prevGenerated = generated;
      bool mismatch = s.v[i] != generated;
      if (id == Signal::ActiveLateralLaw) {
        // enum-valued: compared as a value (the bool is law != NONE); coverage per law value below
        lateral_law generatedLaw = out.ap_fd_logic.active_lateral_law;
        mismatch = s.activeLateralLaw != generatedLaw;
        lawSteps[static_cast<size_t>(generatedLaw)]++;
        lawRises[static_cast<size_t>(generatedLaw)] += generatedLaw != prevLaw;
        prevLaw = generatedLaw;
        if (!freeRunning) {
          s.activeLateralLaw = generatedLaw;
        }
      }
      if (id == Signal::AltSelOrCstr) {
        // value-valued: compared bit-exact (the bool is target != 0)
        const double generatedTarget = out.ap_fd_logic.alt_sel_or_cstr;
        mismatch = std::memcmp(&s.altSelOrCstr, &generatedTarget, sizeof(double)) != 0;
        if (!freeRunning) {
          s.altSelOrCstr = generatedTarget;
        }
      }
      if (mismatch) {
        if (st.firstStep < 0) {
          st.firstStep = steps;
          st.firstTime = simTime;
          st.firstGenerated = generated;
        }
        st.mismatches++;
        st.lastStep = steps;
      }
      if (!freeRunning) {
        s.v[i] = generated;
      }
    }
    lastReadable = s;
    haveLastReadable = true;
    checkedSteps++;
  };

  Replayer::Frame fr;
  while (replayer.next(fr)) {
  }

  long implemented = 0, failing = 0;
  std::printf("file     %s\nmode     %s%s\nsteps    %ld (%ld checked, computer running)\n", path.c_str(),
              freeRunning ? "free-running" : "resync",
              fuzzer ? (", fuzzed inputs (seed " + std::to_string(fuzzSeed) + ", " + std::to_string(fuzzer->episodes()) +
                        " episodes)").c_str()
                     : "",
              steps, checkedSteps);
  if (scenario) {
    std::printf("approach scenario seed %ld\n", approachSeed);
  }
  if (lateralScenario) {
    std::printf("lateral scenario seed %ld\n", lateralSeed);
  }
  std::printf("\n%-18s %6s %9s %10s %10s %10s %6s %10s %7s\n", "signal", "impl", "mismatch", "first", "t_first", "last",
              "gen@1st", "true", "rises");
  for (size_t i = 0; i < kSignalCount; i++) {
    const Stats& st = stats[i];
    if (!st.implemented) {
      continue;
    }
    implemented++;
    if (st.mismatches) {
      failing++;
    }
    std::printf("%-18s %6s %9ld %10ld %10.2f %10ld %6d %10ld %7ld\n", fmgc::readable::signalName(static_cast<Signal>(i)),
                "yes", st.mismatches, st.firstStep, st.firstTime, st.lastStep, st.firstGenerated, st.trueSteps, st.rises);
  }
  if (stats[static_cast<size_t>(Signal::ActiveLateralLaw)].implemented) {
    static const char* lawNames[kLawCount] = {"NONE", "HDG", "TRACK", "HPATH", "LOC_CPT", "LOC_TRACK", "ROLL_OUT"};
    for (size_t k = 0; k < kLawCount; k++) {
      std::printf("law %-12s %10ld steps %7ld rises\n", lawNames[k], lawSteps[k], lawRises[k]);
    }
  }
  std::printf("\n%ld signals rewritten, %ld with mismatches\n", implemented, failing);
  return failing == 0 ? 0 : 1;
}
