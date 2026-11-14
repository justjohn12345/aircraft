// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable AP / FD / A/THR engagement (readable/engagement.cpp) against the generated
// FmgcComputer.
//
//   resync (default): the readable state starts each step from the generated state (src/engagement_resync.h), the
//     previous frame's ap_fd_logic is the generated one, and stage 2 gets the generated stage-1 outputs, so a mismatch
//     points at the step and the stage where the logic differs;
//   --free: the readable engagement runs on its own state, its own previous fmgc_opp_mode_sync and its own stage-1
//     outputs (the other previous-frame modes and this frame's modes still come from the generated code).
//   --fuzz SEED, --approach SEED, --athr SEED, --lateral SEED, --engagement SEED: input perturbations
//     (src/input_fuzzer.h, scenario_approach.h, scenario_athr.h, scenario_lateral.h, scenario_engagement.h).
//
// Inputs are this frame's generated out.data and out.logic (input conditioning, not rewritten; the engagement reads
// conditioned values the generated code copies to the logic bus unchanged), the previous frame's ap_fd_logic
// (Delay_DSTATE) for stage 1 and this frame's out.ap_fd_logic modes for stage 2. Every field is compared exactly.

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

#include "engagement.h"
#include "engagement_resync.h"
#include "input_fuzzer.h"
#include "replay_core.h"
#include "scenario_approach.h"
#include "scenario_athr.h"
#include "scenario_engagement.h"
#include "scenario_lateral.h"

namespace E = fmgc::readable::engagement;
using DWork = FmgcComputer::D_Work_FmgcComputer_T;

namespace {

bool generatedValue(const fmgc_outputs& o, int f) {
  const auto& l = o.logic;
  const auto& a = o.ap_fd_logic;
  switch (f) {
    case E::ApFdAthrCommonCondition: return l.ap_fd_athr_common_condition;
    case E::ApFdCommonCondition: return l.ap_fd_common_condition;
    case E::FdOwnEngaged: return l.fd_own_engaged;
    case E::ApInop: return l.ap_inop;
    case E::ApOwnEngaged: return l.ap_own_engaged;
    case E::AthrInop: return l.athr_inop;
    case E::AthrOwnEngaged: return l.athr_own_engaged;
    case E::FmgcOppPriority: return l.fmgc_opp_priority;
    case E::FmgcOppModeSync: return a.fmgc_opp_mode_sync;
    case E::AnyApFdEngaged: return a.any_ap_fd_engaged;
    case E::LateralModeReset: return a.lateral_mode_reset;
    case E::LongitudinalModeReset: return a.longitudinal_mode_reset;
    case E::FdAutoDisengage: return a.fd_auto_disengage;
    case E::VsTargetNotHeld: return a.vs_target_not_held;
    case E::ApFdModeReversion: return a.ap_fd_mode_reversion;
    case E::PitchFdBarsFlashing: return a.pitch_fd_bars_flashing;
    case E::Land2Capability: return a.land_2_capability;
    case E::Land3FailPassiveCapability: return a.land_3_fail_passive_capability;
    case E::Land3FailOpCapability: return a.land_3_fail_op_capability;
    case E::Land2Inop: return a.land_2_inop;
    case E::Land3FailPassiveInop: return a.land_3_fail_passive_inop;
    case E::Land3FailOpInop: return a.land_3_fail_op_inop;
    case E::Land2Capacity: return a.land_2_capacity;
    case E::Land3FailPassiveCapacity: return a.land_3_fail_passive_capacity;
    case E::Land3FailOpCapacity: return a.land_3_fail_op_capacity;
    default: return false;
  }
}

struct Stats {
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  bool firstGenerated = false;
  long trueSteps = 0;  // coverage: steps where the generated value was true
  long rises = 0;      // coverage: false -> true transitions
  bool prev = false;
};

}  // namespace

int main(int argc, char** argv) {
  bool freeRunning = false;
  long fuzzSeed = -1, approachSeed = -1, athrSeed = -1, lateralSeed = -1, engagementSeed = -1;
  std::string path;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto seed = [&](long& v) {
      if (i + 1 < argc) {
        v = std::atol(argv[++i]);
        return true;
      }
      return false;
    };
    if (a == "--free") {
      freeRunning = true;
    } else if ((a == "--fuzz" && seed(fuzzSeed)) || (a == "--approach" && seed(approachSeed)) ||
               (a == "--athr" && seed(athrSeed)) || (a == "--lateral" && seed(lateralSeed)) ||
               (a == "--engagement" && seed(engagementSeed))) {
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr,
                 "usage: fmgc-check-engagement [--free] [--fuzz SEED] [--approach SEED] [--athr SEED] [--lateral SEED] "
                 "[--engagement SEED] <file.fdr>\n");
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
  std::unique_ptr<LateralScenario> lateral;
  if (lateralSeed >= 0) {
    lateral = std::make_unique<LateralScenario>(static_cast<uint64_t>(lateralSeed));
  }
  std::unique_ptr<AthrScenario> athrScenario;
  if (athrSeed >= 0) {
    athrScenario = std::make_unique<AthrScenario>(static_cast<uint64_t>(athrSeed));
  }
  std::unique_ptr<EngagementScenario> engagement;
  if (engagementSeed >= 0) {
    engagement = std::make_unique<EngagementScenario>(static_cast<uint64_t>(engagementSeed));
  }

  auto pre = std::make_unique<DWork>();
  E::Engagement eng;
  eng.reset();
  E::Outputs lastReadable{};
  bool haveLastReadable = false;
  Stats stats[E::kFieldCount];
  long conditionSteps[E::kConditionCount]{};
  long steps = 0, checkedSteps = 0;

  replayer.beforeStep = [&]() {
    double dt = replayer.inputs().time.dt;
    if (approach) {
      approach->apply(replayer.mutableInputs(), dt);
    }
    if (lateral) {
      lateral->apply(replayer.mutableInputs(), dt);
    }
    if (athrScenario) {
      athrScenario->apply(replayer.mutableInputs(), dt);
    }
    if (engagement) {
      engagement->apply(replayer.mutableInputs(), dt, computer->getExternalOutputs().out);
    }
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), dt);
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
    bool restarted = !pre->Runtime_MODE;

    base_fmgc_ap_fd_logic_outputs last =
        restarted ? FmgcComputer::FmgcComputer_P.Delay_InitialCondition : pre->Delay_DSTATE;
    if (restarted) {
      eng.reset();
    } else if (!freeRunning || !haveLastReadable) {
      resyncEngagement(eng.state(), *pre);
    }
    if (freeRunning && haveLastReadable && !restarted) {
      last.fmgc_opp_mode_sync = lastReadable[E::FmgcOppModeSync];
    }

    double t = out.data.time.simulation_time;
    E::Outputs r;
    auto check = [&](int f) {
      Stats& st = stats[f];
      bool g = generatedValue(out, f);
      st.trueSteps += g;
      st.rises += g && !st.prev;
      st.prev = g;
      if (g == r.v[f]) {
        return;
      }
      if (st.firstStep < 0) {
        st.firstStep = steps;
        st.firstTime = t;
        st.firstGenerated = g;
      }
      st.mismatches++;
    };

    eng.updateEngagement({out.data.time.dt, out.data, out.logic, last}, r);
    for (int f = 0; f < E::kStage1Count; f++) {
      check(f);
      if (!freeRunning) {
        r.v[f] = generatedValue(out, f);  // stages 2 and 3 see the generated stage-1 outputs
      }
    }
    eng.updateFdFlags({out.data.time.dt, out.data, out.logic, out.ap_fd_logic}, r);
    for (int f = E::kStage1Count; f < E::kStage2End; f++) {
      check(f);
    }
    eng.updateLandCapability({out.data.time.dt, out.data, out.logic, out.ap_fd_logic}, r);
    for (int f = E::kStage2End; f < E::kFieldCount; f++) {
      check(f);
    }
    for (int c = 0; c < E::kConditionCount; c++) {
      conditionSteps[c] += eng.cond[c];
    }
    lastReadable = r;
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
  if (lateral) {
    mode += ", lateral scenarios (seed " + std::to_string(lateralSeed) + ")";
  }
  if (athrScenario) {
    mode += ", autothrust scenarios (seed " + std::to_string(athrSeed) + ")";
  }
  if (engagement) {
    mode += ", engagement scenarios (seed " + std::to_string(engagementSeed) + ")";
  }
  std::printf("file     %s\nmode     %s\nsteps    %ld (%ld checked, computer running)\n", path.c_str(), mode.c_str(),
              steps, checkedSteps);
  std::printf("\n%-28s %6s %9s %10s %10s %9s %10s %7s\n", "field", "impl", "mismatch", "first", "t_first", "generated",
              "true", "rises");
  long failing = 0;
  for (int f = 0; f < E::kFieldCount; f++) {
    const Stats& st = stats[f];
    failing += st.mismatches != 0;
    std::printf("%-28s %6s %9ld %10ld %10.2f %9d %10ld %7ld\n", E::fieldName(f), "yes", st.mismatches, st.firstStep,
                st.firstTime, st.firstGenerated ? 1 : 0, st.trueSteps, st.rises);
  }
  std::printf("\nconditions:");
  for (int c = 0; c < E::kConditionCount; c++) {
    std::printf(" %s:%ld", E::conditionName(c), conditionSteps[c]);
  }
  std::printf("\n\n%d fields rewritten, %ld with mismatches\n", E::kFieldCount, failing);
  return failing == 0 ? 0 : 1;
}
