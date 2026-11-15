// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable input conditioning (readable/conditioning.cpp) against the generated
// FmgcComputer: every conditioned field of the logic bus, leaf by leaf (each ARINC word's SSM and data), compared
// exactly (floats bit for bit).
//
//   resync (default): the readable memories (the 5 s in-flight confirm node and the runway heading memory) start each
//     step from the generated ones, so a mismatch points at the step where the logic differs;
//   --free: the readable conditioning runs on its own memories from power-up (initialize()), including across
//     computer restarts (the runway heading memory survives a restart, the confirm node does not).
//   --fuzz SEED, --approach SEED, --engagement SEED, --lateral SEED, --failures SEED: input perturbations
//     (src/input_fuzzer.h, scenario_approach.h, scenario_engagement.h, scenario_lateral.h, scenario_conditioning.h).
//
// The only other input is the previous frame's ap_fd_logic (Delay_DSTATE, generated), for the ILS tune inhibit.
// The eight logic fields computed by the engagement stage are not compared (readable/engagement.cpp).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

// The generated model keeps its state and parameters private; the checker reads them (test-only access).
#define private public
#include "FmgcComputer.h"
#undef private

#include "conditioning.h"
#include "fields.gen.h"
#include "input_fuzzer.h"
#include "replay_core.h"
#include "scenario_approach.h"
#include "scenario_conditioning.h"
#include "scenario_engagement.h"
#include "scenario_lateral.h"

namespace C = fmgc::readable::conditioning;
using DWork = FmgcComputer::D_Work_FmgcComputer_T;

namespace {

// Logic fields owned by the engagement stage: not compared here.
const char* const kEngagementFields[] = {"ap_fd_athr_common_condition", "ap_fd_common_condition", "fd_own_engaged",
                                         "ap_own_engaged", "athr_own_engaged", "ap_inop", "athr_inop",
                                         "fmgc_opp_priority"};

/** One scalar leaf of base_fmgc_logic_outputs: name, byte offset and size, kind. */
struct Leaf {
  std::string name;
  size_t offset, size;
  FieldKind kind;
  // stats
  long mismatches = 0, firstStep = -1, changes = 0, trueSteps = 0;
  double firstTime = 0, firstGenerated = 0, firstReadable = 0;
  unsigned char prev[8]{};
  bool havePrev = false;
};

struct LeafCollector {
  std::vector<Leaf>& out;
  const base_fmgc_logic_outputs* base;
  template <typename T>
  void field(const char* prefix, const char* name, T& a, T&, FieldKind kind) {
    for (const char* e : kEngagementFields) {
      if (std::strcmp(prefix, "") == 0 && std::strcmp(name, e) == 0) {
        return;
      }
    }
    Leaf l;
    l.name = std::string(prefix) + name;
    l.offset = static_cast<size_t>(reinterpret_cast<const unsigned char*>(&a) -
                                   reinterpret_cast<const unsigned char*>(base));
    l.size = sizeof(T);
    l.kind = kind;
    out.push_back(l);
  }
};

double leafValue(const Leaf& l, const unsigned char* p) {
  if (l.kind == FieldKind::Bool) {
    return *reinterpret_cast<const bool*>(p) ? 1 : 0;
  }
  if (l.kind == FieldKind::Float) {
    if (l.size == sizeof(float)) {
      float f;
      std::memcpy(&f, p, sizeof f);
      return f;
    }
    double d;
    std::memcpy(&d, p, sizeof d);
    return d;
  }
  if (l.size == 1) {
    return *reinterpret_cast<const int8_t*>(p);
  }
  uint32_t v;
  std::memcpy(&v, p, sizeof v);
  return v;
}

}  // namespace

int main(int argc, char** argv) {
  bool freeRunning = false;
  long fuzzSeed = -1, approachSeed = -1, engagementSeed = -1, lateralSeed = -1, failuresSeed = -1;
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
               (a == "--engagement" && seed(engagementSeed)) || (a == "--lateral" && seed(lateralSeed)) ||
               (a == "--failures" && seed(failuresSeed))) {
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr,
                 "usage: fmgc-check-conditioning [--free] [--fuzz SEED] [--approach SEED] [--engagement SEED] "
                 "[--lateral SEED] [--failures SEED] <file.fdr>\n");
    return 2;
  }

  // The readable "no ADR / IR" bus is all zero; check the generated constant still is.
  {
    base_adr_bus zeroAdr{};
    base_ir_bus zeroIr{};
    const auto& k = FmgcComputer::FmgcComputer_P.Constant1_Value;
    if (std::memcmp(&k.adr_computation_data, &zeroAdr, sizeof zeroAdr) != 0 ||
        std::memcmp(&k.ir_computation_data, &zeroIr, sizeof zeroIr) != 0) {
      std::fprintf(stderr, "error: generated Constant1_Value ADR/IR bus is not all zero\n");
      return 2;
    }
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
  std::unique_ptr<EngagementScenario> engagement;
  if (engagementSeed >= 0) {
    engagement = std::make_unique<EngagementScenario>(static_cast<uint64_t>(engagementSeed));
  }
  std::unique_ptr<ConditioningScenario> failures;
  if (failuresSeed >= 0) {
    failures = std::make_unique<ConditioningScenario>(static_cast<uint64_t>(failuresSeed));
  }

  std::vector<Leaf> leaves;
  {
    auto a = std::make_unique<base_fmgc_logic_outputs>();
    auto b = std::make_unique<base_fmgc_logic_outputs>();
    LeafCollector c{leaves, a.get()};
    visit(c, "", *a, *b);
  }

  auto pre = std::make_unique<DWork>();
  double preRwyHdgMemo = 0;
  C::Conditioning cond;
  cond.initialize();
  long conditionSteps[C::kConditionCount]{};
  long steps = 0, checkedSteps = 0, restarts = 0;

  replayer.beforeStep = [&]() {
    double dt = replayer.inputs().time.dt;
    if (approach) {
      approach->apply(replayer.mutableInputs(), dt);
    }
    if (lateral) {
      lateral->apply(replayer.mutableInputs(), dt);
    }
    if (engagement) {
      engagement->apply(replayer.mutableInputs(), dt, computer->getExternalOutputs().out);
    }
    if (failures) {
      failures->apply(replayer.mutableInputs(), dt);
    }
    if (fuzzer) {
      fuzzer->apply(replayer.mutableInputs(), dt);
    }
    *pre = computer->FmgcComputer_DWork;
    preRwyHdgMemo = computer->FmgcComputer_B.u_lyjj;
  };
  replayer.afterStep = [&]() {
    steps++;
    if (!replayer.inputs().sim_data.computer_running) {
      return;  // the generated step does nothing; the readable memories are kept
    }
    const fmgc_outputs& out = computer->getExternalOutputs().out;
    bool restarted = !pre->Runtime_MODE;
    const base_fmgc_ap_fd_logic_outputs& last =
        restarted ? FmgcComputer::FmgcComputer_P.Delay_InitialCondition : pre->Delay_DSTATE;
    if (restarted) {
      cond.restart();
      restarts++;
    }
    if (!freeRunning) {
      C::State& s = cond.state();
      s.inFlight5s.output = pre->sf_MATLABFunction_fw.output;
      s.inFlight5s.timeSinceCondition = pre->sf_MATLABFunction_fw.timeSinceCondition;
      if (restarted) {
        s.inFlight5s.reset();
      }
      s.rwyHdgMemo = preRwyHdgMemo;
    }

    base_fmgc_logic_outputs r;
    cond.update({out.data, last}, r);
    const auto* gp = reinterpret_cast<const unsigned char*>(&out.logic);
    const auto* rp = reinterpret_cast<const unsigned char*>(&r);
    double t = out.data.time.simulation_time;
    for (Leaf& l : leaves) {
      const unsigned char* g = gp + l.offset;
      if (l.havePrev && std::memcmp(l.prev, g, l.size) != 0) {
        l.changes++;
      }
      std::memcpy(l.prev, g, l.size);
      l.havePrev = true;
      if (l.kind == FieldKind::Bool && *reinterpret_cast<const bool*>(g)) {
        l.trueSteps++;
      }
      if (std::memcmp(g, rp + l.offset, l.size) != 0) {
        if (l.firstStep < 0) {
          l.firstStep = steps;
          l.firstTime = t;
          l.firstGenerated = leafValue(l, g);
          l.firstReadable = leafValue(l, rp + l.offset);
        }
        l.mismatches++;
      }
    }
    for (int c = 0; c < C::kConditionCount; c++) {
      conditionSteps[c] += cond.cond[c];
    }
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
  if (engagement) {
    mode += ", engagement scenarios (seed " + std::to_string(engagementSeed) + ")";
  }
  if (failures) {
    mode += ", failure scenarios (seed " + std::to_string(failuresSeed) + ", " + std::to_string(failures->episodes) +
            " episodes)";
  }
  std::printf("file     %s\nmode     %s\nsteps    %ld (%ld checked, computer running, %ld restarts)\n", path.c_str(),
              mode.c_str(), steps, checkedSteps, restarts);
  std::printf("\n%-58s %6s %9s %10s %10s %14s %14s %9s %9s\n", "field", "impl", "mismatch", "first", "t_first",
              "generated", "readable", "changes", "true");
  long failing = 0;
  for (const Leaf& l : leaves) {
    failing += l.mismatches != 0;
    std::printf("%-58s %6s %9ld %10ld %10.2f %14.7g %14.7g %9ld %9ld\n", l.name.c_str(), "yes", l.mismatches,
                l.firstStep, l.firstTime, l.firstGenerated, l.firstReadable, l.changes,
                l.kind == FieldKind::Bool ? l.trueSteps : -1L);
  }
  std::printf("\nconditions:");
  for (int c = 0; c < C::kConditionCount; c++) {
    std::printf(" %s:%ld", C::conditionName(c), conditionSteps[c]);
  }
  if (failures) {
    std::printf("\nfaults:");
    for (int k = 0; k < ConditioningScenario::kKinds; k++) {
      std::printf(" %d:%ld", k, failures->faultCount[k]);
    }
  }
  std::printf("\n\n%zu fields rewritten, %ld with mismatches\n", leaves.size(), failing);
  return failing == 0 ? 0 : 1;
}
