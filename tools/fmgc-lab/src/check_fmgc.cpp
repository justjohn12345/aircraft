// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Whole-computer differential check of the readable FMGC (readable/fmgc.{h,cpp}) against the generated FmgcComputer
// (FmgcComputer + FmgcOuterLoops), over a flight data recording.
//
// Both computers run in lockstep on the same inputs, fully free-running: the readable computer only ever sees the
// inputs, never a generated value. Every frame (running or not) the whole fmgc_outputs is compared bit-exact (bools,
// enums and SSMs as integers, floats and doubles by bit pattern; leaf list from fields.gen.h). On the first mismatching
// frame the report lists the differing fields, the stage they belong to in pipeline order (logic bus = conditioning /
// engagement, ap_fd_logic = modes / targets / TCAS / laws / LAND capability, outer loops, A/THR, packing) and the
// first divergence of the hidden state (mode latches, speed memos, the engagement and A/THR latches, the block-output
// memories), which usually precedes the output mismatch.
//
// Input perturbations, combinable, applied in this order: --approach, --lateral, --athr, --targets, --engagement,
// --failures, --fuzz, --power SEED (src/scenario_*.h, input_fuzzer.h). --power: computer_running off for one frame to a
// minute, power flicker, the opposite FMGC lost (src/scenario_power.h).
// --warm FILE: run both computers over FILE first, then re-construct the generated one and initialize() both (checks
// that the readable initialize() is a complete reset, for fuzzing with reused instances).
// --time: also report the step() time of both computers (the default report includes it too; --time skips the
// comparison to measure the readable step alone, plus the generated one).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <type_traits>
#include <vector>

// The generated model keeps its state and parameters private; the checker reads them (test-only access).
#define private public
#include "FmgcComputer.h"
#undef private

#include "fields.gen.h"
#include "fmgc.h"
#include "input_fuzzer.h"
#include "replay_core.h"
#include "scenario_approach.h"
#include "scenario_athr.h"
#include "scenario_conditioning.h"
#include "scenario_engagement.h"
#include "scenario_lateral.h"
#include "scenario_power.h"
#include "scenario_targets.h"
#include "vertical_latches.h"

namespace R = fmgc::readable;
using DWork = FmgcComputer::D_Work_FmgcComputer_T;
using Clock = std::chrono::steady_clock;

namespace {

struct Leaf {
  std::string name;
  size_t offset;
  size_t size;
  FieldKind kind;
  long mismatches = 0;
  long firstStep = -1;
  // coverage (generated values, running frames)
  long nonzero = 0;
  long changes = 0;
  unsigned char prev[8]{};
  bool havePrev = false;
  long enumSteps[16]{};
};

/** Collects every leaf of a struct with its byte offset (visit() over two instances, the first one at `base`). */
struct LeafCollector {
  std::vector<Leaf>& leaves;
  const void* base;
  template <typename T>
  void field(const char* prefix, const char* name, T& a, T& /*b*/, FieldKind kind) {
    leaves.push_back({std::string(prefix) + name,
                      static_cast<size_t>(reinterpret_cast<const char*>(&a) - static_cast<const char*>(base)),
                      sizeof(T), kind});
  }
};

std::string showLeaf(const Leaf& l, const void* p) {
  const char* c = static_cast<const char*>(p) + l.offset;
  char buf[64];
  if (l.kind == FieldKind::Bool) {
    std::snprintf(buf, sizeof buf, "%d", *reinterpret_cast<const bool*>(c) ? 1 : 0);
  } else if (l.kind == FieldKind::Float && l.size == 4) {
    float v;
    std::memcpy(&v, c, 4);
    std::snprintf(buf, sizeof buf, "%.9g", v);
  } else if (l.kind == FieldKind::Float && l.size == 8) {
    double v;
    std::memcpy(&v, c, 8);
    std::snprintf(buf, sizeof buf, "%.17g", v);
  } else {
    int32_t v = 0;
    std::memcpy(&v, c, l.size < 4 ? l.size : 4);
    std::snprintf(buf, sizeof buf, "%d", v);
  }
  return buf;
}

/** The stage that produces an output field, in pipeline order (lower = earlier). */
struct Stage {
  int order;
  const char* name;
};
Stage stageOf(const std::string& n) {
  auto has = [&](const char* s) { return n.find(s) != std::string::npos; };
  auto starts = [&](const char* s) { return n.rfind(s, 0) == 0; };
  if (starts("data.")) return {0, "inputs copy (out.data)"};
  if (starts("logic.")) {
    if (has("ap_fd_athr_common") || has("ap_fd_common") || has("own_engaged") || has("_inop") || has("opp_priority")) {
      return {2, "engagement stage 1 (logic)"};
    }
    return {1, "input conditioning (logic)"};
  }
  if (starts("ap_fd_logic.")) {
    if (has("fmgc_opp_mode_sync") || has("any_ap_fd_engaged") || has("mode_reset")) return {2, "engagement stage 1"};
    if (has("tcas_vs_target") || has("tcas_ra_corrective") || has("tcas_ra_inhibited") || has("trk_fpa_deselected") ||
        has("longi_large_box")) {
      return {3, "TCAS decoding"};
    }
    if (has("fd_auto_disengage") || has("vs_target_not_held") || has("ap_fd_mode_reversion") ||
        has("pitch_fd_bars")) {
      return {5, "engagement stage 2"};
    }
    if (has("hdg_trk_preset") || has("spd_control") || has("mach_control") || has("spd_target") ||
        has("alt_soft")) {
      return {6, "targets"};
    }
    if (has("active_longitudinal_law")) return {7, "longitudinal law selection"};
    if (has("land_2_") || has("land_3_")) return {8, "LAND capability"};
    return {4, "mode logic (vertical/lateral signals)"};
  }
  if (starts("ap_fd_outer_loops.")) return {9, "outer loops (input assembly or laws)"};
  if (starts("athr.")) return {10, "autothrust"};
  return {11, "output packing"};
}

/** One hidden memory compared each frame: generated value vs readable value (as double, compared exactly). */
struct Hidden {
  const char* name;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0, firstReadable = 0;
};

}  // namespace

int main(int argc, char** argv) {
  long fuzzSeed = -1, approachSeed = -1, athrSeed = -1, lateralSeed = -1, engagementSeed = -1, targetsSeed = -1,
       failuresSeed = -1, powerSeed = -1;
  bool timeOnly = false;
  int maxListed = 40;
  std::string path, warmPath;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto seed = [&](long& v) {
      if (i + 1 < argc) {
        v = std::atol(argv[++i]);
        return true;
      }
      return false;
    };
    if ((a == "--fuzz" && seed(fuzzSeed)) || (a == "--approach" && seed(approachSeed)) ||
        (a == "--athr" && seed(athrSeed)) || (a == "--lateral" && seed(lateralSeed)) ||
        (a == "--engagement" && seed(engagementSeed)) || (a == "--targets" && seed(targetsSeed)) ||
        (a == "--failures" && seed(failuresSeed)) || (a == "--power" && seed(powerSeed))) {
    } else if (a == "--time") {
      timeOnly = true;
    } else if (a == "--warm" && i + 1 < argc) {
      warmPath = argv[++i];
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      path.clear();
      break;
    }
  }
  if (path.empty()) {
    std::fprintf(stderr,
                 "usage: fmgc-check-fmgc [--fuzz SEED] [--approach SEED] [--athr SEED] [--lateral SEED] "
                 "[--engagement SEED] [--targets SEED] [--failures SEED] [--power SEED] [--time] [--list N] "
                 "[--warm FILE] <file.fdr>\n");
    return 2;
  }

  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  auto readable = std::make_unique<R::FmgcComputer>();
  readable->initialize();
  if (!warmPath.empty()) {
    // Reuse check: run both computers over another recording first (no comparison), then reset both. The main run must
    // then match as from fresh instances: the readable initialize() is a complete reset (for the fuzzer, which reuses
    // instances between inputs). The generated model is re-constructed in place: its initialize() alone is not a
    // complete reset of a used instance (Runtime_MODE stays true, so the restart block does not run on the next frame).
    Replayer warm(warmPath, {}, *computer);
    if (!warm.ok()) {
      std::fprintf(stderr, "error: %s\n", warm.error().c_str());
      return 2;
    }
    warm.afterStep = [&]() { readable->step(warm.inputs()); };
    Replayer::Frame wf;
    while (warm.next(wf)) {
    }
    computer->~FmgcComputer();
    new (computer.get()) FmgcComputer();
    computer->initialize();
    readable->initialize();
    std::printf("warm     %s (%ld frames), then initialize()\n", warmPath.c_str(), warm.frames());
  }
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    return 2;
  }

  std::unique_ptr<ApproachScenario> approach;
  if (approachSeed >= 0) approach = std::make_unique<ApproachScenario>(static_cast<uint64_t>(approachSeed));
  std::unique_ptr<LateralScenario> lateral;
  if (lateralSeed >= 0) lateral = std::make_unique<LateralScenario>(static_cast<uint64_t>(lateralSeed));
  std::unique_ptr<AthrScenario> athrScenario;
  if (athrSeed >= 0) athrScenario = std::make_unique<AthrScenario>(static_cast<uint64_t>(athrSeed));
  std::unique_ptr<TargetsScenario> targets;
  if (targetsSeed >= 0) targets = std::make_unique<TargetsScenario>(static_cast<uint64_t>(targetsSeed));
  std::unique_ptr<EngagementScenario> engagement;
  if (engagementSeed >= 0) engagement = std::make_unique<EngagementScenario>(static_cast<uint64_t>(engagementSeed));
  std::unique_ptr<ConditioningScenario> failures;
  if (failuresSeed >= 0) failures = std::make_unique<ConditioningScenario>(static_cast<uint64_t>(failuresSeed));
  std::unique_ptr<InputFuzzer> fuzzer;
  if (fuzzSeed >= 0) fuzzer = std::make_unique<InputFuzzer>(static_cast<uint64_t>(fuzzSeed));
  std::unique_ptr<PowerScenario> power;
  if (powerSeed >= 0) power = std::make_unique<PowerScenario>(static_cast<uint64_t>(powerSeed));

  // ---- leaves of fmgc_outputs
  std::vector<Leaf> leaves;
  {
    auto a = std::make_unique<fmgc_outputs>();
    auto b = std::make_unique<fmgc_outputs>();
    LeafCollector c{leaves, a.get()};
    visit(c, "", *a, *b);
  }

  // ---- sanity: the readable computer's power-up values are the generated ones (outputs, restart delays)
  long initMismatches = 0;
  {
    const fmgc_outputs& g = computer->FmgcComputer_Y.out;
    const fmgc_outputs& r = readable->outputs();
    for (const Leaf& l : leaves) {
      if (std::memcmp(reinterpret_cast<const char*>(&g) + l.offset, reinterpret_cast<const char*>(&r) + l.offset,
                      l.size) != 0) {
        std::printf("init mismatch: %s generated %s readable %s\n", l.name.c_str(), showLeaf(l, &g).c_str(),
                    showLeaf(l, &r).c_str());
        initMismatches++;
      }
    }
    // The readable computer restarts with zeroed previous-frame buses; check the generated initial conditions are zero.
    auto allZeroLeaves = [&](auto& value, const char* what) {
      using T = std::remove_reference_t<decltype(value)>;
      std::vector<Leaf> ls;
      auto copy = std::make_unique<T>(value);
      auto zero = std::make_unique<T>();
      *zero = T{};
      LeafCollector c{ls, copy.get()};
      visit(c, "", *copy, *zero);
      for (const Leaf& l : ls) {
        if (std::memcmp(reinterpret_cast<const char*>(copy.get()) + l.offset,
                        reinterpret_cast<const char*>(zero.get()) + l.offset, l.size) != 0) {
          std::printf("init mismatch: %s.%s is not zero\n", what, l.name.c_str());
          initMismatches++;
        }
      }
    };
    allZeroLeaves(FmgcComputer::FmgcComputer_P.Delay_InitialCondition, "Delay_InitialCondition");
    allZeroLeaves(FmgcComputer::FmgcComputer_P.Delay1_InitialCondition, "Delay1_InitialCondition");
    allZeroLeaves(FmgcComputer::FmgcComputer_P.Delay2_InitialCondition, "Delay2_InitialCondition");
  }

  // ---- hidden state: generated memory vs readable memory
  std::vector<Hidden> hidden;
  std::vector<std::pair<double, double>> hiddenValues;  // this frame's (generated, readable)
  auto hiddenSet = [&](size_t& k, const char* name, double g, double r) {
    if (k == hidden.size()) {
      hidden.push_back({name});
    }
    if (hiddenValues.size() <= k) hiddenValues.resize(k + 1);
    hiddenValues[k++] = {g, r};
  };
  auto collectHidden = [&]() {
    const DWork& d = computer->FmgcComputer_DWork;
    const auto& b = computer->FmgcComputer_B;
    size_t k = 0;
    const R::Signals& s = readable->modeSignals();
    for (size_t i = 0; i < R::kSignalCount; i++) {
      if (const boolean_T* l = latchOf(d, static_cast<Signal>(i))) {
        hiddenSet(k, R::signalName(static_cast<Signal>(i)), *l, s.v[i]);
      }
    }
    const auto& es = readable->engagementStage().state();
    hiddenSet(k, "engagement.apEngageRequest", d.Memory_PreviousInput, es.apEngageRequest.q);
    hiddenSet(k, "engagement.apEngaged", d.Delay_DSTATE_p, es.apEngaged.q);
    hiddenSet(k, "engagement.athrInhibited", d.Memory_PreviousInput_g1, es.athrInhibited.q);
    hiddenSet(k, "engagement.athrEngaged", d.Delay_DSTATE_k, es.athrEngaged.q);
    hiddenSet(k, "engagement.land3FailOpHeld", d.pLand3FailOp, es.land3FailOpHeld);
    hiddenSet(k, "engagement.land3FailPassiveHeld", d.pLand3FailPass, es.land3FailPassiveHeld);
    const R::targets::Values& tv = readable->targetValues();
    hiddenSet(k, "targets.manualSpeed", d.Memory_PreviousInput_cu, tv.manualSpeed);
    hiddenSet(k, "targets.autoSpeed", d.Memory_PreviousInput_hk, tv.autoSpeed);
    hiddenSet(k, "targets.machMode", d.Delay_DSTATE_c, tv.machMode);
    hiddenSet(k, "targets.lowerMarginActive", d.Memory_PreviousInput_dh, tv.lowerMarginActive);
    hiddenSet(k, "targets.vappMemoKts", b.u_l, tv.vappMemoKts);
    const auto& am = readable->autothrustStage().modesState();
    hiddenSet(k, "athr.tcasLatch", d.Memory_PreviousInput_jb, am.tcasLatch.q);
    hiddenSet(k, "athr.alphaFloor", d.Memory_PreviousInput_hu, am.alphaFloor.q);
    hiddenSet(k, "athr.thrustMode", d.Memory_PreviousInput_bh, am.thrustMode.q);
    hiddenSet(k, "athr.speedMode", d.Memory_PreviousInput_cm, am.speedMode.q);
    hiddenSet(k, "athr.retardMode", d.Memory_PreviousInput_ol, am.retardMode.q);
    hiddenSet(k, "athr.lvrClbArmed", d.Memory_PreviousInput_kr, am.lvrClbArmed.q);
    hiddenSet(k, "athr.lvrMctArmed", d.Memory_PreviousInput_km, am.lvrMctArmed.q);
    hiddenSet(k, "athr.n1CommandPercent", d.Delay_DSTATE_l, readable->autothrustStage().n1State().n1CommandPercent);
    const auto& ts = readable->tcasStage().state();
    hiddenSet(k, "tcas.hdgVsDuringTcas", d.Memory_PreviousInput_e, ts.hdgVsDuringTcas.q);
    hiddenSet(k, "tcas.tcasEndedAfterHdgVs", d.Delay_DSTATE_o, ts.tcasEndedAfterHdgVs);
    const auto& cs = readable->conditioningStage().state();
    hiddenSet(k, "conditioning.rwyHdgMemo", b.u_lyjj, cs.rwyHdgMemo);
    hiddenSet(k, "conditioning.inFlight5s", d.sf_MATLABFunction_fw.output, cs.inFlight5s.output);
    const R::OuterLoopInputState& os = readable->outerLoopInputState();
    hiddenSet(k, "outerLoopInputs.locDistanceMemoNmi", b.u, os.locDistanceMemoNmi);
    hiddenSet(k, "outerLoopInputs.trackMemoDeg", b.u_lyjjl, os.trackMemoDeg);
    hiddenSet(k, "running(Runtime_MODE)", d.Runtime_MODE, readable->running());
  };

  long steps = 0, runningSteps = 0, mismatchFrames = 0, restarts = 0;
  long firstMismatchStep = -1;
  long firstHiddenStep = -1;
  double genSeconds = 0, readSeconds = 0;
  Clock::time_point genStart;
  bool prevRunning = false;
  auto in = std::make_unique<fmgc_inputs>();

  replayer.beforeStep = [&]() {
    fmgc_inputs& mi = replayer.mutableInputs();
    const double dt = mi.time.dt;
    if (approach) approach->apply(mi, dt);
    if (lateral) lateral->apply(mi, dt);
    if (athrScenario) athrScenario->apply(mi, dt);
    if (targets) targets->apply(mi, dt);
    if (engagement) engagement->apply(mi, dt, computer->getExternalOutputs().out);
    if (failures) failures->apply(mi, dt);
    if (fuzzer) fuzzer->apply(mi, dt);
    if (power) power->apply(mi, dt);
    genStart = Clock::now();
  };
  replayer.afterStep = [&]() {
    genSeconds += std::chrono::duration<double>(Clock::now() - genStart).count();
    steps++;
    *in = replayer.inputs();
    bool runningNow = in->sim_data.computer_running;
    runningSteps += runningNow;
    restarts += runningNow && !prevRunning;
    prevRunning = runningNow;

    auto t0 = Clock::now();
    const fmgc_outputs& r = readable->step(*in);
    readSeconds += std::chrono::duration<double>(Clock::now() - t0).count();
    if (timeOnly) {
      return;
    }
    const fmgc_outputs& g = computer->FmgcComputer_Y.out;
    const char* gp = reinterpret_cast<const char*>(&g);
    const char* rp = reinterpret_cast<const char*>(&r);
    bool any = false;
    if (runningNow) {
      static const unsigned char zero[8]{};
      for (Leaf& l : leaves) {
        const char* v = gp + l.offset;
        l.nonzero += std::memcmp(v, zero, l.size) != 0;
        l.changes += l.havePrev && std::memcmp(v, l.prev, l.size) != 0;
        std::memcpy(l.prev, v, l.size);
        l.havePrev = true;
        if (l.kind == FieldKind::Enum) {
          int32_t e = 0;
          std::memcpy(&e, v, l.size < 4 ? l.size : 4);
          l.enumSteps[e >= 0 && e < 15 ? e : 15]++;
        }
      }
    }
    for (Leaf& l : leaves) {
      if (std::memcmp(gp + l.offset, rp + l.offset, l.size) != 0) {
        any = true;
        if (l.firstStep < 0) l.firstStep = steps;
        l.mismatches++;
      }
    }

    collectHidden();
    for (size_t k = 0; k < hidden.size(); k++) {
      auto [hg, hr] = hiddenValues[k];
      if (std::memcmp(&hg, &hr, sizeof(double)) != 0) {
        Hidden& h = hidden[k];
        if (h.firstStep < 0) {
          h.firstStep = steps;
          h.firstTime = in->time.simulation_time;
          h.firstGenerated = hg;
          h.firstReadable = hr;
        }
        h.mismatches++;
        if (firstHiddenStep < 0) firstHiddenStep = steps;
      }
    }

    if (!any) {
      return;
    }
    mismatchFrames++;
    if (firstMismatchStep >= 0) {
      return;
    }
    firstMismatchStep = steps;
    // ---- first mismatch: fields, stage attribution, hidden state
    std::printf("FIRST MISMATCH at step %ld, t = %.3f s, computer %s\n", steps, in->time.simulation_time,
                runningNow ? "running" : "not running");
    Stage first{99, "?"};
    int listed = 0;
    for (const Leaf& l : leaves) {
      if (l.firstStep != steps) continue;
      Stage st = stageOf(l.name);
      if (st.order < first.order) first = st;
      if (listed++ < maxListed) {
        std::printf("  %-64s generated %-22s readable %-22s [%s]\n", l.name.c_str(), showLeaf(l, gp).c_str(),
                    showLeaf(l, rp).c_str(), st.name);
      }
    }
    if (listed > maxListed) std::printf("  ... %d fields in all\n", listed);
    std::printf("  earliest stage with a mismatch: %s\n", first.name);
    if (firstHiddenStep >= 0) {
      std::printf("  hidden state diverged first at step %ld:", firstHiddenStep);
      for (const Hidden& h : hidden) {
        if (h.firstStep == firstHiddenStep) {
          std::printf(" %s (generated %.17g readable %.17g)", h.name, h.firstGenerated, h.firstReadable);
        }
      }
      std::printf("\n");
    } else {
      std::printf("  hidden state: no divergence before this frame\n");
    }
  };

  Replayer::Frame fr;
  while (replayer.next(fr)) {
  }

  std::string mode = "free-running lockstep";
  if (approach) mode += ", approach (seed " + std::to_string(approachSeed) + ")";
  if (lateral) mode += ", lateral (seed " + std::to_string(lateralSeed) + ")";
  if (athrScenario) mode += ", autothrust (seed " + std::to_string(athrSeed) + ")";
  if (targets) mode += ", targets (seed " + std::to_string(targetsSeed) + ")";
  if (engagement) mode += ", engagement (seed " + std::to_string(engagementSeed) + ")";
  if (failures) mode += ", failures (seed " + std::to_string(failuresSeed) + ")";
  if (fuzzer) {
    mode += ", fuzzed (seed " + std::to_string(fuzzSeed) + ", " + std::to_string(fuzzer->episodes()) + " episodes)";
  }
  if (power) {
    mode += ", power (seed " + std::to_string(powerSeed) + ", " + std::to_string(power->episodes) + " episodes, " +
            std::to_string(power->offFrames) + " frames off)";
  }
  std::printf("file     %s\nmode     %s\nsteps    %ld (%ld running, %ld (re)starts)\n", path.c_str(), mode.c_str(),
              steps, runningSteps, restarts);
  std::printf("time     generated %.3f us/step, readable %.3f us/step (ratio %.2f)\n", genSeconds / steps * 1e6,
              readSeconds / steps * 1e6, readSeconds / genSeconds);
  if (timeOnly) {
    return 0;
  }

  long failing = 0;
  std::printf("\n%-64s %9s %10s\n", "field", "mismatch", "first");
  for (const Leaf& l : leaves) {
    if (l.mismatches == 0) continue;
    failing++;
    std::printf("%-64s %9ld %10ld\n", l.name.c_str(), l.mismatches, l.firstStep);
  }
  // coverage of the computed buses (generated values, running frames): "cov NAME nonzero changes", and per enum value
  // "enum NAME VALUE steps" (15 = out of range)
  std::printf("\n");
  for (const Leaf& l : leaves) {
    if (l.name.rfind("data.", 0) == 0) continue;
    std::printf("cov %-64s %10ld %8ld\n", l.name.c_str(), l.nonzero, l.changes);
    if (l.kind == FieldKind::Enum) {
      for (int e = 0; e < 16; e++) {
        if (l.enumSteps[e]) std::printf("enum %-63s %2d %10ld\n", l.name.c_str(), e, l.enumSteps[e]);
      }
    }
  }
  long hiddenFailing = 0;
  for (const Hidden& h : hidden) {
    if (h.mismatches == 0) continue;
    hiddenFailing++;
    std::printf("hidden.%-57s %9ld %10ld  t %.2f generated %.17g readable %.17g\n", h.name, h.mismatches, h.firstStep,
                h.firstTime, h.firstGenerated, h.firstReadable);
  }
  std::printf("\n%zu fields compared, %ld with mismatches, %ld mismatching steps; %zu hidden memories, %ld diverged; "
              "%ld init mismatches\n",
              leaves.size(), failing, mismatchFrames, hidden.size(), hiddenFailing, initMismatches);
  return failing == 0 && hiddenFailing == 0 && initMismatches == 0 ? 0 : 1;
}
