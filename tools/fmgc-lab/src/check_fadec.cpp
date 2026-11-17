// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable FADEC (readable/fadec/) against the generated FadecComputer over a flight data
// recording. Two pairs (engine 1 and 2) run in lockstep on the same inputs, free-running (the readable FADECs never see
// a generated value); every step the whole athr_out (time, data, data_computed, input, fcu_input, output, ECU bus) is
// compared leaf by leaf, bit-exact, and so are the hidden memories (latches, timers, edge detectors, frozen N1,
// integrators).
//
// Inputs (src/fadec_drive.h): the recorded FADEC inputs of a 3209004 recording, or the synthetic drive built from any
// recording (--synthetic forces it on 3209004; it prints how closely generated FADEC 1 follows the recorded FADEC 1
// outputs, as `fidelity` lines). Injectors (src/fadec_scenarios.h), combinable: --script, --lever, --athr, --sim SEED.
// --warm FILE: run both over FILE first, then re-construct the generated FADECs and initialize() both (checks that the
// readable initialize() is a complete reset).
//
// Also printed: the three SR flip-flop tables against the readable formula, event counts of generated FADEC 1 (A/THR
// control lost by cause, thrust lock, A/THR disabled), and the recorded FADEC 1 thrust lock episodes with the
// instinctive disconnect input around them.
//
// Exit status 0 when every leaf and hidden memory matched on every step, 1 on any mismatch, 2 on usage/file errors.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <vector>

// The generated model keeps its state private; the checker reads it (test-only access).
#define private public
#include "FadecComputer.h"
#undef private

#include "fadec_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"

#include "fadec.h"
#include "fadec_drive.h"
#include "fadec_scenarios.h"

namespace R = fadec::readable;
namespace S = fadec_scenarios;
using Clock = std::chrono::steady_clock;

namespace {

struct Hidden {
  std::string name;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0, firstReadable = 0;
};

/** Pairs (generated, readable) of every hidden memory, as doubles (exact for bools). */
void collectHidden(const FadecComputer& g, const R::FadecComputer& r, const char* prefix, std::vector<Hidden>& hidden,
                   std::vector<std::pair<double, double>>& values, size_t& k) {
  const auto& d = g.FadecComputer_DWork;
  auto add = [&](const char* name, double gv, double rv) {
    if (k == hidden.size()) hidden.push_back({std::string(prefix) + name});
    if (values.size() <= k) values.resize(k + 1);
    values[k++] = {gv, rv};
  };
  const R::AthrInterface& a = r.athr();
  add("athrDisabled(Memory_PreviousInput)", d.Memory_PreviousInput, a.disabled());
  add("athrControlActive(Memory_PreviousInput_p)", d.Memory_PreviousInput_p, a.controlActive());
  add("thrustLock(Memory_PreviousInput_j)", d.Memory_PreviousInput_j, a.thrustLock());
  add("flexLatch(latch)", d.latch, r.flex().latch());
  add("frozenN1(pU)", d.pU, r.n1().frozenN1());
  add("frozenN1Primed(pU_not_empty)", d.pU_not_empty, r.n1().frozenN1Primed());
  add("trim(Delay_DSTATE)", d.Delay_DSTATE, r.n1().trim());
  add("forward(Delay_DSTATE_n)", d.Delay_DSTATE_n, r.n1().forward());
  add("reverse(Delay_DSTATE_l)", d.Delay_DSTATE_l, r.n1().reverse());
  add("disconnectTimer.eventTime", d.sf_TimeSinceCondition.eventTime, a.disconnectTimer().eventTime);
  add("disconnectTimer.primed", d.sf_TimeSinceCondition.eventTime_not_empty, a.disconnectTimer().primed);
  add("onGroundTimer.eventTime", d.sf_TimeSinceCondition1.eventTime, r.onGroundTimer().eventTime);
  add("onGroundTimer.primed", d.sf_TimeSinceCondition1.eventTime_not_empty, r.onGroundTimer().primed);
  add("engageEdge.previous(sf_MATLABFunction_f)", d.sf_MATLABFunction_f.previousInput, a.engageEdge().previousInput);
  add("engageEdge.primed", d.sf_MATLABFunction_f.previousInput_not_empty, a.engageEdge().primed);
  add("lostEdge.previous(sf_MATLABFunction_b)", d.sf_MATLABFunction_b.previousInput, a.lostEdge().previousInput);
  add("lostEdge.primed", d.sf_MATLABFunction_b.previousInput_not_empty, a.lostEdge().primed);
}

/** The SR flip-flop tables against the readable formula (both outputs). Returns the number of differing entries. */
int checkTables() {
  const auto& p = FadecComputer::FadecComputer_P;
  const boolean_T* tables[3] = {p.Logic_table, p.Logic_table_n, p.Logic_table_h};
  int bad = 0;
  for (const boolean_T* t : tables) {
    for (int i = 0; i < 8; i++) {
      R::blocks::SrFlipFlop f;
      f.q = i & 1;
      const bool s = i & 4, r = i & 2;
      f.update(s, r);
      bad += R::blocks::SrFlipFlop::fromTable(t, s, r, i & 1, false) != f.q;
      bad += R::blocks::SrFlipFlop::fromTable(t, s, r, i & 1, true) != f.qbar;
    }
  }
  return bad;
}

/** Event counts of one generated FADEC (from its outputs and inputs, step by step). */
struct Events {
  long controlLost = 0, lostByDisconnect = 0, lostByDisabled = 0, lostByBit13 = 0;
  long lockSet = 0, lockAfterDisconnect = 0, disabledSet = 0, flexLatchInFlight = 0;
  bool prevControl = false, prevLock = false, prevDisabled = false;
  double lastDisconnect = -1e9;

  void update(const athr_in& in, const FadecComputer& g) {
    const athr_out& o = g.FadecComputer_Y.out;
    const double t = in.time.simulation_time;
    if (in.input.ATHR_disconnect) lastDisconnect = t;
    if (prevControl && !o.output.athr_control_active) {
      controlLost++;
      if (in.input.ATHR_disconnect) lostByDisconnect++;
      else if (o.data_computed.ATHR_disabled) lostByDisabled++;
      else lostByBit13++;
    }
    if (!prevLock && o.output.memo_thrust_active) {
      lockSet++;
      lockAfterDisconnect += t - lastDisconnect < 0.5;
    }
    disabledSet += !prevDisabled && o.data_computed.ATHR_disabled;
    flexLatchInFlight += !in.data.on_ground && o.data_computed.is_FLX_active;
    prevControl = o.output.athr_control_active;
    prevLock = o.output.memo_thrust_active;
    prevDisabled = o.data_computed.ATHR_disabled;
  }
};

/** Thrust lock episodes in the recorded FADEC 1 outputs, and how many began within 0.5 s of an instinctive disconnect. */
struct RecordedLocks {
  long episodes = 0, afterDisconnect = 0, frames = 0;
  bool prev = false;
  double lastDisconnect = -1e9;
  void update(const FdrFrame& f) {
    const double t = f.base.simulation_time_s;
    if (f.fmgc1.discrete_inputs.athr_instinctive_disc) lastDisconnect = t;
    const bool lock = f.fadec[0].outputs.memo_thrust_active;
    frames += lock;
    if (lock && !prev) {
      episodes++;
      afterDisconnect += t - lastDisconnect < 0.5;
    }
    prev = lock;
  }
};

}  // namespace

int main(int argc, char** argv) {
  long scriptSeed = -1, leverSeed = -1, athrSeed = -1, simSeed = -1;
  FadecInputDrive::Options opt;
  int maxListed = 40;
  std::string path, warmPath;
  bool usage = false;
  for (int i = 1; i < argc && !usage; i++) {
    std::string a = argv[i];
    auto seed = [&](long& v) {
      if (i + 1 < argc) {
        v = std::atol(argv[++i]);
        return true;
      }
      return false;
    };
    if ((a == "--script" && seed(scriptSeed)) || (a == "--lever" && seed(leverSeed)) ||
        (a == "--athr" && seed(athrSeed)) || (a == "--sim" && seed(simSeed))) {
    } else if (a == "--synthetic") {
      opt.synthetic = true;
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else if (a == "--warm" && i + 1 < argc) {
      warmPath = argv[++i];
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      usage = true;
    }
  }
  if (path.empty() || usage) {
    std::fprintf(stderr,
                 "usage: fadec-check [--synthetic] [--script SEED] [--lever SEED] [--athr SEED] [--sim SEED] "
                 "[--warm FILE] [--list N] <file.fdr>\n");
    return 2;
  }

  std::unique_ptr<FadecComputer> generated[2] = {std::make_unique<FadecComputer>(), std::make_unique<FadecComputer>()};
  R::FadecComputer readable[2];
  for (int e = 0; e < 2; e++) {
    generated[e]->initialize();
    readable[e].initialize();
  }
  auto ext = std::make_unique<FadecComputer::ExternalInputs_FadecComputer_T>();

  if (!warmPath.empty()) {
    FadecInputDrive warm(warmPath, opt);
    if (!warm.ok()) {
      std::fprintf(stderr, "error: %s\n", warm.error().c_str());
      return 2;
    }
    for (int e = 0; e < 2; e++) warm.feedback[e] = &generated[e]->FadecComputer_Y.out;
    S::ScriptScenario script(99);
    while (warm.next([&](athr_in (&in)[2], const FadecInputDrive::StepInfo&) {
      script.apply(in, in[0].time.dt);
      for (int e = 0; e < 2; e++) {
        ext->in = in[e];
        generated[e]->setExternalInputs(ext.get());
        generated[e]->step();
        readable[e].step(in[e]);
      }
    })) {
    }
    // the generated initialize() is not a complete reset of a used instance: re-construct it in place
    for (int e = 0; e < 2; e++) {
      generated[e]->~FadecComputer();
      new (generated[e].get()) FadecComputer();
      generated[e]->initialize();
      readable[e].initialize();
    }
    std::printf("warm     %s (%ld frames, script 99), then initialize()\n", warmPath.c_str(), warm.frames());
  }

  FadecInputDrive drive(path, opt);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }
  for (int e = 0; e < 2; e++) drive.feedback[e] = &generated[e]->FadecComputer_Y.out;

  std::unique_ptr<S::ScriptScenario> script;
  if (scriptSeed >= 0) script = std::make_unique<S::ScriptScenario>(scriptSeed);
  std::unique_ptr<S::LeverScenario> lever;
  if (leverSeed >= 0) lever = std::make_unique<S::LeverScenario>(leverSeed);
  std::unique_ptr<S::AthrScenario> athr;
  if (athrSeed >= 0) athr = std::make_unique<S::AthrScenario>(athrSeed);
  std::unique_ptr<S::SimScenario> sim;
  if (simSeed >= 0) sim = std::make_unique<S::SimScenario>(simSeed);

  std::vector<FcuLeaf> leaves[2] = {fcuLeaves<athr_out>("fadec1."), fcuLeaves<athr_out>("fadec2.")};
  std::vector<FcuLeaf> fidelityOut = fcuLeaves<athr_output>("output.");
  std::vector<FcuLeaf> fidelityBus = fcuLeaves<base_ecu_bus>("fadec_bus_output.");

  // power-up outputs and tables
  long initMismatches = checkTables();
  if (initMismatches) std::printf("init mismatch: SR flip-flop tables differ from the readable formula\n");
  for (int e = 0; e < 2; e++) {
    std::vector<FcuLeaf> l0 = fcuLeaves<athr_out>();
    initMismatches += compareFcuLeaves(l0, generated[e]->FadecComputer_Y.out, readable[e].outputs(), 0, 0, false);
    for (const FcuLeaf& l : l0) {
      if (l.mismatches) std::printf("init mismatch: fadec%d.%s\n", e + 1, l.name.c_str());
    }
  }

  std::vector<Hidden> hidden;
  std::vector<std::pair<double, double>> hiddenValues;
  long steps = 0, mismatchSteps = 0, firstMismatch = -1, firstHidden = -1, fidelitySteps = 0;
  double genSeconds = 0, readSeconds = 0;
  Events events[2];
  RecordedLocks recordedLocks;
  long lastRecordedFrame = -1;

  auto step = [&](athr_in (&in)[2], const FadecInputDrive::StepInfo& info) {
    const double dt = in[0].time.dt;
    if (script) script->apply(in, dt);
    if (lever) lever->apply(in, dt);
    if (athr) athr->apply(in, dt);
    if (sim) sim->apply(in, dt);
    steps++;

    int n = 0;
    for (int e = 0; e < 2; e++) {
      ext->in = in[e];
      auto t0 = Clock::now();
      generated[e]->setExternalInputs(ext.get());
      generated[e]->step();
      auto t1 = Clock::now();
      const athr_out& r = readable[e].step(in[e]);
      auto t2 = Clock::now();
      genSeconds += std::chrono::duration<double>(t1 - t0).count();
      readSeconds += std::chrono::duration<double>(t2 - t1).count();
      n += compareFcuLeaves(leaves[e], generated[e]->FadecComputer_Y.out, r, steps, in[e].time.simulation_time, true);
      events[e].update(in[e], *generated[e]);
    }

    if (!info.gapFill && info.frame != lastRecordedFrame) {
      lastRecordedFrame = info.frame;
      recordedLocks.update(*info.fdr);
      // how closely generated FADEC 1 follows the recorded FADEC 1 (synthetic drive, no injector)
      if (!info.recordedInputs && !script && !lever && !athr && !sim) {
        fidelitySteps++;
        const athr_out& g = generated[0]->FadecComputer_Y.out;
        compareFcuLeaves(fidelityOut, info.fdr->fadec[0].outputs, g.output, steps, 0, false);
        compareFcuLeaves(fidelityBus, info.fdr->fadec[0].bus_outputs, g.fadec_bus_output, steps, 0, false);
      }
    }

    size_t k = 0;
    collectHidden(*generated[0], readable[0], "fadec1.", hidden, hiddenValues, k);
    collectHidden(*generated[1], readable[1], "fadec2.", hidden, hiddenValues, k);
    for (size_t h = 0; h < hidden.size(); h++) {
      auto [hg, hr] = hiddenValues[h];
      if (std::memcmp(&hg, &hr, sizeof(double)) != 0) {
        Hidden& x = hidden[h];
        if (x.firstStep < 0) {
          x.firstStep = steps;
          x.firstTime = in[0].time.simulation_time;
          x.firstGenerated = hg;
          x.firstReadable = hr;
        }
        x.mismatches++;
        if (firstHidden < 0) firstHidden = steps;
      }
    }
    if (n == 0) return;
    mismatchSteps++;
    if (firstMismatch >= 0) return;
    firstMismatch = steps;
    std::printf("FIRST MISMATCH at step %ld (frame %ld%s), t = %.3f s\n", steps, info.frame,
                info.gapFill ? ", gap fill" : "", in[0].time.simulation_time);
    int listed = 0;
    for (const auto& ls : leaves) {
      for (const FcuLeaf& l : ls) {
        if (l.firstStep != steps) continue;
        if (listed++ < maxListed) {
          std::printf("  %-56s generated %-16s readable %-16s\n", l.name.c_str(), l.firstExpected.c_str(),
                      l.firstGot.c_str());
        }
      }
    }
    if (listed > maxListed) std::printf("  ... %d fields in all\n", listed);
    if (firstHidden >= 0) {
      std::printf("  hidden state diverged first at step %ld:", firstHidden);
      for (const Hidden& h : hidden) {
        if (h.firstStep == firstHidden) {
          std::printf(" %s (generated %.17g readable %.17g)", h.name.c_str(), h.firstGenerated, h.firstReadable);
        }
      }
      std::printf("\n");
    } else {
      std::printf("  hidden state: no divergence before this step\n");
    }
  };

  while (drive.next(step)) {
  }

  std::string mode = drive.recordedInputs() ? "recorded FADEC inputs (3209004)" : "synthetic from the record";
  auto addMode = [&](const char* name, long seed, long episodes) {
    if (seed >= 0) mode += std::string(", ") + name + " (seed " + std::to_string(seed) + ", " + std::to_string(episodes) +
                           " episodes)";
  };
  addMode("script", scriptSeed, script ? script->episodes() : 0);
  addMode("lever", leverSeed, lever ? lever->episodes() : 0);
  addMode("athr", athrSeed, athr ? athr->episodes() : 0);
  addMode("sim", simSeed, sim ? sim->episodes() : 0);
  std::printf("file     %s (interface %llu)\nmode     %s\n", path.c_str(),
              static_cast<unsigned long long>(drive.version()), mode.c_str());
  std::printf("steps    %ld (%ld frames, %ld gap-fill steps in %ld gaps), 2 FADECs per step\n", steps, drive.frames(),
              drive.gapSteps(), drive.gaps());
  if (!drive.recordedInputs()) {
    const auto& l = drive.limits();
    std::printf("limits   last IDLE %.2f%s CLB %.2f MCT %.2f FLEX %.2f TOGA %.2f\n", l.idle, l.idleKnown ? "" : " (default)",
                l.clb, l.mct, l.flex, l.toga);
  }
  std::printf("time     generated %.3f us/step, readable %.3f us/step (ratio %.2f)\n", genSeconds / steps / 2 * 1e6,
              readSeconds / steps / 2 * 1e6, readSeconds / genSeconds);
  for (int e = 0; e < 2; e++) {
    const Events& v = events[e];
    std::printf("events   fadec%d: control lost %ld (disconnect %ld, disabled %ld, bit 13 %ld), thrust lock set %ld "
                "(%ld within 0.5 s of a disconnect), A/THR disabled %ld, FLX in flight %ld steps\n",
                e + 1, v.controlLost, v.lostByDisconnect, v.lostByDisabled, v.lostByBit13, v.lockSet,
                v.lockAfterDisconnect, v.disabledSet, v.flexLatchInFlight);
  }
  std::printf("recorded fadec1 thrust lock: %ld episodes (%ld within 0.5 s of an instinctive disconnect), %ld frames\n",
              recordedLocks.episodes, recordedLocks.afterDisconnect, recordedLocks.frames);

  long failing = 0;
  std::printf("\n%-56s %9s %10s  %s\n", "field", "mismatch", "first", "first values (generated / readable)");
  for (const auto& ls : leaves) {
    for (const FcuLeaf& l : ls) {
      if (l.mismatches == 0) continue;
      failing++;
      std::printf("%-56s %9ld %10ld  t %.2f %s / %s\n", l.name.c_str(), l.mismatches, l.firstStep, l.firstTime,
                  l.firstExpected.c_str(), l.firstGot.c_str());
    }
  }
  // coverage of the generated outputs: "cov NAME nonzero changes"
  std::printf("\n");
  for (const auto& ls : leaves) {
    for (const FcuLeaf& l : ls) {
      if (l.name.find(".output.") == std::string::npos && l.name.find(".data_computed.") == std::string::npos &&
          l.name.find(".fadec_bus_output.") == std::string::npos) {
        continue;
      }
      std::printf("cov %-60s %10ld %8ld\n", l.name.c_str(), l.nonzero, l.changes);
    }
  }
  if (fidelitySteps > 0) {
    // generated FADEC 1 vs the recorded FADEC 1 outputs: percentage of frames equal (synthetic inputs are approximate:
    // commanded N1, thrust limits and the FADEC 2 N1 are reconstructed)
    for (const auto* ls : {&fidelityOut, &fidelityBus}) {
      for (const FcuLeaf& l : *ls) {
        std::printf("fidelity %-55s %6.2f %%\n", l.name.c_str(), 100.0 * (fidelitySteps - l.mismatches) / fidelitySteps);
      }
    }
  }
  long hiddenFailing = 0;
  for (const Hidden& h : hidden) {
    if (h.mismatches == 0) continue;
    hiddenFailing++;
    std::printf("hidden.%-50s %9ld %10ld  t %.2f generated %.17g readable %.17g\n", h.name.c_str(), h.mismatches,
                h.firstStep, h.firstTime, h.firstGenerated, h.firstReadable);
  }
  std::printf("\n%zu fields compared, %ld with mismatches, %ld mismatching steps; %zu hidden memories, %ld diverged; "
              "%ld init mismatches\n",
              leaves[0].size() + leaves[1].size(), failing, mismatchSteps, hidden.size(), hiddenFailing, initMismatches);
  return failing == 0 && hiddenFailing == 0 && initMismatches == 0 ? 0 : 1;
}
