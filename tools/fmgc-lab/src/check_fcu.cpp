// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable FCU (readable/fcu/) against the generated FcuComputer over a flight data
// recording. Both computers run in lockstep on the same inputs, free-running (the readable one never sees a generated
// value); every step the whole fcu_outputs (data, logic, discrete outputs, bus outputs) is compared leaf by leaf,
// bit-exact, and so are the hidden memories (window memories, dash timers, latches, EFIS panel states, value change
// detector delays), which usually diverge first.
//
// Inputs (src/fcu_drive.h): the recorded FCU inputs of a 3209001 recording, or the synthetic drive built from the
// FMGC record of any recording (--synthetic forces it on 3209001), with --fmgc2 opp|copy|invalid|ncd|swapped and
// --reconstruct (pilot actions inferred from the recorded FCU bus; prints how closely the generated FCU then follows
// the recorded bus). Injectors (src/fcu_scenarios.h), combinable: --script, --panel, --priority, --bus, --sim, --power
// SEED. --warm FILE: run both over FILE first, then re-construct the generated FCU and initialize() both (checks that
// the readable initialize() is a complete reset).
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
#include "FcuComputer.h"
#undef private

#include "fcu.h"
#include "fcu_drive.h"
#include "fcu_leaves.h"
#include "fcu_scenarios.h"

namespace R = fcu::readable;
namespace S = fcu_scenarios;
using Clock = std::chrono::steady_clock;

namespace {

struct Hidden {
  const char* name;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0, firstReadable = 0;
};

/** Pairs (generated, readable) of every hidden memory, as doubles (exact for floats, bools, enums). */
void collectHidden(const FcuComputer& g, const R::FcuComputer& r, std::vector<Hidden>& hidden,
                   std::vector<std::pair<double, double>>& values) {
  const auto& d = g.FcuComputer_DWork;
  size_t k = 0;
  auto add = [&](const char* name, double gv, double rv) {
    if (k == hidden.size()) hidden.push_back({name});
    if (values.size() <= k) values.resize(k + 1);
    values[k++] = {gv, rv};
  };
  add("running(Runtime_MODE)", d.Runtime_MODE, r.running());
  add("afs.trkFpaActive(p_trk_fpa_active)", d.p_trk_fpa_active, r.afsControls().trkFpaActive());
  add("afs.metricAltActive(p_metric_alt_active)", d.p_metric_alt_active, r.afsControls().metricAltActive());
  add("speed.memory(pValue_p)", d.pValue_p, r.speedWindow().memory());
  add("speed.dashEventTime(eventTime)", d.eventTime, r.speedWindow().dashEventTime());
  add("heading.memory(pValue_m)", d.pValue_m, r.headingWindow().memory());
  add("heading.dashEventTime(sf_MATLABFunction1)", d.sf_MATLABFunction1.eventTime, r.headingWindow().dashEventTime());
  add("altitude.memory(pValue_e)", d.pValue_e, r.altitudeWindow().memory());
  add("vsFpa.memory(pValue)", d.pValue, r.vsFpaWindow().memory());
  add("vsFpa.dashEventTime(sf_MATLABFunction1_o)", d.sf_MATLABFunction1_o.eventTime, r.vsFpaWindow().dashEventTime());
  static const char* delays[4] = {"valueChange.hdgTrk(DelayInput1[0])", "valueChange.alt(DelayInput1[1])",
                                  "valueChange.vsFpa(DelayInput1[2])", "valueChange.spdMach(DelayInput1[3])"};
  for (int i = 0; i < 4; i++) add(delays[i], d.DelayInput1_DSTATE[i], r.valueChangePrevious(i));
  const R::EfisControlPanel* panels[2] = {&r.captEfis(), &r.foEfis()};
  const FcuComputer::rtDW_EFISFilterLogic_FcuComputer_T* filters[2] = {&d.sf_EFISFilterLogic, &d.sf_EFISFilterLogic_k};
  const FcuComputer::rtDW_MATLABFunction_FcuComputer_gf_T* fds[2] = {&d.sf_MATLABFunction_ml, &d.sf_MATLABFunction_bx};
  const FcuComputer::rtDW_MATLABFunction_FcuComputer_k_T* lss[2] = {&d.sf_MATLABFunction_bs, &d.sf_MATLABFunction_ho3};
  const FcuComputer::rtDW_MATLABFunction_FcuComputer_d_T* refs[2] = {&d.sf_MATLABFunction_ofc, &d.sf_MATLABFunction_o3};
  const FcuComputer::rtDW_MATLABFunction1_FcuComputer_j_T* baros[2] = {&d.sf_MATLABFunction1_ou,
                                                                       &d.sf_MATLABFunction1_g};
  static const char* names[2][8] = {
      {"capt.filter", "capt.fdOn", "capt.lsOn", "capt.std", "capt.qnh", "capt.qfe", "capt.baroHpa", "capt.baroInHg"},
      {"fo.filter", "fo.fdOn", "fo.lsOn", "fo.std", "fo.qnh", "fo.qfe", "fo.baroHpa", "fo.baroInHg"}};
  for (int s = 0; s < 2; s++) {
    const R::EfisControlPanel& p = *panels[s];
    add(names[s][0], static_cast<int>(filters[s]->pEfisFilter), static_cast<int>(p.filter()));
    add(names[s][1], fds[s]->fdActive, p.fdOn());
    // the T flip-flop's value only matters once primed (the generated one keeps a stale pY until then)
    add(names[s][2], lss[s]->pY_not_empty ? lss[s]->pY : 0, lss[s]->pY_not_empty ? p.lsOn() : 0);
    add(names[s][3], refs[s]->std_active, p.stdActive());
    add(names[s][4], refs[s]->qnh_active, p.qnhActive());
    add(names[s][5], refs[s]->qfe_active, p.qfeActive());
    add(names[s][6], baros[s]->pValueHpa, p.baroHpa());
    add(names[s][7], baros[s]->pValueInhg, p.baroInHg());
  }
}

std::string modeString(const FcuInputDrive& drive, bool reconstruct, Fmgc2Mode fmgc2) {
  if (drive.recordedInputs()) return "recorded FCU inputs (3209001)";
  static const char* f2[] = {"opp", "copy", "invalid", "ncd", "swapped"};
  std::string m = std::string("synthetic from the FMGC record, FMGC 2 bus ") + f2[static_cast<int>(fmgc2)];
  if (reconstruct) m += ", pilot actions reconstructed from the recorded FCU bus";
  return m;
}

}  // namespace

int main(int argc, char** argv) {
  long scriptSeed = -1, panelSeed = -1, prioritySeed = -1, busSeed = -1, simSeed = -1, powerSeed = -1;
  FcuInputDrive::Options opt;
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
    if ((a == "--script" && seed(scriptSeed)) || (a == "--panel" && seed(panelSeed)) ||
        (a == "--priority" && seed(prioritySeed)) || (a == "--bus" && seed(busSeed)) || (a == "--sim" && seed(simSeed)) ||
        (a == "--power" && seed(powerSeed))) {
    } else if (a == "--synthetic") {
      opt.synthetic = true;
    } else if (a == "--reconstruct") {
      opt.reconstruct = true;
    } else if (a == "--fmgc2" && i + 1 < argc) {
      usage = !parseFmgc2Mode(argv[++i], opt.fmgc2);
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
                 "usage: fcu-check [--synthetic] [--fmgc2 opp|copy|invalid|ncd|swapped] [--reconstruct] [--script SEED] "
                 "[--panel SEED] [--priority SEED] [--bus SEED] [--sim SEED] [--power SEED] [--warm FILE] [--list N] "
                 "<file.fdr>\n");
    return 2;
  }

  auto generated = std::make_unique<FcuComputer>();
  generated->initialize();
  auto readable = std::make_unique<R::FcuComputer>();
  readable->initialize();
  auto ext = std::make_unique<FcuComputer::ExternalInputs_FcuComputer_T>();

  if (!warmPath.empty()) {
    FcuInputDrive warm(warmPath, opt);
    if (!warm.ok()) {
      std::fprintf(stderr, "error: %s\n", warm.error().c_str());
      return 2;
    }
    while (warm.next([&](fcu_inputs& in, const FcuInputDrive::StepInfo&) {
      ext->in = in;
      generated->setExternalInputs(ext.get());
      generated->step();
      readable->step(in);
    })) {
    }
    // the generated initialize() is not a complete reset of a used instance: re-construct it in place
    generated->~FcuComputer();
    new (generated.get()) FcuComputer();
    generated->initialize();
    readable->initialize();
    std::printf("warm     %s (%ld frames), then initialize()\n", warmPath.c_str(), warm.frames());
  }

  FcuInputDrive drive(path, opt);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }
  drive.feedback = &generated->FcuComputer_Y.out;

  std::unique_ptr<S::ScriptScenario> script;
  if (scriptSeed >= 0) script = std::make_unique<S::ScriptScenario>(scriptSeed);
  std::unique_ptr<S::RandomPanelScenario> panel;
  if (panelSeed >= 0) panel = std::make_unique<S::RandomPanelScenario>(panelSeed);
  std::unique_ptr<S::PriorityScenario> priority;
  if (prioritySeed >= 0) priority = std::make_unique<S::PriorityScenario>(prioritySeed);
  std::unique_ptr<S::BusScenario> bus;
  if (busSeed >= 0) bus = std::make_unique<S::BusScenario>(busSeed);
  std::unique_ptr<S::SimScenario> sim;
  if (simSeed >= 0) sim = std::make_unique<S::SimScenario>(simSeed);
  std::unique_ptr<S::PowerScenario> power;
  if (powerSeed >= 0) power = std::make_unique<S::PowerScenario>(powerSeed);

  std::vector<FcuLeaf> leaves = fcuLeaves<fcu_outputs>();
  std::vector<FcuLeaf> fidelity = fcuLeaves<base_fcu_bus>("bus_outputs.");

  // power-up outputs must be identical
  long initMismatches = 0;
  {
    std::vector<FcuLeaf> l0 = fcuLeaves<fcu_outputs>();
    initMismatches = compareFcuLeaves(l0, generated->FcuComputer_Y.out, readable->outputs(), 0, 0, false);
    for (const FcuLeaf& l : l0) {
      if (l.mismatches) std::printf("init mismatch: %s\n", l.name.c_str());
    }
  }

  std::vector<Hidden> hidden;
  std::vector<std::pair<double, double>> hiddenValues;
  long steps = 0, runningSteps = 0, restarts = 0, mismatchSteps = 0, firstMismatch = -1, firstHidden = -1;
  long fidelitySteps = 0;
  bool prevRunning = false;
  double genSeconds = 0, readSeconds = 0;

  auto step = [&](fcu_inputs& in, const FcuInputDrive::StepInfo& info) {
    const double dt = in.time.dt;
    if (script) script->apply(in, dt);
    if (panel) panel->apply(in, dt);
    if (priority) priority->apply(in, dt);
    if (bus) bus->apply(in, dt);
    if (sim) sim->apply(in, dt);
    if (power) power->apply(in, dt);

    steps++;
    const bool running = in.sim_data.computer_running;
    runningSteps += running;
    restarts += running && !prevRunning;
    prevRunning = running;

    ext->in = in;
    auto t0 = Clock::now();
    generated->setExternalInputs(ext.get());
    generated->step();
    auto t1 = Clock::now();
    const fcu_outputs& r = readable->step(in);
    auto t2 = Clock::now();
    genSeconds += std::chrono::duration<double>(t1 - t0).count();
    readSeconds += std::chrono::duration<double>(t2 - t1).count();
    const fcu_outputs& g = generated->FcuComputer_Y.out;

    const int n = compareFcuLeaves(leaves, g, r, steps, in.time.simulation_time, running);

    // how closely the generated FCU follows the recorded FCU bus (synthetic drive: as FMGC 1 received it)
    if (!info.gapFill && !info.recordedInputs && running &&
        info.fdr->fmgc1.bus_inputs.fcu_bus.fcu_discrete_word_1.SSM ==
            static_cast<uint32_t>(SignStatusMatrix::NormalOperation)) {
      fidelitySteps++;
      base_fcu_bus rec = info.fdr->fmgc1.bus_inputs.fcu_bus;
      S::setBit(rec.fcu_discrete_word_2, 24, false);  // FCU 1/2 healthy, set by the sim wrapper
      S::setBit(rec.fcu_discrete_word_2, 25, false);
      compareFcuLeaves(fidelity, rec, g.bus_outputs, steps, in.time.simulation_time, false);
    }

    collectHidden(*generated, *readable, hidden, hiddenValues);
    for (size_t k = 0; k < hidden.size(); k++) {
      auto [hg, hr] = hiddenValues[k];
      if (std::memcmp(&hg, &hr, sizeof(double)) != 0) {
        Hidden& h = hidden[k];
        if (h.firstStep < 0) {
          h.firstStep = steps;
          h.firstTime = in.time.simulation_time;
          h.firstGenerated = hg;
          h.firstReadable = hr;
        }
        h.mismatches++;
        if (firstHidden < 0) firstHidden = steps;
      }
    }
    if (n == 0) return;
    mismatchSteps++;
    if (firstMismatch >= 0) return;
    firstMismatch = steps;
    std::printf("FIRST MISMATCH at step %ld (frame %ld%s), t = %.3f s, computer %s\n", steps, info.frame,
                info.gapFill ? ", gap fill" : "", in.time.simulation_time, running ? "running" : "not running");
    int listed = 0;
    for (const FcuLeaf& l : leaves) {
      if (l.firstStep != steps) continue;
      if (listed++ < maxListed) {
        std::printf("  %-56s generated %-16s readable %-16s\n", l.name.c_str(), l.firstExpected.c_str(),
                    l.firstGot.c_str());
      }
    }
    if (listed > maxListed) std::printf("  ... %d fields in all\n", listed);
    if (firstHidden >= 0) {
      std::printf("  hidden state diverged first at step %ld:", firstHidden);
      for (const Hidden& h : hidden) {
        if (h.firstStep == firstHidden) {
          std::printf(" %s (generated %.17g readable %.17g)", h.name, h.firstGenerated, h.firstReadable);
        }
      }
      std::printf("\n");
    } else {
      std::printf("  hidden state: no divergence before this step\n");
    }
  };

  while (drive.next(step)) {
  }

  std::string mode = modeString(drive, opt.reconstruct, opt.fmgc2);
  auto addMode = [&](const char* name, long seed, long episodes) {
    if (seed >= 0) mode += std::string(", ") + name + " (seed " + std::to_string(seed) + ", " + std::to_string(episodes) +
                           " episodes)";
  };
  addMode("script", scriptSeed, script ? script->episodes() : 0);
  addMode("panel", panelSeed, panel ? panel->episodes() : 0);
  addMode("priority", prioritySeed, priority ? priority->episodes() : 0);
  addMode("bus", busSeed, bus ? bus->episodes() : 0);
  addMode("sim", simSeed, sim ? sim->episodes() : 0);
  addMode("power", powerSeed, power ? power->episodes() : 0);
  std::printf("file     %s (interface %llu)\nmode     %s\n", path.c_str(),
              static_cast<unsigned long long>(drive.version()), mode.c_str());
  std::printf("steps    %ld (%ld running, %ld (re)starts, %ld frames, %ld gap-fill steps in %ld gaps)\n", steps,
              runningSteps, restarts, drive.frames(), drive.gapSteps(), drive.gaps());
  if (opt.reconstruct) {
    std::printf("reconstruct %ld presses, %ld turns\n", drive.reconstructedPresses, drive.reconstructedTurns);
  }
  std::printf("time     generated %.3f us/step, readable %.3f us/step (ratio %.2f)\n", genSeconds / steps * 1e6,
              readSeconds / steps * 1e6, readSeconds / genSeconds);

  long failing = 0;
  std::printf("\n%-56s %9s %10s  %s\n", "field", "mismatch", "first", "first values (generated / readable)");
  for (const FcuLeaf& l : leaves) {
    if (l.mismatches == 0) continue;
    failing++;
    std::printf("%-56s %9ld %10ld  t %.2f %s / %s\n", l.name.c_str(), l.mismatches, l.firstStep, l.firstTime,
                l.firstExpected.c_str(), l.firstGot.c_str());
  }
  // coverage of the generated outputs on running steps: "cov NAME nonzero changes"
  std::printf("\n");
  for (const FcuLeaf& l : leaves) {
    if (l.name.rfind("data.", 0) == 0) continue;
    std::printf("cov %-60s %10ld %8ld\n", l.name.c_str(), l.nonzero, l.changes);
  }
  if (fidelitySteps > 0) {
    // recorded FCU bus vs generated: percentage of steps equal (the wrapper's healthy bits 24/25 of word 2 are masked;
    // builds with the C13 fix mask word 1 bit 15 in managed speed, so that word can differ)
    for (const FcuLeaf& l : fidelity) {
      std::printf("fidelity %-55s %6.2f %%\n", l.name.c_str(), 100.0 * (fidelitySteps - l.mismatches) / fidelitySteps);
    }
  }
  long hiddenFailing = 0;
  for (const Hidden& h : hidden) {
    if (h.mismatches == 0) continue;
    hiddenFailing++;
    std::printf("hidden.%-50s %9ld %10ld  t %.2f generated %.17g readable %.17g\n", h.name, h.mismatches, h.firstStep,
                h.firstTime, h.firstGenerated, h.firstReadable);
  }
  std::printf("\n%zu fields compared, %ld with mismatches, %ld mismatching steps; %zu hidden memories, %ld diverged; "
              "%ld init mismatches\n",
              leaves.size(), failing, mismatchSteps, hidden.size(), hiddenFailing, initMismatches);
  return failing == 0 && hiddenFailing == 0 && initMismatches == 0 ? 0 : 1;
}
