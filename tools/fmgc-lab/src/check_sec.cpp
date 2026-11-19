// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Differential check of the readable SEC (readable/sec/, laws in readable/efcs/) against the generated SecComputer
// over a flight data recording. Three pairs (SEC 1-3) run in lockstep on the same inputs, free-running (the readable SECs
// never see a generated value); every step the whole sec_outputs (data, laws, logic, discretes, analog orders,
// bus) is compared leaf by leaf, bit-exact (floats by bit pattern; two NaNs match whatever their sign and payload), and
// so are the hidden memories (every DWork field of the SEC and of its three laws, src/sec_state.h).
//
// Inputs (src/efcs_drive.h): the three SECs' inputs rebuilt from any recording. Injectors, combinable: --script
// (src/sec_scenarios.h), --failures, --discretes, --power, --sim (src/efcs_scenarios.h). --warm FILE: run both over
// FILE first, then re-construct the generated SECs and initialize() both (checks that the readable initialize() is a
// complete reset). --time: step times only.
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

#include "efcs_drive.h"  // includes ElacComputer.h with test-only access to its state
#include "sec_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"

#include "efcs_scenarios.h"
#include "sec.h"
#include "sec_scenarios.h"
#include "sec_state.h"

namespace R = sec::readable;
namespace S = efcs_scenarios;
using Clock = std::chrono::steady_clock;

namespace {

struct Hidden {
  std::string name;
  long mismatches = 0;
  long firstStep = -1;
  double firstTime = 0;
  double firstGenerated = 0, firstReadable = 0;
};

/** The SR flip-flop table against the readable latch. Returns the number of differing entries. */
int checkTable() {
  const auto& p = SecComputer::SecComputer_P;
  int bad = 0;
  for (int i = 0; i < 8; i++) {
    efcs::blocks::SrLatch l;
    l.q = i & 1;
    const bool s = i & 4, rs = i & 2;
    l.update(s, rs);
    bad += p.Logic_table[i] != l.q;
  }
  return bad;
}

}  // namespace

int main(int argc, char** argv) {
  long scriptSeed = -1, failuresSeed = -1, discretesSeed = -1, powerSeed = -1, simSeed = -1;
  int maxListed = 40;
  bool timeOnly = false;
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
    if ((a == "--script" && seed(scriptSeed)) || (a == "--failures" && seed(failuresSeed)) ||
        (a == "--discretes" && seed(discretesSeed)) || (a == "--power" && seed(powerSeed)) ||
        (a == "--sim" && seed(simSeed))) {
    } else if (a == "--list" && i + 1 < argc) {
      maxListed = std::atoi(argv[++i]);
    } else if (a == "--warm" && i + 1 < argc) {
      warmPath = argv[++i];
    } else if (a == "--time") {
      timeOnly = true;
    } else if (path.empty() && !a.empty() && a[0] != '-') {
      path = a;
    } else {
      usage = true;
    }
  }
  if (path.empty() || usage) {
    std::fprintf(stderr,
                 "usage: sec-check [--script SEED] [--failures SEED] [--discretes SEED] [--power SEED] [--sim SEED] "
                 "[--warm FILE] [--time] [--list N] <file.fdr>\n");
    return 2;
  }

  std::unique_ptr<SecComputer> generated[3] = {std::make_unique<SecComputer>(), std::make_unique<SecComputer>(),
                                               std::make_unique<SecComputer>()};
  auto readable = std::make_unique<R::SecComputer[]>(3);
  for (int e = 0; e < 3; e++) {
    generated[e]->initialize();
    readable[e].initialize();
  }
  auto ext = std::make_unique<SecComputer::ExternalInputs_SecComputer_T>();

  if (!warmPath.empty()) {
    EfcsInputDrive warm(warmPath);
    if (!warm.ok()) {
      std::fprintf(stderr, "error: %s\n", warm.error().c_str());
      return 2;
    }
    sec_scenarios::ScriptScenario script(99);
    auto none = [](elac_inputs (&)[2], const EfcsInputDrive::StepInfo&) {};
    while (warm.next(none, [&](sec_inputs (&in)[3], const EfcsInputDrive::StepInfo&) {
      script.apply(in, in[0].time.dt);
      for (int e = 0; e < 3; e++) {
        ext->in = in[e];
        generated[e]->setExternalInputs(ext.get());
        generated[e]->step();
        readable[e].step(in[e]);
      }
    })) {
    }
    for (int e = 0; e < 3; e++) {
      generated[e]->~SecComputer();
      new (generated[e].get()) SecComputer();
      generated[e]->initialize();
      readable[e].initialize();
    }
    std::printf("warm     %s (%ld frames, script 99), then initialize()\n", warmPath.c_str(), warm.frames());
  }

  EfcsInputDrive drive(path);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }

  std::unique_ptr<sec_scenarios::ScriptScenario> script;
  if (scriptSeed >= 0) script = std::make_unique<sec_scenarios::ScriptScenario>(scriptSeed);
  std::unique_ptr<S::FailureScenario<sec_inputs, 3>> failures;
  if (failuresSeed >= 0) failures = std::make_unique<S::FailureScenario<sec_inputs, 3>>(failuresSeed);
  std::unique_ptr<S::DiscreteScenario<sec_inputs, 3>> discretes;
  if (discretesSeed >= 0) discretes = std::make_unique<S::DiscreteScenario<sec_inputs, 3>>(discretesSeed);
  std::unique_ptr<S::PowerScenario<sec_inputs, 3>> power;
  if (powerSeed >= 0) power = std::make_unique<S::PowerScenario<sec_inputs, 3>>(powerSeed);
  std::unique_ptr<S::SimScenario<sec_inputs, 3>> sim;
  if (simSeed >= 0) sim = std::make_unique<S::SimScenario<sec_inputs, 3>>(simSeed);

  std::vector<FcuLeaf> leaves[3] = {fcuLeaves<sec_outputs>("sec1."), fcuLeaves<sec_outputs>("sec2."),
                                    fcuLeaves<sec_outputs>("sec3.")};

  long initMismatches = checkTable();
  if (initMismatches) std::printf("init mismatch: SR flip-flop table differs from the readable latch\n");
  for (int e = 0; e < 3; e++) {
    std::vector<FcuLeaf> l0 = fcuLeaves<sec_outputs>();
    initMismatches += compareFcuLeaves(l0, generated[e]->SecComputer_Y.out, readable[e].outputs(), 0, 0, false);
    for (const FcuLeaf& l : l0) {
      if (l.mismatches) std::printf("init mismatch: sec%d.%s\n", e + 1, l.name.c_str());
    }
  }

  std::vector<Hidden> hidden;
  std::vector<std::pair<double, double>> hiddenValues;
  long steps = 0, mismatchSteps = 0, firstMismatch = -1, firstHidden = -1, restarts[3] = {0, 0, 0};
  bool prevRunning[3] = {false, false, false};
  double genSeconds = 0, readSeconds = 0;

  auto none = [](elac_inputs (&)[2], const EfcsInputDrive::StepInfo&) {};
  auto step = [&](sec_inputs (&in)[3], const EfcsInputDrive::StepInfo& info) {
    const double dt = in[0].time.dt;
    if (script) script->apply(in, dt);
    if (failures) failures->apply(in, dt);
    if (discretes) discretes->apply(in, dt);
    if (power) power->apply(in, dt);
    if (sim) sim->apply(in, dt);
    steps++;

    int n = 0;
    for (int e = 0; e < 3; e++) {
      ext->in = in[e];
      auto t0 = Clock::now();
      generated[e]->setExternalInputs(ext.get());
      generated[e]->step();
      auto t1 = Clock::now();
      const sec_outputs& r = readable[e].step(in[e]);
      auto t2 = Clock::now();
      genSeconds += std::chrono::duration<double>(t1 - t0).count();
      readSeconds += std::chrono::duration<double>(t2 - t1).count();
      if (timeOnly) continue;
      n += compareFcuLeaves(leaves[e], generated[e]->SecComputer_Y.out, r, steps, in[e].time.simulation_time, true,
                            true);
      const bool running = generated[e]->SecComputer_DWork.Runtime_MODE;
      restarts[e] += running && !prevRunning[e];
      prevRunning[e] = running;
    }
    if (timeOnly) return;

    size_t k = 0;
    for (int e = 0; e < 3; e++) {
      const std::string prefix = "sec" + std::to_string(e + 1) + ".";
      secPairs(*generated[e], readable[e], [&](const char* name, double gv, double rv) {
        if (k == hidden.size()) hidden.push_back({prefix + name});
        if (hiddenValues.size() <= k) hiddenValues.resize(k + 1);
        hiddenValues[k++] = {gv, rv};
      });
    }
    for (size_t h = 0; h < hidden.size(); h++) {
      auto [hg, hr] = hiddenValues[h];
      if (std::memcmp(&hg, &hr, sizeof(double)) != 0 && !(hg != hg && hr != hr)) {
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
    std::printf("FIRST MISMATCH at step %ld (frame %ld), t = %.3f s\n", steps, info.frame, in[0].time.simulation_time);
    int listed = 0;
    for (const auto& ls : leaves) {
      for (const FcuLeaf& l : ls) {
        if (l.firstStep != steps) continue;
        if (listed++ < maxListed) {
          std::printf("  %-64s generated %-16s readable %-16s\n", l.name.c_str(), l.firstExpected.c_str(),
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

  while (drive.next(none, step)) {
  }

  std::string mode = "inputs rebuilt from the record";
  auto addMode = [&](const char* name, long seed, long episodes) {
    if (seed >= 0) mode += std::string(", ") + name + " (seed " + std::to_string(seed) + ", " + std::to_string(episodes) +
                           " episodes)";
  };
  addMode("script", scriptSeed, script ? script->episodes() : 0);
  addMode("failures", failuresSeed, failures ? failures->episodes() : 0);
  addMode("discretes", discretesSeed, discretes ? discretes->episodes() : 0);
  addMode("power", powerSeed, power ? power->episodes() : 0);
  addMode("sim", simSeed, sim ? sim->episodes() : 0);
  std::printf("file     %s (interface %llu)\nmode     %s\n", path.c_str(),
              static_cast<unsigned long long>(drive.version()), mode.c_str());
  std::printf("steps    %ld (%ld frames), 3 SECs per step, restarts %ld + %ld + %ld\n", steps, drive.frames(),
              restarts[0], restarts[1], restarts[2]);
  std::printf("time     generated %.3f us/step, readable %.3f us/step (ratio %.2f)\n", genSeconds / steps / 3 * 1e6,
              readSeconds / steps / 3 * 1e6, readSeconds / genSeconds);
  if (timeOnly) return 0;

  long failing = 0;
  std::printf("\n%-64s %9s %10s  %s\n", "field", "mismatch", "first", "first values (generated / readable)");
  for (const auto& ls : leaves) {
    for (const FcuLeaf& l : ls) {
      if (l.mismatches == 0) continue;
      failing++;
      std::printf("%-64s %9ld %10ld  t %.2f %s / %s\n", l.name.c_str(), l.mismatches, l.firstStep, l.firstTime,
                  l.firstExpected.c_str(), l.firstGot.c_str());
    }
  }
  std::printf("\n");
  for (const auto& ls : leaves) {
    for (const FcuLeaf& l : ls) {
      if (l.name.find(".data.") != std::string::npos) continue;
      std::printf("cov %-64s %10ld %8ld\n", l.name.c_str(), l.nonzero, l.changes);
    }
  }
  long hiddenFailing = 0;
  for (const Hidden& h : hidden) {
    if (h.mismatches == 0) continue;
    hiddenFailing++;
    std::printf("hidden.%-56s %9ld %10ld  t %.2f generated %.17g readable %.17g\n", h.name.c_str(), h.mismatches,
                h.firstStep, h.firstTime, h.firstGenerated, h.firstReadable);
  }
  std::printf("\n%zu fields compared, %ld with mismatches, %ld mismatching steps; %zu hidden memories, %ld diverged; "
              "%ld init mismatches\n",
              leaves[0].size() + leaves[1].size() + leaves[2].size(), failing, mismatchSteps, hidden.size(), hiddenFailing, initMismatches);
  return failing == 0 && hiddenFailing == 0 && initMismatches == 0 ? 0 : 1;
}
