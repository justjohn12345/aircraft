// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Native driver and test of the sim's FAC shadow checker (fbw-a32nx/src/wasm/fbw_a320/src/shadow/FacShadow.{h,cpp} and
// ShadowChecker.{h,cpp}), the same sources the wasm module compiles. Both generated FacComputers (FAC 1 and 2) are
// stepped with the FAC inputs of a recording (src/fac_drive.h: the recorded model inputs of a 3209005 file, else the
// inputs rebuilt from the record) and, after every step, each FacShadow gets exactly what the sim wrapper (fac/Fac.cpp)
// gives it: the inputs of that step and the generated outputs. As in the sim, a FAC that did not step (a recorded power
// outage) steps neither model. --synthetic: rebuild the inputs even on a 3209005 file; --power SEED: power / health
// episodes (fac_scenarios.h, computer_running off: restarts). Command line and self-test: src/shadow_unit_test.h; the
// self-test also checks the NaN rule (two NaNs match whatever their sign and payload, a NaN against a number does not).

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>

#include "FacOutputLeaves.gen.h"
#include "FacShadow.h"
#include "fac_drive.h"  // includes FacComputer.h
#include "fac_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fac_scenarios.h"
#include "fcu_leaves.h"
#include "readable/fac/fac.h"
#include "shadow_unit_test.h"

namespace {

using namespace shadow_unit_test;

std::vector<VisitedLeaf> visitedLeaves() {
  std::vector<VisitedLeaf> out;
  for (const FcuLeaf& l : fcuLeaves<fac_outputs>()) {
    const shadow::LeafKind kind = l.kind == FieldKind::Bool    ? shadow::LeafKind::Bool
                                  : l.kind == FieldKind::Float ? shadow::LeafKind::Float
                                  : l.kind == FieldKind::Enum  ? shadow::LeafKind::Enum
                                                               : shadow::LeafKind::Int;
    out.push_back({l.name, l.offset, l.size, kind});
  }
  return out;
}

RunResult run(const std::string& path, const Options& opt, const Perturbation& p, const std::string& logPath, bool keepRunning) {
  RunResult res;
  const auto start = Clock::now();
  // as fac/Fac.cpp: constructed and initialize()d, then stepped
  std::unique_ptr<FacComputer> generated[2] = {std::make_unique<FacComputer>(), std::make_unique<FacComputer>()};
  for (auto& g : generated) {
    g->initialize();
  }
  auto ext = std::make_unique<FacComputer::ExternalInputs_FacComputer_T>();
  std::unique_ptr<FacShadow> shadows[2] = {std::make_unique<FacShadow>(1), std::make_unique<FacShadow>(2)};
  auto log = FacShadow::makeLog("");
  log->setPath(logPath);
  std::unique_ptr<fac::readable::FacComputer> bare[2];
  if (opt.timing) {
    for (auto& b : bare) {
      b = std::make_unique<fac::readable::FacComputer>();
      b->initialize();
    }
  }
  FacInputDrive::Options dopt;
  dopt.rebuild = opt.synthetic;
  FacInputDrive drive(path, dopt);
  if (!drive.ok()) {
    res.error = drive.error();
    return res;
  }
  res.version = drive.version();
  res.recordedInputs = drive.recordedInputs();
  std::unique_ptr<fac_scenarios::PowerScenario> power;
  if (opt.powerSeed >= 0) {
    power = std::make_unique<fac_scenarios::PowerScenario>(static_cast<uint64_t>(opt.powerSeed));
  }
  auto outCopy = std::make_unique<fac_outputs>();
  shadow::ShadowChecker* units[2] = {&shadows[0]->checker(), &shadows[1]->checker()};

  auto step = [&](fac_inputs(&in)[2], const FacInputDrive::StepInfo& info) {
    if (power) {
      power->apply(in, in[0].time.dt);
    }
    res.steps++;
    for (int e = 0; e < 2; e++) {
      if (!info.stepped[e]) {
        continue;  // the wrapper does not step the model (nor the shadow) during a power outage
      }
      ext->in = in[e];
      const auto t0 = Clock::now();
      generated[e]->setExternalInputs(ext.get());
      generated[e]->step();
      const auto t1 = Clock::now();
      res.genSeconds += std::chrono::duration<double>(t1 - t0).count();
      if (opt.noShadow) {
        continue;
      }
      const fac_outputs& out = generated[e]->getExternalOutputs().out;
      const uint64_t frame = shadows[e]->checker().frames() + 1;
      if (keepRunning && e == 0) {
        res.running.push_back(in[e].sim_data.computer_running);
      }
      if (e == 0 && frame == p.frame && (p.flipLeaf >= 0 || p.perturbInput)) {
        *outCopy = out;
        fac_inputs inCopy = in[e];
        if (p.flipLeaf >= 0) {
          reinterpret_cast<uint8_t*>(outCopy.get())[fac_shadow::kOutputLeaves[p.flipLeaf].offset] ^= 1;
        }
        if (p.perturbInput) {
          inCopy.bus_inputs.adr_own_bus.airspeed_computed_kn.Data += 5.0F;
        }
        shadows[e]->step(inCopy, *outCopy);
        continue;
      }
      if (opt.timing) {
        const auto t2 = Clock::now();
        shadows[e]->step(in[e], out);
        const auto t3 = Clock::now();
        bare[e]->step(in[e]);
        res.shadowSeconds += std::chrono::duration<double>(t3 - t2).count();
        res.bareSeconds += std::chrono::duration<double>(Clock::now() - t3).count();
      } else {
        shadows[e]->step(in[e], out);
      }
    }
  };

  const std::string fdrName = baseName(path);
  long record = 0;
  while (drive.next(step)) {
    log->update(units, 2, fdrName, record++);
  }
  log->writeSummary(units, 2);
  res.wallSeconds = std::chrono::duration<double>(Clock::now() - start).count();
  for (auto& s : shadows) {
    res.units.push_back(unitResult(s->checker()));
  }
  res.ok = true;
  return res;
}

/** The NaN rule: two NaNs in a float leaf match whatever their sign and payload; a NaN against a number does not. */
void nanRuleTest() {
  int leaf = -1;
  for (size_t i = 0; i < fac_shadow::kOutputLeafCount; i++) {
    if (std::strcmp(fac_shadow::kOutputLeaves[i].name, "bus_outputs.v_ls_kn.Data") == 0) {
      leaf = static_cast<int>(i);
    }
  }
  const shadow::Leaf& l = fac_shadow::kOutputLeaves[leaf < 0 ? 0 : leaf];
  auto a = std::make_unique<fac_outputs>();
  auto b = std::make_unique<fac_outputs>();
  std::memset(static_cast<void*>(a.get()), 0, sizeof(fac_outputs));
  std::memset(static_cast<void*>(b.get()), 0, sizeof(fac_outputs));
  auto set = [&](fac_outputs* o, uint32_t bits) { std::memcpy(reinterpret_cast<uint8_t*>(o) + l.offset, &bits, 4); };
  shadow::ShadowChecker checker(1, fac_shadow::kOutputLeaves, fac_shadow::kOutputLeafCount, sizeof(fac_outputs), true, true);
  shadow::FrameInputs in;
  set(a.get(), 0x7fc00000u);  // quiet NaN
  set(b.get(), 0xffc00001u);  // negative NaN, other payload
  checker.step(a.get(), b.get(), in);
  set(b.get(), 0x42c80000u);  // 100.0
  checker.step(a.get(), b.get(), in);
  set(a.get(), 0x42c80000u);
  checker.step(a.get(), b.get(), in);
  check(leaf >= 0 && checker.frames() == 3 && checker.mismatchFrames() == 1 && checker.nanEqualFrames() == 1 &&
            checker.firstMismatchFrame() == 2 && checker.leafMismatches(leaf) == 1,
        "NaN rule: two NaNs of different sign and payload match (1 frame counted), a NaN against 100 does not (1 mismatch)");
}

}  // namespace

int main(int argc, char** argv) {
  const shadow_unit_test::Computer fac = {
      "FAC",
      "fac-shadow-test",
      fac_shadow::kOutputLeaves,
      fac_shadow::kOutputLeafCount,
      sizeof(fac_outputs),
      2,
      true,
      {"bus_outputs.v_ls_kn.Data", "discrete_outputs.yaw_damper_engaged", "flight_envelope.v_max_kn",
       "bus_outputs.discrete_word_5.SSM"},
      "bus_inputs.adr_own_bus.airspeed_computed_kn.Data + 5",
      "data.bus_inputs.adr_own_bus.airspeed_computed_kn.Data",
      visitedLeaves,
      run,
  };
  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "--self-test") == 0) {
      nanRuleTest();
    }
  }
  return shadow_unit_test::main(fac, argc, argv);
}
