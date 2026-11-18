// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Native (and wasm) driver and test of the sim's FCU shadow checker (fbw-a32nx/src/wasm/fbw_a320/src/shadow/
// FcuShadow.{h,cpp} and ShadowChecker.{h,cpp}), the same sources the wasm module compiles. The generated FcuComputer is
// stepped with the FCU inputs of a recording (src/fcu_drive.h: the recorded model inputs of a 3209001+ file, else the
// synthetic drive built from the FMGC record) and, after every step, FcuShadow gets exactly what the sim wrapper
// (fcu/Fcu.cpp) gives it: the inputs of that step and the generated outputs. Command line and self-test:
// src/shadow_unit_test.h.

#include <memory>

#include "FcuComputer.h"
#include "FcuOutputLeaves.gen.h"
#include "FcuShadow.h"
#include "fcu_drive.h"
#include "fcu_leaves.h"
#include "fcu_scenarios.h"
#include "readable/fcu/fcu.h"
#include "shadow_unit_test.h"

namespace {

using namespace shadow_unit_test;

std::vector<VisitedLeaf> visitedLeaves() {
  std::vector<VisitedLeaf> out;
  for (const FcuLeaf& l : fcuLeaves<fcu_outputs>()) {
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
  auto generated = std::make_unique<FcuComputer>();
  generated->initialize();
  auto ext = std::make_unique<FcuComputer::ExternalInputs_FcuComputer_T>();
  auto shadow = std::make_unique<FcuShadow>();
  auto log = FcuShadow::makeLog("");
  log->setPath(logPath);
  std::unique_ptr<fcu::readable::FcuComputer> bare;
  if (opt.timing) {
    bare = std::make_unique<fcu::readable::FcuComputer>();
    bare->initialize();
  }
  FcuInputDrive::Options dopt;
  dopt.synthetic = opt.synthetic;
  FcuInputDrive drive(path, dopt);
  if (!drive.ok()) {
    res.error = drive.error();
    return res;
  }
  res.version = drive.version();
  res.recordedInputs = drive.recordedInputs();
  std::unique_ptr<fcu_scenarios::PowerScenario> power;
  if (opt.powerSeed >= 0) {
    power = std::make_unique<fcu_scenarios::PowerScenario>(static_cast<uint64_t>(opt.powerSeed));
  }
  auto outCopy = std::make_unique<fcu_outputs>();
  shadow::ShadowChecker* units[1] = {&shadow->checker()};

  auto step = [&](fcu_inputs& in, const FcuInputDrive::StepInfo&) {
    if (power) {
      power->apply(in, in.time.dt);
    }
    res.steps++;
    ext->in = in;
    const auto t0 = Clock::now();
    generated->setExternalInputs(ext.get());
    generated->step();
    const auto t1 = Clock::now();
    res.genSeconds += std::chrono::duration<double>(t1 - t0).count();
    if (opt.noShadow) {
      return;
    }
    const fcu_outputs& out = generated->getExternalOutputs().out;
    const uint64_t frame = shadow->checker().frames() + 1;
    if (keepRunning) {
      res.running.push_back(in.sim_data.computer_running);
    }
    if (frame == p.frame && (p.flipLeaf >= 0 || p.perturbInput)) {
      *outCopy = out;
      fcu_inputs inCopy = in;
      if (p.flipLeaf >= 0) {
        reinterpret_cast<uint8_t*>(outCopy.get())[fcu_shadow::kOutputLeaves[p.flipLeaf].offset] ^= 1;
      }
      if (p.perturbInput) {
        inCopy.discrete_inputs.afs_inputs.alt_knob.turns += 3;
      }
      shadow->step(inCopy, *outCopy);
      return;
    }
    if (opt.timing) {
      const auto t2 = Clock::now();
      shadow->step(in, out);
      const auto t3 = Clock::now();
      bare->step(in);
      res.shadowSeconds += std::chrono::duration<double>(t3 - t2).count();
      res.bareSeconds += std::chrono::duration<double>(Clock::now() - t3).count();
    } else {
      shadow->step(in, out);
    }
  };

  const std::string fdrName = baseName(path);
  long record = 0;
  while (drive.next(step)) {
    // once per recorded frame, as the sim does after its flight data recorder update (gap-fill steps are reported
    // with the next recorded frame)
    log->update(units, 1, fdrName, record++);
  }
  log->writeSummary(units, 1);
  res.wallSeconds = std::chrono::duration<double>(Clock::now() - start).count();
  res.units.push_back(unitResult(shadow->checker()));
  res.ok = true;
  return res;
}

}  // namespace

int main(int argc, char** argv) {
  const shadow_unit_test::Computer fcu = {
      "FCU",
      "fcu-shadow-test",
      fcu_shadow::kOutputLeaves,
      fcu_shadow::kOutputLeafCount,
      sizeof(fcu_outputs),
      1,
      true,
      {"bus_outputs.selected_alt_ft.Data", "discrete_outputs.afs_outputs.ap_1_light_on", "logic.capt_efis.efis_filter",
       "bus_outputs.fcu_discrete_word_1.SSM"},
      "discrete_inputs.afs_inputs.alt_knob.turns + 3",
      "data.discrete_inputs.afs_inputs.alt_knob.turns",
      visitedLeaves,
      run,
  };
  return shadow_unit_test::main(fcu, argc, argv);
}
