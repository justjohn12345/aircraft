// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Native (and wasm) driver and test of the sim's FADEC shadow checker (fbw-a32nx/src/wasm/fbw_a320/src/shadow/
// FadecShadow.{h,cpp} and ShadowChecker.{h,cpp}), the same sources the wasm module compiles. Both generated
// FadecComputers (engine 1 and 2) are stepped with the FADEC inputs of a recording (src/fadec_drive.h: the recorded model
// inputs of a 3209004 file, else the synthetic drive) and, after every step, each FadecShadow gets exactly what the sim
// wrapper (FlyByWireInterface::updateFadec) gives it: the inputs of that step and the generated outputs. As in the sim,
// the generated FADECs are only constructed, never initialize()d. Command line and self-test: src/shadow_unit_test.h.

#include <memory>

#include "FadecComputer.h"
#include "FadecOutputLeaves.gen.h"
#include "FadecShadow.h"
#include "fadec_drive.h"
#include "fadec_fields.gen.h"
#define LEAVES_FIELDS_INCLUDED
#include "fcu_leaves.h"
#include "readable/fadec/fadec.h"
#include "shadow_unit_test.h"

namespace {

using namespace shadow_unit_test;

std::vector<VisitedLeaf> visitedLeaves() {
  std::vector<VisitedLeaf> out;
  for (const FcuLeaf& l : fcuLeaves<athr_out>()) {
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
  std::unique_ptr<FadecComputer> generated[2] = {std::make_unique<FadecComputer>(), std::make_unique<FadecComputer>()};
  auto ext = std::make_unique<FadecComputer::ExternalInputs_FadecComputer_T>();
  std::unique_ptr<FadecShadow> shadows[2] = {std::make_unique<FadecShadow>(1), std::make_unique<FadecShadow>(2)};
  auto log = FadecShadow::makeLog("");
  log->setPath(logPath);
  std::unique_ptr<fadec::readable::FadecComputer> bare[2];
  if (opt.timing) {
    for (auto& b : bare) {
      b = std::make_unique<fadec::readable::FadecComputer>();
      b->initialize();
    }
  }
  FadecInputDrive::Options dopt;
  dopt.synthetic = opt.synthetic;
  FadecInputDrive drive(path, dopt);
  if (!drive.ok()) {
    res.error = drive.error();
    return res;
  }
  res.version = drive.version();
  res.recordedInputs = drive.recordedInputs();
  for (int e = 0; e < 2; e++) {
    drive.feedback[e] = &generated[e]->getExternalOutputs().out;
  }
  auto outCopy = std::make_unique<athr_out>();
  shadow::ShadowChecker* units[2] = {&shadows[0]->checker(), &shadows[1]->checker()};

  auto step = [&](athr_in(&in)[2], const FadecInputDrive::StepInfo&) {
    res.steps++;
    if (keepRunning) {
      res.running.push_back(true);  // the FADEC has no computer_running
    }
    for (int e = 0; e < 2; e++) {
      ext->in = in[e];
      const auto t0 = Clock::now();
      generated[e]->setExternalInputs(ext.get());
      generated[e]->step();
      const auto t1 = Clock::now();
      res.genSeconds += std::chrono::duration<double>(t1 - t0).count();
      if (opt.noShadow) {
        continue;
      }
      const athr_out& out = generated[e]->getExternalOutputs().out;
      const uint64_t frame = shadows[e]->checker().frames() + 1;
      if (e == 0 && frame == p.frame && (p.flipLeaf >= 0 || p.perturbInput)) {
        *outCopy = out;
        athr_in inCopy = in[e];
        if (p.flipLeaf >= 0) {
          reinterpret_cast<uint8_t*>(outCopy.get())[fadec_shadow::kOutputLeaves[p.flipLeaf].offset] ^= 1;
        }
        if (p.perturbInput) {
          inCopy.time.dt *= 1.5;
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

}  // namespace

int main(int argc, char** argv) {
  const shadow_unit_test::Computer fadec = {
      "FADEC",
      "fadec-shadow-test",
      fadec_shadow::kOutputLeaves,
      fadec_shadow::kOutputLeafCount,
      sizeof(athr_out),
      2,
      false,
      {"output.N1_c_percent", "output.athr_control_active", "output.thrust_limit_type", "fadec_bus_output.ecu_status_word_3.SSM"},
      "time.dt x 1.5",
      "time.dt",
      visitedLeaves,
      run,
  };
  return shadow_unit_test::main(fadec, argc, argv);
}
