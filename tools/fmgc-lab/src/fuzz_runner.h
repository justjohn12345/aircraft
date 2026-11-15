// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Runs one fuzz case (src/fuzz_case.h) against the generated FMGC from a bank snapshot (src/fuzz_bank.h), calling the
// oracles (src/fuzz_oracle.h) every step. Shared by the libFuzzer harness, the standalone runner/minimiser, the seed
// generator and the coverage runner, so every tool runs exactly the same steps for the same bytes.

#pragma once

#include "fuzz_oracle.h"

#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "fuzz_bank.h"
#include "fuzz_case.h"

namespace fmgc::fuzz {

class FuzzRunner {
 public:
  FuzzRunner(const Bank& bank, OracleSet& oracles) : bank_(bank), oracles_(oracles) {
    computer_ = std::make_unique<FmgcComputer>();
    inputs_ = std::make_unique<FmgcComputer::ExternalInputs_FmgcComputer_T>();
    pre_ = std::make_unique<DWork>();
  }

  /** Optional per-step observer (after the oracles): step, inputs, outputs, running. */
  std::function<void(uint32_t, const fmgc_inputs&, const fmgc_outputs&, bool)> observer;

  FmgcComputer& computer() { return *computer_; }

  /** Runs the case; returns the number of steps run. */
  uint32_t run(const FuzzCase& c, const FindingSink& sink) {
    uint32_t snap = c.snapshot;
    const BankSnapshot& s = bank_.snapshot(snap);
    bank_.restore(snap, *computer_);
    oracles_.begin(bank_.oracleState(snap), bank_.header().oracleSize);
    const fmgc_inputs* frames = bank_.frames(snap);
    uint32_t n = s.frames;
    uint32_t first = static_cast<uint32_t>((static_cast<uint64_t>(c.start) * n) / 256);
    EventApplier events(c);
    fmgc_inputs& in = inputs_->in;
    double simTime = frames[first].time.simulation_time - frames[first].time.dt;
    double monoTime = frames[first].time.monotonic_time - frames[first].time.dt;
    bool needPre = oracles_.needsPreStepState();
    for (uint32_t step = 0; step < c.steps; step++) {
      // base inputs: timeScale window frames per step, dt summed; past the window end, the last frame held
      uint64_t idx = first + static_cast<uint64_t>(step) * c.timeScale;
      double dt = 0;
      for (uint32_t k = 0; k < c.timeScale; k++) {
        uint64_t j = idx + k;
        const fmgc_inputs& fr = frames[j < n ? j : n - 1];
        dt += fr.time.dt;
        if (k + 1 == c.timeScale) {
          in = fr;
        }
      }
      in.time.dt = dt;
      events.apply(step, in, computer_->getExternalOutputs().out);
      simTime += in.time.dt;
      monoTime += in.time.dt;
      in.time.simulation_time = simTime;
      in.time.monotonic_time = monoTime;
      if (needPre) {
        std::memcpy(static_cast<void*>(pre_.get()), &computer_->FmgcComputer_DWork, sizeof(DWork));
      }
      computer_->setExternalInputs(inputs_.get());
      computer_->step();
      const fmgc_outputs& out = computer_->getExternalOutputs().out;
      bool running = in.sim_data.computer_running != 0;
      FrameContext f{static_cast<long>(step), in, out, running, needPre ? pre_.get() : nullptr, *computer_};
      oracles_.onFrame(f, sink);
      if (observer) {
        observer(step, in, out, running);
      }
    }
    return c.steps;
  }

 private:
  const Bank& bank_;
  OracleSet& oracles_;
  std::unique_ptr<FmgcComputer> computer_;
  std::unique_ptr<FmgcComputer::ExternalInputs_FmgcComputer_T> inputs_;
  std::unique_ptr<DWork> pre_;
};

}  // namespace fmgc::fuzz
