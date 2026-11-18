// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The scripted tour of the ELAC differential check (src/check_elac.cpp --script SEED), applied on top of the drive
// (src/efcs_drive.h) before the generic injectors (src/efcs_scenarios.h). It cycles through phases of a few seconds each
// (the seed picks the first one and jitters values); every phase overrides a consistent set of inputs on both ELACs
// to reach a function the recordings do not exercise.

#pragma once

#include <cmath>
#include <cstdint>

#include "ElacComputer_types.h"
#include "efcs_scenarios.h"

namespace elac_scenarios {

using efcs_scenarios::Rng;

constexpr uint32_t kFw = 0, kNcd = 1, kNo = 3;

inline void setBit(base_arinc_429& w, int bit, bool v) {
  float r = std::round(w.Data);
  uint32_t x = r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
  x = v ? (x | (1U << (bit - 1))) : (x & ~(1U << (bit - 1)));
  w.Data = static_cast<float>(x);
}

inline base_arinc_429 no(double v) { return {kNo, static_cast<float>(v)}; }

class ScriptScenario {
 public:
  explicit ScriptScenario(uint64_t seed) : rng_(seed), phase_(static_cast<int>(seed % kPhases)) {}
  long episodes() const { return episodes_; }

  void apply(elac_inputs (&in)[2], double dt) {
    const double step = std::isfinite(dt) && dt > 0 ? dt : 0.02;
    t_ += step;
    if (t_ >= duration_) {
      t_ = 0;
      phase_ = (phase_ + 1) % kPhases;
      duration_ = rng_.u(4.0, 40.0);
      jitter_ = rng_.u(-1, 1);
      variant_ = rng_.irand(8);
      episodes_++;
    }
    for (auto& i : in) applyPhase(i);
  }

 private:
  static constexpr int kPhases = 1;

  void applyPhase(elac_inputs&) {}

  Rng rng_;
  int phase_;
  double t_ = 0, duration_ = 10.0, jitter_ = 0;
  int variant_ = 0;
  long episodes_ = 0;
};

}  // namespace elac_scenarios
