// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Hooks compiled into the instrumented copy of the generated model (scripts/gen_hooked_model.py): the FmgcComputer's
// calls to its outer loops go through fmgcLabOuterLoopsStep/Reset, and probe statements in FmgcOuterLoops.cpp report
// intermediate values. The checker (src/check_laws.cpp) reads them through labHooks().

#pragma once

#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

struct LabHooks {
  // Every FmgcOuterLoops::step() argument of the last call, in parameter order, before and after the call (the call
  // site passes plain pointers, so inputs and outputs are told apart by parameter position: see outer_io.gen.h).
  std::vector<double> outerIn;
  std::vector<double> outerOut;
  std::unordered_map<std::string, double> probes;  // last value of each probe
  std::function<void()> onOuterStep;   // called after each FmgcOuterLoops::step()
  std::function<void()> onOuterReset;  // called after each FmgcOuterLoops::reset()
};

inline LabHooks& labHooks() {
  static LabHooks hooks;
  return hooks;
}

inline void fmgcLabProbe(const char* name, double value) {
  labHooks().probes[name] = value;
}

namespace lab_detail {
template <typename P>
void captureIn(LabHooks& h, P p) {
  h.outerIn.push_back(static_cast<double>(*p));
}
template <typename P>
void captureOut(LabHooks& h, P p) {
  h.outerOut.push_back(static_cast<double>(*p));
}
}  // namespace lab_detail

template <typename Model, typename... Args>
void fmgcLabOuterLoopsStep(Model& model, Args... args) {
  LabHooks& h = labHooks();
  h.outerIn.clear();
  h.outerOut.clear();
  (lab_detail::captureIn(h, args), ...);
  model.step(args...);
  (lab_detail::captureOut(h, args), ...);
  if (h.onOuterStep) {
    h.onOuterStep();
  }
}

template <typename Model>
void fmgcLabOuterLoopsReset(Model& model) {
  model.reset();
  if (labHooks().onOuterReset) {
    labHooks().onOuterReset();
  }
}
