// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "athr.h"

namespace fmgc::readable::athr {

const char* fieldName(int field) {
  static const char* names[kFieldCount] = {"athr_active",        "athr_limited",           "alpha_floor_mode_active",
                                           "thrust_mode_active", "thrust_target_idle",     "speed_mach_mode_active",
                                           "retard_mode_active", "fma_mode",               "fma_message",
                                           "n1_c_percent"};
  return field >= 0 && field < kFieldCount ? names[field] : "?";
}

void Autothrust::reset() {
  modes_ = ModesState{};
  n1_ = N1State{};
}

}  // namespace fmgc::readable::athr
