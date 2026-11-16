// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SPD/MACH window (generated FcuComputer.cpp 738-829). See windows.h.

#include <cmath>

#include "windows.h"

namespace fcu::readable {

void SpeedMachWindow::reset() {
  selectedSpeedLost_.reset();
  dashTimer_.reset();
  memoryPrimed_ = false;
  previousMachPrimed_ = false;
}

SpeedMachWindow::Output SpeedMachWindow::update(const fcu_inputs& in, const FmgcView& fmgc) {
  const auto& knob = in.discrete_inputs.afs_inputs.spd_knob;
  const float simValue = in.sim_input.spd_mach;
  const bool mach = fmgc.machActive;
  const float currentSpeed = mach ? fmgc.data.v_mach : fmgc.data.v_cas_kts;

  // 738-753: dashes
  const bool leftSelectedSpeed = selectedSpeedLost_.update(fmgc.selectedSpeedControl, /*risingEdge*/ false);
  const bool selection = fmgc.selectedSpeedControl || knob.turns != 0 || simValue != kNoSimInput;
  Output out;
  out.dashes = dashTimer_.update(in.time.simulation_time, leftSelectedSpeed, selection, kSpdMachDashDelaySeconds);

  // 775-829: memory
  if (!memoryPrimed_) {
    memory_ = currentSpeed;
    memoryPrimed_ = true;
  }
  if (!previousMachPrimed_) {
    previousMach_ = mach;
    previousMachPrimed_ = true;
  }
  if (simValue != kNoSimInput) {
    memory_ = mach ? simValue / 100.0F : simValue;  // the sim sends a Mach x100
  }
  if (previousMach_ != mach) {
    memory_ = currentSpeed;
  }
  if (out.dashes) {
    memory_ = currentSpeed;
  }
  if (fmgc.presetSpeedOrMach > 0.0F) {
    memory_ = fmgc.presetSpeedOrMach;
  }
  if (mach) {
    memory_ += static_cast<float>(knob.turns) * 0.01F;
    memory_ = std::fmax(std::fmin(memory_, kMaxMach), kMinMach);
    memory_ = std::round(memory_ / kMachResolution) * kMachResolution;
  } else {
    memory_ += static_cast<float>(knob.turns);
    memory_ = std::fmax(std::fmin(memory_, kMaxSpeedKts), kMinSpeedKts);
    memory_ = std::round(memory_);
  }
  previousMach_ = mach;

  out.value = memory_;
  return out;
}

}  // namespace fcu::readable
