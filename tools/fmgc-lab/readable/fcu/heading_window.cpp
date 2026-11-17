// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// HDG/TRK window (generated FcuComputer.cpp 861-933). See windows.h.

#include <cmath>

#include "windows.h"

namespace fcu::readable {

void HdgTrkWindow::reset() {
  managedWithApFd_.reset();
  managedNoPresetWithApFd_.reset();
  dashTimer_.reset();
  memoryPrimed_ = false;
  previousTrkFpaPrimed_ = false;
}

HdgTrkWindow::Output HdgTrkWindow::update(const fcu_inputs& in, const FmgcView& fmgc, bool anyApFdEngaged,
                                          bool trkFpaActive) {
  const auto& knob = in.discrete_inputs.afs_inputs.hdg_trk_knob;
  const float simValue = in.sim_input.hdg_trk;
  const float heading = fmgc.data.hdg_deg;
  const float track = fmgc.data.trk_deg;

  // 861-872: dashes
  const bool notSelected = !fmgc.hdgTrkSelected;
  const bool toManaged = managedWithApFd_.update(notSelected && anyApFdEngaged, /*risingEdge*/ true);
  const bool toManagedNoPreset =
      managedNoPresetWithApFd_.update(notSelected && !fmgc.hdgTrkPresetAvailable && anyApFdEngaged, true);
  const bool noApFd = !anyApFdEngaged;
  const bool selection = fmgc.hdgTrkSelected || noApFd || knob.turns != 0 || simValue != kNoSimInput ||
                         fmgc.hdgTrkPresetAvailable;
  Output out;
  out.dashes = dashTimer_.update(in.time.simulation_time, toManaged || toManagedNoPreset, selection,
                                 kHdgTrkDashDelaySeconds);

  // 873-931: memory
  if (!memoryPrimed_) {
    memory_ = trkFpaActive ? track : heading;
    memoryPrimed_ = true;
  }
  if (!previousTrkFpaPrimed_) {
    previousTrkFpa_ = trkFpaActive;
    previousTrkFpaPrimed_ = true;
  }
  if (simValue != kNoSimInput) {
    memory_ = simValue;
  }
  if (previousTrkFpa_ != trkFpaActive) {
    // distance of the target from the old reference: heading when switching to TRK, track when switching to HDG
    float offset = trkFpaActive ? memory_ - heading : memory_ - track;
    if (offset > 180.0F) {
      offset -= 360.0F;
    } else if (offset < -180.0F) {
      offset += 360.0F;
    }
    offset = std::abs(offset);
    if ((offset < kHdgTrkSyncWindowDeg || noApFd) && trkFpaActive) {
      memory_ = track;
    } else if ((offset < kHdgTrkSyncWindowDeg || noApFd) && !trkFpaActive) {
      memory_ = heading;
    }
  }
  if (out.dashes) {
    memory_ = std::round(trkFpaActive ? track : heading);
  }
  memory_ += static_cast<float>(knob.turns);
  memory_ = std::round(memory_);
  if (memory_ > 359.0F) {
    memory_ -= 360.0F;
  } else if (memory_ < 0.0F) {
    memory_ += 360.0F;
  }
  previousTrkFpa_ = trkFpaActive;

  out.value = memory_;
  return out;
}

}  // namespace fcu::readable
