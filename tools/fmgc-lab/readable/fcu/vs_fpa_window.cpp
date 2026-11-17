// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// V/S-FPA window (generated FcuComputer.cpp 955-1060). See windows.h.

#include <cmath>

#include "windows.h"

namespace fcu::readable {

void VsFpaWindow::reset() {
  leftVsFpaWithApFd_.reset();
  dashTimer_.reset();
  recentVsPush_.reset();
  levelOffPush_.reset();
  recentTcasEnd_.reset();
  vsFpaAfterTcas_.reset();
  apEngagement_.reset();
  memoryPrimed_ = false;
  previousTrkFpaPrimed_ = false;
}

VsFpaWindow::Output VsFpaWindow::update(const fcu_inputs& in, const FmgcView& fmgc, bool anyApFdEngaged,
                                        bool trkFpaActive, bool vsKnobPushPulse, float fcuAltFt) {
  const auto& d = in.discrete_inputs;
  const auto& knob = d.afs_inputs.vs_fpa_knob;
  const float simValue = in.sim_input.vs_fpa;
  const double dt = in.time.dt;
  const bool vsFpaMode = fmgc.lvlChVsFpa;
  const float currentVsFpa = trkFpaActive ? fmgc.data.fpa_deg : fmgc.data.vs_ft_min;

  // 955-962: dashes
  const bool leftVsFpa = leftVsFpaWithApFd_.update(!vsFpaMode && anyApFdEngaged, /*risingEdge*/ true);
  const bool selection = vsFpaMode || !anyApFdEngaged || knob.turns != 0 || simValue != kNoSimInput;
  Output out;
  out.dashes = dashTimer_.update(in.time.simulation_time, leftVsFpa, selection, kVsFpaDashDelaySeconds);

  // 963-985: level-off push, end of TCAS, AP engagement
  const bool vsPushRecently =
      recentVsPush_.update(vsKnobPushPulse, dt, /*risingEdge*/ true, /*retriggerable*/ false, kLevelOffPushWindowSeconds);
  const bool levelOff = levelOffPush_.update(vsFpaMode && vsPushRecently, /*risingEdge*/ true);
  const bool tcasEndedRecently =
      recentTcasEnd_.update(fmgc.tcasActive, dt, /*risingEdge*/ false, /*retriggerable*/ false, kTcasEndWindowSeconds);
  const bool vsFpaAfterTcas = vsFpaAfterTcas_.update(vsFpaMode && tcasEndedRecently, /*risingEdge*/ true);
  const bool apEngaged = apEngagement_.update(d.ap_1_engaged || d.ap_2_engaged, /*risingEdge*/ true);

  // 986-1060: memory
  if (!memoryPrimed_) {
    memory_ = currentVsFpa;
    memoryPrimed_ = true;
  }
  if (!previousTrkFpaPrimed_) {
    previousTrkFpa_ = trkFpaActive;
    previousTrkFpaPrimed_ = true;
  }
  if (previousTrkFpa_ != trkFpaActive) {
    memory_ = currentVsFpa;
  }
  if (simValue != kNoSimInput) {
    memory_ = trkFpaActive ? simValue / 10.0F : simValue;  // the sim sends an FPA x10
  }
  if (apEngaged || out.dashes) {
    memory_ = currentVsFpa;
  }
  if (levelOff) {
    memory_ = 0.0F;
  } else if (vsFpaAfterTcas && fmgc.tcasRaInhibited) {
    // V/S in ft/min even in TRK-FPA
    memory_ = fmgc.tcasVsTargetUp ? fmgc.data.vs_ft_min : 0.0F;
  } else if (vsFpaAfterTcas && fmgc.tcasRaCorrective) {
    const float toFcuAlt = fcuAltFt - fmgc.data.alt_ft;
    if (fmgc.data.alt_ft > kPostTcasHighAltFt && toFcuAlt > 0.0F) {
      memory_ = kPostTcasClimbVsHighFtMin;
    } else {
      const int direction = toFcuAlt < 0.0F ? -1 : (toFcuAlt > 0.0F ? 1 : 0);
      memory_ = kPostTcasVsFtMin * static_cast<float>(direction);
    }
  }
  if (trkFpaActive) {
    memory_ += static_cast<float>(knob.turns) * 0.1F;
    memory_ = std::fmax(std::fmin(memory_, kMaxFpaDeg), -kMaxFpaDeg);
    memory_ = std::round(memory_ / kFpaResolutionDeg) * kFpaResolutionDeg;
  } else {
    memory_ += static_cast<float>(knob.turns) * 100.0F;
    memory_ = std::fmax(std::fmin(memory_, kMaxVsFtMin), -kMaxVsFtMin);
    memory_ = std::round(memory_ / kVsResolutionFtMin) * kVsResolutionFtMin;
  }
  previousTrkFpa_ = trkFpaActive;

  out.value = memory_;
  return out;
}

}  // namespace fcu::readable
