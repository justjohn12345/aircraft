// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SPD_MACH (speed on pitch: CLB, DES, OP CLB/DES, EXP) and SRS laws (laws_speed.cpp): block memories.

#pragma once

#include "blocks.h"

namespace fmgc::readable::laws {

/**
 * Filters of one speed-on-pitch instance: the acceleration estimate, a complementary filter of the inertial
 * acceleration (high-passed) and the airspeed rate (low-passed).
 */
struct SpeedOnPitchFilters {
  blocks::WashoutFilter groundSpeedRate;        // d/dt of the ground speed, lagged
  blocks::LeadLagFilter inertialAccelHighPass;  // 3s/(3s+1) of the inertial along-path acceleration
  blocks::LeadLagFilter airspeedRate;           // s/(3s+1) of IAS: lagged airspeed rate

  void reset() {
    groundSpeedRate.reset();
    inertialAccelHighPass.reset();
    airspeedRate.reset();
  }
};

struct SpeedLawState {
  SpeedOnPitchFilters spdMach;  // generated sf_WashoutFilter_h, sf_LeadLagFilter_c, sf_LeadLagFilter_oer
  SpeedOnPitchFilters srs;      // generated sf_WashoutFilter_eq, sf_LeadLagFilter_k, sf_LeadLagFilter_ay
};

}  // namespace fmgc::readable::laws
