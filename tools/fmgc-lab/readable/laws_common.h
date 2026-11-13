// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Pieces shared by several vertical law groups.

#pragma once

#include "blocks.h"

namespace fmgc::readable::laws {

/**
 * Filters of one speed-on-pitch instance (the SPD/MACH and SRS speed laws, and the VLS / VMAX protection commands of
 * the path laws): a complementary filter that estimates the along-track acceleration from the ground speed rate and
 * the inertial acceleration (high-passed) and the airspeed rate (low-passed).
 */
struct SpeedOnPitchFilters {
  blocks::WashoutFilter groundSpeedRate;        // d(ground speed)/dt, lagged
  blocks::LeadLagFilter inertialAccelHighPass;  // inertial along-path acceleration, high-passed (3s / (3s + 1))
  blocks::LeadLagFilter airspeedRate;           // d(IAS)/dt, low-passed (s / (3s + 1))

  void reset() {
    groundSpeedRate.reset();
    inertialAccelHighPass.reset();
    airspeedRate.reset();
  }
};

}  // namespace fmgc::readable::laws
