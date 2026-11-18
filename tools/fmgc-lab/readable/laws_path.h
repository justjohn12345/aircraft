// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// G/S, VPATH (FINAL and DES on path) and FLARE laws (laws_path.cpp): block memories.

#pragma once

#include "blocks.h"

namespace fmgc::readable::laws {

/** G/S law (vertical_law 6). Generated: FmgcOuterLoops.cpp 3474–3556. */
struct GsLawState {
  // track: beam deviation and inertial vertical acceleration
  blocks::LagFilter<double> deviationLag;     // G/S deviation, 10 rad/s (sf_LagFilter_f5)
  double prevVsFtS = 0;                       // V/S of the previous step, ft/s (Delay_DSTATE_i)
  blocks::LagFilter<double> vsRateLag;        // d(V/S)/dt, 10 rad/s (sf_LagFilter_c)
  blocks::LagFilter<double> pitchRateAccelLag;  // V_gnd·tan(q), 1 rad/s (sf_LagFilter_ft)
  blocks::LagFilter<double> accelLag;         // d(V/S)/dt − V·q, 4 rad/s (sf_LagFilter_l)
  blocks::WashoutFilter accelWashout;         // ... washed out at 10 rad/s (sf_WashoutFilter_bd)
  blocks::SwitchBlend trackBlend;             // capture -> track fade (sf_RateLimiter_j)
  // capture: lagged deviation plus its rate
  blocks::LagFilter<double> captureDeviationLag;  // G/S deviation, 2 rad/s (sf_LagFilter_fd)
  double prevCaptureDeviation = 0;                // (Delay_DSTATE_l)
  blocks::LagFilter<double> captureLag;           // deviation + 14 s·rate, 1 rad/s (sf_LagFilter_k)
  // beam angle memorised while G/S is not the active law (storage)
  double beamAngleDeg = 0;
  bool beamAnglePrimed = false;
};

/** VPATH law (vertical_law 9). Generated: FmgcOuterLoops.cpp 4070–4094. */
struct VpathLawState {
  blocks::LagFilter<double> altitudeErrorLag;  // FMS profile altitude − H, 1 rad/s (sf_LagFilter_ag)
};

/** FLARE law (vertical_law 7) and flare law outputs. Generated: FmgcOuterLoops.cpp 3108–3186, 4310–4319, 4560–4565. */
struct FlareLawState {
  blocks::WashoutFilter bxWashout;            // longitudinal acceleration, 50 rad/s (sf_WashoutFilter_b)
  blocks::LagFilter<double> vsLag;            // V/S, 1/30 rad/s (sf_LagFilter_or)
  blocks::LeadLagFilter radioRate;            // 15 s/(s + 15) on RA: radio height rate, ft/s (pU, pY_l)
  blocks::LagFilter<double> rateCorrectionLag;  // fix: radio rate - V/S, 1 rad/s (sf_LagFilter_flareRate)
  bool wasActive = false;                     // FLARE law active on the previous step (wasActive)
  bool wasActivePrimed = false;
  double tau = 1.0;                           // flare time constant, s (Tau)
  double heightBias = 0.0;                    // flare height bias, ft (H_bias)
  blocks::LeadLagFilter vsCommandRate;        // 15 s/(s + 15) on the V/S command (sf_LeadLagFilter_i)
  blocks::WashoutFilter betaWashout;          // AP sideslip command, 6 rad/s (sf_WashoutFilter_l)
  blocks::LagFilter<double> conditionRateLag; // radio height rate for the flare condition, 4 rad/s (sf_LagFilter_d)
};

struct PathLawState {
  GsLawState gs;
  VpathLawState vpath;
  FlareLawState flare;
};

}  // namespace fmgc::readable::laws
