// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Fills ApproachExternal from the generated model (included by check_vertical.cpp, which gives access to the model's
// private state). pre/post: the generated block memories before and after this step.

#pragma once

inline void fillApproachExternal(fmgc::readable::ApproachExternal& e, const fmgc_outputs& /*out*/,
                                 const FmgcComputer::D_Work_FmgcComputer_T& pre,
                                 const FmgcComputer::D_Work_FmgcComputer_T& /*post*/) {
  // The mode logic reads the outer loops' output of the previous step (Delay2_DSTATE, written at the end of step()).
  // On a (re)start the generated step first sets it to its initial condition.
  const ap_raw_output& lastOuterLoops =
      pre.Runtime_MODE ? pre.Delay2_DSTATE : FmgcComputer::FmgcComputer_P.Delay2_InitialCondition;
  e.lastFlareCondition = lastOuterLoops.flare_law.condition_Flare;
}
