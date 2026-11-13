// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Fills ClimbDescentExternal from the generated model (included by check_vertical.cpp, which gives access to the model's
// private state). pre/post: the generated block memories before and after this step.

#pragma once

inline void fillClimbDescentExternal(fmgc::readable::ClimbDescentExternal& e, const fmgc_outputs& /*out*/,
                                     const FmgcComputer::D_Work_FmgcComputer_T& /*pre*/,
                                     const FmgcComputer::D_Work_FmgcComputer_T& post) {
  // 5 s "in flight" confirmation (FmgcComputer.cpp 1276–1277; no output field): the ConfirmNode's output after this
  // step.
  e.inFlightFor5s = post.sf_MATLABFunction_fw.output;
}
