// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Fills ApproachExternal from the generated model (included by check_vertical.cpp, which gives access to the model's
// private state). pre/post: the generated block memories before and after this step.

#pragma once

inline void fillApproachExternal(fmgc::readable::ApproachExternal&, const fmgc_outputs& /*out*/,
                               const FmgcComputer::D_Work_FmgcComputer_T& /*pre*/,
                               const FmgcComputer::D_Work_FmgcComputer_T& /*post*/) {}
