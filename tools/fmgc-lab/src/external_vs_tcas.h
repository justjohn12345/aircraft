// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Fills VsTcasExternal from the generated model (included by check_vertical.cpp, which gives access to the model's
// private state). pre/post: the generated block memories before and after this step.

#pragma once

inline void fillVsTcasExternal(fmgc::readable::VsTcasExternal& e, const fmgc_outputs& out,
                               const FmgcComputer::D_Work_FmgcComputer_T& /*pre*/,
                               const FmgcComputer::D_Work_FmgcComputer_T& /*post*/) {
  e.altCaptureCond = out.ap_fd_logic.tcas_alt_acq_cond;
  e.altHoldCond = out.ap_fd_logic.tcas_alt_hold_cond;
  e.speedTargetKt = out.ap_fd_logic.spd_target_kts;
  e.machMode = out.ap_fd_logic.mach_control_active;
}
