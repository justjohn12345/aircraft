// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC outer-loop input assembly (Phase 5 integration): the arguments FmgcComputer passes to
// FmgcOuterLoops::step() (generated FmgcComputer.cpp 4055–4170), as the ap_laws_output bus the readable laws take
// (readable/laws.h, laws_lateral.h; the field order is the step() argument order, src/lab_hooks.h /
// outer_io.gen.h). Checked through src/check_fmgc.cpp (the outer-loop outputs depend on every field).
//
// Units are the outer loops' (m/s², kg); body accelerations are converted from g with 9.81 (normal acceleration minus
// 1 g), the weight from lb. Two memories, block outputs of the generated model (FmgcComputer_B) that only initialize()
// resets, so they survive a computer restart:
//   - the ILS DME distance (fms_loc_distance), frozen while the ILS tune is inhibited [FmgcComputer_B.u];
//   - the IR magnetic track, frozen in GA TRK and RWY TRK: the track those modes hold [FmgcComputer_B.u_lyjjl].

#pragma once

#include "FmgcComputer_types.h"
#include "FmgcOuterLoops_types.h"

namespace fmgc::readable {

struct OuterLoopInputState {
  double locDistanceMemoNmi = 0;  // [FmgcComputer_B.u]
  float trackMemoDeg = 0;         // [FmgcComputer_B.u_lyjjl]
};

/**
 * Fills io.time, io.data and io.input (io.output untouched) from this frame's inputs, logic bus and complete
 * ap_fd_logic bus (modes, laws, targets, TCAS V/S target).
 */
void assembleOuterLoopInputs(OuterLoopInputState& st, const fmgc_inputs& in, const base_fmgc_logic_outputs& logic,
                             const base_fmgc_ap_fd_logic_outputs& apFd, ap_laws_output& io);

}  // namespace fmgc::readable
