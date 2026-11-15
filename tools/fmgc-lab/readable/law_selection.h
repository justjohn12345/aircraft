// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC longitudinal law selection (Phase 5 integration). Stock behaviour: must reproduce the
// generated FmgcComputer exactly (checked through src/check_fmgc.cpp). Generated FmgcComputer.cpp 3756–3792, table in
// docs/structure.md ("Law selection"). The lateral law selection is a mode signal (lateral_modes.cpp,
// Signal::ActiveLateralLaw).
//
// The law is chosen from this frame's modes whether or not an AP or FD is engaged, by priority (first match wins).
// No state.

#pragma once

#include "FmgcComputer_types.h"

namespace fmgc::readable {

/** active_longitudinal_law from this frame's longitudinal modes (and LAND), the TCAS submode and the FMS DES submode. */
vertical_law selectLongitudinalLaw(const base_fmgc_ap_fd_logic_outputs& modes, fmgc_des_submode requestedDesSubmode);

}  // namespace fmgc::readable
