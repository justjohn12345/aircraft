// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FCU outputs (generated FcuComputer.cpp 1184-1533): the panel lights and window values (discrete outputs) and the
// FCU bus to the FMGCs, EIS (DMCs) and FADECs. Stateless: everything is packed from this frame's logic.
//
// Discrete words carry their flags in ARINC bits 11-29 (1-based, as in docs/output-buses.md), SSM always normal
// operation; unlisted bits are spare (constant false in the generated code). The sim wrapper (fcu/Fcu.cpp) adds FCU 1/2
// healthy on FCU discrete word 2 bits 24/25 and zeroes the whole bus when the FCU is unhealthy.
//
// FCU discrete word 1                                    | FCU discrete word 2
//   11 SPD knob pushed      19 V/S knob pulled           |   11 HDG knob pushed         21 FMGC 2 has priority
//   12 SPD knob pulled      20 METRIC ALT                |   12 HDG knob pulled         (24 FCU 1 healthy: wrapper)
//   13 ALT value changed    21 SPD/MACH pb pushed        |   13 LOC pb pushed           (25 FCU 2 healthy: wrapper)
//   14 V/S-FPA value changed 22 EXPED pb pushed          |   14 HDG/TRK value changed   26 capt FD off
//   15 SPD/MACH value changed 23 APPR pb pushed          |   20 FMGC 1 has priority     27 F/O FD off
//   16 V/S knob pushed      24 HDG-V/S
//   17 ALT knob pushed      25 TRK-FPA
//   18 ALT knob pulled
// (the pushes/pulls/value changes are the 0.1 s / 0.5 s pulses of afs_controls.h and fcu.h)
//
// EIS discrete word 1 (left = captain panel, right = F/O)  | EIS discrete word 2 (left/right)
//   11 baro in inHg                                        |   11-15 ND mode PLAN, ARC, ROSE NAV, ROSE VOR, ROSE ILS
//   25-29 ND range 10, 20, 40, 80, 160 (320: no bit)       |   17-21 filter CSTR, WPT, VOR.D, NDB, ARPT
//                                                          |   22 LS on, 23 FD off
//                                                          |   24/25 navaid 1/2 ADF, 26/27 navaid 1/2 VOR
//                                                          |   28 baro STD, 29 baro QNH

#pragma once

#include "FcuComputer_types.h"
#include "fmgc_interface.h"

namespace fcu::readable {

/** Panel lights and window values (FcuComputer_Y.out.discrete_outputs). */
base_fcu_discrete_outputs packDiscreteOutputs(const fcu_inputs& in, const base_fcu_logic_outputs& logic,
                                              const base_fcu_efis_panel_outputs& captPanel,
                                              const base_fcu_efis_panel_outputs& foPanel);

/**
 * The FCU bus (FcuComputer_Y.out.bus_outputs). Selected values: HDG and TRK share the HDG/TRK memory, V/S and FPA the
 * V/S-FPA memory, each NCD when dashed or when the other unit is shown; SPD and MACH: NCD when dashed or in the other
 * unit, and the other unit's word carries the aircraft's current CAS/Mach from the priority FMGC. ALT is always normal.
 * The A/THR words (ATS, ATS FMA, flex temperature, N1 command) are the priority FMGC's, passed through.
 */
base_fcu_bus packBusOutputs(const fcu_inputs& in, const FmgcView& fmgc, const base_fcu_logic_outputs& logic);

}  // namespace fcu::readable
