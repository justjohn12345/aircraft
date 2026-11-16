// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// One EFIS control panel (captain or first officer; generated FcuComputer.cpp 1095-1183, blocks 118-310): the FD and
// LS pushbuttons, the ND filter pushbuttons (CSTR, WPT, VOR.D, NDB, ARPT), and the baro reference (STD/QNH/QFE) and
// setting (hPa/inHg). The two panels are identical instances; they share only the FD auto on/off signals (from FMGC 1)
// and the QFE pin programming. The ND mode, range and navaid selectors are not processed here: they are packed raw
// into the EIS discrete words (output_packing.cpp).

#pragma once

#include "FcuComputer_types.h"
#include "fcu_blocks.h"

namespace fcu::readable {

/** Baro setting limits and conversions (generated FcuComputer_MATLABFunction1_o). */
constexpr float kMinBaroHpa = 745.0F, kMaxBaroHpa = 1100.0F;
constexpr float kMinBaroInHg = 22.0F, kMaxBaroInHg = 32.48F;
constexpr float kInHgPerHpa = 0.02953F, kHpaPerInHg = 33.8638687F;
constexpr float kStandardHpa = 1013.0F, kStandardInHg = 29.92F;

class EfisControlPanel {
 public:
  /** Power-up / restart state: FD on, LS off, no filter, QNH (not STD), 1013 hPa / 29.92 inHg. */
  void reset();

  /**
   * One frame of the panel.
   * - ND filters (EFISFilterLogic, 123-144): a push (rising edge) on the lit filter clears it; otherwise the pushed
   *   filter replaces the current one (one at a time; with several pushes in one frame CSTR > WPT > VOR.D > NDB > ARPT).
   * - FD (230-242): FD auto activate wins, then auto deactivate, then the FD button toggles. The FD button input is
   *   NOT edge-detected: a button held for n frames toggles n times (the sim sends one-frame presses).
   * - LS: toggles on each push (rising edge, T flip-flop starting off).
   * - Baro reference (158-180): a push (rising edge) in STD returns to the previous QNH/QFE, otherwise it swaps QNH and
   *   QFE; a pull goes to STD. Without the QFE pin programming QNH is forced (the sim never sets the pin: QFE is
   *   unreachable). Push wins over pull in the same frame.
   * - Baro setting (194-218): a value from the sim (hPa, -1 = none) sets both units (inHg unrounded: hPa * 0.02953). Not
   *   in STD, the knob turns change the unit shown (hPa: +1 per click within 745-1100, inHg recomputed rounded to
   *   0.01; inHg: +0.01 per click within 22.00-32.48, hPa recomputed rounded to 1). In STD the knob does nothing.
   */
  base_fcu_efis_logic_outputs update(const base_fcu_efis_panel_inputs& panel, bool fdAutoActivate,
                                     bool fdAutoDeactivate, bool qfeAvailable, float simBaroHpa);

  /** The panel lights and the PFD baro display (generated FcuComputer_MATLABFunction_n, 275-310, and 1406-1429). */
  static base_fcu_efis_panel_outputs displayOutputs(const base_fcu_efis_logic_outputs& logic,
                                                    const base_fcu_efis_panel_inputs& panel);

  // state (for the checker)
  efis_filter_selection filter() const { return filter_; }
  bool fdOn() const { return fdOn_; }
  bool lsOn() const { return ls_.q; }
  bool stdActive() const { return std_; }
  bool qnhActive() const { return qnh_; }
  bool qfeActive() const { return qfe_; }
  float baroHpa() const { return hpa_; }
  float baroInHg() const { return inHg_; }

 private:
  blocks::EdgeDetector cstrPb_, wptPb_, vordPb_, ndbPb_, arptPb_, lsPb_, baroPush_, baroPull_;
  efis_filter_selection filter_ = efis_filter_selection::NONE;  // pEfisFilter
  bool fdOn_ = true;                                            // fdActive
  blocks::ToggleFlipFlop ls_;                                   // TFlipFlop1 (init false)
  bool std_ = false, qnh_ = true, qfe_ = false;                 // std_active, qnh_active, qfe_active
  float hpa_ = kStandardHpa, inHg_ = kStandardInHg;             // pValueHpa, pValueInhg
};

}  // namespace fcu::readable
