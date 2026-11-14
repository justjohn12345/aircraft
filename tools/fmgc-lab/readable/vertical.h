// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC vertical mode logic (Phase 2). Stock behaviour: this must reproduce the generated
// FmgcComputer exactly (checked frame by frame by src/check_vertical.cpp); fixes belong on personal/fmgc-fixes.
//
// Each vertical mode, armed mode or derived flag is a Signal. Every frame the signals are updated one by one in the
// order the generated step() computes them (the Signal enum order), because a condition may read a signal that was
// already updated this frame (`s[...]`) or the previous frame's outputs (`f.last`). Latches use blocks::SrLatch
// semantics (reset wins). The parts of the FMGC not rewritten yet (input conditioning, AP/FD engagement, TCAS
// computations) come in through External, filled from the generated model. The lateral modes (lateral_modes.cpp) are
// signals too, interleaved with the vertical ones in the generated update order.

#pragma once

#include <array>
#include <cstddef>

#include "FmgcComputer_types.h"
#include "blocks.h"
#include "lateral_modes.h"
#include "vertical_alt.h"
#include "vertical_approach.h"
#include "vertical_climb_descent.h"
#include "vertical_vs_tcas.h"

namespace fmgc::readable {

/** In the order the generated step() updates them (FmgcComputer.cpp line in brackets). */
enum class Signal : int {
  AltCstrApplicable,  // [1659] alt_cstr_applicable: the FMS altitude constraint is the altitude target (combinational)
  AltSelOrCstr,     // [1664] alt_sel_or_cstr (value in Signals::altSelOrCstr; the bool is target != 0)
  SrsTakeoff,       // [1696] longitudinal_modes.pitch_takeoff_active
  RwyLoc,           // [1754] lateral_modes.rwy_loc_submode_active (RWY)
  RwyTrk,           // [1782] lateral_modes.rwy_trk_submode_active (RWY TRK)
  RwyActive,        // [1793] lateral_modes.rwy_active (combinational: RWY or RWY TRK)
  LandArmed,        // [1839] armed_modes.land_armed
  Land,             // [1872] lateral_modes.land_active
  RollOut,          // [1906] lateral_modes.rollout_submode_active
  Align,            // [1910] lateral_modes.align_submode_active (combinational)
  FlareMemory,      // [1917] FLARE memory latch (no output)
  FinalArmed,       // [1957] armed_modes.final_des_armed
  Final,            // [1986] longitudinal_modes.final_des_active
  SrsGoAround,      // [2026] longitudinal_modes.pitch_goaround_active
  GaTrk,            // [2052] lateral_modes.roll_goaround_active (GA TRK)
  LateralReversion, // [2101] lateral_mode_reversion (combinational, output at 4035)
  HdgTrk,           // [2129] HDG/TRK latch, split into hdg_active / trk_active by the HDG-V/S / TRK-FPA selection
  HdgActive,        // [2139] lateral_modes.hdg_active (combinational)
  TrkActive,        // [2140] lateral_modes.trk_active (combinational)
  NavArmed,         // [2183] armed_modes.nav_armed
  Nav,              // [2214] lateral_modes.nav_active
  LocArmed,         // [2241] armed_modes.loc_armed
  LocCapture,       // [2374] lateral_modes.loc_cpt_active (LOC*)
  LocTrack,         // [2404] lateral_modes.loc_trk_active (LOC)
  AltArmPossible,   // [2418] armed_modes.alt_acq_arm_possible
  AltArmed,         // [2465] armed_modes.alt_acq_armed (combinational)
  AltCapture,       // [2547] longitudinal_modes.alt_acq_active (ALT*)
  AltHold,          // [2589] longitudinal_modes.alt_hold_active (ALT)
  AltCruiseMemory,  // [2596] "at cruise altitude" latch (no output)
  AltCruise,        // [2609] longitudinal_modes.fma_dash_display (ALT CRZ, combinational)
  ReversionVs,      // [2668] longitudinal_mode_reversion_vs (combinational)
  VsFpa,            // [2697] V/S-FPA latch, split into vs_active / fpa_active by the HDG-V/S / TRK-FPA selection
  ClbArmed,         // [2761] armed_modes.clb_armed
  Clb,              // [2842] longitudinal_modes.clb_active
  DesArmed,         // [2875] armed_modes.des_armed
  Des,              // [2961] longitudinal_modes.des_active
  OpClb,            // [3021] longitudinal_modes.op_clb_active
  ReversionOpClb,   // [3023] longitudinal_mode_reversion_op_clb (combinational)
  OpDes,            // [3067] longitudinal_modes.op_des_active
  ExpClb,           // [3107] longitudinal_modes.exp_clb_active
  ExpDes,           // [3147] longitudinal_modes.exp_des_active
  GlideArmed,       // [3162] armed_modes.glide_armed
  GsCapture,        // [3215] longitudinal_modes.gs_capt_active
  GsTrack,          // [3258] longitudinal_modes.gs_trk_active
  TcasArmed,        // [3266] armed_modes.tcas_armed
  Tcas,             // [3300] longitudinal_modes.tcas_active
  TcasAltAcq,       // [3311] TCAS ALT_ACQ submode latch
  TcasAltHold,      // [3318] TCAS ALT_HOLD submode latch
  RollFdBarsFlashing,  // [3398] roll_fd_bars_flashing (10 s after a lateral reversion or AP/FD engagement)
  HdgTrkPreset,     // [3434] hdg_trk_preset_available
  AltSoft,          // [3734] alt_soft_mode_active
  AnyLateralMode,   // [3736] any_lateral_mode_engaged (combinational)
  ActiveLateralLaw, // [3742] active_lateral_law (enum, in Signals::activeLateralLaw; the bool is law != NONE)
  VsActive,         // [3993] longitudinal_modes.vs_active (combinational)
  FpaActive,        // [3995] longitudinal_modes.fpa_active (combinational)
  Flare,            // [4001] longitudinal_modes.flare_active (combinational)
  LocBcSelection,   // [4039] loc_bc_selection (a constant: back course is never selected)
  Count
};

constexpr std::size_t kSignalCount = static_cast<std::size_t>(Signal::Count);
const char* signalName(Signal s);

struct Signals {
  std::array<bool, kSignalCount> v{};
  lateral_law activeLateralLaw = lateral_law::NONE;  // Signal::ActiveLateralLaw
  double altSelOrCstr = 0;  // Signal::AltSelOrCstr: constraint altitude if applicable, else FCU altitude (ft)
  bool& operator[](Signal s) { return v[static_cast<std::size_t>(s)]; }
  bool operator[](Signal s) const { return v[static_cast<std::size_t>(s)]; }
};

/**
 * Parts of the FMGC that are not rewritten yet, for the current frame, filled from the generated model by the checker
 * (src/check_vertical.cpp, fillExternal). Add fields here as the rewrite needs them, with where they come from.
 */
struct External {
  const base_fmgc_logic_outputs* logic = nullptr;  // input conditioning (ADR/IR/RA source selection, on ground, ...)

  // AP/FD engagement and opposite-FMGC synchronisation
  bool oppModeSync = false;     // fmgc_opp_mode_sync: this FMGC copies the opposite FMGC's modes
  bool anyApFdEngaged = false;  // any_ap_fd_engaged: own or opposite AP or FD engaged
  bool modeReset = false;       // longitudinal_mode_reset: no AP/FD engaged, or first engine start on the ground

  // TCAS
  double tcasVsTarget = 0;        // tcas_vs_target (fpm)
  bool tcasRaCorrective = false;  // tcas_ra_corrective (also true when there is no RA)
  bool tcasModeAvailable = false; // logic.tcas_mode_available: TA/RA mode above 900 ft

  // Per-group additions (each group's header; filled by src/external_<group>.h)
  VsTcasExternal vsTcas;
  ClimbDescentExternal climbDescent;
  ApproachExternal approach;
  LateralExternal lateral;
};

struct Frame {
  double dt;
  const fmgc_inputs& in;                        // raw inputs of this frame
  const base_fmgc_ap_fd_logic_outputs& last;    // the FMGC's ap_fd_logic outputs of the previous frame
  const External& ext;
};

class VerticalModeLogic {
 public:
  /** Power-up / computer restart: latches to their initial values, block memories cleared. */
  void reset(Signals& s);

  /**
   * Updates one signal for this frame. Returns false when the signal is not rewritten yet; the checker then takes the
   * generated value.
   */
  bool update(Signal id, const Frame& f, Signals& s);

 private:
  void resetAlt(Signals& s);
  bool updateAlt(Signal id, const Frame& f, Signals& s);
  void resetVsTcas(Signals& s);
  bool updateVsTcas(Signal id, const Frame& f, Signals& s);
  void resetClimbDescent(Signals& s);
  bool updateClimbDescent(Signal id, const Frame& f, Signals& s);
  void resetApproach(Signals& s);
  bool updateApproach(Signal id, const Frame& f, Signals& s);
  void resetLateral(Signals& s);
  bool updateLateral(Signal id, const Frame& f, Signals& s);

  AltState alt_;
  VsTcasState vsTcas_;
  ClimbDescentState climbDescent_;
  ApproachState approach_;
  LateralModesState lateral_;
};

// ---------------------------------------------------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------------------------------------------------

/** Any longitudinal mode (or LAND) active last frame, except `except`. The common "another mode took over" reset. */
bool anyOtherVerticalModeLast(const base_fmgc_ap_fd_logic_outputs& last, Signal except);

/** Indicated altitude (ft), STD or baro-corrected per the selected reference; 0 when not normal. */
inline float indicatedAltitude(const Frame& f) {
  return blocks::a429::valueOr(f.ext.logic->altitude_indicated_ft, 0.0F);
}

/** FCU selected altitude (ft), 0 when not normal. */
inline float fcuAltitude(const Frame& f) {
  return blocks::a429::valueOr(f.in.bus_inputs.fcu_bus.selected_alt_ft, 0.0F);
}

inline bool fcuWord1Bit(const Frame& f, int bit) {
  return blocks::a429::bit(f.in.bus_inputs.fcu_bus.fcu_discrete_word_1, bit) != 0;
}

inline bool oppFmgcBit(const base_arinc_429& word, int bit) {
  return blocks::a429::bit(word, bit) != 0;
}

}  // namespace fmgc::readable
