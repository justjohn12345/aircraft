// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC TCAS RA decoding and TCAS FMA flags (Phase 5 integration). Stock behaviour: must
// reproduce the generated FmgcComputer exactly (checked through src/check_fmgc.cpp); fixes belong on
// personal/fmgc-fixes. Spec: docs/modes/alt-vs-fpa-tcas.md ("TCAS outputs"), docs/integration.md.
//
// Generated FmgcComputer.cpp 1560–1668 (after the engagement stage 1, before the altitude target), outputs at 1635 and
// 1666–1668, tcas_ra_inhibited at 1792; the TCAS submode (3320–3326) is a function of two vertical-mode latches.
//   - tcas_ra_corrective: RA word bits 18/19/20 (also true when there is no RA at all);
//   - tcas_vs_target: for a corrective RA the RA rate (bits 11–16 x 100 fpm, + 200 fpm when non-zero, negative with bit
//     17), otherwise the selected ADR's barometric V/S. The generated code passes the latter through a sample-and-hold
//     (FmgcComputer_B.u_lyjjlj) that is updated in exactly the cases where it is output, so it carries no state;
//   - tcas_ra_inhibited: 10 s after TCAS mode (previous frame) while the TCAS RA mode is not available;
//   - trk_fpa_deselected: 10 s after TCAS mode ends if the FCU was switched to HDG-V/S during TCAS mode;
//   - longi_large_box_tcas: 10 s after TCAS mode engages with CLB, DES, FINAL, G/S or ALT armed (previous frame).
// Every memory is reset on the computer's first running frame.

#pragma once

#include "FmgcComputer_types.h"
#include "blocks.h"

namespace fmgc::readable::tcas {

/** Block memories, generated counterparts in brackets. */
struct State {
  blocks::Monostable raInhibited10s;       // TCAS mode while the RA mode is unavailable, 10 s  [sf_MATLABFunction_an]
  blocks::EdgeDetector hdgVsSelected;      // FCU word 1 bit 24 (HDG-V/S) set                     [sf_MATLABFunction_bz]
  blocks::SrLatch hdgVsDuringTcas;         // HDG-V/S selected during TCAS mode                   [Memory_PreviousInput_e]
  bool tcasEndedAfterHdgVs = false;        // ... and TCAS mode no longer active (latch reset)    [Delay_DSTATE_o]
  blocks::Monostable trkFpaDeselected10s;  //                                                     [sf_MATLABFunction_g0]
  blocks::Monostable largeBox10s;          //                                                     [sf_MATLABFunction_pl4]
};

struct Outputs {
  bool raCorrective = false;      // ap_fd_logic.tcas_ra_corrective
  double vsTargetFpm = 0;         // ap_fd_logic.tcas_vs_target
  bool raInhibited = false;       // ap_fd_logic.tcas_ra_inhibited
  bool trkFpaDeselected = false;  // ap_fd_logic.trk_fpa_deselected
  bool longiLargeBox = false;     // ap_fd_logic.longi_large_box_tcas
};

struct Inputs {
  double dt;
  const fmgc_inputs& in;
  const base_fmgc_logic_outputs& logic;       // this frame (selected ADR V/S, TCAS mode available)
  const base_fmgc_ap_fd_logic_outputs& last;  // previous frame's modes (Delay_DSTATE)
};

class TcasLogic {
 public:
  /** As on the computer's first running frame. */
  void reset() { s_ = State{}; }
  void update(const Inputs& in, Outputs& out);
  State& state() { return s_; }
  const State& state() const { return s_; }

 private:
  State s_;
};

/** active_tcas_submode: ALT_ACQ while its latch is set, else ALT_HOLD while its latch is set, else VS. */
inline tcas_submode activeSubmode(bool altAcqLatch, bool altHoldLatch) {
  return altAcqLatch ? tcas_submode::ALT_ACQ : (altHoldLatch ? tcas_submode::ALT_HOLD : tcas_submode::VS);
}

}  // namespace fmgc::readable::tcas
