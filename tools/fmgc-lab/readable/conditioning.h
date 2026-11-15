// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC input conditioning (Phase 5e): the code that builds the `logic` bus
// (base_fmgc_logic_outputs) from the raw inputs. Stock behaviour: must reproduce the generated FmgcComputer exactly
// (checked by src/check_conditioning.cpp); fixes belong on personal/fmgc-fixes. Spec: docs/conditioning.md.
//
// Generated FmgcComputer.cpp 695–1379 (master), copied to the logic bus at 3864–3971; runs first in every running step,
// before the AP/FD engagement (1380). Families, in generated order:
//   - ADR: validity of own / opposite / 3 (any of CAS, AoA, Mach, standard altitude failed), 2-of-3 vote, selection own
//     -> 3 -> zero bus (the opposite ADR is voted but never selected);
//   - baro-corrected altitude for the side (FCU EIS discrete word 2 STD bit);
//   - IR: validity of own / opposite / 3 (any of 8 labels not normal operation), 2-of-3 vote, selection as for the ADR;
//   - RA: own unless failed, then opposite; both failed: 2500 ft with the own SSM;
//   - FAC: own bus if the own FAC is healthy, else the opposite one; rudder control (both FAC words 2); speeds and
//     weights validity; landing gear / on ground, flap lever, flap/slat data (FAC discrete word 5);
//   - engines, FCU, ILS (own unless failed, LOC deviation inverted for a back beam), TCAS, ILS tune inhibit and the
//     runway heading memory.
//
// Not here: the logic fields computed by the AP/FD engagement (readable/engagement.cpp): ap_fd_athr_common_condition,
// ap_fd_common_condition, fd_own_engaged, ap_own_engaged, athr_own_engaged, ap_inop, athr_inop, fmgc_opp_priority. The
// conditioning leaves them false.

#pragma once

#include "FmgcComputer_types.h"
#include "blocks.h"

namespace fmgc::readable::conditioning {

struct Inputs {
  const fmgc_inputs& in;                      // this frame's model inputs
  const base_fmgc_ap_fd_logic_outputs& last;  // the previous frame's modes (Delay_DSTATE), for the ILS tune inhibit
};

/**
 * Block memories, with the generated counterpart in brackets. The defaults are the generated initialize() values.
 */
struct State {
  blocks::ConfirmNode inFlight5s;  // not on ground for 5 s                                  [sf_MATLABFunction_fw]
  double rwyHdgMemo = 0.0;         // ILS runway heading, frozen while the ILS tune is inhibited [FmgcComputer_B.u_lyjj]
};

/** Intermediate conditions of the last update, for the checker's coverage report (not outputs). */
enum Condition : int {
  CondAdrOwnInvalid = 0,     // own ADR: one of the four voted labels failed
  CondAdrOppInvalid,         // opposite ADR invalid
  CondAdr3Invalid,           // ADR 3 invalid
  CondAdrSelOwn,             // adr_computation_data = own ADR
  CondAdrSel3,               // ... = ADR 3 (own invalid)
  CondAdrSelNone,            // ... = zero bus (own and 3 invalid)
  CondAltStd,                // altitude_indicated = standard (STD selected)
  CondAltStdFcuWordFailed,   // ... = standard because the FCU EIS word 2 failed
  CondAltCorrected1,         // ... = baro-corrected 1 (unit 1)
  CondAltCorrected2,         // ... = baro-corrected 2 (unit 2)
  CondIrOwnInvalid,          // own IR: one of the eight voted labels not normal operation
  CondIrOppInvalid,          // opposite IR invalid
  CondIr3Invalid,            // IR 3 invalid
  CondIrSelOwn,              // ir_computation_data = own IR
  CondIrSel3,                // ... = IR 3
  CondIrSelNone,             // ... = zero bus
  CondRaBothOk,              // radio height from the own RA, both not failed
  CondRaOwnOnly,             // ... own RA, opposite failed
  CondRaOppOnly,             // ... opposite RA, own failed
  CondRaNone,                // both failed: 2500 ft
  CondFacSelOwn,             // chosen_fac_bus = own FAC (healthy)
  CondFacSelOpp,             // ... = opposite FAC
  CondFacRudderNoYawDamper,  // rudder control failure: no yaw damper engaged in either FAC
  CondFacRudderNoTrim,       // ... no rudder trim engaged in either FAC
  CondFacRudderBothUnhealthy,  // ... both FACs unhealthy
  CondOnGround,              // on ground (both main gears pressed, LGCIU data valid)
  CondInFlight5s,            // not on ground for 5 s
  CondGroundEnginesStopped,  // on ground with both engines stopped
  CondFlapLever0,            // flap lever position 0 (no bit set)
  CondFlapLever1,
  CondFlapLever2,
  CondFlapLever3,
  CondFlapLever4,
  CondFlapLever5,
  CondIlsOwnValid,           // own ILS: LOC and G/S not failed
  CondIlsOppValid,           // opposite ILS valid
  CondIlsSelOpp,             // ils_computation_data = opposite ILS
  CondLocBackbeam,           // localizer deviation inverted (back beam)
  CondTcasAvailable,         // TCAS mode available
  CondTcasRaInhibit,         // TCAS in TA/RA, word valid, but radio height ≤ 900 ft
  CondTuneInhibitRwyLoc,     // ILS tune inhibit: RWY LOC submode (last frame)
  CondTuneInhibitLand,       // ... LAND (last frame)
  CondTuneInhibitLandArmedLow,  // ... LAND armed (last frame) at or below 700 ft RA
  CondRwyHdgMemoUpdated,     // runway heading memory follows the ILS
  kConditionCount
};
const char* conditionName(int c);

class Conditioning {
 public:
  bool cond[kConditionCount]{};  // conditions of the last update (coverage)

  /** As at FmgcComputer::initialize(): everything, including the runway heading memory. */
  void initialize() { s_ = State{}; }

  /**
   * As on the computer's first running frame (generated initialisation block): the confirm node only. The runway
   * heading memory is a block output (FmgcComputer_B), reset only by initialize().
   */
  void restart() { s_.inFlight5s.reset(); }

  /** Fills every conditioned field of out (the engagement fields are set to false). */
  void update(const Inputs& in, base_fmgc_logic_outputs& out);

  State& state() { return s_; }

 private:
  State s_;
};

// Building blocks, exposed for reuse and for the tests.

/** ADR validity: none of CAS, AoA, Mach and standard altitude reports a failure (NCD and FT count as valid). */
bool adrValid(const base_adr_bus& adr);
/** IR validity: yaw rate, lateral acceleration, pitch, roll, true heading, true track, inertial V/S and latitude all in
 * normal operation (NCD and FT count as invalid). */
bool irValid(const base_ir_bus& ir);
/** Two (or three) of three failed. */
inline bool twoOfThree(bool a, bool b, bool c) {
  return (a && b) || (a && c) || (b && c);
}

}  // namespace fmgc::readable::conditioning
