// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC autothrust (Phase 5a). Stock behaviour: must reproduce the generated FmgcComputer
// exactly (checked by src/check_athr.cpp); fixes belong on personal/fmgc-fixes.
//
// The autothrust runs after the vertical and lateral logic and the outer loops (generated FmgcComputer.cpp, about lines
// 4181-4640 on master). Its inputs are this frame's FMGC buses (data, logic, ap_fd_logic), which the generated code does
// not change after the autothrust and copies to the outputs, and its own outputs of the previous frame (Delay1_DSTATE).
// It produces base_fmgc_athr_outputs in two stages:
//   - modes (athr_modes.cpp): alpha floor, THR/SPEED/RETARD mode latches, A/THR active and limited, thrust target idle,
//     LVR CLB/MCT/ASYM messages, FMA mode and message;
//   - N1 command (athr_n1.cpp): the SPEED/MACH controller, the THR mode ramp, RETARD, alpha floor, the N1 integrator
//     and its limits. It uses this frame's mode outputs (the readable ones, or the generated ones while the modes are
//     not rewritten or in resync mode).

#pragma once

#include <cstddef>

#include "FmgcComputer_types.h"
#include "blocks.h"

namespace fmgc::readable::athr {

/** One flag per base_fmgc_athr_outputs field, in declaration order. */
enum Field : int {
  AthrActive = 0,
  AthrLimited,
  AlphaFloorMode,
  ThrustMode,
  ThrustTargetIdle,
  SpeedMachMode,
  RetardMode,
  FmaMode,
  FmaMessage,
  N1Command,
  kFieldCount
};
const char* fieldName(int field);

struct Inputs {
  double dt;
  const fmgc_inputs& data;                        // out.data (= the model inputs as conditioned)
  const base_fmgc_logic_outputs& logic;           // out.logic
  const base_fmgc_ap_fd_logic_outputs& apFdLogic;  // out.ap_fd_logic (this frame's modes, targets, laws)
  const base_fmgc_athr_outputs& last;             // autothrust outputs of the previous frame
};

struct Result {
  base_fmgc_athr_outputs out{};
  bool rewritten[kFieldCount]{};  // fields computed by the readable code (the others are filled by the checker)
};

/** Mode-stage state: latches (SR flip-flops) and other memories, named after what they hold. */
struct ModesState {
  // TCAS: holds the A/THR in its pre-RA thrust behaviour (inhibits alpha floor) from TCAS mode engagement until the
  // pilot retards a lever (after 5 s) or TCAS mode ends.
  blocks::ConfirmNode tcasActiveFor5s;   // TCAS mode active for 5 s            (sf_MATLABFunction_a)
  blocks::EdgeDetector tcasEngagePulse;  // TCAS mode engagement               (sf_MATLABFunction_g)
  blocks::SrLatch tcasLatch;             //                                     (Memory_PreviousInput_jb)
  // Alpha floor
  blocks::EdgeDetector alphaFloorRequestPulse;  // FAC alpha floor request rising edge (sf_MATLABFunction_k)
  blocks::SrLatch alphaFloor;                    // A.FLOOR / TOGA LK                   (Memory_PreviousInput_hu)
  // Thrust modes: each set by a vertical law engagement (or the opposite FMGC), reset by another thrust mode
  blocks::EdgeDetector thrustLawPulse;   // THR law (SPD/MACH on pitch, SRS, VPATH thrust) engaged (sf_MATLABFunction_j)
  blocks::ConfirmNode thrustSetRecent;   // THR set condition true within the last 0.72 s         (sf_MATLABFunction_ay)
  blocks::SrLatch thrustMode;            // THR mode                                              (Memory_PreviousInput_bh)
  blocks::EdgeDetector speedLawPulse;    // SPEED law (pitch flies a path or altitude) engaged     (sf_MATLABFunction_ge)
  blocks::ConfirmNode speedSetRecent;    //                                                       (sf_MATLABFunction_ig)
  blocks::SrLatch speedMode;             // SPEED/MACH mode                                       (Memory_PreviousInput_cm)
  blocks::ConfirmNode retardSetRecent;   //                                                       (sf_MATLABFunction_kh)
  blocks::SrLatch retardMode;            // RETARD mode                                           (Memory_PreviousInput_ol)
  // Lever messages
  blocks::SrLatch lvrClbArmed;  // above thrust reduction altitude, both engines: LVR CLB possible (Memory_PreviousInput_kr)
  blocks::SrLatch lvrMctArmed;  // same with one engine out: LVR MCT possible                      (Memory_PreviousInput_km)
};

/** N1-stage state: filters, integrator, delays. */
struct N1State {
  // filled in by the N1 rewrite
  bool placeholder = false;
};

class Autothrust {
 public:
  /** As on the computer's first running frame (generated initialisation block). */
  void reset();

  /** Mode stage: sets the mode fields of r.out it rewrites and their r.rewritten flags. */
  void updateModes(const Inputs& in, Result& r);

  /**
   * N1 stage: sets r.out.n1_c_percent (and r.rewritten[N1Command]). Reads this frame's mode fields from r.out, which the
   * checker completes with generated values for fields the mode stage does not rewrite (and in resync mode).
   */
  void updateN1(const Inputs& in, Result& r);

  ModesState& modesState() { return modes_; }
  N1State& n1State() { return n1_; }

 private:
  ModesState modes_;
  N1State n1_;
};

}  // namespace fmgc::readable::athr
