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
  // filled in by the modes rewrite
  bool placeholder = false;
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
