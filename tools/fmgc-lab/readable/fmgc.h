// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable FMGC computer (Phase 5 integration): every readable stage chained into one step() that runs on its own
// state and reproduces the generated FmgcComputer (FmgcComputer + FmgcOuterLoops) bit for bit, frame by frame
// (checked free-running by src/check_fmgc.cpp). Stock behaviour, bugs included; fixes belong on personal/fmgc-fixes.
// Stage graph, state inventory and restart semantics: docs/integration.md.
//
// Interface as the generated model's: initialize() once at power-up (generated FmgcComputer::initialize()), then
// step() every frame with the same fmgc_inputs (time, sim_data incl. computer_running, discretes, FMS, buses). When
// computer_running is false the outputs are held and nothing else changes; the first running frame after that (or
// after initialize()) restarts the computer: every memory is reset except the few the generated code keeps in block
// outputs (see State).
//
// No heap allocation, no static mutable state: every memory is a member, so several instances can run side by side
// (e.g. a shadow of the generated FMGC in the sim) and an instance can be copied to snapshot its state.

#pragma once

#include "FmgcComputer_types.h"
#include "FmgcOuterLoops_types.h"
#include "athr.h"
#include "conditioning.h"
#include "engagement.h"
#include "laws.h"
#include "laws_lateral.h"
#include "outer_loop_inputs.h"
#include "targets.h"
#include "tcas.h"
#include "vertical.h"

namespace fmgc::readable {

class FmgcComputer {
 public:
  /** This frame's intermediate results, kept after step() for diagnostics (the checker attributes mismatches). */
  struct Buses {
    base_fmgc_logic_outputs logic{};           // conditioning + engagement stage 1
    base_fmgc_ap_fd_logic_outputs apFd{};      // modes, targets, flags, laws, LAND capability
    ap_laws_output outerLoops{};               // outer-loop inputs (time, data, input) and outputs (output)
    base_fmgc_athr_outputs athr{};             // A/THR modes and N1 command
  };

  /** Power-up: every memory to its initial value, outputs to zero (generated initialize()). */
  void initialize();

  /**
   * One frame. Returns the outputs (FmgcComputer_Y.out): this frame's when computer_running, else the last running
   * frame's (zero before the first one).
   */
  const fmgc_outputs& step(const fmgc_inputs& in);

  const fmgc_outputs& outputs() const { return out_; }
  const Buses& buses() const { return bus_; }
  /** True after a running frame (generated Runtime_MODE): the next running frame continues without a restart. */
  bool running() const { return running_; }

  // Stage access for checkers and probes.
  const conditioning::Conditioning& conditioningStage() const { return conditioning_; }
  const engagement::Engagement& engagementStage() const { return engagement_; }
  const VerticalModeLogic& modeLogic() const { return modes_; }
  const Signals& modeSignals() const { return signals_; }
  const targets::Values& targetValues() const { return targetValues_; }
  const athr::Autothrust& autothrustStage() const { return athr_; }
  const tcas::TcasLogic& tcasStage() const { return tcas_; }
  const OuterLoopInputState& outerLoopInputState() const { return outerLoopInputs_; }

 private:
  /** The computer's first running frame after initialize() or a stop (generated 452–692). */
  void restart();

  // ---- Stages, each owning its memories (reset by restart() unless noted)
  conditioning::Conditioning conditioning_;  // keeps the runway heading memory over a restart
  engagement::Engagement engagement_;
  tcas::TcasLogic tcas_;
  VerticalModeLogic modes_;  // keeps the ALT* and TCAS ALT_ACQ target sample-and-holds over a restart
  Signals signals_;          // mode latches and combinational mode signals (this frame's, then next frame's memory)
  targets::Targets targets_;
  targets::Values targetValues_;  // keeps the VAPP memo over a restart
  laws::LateralLaws lateralLaws_;
  laws::VerticalLaws verticalLaws_;  // keeps the ALT hold target over a restart
  OuterLoopInputState outerLoopInputs_;  // DME and track memories: kept over a restart
  athr::Autothrust athr_;

  // ---- End-of-step delays: this frame's buses as the next frame's previous values (generated 5107–5128); zero on a
  // restart (Delay_InitialCondition, Delay1_InitialCondition, Delay2_InitialCondition are all zero / NONE / VS)
  base_fmgc_ap_fd_logic_outputs lastApFd_{};  // [Delay_DSTATE]
  base_fmgc_athr_outputs lastAthr_{};         // [Delay1_DSTATE]
  ap_raw_output lastOuterLoops_{};            // [Delay2_DSTATE]

  bool running_ = false;  // [Runtime_MODE]
  bool firstFrame_ = true;  // first running frame since the restart [Delay_DSTATE_fe]

  Buses bus_{};
  fmgc_outputs out_{};  // [FmgcComputer_Y.out]
};

}  // namespace fmgc::readable
