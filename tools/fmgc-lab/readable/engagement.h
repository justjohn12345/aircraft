// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC AP / FD / A/THR engagement and the opposite-FMGC synchronisation (Phase 5c). Stock
// behaviour: must reproduce the generated FmgcComputer exactly (checked by src/check_engagement.cpp); fixes belong on
// personal/fmgc-fixes. Spec: docs/modes/engagement.md.
//
// Three stages, at their places in the generated step():
//   - engagement (generated FmgcComputer.cpp 1380–1559): the AP/FD/A/THR common conditions, FD engaged, the AP engage
//     request and AP engaged latches, the A/THR inhibit (instinctive disconnect held 15 s) and A/THR engaged latches,
//     AP and A/THR inop, opposite-FMGC priority and mode synchronisation, any AP/FD engaged and the mode reset. It runs
//     before every mode latch and reads the previous frame's modes (Delay_DSTATE);
//   - FD/AP flags (generated 3335–3403, after the vertical modes): FD auto disengagement (THR mode at VLS/VMAX with the
//     FD only), V/S-FPA target not held (AP at VLS/VMAX), the AP/FD mode reversion pulse (triple click) and the pitch FD
//     bars flashing. It reads this frame's vertical and lateral modes. (The roll FD bars flashing, generated alongside,
//     is in the lateral stage: lateral_modes.cpp.)
//   - landing capability (generated 3796–3863, after the law selection; engagement_land.cpp): LAND 2 / LAND 3 FAIL
//     PASSIVE / LAND 3 FAIL OP capability (with the AP in LAND), inop (equipment missing) and capacity (own, or the
//     opposite FMGC's when it is master).
//
// Not here: the A/THR modes (athr_modes.cpp).

#pragma once

#include "FmgcComputer_types.h"
#include "blocks.h"

namespace fmgc::readable::engagement {

/** One flag per rewritten output, in generated update order. */
enum Field : int {
  // stage 1: logic bus
  ApFdAthrCommonCondition = 0,  // logic.ap_fd_athr_common_condition
  ApFdCommonCondition,          // logic.ap_fd_common_condition
  FdOwnEngaged,                 // logic.fd_own_engaged
  ApInop,                       // logic.ap_inop
  ApOwnEngaged,                 // logic.ap_own_engaged
  AthrInop,                     // logic.athr_inop
  AthrOwnEngaged,               // logic.athr_own_engaged
  FmgcOppPriority,              // logic.fmgc_opp_priority
  // stage 1: ap_fd_logic bus
  FmgcOppModeSync,              // ap_fd_logic.fmgc_opp_mode_sync
  AnyApFdEngaged,               // ap_fd_logic.any_ap_fd_engaged
  LateralModeReset,             // ap_fd_logic.lateral_mode_reset
  LongitudinalModeReset,        // ap_fd_logic.longitudinal_mode_reset (the same signal)
  kStage1Count,
  // stage 2: ap_fd_logic bus
  FdAutoDisengage = kStage1Count,  // ap_fd_logic.fd_auto_disengage
  VsTargetNotHeld,                 // ap_fd_logic.vs_target_not_held
  ApFdModeReversion,               // ap_fd_logic.ap_fd_mode_reversion
  PitchFdBarsFlashing,             // ap_fd_logic.pitch_fd_bars_flashing
  kStage2End,
  // stage 3: ap_fd_logic bus
  Land2Capability = kStage2End,  // ap_fd_logic.land_2_capability
  Land3FailPassiveCapability,    // ap_fd_logic.land_3_fail_passive_capability
  Land3FailOpCapability,         // ap_fd_logic.land_3_fail_op_capability
  Land2Inop,                     // ap_fd_logic.land_2_inop
  Land3FailPassiveInop,          // ap_fd_logic.land_3_fail_passive_inop
  Land3FailOpInop,               // ap_fd_logic.land_3_fail_op_inop
  Land2Capacity,                 // ap_fd_logic.land_2_capacity
  Land3FailPassiveCapacity,      // ap_fd_logic.land_3_fail_passive_capacity
  Land3FailOpCapacity,           // ap_fd_logic.land_3_fail_op_capacity
  kFieldCount
};
const char* fieldName(int field);

/** The outputs, by value. */
struct Outputs {
  bool v[kFieldCount]{};
  bool& operator[](Field f) { return v[f]; }
  bool operator[](Field f) const { return v[f]; }
};

struct Stage1Inputs {
  double dt;
  const fmgc_inputs& in;                       // this frame's model inputs
  const base_fmgc_logic_outputs& logic;        // this frame's conditioned inputs (on ground, failures, ADR/IR/RA/FAC)
  const base_fmgc_ap_fd_logic_outputs& last;   // the ap_fd_logic outputs of the previous frame (Delay_DSTATE)
};

struct Stage2Inputs {
  double dt;
  const fmgc_inputs& in;
  const base_fmgc_logic_outputs& logic;
  const base_fmgc_ap_fd_logic_outputs& modes;  // this frame's modes and reversions (vertical and lateral stages)
};
using Stage3Inputs = Stage2Inputs;

/**
 * Block memories, named after what they hold, with the generated counterpart in brackets (D_Work_FmgcComputer_T). The
 * defaults are the generated first-running-frame initialisation.
 */
struct State {
  // Common conditions
  blocks::ConfirmNode apFdAthrCommon;      // AP/FD/A/THR common condition held 0.72 s           [sf_MATLABFunction_kv]
  blocks::Monostable rolloutHeadingDev;    // 3 s after ROLL OUT with the heading > 20° off the runway [sf_MATLABFunction_m3]
  blocks::ConfirmNode apFdCommon;          // AP/FD common condition held 0.72 s                 [sf_MATLABFunction_bv]
  // AP
  blocks::EdgeDetector apPbPush;           // FCU AP pushbutton pressed                         [sf_MATLABFunction_il]
  blocks::EdgeDetector apEngineStart;      // first engine running                              [sf_MATLABFunction_jp]
  blocks::EdgeDetector apOppEngage;        // opposite AP engaged                               [sf_MATLABFunction_cb]
  blocks::EdgeDetector apLandGaLost;       // no more LAND armed, LAND or SRS GA (last frame)   [sf_MATLABFunction_hp]
  blocks::SrLatch apEngageRequest;         // AP engage request (pushbutton accepted)           [Memory_PreviousInput]
  blocks::ConfirmNode apRequestHeld;       // request true, or false for less than 0.2 s        [sf_MATLABFunction_it]
  blocks::SrLatch apEngaged;               // AP engaged                [Delay_DSTATE_p; Q from Memory_PreviousInput_g]
  // A/THR
  blocks::ConfirmNode athrInstinctive15s;  // A/THR instinctive disconnect held 15 s            [sf_MATLABFunction_kz]
  blocks::SrLatch athrInhibited;           // A/THR inoperative until the FMGC restarts         [Memory_PreviousInput_g1]
  blocks::EdgeDetector athrPbPush;         // FCU A/THR pushbutton pressed                      [sf_MATLABFunction_o1]
  blocks::ConfirmNode athrSrs09s;          // SRS takeoff or go-around engaged for 0.9 s        [sf_MATLABFunction_mn]
  blocks::EdgeDetector athrSrsEdge;        // ... rising edge                                   [sf_MATLABFunction_jt]
  blocks::EdgeDetector athrLeversIdle;     // both thrust levers below 2.6°                     [sf_MATLABFunction_d4]
  blocks::EdgeDetector athrAlphaFloor;     // FAC alpha floor                                   [sf_MATLABFunction_ga]
  blocks::EdgeDetector athrOppSyncLost;    // synchronised to an opposite FMGC without A/THR    [sf_MATLABFunction_e5]
  blocks::EdgeDetector athrTcas;           // TCAS mode engaged (last frame)                    [sf_MATLABFunction_ag]
  blocks::ConfirmNode athrEcuNotEngaged;   // engaged here but not in both ECUs for 0.9 s       [sf_MATLABFunction_dx]
  blocks::SrLatch athrEngaged;             // A/THR engaged             [Delay_DSTATE_k; Q from Memory_PreviousInput_p]
  // Mode reset
  blocks::EdgeDetector resetEngineStart;   // first engine running                              [sf_MATLABFunction_p2o]
  // Stage 2
  blocks::Monostable modeReversion1s;      // 1 s AP/FD mode reversion pulse                    [sf_MATLABFunction_hdx]
  blocks::EdgeDetector pitchBarsApFdEngage;  // first AP or FD engaged                          [sf_MATLABFunction_h0]
  blocks::Monostable pitchBarsFlash10s;    // 10 s of pitch FD bars flashing                    [sf_MATLABFunction_ppo]
  // Stage 3: LAND 3 capabilities frozen below 100 ft RA
  bool land3FailOpHeld = false;            // LAND 3 FAIL OP held                               [pLand3FailOp]
  bool land3FailPassiveHeld = false;       // LAND 3 FAIL PASSIVE held                          [pLand3FailPass]
};

/** Intermediate conditions of the last update, for the checker's coverage report (not outputs). */
enum Condition : int {
  CondApPbPush = 0,         // AP pushbutton pressed
  CondApPbRejected,         // ... with the AP off, but not engaged (envelope, conditions)
  CondApEnvelopeReject,     // ... outside the speed/attitude envelope only
  CondApPbDisengage,        // ... with the AP engaged
  CondApOnGround,           // AP engaged on the ground
  CondApOppEngageReset,     // the opposite AP engages, single AP: own request reset
  CondApLandGaEndReset,     // FMGC 2, both APs: LAND/GA ended
  CondApGroundTogaReset,    // on the ground: SRS GA or a lever at TOGA/FLX
  CondApEngineStartReset,   // on the ground: first engine start
  CondApRequestDrop,        // request reset, AP still engaged (0.2 s)
  CondDualAp,               // both APs engaged
  CondFdTransfer,           // own FD engaged with its FD pushbutton off (opposite side's FD)
  CondRolloutOffCourse,     // ROLL OUT more than 20° off the runway course
  CondAthrInhibited,        // A/THR inoperative (instinctive disconnect held 15 s)
  CondAthrPbEngage,         // A/THR engaged by the pushbutton
  CondAthrPbLowRa,          // A/THR pushbutton below 100 ft RA (no engagement by the pushbutton)
  CondAthrSrsEngage,        // A/THR armed by SRS takeoff/go-around
  CondAthrAlphaFloorEngage, // A/THR engaged by alpha floor
  CondAthrTcasEngage,       // A/THR engaged by TCAS mode
  CondAthrPbDisengage,      // A/THR disconnected by the pushbutton
  CondAthrLeversIdle,       // A/THR disconnected by both levers to idle
  CondAthrEcuDisengage,     // A/THR disconnected: not engaged in an ECU for 0.9 s
  CondAthrOppSyncLost,      // A/THR disconnected: synchronised to an opposite FMGC without A/THR
  CondUnit2,                // this FMGC is FMGC 2
  CondOppUnhealthy,         // opposite FMGC not healthy
  CondOppPriorityUnit1,     // FMGC 1 yields to FMGC 2
  CondOppPriorityUnit2,     // FMGC 2 yields to FMGC 1
  CondModeResetEngineStart, // mode reset by the first engine start on the ground
  CondFdDisengageClimb,     // FD auto disengage condition: climb mode above VMAX + 4
  CondFdDisengageDescent,   // ... descent mode below VLS - 2
  CondVsNotHeldLow,         // V/S-FPA target not held near VLS
  CondVsNotHeldHigh,        // ... near VMAX
  CondLandOwnAp,            // own AP engaged with LAND armed or engaged
  CondLandOppAp,            // opposite AP engaged with LAND armed or engaged (opposite bus)
  CondLand3FailOpHeld,      // LAND 3 FAIL OP frozen below 100 ft
  CondLand3FailPassiveHeld, // LAND 3 FAIL PASSIVE frozen below 100 ft
  CondLandCapacityOwn,      // capacity from this FMGC's own capability
  CondLandCapacityOpp,      // capacity from the opposite FMGC's bus
  kConditionCount
};
const char* conditionName(int c);

class Engagement {
 public:
  bool cond[kConditionCount]{};  // conditions of the last update (coverage)

  /** As on the computer's first running frame (generated initialisation block). */
  void reset() { s_ = State{}; }

  /** Stage 1: fills the stage-1 fields of out. */
  void updateEngagement(const Stage1Inputs& in, Outputs& out);

  /**
   * Stage 2: fills the stage-2 fields of out. Reads this frame's stage-1 outputs from out (the checker replaces them by
   * the generated ones in resync mode).
   */
  void updateFdFlags(const Stage2Inputs& in, Outputs& out);

  /** Stage 3: fills the landing capability fields of out. Reads this frame's stage-1 outputs from out. */
  void updateLandCapability(const Stage3Inputs& in, Outputs& out);

  State& state() { return s_; }

 private:
  State s_;
};

}  // namespace fmgc::readable::engagement
