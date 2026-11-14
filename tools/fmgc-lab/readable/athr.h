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

/**
 * N1-stage state: filters, integrator, delays. The defaults are the generated first-running-frame initialisation
 * (filters re-prime on their first input, delays 0, integrator loaded from the actual N1 on the first frame).
 * Generated counterparts in brackets (D_Work_FmgcComputer_T).
 */
struct N1State {
  /** Washout (high pass) in float arithmetic: the generated FmgcComputer inlines this float variant once. */
  struct FloatWashoutFilter {
    float pY = 0, pU = 0;
    bool primed = false;

    void reset() { primed = false; }

    float update(float u, double c1, double dt) {
      if (!primed) {
        pU = u;
        pY = u;
        primed = true;
      }
      double denom = dt * c1 + 2.0;
      double ca = 2.0 / denom;
      float y = static_cast<float>((2.0 - dt * c1) / denom) * pY + (u * static_cast<float>(ca) - pU * static_cast<float>(ca));
      pY = y;
      pU = u;
      return y;
    }
  };

  // SPEED/MACH controller: acceleration estimate and predicted speed error
  blocks::WashoutFilter groundSpeedRate;        // d(ground speed)/dt, 0.5 s lag [pY_k, pU_l]
  blocks::LeadLagFilter inertialAccelHighPass;  // attitude-corrected inertial acceleration, 0.2 s high pass [sf_LeadLagFilter]
  blocks::LeadLagFilter airspeedWashout;        // IAS through 5s/(5s+1) = 5 s x lagged IAS rate [sf_LeadLagFilter_b]
  double lastPredictedSpeedErrorKn = 0;         // discrete derivative memory [Delay_DSTATE_i]
  blocks::LagFilter<double> errorRateLag;       // predicted speed error rate, 0.1 s lag [pY_e, pU]
  // SPEED/MACH controller: load factor feedback
  blocks::LagFilter<float> normalAccelLag;      // body normal acceleration (g), 1/3 s lag [sf_LagFilter_g]
  FloatWashoutFilter loadFactorWashout;         // 10 x load factor increment, 0.1 s washout [pY_h, pU_p]
  // ALT soft mode gain fade (0 -> 1 in 10 s, 1 -> 0 in 0.5 s) [pY]
  blocks::SwitchBlend altSoftBlend;
  // N1 command integrator
  double n1CommandPercent = 0;   // limited integrator output of the previous frame [Delay_DSTATE_l = Delay_DSTATE_f]
  bool loadIntegrator = true;    // re-initialise from the actual N1 this frame (first frame) [icLoad]
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
