// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC lateral control laws (FmgcOuterLoops.cpp 2232–2757, Phase 3). Stock behaviour: must
// reproduce the generated FmgcOuterLoops exactly (checked by src/check_laws.cpp); fixes belong on personal/fmgc-fixes.
//
// Every frame every lateral law computes its commands, whichever law is active: a roll command (phi, deg), an AP
// sideslip/yaw command (Beta_c, deg) and a nosewheel command (before the AP gate). Only LOC_TRACK (runway alignment
// below 30 ft) and ROLL_OUT command anything but 0 for the last two. The output stage picks the active law's commands
// and turns them into the FD and AP roll orders (bank limit, FD lag, AP roll-rate limit and fade-in), the FD and AP
// sideslip orders and the nosewheel order. LOC_CPT's roll command is also an output (Phi_loc_c). See
// docs/laws/lateral.md.

#pragma once

#include <limits>

#include "FmgcOuterLoops_types.h"
#include "blocks.h"

namespace fmgc::readable::laws {

/** lateral_law enum values (FmgcComputer_types.h). */
enum LateralLaw : int { LatNone = 0, Hdg, Track, Hpath, LocCpt, LocTrack, RollOut, kLateralLawCount };
const char* lateralLawName(int law);

/** A lateral law's commands. NaN = not rewritten yet. */
struct LateralCommand {
  double phi = std::numeric_limits<double>::quiet_NaN();        // roll command, deg (before the bank limit)
  double beta = std::numeric_limits<double>::quiet_NaN();       // AP sideslip / yaw command, deg
  double nosewheel = std::numeric_limits<double>::quiet_NaN();  // nosewheel command before the AP gate, deg
};

struct LateralResults {
  LateralCommand cmd[kLateralLawCount];
  bool rewritten[kLateralLawCount]{};
  double phiLocDeg = std::numeric_limits<double>::quiet_NaN();  // output.Phi_loc_c (LOC_CPT roll command)
};

struct LateralOutputs {
  double fdPhiDeg;    // output.flight_director.Phi_c_deg
  double apPhiDeg;    // output.autopilot.Phi_c_deg
  double fdBetaDeg;   // output.flight_director.Beta_c_deg
  double apBetaDeg;   // output.autopilot.Beta_c_deg
  double nosewheel;   // output.Nosewheel_c (-1..1)
};

/**
 * True during the first second after `active` goes true (1/dt samples, at most 100), as a 100-sample delay line of
 * `active` that is cleared while `active` is false (generated: Delay_DSTATE_l4 / Delay_DSTATE_n, a Delay block whose
 * length is 1 s / dt, compared with its input). With dt > 1 s it is never true.
 */
struct FirstSecondDetector {
  bool history[100]{};  // history[99] = previous sample

  void reset();
  bool update(bool active, double dt);
};

/** HDG or TRACK select: heading (track) error with the turn-direction memory, gain on TAS, yaw-rate damping. */
struct HeadingSelectLaw {
  FirstSecondDetector justEngaged;   // the turn direction is the shorter one during the first second of the mode
  blocks::TurnDirectionChart turn;   // sf_Chart (HDG) / sf_Chart_b (TRACK)
  blocks::HeadingBankLimit bankLimit;  // sf_MATLABFunction_g / _n

  void reset() {
    justEngaged.reset();
    turn.reset();
    bankLimit.reset();
  }
};

/** HPATH (NAV): FMS roll feedforward. */
struct NavLawState {
  double fmsPhiRateLimited = 0;  // pY_f, FMS roll order rate limited to 10°/s
  bool fmsPhiPrimed = false;
  blocks::LagFilter<double> fmsPhiLag;  // sf_LagFilter, 4 rad/s
};

/** LOC_CPT: deviation-term limit frozen at capture. */
struct LocCaptureState {
  double deviationLimitDeg = 0;  // limit (deg of track): the intercept angle when LOC* engaged, 15..115°
  bool deviationLimitPrimed = false;
};

/** LOC_TRACK: deviation with rate lead, runway-alignment (decrab) sideslip below 30 ft, blend from LOC_CPT. */
struct LocTrackState {
  blocks::LagFilter<double> deviationLag;  // sf_LagFilter_i, 1 rad/s
  double previousDeviation = 0;            // Delay_DSTATE (discrete derivative)
  blocks::LagFilter<double> deviationLeadLag;  // sf_LagFilter_m, deviation + 10 s · rate, 1 rad/s
  blocks::TurnDirectionChart runwayHeadingError;  // sf_Chart_k (always the shorter way)
  blocks::LagFilter<double> alignLag;      // sf_LagFilter_a, 0.7 rad/s
  blocks::SwitchBlend captureToTrack;      // sf_RateLimiter_e, LOC_CPT -> LOC_TRACK fade over 2 s
};

/** ROLL_OUT: runway centreline tracking with the rudder / nosewheel. */
struct RollOutState {
  double deviationIntegral = 0;      // Delay_DSTATE_p (deg), ±3
  double courseDeg = 0;              // storage_c: runway course memorised at roll-out engagement
  bool coursePrimed = false;
  blocks::TurnDirectionChart headingError;  // sf_Chart_h (always the shorter way)
  blocks::SwitchBlend rateDampingBlend;     // sf_RateLimiter_o, over 8 s
  double rudderAtEngagementDeg = 0;  // storage_a: rudder position memorised at roll-out engagement
  bool rudderPrimed = false;
  blocks::SwitchBlend commandBlend;         // sf_RateLimiter_k, over 3 s
};

/** Output stage memories. */
struct LateralOutputState {
  double bankLimitDeg = 25;  // pY_b, rate limited at 5°/s
  bool bankLimitPrimed = false;
  blocks::LagFilter<double> fdPhiLag;   // sf_LagFilter_f, 2 rad/s
  double apPhiIntegrator = 0;           // Delay_DSTATE_f: AP roll command, moved at the roll-rate limit
  bool apPhiIntegratorLoad = true;      // icLoad
  double apRollRateLimit = 5;           // pY_c, 5°/s (7.5°/s in LOC and roll-out), changed at 2.5°/s²
  bool apRollRateLimitPrimed = false;
  blocks::LagFilter<double> apPhiLag;   // sf_LagFilter_g, 1 rad/s
  blocks::SwitchBlend apEngagedBlend;   // sf_RateLimiter_g, AP fade in over 1 s, out over 0.1 s
  blocks::LagFilter<double> fdBetaLag;  // sf_LagFilter_n, 2 rad/s
};

class LateralLaws {
 public:
  /** Power-up (generated init()) and computer restart (generated reset()). */
  void reset();

  /** Every lateral law's commands for this step, plus Phi_loc_c. */
  void computeLaws(const ap_laws_output& in, LateralResults& out);

  /** Output stage: picks in.input.lateral_law's commands. Returns false if a selected command is NaN. */
  bool computeOutputs(const ap_laws_output& in, const LateralCommand (&cmd)[kLateralLawCount], LateralOutputs& out);

 private:
  double headingSelect(HeadingSelectLaw& law, bool active, double targetDeg, double currentDeg,
                       const ap_laws_output& in);

  NavLawState nav_;
  HeadingSelectLaw hdg_;
  HeadingSelectLaw track_;
  LocCaptureState locCapture_;
  LocTrackState locTrack_;
  RollOutState rollOut_;
  LateralOutputState output_;
};

}  // namespace fmgc::readable::laws
