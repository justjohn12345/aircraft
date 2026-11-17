// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Autothrust modes and FMA: alpha floor, THR / SPEED-MACH / RETARD mode latches, A/THR active and limited, thrust target
// idle, the LVR CLB / LVR MCT / LVR ASYM message and the FMA mode.
// Spec: docs/athr-modes.md. Generated: FmgcComputer.cpp 4181–4423 (outputs 4947–4955, delays 5116–5125, init 511–518 and
// 674–681).
//
// Thrust lever angles (TLA, degrees): IDLE 0, CL 25, FLX/MCT 35, TOGA 45; negative is reverse.
//
// The generated code reads the lever angles two ways: through A429ValueOrDefault (0 unless the FADEC word is normal
// operation) for the A/THR active range, the limited flag and the TCAS latch, but as the raw word value (any SSM) for
// the lever messages and the FMA mode. Both are kept.

#include <cmath>

#include "athr.h"

namespace fmgc::readable::athr {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::rawValue;
using blocks::a429::valueOr;
using Law = vertical_law;

// Lever angle bands (degrees)
constexpr float kTlaDefaultDeg = 0;           // lever angle used when the FADEC word is not normal operation
constexpr float kTlaIdleDeg = 0;              // A/THR active range: [IDLE, CL] (one engine out: (IDLE, MCT])
constexpr float kTlaClimbDeg = 25;            // CL detent
constexpr float kTlaMctDeg = 35;              // FLX/MCT detent
constexpr float kTlaClimbDetentLowDeg = 24;   // "in the CL detent" for the lever messages: (24, 26)
constexpr float kTlaClimbDetentHighDeg = 26;  //
constexpr float kTlaMctDetentLowDeg = 34;     // "in the FLX/MCT detent": (34, 36); also the THR MCT / LVR MCT limit
constexpr float kTlaMctDetentHighDeg = 36;    //
constexpr float kTlaTogaDeg = 44;             // above: TOGA
constexpr float kTlaLimitedDeg = 24;          // A/THR limited (manual thrust ceiling) below CL ...
constexpr float kTlaLimitedOneEngineDeg = 34;  // ... or below MCT with one engine out
constexpr float kTlaTcasRetardDeg = 24;       // a lever pulled below this ends the TCAS latch

constexpr double kTcasConfirmS = 5;            // TCAS mode active this long before a lever retard ends the TCAS latch
constexpr double kModeTakeoverDelayS = 0.72;   // another thrust mode replaces this one only after this
constexpr float kRetardRadioHeightFt = 40;     // RETARD at or below 40 ft RA in LAND with the AP engaged
constexpr double kTcasRaClimbVsFpm = 500;      // TCAS V/S target above this: a climbing RA
constexpr float kVmaxMarginKt = 20;            // climbing RA without LVR CLB only below VMAX - 20 kt

// Bits (1-based)
constexpr int kFacWord5AlphaFloor = 29;  // FAC discrete word 5: alpha floor condition
constexpr int kOppAtsSpeedMode = 18;     // opposite FMGC ATS discrete word: SPEED/MACH mode
constexpr int kOppAtsRetardMode = 20;    //                                  RETARD mode
constexpr int kOppAtsThrustMode = 21;    //                                  THR mode

/** True when lo <= x <= hi. */
bool within(float x, float lo, float hi) {
  return x >= lo && x <= hi;
}

/** True when lo < x <= hi. */
bool aboveUpTo(float x, float lo, float hi) {
  return x > lo && x <= hi;
}

/** True when lo < x < hi. */
bool strictlyBetween(float x, float lo, float hi) {
  return x > lo && x < hi;
}

/**
 * The !Q output of the SR flip-flop (Logic_table[index + 8]): the complement of Q, except that it is also false when set
 * and reset are both true (Q is then false too, reset priority).
 */
bool latchNotQ(bool set, bool reset, bool qBefore) {
  return !set && (reset || !qBefore);
}

}  // namespace

void Autothrust::updateModes(const Inputs& in, Result& r) {
  ModesState& s = modes_;
  const auto& bus = in.data.bus_inputs;
  const auto& logic = in.logic;
  const auto& ap = in.apFdLogic;
  const auto& lm = ap.longitudinal_modes;
  const auto& last = in.last;
  const double dt = in.dt;
  const bool engaged = logic.athr_own_engaged;  // A/THR pushbutton engaged (this FMGC)
  const bool oppSync = ap.fmgc_opp_mode_sync;   // this FMGC follows the opposite FMGC's modes
  const Law law = ap.active_longitudinal_law;

  // Lever angles: validated (0 when the FADEC word is not normal) and raw.
  const float tlaOwn = valueOr(bus.fadec_own_bus.selected_tla_deg, kTlaDefaultDeg);
  const float tlaOpp = valueOr(bus.fadec_opp_bus.selected_tla_deg, kTlaDefaultDeg);
  const float tlaOwnRaw = rawValue(bus.fadec_own_bus.selected_tla_deg);
  const float tlaOppRaw = rawValue(bus.fadec_opp_bus.selected_tla_deg);
  const float tlaMaxRaw = std::fmax(tlaOppRaw, tlaOwnRaw);
  const bool flexOwn = isNormal(bus.fadec_own_bus.selected_flex_temp_deg);  // a FLEX temperature is set
  const bool flexOpp = isNormal(bus.fadec_opp_bus.selected_flex_temp_deg);
  const bool facAlphaFloor = bit(logic.chosen_fac_bus.discrete_word_5, kFacWord5AlphaFloor) != 0;
  const auto& oppAts = bus.fmgc_opp_bus.ats_discrete_word;

  // --- TCAS latch -----------------------------------------------------------------------------------------------------
  // Set when TCAS mode engages; reset when it ends, or when, after 5 s in TCAS mode, a lever is pulled below 24°.
  // While it is set, alpha floor cannot engage.
  const bool tcasConfirmed = s.tcasActiveFor5s.update(lm.tcas_active, dt, true, kTcasConfirmS);
  const bool tcasLeverRetarded = tlaOpp < kTlaTcasRetardDeg || tlaOwn < kTlaTcasRetardDeg;
  const bool tcasSet = s.tcasEngagePulse.update(lm.tcas_active, true);
  const bool tcasReset = !lm.tcas_active || (tcasConfirmed && tcasLeverRetarded);
  const bool notInTcas = latchNotQ(tcasSet, tcasReset, s.tcasLatch.q);
  s.tcasLatch.update(tcasSet, tcasReset);

  // --- Alpha floor ----------------------------------------------------------------------------------------------------
  // Engages on the rising edge of the FAC alpha floor condition (outside the TCAS latch), and stays (TOGA LK once the
  // condition is gone) until the A/THR is disengaged.
  const bool alphaFloorPulse = s.alphaFloorRequestPulse.update(notInTcas && facAlphaFloor, true);
  const bool alphaFloor = s.alphaFloor.update(alphaFloorPulse, !engaged);

  // --- A/THR active and limited ---------------------------------------------------------------------------------------
  // Active (the A/THR controls the thrust) when alpha floor is on, or both levers are between IDLE and CL, or, with one
  // engine out and no FLEX temperature, both levers are above IDLE and at or below MCT.
  const bool bothInClimbRange =
      within(tlaOpp, kTlaIdleDeg, kTlaClimbDeg) && within(tlaOwn, kTlaIdleDeg, kTlaClimbDeg);
  const bool bothInMctRange = aboveUpTo(tlaOpp, kTlaIdleDeg, kTlaMctDeg) && aboveUpTo(tlaOwn, kTlaIdleDeg, kTlaMctDeg);
  const bool oneEngineRange = !flexOpp && !flexOwn && logic.one_engine_out && bothInMctRange;
  const bool active = engaged && (alphaFloor || bothInClimbRange || oneEngineRange);
  // Limited: active with a lever below the CL detent (below MCT with one engine out); the lever caps the thrust.
  const bool leverBelowMct = tlaOpp < kTlaLimitedOneEngineDeg || tlaOwn < kTlaLimitedOneEngineDeg;
  const bool leverBelowClimb = tlaOpp < kTlaLimitedDeg || tlaOwn < kTlaLimitedDeg;
  const bool limited = active && (leverBelowClimb || (logic.one_engine_out && leverBelowMct));

  // --- THR, SPEED/MACH and RETARD mode latches ------------------------------------------------------------------------
  // Each is set by the opposite FMGC's mode when this FMGC follows it, else by the engagement (rising edge, A/THR
  // engaged) of a vertical law that needs that thrust mode, not in alpha floor. It is reset when the A/THR disengages,
  // or when another thrust mode (or alpha floor) was active last frame and its own set condition has not been true for
  // 0.72 s.
  const bool ownCompute = !oppSync;
  // Fix (= personal/fg-final-athr-speed): not in FINAL, where the A/THR is in SPEED.
  const bool thrustLaw = law == Law::SPD_MACH || law == Law::SRS ||
                         (law == Law::VPATH && in.data.fms_inputs.requested_des_submode == fmgc_des_submode::VPATH_THRUST &&
                          !lm.final_des_active);
  const bool thrustLawEngaged = s.thrustLawPulse.update(thrustLaw && engaged, true);
  const bool thrustSet = (bit(oppAts, kOppAtsThrustMode) != 0 && oppSync) ||
                         (ownCompute && thrustLawEngaged && !alphaFloor);
  const bool thrustSetRecent = s.thrustSetRecent.update(thrustSet, dt, false, kModeTakeoverDelayS);
  const bool otherThanThrust = last.alpha_floor_mode_active || last.retard_mode_active || last.speed_mach_mode_active;
  const bool thrustMode = s.thrustMode.update(thrustSet, !engaged || (otherThanThrust && !thrustSetRecent));

  // SPEED: pitch flies an altitude, a V/S, an FPA, a G/S, the flare or a VPATH speed descent, or there is no vertical law
  // (except while RETARD on the ground).
  const bool speedLaw =
      (law == Law::NONE && (!last.retard_mode_active || !logic.on_ground)) || law == Law::ALT_HOLD ||
      law == Law::ALT_ACQ || law == Law::VS || law == Law::FPA || law == Law::GS || law == Law::FLARE ||
      (law == Law::VPATH &&
       (in.data.fms_inputs.requested_des_submode == fmgc_des_submode::VPATH_SPEED || lm.final_des_active));
  const bool speedLawEngaged = s.speedLawPulse.update(speedLaw && engaged, true);
  const bool speedSet = (bit(oppAts, kOppAtsSpeedMode) != 0 && oppSync) ||
                        (ownCompute && speedLawEngaged && !alphaFloor);
  const bool speedSetRecent = s.speedSetRecent.update(speedSet, dt, false, kModeTakeoverDelayS);
  const bool otherThanSpeed = last.alpha_floor_mode_active || last.retard_mode_active || last.thrust_mode_active;
  const bool speedMode = s.speedMode.update(speedSet, !engaged || (otherThanSpeed && !speedSetRecent));

  // RETARD: LAND mode with this FMGC's AP engaged at or below 40 ft RA (raw radio height, any SSM).
  const bool retardCondition = ap.lateral_modes.land_active && logic.ap_own_engaged &&
                               rawValue(logic.ra_computation_data.radio_height_ft) <= kRetardRadioHeightFt;
  const bool retardSet = (bit(oppAts, kOppAtsRetardMode) != 0 && oppSync) ||
                         (ownCompute && retardCondition && !alphaFloor);
  const bool retardSetRecent = s.retardSetRecent.update(retardSet, dt, false, kModeTakeoverDelayS);
  const bool otherThanRetard = last.alpha_floor_mode_active || last.thrust_mode_active || last.speed_mach_mode_active;
  const bool retardMode = s.retardMode.update(retardSet, !engaged || (otherThanRetard && !retardSetRecent));

  // Thrust target idle (THR IDLE rather than THR CLB/MCT): a descent mode is active (DES, OP DES, EXP DES, FINAL DES).
  const bool idleTarget = lm.des_active || lm.op_des_active || lm.exp_des_active || lm.final_des_active;

  // --- Lever message latches ------------------------------------------------------------------------------------------
  // Armed above the thrust reduction altitude, or once ALT*, ALT, V/S or FPA engages; disarmed on the ground, in
  // go-around below the thrust reduction altitude. LVR CLB with both engines, LVR MCT with one engine out.
  const double thrustReductionAltFt = in.data.fms_inputs.thrust_reduction_alt_ft;
  const float altitudeFt = logic.altitude_indicated_ft.Data;  // raw, any SSM
  const bool clbPhaseReached = altitudeFt >= thrustReductionAltFt || lm.alt_acq_active || lm.alt_hold_active ||
                               lm.vs_active || lm.fpa_active;
  const bool goAroundBelowThrRed =
      in.data.fms_inputs.fms_flight_phase == fmgc_flight_phase::Goaround && altitudeFt < thrustReductionAltFt;
  const bool oei = logic.one_engine_out;
  const bool lvrClbArmed =
      s.lvrClbArmed.update(clbPhaseReached && !oei, oei || logic.on_ground || goAroundBelowThrRed);
  const bool lvrMctArmed =
      s.lvrMctArmed.update(clbPhaseReached && oei, !oei || logic.on_ground || goAroundBelowThrRed);

  // --- FMA message ----------------------------------------------------------------------------------------------------
  // TCAS RA with the A/THR not active (levers out of the active range): a climbing RA (> 500 fpm) below VMAX - 20 kt
  // suppresses LVR CLB (keep the manual thrust); a level or descending RA asks for LVR CLB even before the latch.
  const float iasKt = valueOr(logic.adr_computation_data.airspeed_computed_kn, 0.0F);
  const float vmaxKt = valueOr(logic.chosen_fac_bus.v_max_kn, 0.0F);
  const bool belowVmaxMargin = iasKt <= vmaxKt - kVmaxMarginKt;
  const bool tcasManualThrust = lm.tcas_active && !active;
  const bool tcasClimbRaKeepThrust = belowVmaxMargin && ap.tcas_vs_target > kTcasRaClimbVsFpm && tcasManualThrust;
  const bool tcasLevelOrDescentRa = tcasManualThrust && ap.tcas_vs_target <= kTcasRaClimbVsFpm;
  const bool engagedNoAlphaFloor = engaged && !alphaFloor;
  const bool bothBelowClimbDetent = tlaOppRaw < kTlaClimbDetentLowDeg && tlaOwnRaw < kTlaClimbDetentLowDeg;
  const bool leverNotInClimbDetent =
      bothBelowClimbDetent || tlaOppRaw > kTlaClimbDetentHighDeg || tlaOwnRaw > kTlaClimbDetentHighDeg;
  const bool bothBelowMctDetent = tlaOppRaw < kTlaMctDetentLowDeg && tlaOwnRaw < kTlaMctDetentLowDeg;
  auto inClimbDetent = [](float tla) { return strictlyBetween(tla, kTlaClimbDetentLowDeg, kTlaClimbDetentHighDeg); };
  auto outOfClimbDetent = [](float tla) { return tla < kTlaClimbDetentLowDeg || tla > kTlaClimbDetentHighDeg; };
  const bool asymmetric = (inClimbDetent(tlaOppRaw) && outOfClimbDetent(tlaOwnRaw)) ||
                          (inClimbDetent(tlaOwnRaw) && outOfClimbDetent(tlaOppRaw));

  athr_fma_message message = athr_fma_message::NONE;
  if (engagedNoAlphaFloor && ((lvrClbArmed && !tcasClimbRaKeepThrust) || tcasLevelOrDescentRa) &&
      leverNotInClimbDetent) {
    message = athr_fma_message::LVR_CLB;
  } else if (engagedNoAlphaFloor && lvrMctArmed && bothBelowMctDetent) {
    message = athr_fma_message::LVR_MCT;
  } else if (engagedNoAlphaFloor && !oei && asymmetric) {
    message = athr_fma_message::LVR_ASYM;
  }

  // --- FMA mode -------------------------------------------------------------------------------------------------------
  // Engaged but not active: manual thrust, by the highest lever. Active: the thrust mode; THR mode shows the lever
  // detent the thrust is limited to (or IDLE in a descent mode); alpha floor shows A.FLOOR while the FAC condition
  // remains, then TOGA LK.
  const bool flexSet = flexOpp || flexOwn;
  const bool inMctDetent = strictlyBetween(tlaMaxRaw, kTlaMctDetentLowDeg, kTlaMctDetentHighDeg);
  const bool mach = ap.mach_control_active;
  const bool manual = engaged && !active;
  const bool auto_ = engaged && active;
  athr_fma_mode mode = athr_fma_mode::NONE;
  if (manual && tlaMaxRaw > kTlaTogaDeg) {
    mode = athr_fma_mode::MAN_TOGA;
  } else if (manual && inMctDetent && flexSet) {
    mode = athr_fma_mode::MAN_FLEX;
  } else if (manual && inMctDetent && !flexSet) {
    mode = athr_fma_mode::MAN_MCT;
  } else if (manual && tlaMaxRaw > kTlaClimbDetentLowDeg) {
    mode = athr_fma_mode::MAN_THR;
  } else if (auto_ && speedMode && !mach) {
    mode = athr_fma_mode::SPEED;
  } else if (auto_ && speedMode && mach) {
    mode = athr_fma_mode::MACH;
  } else if (auto_ && thrustMode && tlaMaxRaw > kTlaMctDetentLowDeg && !idleTarget) {
    mode = athr_fma_mode::THR_MCT;
  } else if (auto_ && thrustMode && tlaMaxRaw > kTlaClimbDetentLowDeg && !idleTarget) {
    mode = athr_fma_mode::THR_CLB;
  } else if (auto_ && thrustMode && tlaMaxRaw < kTlaClimbDeg && !idleTarget) {
    mode = athr_fma_mode::THR_LVR;  // (always true here unless both raw lever angles are NaN)
  } else if (auto_ && ((thrustMode && idleTarget) || retardMode)) {
    mode = athr_fma_mode::THR_IDLE;
  } else if (auto_ && alphaFloor && facAlphaFloor) {
    mode = athr_fma_mode::A_FLOOR;
  } else if (auto_ && alphaFloor && !facAlphaFloor) {
    mode = athr_fma_mode::TOGA_LK;
  }

  r.out.athr_active = active;
  r.out.athr_limited = limited;
  r.out.alpha_floor_mode_active = alphaFloor;
  r.out.thrust_mode_active = thrustMode;
  r.out.thrust_target_idle = idleTarget;
  r.out.speed_mach_mode_active = speedMode;
  r.out.retard_mode_active = retardMode;
  r.out.fma_mode = mode;
  r.out.fma_message = message;
  for (int f = AthrActive; f <= FmaMessage; f++) {
    r.rewritten[f] = true;
  }
}

}  // namespace fmgc::readable::athr
