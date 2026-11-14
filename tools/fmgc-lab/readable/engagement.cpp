// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// AP / FD / A/THR engagement, opposite-FMGC priority and synchronisation, FD/AP flags.
// Spec: docs/modes/engagement.md. Generated: FmgcComputer.cpp 1380–1559 (stage 1) and 3335–3403 (stage 2); outputs at
// 3357, 3402–3403, 3869–3876, 4020–4021, 4031–4032, 4040; init 454–459 and 523–540; end-of-step copies 5108–5109.
//
// Generated temporaries are heavily reused; the conditioned inputs this block reads were matched to the logic bus
// fields they are copied to (e.g. rtb_y_hy = logic.on_ground, rtb_y_ew = logic.gnd_eng_stop_flt_5s, rtb_y_er =
// logic.double_adr_failure, rtb_adr3Invalid = logic.double_ir_failure, rtb_AND10_k = logic.fac_lg_data_failure,
// raOwnInvalid = logic.dual_ra_failure, rtb_irOwnInvalid (at 1462) = logic.fac_rudder_control_failure).

#include <cmath>

#include "engagement.h"

namespace fmgc::readable::engagement {

const char* fieldName(int field) {
  static const char* names[kFieldCount] = {
      "ap_fd_athr_common_condition", "ap_fd_common_condition", "fd_own_engaged", "ap_inop", "ap_own_engaged",
      "athr_inop", "athr_own_engaged", "fmgc_opp_priority", "fmgc_opp_mode_sync", "any_ap_fd_engaged",
      "lateral_mode_reset", "longitudinal_mode_reset", "fd_auto_disengage", "vs_target_not_held",
      "ap_fd_mode_reversion", "pitch_fd_bars_flashing"};
  return field >= 0 && field < kFieldCount ? names[field] : "?";
}

const char* conditionName(int c) {
  static const char* names[kConditionCount] = {
      "ap_pb_push", "ap_pb_rejected", "ap_envelope_reject", "ap_pb_disengage", "ap_on_ground", "ap_opp_engage_reset",
      "ap_land_ga_end_reset", "ap_ground_toga_reset", "ap_engine_start_reset", "ap_request_drop", "dual_ap",
      "fd_transfer", "rollout_off_course", "athr_inhibited", "athr_pb_engage", "athr_pb_low_ra", "athr_srs_engage",
      "athr_alpha_floor_engage", "athr_tcas_engage", "athr_pb_disengage", "athr_levers_idle", "athr_ecu_disengage",
      "athr_opp_sync_lost", "unit_2", "opp_unhealthy", "opp_priority_unit_1", "opp_priority_unit_2",
      "mode_reset_engine_start", "fd_disengage_climb", "fd_disengage_descent", "vs_not_held_low", "vs_not_held_high"};
  return c >= 0 && c < kConditionCount ? names[c] : "?";
}

namespace {

using blocks::a429::bit;
using blocks::a429::valueOr;

constexpr double kCommonConfirmS = 0.72;         // common conditions must hold this long
constexpr float kGsTrackFmLossMaxRaFt = 700;     // in G/S, an FM failure is tolerated below this radio height
constexpr double kRolloutHeadingDevDeg = 20;     // ROLL OUT: heading this far from the memorised runway course ...
constexpr double kRolloutHeadingDevS = 3;        // ... removes the AP/FD common condition for 3 s (not retriggered)
constexpr double kApRequestDropS = 0.2;          // the AP disengages 0.2 s after its engage request is reset
constexpr float kApEngageMinPitchDeg = -10;      // AP engagement attitude envelope
constexpr float kApEngageMaxPitchDeg = 22;
constexpr float kApEngageMaxBankDeg = 40;
constexpr double kTogaLeverDeg = 36.7;           // on the ground, a lever above this (FLX/MCT-TOGA) disengages the AP
constexpr double kAthrInstinctiveLockS = 15;     // instinctive disconnect held this long: A/THR inoperative
constexpr double kAthrSrsConfirmS = 0.9;         // SRS takeoff/GA engaged this long arms the A/THR
constexpr double kLeverIdleDeg = 2.6;            // both levers below this: A/THR disconnects
constexpr float kAthrPbMinRaFt = 100;            // A/THR pushbutton engages only above this radio height
constexpr double kAthrEcuConfirmS = 0.9;         // A/THR not engaged in an ECU this long: disconnects
constexpr float kFdDisengageVmaxMarginKt = 4;    // FD auto disengage above VMAX + 4 kt in CLB/OP CLB/EXP CLB ...
constexpr float kFdDisengageVlsMarginKt = -2;    // ... or below VLS - 2 kt in DES/OP DES/EXP DES
constexpr float kVsNotHeldMarginKt = 3;          // V/S-FPA target not held within 3 kt of VLS/VMAX ...
constexpr double kVsNotHeldVsFpm = 50;           // ... when V/S deviates from the target by more than 50 (see doc)
constexpr double kVsNotHeldFpaDeg = 0.1;         // ... or FPA by more than 0.1°
constexpr double kModeReversionPulseS = 1;       // AP/FD mode reversion (triple click) pulse
constexpr double kPitchBarsFlashS = 10;          // pitch FD bars flash 10 s

// Bits (1-based)
constexpr int kFcuWord2Fd1Off = 26;   // FCU discrete word 2: FD 1 (captain) off
constexpr int kFcuWord2Fd2Off = 27;   //                      FD 2 (first officer) off
constexpr int kFacWord5AlphaFloor = 29;  // FAC discrete word 5: alpha floor
constexpr int kEcuWord3AthrEngaged = 19;  // ECU status word 3: A/THR engaged in the ECU

}  // namespace

void Engagement::updateEngagement(const Stage1Inputs& in, Outputs& out) {
  const auto& di = in.in.discrete_inputs;
  const auto& bus = in.in.bus_inputs;
  const auto& logic = in.logic;
  const auto& last = in.last;
  const double dt = in.dt;
  State& s = s_;

  const bool landOrGaLast = last.lateral_modes.land_active || last.longitudinal_modes.pitch_goaround_active;
  const float radioHeightFt = logic.ra_computation_data.radio_height_ft.Data;  // raw (2500 ft when both RAs failed)

  // --- AP/FD/A/THR common condition: no double ADR or IR failure, FAC landing gear data and FM valid (except in LAND
  // or SRS GA, or FM lost in G/S below 700 ft), for 0.72 s.
  const bool apFdAthrCommonCond =
      !logic.double_adr_failure && !logic.double_ir_failure && (!logic.fac_lg_data_failure || landOrGaLast) &&
      (in.in.fms_inputs.fm_valid || landOrGaLast ||
       (last.longitudinal_modes.gs_trk_active && radioHeightFt < kGsTrackFmLossMaxRaFt));
  const bool apFdAthrCommon = s.apFdAthrCommon.update(apFdAthrCommonCond, dt, true, kCommonConfirmS);

  // --- AP/FD common condition: FAC weights (except LAND/GA) and speeds (except LAND) valid, a radio altimeter (unless
  // nothing ILS is armed or engaged), FCU healthy (except LAND/GA), and not in ROLL OUT more than 20° off the runway
  // course, for 0.72 s.
  const float headingTrueDeg = valueOr(logic.ir_computation_data.heading_true_deg, 0.0F);
  const bool rolloutOffCourse = last.lateral_modes.rollout_submode_active &&
                                std::abs(logic.rwy_hdg_memo - headingTrueDeg) > kRolloutHeadingDevDeg;
  const bool rolloutOffCoursePulse = s.rolloutHeadingDev.update(rolloutOffCourse, dt, true, false, kRolloutHeadingDevS);
  const bool noIlsModeLast = !last.armed_modes.land_armed && !last.longitudinal_modes.gs_capt_active &&
                             !last.longitudinal_modes.gs_trk_active && !last.lateral_modes.land_active;
  const bool apFdCommonCond = (!logic.fac_weights_failure || landOrGaLast) &&
                              (!logic.fac_speeds_failure || last.lateral_modes.land_active) &&
                              (!logic.dual_ra_failure || noIlsModeLast) && (!logic.fcu_failure || landOrGaLast) &&
                              !rolloutOffCoursePulse;
  const bool apFdCommon = s.apFdCommon.update(apFdCommonCond, dt, true, kCommonConfirmS);

  // --- FD: engaged with its FCU FD pushbutton on, or with it off when the opposite FD pushbutton is on and the
  // opposite FMGC's FD is not engaged (this FMGC then drives the opposite side's FD).
  const bool fd1Off = bit(bus.fcu_bus.fcu_discrete_word_2, kFcuWord2Fd1Off) != 0;
  const bool fd2Off = bit(bus.fcu_bus.fcu_discrete_word_2, kFcuWord2Fd2Off) != 0;
  const bool ownFdOff = di.is_unit_1 ? fd1Off : fd2Off;
  const bool oppFdOff = di.is_unit_1 ? fd2Off : fd1Off;
  const bool fdOwnEngaged = apFdAthrCommon && apFdCommon && (!ownFdOff || (!oppFdOff && !di.fd_opp_engaged));

  // --- AP
  const bool apPb = s.apPbPush.update(di.fcu_ap_button, true);
  const bool engineStart = s.apEngineStart.update(logic.engine_running, true);
  const bool oppApEngages = s.apOppEngage.update(di.ap_opp_engaged, true);
  const bool noLandOrGaLast = !last.armed_modes.land_armed && !last.lateral_modes.land_active &&
                              !last.longitudinal_modes.pitch_goaround_active;
  const bool landOrGaEnded = s.apLandGaLost.update(noLandOrGaLast, true);
  const bool oppLeverToga = valueOr(bus.fadec_opp_bus.selected_tla_deg, 0.0F) >= kTogaLeverDeg;
  const double ownLeverDeg = valueOr(bus.fadec_own_bus.selected_tla_deg, 0.0F);
  const float vmaxKt = valueOr(logic.chosen_fac_bus.v_max_kn, 0.0F);
  const float iasKt = valueOr(logic.adr_computation_data.airspeed_computed_kn, 0.0F);
  const float vlsKt = valueOr(logic.chosen_fac_bus.v_ls_kn, 0.0F);
  const float pitchDeg = valueOr(logic.ir_computation_data.pitch_angle_deg, 0.0F);
  const float rollDeg = valueOr(logic.ir_computation_data.roll_angle_deg, 0.0F);

  // AP condition: no instinctive disconnect, at least one ELAC authorising the AP, FAC rudder control.
  const bool apCondition = !di.ap_instinctive_disc && (!di.elac_own_ap_disc || !di.elac_opp_ap_disc) &&
                           !logic.fac_rudder_control_failure;
  const bool apInop = !apFdAthrCommon || !apFdCommon || !apCondition;

  const bool apEngagedLast = s.apEngaged.q;
  const bool inEngageEnvelope = vmaxKt >= iasKt && iasKt >= vlsKt && pitchDeg >= kApEngageMinPitchDeg &&
                                pitchDeg <= kApEngageMaxPitchDeg && std::abs(rollDeg) <= kApEngageMaxBankDeg;
  const bool requestSet = apPb && !apEngagedLast && logic.gnd_eng_stop_flt_5s && apFdCommon && apFdAthrCommon &&
                          apCondition && inEngageEnvelope;
  const bool requestReset =
      apInop || (apPb && apEngagedLast) || di.ap_instinctive_disc ||
      // the opposite AP engages (single AP outside LAND/GA), or on FMGC 2 LAND/GA ends with both APs engaged
      (oppApEngages && noLandOrGaLast) || (di.ap_opp_engaged && landOrGaEnded && !di.is_unit_1) ||
      // on the ground: SRS GA or a lever at FLX/MCT-TOGA, or the first engine start
      (logic.on_ground && (last.longitudinal_modes.pitch_goaround_active || oppLeverToga ||
                           ownLeverDeg >= kTogaLeverDeg)) ||
      (engineStart && logic.on_ground);
  const bool request = s.apEngageRequest.update(requestSet, requestReset);
  const bool requestHeld = s.apRequestHeld.update(request, dt, false, kApRequestDropS);
  const bool apEngaged = s.apEngaged.update(apPb && request, !requestHeld || di.ap_instinctive_disc);

  // --- A/THR
  const bool instinctive15s = s.athrInstinctive15s.update(di.athr_instinctive_disc, dt, true, kAthrInstinctiveLockS);
  const bool athrInhibited = s.athrInhibited.update(instinctive15s, false);  // reset: Constant_Value_e = 0
  const bool athrPb = s.athrPbPush.update(di.fcu_athr_button, true);
  const bool srs09s = s.athrSrs09s.update(
      last.longitudinal_modes.pitch_goaround_active || last.longitudinal_modes.pitch_takeoff_active, dt, true,
      kAthrSrsConfirmS);
  const bool srsEngages = s.athrSrsEdge.update(srs09s, true);
  const bool leversIdle = static_cast<double>(valueOr(bus.fadec_opp_bus.selected_tla_deg, 0.0F)) < kLeverIdleDeg &&
                          static_cast<double>(valueOr(bus.fadec_own_bus.selected_tla_deg, 0.0F)) < kLeverIdleDeg;
  const bool leversToIdle = s.athrLeversIdle.update(leversIdle, true);
  const bool alphaFloor = s.athrAlphaFloor.update(bit(logic.chosen_fac_bus.discrete_word_5, kFacWord5AlphaFloor) != 0,
                                                  true);
  const bool oppSyncWithoutAthr = s.athrOppSyncLost.update(last.fmgc_opp_mode_sync && !di.athr_opp_engaged, true);
  const bool tcasEngages = s.athrTcas.update(last.longitudinal_modes.tcas_active, true);
  const bool athrEngagedLast = s.athrEngaged.q;
  const bool ecuOppEngaged = bit(bus.fadec_opp_bus.ecu_status_word_3, kEcuWord3AthrEngaged) != 0;
  const bool ecuOwnEngaged = bit(bus.fadec_own_bus.ecu_status_word_3, kEcuWord3AthrEngaged) != 0;
  const bool ecuNotEngaged =
      s.athrEcuNotEngaged.update((!ecuOppEngaged || !ecuOwnEngaged) && athrEngagedLast, dt, true, kAthrEcuConfirmS);

  // A/THR possible: a speed target (managed or selected) last frame, not inhibited, FCU and FAC speeds valid.
  const bool athrPossible = (last.manual_spd_control_active || last.auto_spd_control_active) && !athrInhibited &&
                            !logic.fcu_failure && !logic.fac_speeds_failure;
  const bool athrInop = !apFdAthrCommon || !athrPossible;
  const bool athrSet = apFdAthrCommon && athrPossible &&
                       ((athrPb && (radioHeightFt > kAthrPbMinRaFt || logic.dual_ra_failure)) || srsEngages ||
                        alphaFloor || tcasEngages);
  // The pushbutton disconnect is gated by a constant bus (rtP_fmgc_ap_fd_logic_output_MATLABStruct, all false)
  // where the LAND mode was meant: a push disconnects whatever the mode.
  constexpr bool kLandFromConstantBus = false;
  const bool athrReset = athrInop || oppSyncWithoutAthr || (athrEngagedLast && athrPb && !kLandFromConstantBus) ||
                         di.athr_instinctive_disc || leversToIdle || ecuNotEngaged;
  const bool athrEngaged = s.athrEngaged.update(athrSet, athrReset);

  // --- Opposite FMGC priority: the opposite FMGC is master. FMGC 1 yields when it has no AP and the opposite FMGC has
  // its AP, or its FD where FMGC 1 has no FD and no A/THR beyond the opposite's. FMGC 2 yields unless it has the only
  // AP (or the only FD, or the only A/THR) and only if it has something engaged (or the opposite FMGC is unhealthy).
  const bool noOwnAp = !apEngaged;
  const bool apOpp = di.ap_opp_engaged, fdOpp = di.fd_opp_engaged, athrOpp = di.athr_opp_engaged;
  const bool unit1Yields = noOwnAp && (apOpp || !fdOwnEngaged) && (apOpp || fdOpp || !athrEngaged) &&
                           (apOpp || fdOpp || athrOpp);
  const bool unit2Yields = (noOwnAp || apOpp) && (apOpp || fdOpp || !fdOwnEngaged) &&
                           (apOpp || fdOpp || athrOpp || !athrEngaged) &&
                           (apEngaged || fdOwnEngaged || athrEngaged || !di.fmgc_opp_healthy);
  const bool oppPriority = (!di.is_unit_1 || unit1Yields) && (di.is_unit_1 || unit2Yields);
  // Synchronised to the opposite FMGC's modes when it has priority, except with both APs engaged.
  const bool oppModeSync = oppPriority && (noOwnAp || !apOpp);

  const bool anyApFd = apEngaged || apOpp || fdOwnEngaged || fdOpp;
  const bool engineStartReset = s.resetEngineStart.update(logic.engine_running, true);
  const bool modeReset = !anyApFd || (engineStartReset && logic.on_ground);

  // coverage only
  cond[CondApPbPush] = apPb;
  cond[CondApPbRejected] = apPb && !apEngagedLast && !apEngaged;
  cond[CondApEnvelopeReject] = apPb && !apEngagedLast && !inEngageEnvelope && logic.gnd_eng_stop_flt_5s && apFdCommon &&
                               apFdAthrCommon && apCondition;
  cond[CondApPbDisengage] = apPb && apEngagedLast;
  cond[CondApOnGround] = apEngaged && logic.on_ground;
  cond[CondApOppEngageReset] = oppApEngages && noLandOrGaLast && apEngagedLast;
  cond[CondApLandGaEndReset] = di.ap_opp_engaged && landOrGaEnded && !di.is_unit_1;
  cond[CondApGroundTogaReset] =
      logic.on_ground && (last.longitudinal_modes.pitch_goaround_active || oppLeverToga || ownLeverDeg >= kTogaLeverDeg);
  cond[CondApEngineStartReset] = engineStart && logic.on_ground;
  cond[CondApRequestDrop] = apEngaged && !request;
  cond[CondDualAp] = apEngaged && di.ap_opp_engaged;
  cond[CondFdTransfer] = fdOwnEngaged && ownFdOff;
  cond[CondRolloutOffCourse] = rolloutOffCoursePulse;
  cond[CondAthrInhibited] = athrInhibited;
  cond[CondAthrPbEngage] = athrEngaged && !athrEngagedLast && athrPb;
  cond[CondAthrPbLowRa] = athrPb && !athrEngagedLast && !(radioHeightFt > kAthrPbMinRaFt || logic.dual_ra_failure);
  cond[CondAthrSrsEngage] = athrEngaged && !athrEngagedLast && srsEngages;
  cond[CondAthrAlphaFloorEngage] = athrEngaged && !athrEngagedLast && alphaFloor;
  cond[CondAthrTcasEngage] = athrEngaged && !athrEngagedLast && tcasEngages;
  cond[CondAthrPbDisengage] = athrEngagedLast && athrPb;
  cond[CondAthrLeversIdle] = athrEngagedLast && leversToIdle;
  cond[CondAthrEcuDisengage] = athrEngagedLast && ecuNotEngaged;
  cond[CondAthrOppSyncLost] = athrEngagedLast && oppSyncWithoutAthr;

  out[ApFdAthrCommonCondition] = apFdAthrCommon;
  out[ApFdCommonCondition] = apFdCommon;
  out[FdOwnEngaged] = fdOwnEngaged;
  out[ApInop] = apInop;
  out[ApOwnEngaged] = apEngaged;
  out[AthrInop] = athrInop;
  out[AthrOwnEngaged] = athrEngaged;
  out[FmgcOppPriority] = oppPriority;
  out[FmgcOppModeSync] = oppModeSync;
  out[AnyApFdEngaged] = anyApFd;
  cond[CondUnit2] = !di.is_unit_1;
  cond[CondOppUnhealthy] = !di.fmgc_opp_healthy;
  cond[CondOppPriorityUnit1] = oppPriority && di.is_unit_1;
  cond[CondOppPriorityUnit2] = oppPriority && !di.is_unit_1;
  cond[CondModeResetEngineStart] = engineStartReset && logic.on_ground && anyApFd;
  out[LateralModeReset] = modeReset;
  out[LongitudinalModeReset] = modeReset;
}

void Engagement::updateFdFlags(const Stage2Inputs& in, Outputs& out) {
  const auto& logic = in.logic;
  const auto& m = in.modes.longitudinal_modes;
  const auto& fcu = in.in.bus_inputs.fcu_bus;
  const double dt = in.dt;
  State& s = s_;

  const bool apEngaged = out[ApOwnEngaged];
  const float vmaxKt = valueOr(logic.chosen_fac_bus.v_max_kn, 0.0F);
  const float iasKt = valueOr(logic.adr_computation_data.airspeed_computed_kn, 0.0F);
  const float vlsKt = valueOr(logic.chosen_fac_bus.v_ls_kn, 0.0F);

  // --- FD auto disengagement: with the FD only (no AP) and the A/THR engaged, in a climb mode above VMAX + 4 kt or a
  // descent mode below VLS - 2 kt.
  const bool climbMode = m.clb_active || m.op_clb_active || m.exp_clb_active;
  const bool descentMode = m.des_active || m.op_des_active || m.exp_des_active;
  const bool speedLimit = (climbMode && vmaxKt + kFdDisengageVmaxMarginKt < iasKt) ||
                          (iasKt < vlsKt + kFdDisengageVlsMarginKt && descentMode);
  const bool fdAutoDisengage = speedLimit && !apEngaged && out[AthrOwnEngaged] && out[FdOwnEngaged];

  // --- V/S or FPA target not held: AP in V/S or FPA within 3 kt of VLS while the aircraft is below the target, or of
  // VMAX while above it. V/S compares the IR inertial vertical speed word with the FCU V/S (threshold 50), FPA the IR
  // flight path angle with the FCU FPA (0.1°).
  float targetError;
  double threshold;
  if (m.vs_active) {
    targetError = valueOr(logic.ir_computation_data.inertial_vertical_speed_ft_s, 0.0F) -
                  valueOr(fcu.selected_vz_ft_min, 0.0F);
    threshold = kVsNotHeldVsFpm;
  } else {
    targetError = valueOr(logic.ir_computation_data.flight_path_angle_deg, 0.0F) - valueOr(fcu.selected_fpa_deg, 0.0F);
    threshold = kVsNotHeldFpaDeg;
  }
  const bool apInVsFpa = apEngaged && (m.vs_active || m.fpa_active);
  const bool vsTargetNotHeld =
      (apInVsFpa && iasKt < vlsKt + kVsNotHeldMarginKt && targetError < -threshold) ||
      (apInVsFpa && iasKt > vmaxKt - kVsNotHeldMarginKt && targetError > threshold);

  // --- AP/FD mode reversion (triple click), 1 s: any automatic reversion with an AP or FD engaged.
  const bool anyApFd = out[AnyApFdEngaged];
  const bool reversion = vsTargetNotHeld || fdAutoDisengage || in.modes.longitudinal_mode_reversion_op_clb ||
                         in.modes.longitudinal_mode_reversion_vs || in.modes.lateral_mode_reversion;
  const bool modeReversion = s.modeReversion1s.update(anyApFd && reversion, dt, true, false, kModeReversionPulseS);

  // --- Pitch FD bars flash 10 s after a reversion to V/S, or after the first AP/FD engagement unless TCAS is engaged.
  const bool apFdEngages = s.pitchBarsApFdEngage.update(anyApFd, true) && !m.tcas_active;
  const bool pitchBarsFlashing =
      s.pitchBarsFlash10s.update(in.modes.longitudinal_mode_reversion_vs || apFdEngages, dt, true, false,
                                 kPitchBarsFlashS);

  cond[CondFdDisengageClimb] = climbMode && vmaxKt + kFdDisengageVmaxMarginKt < iasKt;
  cond[CondFdDisengageDescent] = iasKt < vlsKt + kFdDisengageVlsMarginKt && descentMode;
  cond[CondVsNotHeldLow] = apInVsFpa && iasKt < vlsKt + kVsNotHeldMarginKt;
  cond[CondVsNotHeldHigh] = apInVsFpa && iasKt > vmaxKt - kVsNotHeldMarginKt;
  out[FdAutoDisengage] = fdAutoDisengage;
  out[VsTargetNotHeld] = vsTargetNotHeld;
  out[ApFdModeReversion] = modeReversion;
  out[PitchFdBarsFlashing] = pitchBarsFlashing;
}

}  // namespace fmgc::readable::engagement
