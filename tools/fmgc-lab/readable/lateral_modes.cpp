// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Lateral modes: RWY (LOC submode), RWY TRK, ROLL OUT, ALIGN, GA TRK, HDG/TRK and the lateral mode reversion, NAV
// armed, NAV, LOC armed, LOC*, LOC, roll FD bars flashing, HDG/TRK preset, any lateral mode engaged, lateral law
// selection. LAND armed / LAND live in vertical_approach.cpp.
// Spec: docs/modes/lateral.md. Generated: FmgcComputer.cpp 1722–1793 (RWY, RWY TRK), 1894–1912 (ROLL OUT, ALIGN),
// 2026–2404 (GA TRK, HDG/TRK, NAV, LOC), 3392–3404 (roll FD bars), 3405–3434 (preset), 3736–3798 (law, any mode).
//
// Conditions read the previous frame's outputs (`f.last`) except where a signal updated earlier in this step is named
// (`s[...]`): NAV armed reads this frame's FINAL armed, the preset and the law selection read this frame's modes.

#include <cmath>

#include "vertical.h"

namespace fmgc::readable {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::rawValue;
using blocks::a429::valueOr;

constexpr double kOtherModeTakeoverDelayS = 0.72;  // another mode replaces this one only after this
constexpr double kPulseS = 0.72;                    // edge-triggered pulses (MTrigNode) last this long
constexpr double kMinFlapHandle = 1;                // flap handle index: not flaps 0

// RWY
constexpr double kRwyTakeoffConfirmS = 0.9;      // SRS takeoff engaged for this long
constexpr double kRwyOnGroundS = 30;             // on ground for this long
constexpr double kRwyMaxHeadingDiffDeg = 20;     // |ILS course − magnetic heading|, NOT wrapped at 360°
constexpr double kRwyMaxLocDevDeg = 0.4;         // localizer deviation
constexpr bool kRwyLocSubmodeEnabled = true;     // Constant_Value_j

// RWY TRK
constexpr double kRwyTrkRadioHeightFt = 30;      // RWY TRK engages when the radio height reaches 30 ft ...
constexpr double kRwyTrkTriggerS = 0.9;          // ... within this pulse

// ROLL OUT, ALIGN
constexpr double kRollOutRadioHeightFt = 50;
constexpr double kRollOutConfirmS = 0.36;
constexpr double kAlignRadioHeightFt = 30;

// HDG/TRK, NAV
constexpr double kGaMinRadioHeightFt = 100;      // in GA TRK, HDG/TRK pull and NAV engage only above 100 ft RA
constexpr double kNavMinRadioHeightFt = 30;      // NAV engages above 30 ft RA (or with both RA failed)

// LOC armed (LOC push)
constexpr double kLocArmMinRadioHeightFt = 400;

// LOC* (degrees)
constexpr double kLocCaptureMaxBankCmdDeg = 5;     // capture bank command large enough to override the roll check
constexpr float kLocCaptureMaxCourseErrorDeg = 115;  // no capture when the course error is this large or more
constexpr float kLocCaptureMinInterceptDeg = 25;     // large intercept angle: capture from 10° of deviation...
constexpr float kLocCaptureWideDevDeg = 10;          // ...when the capture bank command is reached
constexpr double kLocCaptureDevDeg = 1.92;           // otherwise within 1.92°...
constexpr float kLocCaptureCloseInterceptDeg = 15;   // ...and either the bank command is reached, or a small
constexpr double kLocCaptureCloseDevDeg = 1.1;       // intercept angle (< 15°) within 1.1°

// LOC
constexpr double kLocTrackDevDeg = 0.16;
constexpr double kLocTrackConfirmS = 10;

// Roll FD bars
constexpr double kRollBarsFlashS = 10;

// HDG/TRK preset
constexpr double kPresetMaxRadioHeightFt = 30;

constexpr bool kLocBackCourseSelected = false;  // rtP_fmgc_ap_fd_logic_output_MATLABStruct.loc_bc_selection

// FCU discrete words
constexpr int kFcuWord1HdgVsSelected = 24;  // HDG-V/S selected (else TRK-FPA)
constexpr int kFcuWord2HdgKnobPushed = 11;
constexpr int kFcuWord2HdgKnobPulled = 12;
constexpr int kFcuWord2LocPushed = 13;

// Opposite-FMGC discrete words (the mode this FMGC copies when it is the slave)
constexpr int kOppWord2Rwy = 11;
constexpr int kOppWord2Nav = 12;
constexpr int kOppWord2LocCapture = 13;
constexpr int kOppWord2LocTrack = 14;
constexpr int kOppWord2GaTrk = 15;
constexpr int kOppWord2Hdg = 16;
constexpr int kOppWord2Trk = 17;
constexpr int kOppWord2RwyLoc = 20;
constexpr int kOppWord2RwyTrk = 23;
constexpr int kOppWord3NavArmed = 14;
constexpr int kOppWord3LocArmed = 16;

const auto& oppBus(const Frame& f) {
  return f.in.bus_inputs.fmgc_opp_bus;
}

bool latch(bool set, bool reset, bool q) {
  return reset ? false : (set ? true : q);
}

bool fcuWord2Bit(const Frame& f, int n) {
  return bit(f.in.bus_inputs.fcu_bus.fcu_discrete_word_2, n) != 0;
}

/** Selected radio height (ft), raw: 2500 ft when both radio altimeters failed. */
float radioHeight(const Frame& f) {
  return rawValue(f.ext.logic->ra_computation_data.radio_height_ft);
}

template <typename T>
int sign(T x) {
  return x < 0 ? -1 : (x > 0 ? 1 : 0);
}

/**
 * Any lateral mode active last frame other than `except`. RWY and RWY TRK are the two submodes of RWY: for each, the
 * other submode counts (not rwy_active). For HDG/TRK, neither HDG nor TRK counts.
 */
bool anyOtherLateralModeLast(const base_fmgc_ap_fd_logic_outputs& last, Signal except) {
  const auto& l = last.lateral_modes;
  auto active = [&](Signal s, bool v) { return s != except && v; };
  bool rwy = except == Signal::RwyLoc ? l.rwy_trk_submode_active
                                      : (except == Signal::RwyTrk ? l.rwy_loc_submode_active : l.rwy_active);
  return rwy || active(Signal::GaTrk, l.roll_goaround_active) || active(Signal::Nav, l.nav_active) ||
         active(Signal::LocCapture, l.loc_cpt_active) || active(Signal::LocTrack, l.loc_trk_active) ||
         active(Signal::HdgTrk, l.hdg_active) || active(Signal::HdgTrk, l.trk_active) ||
         active(Signal::Land, l.land_active);
}

/**
 * ILS course (memorised runway heading) minus magnetic heading, wrapped to [-180, 180] (deg): positive when the course
 * is to the right. Single-precision as generated (2267–2278).
 */
float courseError(float heading, double courseMemo) {
  float d = std::fmod((heading - static_cast<float>(courseMemo + 360.0)) + 360.0F, 360.0F);  // heading − course
  if (d == 0.0F) {
    d = 0.0F;
  } else if (d < 0.0F) {
    d += 360.0F;
  }
  float e = std::fmod(360.0F - d, 360.0F);
  if (d < e) {
    e = -d;
  }
  return e;
}

/**
 * The LOC capture law's bank command (last step) towards the course has reached the aircraft's bank: same direction
 * and at least as large, or larger than 5° while the aircraft is within 5° of wings level or banked the other way.
 */
bool captureBankReached(float courseErr, double phiLocCmd, float roll) {
  if (sign(courseErr) != sign(phiLocCmd)) {
    return false;
  }
  if (std::abs(phiLocCmd) > kLocCaptureMaxBankCmdDeg &&
      (std::abs(roll) <= static_cast<float>(kLocCaptureMaxBankCmdDeg) || sign(phiLocCmd) != sign(roll))) {
    return true;
  }
  return std::abs(phiLocCmd) >= std::abs(roll) && sign(phiLocCmd) == sign(roll);
}

}  // namespace

void VerticalModeLogic::resetLateral(Signals& s) {
  for (Signal id : {Signal::RwyLoc, Signal::RwyTrk, Signal::RwyActive, Signal::RollOut, Signal::Align, Signal::GaTrk,
                    Signal::LateralReversion, Signal::HdgTrk, Signal::HdgActive, Signal::TrkActive, Signal::NavArmed,
                    Signal::Nav, Signal::LocArmed, Signal::LocCapture, Signal::LocTrack, Signal::RollFdBarsFlashing,
                    Signal::HdgTrkPreset, Signal::AnyLateralMode, Signal::ActiveLateralLaw, Signal::LocBcSelection}) {
    s[id] = false;
  }
  s.activeLateralLaw = lateral_law::NONE;
  lateral_ = LateralModesState{};
}

bool VerticalModeLogic::updateLateral(Signal id, const Frame& f, Signals& s) {
  const auto& lastLat = f.last.lateral_modes;
  const auto& lastVert = f.last.longitudinal_modes;
  const auto& lastArmed = f.last.armed_modes;
  const auto& logic = *f.ext.logic;
  const auto& fms = f.in.fms_inputs;
  const bool oppSync = f.ext.oppModeSync;
  const bool ownCompute = !oppSync;
  auto& l = lateral_;
  const bool locCapturedLast = lastLat.loc_cpt_active || lastLat.loc_trk_active;
  const bool noLocOrLandLast = !lastLat.loc_cpt_active && !lastLat.loc_trk_active && !lastLat.land_active;

  switch (id) {
    case Signal::RwyLoc: {
      // RWY: SRS takeoff engaged for 0.9 s, on the ground for 30 s, flaps out, heading within 20° of the ILS course
      // and a valid localizer within 0.4°.
      const auto& w2 = oppBus(f).discrete_word_2;
      bool oppRwyLoc = oppFmgcBit(w2, kOppWord2Rwy) && oppFmgcBit(w2, kOppWord2RwyLoc) && oppSync;
      bool takeoffFor09s =
          l.rwyTakeoff09s.update(lastVert.pitch_takeoff_active, f.dt, true, kRwyTakeoffConfirmS);
      float ilsCourse = valueOr(logic.ils_computation_data.runway_heading_deg, 0.0F);
      float heading = valueOr(logic.ir_computation_data.heading_magnetic_deg, 0.0F);
      float headingDiff = std::abs(ilsCourse - heading);  // not wrapped: course 001° vs heading 359° gives 358°
      const auto& locWord = logic.ils_computation_data.localizer_deviation_deg;
      float locDev = valueOr(locWord, 0.0F);
      bool locValid = isNormal(locWord);
      bool onGround30s = l.rwyOnGround30s.update(logic.on_ground, f.dt, true, kRwyOnGroundS);
      float absLocDev = locDev < 0.0F ? -locDev : locDev;
      bool set = oppRwyLoc ||
                 (ownCompute && !lastLat.rwy_loc_submode_active && logic.flap_slat_lever_position >= kMinFlapHandle &&
                  takeoffFor09s && headingDiff <= kRwyMaxHeadingDiffDeg && absLocDev < kRwyMaxLocDevDeg &&
                  kRwyLocSubmodeEnabled && locValid && onGround30s);
      bool setRecently = l.rwyLocSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = (!setRecently && anyOtherLateralModeLast(f.last, Signal::RwyLoc)) || f.ext.modeReset || !locValid;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::RwyTrk: {
      // RWY TRK: the radio height reaches 30 ft (within 0.9 s) with neither NAV engaged nor armed.
      const auto& w2 = oppBus(f).discrete_word_2;
      bool oppRwyTrk = oppFmgcBit(w2, kOppWord2Rwy) && oppFmgcBit(w2, kOppWord2RwyTrk) && oppSync;
      bool reaches30ft = l.rwyTrk30ftEdge.update(radioHeight(f) >= kRwyTrkRadioHeightFt, true);
      bool liftoff = l.rwyTrk30ftPulse.update(reaches30ft, f.dt, true, false, kRwyTrkTriggerS);
      bool set = oppRwyTrk || (ownCompute && !lastLat.nav_active && !lastArmed.nav_armed && liftoff);
      bool setRecently = l.rwyTrkSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = (!setRecently && anyOtherLateralModeLast(f.last, Signal::RwyTrk)) || f.ext.modeReset;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::RwyActive:
      s[id] = s[Signal::RwyLoc] || s[Signal::RwyTrk];
      return true;

    case Signal::RollOut: {
      // ROLL OUT: LAND, below 50 ft RA, wheels spinning above 70 kt (or on ground when no BSCU is valid), for 0.36 s.
      // Holds until LAND drops.
      bool bscuValid = f.in.discrete_inputs.bscu_opp_valid || f.in.discrete_inputs.bscu_own_valid;
      bool wheelSpinUp =
          f.in.discrete_inputs.left_wheel_spd_abv_70_kts || f.in.discrete_inputs.right_wheel_spd_abv_70_kts;
      bool touchdown = lastLat.land_active && radioHeight(f) <= kRollOutRadioHeightFt &&
                       ((wheelSpinUp && bscuValid) || (!bscuValid && logic.on_ground));
      bool confirmed = l.rollOutCondition036s.update(touchdown, f.dt, true, kRollOutConfirmS);
      s[id] = latch(confirmed, !lastLat.land_active, s[id]);
      return true;
    }

    case Signal::Align:
      // ALIGN (decrab): LAND, below 30 ft RA, rudder control available on a FAC.
      s[id] = lastLat.land_active && radioHeight(f) <= kAlignRadioHeightFt && !logic.fac_rudder_control_failure;
      return true;

    case Signal::GaTrk: {
      // GA TRK: SRS go-around engaged (0.72 s pulse) while not in GA TRK, NAV, HDG or TRK.
      bool oppGaTrk = oppFmgcBit(oppBus(f).discrete_word_2, kOppWord2GaTrk) && oppSync;
      bool srsGaEngaged = l.gaTrkSrsGaPulse.update(lastVert.pitch_goaround_active, f.dt, true, false, kPulseS);
      bool set = oppGaTrk || (ownCompute && !lastLat.roll_goaround_active && !lastLat.nav_active &&
                              !lastLat.hdg_active && !lastLat.trk_active && srsGaEngaged);
      bool setRecently = l.gaTrkSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = f.ext.modeReset || (anyOtherLateralModeLast(f.last, Signal::GaTrk) && !setRecently);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::LateralReversion: {
      // Lateral mode reversion to HDG/TRK (computed by every FMGC, whether it is the master or not).
      bool navLost = lastLat.nav_active && (!fms.lateral_flight_plan_valid || !fms.nav_capture_condition) &&
                     !lastVert.final_des_active;
      bool landLost = l.revLandLost.update(lastArmed.land_armed || lastLat.land_active, false);
      bool noRwyGaNavLand =
          !lastLat.rwy_active && !lastLat.roll_goaround_active && !lastLat.nav_active && !lastLat.land_active;
      bool noVerticalModeTakingOver = !lastVert.vs_active && !lastVert.fpa_active && !lastVert.exp_clb_active &&
                                      !lastVert.exp_des_active && !lastVert.op_des_active &&
                                      !lastVert.op_clb_active && !lastVert.des_active && !lastVert.clb_active;
      bool locPushed = fcuWord2Bit(f, kFcuWord2LocPushed);
      bool locPushRecent = l.revLocPush072s.update(locPushed, f.dt, true, false, kPulseS);
      // LAND disarmed or lost on the localizer (not by a LOC push or a selected/managed vertical mode)
      bool landLostOnLoc =
          locCapturedLast && landLost && noRwyGaNavLand && noVerticalModeTakingOver && !locPushRecent;
      bool finalLost = l.revFinalLost.update(lastArmed.final_des_armed || lastVert.final_des_active, false);
      // LOC pushed while on the localizer without LAND
      bool locPushLoc = l.revLocPushLoc.update(locPushed, true);
      bool locPushedOnLoc = locCapturedLast && !lastArmed.land_armed && !lastLat.land_active && locPushLoc;
      bool locPushFinal = l.revLocPushFinal.update(locPushed, true);
      // FINAL disarmed or lost in NAV (not by a selected/managed vertical mode)
      bool finalLostOnNav = noVerticalModeTakingOver && finalLost && lastLat.nav_active && !lastLat.rwy_active &&
                            !lastLat.roll_goaround_active && !lastLat.loc_cpt_active && !lastLat.loc_trk_active &&
                            !lastLat.land_active;
      s[id] = navLost || landLostOnLoc || finalLostOnNav || locPushedOnLoc ||
              (lastLat.nav_active && lastVert.final_des_active && locPushFinal);  // LOC pushed in FINAL APP
      return true;
    }

    case Signal::HdgTrk: {
      const auto& w2 = oppBus(f).discrete_word_2;
      bool oppHdgTrk = (oppFmgcBit(w2, kOppWord2Hdg) || oppFmgcBit(w2, kOppWord2Trk)) && oppSync;
      bool knobPull = l.hdgKnobPull.update(fcuWord2Bit(f, kFcuWord2HdgKnobPulled), true);
      // HDG/TRK knob pulled: on ground with engines stopped or 5 s after liftoff, not in LAND, above 100 ft RA in
      // GA TRK.
      bool pullEngages = logic.gnd_eng_stop_flt_5s && knobPull && !lastLat.land_active &&
                         (!lastLat.roll_goaround_active || radioHeight(f) >= kGaMinRadioHeightFt);
      // First AP/FD engagement in flight with no lateral mode and no SRS go-around: HDG/TRK is the basic mode.
      bool apFdEngaged = l.hdgApFdEngage.update(f.ext.anyApFdEngaged, true);
      bool noLateralModeNoGa = !f.last.any_lateral_mode_engaged && !lastVert.pitch_goaround_active;
      bool set = oppHdgTrk || (ownCompute && (pullEngages || (!logic.on_ground && apFdEngaged && noLateralModeNoGa) ||
                                              s[Signal::LateralReversion]));
      bool setRecently = l.hdgSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = (!setRecently && anyOtherLateralModeLast(f.last, Signal::HdgTrk)) || f.ext.modeReset;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::HdgActive:
      s[id] = fcuWord1Bit(f, kFcuWord1HdgVsSelected) && s[Signal::HdgTrk];
      return true;

    case Signal::TrkActive:
      s[id] = s[Signal::HdgTrk] && !fcuWord1Bit(f, kFcuWord1HdgVsSelected);
      return true;

    case Signal::NavArmed: {
      bool oppArmed = oppFmgcBit(oppBus(f).discrete_word_3, kOppWord3NavArmed) && oppSync;
      // HDG/TRK knob pushed, or a DIR TO while NAV cannot capture yet; not on the localizer
      bool knobPush = l.navArmKnobPush.update(fcuWord2Bit(f, kFcuWord2HdgKnobPushed), true);
      bool dirTo = l.navArmDirTo.update(fms.direct_to_nav_engage, true);
      bool pilotArms = noLocOrLandLast && (knobPush || (dirTo && !fms.nav_capture_condition));
      // On the ground: no heading preset (or the flight plan just became valid), and no lateral mode or only RWY.
      bool flightPlanValidEdge = l.navArmFlightPlanValid.update(fms.lateral_flight_plan_valid, true);
      bool noPreset = !f.last.hdg_trk_preset_available;
      bool finalArmedEdge = l.navArmFinalArmed.update(lastArmed.final_des_armed, true);
      bool srsGaEngaged = l.navArmSrsGaPulse.update(lastVert.pitch_goaround_active, f.dt, true, false, kPulseS);
      bool groundArms = logic.on_ground && (noPreset || flightPlanValidEdge) &&
                        (!f.last.any_lateral_mode_engaged || lastLat.rwy_loc_submode_active);
      bool set = oppArmed || (ownCompute && fms.lateral_flight_plan_valid &&
                              (pilotArms || groundArms || finalArmedEdge || (srsGaEngaged && noPreset)));

      bool presetEdge = l.navArmPresetEdge.update(f.last.hdg_trk_preset_available, true);
      bool knobPull = l.navArmKnobPull.update(fcuWord2Bit(f, kFcuWord2HdgKnobPulled), true);
      bool locEdge = l.navArmLocEdge.update(
          lastLat.loc_cpt_active || lastLat.loc_trk_active || lastArmed.loc_armed || lastLat.land_active, true);
      bool finalDisarmed = l.navArmFinalLost.update(s[Signal::FinalArmed], false);  // this frame's FINAL armed
      bool reset = presetEdge || knobPull || locEdge || !fms.lateral_flight_plan_valid || finalDisarmed ||
                   lastLat.nav_active || !f.ext.anyApFdEngaged;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::Nav: {
      // NAV: armed (or a DIR TO within 0.72 s, not on the localizer), the FMS can capture, above 30 ft RA (100 ft in
      // GA TRK).
      bool oppNav = oppFmgcBit(oppBus(f).discrete_word_2, kOppWord2Nav) && oppSync;
      bool dirTo = l.navDirToPulse.update(fms.direct_to_nav_engage, f.dt, true, false, kPulseS);
      bool armed = lastArmed.nav_armed || (dirTo && noLocOrLandLast);
      float ra = radioHeight(f);
      bool set = oppNav || (ownCompute && fms.nav_capture_condition && armed &&
                            (ra >= kNavMinRadioHeightFt || logic.dual_ra_failure) &&
                            (!lastLat.roll_goaround_active || ra >= kGaMinRadioHeightFt));
      bool setRecently = l.navSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = !fms.lateral_flight_plan_valid || f.ext.modeReset ||
                   (anyOtherLateralModeLast(f.last, Signal::Nav) && !setRecently);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::LocArmed: {
      bool oppArmed = oppFmgcBit(oppBus(f).discrete_word_3, kOppWord3LocArmed) && oppSync;
      bool landArmedEdge = l.locArmLandEdge.update(lastArmed.land_armed || lastLat.land_active, true);
      bool locPush = l.locArmLocPush.update(fcuWord2Bit(f, kFcuWord2LocPushed), true);
      // LOC pushed: ILS available, above 400 ft RA, not in the takeoff or go-around phase.
      bool pushArms = locPush && !logic.ils_failure && radioHeight(f) >= kLocArmMinRadioHeightFt &&
                      fms.fms_flight_phase != fmgc_flight_phase::Takeoff &&
                      fms.fms_flight_phase != fmgc_flight_phase::Goaround;
      bool set = oppArmed || (ownCompute && (landArmedEdge || pushArms));
      bool pushArmedRecently = l.locArmPushRecent.update(pushArms, f.dt, false, kOtherModeTakeoverDelayS);
      bool landDisarmed = l.locArmLandDisarmed.update(lastArmed.land_armed, false);
      bool navArmedEdge = l.locArmNavArmed.update(lastArmed.nav_armed, true);
      bool reset = (!pushArmedRecently && landDisarmed) || lastLat.loc_cpt_active || lastLat.loc_trk_active ||
                   lastVert.pitch_goaround_active ||
                   (!lastArmed.land_armed && !lastLat.land_active && locPush && lastArmed.loc_armed) ||  // pushed again
                   navArmedEdge;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::LocCapture: {
      bool oppCapture = oppFmgcBit(oppBus(f).discrete_word_2, kOppWord2LocCapture) && oppSync;
      float heading = valueOr(logic.ir_computation_data.heading_magnetic_deg, 0.0F);
      const auto& locWord = logic.ils_computation_data.localizer_deviation_deg;
      float locDev = valueOr(locWord, 0.0F);
      float roll = valueOr(logic.ir_computation_data.roll_angle_deg, 0.0F);
      float courseErr = courseError(heading, logic.rwy_hdg_memo);
      bool bankReached = captureBankReached(courseErr, f.ext.lateral.lastPhiLocC, roll);

      bool capture = false;
      float absCourseErr = std::abs(courseErr);
      if (absCourseErr < kLocCaptureMaxCourseErrorDeg) {
        float absDev = std::abs(locDev);
        // Large intercept angle, beam ahead (deviation on the other side of the course error), within 10°, bank
        // command reached; or within 1.92°.
        bool wideCapture = absCourseErr > kLocCaptureMinInterceptDeg && absDev < kLocCaptureWideDevDeg &&
                           sign(courseErr) != sign(locDev) && bankReached;
        if (wideCapture || absDev < kLocCaptureDevDeg) {
          capture = bankReached || (absCourseErr < kLocCaptureCloseInterceptDeg && absDev < kLocCaptureCloseDevDeg);
        }
      }
      bool ownCapture = ownCompute && lastArmed.loc_armed && capture && isNormal(locWord);
      bool set = oppCapture || ownCapture;
      bool setRecently = l.locCaptureSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = (!setRecently && anyOtherLateralModeLast(f.last, Signal::LocCapture)) || f.ext.modeReset;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::LocTrack: {
      // LOC: LOC* and the deviation within 0.16° for 10 s.
      bool oppTrack = oppFmgcBit(oppBus(f).discrete_word_2, kOppWord2LocTrack) && oppSync;
      float locDev = valueOr(logic.ils_computation_data.localizer_deviation_deg, 0.0F);
      float absDev = locDev < 0.0F ? -locDev : locDev;
      bool onCourse10s = l.locTrackDev10s.update(absDev < kLocTrackDevDeg, f.dt, true, kLocTrackConfirmS);
      bool set = oppTrack || (ownCompute && lastLat.loc_cpt_active && onCourse10s);
      bool setRecently = l.locTrackSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = (!setRecently && anyOtherLateralModeLast(f.last, Signal::LocTrack)) || f.ext.modeReset;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::RollFdBarsFlashing: {
      // 10 s after a lateral reversion, or after the first AP/FD engagement unless TCAS is engaged.
      bool apFdEngaged = l.rollBarsApFdEngage.update(f.ext.anyApFdEngaged, true);
      bool trigger = s[Signal::LateralReversion] || (apFdEngaged && !s[Signal::Tcas]);
      s[id] = l.rollBarsFlash10s.update(trigger, f.dt, true, false, kRollBarsFlashS);
      return true;
    }

    case Signal::HdgTrkPreset: {
      // A heading/track preset (FCU selected value, or the knob pulled) is kept when made in a guided mode or on the
      // ground; it is cleared by HDG/TRK, NAV, the knob pushed, no AP/FD, or leaving those conditions.
      const auto& fcu = f.in.bus_inputs.fcu_bus;
      bool presetSelected =
          isNormal(fcu.selected_hdg_deg) || isNormal(fcu.selected_trk_deg) || fcuWord2Bit(f, kFcuWord2HdgKnobPulled);
      bool selectedPulse = l.presetSelectPulse.update(presetSelected, f.dt, true, false, kPulseS);
      bool navArmedRecently = l.presetNavArmedRecent.update(s[Signal::NavArmed], f.dt, false, kPulseS);
      bool guidedMode = s[Signal::LocCapture] || s[Signal::LocTrack] || s[Signal::Land] || s[Signal::Final] ||
                        s[Signal::GaTrk] || s[Signal::RwyTrk];
      bool presetPossible = navArmedRecently || guidedMode;
      bool guidedOrLow = guidedMode || logic.on_ground ||
                         (radioHeight(f) < kPresetMaxRadioHeightFt && !logic.dual_ra_failure);
      bool set = selectedPulse && presetPossible && guidedOrLow;
      bool knobPush = l.presetKnobPushPulse.update(fcuWord2Bit(f, kFcuWord2HdgKnobPushed), f.dt, true, false, kPulseS);
      bool noApFdRecently = l.presetNoApFdRecent.update(!f.ext.anyApFdEngaged, f.dt, false, kPulseS);
      bool reset = !guidedOrLow || s[Signal::HdgActive] || s[Signal::TrkActive] || s[Signal::Nav] || knobPush ||
                   noApFdRecently;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::AnyLateralMode:
      s[id] = s[Signal::RwyActive] || s[Signal::GaTrk] || s[Signal::LocCapture] || s[Signal::LocTrack] ||
              s[Signal::HdgActive] || s[Signal::TrkActive] || s[Signal::Nav] || s[Signal::Land];
      return true;

    case Signal::ActiveLateralLaw: {
      lateral_law law = lateral_law::NONE;
      if (s[Signal::RollOut] || s[Signal::RwyLoc]) {
        law = lateral_law::ROLL_OUT;  // also RWY: the takeoff runway centreline uses the roll-out law
      } else if (s[Signal::LocTrack] || s[Signal::Land]) {
        law = lateral_law::LOC_TRACK;
      } else if (s[Signal::LocCapture]) {
        law = lateral_law::LOC_CPT;
      } else if (s[Signal::Nav]) {
        law = lateral_law::HPATH;
      } else if (s[Signal::TrkActive] || s[Signal::RwyTrk] || s[Signal::GaTrk]) {
        law = lateral_law::TRACK;
      } else if (s[Signal::HdgActive]) {
        law = lateral_law::HDG;
      }
      s.activeLateralLaw = law;
      s[id] = law != lateral_law::NONE;
      return true;
    }

    case Signal::LocBcSelection:
      s[id] = kLocBackCourseSelected;
      return true;

    default:
      return false;
  }
}

}  // namespace fmgc::readable
