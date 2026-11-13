// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SRS takeoff and go-around, LAND armed, LAND, FLARE, FINAL armed, FINAL, G/S armed, G/S*, G/S.
// Spec: docs/modes/takeoff-ga-approach.md. Generated: FmgcComputer.cpp 1667–2026 (SRS T/O, LAND, FLARE, FINAL,
// SRS G/A), 3148–3258 (G/S), 1913–1920 and 4001 (FLARE output).
//
// Every latch here reads the lateral modes only through the previous frame's outputs (`f.last`), never a lateral latch
// updated earlier in the same step.

#include "vertical.h"

namespace fmgc::readable {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::rawValue;
using blocks::a429::valueOr;

constexpr double kOtherModeTakeoverDelayS = 0.72;  // another mode replaces this one only after this

// Thrust levers (TLA, deg)
constexpr double kTogaTlaDeg = 36.7;  // at or above: TOGA
constexpr double kFlxTlaDeg = 33.3;   // between this and TOGA: FLX/MCT detent

// SRS
constexpr double kOnGroundTakeoffS = 30;  // on ground this long: a thrust lever edge is a takeoff, else a go-around
constexpr double kMinFlapHandle = 1;      // flap handle index: not flaps 0

// LAND
constexpr double kLandArmMinRadioHeightFt = 400;  // with an engine running, LAND arms only above 400 ft RA
constexpr double kLandArmedDisarmDelayS = 0.45;   // APPR pushed again after this disarms instead of re-arming
constexpr double kLandRadioHeightFt = 400;        // LAND engages below 400 ft RA ...
constexpr double kLandRadioHeightConfirmS = 1.2;  // ... held for 1.2 s
constexpr double kLandOnGroundResetS = 10;        // on ground this long without AP: APPR push drops LAND

// FLARE
constexpr double kFlareConfirmS = 0.36;  // the flare condition latches after this

// FINAL
constexpr double kFinalArmMinRadioHeightFt = 400;

// G/S (deviation in deg)
constexpr double kGsLagC1 = 1;                  // deviation lag filter, rad/s
constexpr double kGsCaptureDevDecreasingDeg = 0.8;  // capture early when the (signed) filtered deviation decreases
constexpr double kGsCaptureDevDeg = 0.133;
constexpr double kGsTrackDevDeg = 0.133;
constexpr double kGsCaptureMinS = 15;  // G/S* lasts at least 15 s before G/S

// FCU discrete words
constexpr int kFcuWord1ApprPushed = 23;
constexpr int kFcuWord2LocPushed = 13;

// Opposite-FMGC discrete words (the mode this FMGC copies when it is the slave)
constexpr int kOppWord1PitchTakeoff = 15;
constexpr int kOppWord1PitchGoAround = 16;
constexpr int kOppWord1AltHoldOrGs = 20;     // "track": ALT or G/S
constexpr int kOppWord1AltCaptOrGsCapt = 21; // "capture": ALT* or G/S*
constexpr int kOppWord1GsCaptOrTrack = 22;
constexpr int kOppWord1Final = 23;
constexpr int kOppWord3LandArmed = 20;
constexpr int kOppWord3GlideArmed = 22;
constexpr int kOppWord3FinalArmed = 23;
constexpr int kOppWord4Land = 14;

const auto& oppBus(const Frame& f) {
  return f.in.bus_inputs.fmgc_opp_bus;
}

bool latch(bool set, bool reset, bool q) {
  return reset ? false : (set ? true : q);
}

float tlaOwn(const Frame& f) {
  return valueOr(f.in.bus_inputs.fadec_own_bus.selected_tla_deg, 0.0F);
}

float tlaOpp(const Frame& f) {
  return valueOr(f.in.bus_inputs.fadec_opp_bus.selected_tla_deg, 0.0F);
}

/** Takeoff thrust on one engine: TOGA, or the FLX/MCT detent with a FLEX temperature on that FADEC. */
bool takeoffThrust(float tla, const base_arinc_429& flexTemp) {
  bool belowToga = tla < kTogaTlaDeg;
  return !belowToga || (belowToga && tla > kFlxTlaDeg && isNormal(flexTemp));
}

/** Selected radio height (ft), raw: 2500 ft when both radio altimeters failed. */
float radioHeight(const Frame& f) {
  return rawValue(f.ext.logic->ra_computation_data.radio_height_ft);
}

bool apprPushedBit(const Frame& f) {
  return fcuWord1Bit(f, kFcuWord1ApprPushed);
}

bool locPushedBit(const Frame& f) {
  return bit(f.in.bus_inputs.fcu_bus.fcu_discrete_word_2, kFcuWord2LocPushed) != 0;
}

/** Any lateral mode other than LAND active last frame (LAND's own "another mode took over" reset adds these). */
bool anyLateralModeExceptLandLast(const base_fmgc_ap_fd_logic_outputs& last) {
  const auto& l = last.lateral_modes;
  return l.rwy_active || l.roll_goaround_active || l.loc_cpt_active || l.loc_trk_active || l.hdg_active ||
         l.trk_active || l.nav_active;
}

/** FLARE condition: LAND active, a radio altimeter available, and the outer loops' flare trigger (last step). */
bool flareCondition(const Frame& f) {
  return !f.ext.logic->dual_ra_failure && f.ext.approach.lastFlareCondition && f.last.lateral_modes.land_active;
}

}  // namespace

void VerticalModeLogic::resetApproach(Signals& s) {
  s[Signal::SrsTakeoff] = false;
  s[Signal::LandArmed] = false;
  s[Signal::Land] = false;
  s[Signal::FlareMemory] = false;
  s[Signal::FinalArmed] = false;
  s[Signal::Final] = false;
  s[Signal::SrsGoAround] = false;
  s[Signal::GlideArmed] = false;
  s[Signal::GsCapture] = false;
  s[Signal::GsTrack] = false;
  s[Signal::Flare] = false;
  approach_ = ApproachState{};
}

bool VerticalModeLogic::updateApproach(Signal id, const Frame& f, Signals& s) {
  const auto& last = f.last.longitudinal_modes;
  const auto& lastLat = f.last.lateral_modes;
  const auto& lastArmed = f.last.armed_modes;
  const auto& logic = *f.ext.logic;
  const bool oppSync = f.ext.oppModeSync;
  const bool ownCompute = !oppSync;
  auto& a = approach_;

  switch (id) {
    case Signal::SrsTakeoff: {
      // Takeoff: on the ground for 30 s with V2 entered and flaps out, a thrust lever is moved to FLX or TOGA.
      bool oppTakeoff = oppFmgcBit(oppBus(f).discrete_word_1, kOppWord1PitchTakeoff);
      bool onGround30s = a.toOnGround30s.update(logic.on_ground, f.dt, true, kOnGroundTakeoffS);
      bool takeoffThrustSet = takeoffThrust(tlaOpp(f), f.in.bus_inputs.fadec_opp_bus.selected_flex_temp_deg) ||
                              takeoffThrust(tlaOwn(f), f.in.bus_inputs.fadec_own_bus.selected_flex_temp_deg);
      bool thrustEdge = a.toTakeoffThrustEdge.update(takeoffThrustSet, true);
      bool v2Entered = f.in.fms_inputs.v_2_kts > 0.0;
      bool set = (v2Entered && oppTakeoff && oppSync) ||
                 (ownCompute && !last.pitch_takeoff_active && !last.pitch_goaround_active && onGround30s &&
                  v2Entered && logic.flap_slat_lever_position >= kMinFlapHandle && thrustEdge);
      bool setRecently = a.toSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = f.ext.modeReset || (anyOtherVerticalModeLast(f.last, Signal::SrsTakeoff) && !setRecently);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::LandArmed: {
      bool oppArmed = oppFmgcBit(oppBus(f).discrete_word_3, kOppWord3LandArmed) && oppSync;
      bool apprPush = a.landArmApprPush.update(apprPushedBit(f), true);
      bool armedFor045s = a.landArmedFor045s.update(lastArmed.land_armed, f.dt, true, kLandArmedDisarmDelayS);

      // ILS approach possible: ILS selected and received, above 400 ft RA (or engines stopped), an RA available, APPR
      // pushed (not as a second push to disarm), no RNAV approach armed or engaged.
      bool ilsApproach = f.in.fms_inputs.selected_approach_type == fmgc_approach_type::ILS && !logic.ils_failure &&
                         (!logic.engine_running || radioHeight(f) >= kLandArmMinRadioHeightFt) &&
                         !logic.dual_ra_failure && apprPush && !armedFor045s && !last.final_des_active &&
                         !lastArmed.final_des_armed;

      // Both ILS receivers tuned to the same ILS (or not both valid).
      const auto& ilsOpp = f.in.bus_inputs.ils_opp_bus;
      const auto& ilsOwn = f.in.bus_inputs.ils_own_bus;
      bool receiversAgree = (valueOr(ilsOpp.ils_frequency_mhz, 0.0F) == valueOr(ilsOwn.ils_frequency_mhz, 0.0F) &&
                             valueOr(ilsOpp.runway_heading_deg, 0.0F) == valueOr(ilsOwn.runway_heading_deg, 0.0F)) ||
                            !logic.both_ils_valid;

      bool gsCaptured = last.gs_capt_active || last.gs_trk_active;
      bool locCaptured = lastLat.loc_cpt_active || lastLat.loc_trk_active;
      bool selectedVerticalEngaged = a.landArmSelectedVertical.update(
          last.vs_active || last.fpa_active || last.op_clb_active || last.op_des_active || last.exp_clb_active ||
              last.exp_des_active,
          true);
      bool otherLateralEngaged =
          a.landArmSelectedLateral.update(lastLat.hdg_active || lastLat.trk_active || lastLat.nav_active, true);
      bool locPush = a.landArmLocPush.update(locPushedBit(f), true);

      bool set = !lastLat.land_active &&
                 (oppArmed || (ownCompute && ilsApproach && !last.pitch_takeoff_active &&
                               !last.pitch_goaround_active && !lastLat.rwy_active && !lastLat.roll_goaround_active &&
                               receiversAgree));
      bool reset = lastLat.land_active || last.pitch_goaround_active ||
                   (!gsCaptured && !locCaptured && lastArmed.nav_armed) ||  // NAV armed instead
                   (gsCaptured && selectedVerticalEngaged) || (locCaptured && otherLateralEngaged) ||
                   (apprPush && lastArmed.land_armed) ||  // APPR pushed again
                   locPush || f.ext.modeReset || last.tcas_active;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::Land: {
      // LAND: G/S and LOC tracking, below 400 ft RA for 1.2 s, at least 0.72 s after LAND was last active.
      bool notLand072s = a.landNotActive072s.update(!lastLat.land_active, f.dt, true, kOtherModeTakeoverDelayS);
      bool oppLand = oppFmgcBit(oppBus(f).discrete_word_4, kOppWord4Land);
      bool below400ft =
          a.landBelow400ft.update(radioHeight(f) < kLandRadioHeightFt, f.dt, true, kLandRadioHeightConfirmS);
      bool set = notLand072s && ((oppLand && oppSync) ||
                                 (ownCompute && last.gs_trk_active && lastLat.loc_trk_active && below400ft));
      bool setRecently = a.landSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool apprPush = a.landApprPush.update(apprPushedBit(f), true);
      bool onGround10s = a.landOnGround10s.update(logic.on_ground, f.dt, true, kLandOnGroundResetS);

      bool otherModes = anyOtherVerticalModeLast(f.last, Signal::Land) || anyLateralModeExceptLandLast(f.last);
      bool reset = (otherModes && !setRecently) || last.pitch_goaround_active ||
                   // on the ground after landing, no AP: APPR push drops LAND
                   (!logic.ap_own_engaged && !f.in.discrete_inputs.ap_opp_engaged && apprPush && onGround10s) ||
                   f.ext.modeReset;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::FlareMemory: {
      // The flare condition latches after 0.36 s and holds until LAND drops.
      bool confirmed = a.flareCondition036s.update(flareCondition(f), f.dt, true, kFlareConfirmS);
      s[id] = latch(confirmed, !lastLat.land_active, s[id]);
      return true;
    }

    case Signal::FinalArmed: {
      bool oppArmed = oppFmgcBit(oppBus(f).discrete_word_3, kOppWord3FinalArmed) && oppSync;
      bool armedRecently = a.finalArmedRecent.update(lastArmed.final_des_armed, f.dt, false, kOtherModeTakeoverDelayS);
      bool apprPush = a.finalArmApprPush.update(apprPushedBit(f), true);

      // APPR pushed with no T/O, RWY or G/A mode and FINAL neither engaged nor armed within the last 0.72 s.
      bool apprPushArms = !last.pitch_takeoff_active && !lastLat.rwy_active && !last.pitch_goaround_active &&
                          !lastLat.roll_goaround_active && !last.final_des_active && !armedRecently && apprPush;
      // RNAV approach with lateral and vertical flight plans, above 400 ft RA (or engines stopped, or no RA), no LAND
      // armed.
      bool ownArm = ownCompute && f.in.fms_inputs.lateral_flight_plan_valid &&
                    f.in.fms_inputs.vertical_flight_plan_valid &&
                    f.in.fms_inputs.selected_approach_type == fmgc_approach_type::RNAV &&
                    (!logic.engine_running || radioHeight(f) >= kFinalArmMinRadioHeightFt || logic.dual_ra_failure) &&
                    !lastArmed.land_armed && apprPushArms;
      bool set = !lastLat.land_active && (oppArmed || ownArm);

      bool navLost = a.finalArmNavLost.update(lastArmed.nav_armed || lastLat.nav_active, false);
      bool locPush = a.finalArmLocPush.update(locPushedBit(f), true);
      bool reset = last.final_des_active || last.pitch_goaround_active || !f.in.fms_inputs.vertical_flight_plan_valid ||
                   navLost || (apprPush && lastArmed.final_des_armed) ||  // APPR pushed again
                   locPush || f.ext.modeReset || last.tcas_active;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::Final: {
      // FINAL engages from FINAL armed with NAV, when the FMS says it can, in the approach phase.
      bool oppFinal = oppFmgcBit(oppBus(f).discrete_word_1, kOppWord1Final) && oppSync;
      bool set = oppFinal || (ownCompute && lastLat.nav_active && lastArmed.final_des_armed &&
                              f.in.fms_inputs.final_app_can_engage &&
                              f.in.fms_inputs.fms_flight_phase == fmgc_flight_phase::Approach);
      bool setRecently = a.finalSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = f.ext.modeReset || (anyOtherVerticalModeLast(f.last, Signal::Final) && !setRecently);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::SrsGoAround: {
      // Go-around: flaps out, a thrust lever is moved to TOGA, not on the ground for 30 s.
      bool oppGoAround = oppFmgcBit(oppBus(f).discrete_word_1, kOppWord1PitchGoAround) && oppSync;
      bool onGround30s = a.gaOnGround30s.update(logic.on_ground, f.dt, true, kOnGroundTakeoffS);
      bool flapsOut = logic.flap_slat_lever_position >= kMinFlapHandle;
      bool oppToga = tlaOpp(f) >= kTogaTlaDeg;
      bool togaEdge = a.gaTogaEdge.update(oppToga || tlaOwn(f) >= kTogaTlaDeg, true);
      bool set = oppGoAround || (ownCompute && !last.pitch_takeoff_active && !last.pitch_goaround_active &&
                                 !onGround30s && flapsOut && togaEdge);
      bool setRecently = a.gaSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      // Unlike the other modes, the AP/FD mode reset is also delayed by 0.72 s after the set condition.
      bool reset = (f.ext.modeReset || anyOtherVerticalModeLast(f.last, Signal::SrsGoAround)) && !setRecently;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::GlideArmed: {
      // G/S arms the frame after LAND arms (or engages) and disarms when LAND disarms or G/S is captured.
      bool oppArmed = oppFmgcBit(oppBus(f).discrete_word_3, kOppWord3GlideArmed) && oppSync;
      bool landArmedEdge = a.glideArmLandArmedOrActive.update(lastArmed.land_armed || lastLat.land_active, true);
      bool set = oppArmed ||
                 (ownCompute && landArmedEdge && !last.gs_capt_active && !last.gs_trk_active);
      bool landDisarmed = a.glideArmLandDisarmed.update(lastArmed.land_armed, false);
      bool reset = landDisarmed || last.gs_capt_active || last.gs_trk_active;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::GsCapture: {
      const auto& opp = oppBus(f).discrete_word_1;
      bool oppCapture = oppFmgcBit(opp, kOppWord1AltCaptOrGsCapt) && oppFmgcBit(opp, kOppWord1GsCaptOrTrack) && oppSync;
      bool locCaptured = lastLat.loc_cpt_active || lastLat.loc_trk_active;

      const auto& gsDevWord = logic.ils_computation_data.glideslope_deviation_deg;
      float dev = valueOr(gsDevWord, 0.0F);
      float devLag = a.gsDeviationLag.update(dev, kGsLagC1, f.dt);
      double absDev = dev < 0.0F ? -dev : dev;
      // Capture within 0.133°, or within 0.8° while the signed, filtered deviation is decreasing.
      bool decreasing = devLag < a.gsDeviationLagLast;
      a.gsDeviationLagLast = devLag;
      bool ownCapture = ownCompute && lastArmed.glide_armed && locCaptured &&
                        ((decreasing && absDev < kGsCaptureDevDecreasingDeg) || absDev < kGsCaptureDevDeg) &&
                        isNormal(gsDevWord);
      bool set = oppCapture || ownCapture;
      bool setRecently = a.gsCaptureSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = (anyOtherVerticalModeLast(f.last, Signal::GsCapture) && !setRecently) || f.ext.modeReset;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::GsTrack: {
      const auto& opp = oppBus(f).discrete_word_1;
      bool oppTrack = oppFmgcBit(opp, kOppWord1AltHoldOrGs) && oppFmgcBit(opp, kOppWord1GsCaptOrTrack) && oppSync;
      // G/S* (this frame's latch) for 15 s, then within 0.133°.
      bool captureFor15s = a.gsCaptureFor15s.update(s[Signal::GsCapture], f.dt, true, kGsCaptureMinS);
      float dev = valueOr(logic.ils_computation_data.glideslope_deviation_deg, 0.0F);
      double absDev = dev < 0.0F ? -dev : dev;
      bool set = oppTrack || (ownCompute && captureFor15s && absDev < kGsTrackDevDeg);
      bool setRecently = a.gsTrackSetRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      bool reset = (anyOtherVerticalModeLast(f.last, Signal::GsTrack) && !setRecently) || f.ext.modeReset;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::Flare:
      // FLARE shows from the first frame of the flare condition; the memory keeps it until LAND drops.
      s[id] = flareCondition(f) || s[Signal::FlareMemory];
      return true;

    default:
      return false;
  }
}

}  // namespace fmgc::readable
