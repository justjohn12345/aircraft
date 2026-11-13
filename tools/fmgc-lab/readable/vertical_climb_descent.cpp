// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// CLB armed, CLB, DES armed, DES, OP CLB (and its reversion flag), OP DES, EXP CLB, EXP DES.
// Spec: docs/modes/climb-descent.md. Generated: FmgcComputer.cpp 2721–3148.
//
// Every engaged mode here follows the same pattern: it is set by the opposite FMGC's mode (when this FMGC follows the
// opposite one) or by its own engage condition, and reset by the general mode reset, or by another vertical mode that
// was active last frame unless its own set condition was true within the last 0.72 s.

#include <cmath>

#include "vertical.h"

namespace fmgc::readable {

namespace {

using Phase = fmgc_flight_phase;

constexpr double kOtherModeTakeoverDelayS = 0.72;  // another mode replaces this one only after this
constexpr double kManualSpeedReversionS = 0.72;    // selected speed in EXP CLB/SRS (EXP DES) before OP CLB (OP DES)
constexpr double kNearConstraintFt = 250;          // ALT push within this of the constraint keeps the managed mode armed

// FCU discrete word 1
constexpr int kFcuWord1AltPushed = 17;
constexpr int kFcuWord1AltPulled = 18;
constexpr int kFcuWord1ExpedPushed = 22;

// Opposite FMGC discrete word 1 (active modes) and word 3 (armed modes)
constexpr int kOppWord1ClbFamily = 11;  // CLB, OP CLB or EXP CLB
constexpr int kOppWord1DesFamily = 12;  // DES, OP DES or EXP DES
constexpr int kOppWord1Open = 14;       // OP CLB or OP DES
constexpr int kOppWord1Exped = 24;      // EXP CLB or EXP DES
constexpr int kOppWord3ClbArmed = 24;
constexpr int kOppWord3DesArmed = 25;

bool latch(bool set, bool reset, bool q) {
  return reset ? false : (set ? true : q);
}

/** The opposite FMGC's active mode word shows the CLB or DES `family`, with exactly the given OPEN and EXPED bits. */
bool oppMode(const Frame& f, int family, bool open, bool exped) {
  const auto& w = f.in.bus_inputs.fmgc_opp_bus.discrete_word_1;
  return oppFmgcBit(w, family) && oppFmgcBit(w, kOppWord1Open) == open && oppFmgcBit(w, kOppWord1Exped) == exped;
}

bool oppModeOpen(const Frame& f, int family) {
  const auto& w = f.in.bus_inputs.fmgc_opp_bus.discrete_word_1;
  return oppFmgcBit(w, family) && oppFmgcBit(w, kOppWord1Open);
}

bool oppModeExped(const Frame& f, int family) {
  const auto& w = f.in.bus_inputs.fmgc_opp_bus.discrete_word_1;
  return oppFmgcBit(w, family) && oppFmgcBit(w, kOppWord1Exped);
}

}  // namespace

void VerticalModeLogic::resetClimbDescent(Signals& s) {
  s[Signal::ClbArmed] = false;
  s[Signal::Clb] = false;
  s[Signal::DesArmed] = false;
  s[Signal::Des] = false;
  s[Signal::OpClb] = false;
  s[Signal::ReversionOpClb] = false;
  s[Signal::OpDes] = false;
  s[Signal::ExpClb] = false;
  s[Signal::ExpDes] = false;
  climbDescent_ = ClimbDescentState{};
}

bool VerticalModeLogic::updateClimbDescent(Signal id, const Frame& f, Signals& s) {
  auto& st = climbDescent_;
  const auto& last = f.last.longitudinal_modes;
  const auto& lastLat = f.last.lateral_modes;
  const auto& fms = f.in.fms_inputs;
  const Phase phase = fms.fms_flight_phase;
  const bool ownCompute = !f.ext.oppModeSync;
  const bool onGround = f.ext.logic->on_ground;

  const float fcuAlt = fcuAltitude(f);
  const float h = indicatedAltitude(f);
  const bool fcuAbove = fcuAlt > h;
  const bool fcuBelow = fcuAlt < h;
  const double accAlt = fms.acceleration_alt_ft;  // 0: no ACC ALT
  const double nextCstr = fms.next_alt_cstr_ft;   // 0: no constraint
  const bool vertFpValid = fms.vertical_flight_plan_valid;

  const bool levelled = last.alt_acq_active || last.alt_hold_active;  // ALT* or ALT last frame
  const bool cstrApplicable = f.ext.altCstrApplicable;
  // Distance to the altitude target (constraint or FCU altitude), double as generated.
  const double toTargetFt = std::abs(f.ext.altSelOrCstr - h);
  const bool navOrLoc = lastLat.nav_active || lastLat.loc_cpt_active || lastLat.loc_trk_active;
  // Engaging any mode of this group needs: own computation, "ALT arm possible" (not holding ALT at the FCU altitude
  // without a knob turn since), more than 5 s in flight (or on the ground with both engines stopped), and no LAND.
  const bool airborne5s = f.ext.climbDescent.inFlightFor5s ||
                          (f.in.discrete_inputs.eng_opp_stop && f.in.discrete_inputs.eng_own_stop && onGround);
  const bool canEngage = ownCompute && f.last.armed_modes.alt_acq_arm_possible && airborne5s && !lastLat.land_active;
  // In SRS (takeoff or go-around) or on the ground.
  const bool groundOrSrs = onGround || last.pitch_goaround_active || last.pitch_takeoff_active;

  auto otherModeReset = [&](Signal mode, blocks::ConfirmNode& setRecently, bool set) {
    bool recent = setRecently.update(set, f.dt, false, kOtherModeTakeoverDelayS);
    return f.ext.modeReset || (anyOtherVerticalModeLast(f.last, mode) && !recent);
  };

  switch (id) {
    case Signal::ClbArmed: {
      // In flight: levelled (ALT*/ALT) at an applicable climb constraint with NAV. The ALT-push term is a subset of the
      // first one (no effect); its edge detector is kept so it updates like the generated one.
      bool altPushed = st.clbArmAltPushed.update(fcuWord1Bit(f, kFcuWord1AltPushed), true);
      bool levelAtClimbCstr = !onGround && phase != Phase::Descent && phase != Phase::Approach &&
                              lastLat.nav_active && fcuAbove && vertFpValid &&
                              ((levelled && cstrApplicable) ||
                               (altPushed && levelled && cstrApplicable && toTargetFt <= kNearConstraintFt));
      // On the ground or in SRS: pre-armed while ACC ALT is below the FCU altitude and the next constraint. (The
      // generated code also ANDs a "no lateral mode or RWY" term read from a constant struct: always true.)
      bool accAltBelowTargets = accAlt != 0 && accAlt < fcuAlt && (accAlt < nextCstr || nextCstr == 0);

      bool oppArmed = f.ext.oppModeSync && oppFmgcBit(f.in.bus_inputs.fmgc_opp_bus.discrete_word_3, kOppWord3ClbArmed);
      bool set = oppArmed || (ownCompute && (levelAtClimbCstr || (groundOrSrs && accAltBelowTargets)));
      bool reset = (groundOrSrs && !accAltBelowTargets) || last.vs_active || last.fpa_active || last.exp_clb_active ||
                   last.exp_des_active || last.clb_active || last.des_active || last.gs_capt_active ||
                   last.gs_trk_active || last.op_clb_active || last.op_des_active || last.tcas_active || fcuBelow ||
                   (levelled && (!cstrApplicable || !lastLat.nav_active || !vertFpValid)) ||
                   phase == Phase::Descent || phase == Phase::Approach || !f.ext.anyApFdEngaged;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::Clb: {
      // Trigger from the arm: crossing ACC ALT upwards, or a change of the next constraint.
      bool accAltCrossedUp = st.clbAccAltCrossedUp.update(h > accAlt, true);
      bool cstrChanged = nextCstr != st.clbPreviousNextCstrFt;
      st.clbPreviousNextCstrFt = nextCstr;
      bool armTrigger = f.last.armed_modes.clb_armed && (accAltCrossedUp || cstrChanged);
      // ALT push: engages unless levelled at (or within 250 ft of) the constraint CLB is armed for.
      bool altPushed = st.clbAltPushed.update(fcuWord1Bit(f, kFcuWord1AltPushed), true);
      bool awayFromCstr = !cstrApplicable || toTargetFt > kNearConstraintFt;
      bool pushTrigger = altPushed && (!levelled || (!f.last.armed_modes.clb_armed && awayFromCstr));

      bool own = canEngage && !last.gs_trk_active && lastLat.nav_active && vertFpValid && fcuAbove &&
                 phase != Phase::Descent && phase != Phase::Approach && (armTrigger || pushTrigger);
      bool set = (f.ext.oppModeSync && oppMode(f, kOppWord1ClbFamily, false, false)) || own;
      bool reset = otherModeReset(Signal::Clb, st.clbSetRecently, set);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::DesArmed: {
      // Levelled (ALT*/ALT) at an applicable descent constraint with NAV or LOC. The ALT-push term is redundant, as
      // for CLB armed.
      bool altPushed = st.desArmAltPushed.update(fcuWord1Bit(f, kFcuWord1AltPushed), true);
      bool levelAtDescentCstr = phase != Phase::Takeoff && phase != Phase::Goaround && fcuBelow && navOrLoc &&
                                vertFpValid &&
                                ((levelled && cstrApplicable) ||
                                 (altPushed && levelled && cstrApplicable && toTargetFt <= kNearConstraintFt));

      bool oppArmed = f.ext.oppModeSync && oppFmgcBit(f.in.bus_inputs.fmgc_opp_bus.discrete_word_3, kOppWord3DesArmed);
      bool set = oppArmed || (ownCompute && levelAtDescentCstr);
      // Unlike CLB armed: not reset by CLB or SRS TO, but by SRS GA and FINAL DES.
      bool reset = last.vs_active || last.fpa_active || last.exp_clb_active || last.exp_des_active ||
                   last.des_active || last.gs_capt_active || last.gs_trk_active || last.final_des_active ||
                   last.pitch_goaround_active || last.op_clb_active || last.op_des_active || last.tcas_active ||
                   fcuAbove || !navOrLoc || phase == Phase::Takeoff || phase == Phase::Climb ||
                   phase == Phase::Goaround || !vertFpValid || (levelled && !cstrApplicable) ||
                   !f.ext.anyApFdEngaged;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::Des: {
      // Trigger from the arm: a change of the next constraint only (no top-of-descent trigger).
      bool altPushed = st.desAltPushed.update(fcuWord1Bit(f, kFcuWord1AltPushed), true);
      bool cstrChanged = nextCstr != st.desPreviousNextCstrFt;
      st.desPreviousNextCstrFt = nextCstr;
      bool armTrigger = f.last.armed_modes.des_armed && cstrChanged;
      bool awayFromCstr = !cstrApplicable || toTargetFt > kNearConstraintFt;
      bool pushTrigger = altPushed && (!levelled || (!f.last.armed_modes.des_armed && awayFromCstr));

      bool notInFinalModes = !last.gs_trk_active && !last.des_active && !last.final_des_active &&
                             !last.pitch_goaround_active && !last.pitch_takeoff_active;
      bool own = canEngage && notInFinalModes && navOrLoc && vertFpValid && fcuBelow && phase != Phase::Takeoff &&
                 phase != Phase::Climb && phase != Phase::Goaround && (armTrigger || pushTrigger);
      bool set = (f.ext.oppModeSync && oppMode(f, kOppWord1DesFamily, false, false)) || own;
      bool reset = otherModeReset(Signal::Des, st.desSetRecently, set);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::OpClb: {
      bool altPulled = st.opClbAltPulled.update(fcuWord1Bit(f, kFcuWord1AltPulled), true);
      bool manualSpdHeld = st.manualSpdInExpClbOrSrs.update(
          f.last.manual_spd_control_active &&
              (last.exp_clb_active || last.pitch_goaround_active || last.pitch_takeoff_active),
          f.dt, true, kManualSpeedReversionS);
      // Reversion: CLB lost its phase, NAV or vertical flight plan, or a selected speed in EXP CLB / SRS.
      bool clbLost = last.clb_active && (phase == Phase::Descent || phase == Phase::Approach ||
                                         !lastLat.nav_active || !vertFpValid);
      bool reversion = clbLost || manualSpdHeld;
      // CLB armed but crossing ACC ALT without NAV or vertical flight plan: OP CLB instead of CLB.
      bool accAltCrossedUp = st.opClbAccAltCrossedUp.update(h > accAlt, true);
      bool accAltWithoutNav = accAltCrossedUp && accAlt != 0 && f.last.armed_modes.clb_armed &&
                              (!lastLat.nav_active || !vertFpValid);

      bool own = canEngage && fcuAbove && (altPulled || reversion || accAltWithoutNav);
      bool set = (f.ext.oppModeSync && oppModeOpen(f, kOppWord1ClbFamily)) || own;
      bool reset = otherModeReset(Signal::OpClb, st.opClbSetRecently, set);
      s[id] = latch(set, reset, s[id]);
      st.opClbReversion = reversion;
      return true;
    }

    case Signal::ReversionOpClb:
      // OP CLB active by reversion (output only, no consumer).
      s[id] = s[Signal::OpClb] && st.opClbReversion;
      return true;

    case Signal::OpDes: {
      bool altPulled = st.opDesAltPulled.update(fcuWord1Bit(f, kFcuWord1AltPulled), true);
      bool manualSpdHeld = st.manualSpdInExpDes.update(f.last.manual_spd_control_active && last.exp_des_active, f.dt,
                                                       true, kManualSpeedReversionS);
      bool own = canEngage && fcuBelow && (altPulled || manualSpdHeld);
      bool set = (f.ext.oppModeSync && oppModeOpen(f, kOppWord1DesFamily)) || own;
      bool reset = otherModeReset(Signal::OpDes, st.opDesSetRecently, set);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::ExpClb: {
      bool expedPushed = st.expClbExpedPushed.update(fcuWord1Bit(f, kFcuWord1ExpedPushed), true);
      bool set = (f.ext.oppModeSync && oppModeExped(f, kOppWord1ClbFamily)) || (canEngage && fcuAbove && expedPushed);
      bool reset = otherModeReset(Signal::ExpClb, st.expClbSetRecently, set);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::ExpDes: {
      bool expedPushed = st.expDesExpedPushed.update(fcuWord1Bit(f, kFcuWord1ExpedPushed), true);
      bool set = (f.ext.oppModeSync && oppModeExped(f, kOppWord1DesFamily)) || (canEngage && fcuBelow && expedPushed);
      bool reset = otherModeReset(Signal::ExpDes, st.expDesSetRecently, set);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    default:
      return false;
  }
}

}  // namespace fmgc::readable
