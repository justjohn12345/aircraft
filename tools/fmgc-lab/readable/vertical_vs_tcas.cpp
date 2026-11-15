// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// V/S reversion, V/S-FPA, TCAS armed, TCAS, the TCAS ALT_ACQ / ALT_HOLD submodes, ALT soft mode, V/S and FPA.
// Spec: docs/modes/alt-vs-fpa-tcas.md. Generated: FmgcComputer.cpp 2611–2720, 3260–3326, 3713–3735.

#include <cmath>

#include "vertical.h"

namespace fmgc::readable {

namespace {

using blocks::a429::bit;

constexpr double kAltTargetChangeFt = 250;        // ALT* (or TCAS ALT_ACQ) target moved this far: abandon the capture
constexpr double kAirborneConfirmS = 5;           // V/S engages by default 5 s after liftoff
constexpr double kOtherModeTakeoverDelayS = 0.72; // another mode replaces V/S-FPA or TCAS only after this

constexpr int kFcuWord1VsKnobPushed = 16;    // V/S-FPA knob pushed (level off)
constexpr int kFcuWord1VsKnobPulled = 19;    // V/S-FPA knob pulled
constexpr int kFcuWord1ApprPushed = 23;
constexpr int kFcuWord1HdgVsSelected = 24;   // HDG-V/S selected (else TRK-FPA)
constexpr int kOppWord1VsActive = 17;
constexpr int kOppWord1FpaActive = 18;
constexpr int kOppWord7TcasArmed = 12;
constexpr int kOppWord7TcasActive = 13;
constexpr int kTcasRaWordRaActive = 20;      // TCAS RA word: RA in progress

bool latch(bool set, bool reset, bool q) {
  return reset ? false : (set ? true : q);
}

bool raActive(const Frame& f) {
  return bit(f.in.bus_inputs.tcas_bus.vertical_resolution_advisory, kTcasRaWordRaActive) != 0;
}

/** On ground (FAC discrete word 5, see logic.on_ground). */
bool onGround(const Frame& f) {
  return f.ext.logic->on_ground;
}

}  // namespace

void VerticalModeLogic::resetVsTcas(Signals& s) {
  s[Signal::ReversionVs] = false;
  s[Signal::VsFpa] = false;
  s[Signal::TcasArmed] = false;
  s[Signal::Tcas] = false;
  s[Signal::TcasAltAcq] = false;
  s[Signal::TcasAltHold] = false;
  s[Signal::AltSoft] = false;
  s[Signal::VsActive] = false;
  s[Signal::FpaActive] = false;
  // The sample-and-holds are not reset on restart; they are rewritten on the restart frame anyway (ALT* is not active
  // and the TCAS submode is VS in the initial previous-frame outputs).
  auto altCaptureTarget = vsTcas_.altCaptureTargetFt;
  auto tcasAltAcqTarget = vsTcas_.tcasAltAcqTargetFt;
  vsTcas_ = VsTcasState{};
  vsTcas_.altCaptureTargetFt = altCaptureTarget;
  vsTcas_.tcasAltAcqTargetFt = tcasAltAcqTarget;
}

bool VerticalModeLogic::updateVsTcas(Signal id, const Frame& f, Signals& s) {
  const auto& last = f.last.longitudinal_modes;
  const bool ownCompute = !f.ext.oppModeSync;
  const auto& opp = f.in.bus_inputs.fmgc_opp_bus;
  const bool tcasRaAvail = f.ext.tcasModeAvailable;  // TA/RA mode above 900 ft RA
  auto& st = vsTcas_;

  switch (id) {
    case Signal::ReversionVs: {
      // Every frame: edge detectors and the ALT* target sample-and-hold.
      bool landLost = st.landLost.update(f.last.armed_modes.land_armed || f.last.lateral_modes.land_active, false);
      bool navLost = st.navLost.update(f.last.lateral_modes.nav_active, false);
      bool apprPushed = st.apprPushed.update(fcuWord1Bit(f, kFcuWord1ApprPushed), true);
      if (!last.alt_acq_active) {
        st.altCaptureTargetFt = s.altSelOrCstr;
      }

      const auto& fms = f.in.fms_inputs;
      float fcuAboveAircraft = fcuAltitude(f) - indicatedAltitude(f);
      bool desFamily = last.des_active || last.exp_des_active || last.op_des_active;
      bool clbFamily = last.clb_active || last.exp_clb_active || last.op_clb_active;
      bool lateralManaged = f.last.lateral_modes.nav_active || f.last.lateral_modes.loc_cpt_active ||
                            f.last.lateral_modes.loc_trk_active;

      // G/S without LAND anymore.
      bool gsLostLand = (last.gs_trk_active || last.gs_capt_active) && landLost;
      // FINAL DES without NAV, APPR pushed again, or no vertical flight plan.
      bool finalLost = last.final_des_active && (navLost || apprPushed || !fms.vertical_flight_plan_valid);
      // DES in a takeoff/climb/go-around phase, without a managed lateral mode, or without a vertical flight plan.
      bool desInvalid = last.des_active && (fms.fms_flight_phase == fmgc_flight_phase::Takeoff ||
                                            fms.fms_flight_phase == fmgc_flight_phase::Climb ||
                                            fms.fms_flight_phase == fmgc_flight_phase::Goaround || !lateralManaged ||
                                            !fms.vertical_flight_plan_valid);
      // ALT* target (FCU altitude or constraint) moved by 250 ft since ALT* engaged.
      bool altCaptureTargetChanged =
          last.alt_acq_active && std::abs(st.altCaptureTargetFt - s.altSelOrCstr) >= kAltTargetChangeFt;
      // Descending with the FCU altitude above, or climbing with it below.
      bool wrongWay = (desFamily && fcuAboveAircraft > 0.0F) || (clbFamily && fcuAboveAircraft < 0.0F);
      // TCAS mode ends in V/S once the RA is over (or TCAS RA unavailable) in submode VS.
      bool tcasEnded = (!raActive(f) || !tcasRaAvail) && last.tcas_active &&
                       (!tcasRaAvail || f.last.active_tcas_submode == tcas_submode::VS);
      // (The generated code also has VLS/VMAX speed-protection reversions, disabled by a constant 0.)
      s[id] = gsLostLand || finalLost || desInvalid || altCaptureTargetChanged || wrongWay || tcasEnded;
      return true;
    }

    case Signal::VsFpa: {
      // One latch for V/S and FPA; the HDG-V/S / TRK-FPA selection decides which one it is (VsActive, FpaActive).
      bool oppVsFpa = f.ext.oppModeSync && (oppFmgcBit(opp.discrete_word_1, kOppWord1VsActive) ||
                                            oppFmgcBit(opp.discrete_word_1, kOppWord1FpaActive));
      bool knob = st.vsKnobPushedOrPulled.update(
          fcuWord1Bit(f, kFcuWord1VsKnobPulled) || fcuWord1Bit(f, kFcuWord1VsKnobPushed), true);
      bool apFdEngaged = st.apFdEngaged.update(f.ext.anyApFdEngaged, true);
      bool noModeLast = !f.last.any_longitudinal_mode_engaged;
      bool inAir = !onGround(f);
      bool airborne5s = st.airborne5s.update(inAir, f.dt, true, kAirborneConfirmS);
      // In the air for 5 s, or on the ground with both engines stopped (the generated rtb_y_ew; its own 5 s confirm
      // node has the same input and delay as this one, so the same output).
      bool airborneOrParked = (f.in.discrete_inputs.eng_opp_stop && f.in.discrete_inputs.eng_own_stop && !inAir) ||
                              airborne5s;

      bool set = oppVsFpa || (ownCompute && ((airborneOrParked && knob && !f.last.lateral_modes.land_active) ||
                                             (inAir && apFdEngaged && noModeLast) ||  // AP/FD on in flight
                                             (airborne5s && noModeLast) ||           // 5 s after liftoff
                                             s[Signal::ReversionVs]));
      bool setRecently = st.vsFpaCondRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      // V/S and FPA are the same latch, so neither counts as "another mode".
      base_fmgc_ap_fd_logic_outputs others = f.last;
      others.longitudinal_modes.fpa_active = false;
      bool otherMode = anyOtherVerticalModeLast(others, Signal::VsActive);
      bool reset = f.ext.modeReset || (otherMode && !setRecently);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::TcasArmed: {
      bool ta = f.in.discrete_inputs.tcas_ta_display;
      bool set = (f.ext.oppModeSync && oppFmgcBit(opp.discrete_word_7, kOppWord7TcasArmed)) ||
                 (ownCompute && tcasRaAvail && ta);
      bool reset = !ta || !tcasRaAvail || last.tcas_active;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::Tcas: {
      bool raOnset = st.raOnset.update(raActive(f), true);
      bool set = (f.ext.oppModeSync && oppFmgcBit(opp.discrete_word_7, kOppWord7TcasActive)) ||
                 (ownCompute && tcasRaAvail && raOnset);
      bool setRecently = st.tcasCondRecent.update(set, f.dt, false, kOtherModeTakeoverDelayS);
      // Unlike the other modes, the mode reset is also delayed by the 0.72 s confirm.
      bool reset = (f.ext.modeReset || anyOtherVerticalModeLast(f.last, Signal::Tcas)) && !setRecently;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::TcasAltAcq: {
      // TCAS submode ALT_ACQ: the ALT* capture condition during an RA with ALT armed; abandoned when the target moves by
      // 250 ft from its value when the submode became ALT_ACQ.
      if (f.last.active_tcas_submode != tcas_submode::ALT_ACQ) {
        st.tcasAltAcqTargetFt = s.altSelOrCstr;
      }
      bool set = f.ext.vsTcas.altCaptureCond;
      bool reset = std::abs(st.tcasAltAcqTargetFt - s.altSelOrCstr) >= kAltTargetChangeFt || !raActive(f) ||
                   !f.last.armed_modes.alt_acq_armed;
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::TcasAltHold: {
      // TCAS submode ALT_HOLD: from ALT_ACQ once within the ALT hold band, during an RA with ALT armed.
      bool set = f.last.active_tcas_submode == tcas_submode::ALT_ACQ && f.ext.vsTcas.altHoldCond;
      bool reset = !f.last.armed_modes.alt_acq_armed || !raActive(f);
      s[id] = latch(set, reset, s[id]);
      return true;
    }

    case Signal::AltSoft:
      // ALT soft mode: readable/targets.cpp (the targets block evaluates it at this point of the step).
      s[id] = targets::updateAltSoft(st.altSoft, f.dt, f.in, *f.ext.logic, s[Signal::AltHold], f.ext.vsTcas.machMode,
                                     f.ext.vsTcas.speedTargetKt, s[id]);
      return true;

    case Signal::VsActive:
      s[id] = fcuWord1Bit(f, kFcuWord1HdgVsSelected) && s[Signal::VsFpa];
      return true;

    case Signal::FpaActive:
      s[id] = s[Signal::VsFpa] && !fcuWord1Bit(f, kFcuWord1HdgVsSelected);
      return true;

    default:
      return false;
  }
}

}  // namespace fmgc::readable
