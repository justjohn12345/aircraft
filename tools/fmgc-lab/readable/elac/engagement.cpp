// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ELAC engagement (generated ElacComputer.cpp 802-1085): sidestick priority, surface availability, pitch and roll
// engagement, law capability and active laws, THS engagement.

#include <cmath>

#include "elac.h"

namespace elac::readable {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::isNotFailed;

bool isOn(const base_arinc_429& w, int n) {
  return bit(w, n) != 0;
}

}  // namespace

void ElacComputer::updateSidestickPriority(Frame& f) {
  // (802-847) a takeover pushbutton press disables the other sidestick; it stays disabled while the button is held,
  // and for good once held 30 s (priority locked). The last press wins.
  const elac_inputs& in = *f.in;
  EngagementState& e = engagement_;
  const bool capt = in.discrete_inputs.capt_priority_takeover_pressed;
  const bool fo = in.discrete_inputs.fo_priority_takeover_pressed;
  const bool captPressed = e.captTakeover.update(capt, true);
  const bool foPressed = e.foTakeover.update(fo, true);
  if (captPressed) {
    e.rightStickDisabled = true;
    e.leftStickDisabled = false;
  } else if (foPressed) {
    e.leftStickDisabled = true;
    e.rightStickDisabled = false;
  }
  if (e.rightStickDisabled && !capt && !e.rightLockedPrev) {
    e.rightStickDisabled = false;
  } else if (e.leftStickDisabled) {
    e.leftStickDisabled = fo || e.leftLockedPrev;
  }
  f.leftStickLocked =
      e.leftLockConfirm.update(e.leftStickDisabled && (fo || e.leftLockedPrev), in.time.dt, true, 30.0);
  f.rightStickLocked =
      e.rightLockConfirm.update(e.rightStickDisabled && (capt || e.rightLockedPrev), in.time.dt, true, 30.0);
  // read next frame (the generated delays are written at the end of the step, 1909-1910)
  e.leftLockedPrev = f.leftStickLocked;
  e.rightLockedPrev = f.rightStickLocked;

  // total stick deflections, +-1 (842-847, 1054-1071)
  const double captRoll = e.leftStickDisabled ? 0.0 : in.analog_inputs.capt_roll_stick_pos;
  const double foRoll = e.rightStickDisabled ? 0.0 : in.analog_inputs.fo_roll_stick_pos;
  f.rollStick = blocks::limit(foRoll + captRoll, -1.0, 1.0);
}

void ElacComputer::updateEngagement(Frame& f) {
  const base_elac_discrete_inputs& d = f.in->discrete_inputs;
  // elevators and THS (849-869): ELAC 1 drives both elevators on blue, ELAC 2 the left on green and the right on
  // yellow; the THS motors need green or yellow. ELAC 2 can engage in pitch with green and yellow, or with green or
  // yellow when blue is lost.
  if (d.is_unit_1) {
    f.leftElevAvail = !d.l_elev_servo_failed && f.blueAvail;
    f.rightElevAvail = !d.r_elev_servo_failed && f.blueAvail;
  } else {
    f.leftElevAvail = !d.l_elev_servo_failed && f.greenAvail;
    f.rightElevAvail = !d.r_elev_servo_failed && f.yellowAvail;
  }
  f.thsAvail = !d.ths_motor_fault && (f.yellowAvail || f.greenAvail);
  const bool pitchHydraulics = d.is_unit_1 ? f.blueAvail
                                           : (f.yellowAvail && f.greenAvail) ||
                                                 (!f.blueAvail && (f.greenAvail || f.yellowAvail));
  f.canEngageInPitch = !d.r_elev_servo_failed && !d.l_elev_servo_failed && !d.ths_motor_fault && pitchHydraulics;
  // ELAC 2 has pitch priority; ELAC 1 takes over when ELAC 2 reports its pitch axis failed
  f.hasPriorityInPitch = !d.is_unit_1 || d.opp_axis_pitch_failure;
  f.isEngagedInPitch = f.canEngageInPitch && f.hasPriorityInPitch;

  // ailerons (870-893): ELAC 1 left on blue, right on green; ELAC 2 the opposite. ELAC 1 has roll priority; ELAC 2
  // takes over when ELAC 1 has lost both ailerons, and otherwise cross-commands an aileron ELAC 1 lost with the
  // opposite ELAC's aileron order
  if (d.is_unit_1) {
    f.leftAileronAvail = !d.l_ail_servo_failed && f.blueAvail;
    f.rightAileronAvail = !d.r_ail_servo_failed && f.greenAvail;
  } else {
    f.leftAileronAvail = !d.l_ail_servo_failed && f.greenAvail;
    f.rightAileronAvail = !d.r_ail_servo_failed && f.blueAvail;
  }
  f.canEngageInRoll = f.leftAileronAvail || f.rightAileronAvail;
  f.hasPriorityInRoll = d.is_unit_1 || (d.opp_left_aileron_lost && d.opp_right_aileron_lost);
  if (!d.is_unit_1 && !f.hasPriorityInRoll && isNormal(f.in->bus_inputs.elac_opp_bus.aileron_command_deg)) {
    f.leftAileronCrossCommand = d.opp_left_aileron_lost && f.leftAileronAvail;
    f.rightAileronCrossCommand = d.opp_right_aileron_lost && f.rightAileronAvail;
  } else {
    f.leftAileronCrossCommand = false;
    f.rightAileronCrossCommand = false;
  }
  f.isEngagedInRoll = f.canEngageInRoll && f.hasPriorityInRoll;
}

void ElacComputer::updateLawCapability(Frame& f) {
  const elac_inputs& in = *f.in;
  const base_elac_bus_inputs& b = in.bus_inputs;
  const base_elac_discrete_inputs& d = in.discrete_inputs;

  // lateral (895-930): direct law with both FACs' yaw damping lost, with all three SECs lost (SEC 1 and 2 words not
  // normal and both FCDCs reporting SEC 3 invalid), or when SEC 1 bits 15 and 16, SEC 2 bit 15 and the FCDCs' copies
  // of SEC 1 bits 15 / 16 (status word 3 bits 23 / 24) are all clear (Fcdc.cpp 601-602)
  const bool sec1Bit15 = isOn(b.sec_1_bus.discrete_status_word_1, 15);
  const bool sec1Bit16 = isOn(b.sec_1_bus.discrete_status_word_1, 16);
  const bool sec2Bit15 = isOn(b.sec_2_bus.discrete_status_word_1, 15);
  const bool fcdcSec1Bit15 = isOn(b.fcdc_1_bus.efcs_status_word_3, 23) || isOn(b.fcdc_2_bus.efcs_status_word_3, 23);
  const bool fcdcSec1Bit16 = isOn(b.fcdc_1_bus.efcs_status_word_3, 24) || isOn(b.fcdc_2_bus.efcs_status_word_3, 24);
  const bool fcdc1Sec3Failed = isOn(b.fcdc_1_bus.efcs_status_word_1, 29);
  const bool fcdc2Sec3Failed = isOn(b.fcdc_2_bus.efcs_status_word_1, 29);
  const bool allSecsLost = !isNormal(b.sec_1_bus.discrete_status_word_1) &&
                           !isNormal(b.sec_2_bus.discrete_status_word_1) && fcdc1Sec3Failed && fcdc2Sec3Failed;
  if ((d.fac_1_yaw_control_lost && d.fac_2_yaw_control_lost) || allSecsLost ||
      (!sec1Bit15 && !sec1Bit16 && !sec2Bit15 && !fcdcSec1Bit15 && !fcdcSec1Bit16)) {
    f.lateralLawCapability = lateral_efcs_law::DirectLaw;
  } else {
    f.lateralLawCapability = lateral_efcs_law::NormalLaw;
  }

  // landing gear down from the SECs (discrete word 2 bit 18, either SEC), or, when neither SEC gives it (both words
  // failed, or both report bit 17), slats / flaps from the SFCCs (actual position word bit 20) (932-953)
  const bool sec1Word2 = isNotFailed(b.sec_1_bus.discrete_status_word_2);
  const bool sec2Word2 = isNotFailed(b.sec_2_bus.discrete_status_word_2);
  const bool secGearUnknown = (!sec1Word2 && !sec2Word2) || (isOn(b.sec_1_bus.discrete_status_word_2, 17) &&
                                                             isOn(b.sec_2_bus.discrete_status_word_2, 17));
  const bool secGearDown = isOn(b.sec_1_bus.discrete_status_word_2, 18) || isOn(b.sec_2_bus.discrete_status_word_2, 18);
  const base_arinc_429& sfcc1 = b.sfcc_1_bus.slat_flap_actual_position_word;
  const base_arinc_429& sfcc2 = b.sfcc_2_bus.slat_flap_actual_position_word;
  const bool sfccBit20 = (isOn(sfcc1, 20) && isNotFailed(sfcc1)) || (isOn(sfcc2, 20) && isNotFailed(sfcc2));
  const bool gearDown = (secGearDown && !secGearUnknown) || (sfccBit20 && secGearUnknown);

  // abnormal attitude law (954-964): in flight and not within 0.2 s after a slew, CAS > 440 or < 60 kt, Mach > 0.91,
  // AoA > 40 or < -10 deg (ADRs not all failed), or bank > 125 deg, pitch > 50 or < -30 deg (IRs not all failed);
  // remembered until the ground
  EngagementState& e = engagement_;
  const bool slewing = e.slewConfirm.update(in.sim_data.slew_on, in.time.dt, false, 0.2);
  const double absPhi = std::abs(static_cast<double>(f.phi));
  f.abnormalCondition =
      !slewing && !f.onGround &&
      ((!f.tripleAdrFault && (f.mach > 0.91 || f.alphaFiltered < -10.0 || f.alphaFiltered > 40.0 || f.vIas > 440.0F ||
                              f.vIas < 60.0F)) ||
       (!f.tripleIrFault && (absPhi > 125.0 || (f.theta > 50.0F || f.theta < -30.0F))));
  e.abnormalWasActive = f.abnormalCondition || (!f.onGround && e.abnormalWasActive);

  // pitch (965-983): alternate law 2 with all ADRs failed, after an abnormal attitude, or with both ailerons and an
  // elevator lost; alternate law 1 with two IRs or two ADRs failed or an elevator lost; direct law with all IRs failed,
  // or in flight with the gear down when the conditions for an alternate law (or both RAs failed, or the lateral
  // direct law) hold
  const bool elevatorLost = !f.leftElevAvail || !f.rightElevAvail;
  const bool alternate2 = f.tripleAdrFault || e.abnormalWasActive ||
                          (!f.leftAileronAvail && !f.rightAileronAvail && elevatorLost);
  const bool alternate1 = f.doubleIrFault || f.doubleAdrFault || elevatorLost;
  if (f.tripleIrFault || ((alternate2 || alternate1 || f.dualRaFailure ||
                           f.lateralLawCapability == lateral_efcs_law::DirectLaw) &&
                          f.inFlight && gearDown)) {
    f.pitchLawCapability = pitch_efcs_law::DirectLaw;
  } else if (alternate2) {
    f.pitchLawCapability = pitch_efcs_law::AlternateLaw2;
  } else if (alternate1) {
    f.pitchLawCapability = pitch_efcs_law::AlternateLaw1;
  } else {
    f.pitchLawCapability = pitch_efcs_law::NormalLaw;
  }
}

void ElacComputer::updateActiveLaws(Frame& f) {
  const base_arinc_429& opp = f.in->bus_inputs.elac_opp_bus.discrete_status_word_2;
  // the opposite ELAC's capabilities (985-999): discrete word 2 bits 11 / 12 (pitch: normal or direct / an alternate
  // or direct: both = direct; ALT2 reads as ALT1, finding L4), bits 13 / 14 (lateral normal / direct)
  const bool oppPitch11 = isOn(opp, 11), oppPitch12 = isOn(opp, 12);
  const bool oppLateralNormal = isOn(opp, 13), oppLateralDirect = isOn(opp, 14);
  lateral_efcs_law oppLateralCap;
  if (oppLateralNormal && !oppLateralDirect) {
    oppLateralCap = lateral_efcs_law::NormalLaw;
  } else if (!oppLateralNormal && oppLateralDirect) {
    oppLateralCap = lateral_efcs_law::DirectLaw;
  } else {
    oppLateralCap = lateral_efcs_law::None;
  }

  // the capabilities of the ELAC in charge of pitch (1001-1019): this one when engaged in pitch, else the other one
  pitch_efcs_law pitchCap;
  lateral_efcs_law pitchElacLateralCap;
  if (f.hasPriorityInPitch && f.isEngagedInPitch) {
    pitchCap = f.pitchLawCapability;
    pitchElacLateralCap = f.lateralLawCapability;
  } else {
    if (oppPitch11 && !oppPitch12) {
      pitchCap = pitch_efcs_law::NormalLaw;
    } else if (!oppPitch11 && oppPitch12) {
      pitchCap = pitch_efcs_law::AlternateLaw1;
    } else if (oppPitch11 && oppPitch12) {
      pitchCap = pitch_efcs_law::DirectLaw;
    } else {
      pitchCap = pitch_efcs_law::None;
    }
    pitchElacLateralCap = oppLateralCap;
  }
  // the lateral capability of the ELAC in charge of roll (1021-1025)
  const lateral_efcs_law rollCap =
      f.hasPriorityInRoll && f.isEngagedInRoll ? f.lateralLawCapability : oppLateralCap;
  const bool allNormal = rollCap == lateral_efcs_law::NormalLaw && pitchCap == pitch_efcs_law::NormalLaw &&
                         pitchElacLateralCap == lateral_efcs_law::NormalLaw;

  // active laws (1027-1052): normal laws only when both ELACs are capable of them; roll otherwise direct; pitch
  // otherwise alternate 1 when only roll lost its normal law, else this ELAC's own capability; direct when the
  // pitch ELAC alone has lost the lateral normal law (finding L5)
  if (f.isEngagedInRoll) {
    f.activeLateralLaw = allNormal ? lateral_efcs_law::NormalLaw : lateral_efcs_law::DirectLaw;
  } else {
    f.activeLateralLaw = lateral_efcs_law::None;
  }
  if (!f.isEngagedInPitch) {
    f.activePitchLaw = pitch_efcs_law::None;
  } else if (allNormal) {
    f.activePitchLaw = pitch_efcs_law::NormalLaw;
  } else if (rollCap != lateral_efcs_law::NormalLaw && pitchCap == pitch_efcs_law::NormalLaw) {
    f.activePitchLaw = pitch_efcs_law::AlternateLaw1;
  } else if (pitchCap != pitch_efcs_law::NormalLaw) {
    f.activePitchLaw = f.pitchLawCapability;
  } else {
    f.activePitchLaw = pitch_efcs_law::DirectLaw;
  }

  // pitch stick total (1054-1071)
  const elac_inputs& in = *f.in;
  const double foPitch = engagement_.rightStickDisabled ? 0.0 : in.analog_inputs.fo_pitch_stick_pos;
  const double captPitch = engagement_.leftStickDisabled ? 0.0 : in.analog_inputs.capt_pitch_stick_pos;
  f.pitchStick = blocks::limit(foPitch + captPitch, -1.0, 1.0);
}

void ElacComputer::updateThsEngagement(Frame& f) {
  // (1073-1085) after touchdown (falling edge of in flight) the THS goes back to 0 (ground setting) until it is within
  // 0.02 deg of 0, the THS is lost or manually overridden. The THS is commanded in flight in normal or alternate law
  // (not in direct law, not with an abnormal attitude), and for the ground setting.
  const base_elac_discrete_inputs& d = f.in->discrete_inputs;
  EngagementState& e = engagement_;
  const bool landed = e.landing.update(f.inFlight, false);
  const bool thsEngageable = (d.is_unit_1 && f.thsAvail && f.canEngageInPitch) ||
                             (d.is_unit_2 && f.thsAvail && f.canEngageInPitch && d.opp_axis_pitch_failure);
  const bool atZero = std::abs(f.in->analog_inputs.ths_pos_deg) <= 0.02;
  e.thsGroundLatch = blocks::SrLatch{e.thsGroundLatch}.update(landed, !thsEngageable || atZero || d.ths_override_active);
  f.thsGroundSetting = thsEngageable && e.thsGroundLatch;
  f.thsActiveCommanded = (f.isEngagedInPitch && f.inFlight &&
                          (f.activePitchLaw != pitch_efcs_law::DirectLaw && !f.abnormalCondition)) ||
                         f.thsGroundSetting;
}

}  // namespace elac::readable
