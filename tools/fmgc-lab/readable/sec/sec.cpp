// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable SEC (generated SecComputer::step / initialize, SecComputer.cpp 154-1425): step, restart, sensor
// selection, landing gear monitoring, flight / ground, hydraulics, sidestick priority, engagement, pitch law capability.
// Parameters: docs/params-SecComputer.txt.

#include "sec.h"

#include <cmath>

namespace sec::readable {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::isNotFailed;

constexpr uint32_t kFw = static_cast<uint32_t>(SignStatusMatrix::FailureWarning);

bool on(const base_arinc_429& w, int n) { return bit(w, n) != 0; }

bool adrInvalid(const base_adr_bus& b) {
  return b.mach.SSM == kFw || b.airspeed_computed_kn.SSM == kFw || b.airspeed_true_kn.SSM == kFw ||
         b.aoa_corrected_deg.SSM == kFw;
}

bool irInvalid(const base_ir_bus& b) {
  return !isNormal(b.pitch_angle_deg) || !isNormal(b.roll_angle_deg) || !isNormal(b.body_yaw_rate_deg_s) ||
         !isNormal(b.body_long_accel_g) || !isNormal(b.body_lat_accel_g) || !isNormal(b.body_normal_accel_g) ||
         !isNormal(b.pitch_att_rate_deg_s) || !isNormal(b.roll_att_rate_deg_s);
}

}  // namespace

double flapHandleIndex(const base_arinc_429& status) {
  if (on(status, 17)) return 0.0;
  if (on(status, 18) && on(status, 26)) return 1.0;
  if (on(status, 18) && !on(status, 26)) return 2.0;
  if (on(status, 19)) return 3.0;
  if (on(status, 20)) return 4.0;
  if (on(status, 21)) return 5.0;
  return 0.0;
}

void SecComputer::initialize() {
  running_ = false;
  s_ = State{};
  s_.reloadThs = true;
  lateralDirect_ = {};
  pitchAlternate_ = {};
  pitchDirect_ = {};
  pitchAlternate_.init();  // the generated initialize() (1411-1425)
  in_ = sec_inputs{};
  out_ = sec_outputs{};
}

void SecComputer::restart() {
  // (233-279): every memory back to its initial state except the THS integrator (reloaded instead)
  const double thsIntegrator = s_.thsIntegrator;
  s_ = State{};
  s_.thsIntegrator = thsIntegrator;
  s_.reloadThs = true;
  lateralDirect_.reset();
  pitchAlternate_.reset();
  pitchDirect_.reset();
}

void SecComputer::selectAirData(Frame& f) {
  const base_sec_bus_inputs& b = f.in->bus_inputs;
  const bool adr1 = !adrInvalid(b.adr_1_bus), adr2 = !adrInvalid(b.adr_2_bus);
  f.singleAdrFault = !adr1 || !adr2;
  f.doubleAdrFault = !adr1 && !adr2;
  const bool ir1 = !irInvalid(b.ir_1_bus), ir2 = !irInvalid(b.ir_2_bus);
  f.singleIrFault = !ir1 || !ir2;
  f.doubleIrFault = !ir1 && !ir2;

  // ADRs (320-346): with both valid, the "average" is ADR 2 with itself (finding S1)
  const base_adr_bus* a = nullptr;
  if (adr1 && adr2) {
    f.vIas = (b.adr_2_bus.airspeed_computed_kn.Data + b.adr_2_bus.airspeed_computed_kn.Data) / 2.0F;
    f.vTas = (b.adr_2_bus.airspeed_true_kn.Data + b.adr_2_bus.airspeed_true_kn.Data) / 2.0F;
    f.mach = (b.adr_2_bus.mach.Data + b.adr_2_bus.mach.Data) / 2.0F;
    f.alpha = (b.adr_2_bus.aoa_corrected_deg.Data + b.adr_2_bus.aoa_corrected_deg.Data) / 2.0F;
  } else {
    a = adr1 ? &b.adr_1_bus : (adr2 ? &b.adr_2_bus : nullptr);
    f.vIas = a ? a->airspeed_computed_kn.Data : 0.0F;
    f.vTas = a ? a->airspeed_true_kn.Data : 0.0F;
    f.mach = a ? a->mach.Data : 0.0F;
    f.alpha = a ? a->aoa_corrected_deg.Data : 0.0F;
  }

  // IRs (351-402): the average of both, else the valid one, else 0
  const base_ir_bus& x = b.ir_1_bus;
  const base_ir_bus& y = b.ir_2_bus;
  if (ir1 && ir2) {
    f.theta = (x.pitch_angle_deg.Data + y.pitch_angle_deg.Data) / 2.0F;
    f.phi = (x.roll_angle_deg.Data + y.roll_angle_deg.Data) / 2.0F;
    f.q = (x.body_pitch_rate_deg_s.Data + y.body_pitch_rate_deg_s.Data) / 2.0F;
    f.r = (x.body_yaw_rate_deg_s.Data + y.body_yaw_rate_deg_s.Data) / 2.0F;
    f.nx = (x.body_long_accel_g.Data + y.body_long_accel_g.Data) / 2.0F;
    f.ny = (x.body_lat_accel_g.Data + y.body_lat_accel_g.Data) / 2.0F;
    f.nz = (x.body_normal_accel_g.Data + y.body_normal_accel_g.Data) / 2.0F;
    f.thetaDot = (x.pitch_att_rate_deg_s.Data + y.pitch_att_rate_deg_s.Data) / 2.0F;
    f.phiDot = (x.roll_att_rate_deg_s.Data + y.roll_att_rate_deg_s.Data) / 2.0F;
  } else if (ir1 || ir2) {
    const base_ir_bus& v = ir1 ? x : y;
    f.theta = v.pitch_angle_deg.Data;
    f.phi = v.roll_angle_deg.Data;
    f.q = v.body_pitch_rate_deg_s.Data;
    f.r = v.body_yaw_rate_deg_s.Data;
    f.nx = v.body_long_accel_g.Data;
    f.ny = v.body_lat_accel_g.Data;
    f.nz = v.body_normal_accel_g.Data;
    f.thetaDot = v.pitch_att_rate_deg_s.Data;
    f.phiDot = v.roll_att_rate_deg_s.Data;
  } else {
    f.theta = f.phi = f.q = f.r = f.nx = f.ny = f.nz = f.thetaDot = f.phiDot = 0.0F;
  }
}

void SecComputer::monitorLandingGear(Frame& f) {
  const base_sec_bus_inputs& b = f.in->bus_inputs;
  const double dt = f.in->time.dt;
  const base_arinc_429& l1 = b.lgciu_1_bus.discrete_word_3;
  const base_arinc_429& l2 = b.lgciu_2_bus.discrete_word_3;
  const bool valid1 = isNotFailed(l1), valid2 = isNotFailed(l2);

  // LGCIU discrete word 3 bits 11-13 (a gear not uplocked) and 14 (408-476): a gear not uplocked when a valid LGCIU
  // says so and the other is invalid or agrees
  const bool any1 = on(l1, 11) || on(l1, 12) || on(l1, 13);
  const bool any2 = on(l2, 11) || on(l2, 12) || on(l2, 13);
  const bool says1 = valid1 && any1, says2 = valid2 && any2;
  f.gearNotUplocked = (says2 && !valid1) || (says1 && !valid2) || (says2 && says1);
  // fault: one LGCIU reports bit 14 with no gear bit while the other reports bit 14 with a gear bit for 10 s, or both
  // LGCIUs invalid
  const bool l1Bit14Only = on(l1, 14) && valid1 && !on(l1, 11) && !on(l1, 12) && !on(l1, 13);
  const bool l2Bit14WithGear = s_.lgciu2UplockConfirm.update(on(l2, 14) && valid2 && any2, dt, true, 10.0);
  const bool l2Bit14Only = on(l2, 14) && valid2 && !on(l2, 11) && !on(l2, 12) && !on(l2, 13);
  const bool l1Bit14WithGear = s_.lgciu1UplockConfirm.update(on(l1, 14) && valid1 && any1, dt, true, 10.0);
  f.lgciuFault = (l1Bit14Only && l2Bit14WithGear) || (!valid1 && !valid2) || (l2Bit14Only && l1Bit14WithGear);

  // on ground (477-506): any main gear compressed (discrete word 2 bits 13 / 14 of either LGCIU)
  const bool mainA = on(b.lgciu_1_bus.discrete_word_2, 14) || on(b.lgciu_2_bus.discrete_word_2, 14);
  const bool mainB = on(b.lgciu_1_bus.discrete_word_2, 13) || on(b.lgciu_2_bus.discrete_word_2, 13);
  if (!s_.groundChartActive) {
    s_.groundChartActive = true;
    s_.groundPhase = GroundPhase::OnGround;
    f.onGround = true;
  } else if (s_.groundPhase == GroundPhase::InAir) {
    if (static_cast<double>(mainA) > 0.1 || static_cast<double>(mainB) > 0.1) {
      s_.groundPhase = GroundPhase::OnGround;
      f.onGround = true;
    } else {
      f.onGround = false;
    }
  } else if (!mainA && !mainB) {
    s_.groundPhase = GroundPhase::InAir;
    f.onGround = false;
  } else {
    f.onGround = true;
  }
  f.trackingMode = f.in->sim_data.slew_on || f.in->sim_data.pause_on || f.in->sim_data.tracking_mode_on_override;
}

void SecComputer::updateFlightPhase(Frame& f) {
  // the ELAC's flight / ground chart (510-546) without radio height (the 400 ft condition compares a constant 0)
  const double now = f.in->time.simulation_time;
  if (!s_.flightChartActive) {
    s_.flightChartActive = true;
    s_.flightPhase = FlightPhase::Ground;
    s_.inFlight = 0.0;
  } else if (s_.flightPhase == FlightPhase::Flight) {
    if (f.onGround && f.theta < 2.5F) {
      s_.onGroundTime = now;
      s_.flightPhase = FlightPhase::FlightToGround;
    } else {
      s_.inFlight = 1.0;
    }
  } else if (s_.flightPhase == FlightPhase::FlightToGround) {
    if (now - s_.onGroundTime >= 5.0) {
      s_.flightPhase = FlightPhase::Ground;
      s_.inFlight = 0.0;
    } else if (!f.onGround || f.theta >= 2.5F) {
      s_.onGroundTime = 0.0;
      s_.flightPhase = FlightPhase::Flight;
      s_.inFlight = 1.0;
    }
  } else if (!f.onGround && f.theta > 8.0F) {
    s_.onGroundTime = 0.0;
    s_.flightPhase = FlightPhase::Flight;
    s_.inFlight = 1.0;
  } else {
    s_.inFlight = 0.0;
  }
  f.inFlight = s_.inFlight != 0.0;

  // hydraulics (549-557): the pressure switches, confirmed 0.5 s (no pressure values)
  const sec_inputs& in = *f.in;
  f.yellowAvail = s_.yellowConfirm.update(!in.discrete_inputs.yellow_low_pressure, in.time.dt, true, 0.5);
  f.blueAvail = s_.blueConfirm.update(!in.discrete_inputs.blue_low_pressure, in.time.dt, true, 0.5);
  f.greenAvail = s_.greenConfirm.update(!in.discrete_inputs.green_low_pressure, in.time.dt, true, 0.5);
}

void SecComputer::updateEngagement(Frame& f) {
  const sec_inputs& in = *f.in;
  const base_sec_discrete_inputs& d = in.discrete_inputs;

  // sidestick priority (560-605), as the ELAC
  const bool capt = d.capt_priority_takeover_pressed, fo = d.fo_priority_takeover_pressed;
  const bool captPressed = s_.captTakeover.update(capt, true);
  const bool foPressed = s_.foTakeover.update(fo, true);
  if (captPressed) {
    s_.rightStickDisabled = true;
    s_.leftStickDisabled = false;
  } else if (foPressed) {
    s_.leftStickDisabled = true;
    s_.rightStickDisabled = false;
  }
  if (s_.rightStickDisabled && !capt && !s_.rightLockedPrev) {
    s_.rightStickDisabled = false;
  } else if (s_.leftStickDisabled) {
    s_.leftStickDisabled = fo || s_.leftLockedPrev;
  }
  f.leftStickLocked = s_.leftLockConfirm.update(s_.leftStickDisabled && (fo || s_.leftLockedPrev), in.time.dt, true, 30.0);
  f.rightStickLocked =
      s_.rightLockConfirm.update(s_.rightStickDisabled && (capt || s_.rightLockedPrev), in.time.dt, true, 30.0);
  s_.leftLockedPrev = f.leftStickLocked;  // (1402-1403)
  s_.rightLockedPrev = f.rightStickLocked;
  const double captRoll = s_.leftStickDisabled ? 0.0 : in.analog_inputs.capt_roll_stick_pos;
  const double foRoll = s_.rightStickDisabled ? 0.0 : in.analog_inputs.fo_roll_stick_pos;
  f.rollStick = blocks::limit(foRoll + captRoll, -1.0, 1.0);

  // elevators (607-637): SEC 1 on blue, SECs 2 and 3 left on green and right on yellow; SEC 3 never in pitch. SEC 1
  // takes pitch when both ELACs and SEC 2's elevators are lost, SEC 2 when both ELACs are lost.
  if (d.is_unit_1) {
    f.leftElevAvail = !d.l_elev_servo_failed && f.blueAvail;
    f.rightElevAvail = !d.r_elev_servo_failed && f.blueAvail;
  } else {
    f.leftElevAvail = !d.l_elev_servo_failed && f.greenAvail;
    f.rightElevAvail = !d.r_elev_servo_failed && f.yellowAvail;
  }
  f.thsAvail = !d.ths_motor_fault && (f.yellowAvail || f.greenAvail);
  f.canEngageInPitch = (f.leftElevAvail || f.rightElevAvail) && !d.is_unit_3;
  // spoiler pairs: SEC 1 pair 1 (spoilers 3) blue, pair 2 (spoilers 4) yellow; SEC 2 one pair (spoilers 5) green;
  // SEC 3 pair 1 (spoilers 1) green, pair 2 (spoilers 2) yellow (FlyByWireInterface::updateSec spoiler indexes)
  bool pair1Supply, pair2Supply;
  if (d.is_unit_1) {
    f.hasPriorityInPitch = d.pitch_not_avail_elac_1 && d.pitch_not_avail_elac_2 && d.left_elev_not_avail_sec_opp &&
                           d.right_elev_not_avail_sec_opp;
    pair1Supply = f.blueAvail;
    pair2Supply = f.yellowAvail;
  } else {
    f.hasPriorityInPitch = d.is_unit_2 && (d.pitch_not_avail_elac_1 && d.pitch_not_avail_elac_2);
    pair1Supply = f.greenAvail;
    pair2Supply = d.is_unit_2 ? false : f.yellowAvail;
  }
  f.isEngagedInPitch = f.canEngageInPitch && f.hasPriorityInPitch;
  f.spoilerPair1Avail = !d.l_spoiler_1_servo_failed && !d.r_spoiler_1_servo_failed && pair1Supply;
  f.spoilerPair2Avail = !d.l_spoiler_2_servo_failed && !d.r_spoiler_2_servo_failed && pair2Supply;
  // "engaged in roll" with both ELACs' digital outputs failed (the roll spoilers are driven anyway, see spoilers.cpp)
  f.isEngagedInRoll = (f.spoilerPair1Avail || f.spoilerPair2Avail) && d.digital_output_failed_elac_1 &&
                      d.digital_output_failed_elac_2;
}

void SecComputer::updateLawCapability(Frame& f) {
  const sec_inputs& in = *f.in;
  const base_sec_bus_inputs& b = in.bus_inputs;
  const base_sec_discrete_inputs& d = in.discrete_inputs;

  // ELACs engaged in roll (discrete word 1 bit 20), slats / flaps from the SFCCs (actual position word bit 20)
  // (645-660)
  const bool elac1Roll = on(b.elac_1_bus.discrete_status_word_1, 20) && isNotFailed(b.elac_1_bus.discrete_status_word_1);
  const bool elac2Roll = on(b.elac_2_bus.discrete_status_word_1, 20) && isNotFailed(b.elac_2_bus.discrete_status_word_1);
  const base_arinc_429& sfcc1 = b.sfcc_1_bus.slat_flap_actual_position_word;
  const base_arinc_429& sfcc2 = b.sfcc_2_bus.slat_flap_actual_position_word;
  const bool sfccBit20 = (on(sfcc1, 20) && isNotFailed(sfcc1)) || (on(sfcc2, 20) && isNotFailed(sfcc2));

  // abnormal attitude (661-670): as the ELAC, but unfiltered alpha, with both ADRs / IRs failed as the exclusions
  const bool slewing = s_.slewConfirm.update(in.sim_data.slew_on, in.time.dt, false, 0.2);
  f.abnormalCondition =
      !slewing && !f.onGround &&
      ((!f.doubleAdrFault && (f.mach > 0.91 || f.alpha < -10.0F || f.alpha > 40.0F || f.vIas > 440.0F ||
                              f.vIas < 60.0F)) ||
       (!f.doubleIrFault && (std::abs(static_cast<double>(f.phi)) > 125.0 || (f.theta > 50.0F || f.theta < -30.0F))));
  s_.abnormalWasActive = f.abnormalCondition || (!f.onGround && s_.abnormalWasActive);

  // pitch law capability (671-686): direct with both IRs failed, or in flight with a gear not uplocked (slats / flaps
  // with the LGCIUs in fault); alternate 2 with both ADRs failed, after an abnormal attitude, or with an elevator lost
  // and neither ELAC engaged in roll; else alternate 1. Active = capability when engaged in pitch.
  if (f.doubleIrFault || (f.inFlight && ((f.gearNotUplocked && !f.lgciuFault) || (sfccBit20 && f.lgciuFault)))) {
    f.pitchLawCapability = pitch_efcs_law::DirectLaw;
  } else if (f.doubleAdrFault || s_.abnormalWasActive ||
             (!elac1Roll && !elac2Roll && (!f.leftElevAvail || !f.rightElevAvail))) {
    f.pitchLawCapability = pitch_efcs_law::AlternateLaw2;
  } else {
    f.pitchLawCapability = pitch_efcs_law::AlternateLaw1;
  }
  f.activePitchLaw = f.isEngagedInPitch ? f.pitchLawCapability : pitch_efcs_law::None;

  // pitch stick (688-705)
  const double foPitch = s_.rightStickDisabled ? 0.0 : in.analog_inputs.fo_pitch_stick_pos;
  double pitch = s_.leftStickDisabled ? 0.0 : in.analog_inputs.capt_pitch_stick_pos;
  pitch += foPitch;
  f.pitchStick = blocks::limit(pitch, -1.0, 1.0);

  // THS (707-720): SEC 1 only, when both ELACs lost pitch; the ground setting after touchdown as the ELAC's
  const bool landed = s_.landing.update(f.inFlight, false);
  f.elacsPitchLost = d.pitch_not_avail_elac_1 && d.pitch_not_avail_elac_2;
  const bool thsEngageable = d.is_unit_1 && f.thsAvail && f.elacsPitchLost;
  const bool atZero = std::abs(in.analog_inputs.ths_pos_deg) <= 0.02;
  s_.thsGroundLatch = blocks::SrLatch{s_.thsGroundLatch}.update(landed, !thsEngageable || atZero || d.ths_override_active);
  f.thsGroundSetting = thsEngageable && s_.thsGroundLatch;
  f.thsActiveCommanded = (f.isEngagedInPitch && f.inFlight &&
                          (f.activePitchLaw != pitch_efcs_law::DirectLaw && !f.abnormalCondition)) ||
                         f.thsGroundSetting;
}

const sec_outputs& SecComputer::step(const sec_inputs& in) {
  if (!in.sim_data.computer_running) {
    running_ = false;  // (1406-1408) outputs and memories held
    return out_;
  }
  if (!running_) {
    restart();
    running_ = true;
  }
  in_ = in;
  Frame f;
  f.in = &in_;
  selectAirData(f);
  monitorLandingGear(f);
  updateFlightPhase(f);
  updateEngagement(f);
  updateLawCapability(f);
  updateSpoilers(f);
  updatePitchOrders(f);
  packOutputs(f);
  return out_;
}

}  // namespace sec::readable
