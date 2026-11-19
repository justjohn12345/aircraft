// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SEC outputs (generated SecComputer.cpp 1139-1405).
//
// SEC bus (SSM: NO NormalOperation, NCD NoComputedData; bits 1-based):
//   spoiler 1 / 2 left / right, elevator left / right, THS position  NO with the measured position, NCD 0 when failed
//   sidestick pitch / roll commands     NO: 16 deg x pitch, 20 deg x roll stick position
//   speed brake lever command           NO: 30 deg x lever position (-0.05 when armed)
//   speed brake command                 NO: the speed brake order (deg, negative up)
//   thrust lever angle 1 / 2            NO: the thrust lever angles
//   discrete status word 1 (NO)         11 / 12 spoiler pair 1 / 2 servo failed; 13 / 14 L/R elevator servo failed;
//                                       15 / 16 spoiler pair 1 / 2 available; 17 / 18 L/R elevator available; 19 ALT2;
//                                       20 ALT1 or ALT2; 21 direct; 22 engaged in roll; 23 engaged in pitch; 25 ground
//                                       spoilers out; 26 ground spoilers armed
//   discrete status word 2 (NO)         13 / 14 left / right sidestick disabled; 15 / 16 priority locked; 17 LGCIU
//                                       uplock disagree or fault; 18 a landing gear not uplocked

#include "sec.h"

namespace sec::readable {

namespace {

constexpr uint32_t kNcd = static_cast<uint32_t>(SignStatusMatrix::NoComputedData);
constexpr uint32_t kNo = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);

void position(base_arinc_429& w, bool failed, double value) {
  w.SSM = failed ? kNcd : kNo;
  w.Data = failed ? 0.0F : static_cast<float>(value);
}

}  // namespace

void SecComputer::packOutputs(const Frame& f) {
  const sec_inputs& in = *f.in;
  const base_sec_discrete_inputs& d = in.discrete_inputs;
  const base_sec_analog_inputs& a = in.analog_inputs;
  sec_outputs& o = out_;
  base_sec_out_bus& bus = o.bus_outputs;
  const pitch_efcs_law law = f.activePitchLaw;

  const bool word1[19] = {
      d.l_spoiler_1_servo_failed || d.r_spoiler_1_servo_failed,
      d.l_spoiler_2_servo_failed || d.r_spoiler_2_servo_failed,
      d.l_elev_servo_failed != 0,
      d.r_elev_servo_failed != 0,
      f.spoilerPair1Avail,
      f.spoilerPair2Avail,
      f.leftElevAvail,
      f.rightElevAvail,
      law == pitch_efcs_law::AlternateLaw2,
      law == pitch_efcs_law::AlternateLaw1 || law == pitch_efcs_law::AlternateLaw2,
      law == pitch_efcs_law::DirectLaw,
      f.isEngagedInRoll,
      f.isEngagedInPitch,
      false,
      s_.groundSpoilersOut,
      f.groundSpoilersArmed,
      false,
      false,
      false,
  };
  bus.discrete_status_word_1.Data = blocks::packBits(word1);
  const bool word2[19] = {
      false, false, s_.leftStickDisabled, s_.rightStickDisabled, f.leftStickLocked, f.rightStickLocked,
      f.lgciuFault, f.gearNotUplocked, false, false, false, false, false, false, false, false, false, false, false,
  };
  bus.discrete_status_word_2.Data = blocks::packBits(word2);

  o.data = in;
  o.laws.lateral_law_outputs.left_spoiler_1_command_deg = f.leftSpoiler1;
  o.laws.lateral_law_outputs.right_spoiler_1_command_deg = f.rightSpoiler1;
  o.laws.lateral_law_outputs.left_spoiler_2_command_deg = f.leftSpoiler2;
  o.laws.lateral_law_outputs.right_spoiler_2_command_deg = f.rightSpoiler2;
  o.laws.lateral_law_outputs.speedbrake_command_deg = f.speedBrakeCommand;
  o.laws.pitch_law_outputs.elevator_command_deg = f.elevatorCommand;
  o.laws.pitch_law_outputs.ths_command_deg = f.thsCommand;

  base_sec_logic_outputs& l = o.logic;
  l.on_ground = f.onGround;
  l.pitch_law_in_flight = f.inFlight;
  l.tracking_mode_on = f.trackingMode;
  l.pitch_law_capability = f.pitchLawCapability;
  l.active_pitch_law = f.activePitchLaw;
  l.abnormal_condition_law_active = f.abnormalCondition;
  l.is_engaged_in_pitch = f.isEngagedInPitch;
  l.can_engage_in_pitch = f.canEngageInPitch;
  l.has_priority_in_pitch = f.hasPriorityInPitch;
  l.left_elevator_avail = f.leftElevAvail;
  l.right_elevator_avail = f.rightElevAvail;
  l.ths_avail = f.thsAvail;
  l.ths_active_commanded = f.thsActiveCommanded;
  l.ths_ground_setting_active = f.thsGroundSetting;
  l.is_engaged_in_roll = f.isEngagedInRoll;
  l.spoiler_pair_1_avail = f.spoilerPair1Avail;
  l.spoiler_pair_2_avail = f.spoilerPair2Avail;
  l.is_yellow_hydraulic_power_avail = f.yellowAvail;
  l.is_blue_hydraulic_power_avail = f.blueAvail;
  l.is_green_hydraulic_power_avail = f.greenAvail;
  l.left_sidestick_disabled = s_.leftStickDisabled;
  l.right_sidestick_disabled = s_.rightStickDisabled;
  l.left_sidestick_priority_locked = f.leftStickLocked;
  l.right_sidestick_priority_locked = f.rightStickLocked;
  l.total_sidestick_pitch_command = f.pitchStick;
  l.total_sidestick_roll_command = f.rollStick;
  l.ground_spoilers_armed = f.groundSpoilersArmed;
  l.ground_spoilers_out = s_.groundSpoilersOut;
  l.partial_lift_dumping_active = f.partialLiftDumping;
  l.speed_brake_inhibited = f.speedBrakeInhibited;
  l.single_adr_failure = f.singleAdrFault;
  l.double_adr_failure = f.doubleAdrFault;
  l.cas_or_mach_disagree = false;
  l.alpha_disagree = false;
  l.single_ir_failure = f.singleIrFault;
  l.double_ir_failure = f.doubleIrFault;
  l.ir_disagree = false;
  l.adr_computation_data.V_ias_kn = f.vIas;
  l.adr_computation_data.V_tas_kn = f.vTas;
  l.adr_computation_data.mach = f.mach;
  l.adr_computation_data.alpha_deg = f.alpha;
  l.ir_computation_data.theta_deg = f.theta;
  l.ir_computation_data.phi_deg = f.phi;
  l.ir_computation_data.q_deg_s = f.q;
  l.ir_computation_data.r_deg_s = f.r;
  l.ir_computation_data.n_x_g = f.nx;
  l.ir_computation_data.n_y_g = f.ny;
  l.ir_computation_data.n_z_g = f.nz;
  l.ir_computation_data.theta_dot_deg_s = f.thetaDot;
  l.ir_computation_data.phi_dot_deg_s = f.phiDot;
  l.any_landing_gear_not_uplocked = f.gearNotUplocked;
  l.lgciu_uplock_disagree_or_fault = f.lgciuFault;

  base_sec_discrete_outputs& dis = o.discrete_outputs;
  dis.thr_reverse_selected = false;
  dis.left_elevator_ok = f.leftElevAvail;
  dis.right_elevator_ok = f.rightElevAvail;
  dis.ground_spoiler_out = s_.groundSpoilersOut;
  dis.sec_failed = false;
  dis.left_elevator_damping_mode = f.isEngagedInPitch && f.leftElevAvail;
  dis.right_elevator_damping_mode = f.isEngagedInPitch && f.rightElevAvail;
  const bool thsActive = f.thsActiveCommanded && f.thsAvail;
  dis.ths_active = thsActive;
  dis.batt_power_supply = f.battPowerSupply;

  base_sec_analog_outputs& an = o.analog_outputs;
  const bool elevatorDriven = f.isEngagedInPitch || f.useElacElevatorCommand;
  an.left_elev_pos_order_deg = elevatorDriven && f.leftElevAvail ? f.elevatorOrder : 0.0;
  an.right_elev_pos_order_deg = elevatorDriven && f.rightElevAvail ? f.elevatorOrder : 0.0;
  an.ths_pos_order_deg = thsActive ? f.thsCommand : 0.0;
  an.left_spoiler_1_pos_order_deg = f.spoilerPair1Avail ? f.leftSpoiler1 : 0.0;
  an.right_spoiler_1_pos_order_deg = f.spoilerPair1Avail ? f.rightSpoiler1 : 0.0;
  an.left_spoiler_2_pos_order_deg = f.spoilerPair2Avail ? f.leftSpoiler2 : 0.0;
  an.right_spoiler_2_pos_order_deg = f.spoilerPair2Avail ? f.rightSpoiler2 : 0.0;

  position(bus.left_spoiler_1_position_deg, d.l_spoiler_1_servo_failed, a.left_spoiler_1_pos_deg);
  position(bus.right_spoiler_1_position_deg, d.r_spoiler_1_servo_failed, a.right_spoiler_1_pos_deg);
  position(bus.left_spoiler_2_position_deg, d.l_spoiler_2_servo_failed, a.left_spoiler_2_pos_deg);
  position(bus.right_spoiler_2_position_deg, d.r_spoiler_2_servo_failed, a.right_spoiler_2_pos_deg);
  position(bus.left_elevator_position_deg, d.l_elev_servo_failed, a.left_elevator_pos_deg);
  position(bus.right_elevator_position_deg, d.r_elev_servo_failed, a.right_elevator_pos_deg);
  position(bus.ths_position_deg, d.ths_motor_fault, a.ths_pos_deg);
  bus.left_sidestick_pitch_command_deg = {kNo, 16.0F * static_cast<float>(a.capt_pitch_stick_pos)};
  bus.right_sidestick_pitch_command_deg = {kNo, 16.0F * static_cast<float>(a.fo_pitch_stick_pos)};
  bus.left_sidestick_roll_command_deg = {kNo, 20.0F * static_cast<float>(a.capt_roll_stick_pos)};
  bus.right_sidestick_roll_command_deg = {kNo, 20.0F * static_cast<float>(a.fo_roll_stick_pos)};
  bus.speed_brake_lever_command_deg = {kNo, 30.0F * static_cast<float>(a.spd_brk_lever_pos)};
  bus.speed_brake_command_deg = {kNo, static_cast<float>(f.speedBrakeCommand)};
  bus.thrust_lever_angle_1_deg = {kNo, static_cast<float>(a.thr_lever_1_pos)};
  bus.thrust_lever_angle_2_deg = {kNo, static_cast<float>(a.thr_lever_2_pos)};
  bus.discrete_status_word_1.SSM = kNo;
  bus.discrete_status_word_2.SSM = kNo;
}

}  // namespace sec::readable
