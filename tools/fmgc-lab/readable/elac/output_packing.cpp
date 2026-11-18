// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ELAC outputs (generated ElacComputer.cpp 1621-1908): discrete status words, the bus, the discrete and analog
// outputs, the logic and law copies.
//
// ELAC bus (SSM: NO NormalOperation, NCD NoComputedData; bits 1-based):
//   left / right aileron, elevator, THS position   NO with the measured position, NCD 0 with that servo failed
//   left / right sidestick pitch / roll command    NO: 16 deg x pitch, 20 deg x roll stick position
//   rudder pedal position                          NO: 30 deg x pedal position
//   aileron command                                NO when engaged in roll with an aileron lost: that aileron's order
//                                                  (for the other ELAC to cross-command), else NCD 0
//   roll spoiler command                           NO when engaged in roll, else NCD 0
//   yaw damper command                             NO in roll normal law, else NCD 0
//   elevator double pressurization command        NO with the elevator order during double pressurisation, else NCD 0
//   speedbrake extension                           SEC 1's speed brake command word, copied
//   discrete status word 1 (NO)                    11-14 L/R aileron, L/R elevator servo failed; 15-18 L/R aileron,
//                                                  L/R elevator available; 19 engaged in pitch; 20 in roll; 21 cannot
//                                                  engage in pitch; 22 in roll; 23 pitch normal or ALT2 (L4); 24 ALT1 or
//                                                  ALT2; 25 direct; 26 lateral normal; 27 lateral direct; 28 lateral
//                                                  normal, abnormal attitude or SEC 1 discrete word 1 bit 16 clear (L7);
//                                                  29 lateral normal or abnormal attitude
//   discrete status word 2 (NO)                    11 pitch capability normal or direct; 12 an alternate or direct; 13 /
//                                                  14 lateral capability normal / direct; 17 / 18 left / right sidestick
//                                                  disabled; 19 / 20 priority locked; 21 aileron droop; 22 an AP
//                                                  discrete engaged; 23 alpha protection

#include "elac.h"

namespace elac::readable {

namespace {

constexpr uint32_t kNcd = static_cast<uint32_t>(SignStatusMatrix::NoComputedData);
constexpr uint32_t kNo = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);

void position(base_arinc_429& w, bool failed, double value) {
  w.SSM = failed ? kNcd : kNo;
  w.Data = failed ? 0.0F : static_cast<float>(value);
}

}  // namespace

void ElacComputer::packOutputs(const Frame& f) {
  const elac_inputs& in = *f.in;
  const base_elac_discrete_inputs& d = in.discrete_inputs;
  elac_outputs& o = out_;
  base_elac_out_bus& bus = o.bus_outputs;
  const bool lateralNormal = f.activeLateralLaw == lateral_efcs_law::NormalLaw;

  bus.elevator_double_pressurization_command_deg.SSM = f.doublePressurization ? kNo : kNcd;
  bus.elevator_double_pressurization_command_deg.Data =
      f.doublePressurization ? static_cast<float>(f.elevatorCommand) : 0.0F;
  bus.yaw_damper_command_deg.SSM = lateralNormal ? kNo : kNcd;
  bus.yaw_damper_command_deg.Data = lateralNormal ? static_cast<float>(f.yawDamperCommand) : 0.0F;

  // discrete status word 1 (1641-1666)
  const pitch_efcs_law pitch = f.activePitchLaw;
  const base_arinc_429& sec1 = in.bus_inputs.sec_1_bus.discrete_status_word_1;
  const bool lateralNormalOrAbnormal = lateralNormal || f.abnormalCondition;
  const bool word1[19] = {
      d.l_ail_servo_failed,
      d.r_ail_servo_failed,
      d.l_elev_servo_failed,
      d.r_elev_servo_failed,
      f.leftAileronAvail,
      f.rightAileronAvail,
      f.leftElevAvail,
      f.rightElevAvail,
      f.isEngagedInPitch,
      f.isEngagedInRoll,
      !f.canEngageInPitch,
      !f.canEngageInRoll,
      pitch == pitch_efcs_law::NormalLaw || pitch == pitch_efcs_law::AlternateLaw2,
      pitch == pitch_efcs_law::AlternateLaw1 || pitch == pitch_efcs_law::AlternateLaw2,
      pitch == pitch_efcs_law::DirectLaw,
      lateralNormal,
      f.activeLateralLaw == lateral_efcs_law::DirectLaw,
      lateralNormalOrAbnormal || (blocks::a429::bit(sec1, 16) == 0 && blocks::a429::isNormal(sec1)),
      lateralNormalOrAbnormal,
  };
  bus.discrete_status_word_1.Data = blocks::packBits(word1);

  // discrete status word 2 (1667-1687)
  const pitch_efcs_law cap = f.pitchLawCapability;
  const bool word2[19] = {
      cap == pitch_efcs_law::NormalLaw || cap == pitch_efcs_law::DirectLaw,
      cap == pitch_efcs_law::AlternateLaw1 || cap == pitch_efcs_law::AlternateLaw2 || cap == pitch_efcs_law::DirectLaw,
      f.lateralLawCapability == lateral_efcs_law::NormalLaw,
      f.lateralLawCapability == lateral_efcs_law::DirectLaw,
      false,
      false,
      engagement_.leftStickDisabled,
      engagement_.rightStickDisabled,
      f.leftStickLocked,
      f.rightStickLocked,
      f.aileronDroop,
      !d.ap_1_disengaged || !d.ap_2_disengaged,
      protections_.alphaProt,
      false,
      false,
      false,
      false,
      false,
      false,
  };
  bus.discrete_status_word_2.Data = blocks::packBits(word2);

  // data copy, law outputs and logic (1688-1760)
  o.data = in;
  o.laws.lateral_law_outputs.left_aileron_command_deg = f.leftAileronCommand;
  o.laws.lateral_law_outputs.right_aileron_command_deg = f.rightAileronCommand;
  o.laws.lateral_law_outputs.roll_spoiler_command_deg = f.rollSpoilerCommand;
  o.laws.lateral_law_outputs.yaw_damper_command_deg = f.yawDamperCommand;
  o.laws.pitch_law_outputs.elevator_command_deg = f.elevatorCommand;
  o.laws.pitch_law_outputs.ths_command_deg = f.thsCommand;
  o.laws.pitch_law_outputs.elevator_double_pressurization_active = f.doublePressurization;

  base_elac_logic_outputs& l = o.logic;
  l.on_ground = f.onGround;
  l.pitch_law_in_flight = f.inFlight;
  l.tracking_mode_on = f.trackingMode;
  l.lateral_law_capability = f.lateralLawCapability;
  l.active_lateral_law = f.activeLateralLaw;
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
  l.can_engage_in_roll = f.canEngageInRoll;
  l.has_priority_in_roll = f.hasPriorityInRoll;
  l.left_aileron_crosscommand_active = f.leftAileronCrossCommand;
  l.right_aileron_crosscommand_active = f.rightAileronCrossCommand;
  l.left_aileron_avail = f.leftAileronAvail;
  l.right_aileron_avail = f.rightAileronAvail;
  l.aileron_droop_active = f.aileronDroop;
  l.aileron_antidroop_active = f.aileronAntiDroop;
  l.is_yellow_hydraulic_power_avail = f.yellowAvail;
  l.is_blue_hydraulic_power_avail = f.blueAvail;
  l.is_green_hydraulic_power_avail = f.greenAvail;
  l.left_sidestick_disabled = engagement_.leftStickDisabled;
  l.right_sidestick_disabled = engagement_.rightStickDisabled;
  l.left_sidestick_priority_locked = f.leftStickLocked;
  l.right_sidestick_priority_locked = f.rightStickLocked;
  l.total_sidestick_pitch_command = f.pitchStick;
  l.total_sidestick_roll_command = f.rollStick;
  l.ap_authorised = f.apAuthorised;
  l.ap_1_control = f.ap1Control;
  l.ap_2_control = f.ap2Control;
  l.protection_ap_disconnect = f.apDisconnect;
  l.high_alpha_prot_active = protections_.alphaProt;
  l.alpha_prot_deg = f.alphaProt;
  l.alpha_max_deg = f.alphaMax;
  l.high_speed_prot_active = protections_.highSpeedProt;
  l.high_speed_prot_lo_thresh_kn = f.highSpeedProtLow;
  l.high_speed_prot_hi_thresh_kn = f.highSpeedProtHigh;
  l.double_adr_failure = f.doubleAdrFault;
  l.triple_adr_failure = f.tripleAdrFault;
  l.cas_or_mach_disagree = false;
  l.alpha_disagree = false;
  l.double_ir_failure = f.doubleIrFault;
  l.triple_ir_failure = f.tripleIrFault;
  l.ir_failure_not_self_detected = false;
  l.adr_computation_data.V_ias_kn = f.vIas;
  l.adr_computation_data.V_tas_kn = f.vTas;
  l.adr_computation_data.mach = f.mach;
  l.adr_computation_data.alpha_deg = f.alphaFiltered;
  l.ir_computation_data.theta_deg = f.theta;
  l.ir_computation_data.phi_deg = f.phi;
  l.ir_computation_data.q_deg_s = f.q;
  l.ir_computation_data.r_deg_s = f.r;
  l.ir_computation_data.n_x_g = f.nx;
  l.ir_computation_data.n_y_g = f.ny;
  l.ir_computation_data.n_z_g = f.nz;
  l.ir_computation_data.theta_dot_deg_s = f.thetaDot;
  l.ir_computation_data.phi_dot_deg_s = f.phiDot;
  l.ra_computation_data_ft = f.radioHeight;
  l.dual_ra_failure = f.dualRaFailure;

  // discrete and analog outputs (1761-1792)
  base_elac_discrete_outputs& dis = o.discrete_outputs;
  dis.pitch_axis_ok = f.canEngageInPitch;
  dis.left_aileron_ok = f.leftAileronAvail;
  dis.right_aileron_ok = f.rightAileronAvail;
  dis.digital_output_validated = true;
  dis.ap_1_authorised = f.apAuthorised;
  dis.ap_2_authorised = f.apAuthorised;
  const bool leftAileronActive = (f.isEngagedInRoll || f.leftAileronCrossCommand) && f.leftAileronAvail;
  const bool rightAileronActive = (f.isEngagedInRoll || f.rightAileronCrossCommand) && f.rightAileronAvail;
  dis.left_aileron_active_mode = leftAileronActive;
  dis.right_aileron_active_mode = rightAileronActive;
  dis.left_elevator_damping_mode = f.isEngagedInPitch && f.leftElevAvail && !f.leftDampingInhibited;
  dis.right_elevator_damping_mode = f.isEngagedInPitch && f.rightElevAvail && !f.rightDampingInhibited;
  const bool thsActive = f.thsActiveCommanded && f.thsAvail;
  dis.ths_active = thsActive;
  dis.batt_power_supply = f.battPowerSupply;

  base_elac_analog_outputs& a = o.analog_outputs;
  const bool elevatorDriven = f.useOppElevatorCommand || f.isEngagedInPitch;
  a.right_elev_pos_order_deg = elevatorDriven && f.rightElevAvail ? f.elevatorOrder : 0.0;
  a.left_elev_pos_order_deg = elevatorDriven && f.leftElevAvail ? f.elevatorOrder : 0.0;
  a.ths_pos_order = thsActive ? f.thsCommand : 0.0;
  a.left_aileron_pos_order = leftAileronActive ? f.leftAileronCommand : 0.0;
  a.right_aileron_pos_order = rightAileronActive ? f.rightAileronCommand : 0.0;

  // bus (1794-1908)
  position(bus.left_aileron_position_deg, d.l_ail_servo_failed, in.analog_inputs.left_aileron_pos_deg);
  position(bus.right_aileron_position_deg, d.r_ail_servo_failed, in.analog_inputs.right_aileron_pos_deg);
  position(bus.left_elevator_position_deg, d.l_elev_servo_failed, in.analog_inputs.left_elevator_pos_deg);
  position(bus.right_elevator_position_deg, d.r_elev_servo_failed, in.analog_inputs.right_elevator_pos_deg);
  position(bus.ths_position_deg, d.ths_motor_fault, in.analog_inputs.ths_pos_deg);
  bus.left_sidestick_pitch_command_deg = {kNo, 16.0F * static_cast<float>(in.analog_inputs.capt_pitch_stick_pos)};
  bus.right_sidestick_pitch_command_deg = {kNo, 16.0F * static_cast<float>(in.analog_inputs.fo_pitch_stick_pos)};
  bus.left_sidestick_roll_command_deg = {kNo, 20.0F * static_cast<float>(in.analog_inputs.capt_roll_stick_pos)};
  bus.right_sidestick_roll_command_deg = {kNo, 20.0F * static_cast<float>(in.analog_inputs.fo_roll_stick_pos)};
  bus.rudder_pedal_position_deg = {kNo, 30.0F * static_cast<float>(in.analog_inputs.rudder_pedal_pos)};
  if ((!f.leftAileronAvail || !f.rightAileronAvail) && f.isEngagedInRoll) {
    bus.aileron_command_deg = {kNo, static_cast<float>(!f.leftAileronAvail ? f.leftAileronCommand
                                                                            : f.rightAileronCommand)};
  } else {
    bus.aileron_command_deg = {kNcd, 0.0F};
  }
  bus.roll_spoiler_command_deg.SSM = f.isEngagedInRoll ? kNo : kNcd;
  bus.roll_spoiler_command_deg.Data = f.isEngagedInRoll ? static_cast<float>(f.rollSpoilerCommand) : 0.0F;
  bus.speedbrake_extension_deg = in.bus_inputs.sec_1_bus.speed_brake_command_deg;
  bus.discrete_status_word_1.SSM = kNo;
  bus.discrete_status_word_2.SSM = kNo;
}

}  // namespace elac::readable
