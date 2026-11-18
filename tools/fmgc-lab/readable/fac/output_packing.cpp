// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's outputs (generated FacComputer.cpp 1223-1586). See output_packing.h.

#include "output_packing.h"

namespace fac::readable {

namespace {

constexpr uint32_t kNormal = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
constexpr uint32_t kNcd = static_cast<uint32_t>(SignStatusMatrix::NoComputedData);

base_arinc_429 word(uint32_t ssm, double value) { return {ssm, static_cast<real32_T>(value)}; }
uint32_t normalIf(bool c) { return c ? kNormal : kNcd; }

bool twoOfThree(bool a, bool b, bool c) { return (a && b) || (a && c) || (b && c); }

/** Discrete word 2: which rudder functions this FAC and the opposite one have engaged. */
float discreteWord2(const YawEngagement& e, const base_fac_discrete_inputs& d) {
  bool u[19] = {};
  u[0] = e.yawDamperEngaged;
  u[1] = d.yaw_damper_opp_engaged;
  u[2] = e.rudderTrimEngaged;
  u[3] = d.rudder_trim_opp_engaged;
  u[4] = e.travelLimEngaged;
  u[5] = d.rudder_travel_lim_opp_engaged;
  return blocks::packBits(u);
}

/** Discrete word 3: the low energy (bit 11) and pitch-pitch (bit 15) warnings. */
float discreteWord3(const FlightPathOutputs& f) {
  bool u[19] = {};
  u[0] = f.lowEnergy;
  u[4] = f.pitchPitch;
  return blocks::packBits(u);
}

/** Discrete word 5: configuration, landing gear and slat/flap source status, alpha floor. */
float discreteWord5(const Sensors& s, bool alphaFloor) {
  bool u[19] = {};
  for (int k = 0; k < 5; k++) u[k] = s.slatFlap.configIndex == static_cast<float>(k + 1);
  u[5] = s.gear.lgciuOwnValid;
  u[6] = s.gear.allLgciuLost;
  u[7] = s.gear.leftMainPressed;
  u[8] = s.gear.rightMainPressed;
  u[9] = s.gear.mainGearOut;
  u[10] = s.slatFlap.sfccOwnValid;
  u[11] = s.slatFlap.allSfccLost;
  u[18] = alphaFloor;
  return blocks::packBits(u);
}

void packBus(const FrameResults& r, base_fac_bus& bus) {
  const Sensors& s = *r.sensors;
  const SpeedOutputs& v = *r.speeds;
  const FlightPathOutputs& f = *r.flightPath;
  const fac_inputs& in = *r.in;
  const uint32_t sfcc = normalIf(s.slatFlap.sfccOwnValid);
  const bool scale = v.speedScaleVisible;

  bus.discrete_word_1 = {sfcc, s.slatFlap.actualPositionWord};
  bus.gamma_a_deg = word(kNormal, f.gammaADeg);
  bus.gamma_t_deg = word(kNormal, f.gammaTDeg);
  bus.total_weight_lbs = {kNormal, 0.0F};
  bus.center_of_gravity_pos_percent = {kNormal, 0.0F};
  bus.sideslip_target_deg = word(kNcd, f.betaTargetDeg);
  bus.fac_slat_angle_deg = {sfcc, s.slatFlap.slatFppuDeg};
  bus.fac_flap_angle_deg = {sfcc, s.slatFlap.flapFppuDeg};
  bus.discrete_word_2 = {kNormal, discreteWord2(r.engagement, in.discrete_inputs)};
  bus.rudder_travel_limit_command_deg = word(kNormal, in.analog_inputs.rudder_travel_lim_position_deg);
  bus.delta_r_yaw_damper_deg = {kNormal, 0.0F};
  bus.estimated_sideslip_deg = word(kNormal, f.estimatedBetaDeg);
  bus.v_alpha_lim_kn = word(normalIf(scale), v.vAlphaMax);
  bus.v_ls_kn = word(normalIf(scale), v.vls);
  bus.v_stall_kn = word(kNormal, v.vStall);
  bus.v_alpha_prot_kn = word(normalIf(scale), v.vAlphaProt);
  bus.v_stall_warn_kn = word(normalIf(scale), v.vStallWarn);
  bus.speed_trend_kn = word(kNormal, v.trend);
  bus.v_3_kn = word(normalIf(scale && v.v3Visible), v.v3);
  bus.v_4_kn = word(normalIf(scale && v.v4Visible), v.v4);
  bus.v_man_kn = word(normalIf(scale && v.vManVisible), v.vMan);
  bus.v_max_kn = word(kNormal, v.vMax);
  bus.v_fe_next_kn = word(normalIf(scale && v.vFeNextVisible), v.vFeNext);
  bus.discrete_word_3 = {kNormal, discreteWord3(f)};
  bus.discrete_word_4 = {kNormal, 0.0F};
  bus.discrete_word_5 = {kNormal, discreteWord5(s, r.alphaFloor)};
  bus.delta_r_rudder_trim_deg = word(kNormal, r.rudderTrimOrder);
  bus.rudder_trim_pos_deg = word(kNormal, in.analog_inputs.rudder_trim_position_deg);
}

void fillLogic(const FrameResults& r, base_fac_logic_outputs& l) {
  const Sensors& s = *r.sensors;
  const fac_inputs& in = *r.in;
  const YawEngagement& e = r.engagement;
  l.lgciu_own_valid = s.gear.lgciuOwnValid;
  l.all_lgciu_lost = s.gear.allLgciuLost;
  l.left_main_gear_pressed = s.gear.leftMainPressed;
  l.right_main_gear_pressed = s.gear.rightMainPressed;
  l.main_gear_out = s.gear.mainGearOut;
  l.sfcc_own_valid = s.slatFlap.sfccOwnValid;
  l.all_sfcc_lost = s.slatFlap.allSfccLost;
  l.flap_handle_index = s.slatFlap.configIndex;
  l.flap_angle_deg = s.slatFlap.flapFppuDeg;
  l.slat_angle_deg = s.slatFlap.slatFppuDeg;
  l.flap_surface_angle_deg = s.slatFlap.flapSurfaceDeg;
  l.slat_surface_angle_deg = s.slatFlap.slatSurfaceDeg;
  l.slat_flap_actual_pos = s.slatFlap.actualPositionWord;
  l.on_ground = s.onGround;
  l.tracking_mode_on = in.sim_data.slew_on || in.sim_data.pause_on || in.sim_data.tracking_mode_on_override;
  l.double_self_detected_adr_failure = twoOfThree(s.adr.ownInvalid, s.adr.oppInvalid, s.adr.adr3Invalid);
  l.double_self_detected_ir_failure = twoOfThree(s.ir.ownInvalid, s.ir.oppInvalid, s.ir.ir3Invalid);
  l.double_not_self_detected_adr_failure = false;  // constant
  l.double_not_self_detected_ir_failure = false;   // constant
  l.adr_computation_data = {s.adr.casKn, s.adr.tasKn, s.adr.mach, s.adr.alphaDeg, s.adr.staticPressureHpa,
                            s.adr.altitudeFt};  // "altitude_corrected" is the standard altitude
  l.ir_computation_data = {s.ir.thetaDeg, s.ir.phiDeg,       s.ir.qDegS,      s.ir.rDegS,
                           s.ir.nxG,      s.ir.nyG,          s.ir.nzG,        s.ir.thetaDotDegS,
                           s.ir.phiDotDegS, s.ir.groundSpeedKn, s.ir.verticalSpeed};
  l.all_ra_failed = s.allRaFailed;
  l.ra_height = s.radioHeight;
  l.yaw_damper_engaged = e.yawDamperEngaged;
  l.yaw_damper_can_engage = e.yawDamperCanEngage;
  l.yaw_damper_has_priority = e.yawDamperHasPriority;
  l.rudder_trim_engaged = e.rudderTrimEngaged;
  l.rudder_trim_can_engage = e.rudderTrimCanEngage;
  l.rudder_trim_has_priority = e.rudderTrimHasPriority;
  l.rudder_travel_lim_engaged = e.travelLimEngaged;
  l.rudder_travel_lim_can_engage = e.travelLimCanEngage;
  l.rudder_travel_lim_has_priority = e.travelLimHasPriority;
  l.speed_scale_lost = false;  // constant
  l.speed_scale_visible = r.speeds->speedScaleVisible;
  l.any_ap_engaged = s.anyApEngaged;
  l.fmgc_own_selected = s.fmgcOwnSelected;
}

void fillFlightEnvelope(const FrameResults& r, base_fac_flight_envelope_outputs& e) {
  const SpeedOutputs& v = *r.speeds;
  const FlightPathOutputs& f = *r.flightPath;
  e.estimated_beta_deg = f.estimatedBetaDeg;
  e.beta_target_deg = f.betaTargetDeg;
  e.beta_target_visible = false;  // constant
  e.alpha_floor_condition = r.alphaFloor;
  e.alpha_filtered_deg = r.alphaFilteredDeg;
  e.computed_weight_lbs = v.weightLbs;
  e.computed_cg_percent = r.in->bus_inputs.fmgc_own_bus.fm_cg_percent.Data;
  e.v_alpha_max_kn = v.vAlphaMax;
  e.v_alpha_prot_kn = v.vAlphaProt;
  e.v_stall_warn_kn = v.vStallWarn;
  e.v_ls_kn = v.vls;
  e.v_stall_kn = v.vStall;
  e.v_3_kn = v.v3;
  e.v_3_visible = v.v3Visible;
  e.v_4_kn = v.v4;
  e.v_4_visible = v.v4Visible;
  e.v_man_kn = v.vMan;
  e.v_man_visible = v.vManVisible;
  e.v_max_kn = v.vMax;
  e.v_fe_next_kn = v.vFeNext;
  e.v_fe_next_visible = v.vFeNextVisible;
  e.v_c_trend_kn = v.trend;
  e.gamma_a_deg = f.gammaADeg;
  e.gamma_t_deg = f.gammaTDeg;
  e.pitch_pitch_warning_active = f.pitchPitch;
  e.low_energy_warning_active = f.lowEnergy;
}

}  // namespace

void assembleOutputs(const FrameResults& r, fac_outputs& out) {
  const YawEngagement& e = r.engagement;
  out.data = *r.in;
  out.laws = {r.yawDamperOrder, r.rudderTrimOrder, r.travelLimitOrder};
  fillLogic(r, out.logic);
  fillFlightEnvelope(r, out.flight_envelope);
  out.discrete_outputs.fac_healthy = true;  // constant: the wrapper decides the health
  out.discrete_outputs.yaw_damper_engaged = e.yawDamperEngaged;
  out.discrete_outputs.rudder_trim_engaged = e.rudderTrimEngaged;
  out.discrete_outputs.rudder_travel_lim_engaged = e.travelLimEngaged;
  out.discrete_outputs.rudder_travel_lim_emergency_reset = false;  // constant
  out.discrete_outputs.yaw_damper_avail_for_norm_law = e.yawDamperCanEngage;
  // orders to the actuators: 0 for a function this FAC has not engaged
  out.analog_outputs.yaw_damper_order_deg = e.yawDamperEngaged ? r.yawDamperOrder : 0.0;
  out.analog_outputs.rudder_trim_order_deg = e.rudderTrimEngaged ? r.rudderTrimOrder : 0.0;
  out.analog_outputs.rudder_travel_limit_order_deg = e.travelLimEngaged ? r.travelLimitOrder : 0.0;
  packBus(r, out.bus_outputs);
}

}  // namespace fac::readable
