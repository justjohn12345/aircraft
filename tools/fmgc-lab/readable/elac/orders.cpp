// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ELAC orders (generated ElacComputer.cpp 1239-1620): AP orders, the lateral laws, aileron droop / anti-droop, aileron
// orders and cross-commands, roll spoiler order; the pitch laws, the elevator order and double pressurisation, the THS
// order (integrator, ground setting).

#include <cmath>

#include "elac.h"

namespace elac::readable {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::isNotFailed;
namespace R = efcs::readable;

/** SFCC actual position word bit n set and the word not failed. */
bool sfccBit(const base_arinc_429& w, int n) {
  return bit(w, n) != 0 && isNotFailed(w);
}

// double pressurisation of the elevators when the order exceeds this (deg) against CAS (kt)
constexpr double kDoublePressBp[7] = {0.0, 180.0, 220.0, 280.0, 350.0, 400.0, 450.0};
constexpr double kDoublePressDeg[7] = {30.0, 30.0, 20.0, 12.4, 8.0, 6.0, 6.0};

}  // namespace

void ElacComputer::updateLateralOrders(Frame& f) {
  const elac_inputs& in = *f.in;
  const base_elac_bus_inputs& b = in.bus_inputs;
  const double dt = in.time.dt;

  // AP in control (1239-1246): its discrete engaged and its roll and pitch orders NormalOperation
  f.ap1Control = !in.discrete_inputs.ap_1_disengaged && isNormal(b.fmgc_1_bus.delta_p_ail_cmd_deg) &&
                 isNormal(b.fmgc_1_bus.delta_q_cmd_deg);
  f.ap2Control = !in.discrete_inputs.ap_2_disengaged && isNormal(b.fmgc_2_bus.delta_p_ail_cmd_deg) &&
                 isNormal(b.fmgc_2_bus.delta_q_cmd_deg);

  // aileron droop with the flaps extended (SFCC actual position word bit 19 clear) and the ailerons available
  // (1247-1257); anti-droop with both ground spoiler discretes, flaps extended, pitch < 2.5 deg, no AP, roll normal law
  // (1258-1259)
  const bool flapsExtended = (bit(b.sfcc_1_bus.slat_flap_actual_position_word, 19) == 0 &&
                              isNotFailed(b.sfcc_1_bus.slat_flap_actual_position_word)) ||
                             (bit(b.sfcc_2_bus.slat_flap_actual_position_word, 19) == 0 &&
                              isNotFailed(b.sfcc_2_bus.slat_flap_actual_position_word));
  const base_elac_discrete_inputs& d = in.discrete_inputs;
  f.aileronDroop = flapsExtended && ((f.leftAileronAvail && f.rightAileronAvail) ||
                                     (!d.opp_left_aileron_lost && f.rightAileronAvail) ||
                                     (f.leftAileronAvail && !d.opp_right_aileron_lost));
  f.aileronAntiDroop = d.ground_spoilers_active_1 && d.ground_spoilers_active_2 && flapsExtended && f.theta < 2.5F &&
                       f.noApEngaged && f.activeLateralLaw == lateral_efcs_law::NormalLaw;

  // the laws (1261-1279): AP 1's orders when it is in control, else AP 2's
  const base_fmgc_b_bus& fmgc = f.ap1Control ? b.fmgc_1_bus : b.fmgc_2_bus;
  R::LateralNormalLawInputs ln;
  ln.time_dt = dt;
  ln.Theta_deg = f.theta;
  ln.Phi_deg = f.phi;
  ln.r_deg_s = f.r;
  ln.pk_deg_s = f.phiDot;  // the roll attitude rate
  ln.V_ias_kn = f.vIas;
  ln.V_tas_kn = f.vTas;
  ln.H_radio_ft = f.radioHeight;
  ln.delta_xi_pos = f.rollStick;
  ln.delta_zeta_pos = in.analog_inputs.rudder_pedal_pos;
  ln.on_ground = f.onGround;
  ln.tracking_mode_on = f.trackingMode || f.activeLateralLaw != lateral_efcs_law::NormalLaw;
  ln.high_aoa_prot_active = protections_.alphaProt;
  ln.high_speed_prot_active = protections_.highSpeedProt;
  ln.ap_phi_c_deg = fmgc.delta_p_ail_cmd_deg.Data;
  ln.ap_beta_c_deg = fmgc.delta_r_cmd_deg.Data;
  ln.any_ap_engaged = f.ap1Control || f.ap2Control;
  const R::LateralLawOutputs normal = lateralNormal_.step(ln);
  const R::LateralLawOutputs direct = lateralDirect_.step({dt, f.rollStick});

  double xi;  // (1280-1291)
  switch (f.activeLateralLaw) {
    case lateral_efcs_law::NormalLaw:
      xi = normal.xi_deg;
      break;
    case lateral_efcs_law::DirectLaw:
      xi = direct.xi_deg;
      break;
    default:
      xi = 0.0;
      break;
  }

  // droop +5 deg at 1 deg/s, anti-droop -30 deg at 20 deg/s (1293-1311)
  const double droop = orders_.droop.update(f.aileronDroop ? 5.0 : 0.0, 1.0, -1.0, dt, 0.0);
  const double antiDroop = orders_.antiDroop.update(f.aileronAntiDroop ? -30.0 : 0.0, 20.0, -20.0, dt, 0.0);
  const double offset = antiDroop + droop;

  // aileron orders (1312-1344): +-25 deg, 50 deg/s; the opposite ELAC's order when cross-commanding; while neither
  // engaged in roll nor cross-commanding, they follow the measured positions
  const double left = blocks::limit(f.leftAileronCrossCommand ? static_cast<double>(b.elac_opp_bus.aileron_command_deg.Data)
                                                              : -1.0 * xi + offset,
                                    -25.0, 25.0);
  f.leftAileronCommand = orders_.leftAileron.update(left, 50.0, -50.0, dt, in.analog_inputs.left_aileron_pos_deg,
                                                    !f.leftAileronCrossCommand && !f.isEngagedInRoll);
  const double right = blocks::limit(
      f.rightAileronCrossCommand ? static_cast<double>(b.elac_opp_bus.aileron_command_deg.Data) : xi + offset, -25.0,
      25.0);
  f.rightAileronCommand = orders_.rightAileron.update(right, 50.0, -50.0, dt, in.analog_inputs.right_aileron_pos_deg,
                                                      !f.rightAileronCrossCommand && !f.isEngagedInRoll);

  // roll spoiler order (1345-1371): 1.4 x the roll order; without flaps FULL (SFCC bit 23) a 5 deg dead band and a
  // gain of 1.25, at most 20 deg before the gain
  double spoiler;
  if (sfccBit(b.sfcc_1_bus.slat_flap_actual_position_word, 23) ||
      sfccBit(b.sfcc_2_bus.slat_flap_actual_position_word, 23)) {
    spoiler = xi;
  } else {
    const double magnitude = blocks::limit(std::abs(xi) + -5.0, 0.0, 20.0);
    const int sign = xi < 0.0 ? -1 : (xi > 0.0);
    spoiler = magnitude * static_cast<double>(sign) * 1.25;
  }
  f.rollSpoilerCommand = 1.4 * spoiler;

  switch (f.activeLateralLaw) {  // (1372-1383)
    case lateral_efcs_law::NormalLaw:
      f.yawDamperCommand = normal.zeta_deg;
      break;
    case lateral_efcs_law::DirectLaw:
      f.yawDamperCommand = direct.zeta_deg;
      break;
    default:
      f.yawDamperCommand = 0.0;
      break;
  }
}

void ElacComputer::updatePitchOrders(Frame& f) {
  const elac_inputs& in = *f.in;
  const base_elac_bus_inputs& b = in.bus_inputs;
  const double dt = in.time.dt;

  // pitch acceleration: the derivative of the pitch attitude rate, lag 2 rad/s (1387-1389)
  const double thetaDot = 1.0 * f.thetaDot;
  const double pitchAcceleration =
      orders_.pitchAcceleration.update((thetaDot - orders_.previousThetaDot) / dt, 2.0, dt);
  orders_.previousThetaDot = thetaDot;  // (1911)

  // thrust lever angles from SEC 1, else SEC 2, else 0 (1409-1426)
  double tla1 = 0.0, tla2 = 0.0;
  if (isNormal(b.sec_1_bus.thrust_lever_angle_1_deg) && isNormal(b.sec_1_bus.thrust_lever_angle_2_deg)) {
    tla1 = b.sec_1_bus.thrust_lever_angle_1_deg.Data;
    tla2 = b.sec_1_bus.thrust_lever_angle_2_deg.Data;
  } else if (isNormal(b.sec_2_bus.thrust_lever_angle_1_deg) && isNormal(b.sec_2_bus.thrust_lever_angle_2_deg)) {
    tla1 = b.sec_2_bus.thrust_lever_angle_1_deg.Data;
    tla2 = b.sec_2_bus.thrust_lever_angle_2_deg.Data;
  }
  const double config = flapHandleIndex(b.sfcc_1_bus.slat_flap_system_status_word);
  const base_fmgc_b_bus& fmgc = f.ap1Control ? b.fmgc_1_bus : b.fmgc_2_bus;

  // the three pitch laws, every frame (1435-1480)
  R::PitchNormalLawInputs pn;
  pn.time_dt = dt;
  pn.nz_g = f.nz;
  pn.Theta_deg = f.theta;
  pn.Phi_deg = f.phi;
  pn.qk_deg_s = f.thetaDot;
  pn.qk_dot_deg_s2 = pitchAcceleration;
  pn.eta_deg = in.analog_inputs.left_elevator_pos_deg;
  pn.eta_trim_deg = in.analog_inputs.ths_pos_deg;
  pn.alpha_deg = f.alphaFiltered;
  pn.V_ias_kn = f.vIas;
  pn.V_tas_kn = f.vTas;
  pn.H_radio_ft = f.radioHeight;
  pn.flaps_handle_index = config;
  pn.spoilers_left_pos = 0.0;
  pn.spoilers_right_pos = 0.0;
  pn.thrust_lever_1_pos = tla1;
  pn.thrust_lever_2_pos = tla2;
  pn.tailstrike_protection_on = in.sim_data.tailstrike_protection_on;
  pn.VLS_kn = 0.0;
  pn.delta_eta_pos = f.pitchStick;
  pn.on_ground = f.onGround;
  pn.in_flight = f.inFlight;
  pn.tracking_mode_on = f.trackingMode || f.activePitchLaw != pitch_efcs_law::NormalLaw;
  pn.high_aoa_prot_active = protections_.alphaProt;
  pn.high_speed_prot_active = protections_.highSpeedProt;
  pn.alpha_prot = f.alphaProt;
  pn.alpha_max = f.alphaMax;
  pn.high_speed_prot_high_kn = f.highSpeedProtHigh;
  pn.high_speed_prot_low_kn = f.highSpeedProtLow;
  pn.ap_theta_c_deg = fmgc.delta_q_cmd_deg.Data;
  pn.any_ap_engaged = f.ap1Control || f.ap2Control;
  const R::PitchLawOutputs normal = pitchNormal_.step(pn);

  R::PitchAlternateLawInputs pa;
  pa.time_dt = dt;
  pa.nz_g = f.nz;
  pa.Theta_deg = f.theta;
  pa.Phi_deg = f.phi;
  pa.qk_deg_s = f.thetaDot;
  pa.eta_deg = 0.0;
  pa.eta_trim_deg = in.analog_inputs.ths_pos_deg;
  pa.V_ias_kn = f.vIas;
  pa.mach = f.mach;
  pa.V_tas_kn = f.vTas;
  pa.flaps_handle_index = config;
  pa.spoilers_left_pos = 0.0;
  pa.spoilers_right_pos = 0.0;
  pa.delta_eta_pos = f.pitchStick;
  pa.in_flight = f.inFlight;
  pa.tracking_mode_on = f.trackingMode || (f.activePitchLaw != pitch_efcs_law::AlternateLaw1 &&
                                           f.activePitchLaw != pitch_efcs_law::AlternateLaw2);
  pa.stabilities_available = f.activePitchLaw != pitch_efcs_law::AlternateLaw2;
  const R::PitchLawOutputs alternate = pitchAlternate_.step(pa);
  const R::PitchLawOutputs direct = pitchDirect_.step({dt, f.pitchStick});

  // the active law's outputs (1481-1515, 1527-1543, 1557-1573); no law: 0, THS limits -11..3.5 deg
  const R::PitchLawOutputs* law = nullptr;
  switch (f.activePitchLaw) {
    case pitch_efcs_law::NormalLaw:
      law = &normal;
      break;
    case pitch_efcs_law::AlternateLaw1:
    case pitch_efcs_law::AlternateLaw2:
      law = &alternate;
      break;
    case pitch_efcs_law::DirectLaw:
      law = &direct;
      break;
    default:
      break;
  }
  const double elevator = law ? law->eta_deg : 0.0;
  const double thsUpper = law ? law->eta_trim_limit_up : 3.5;
  const double thsLower = law ? law->eta_trim_limit_lo : -11.0;

  // THS rate (1517-1546): the ground setting drives the THS to 0 (2 x the order, +-0.7 deg/s); a manual override
  // stops it; else the law's trim rate
  double thsRate;
  if (f.thsGroundSetting) {
    thsRate = blocks::limit(-2.0 * orders_.thsOrder, -0.7, 0.7);
  } else if (in.discrete_inputs.ths_override_active) {
    thsRate = 0.0;
  } else {
    thsRate = law ? law->eta_trim_dot_deg_s : 0.0;
  }
  const double increment = 1.0 * thsRate * dt;

  // THS integrator (1548-1578): follows the measured THS position while the THS is not commanded; limited to the law's
  // trim limits
  orders_.reloadThs = !f.thsActiveCommanded || orders_.reloadThs;
  if (orders_.reloadThs) orders_.thsIntegrator = in.analog_inputs.ths_pos_deg - increment;
  orders_.thsOrder = increment + orders_.thsIntegrator;
  if (orders_.thsOrder > thsUpper) {
    orders_.thsOrder = thsUpper;
  } else if (orders_.thsOrder < thsLower) {
    orders_.thsOrder = thsLower;
  }
  orders_.reloadThs = false;               // (1912)
  orders_.thsIntegrator = orders_.thsOrder;  // (1913)
  f.thsCommand = orders_.thsOrder;

  // elevator order and double pressurisation (1580-1600): both actuators of an elevator active when the order exceeds
  // 30 deg (up to 180 kt) .. 6 deg (from 400 kt); not engaged in pitch: the opposite ELAC's double pressurisation order
  f.elevatorCommand = elevator;
  f.doublePressurization =
      blocks::lookup1(static_cast<double>(f.vIas), kDoublePressBp, kDoublePressDeg) < std::abs(elevator) &&
      f.isEngagedInPitch;
  f.useOppElevatorCommand =
      !f.isEngagedInPitch && isNotFailed(b.elac_opp_bus.elevator_double_pressurization_command_deg);
  f.elevatorOrder = f.useOppElevatorCommand
                        ? static_cast<double>(b.elac_opp_bus.elevator_double_pressurization_command_deg.Data)
                        : elevator;

  // damping mode inhibited (both actuators active) on the elevator side the other ELAC or the SEC (SEC 2 for ELAC 1,
  // SEC 1 for ELAC 2) also has available (their discrete word 1 bits 17 / 18), during double pressurisation
  // (1602-1617)
  const base_arinc_429& sec = in.discrete_inputs.is_unit_2 ? b.sec_1_bus.discrete_status_word_1
                                                           : b.sec_2_bus.discrete_status_word_1;
  const base_arinc_429& opp = b.elac_opp_bus.discrete_status_word_1;
  f.leftDampingInhibited = f.doublePressurization && (bit(opp, 17) != 0 || bit(sec, 17) != 0);
  f.rightDampingInhibited = (bit(opp, 18) != 0 || bit(sec, 18) != 0) && f.doublePressurization;

  // battery power supply: a hydraulic system available, and 30 s after the last one is lost (1618-1620)
  f.battPowerSupply = orders_.battPower.update(f.yellowAvail || f.blueAvail || f.greenAvail, dt, false, 30.0);
}

}  // namespace elac::readable
