// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SEC pitch (generated SecComputer.cpp 988-1138): the alternate and direct laws, the elevator order (or an ELAC's
// double pressurisation order), the THS order.

#include <cmath>

#include "sec.h"

namespace sec::readable {

namespace R = efcs::readable;

void SecComputer::updatePitchOrders(Frame& f) {
  const sec_inputs& in = *f.in;
  const base_sec_discrete_inputs& d = in.discrete_inputs;
  const double dt = in.time.dt;

  // both laws every frame (988-1029)
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
  pa.flaps_handle_index = flapHandleIndex(in.bus_inputs.sfcc_1_bus.slat_flap_system_status_word);
  pa.spoilers_left_pos = 0.0;
  pa.spoilers_right_pos = 0.0;
  pa.delta_eta_pos = f.pitchStick;
  pa.in_flight = f.inFlight;
  pa.tracking_mode_on = f.trackingMode || (f.activePitchLaw != pitch_efcs_law::AlternateLaw1 &&
                                           f.activePitchLaw != pitch_efcs_law::AlternateLaw2);
  pa.stabilities_available = f.activePitchLaw != pitch_efcs_law::AlternateLaw2;
  const R::PitchLawOutputs alternate = pitchAlternate_.step(pa);
  const R::PitchLawOutputs direct = pitchDirect_.step({dt, f.pitchStick});

  // the active law's outputs (1030-1056, 1068-1080, 1093-1105); none (or normal, never a SEC capability): 0, THS
  // limits -11..3.5 deg
  const R::PitchLawOutputs* law = nullptr;
  if (f.activePitchLaw == pitch_efcs_law::AlternateLaw1 || f.activePitchLaw == pitch_efcs_law::AlternateLaw2) {
    law = &alternate;
  } else if (f.activePitchLaw == pitch_efcs_law::DirectLaw) {
    law = &direct;
  }
  const double elevator = law ? law->eta_deg : 0.0;
  const double thsUpper = law ? law->eta_trim_limit_up : 3.5;
  const double thsLower = law ? law->eta_trim_limit_lo : -11.0;

  // THS (1058-1110): as the ELAC's
  double thsRate;
  if (f.thsGroundSetting) {
    thsRate = blocks::limit(-2.0 * s_.thsOrder, -0.7, 0.7);
  } else if (d.ths_override_active) {
    thsRate = 0.0;
  } else {
    thsRate = law ? law->eta_trim_dot_deg_s : 0.0;
  }
  const double increment = 1.0 * thsRate * dt;
  s_.reloadThs = !f.thsActiveCommanded || s_.reloadThs;
  if (s_.reloadThs) s_.thsIntegrator = in.analog_inputs.ths_pos_deg - increment;
  s_.thsOrder = increment + s_.thsIntegrator;
  if (s_.thsOrder > thsUpper) {
    s_.thsOrder = thsUpper;
  } else if (s_.thsOrder < thsLower) {
    s_.thsOrder = thsLower;
  }
  s_.reloadThs = false;           // (1404)
  s_.thsIntegrator = s_.thsOrder;  // (1405)
  f.thsCommand = s_.thsOrder;

  // elevator order (1112-1134): not engaged in pitch, SEC 1 takes ELAC 2's double pressurisation order when ELAC 1's
  // pitch is lost, SEC 2 ELAC 1's when ELAC 2's is lost; SEC 3 has none (a constant FailureWarning word)
  f.elevatorCommand = elevator;
  base_arinc_429 elacOrder{0U, 0.0F};
  if (d.is_unit_1) {
    elacOrder = in.bus_inputs.elac_2_bus.elevator_double_pressurization_command_deg;
  } else if (d.is_unit_2) {
    elacOrder = in.bus_inputs.elac_1_bus.elevator_double_pressurization_command_deg;
  }
  const bool elacPitchLost = d.is_unit_1 ? d.pitch_not_avail_elac_1 : d.pitch_not_avail_elac_2;
  f.useElacElevatorCommand = elacPitchLost && !f.isEngagedInPitch && blocks::a429::isNotFailed(elacOrder);
  f.elevatorOrder = f.useElacElevatorCommand ? static_cast<double>(elacOrder.Data) : elevator;

  // battery power supply (1136-1138)
  f.battPowerSupply = s_.battPower.update(f.yellowAvail || f.blueAvail || f.greenAvail, dt, false, 30.0);
}

}  // namespace sec::readable
