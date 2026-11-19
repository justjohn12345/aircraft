// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Inputs and outputs of the readable lateral laws (LateralNormalLaw, LateralDirectLaw). Field names are the generated
// step() argument names without the rtu_In_ / rty_Out_ prefix.

#pragma once

namespace efcs::readable {

/** LateralNormalLaw::step() inputs (LateralNormalLaw.h 106). */
struct LateralNormalLawInputs {
  double time_dt = 0;
  double Theta_deg = 0;
  double Phi_deg = 0;
  double r_deg_s = 0;
  double pk_deg_s = 0;
  double V_ias_kn = 0;
  double V_tas_kn = 0;
  double H_radio_ft = 0;
  double delta_xi_pos = 0;
  double delta_zeta_pos = 0;
  bool on_ground = false;
  bool tracking_mode_on = false;
  bool high_aoa_prot_active = false;
  bool high_speed_prot_active = false;
  double ap_phi_c_deg = 0;
  double ap_beta_c_deg = 0;
  bool any_ap_engaged = false;
};

/** LateralDirectLaw::step() inputs (LateralDirectLaw.h 27). */
struct LateralDirectLawInputs {
  double time_dt = 0;
  double delta_xi_pos = 0;
};

/** Outputs of both lateral laws. */
struct LateralLawOutputs {
  double xi_deg = 0;
  double zeta_deg = 0;
};

}  // namespace efcs::readable
