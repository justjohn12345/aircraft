// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Inputs and outputs of the readable pitch laws (PitchNormalLaw, PitchAlternateLaw, PitchDirectLaw). Field names are
// the generated step() argument names without the rtu_In_ / rty_Out_ prefix, so a call site maps one to one.

#pragma once

namespace efcs::readable {

/** PitchNormalLaw::step() inputs (PitchNormalLaw.h 590). */
struct PitchNormalLawInputs {
  double time_dt = 0;
  double nz_g = 0;
  double Theta_deg = 0;
  double Phi_deg = 0;
  double qk_deg_s = 0;
  double qk_dot_deg_s2 = 0;
  double eta_deg = 0;
  double eta_trim_deg = 0;
  double alpha_deg = 0;
  double V_ias_kn = 0;
  double V_tas_kn = 0;
  double H_radio_ft = 0;
  double flaps_handle_index = 0;
  double spoilers_left_pos = 0;
  double spoilers_right_pos = 0;
  double thrust_lever_1_pos = 0;
  double thrust_lever_2_pos = 0;
  bool tailstrike_protection_on = false;
  double VLS_kn = 0;
  double delta_eta_pos = 0;
  bool on_ground = false;
  double in_flight = 0;
  bool tracking_mode_on = false;
  bool high_aoa_prot_active = false;
  bool high_speed_prot_active = false;
  double alpha_prot = 0;
  double alpha_max = 0;
  double high_speed_prot_high_kn = 0;
  double high_speed_prot_low_kn = 0;
  double ap_theta_c_deg = 0;
  bool any_ap_engaged = false;
};

/** PitchAlternateLaw::step() inputs (PitchAlternateLaw.h 204). */
struct PitchAlternateLawInputs {
  double time_dt = 0;
  double nz_g = 0;
  double Theta_deg = 0;
  double Phi_deg = 0;
  double qk_deg_s = 0;
  double eta_deg = 0;
  double eta_trim_deg = 0;
  double V_ias_kn = 0;
  double mach = 0;
  double V_tas_kn = 0;
  double flaps_handle_index = 0;
  double spoilers_left_pos = 0;
  double spoilers_right_pos = 0;
  double delta_eta_pos = 0;
  double in_flight = 0;
  bool tracking_mode_on = false;
  bool stabilities_available = false;
};

/** PitchDirectLaw::step() inputs (PitchDirectLaw.h 31). */
struct PitchDirectLawInputs {
  double time_dt = 0;
  double delta_eta_pos = 0;
};

/** Outputs of every pitch law. */
struct PitchLawOutputs {
  double eta_deg = 0;
  double eta_trim_dot_deg_s = 0;
  double eta_trim_limit_lo = 0;
  double eta_trim_limit_up = 0;
};

}  // namespace efcs::readable
