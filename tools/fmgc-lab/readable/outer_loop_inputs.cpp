// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "outer_loop_inputs.h"

#include "blocks.h"

namespace fmgc::readable {

namespace {

using blocks::a429::isNormal;
using blocks::a429::rawValue;
using blocks::a429::valueOr;

constexpr double kG = 9.8100000000000005;              // m/s² per g (Gain_Gain_h, Gain1_Gain_n, Gain2_Gain)
constexpr double kOneG = -1;                          // the normal acceleration is sent as an increment (Bias_Bias_p)
constexpr double kKgPerLb = 0.45351473922902491;     // Gain3_Gain
constexpr double kLocMagvarDeg = 0;                   // nav_loc_magvar_deg (Constant_Value_i)
constexpr double kZetaDeg = 0;                        // zeta_deg, rudder position (Constant1_Value_io): not connected

}  // namespace

void assembleOuterLoopInputs(OuterLoopInputState& st, const fmgc_inputs& in, const base_fmgc_logic_outputs& logic,
                             const base_fmgc_ap_fd_logic_outputs& apFd, ap_laws_output& io) {
  const auto& adr = logic.adr_computation_data;
  const auto& ir = logic.ir_computation_data;
  const auto& ils = logic.ils_computation_data;
  const auto& fac = logic.chosen_fac_bus;
  const auto& fcu = in.bus_inputs.fcu_bus;
  const auto& fms = in.fms_inputs;
  const auto& lat = apFd.lateral_modes;
  const auto& lon = apFd.longitudinal_modes;

  io.time.dt = in.time.dt;
  io.time.simulation_time = in.time.simulation_time;

  // Selected sensor data, whatever its SSM (the logic bus holds a zero word when every source failed).
  ap_raw_data& d = io.data;
  d.Theta_deg = ir.pitch_angle_deg.Data;
  d.Phi_deg = ir.roll_angle_deg.Data;
  d.qk_deg_s = ir.pitch_att_rate_deg_s.Data;
  d.rk_deg_s = ir.body_yaw_rate_deg_s.Data;
  d.pk_deg_s = ir.body_roll_rate_deg_s.Data;
  d.V_ias_kn = adr.airspeed_computed_kn.Data;
  d.V_tas_kn = adr.airspeed_true_kn.Data;
  d.V_mach = adr.mach.Data;
  d.V_gnd_kn = ir.ground_speed_kn.Data;
  d.alpha_deg = adr.aoa_corrected_deg.Data;
  d.beta_deg = valueOr(fac.estimated_sideslip_deg, 0.0F);
  d.H_ft = adr.altitude_standard_ft.Data;
  d.H_ind_ft = logic.altitude_indicated_ft.Data;
  d.H_radio_ft = rawValue(logic.ra_computation_data.radio_height_ft);
  d.H_dot_ft_min = ir.inertial_vertical_speed_ft_s.Data;  // fpm despite the name
  d.Psi_magnetic_deg = ir.heading_magnetic_deg.Data;
  d.Psi_magnetic_track_deg = ir.track_angle_magnetic_deg.Data;
  d.Psi_true_deg = ir.heading_true_deg.Data;
  d.Chi_true_deg = ir.track_angle_true_deg.Data;
  d.bx_m_s2 = kG * ir.body_long_accel_g.Data;
  d.by_m_s2 = kG * ir.body_lat_accel_g.Data;
  d.bz_m_s2 = (ir.body_normal_accel_g.Data + kOneG) * kG;

  // ILS: the runway course is the memorised one; the DME distance is frozen with it.
  if (!logic.ils_tune_inhibit) {
    st.locDistanceMemoNmi = fms.fms_loc_distance;
  }
  d.nav_loc_deg = logic.rwy_hdg_memo;
  d.nav_gs_deg = fms.fms_unrealistic_gs_angle_deg;
  d.nav_dme_nmi = st.locDistanceMemoNmi;
  d.nav_loc_magvar_deg = kLocMagvarDeg;
  d.nav_loc_error_deg = ils.localizer_deviation_deg.Data;
  d.nav_gs_valid = isNormal(ils.glideslope_deviation_deg);
  d.nav_gs_error_deg = ils.glideslope_deviation_deg.Data;

  d.fms_xtk_nmi = fms.xtk_nmi;
  d.fms_tae_deg = fms.tke_deg;
  d.fms_phi_deg = fms.phi_c_deg;
  d.fms_phi_limit_deg = fms.phi_limit_deg;
  d.fms_H_c_profile_ft = fms.alt_profile_tgt_ft;
  d.fms_H_dot_c_profile_ft_min = fms.vs_target_ft_min;
  d.VLS_kn = valueOr(fac.v_ls_kn, 0.0F);
  d.VMAX_kn = valueOr(fac.v_max_kn, 0.0F);
  d.on_ground = logic.on_ground;
  d.zeta_deg = kZetaDeg;
  d.total_weight_kg = kKgPerLb * fms.fms_weight_lbs;

  ap_raw_laws_input& u = io.input;
  u.ap_engaged = logic.ap_own_engaged;
  u.lateral_law = static_cast<double>(apFd.active_lateral_law);
  u.vertical_law = static_cast<double>(apFd.active_longitudinal_law);
  u.Psi_c_deg = valueOr(fcu.selected_hdg_deg, 0.0F);

  // GA TRK and RWY TRK hold the track at their engagement, the other track modes the FCU track.
  const bool trackHold = lat.roll_goaround_active || lat.rwy_trk_submode_active;
  if (!trackHold) {
    st.trackMemoDeg = valueOr(ir.track_angle_magnetic_deg, 0.0F);
  }
  u.Chi_c_deg = trackHold ? st.trackMemoDeg : valueOr(fcu.selected_trk_deg, 0.0F);

  u.H_c_ft = apFd.alt_sel_or_cstr;
  // V/S target: TCAS's, the FMS's in DES, else the FCU's. The FPA target in DES is the same FMS V/S field.
  if (lon.tcas_active) {
    u.H_dot_c_fpm = apFd.tcas_vs_target;
  } else if (lon.des_active) {
    u.H_dot_c_fpm = fms.vs_target_ft_min;
  } else {
    u.H_dot_c_fpm = valueOr(fcu.selected_vz_ft_min, 0.0F);
  }
  u.FPA_c_deg = lon.des_active ? fms.vs_target_ft_min : valueOr(fcu.selected_fpa_deg, 0.0F);
  u.V_c_kn = apFd.spd_target_kts;
  u.ALT_soft_mode_active = apFd.alt_soft_mode_active;
  u.TCAS_mode_active = lon.tcas_active;
  u.FINAL_DES_mode_active = lon.final_des_active;
  u.GS_track_mode = lon.gs_trk_active || lat.land_active;
}

}  // namespace fmgc::readable
