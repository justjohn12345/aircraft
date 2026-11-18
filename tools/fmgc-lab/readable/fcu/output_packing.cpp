// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FCU output packing (generated FcuComputer.cpp 1184-1533). See output_packing.h.

#include "output_packing.h"

#include <cstdint>

namespace fcu::readable {

namespace {

constexpr uint32_t kNormal = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
constexpr uint32_t kNoComputedData = static_cast<uint32_t>(SignStatusMatrix::NoComputedData);

/** A discrete word: flags in ARINC bits 11-29, SSM normal operation (generated MATLABFunction_e). */
class DiscreteWord {
 public:
  DiscreteWord& bit(int arincBit, bool value) {
    bits_ |= static_cast<uint32_t>(value) << (arincBit - 1);
    return *this;
  }
  base_arinc_429 word() const { return {kNormal, static_cast<real32_T>(bits_)}; }

 private:
  uint32_t bits_ = 0;
};

base_arinc_429 value(float data, bool noComputedData) {
  return {noComputedData ? kNoComputedData : kNormal, data};
}

/** EIS discrete word 1 (1210-1259): baro unit and ND range. */
base_arinc_429 eisWord1(const base_fcu_efis_panel_inputs& p) {
  using R = efis_range_selection;
  return DiscreteWord()
      .bit(11, p.baro_is_inhg)
      .bit(25, p.efis_range == R::RANGE_10)
      .bit(26, p.efis_range == R::RANGE_20)
      .bit(27, p.efis_range == R::RANGE_40)
      .bit(28, p.efis_range == R::RANGE_80)
      .bit(29, p.efis_range == R::RANGE_160)
      .word();
}

/** EIS discrete word 2 (1260-1319): ND mode, filter, LS, FD, navaids, baro reference. */
base_arinc_429 eisWord2(const base_fcu_efis_panel_inputs& p, const base_fcu_efis_logic_outputs& l) {
  using M = efis_mode_selection;
  using F = efis_filter_selection;
  using N = efis_navaid_selection;
  return DiscreteWord()
      .bit(11, p.efis_mode == M::PLAN)
      .bit(12, p.efis_mode == M::ARC)
      .bit(13, p.efis_mode == M::ROSE_NAV)
      .bit(14, p.efis_mode == M::ROSE_VOR)
      .bit(15, p.efis_mode == M::ROSE_ILS)
      .bit(17, l.efis_filter == F::CSTR)
      .bit(18, l.efis_filter == F::WPT)
      .bit(19, l.efis_filter == F::VORD)
      .bit(20, l.efis_filter == F::NDB)
      .bit(21, l.efis_filter == F::ARPT)
      .bit(22, l.ls_on)
      .bit(23, !l.fd_on)
      .bit(24, p.efis_navaid_1 == N::ADF)
      .bit(25, p.efis_navaid_2 == N::ADF)
      .bit(26, p.efis_navaid_1 == N::VOR)
      .bit(27, p.efis_navaid_2 == N::VOR)
      .bit(28, l.baro_std)
      .bit(29, l.baro_qnh)
      .word();
}

}  // namespace

base_fcu_discrete_outputs packDiscreteOutputs(const fcu_inputs& in, const base_fcu_logic_outputs& logic,
                                              const base_fcu_efis_panel_outputs& captPanel,
                                              const base_fcu_efis_panel_outputs& foPanel) {
  const auto& d = in.discrete_inputs;
  const base_afs_logic_outputs& afs = logic.afs;
  base_fcu_discrete_outputs o{};
  o.capt_efis_outputs = captPanel;
  o.fo_efis_outputs = foPanel;
  auto& a = o.afs_outputs;
  a.loc_light_on = afs.loc_only_active;
  a.exped_light_on = afs.exped_active;
  a.appr_light_on = afs.appr_active;
  a.ap_1_light_on = d.ap_1_engaged;
  a.ap_2_light_on = d.ap_2_engaged;
  a.athr_light_on = d.athr_1_engaged || d.athr_2_engaged;
  a.trk_fpa_mode = afs.trk_fpa_active;
  a.mach_mode = afs.mach_active;
  a.spd_mach_value = afs.spd_mach_display_value;
  a.spd_mach_dashes = afs.spd_mach_dashes;
  a.spd_mach_managed = afs.auto_speed_control;
  a.hdg_trk_value = afs.hdg_trk_display_value;
  a.hdg_trk_dashes = afs.hdg_trk_dashes;
  a.hdg_trk_managed = afs.hdg_trk_managed;
  a.alt_value = afs.alt_display_value;
  a.lvl_ch_managed = afs.lvl_ch_managed;
  a.vs_fpa_value = afs.vs_fpa_display_value;
  a.vs_fpa_dashes = afs.vs_fpa_dashes;
  o.fcu_healthy = true;  // Constant1_Value_i: the sim wrapper sets the real health
  return o;
}

base_fcu_bus packBusOutputs(const fcu_inputs& in, const FmgcView& fmgc, const base_fcu_logic_outputs& logic) {
  const auto& d = in.discrete_inputs;
  const base_afs_logic_outputs& afs = logic.afs;
  const base_fmgc_a_bus& prio = fmgc.fmgc1HasPriority ? in.bus_inputs.fmgc_1_bus : in.bus_inputs.fmgc_2_bus;
  const bool trk = afs.trk_fpa_active;
  const bool mach = afs.mach_active;
  base_fcu_bus b{};

  // selected targets (1450-1505)
  b.selected_hdg_deg = value(afs.hdg_trk_display_value, afs.hdg_trk_dashes || trk);
  b.selected_trk_deg = value(afs.hdg_trk_display_value, afs.hdg_trk_dashes || !trk);
  b.selected_alt_ft = value(afs.alt_display_value, false);
  b.selected_spd_kts =
      value(mach ? afs.chosen_fmgc_data.v_cas_kts : afs.spd_mach_display_value, afs.spd_mach_dashes || mach);
  b.selected_mach = value(mach ? afs.spd_mach_display_value : afs.chosen_fmgc_data.v_mach, afs.spd_mach_dashes || !mach);
  b.selected_vz_ft_min = value(afs.vs_fpa_display_value, afs.vs_fpa_dashes || trk);
  b.selected_fpa_deg = value(afs.vs_fpa_display_value, afs.vs_fpa_dashes || !trk);

  // A/THR words of the priority FMGC (1184-1208, 1360-1366)
  b.ats_fma_discrete_word = prio.ats_fma_discrete_word;
  b.fcu_flex_to_temp_deg_c = prio.flx_to_temp_deg_c;
  b.ats_discrete_word = prio.ats_discrete_word;
  b.n1_cmd_percent = prio.n1_command_percent;

  // EIS words and baro settings
  b.eis_discrete_word_1_left = eisWord1(d.capt_efis_inputs);
  b.eis_discrete_word_1_right = eisWord1(d.fo_efis_inputs);
  b.eis_discrete_word_2_left = eisWord2(d.capt_efis_inputs, logic.capt_efis);
  b.eis_discrete_word_2_right = eisWord2(d.fo_efis_inputs, logic.fo_efis);
  b.baro_setting_left_hpa = value(logic.capt_efis.baro_value_hpa, false);
  b.baro_setting_right_hpa = value(logic.fo_efis.baro_value_hpa, false);
  b.baro_setting_left_inhg = value(logic.capt_efis.baro_value_inhg, false);
  b.baro_setting_right_inhg = value(logic.fo_efis.baro_value_inhg, false);

  // FCU discrete words (1320-1359)
  b.fcu_discrete_word_2 = DiscreteWord()
                              .bit(11, afs.hdg_trk_buttons.pushed)
                              .bit(12, afs.hdg_trk_buttons.pulled)
                              .bit(13, afs.loc_pushed)
                              .bit(14, afs.lat_value_changed)
                              .bit(20, afs.fmgc_1_has_priority)
                              .bit(21, !afs.fmgc_1_has_priority)
                              .bit(26, !logic.capt_efis.fd_on)
                              .bit(27, !logic.fo_efis.fd_on)
                              .word();
  b.fcu_discrete_word_1 = DiscreteWord()
                              .bit(11, afs.spd_mach_buttons.pushed)
                              .bit(12, afs.spd_mach_buttons.pulled)
                              .bit(13, afs.alt_value_changed)
                              .bit(14, afs.vpath_value_changed)
                              .bit(15, afs.spd_mach_value_changed)
                              .bit(16, afs.vs_fpa_buttons.pushed)
                              .bit(17, afs.alt_buttons.pushed)
                              .bit(18, afs.alt_buttons.pulled)
                              .bit(19, afs.vs_fpa_buttons.pulled)
                              .bit(20, afs.metric_alt_active)
                              .bit(21, afs.spd_mach_switching_pushed)
                              .bit(22, afs.exped_pushed)
                              .bit(23, afs.appr_pushed)
                              .bit(24, !trk)
                              .bit(25, trk)
                              .word();
  return b;
}

}  // namespace fcu::readable
