// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FMGC output packing. Generated: FmgcComputer.cpp 4642–4925 (SSM decisions and discrete words), 4926–5106 (assembly
// of FmgcComputer_Y.out). Spec: docs/output-buses.md. Bit numbers are ARINC bits (1-based), see output_packing.h.

#include "output_packing.h"

namespace fmgc::readable::output {

namespace {

constexpr uint32_t kNormal = static_cast<uint32_t>(SignStatusMatrix::NormalOperation);
constexpr uint32_t kNoComputedData = static_cast<uint32_t>(SignStatusMatrix::NoComputedData);

/** SSM: NormalOperation when `valid`, else NoComputedData. Every conditional SSM of the FMGC buses is of this form. */
uint32_t normalIf(bool valid) {
  return valid ? kNormal : kNoComputedData;
}

base_arinc_429 word(uint32_t ssm, double value) {
  return {ssm, static_cast<real32_T>(value)};
}

/**
 * A discrete word: bits 11–29 (the ARINC data field), SSM always NormalOperation. Unset bits are spares (the generated
 * packer writes a constant false there).
 */
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

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// Discrete words
// ---------------------------------------------------------------------------------------------------------------------

/**
 * Discrete word 1: vertical modes (generated 4827–4858, `rtb_y_dd`; A bus and B bus). The FMA longitudinal column is
 * decoded from these bits (FlyByWireInterface 1937–1949); the opposite FMGC copies its modes from bits 19–21.
 *
 * | bit | meaning                                   | bit | meaning                                    |
 * |-----|-------------------------------------------|-----|--------------------------------------------|
 * | 11  | climb mode: CLB, OP CLB or EXP CLB        | 21  | capture mode: ALT* or G/S*                 |
 * | 12  | descent mode: DES, OP DES or EXP DES      | 22  | G/S mode: G/S* or G/S                      |
 * | 13  | spare ("immediate mode")                  | 23  | FINAL DES                                  |
 * | 14  | open mode: OP CLB or OP DES               | 24  | EXPED: EXP CLB or EXP DES                  |
 * | 15  | SRS takeoff                               | 25  | FLARE                                      |
 * | 16  | SRS go-around                             | 26  | FMA dash display (ALT CRZ)                 |
 * | 17  | V/S                                       | 27  | FMA SPEED/MACH preset display: MCDU preset |
 * | 18  | FPA                                       |     | Mach > 0 or preset speed > 0               |
 * | 19  | ALT mode: ALT* or ALT                     | 28  | spare                                      |
 * | 20  | track mode: ALT or G/S                    | 29  | TOGA lever set for go-around (tla_to_ga)   |
 */
base_arinc_429 discreteWord1(const StageResults& r) {
  const auto& m = r.apFd.longitudinal_modes;
  const auto& fms = r.data.fms_inputs;
  return DiscreteWord()
      .bit(11, m.clb_active || m.op_clb_active || m.exp_clb_active)
      .bit(12, m.des_active || m.op_des_active || m.exp_des_active)
      .bit(14, m.op_clb_active || m.op_des_active)
      .bit(15, m.pitch_takeoff_active)
      .bit(16, m.pitch_goaround_active)
      .bit(17, m.vs_active)
      .bit(18, m.fpa_active)
      .bit(19, m.alt_acq_active || m.alt_hold_active)
      .bit(20, m.alt_hold_active || m.gs_trk_active)
      .bit(21, m.alt_acq_active || m.gs_capt_active)
      .bit(22, m.gs_capt_active || m.gs_trk_active)
      .bit(23, m.final_des_active)
      .bit(24, m.exp_des_active || m.exp_clb_active)
      .bit(25, m.flare_active)
      .bit(26, m.fma_dash_display)
      .bit(27, fms.preset_mach > 0.0 || fms.preset_spd_kts > 0.0)
      .bit(29, r.apFd.tla_to_ga_set)
      .word();
}

/**
 * Discrete word 2: lateral modes (generated 4859–4878, `rtb_y_j`; A bus and B bus).
 *
 * | bit | meaning                  | bit | meaning                         |
 * |-----|--------------------------|-----|---------------------------------|
 * | 11  | RWY or RWY TRK (rwy_active) | 20  | RWY LOC submode (RWY)         |
 * | 12  | NAV                      | 21  | spare ("H/PATH submode")        |
 * | 13  | LOC*                     | 22  | spare ("HDG submode")           |
 * | 14  | LOC                      | 23  | RWY TRK submode                 |
 * | 15  | GA TRK (roll go-around)  | 24  | spare ("VOR submode")           |
 * | 16  | HDG                      | 25  | ALIGN submode                   |
 * | 17  | TRK                      | 26  | ROLL OUT submode                |
 * | 18  | spare                    | 27  | spare                           |
 * | 19  | spare                    | 28  | roll FD bars flashing           |
 * |     |                          | 29  | spare                           |
 */
base_arinc_429 discreteWord2(const StageResults& r) {
  const auto& l = r.apFd.lateral_modes;
  return DiscreteWord()
      .bit(11, l.rwy_active)
      .bit(12, l.nav_active)
      .bit(13, l.loc_cpt_active)
      .bit(14, l.loc_trk_active)
      .bit(15, l.roll_goaround_active)
      .bit(16, l.hdg_active)
      .bit(17, l.trk_active)
      .bit(20, l.rwy_loc_submode_active)
      .bit(23, l.rwy_trk_submode_active)
      .bit(25, l.align_submode_active)
      .bit(26, l.rollout_submode_active)
      .bit(28, r.apFd.roll_fd_bars_flashing)
      .word();
}

/**
 * Discrete word 3: armed modes (generated 4807–4826, `rtb_y_a`; A bus and B bus).
 *
 * | bit | meaning                       | bit | meaning                               |
 * |-----|-------------------------------|-----|---------------------------------------|
 * | 11  | HDG/TRK preset available      | 20  | LAND armed                            |
 * | 12  | ALT armed                     | 21  | spare ("G/S capt before LOC capt")    |
 * | 13  | ALT arm possible              | 22  | G/S armed                             |
 * | 14  | NAV armed                     | 23  | FINAL DES armed                       |
 * | 15  | spare                         | 24  | CLB armed                             |
 * | 16  | LOC armed                     | 25  | DES armed                             |
 * | 17  | spare ("FG approach phase")   | 26  | spare ("long mode reset")             |
 * | 18  | spare ("FMA longi mode")      | 27  | spare ("lat mode reset")              |
 * | 19  | LOC backbeam selected (FMS)   | 28  | spare ("QFU equal COM")               |
 * |     |                               | 29  | spare ("FMA longi box")               |
 */
base_arinc_429 discreteWord3(const StageResults& r) {
  const auto& a = r.apFd.armed_modes;
  return DiscreteWord()
      .bit(11, r.apFd.hdg_trk_preset_available)
      .bit(12, a.alt_acq_armed)
      .bit(13, a.alt_acq_arm_possible)
      .bit(14, a.nav_armed)
      .bit(16, a.loc_armed)
      .bit(19, r.data.fms_inputs.backbeam_selected)
      .bit(20, a.land_armed)
      .bit(22, a.glide_armed)
      .bit(23, a.final_des_armed)
      .bit(24, a.clb_armed)
      .bit(25, a.des_armed)
      .word();
}

/**
 * Discrete word 4: AP/FD engagement and landing capability (generated 4737–4756, `rtb_y_i3`; A bus and B bus).
 *
 * | bit | meaning                                | bit | meaning                                     |
 * |-----|----------------------------------------|-----|---------------------------------------------|
 * | 11  | AP instinctive disconnect (pushbutton) | 21  | LAND 3 FAIL PASSIVE inop                    |
 * | 12  | AP own engaged                         | 22  | LAND 3 FAIL OP inop                         |
 * | 13  | FD own engaged                         | 23  | LAND 2 capacity                             |
 * | 14  | LAND (land_active)                     | 24  | LAND 3 FAIL PASSIVE capacity                |
 * | 15  | spare                                  | 25  | LAND 3 FAIL OP capacity                     |
 * | 16  | LAND 2 capability                      | 26  | ILS tune inhibit (doc: "RWY hdg memorized") |
 * | 17  | LAND 3 FAIL PASSIVE capability         | 27  | FD auto disengage                           |
 * | 18  | LAND 3 FAIL OP capability              | 28  | AP/FD mode reversion                        |
 * | 19  | AP inop                                | 29  | V/S target not held (speed protection)      |
 * | 20  | LAND 2 inop                            |     |                                             |
 */
base_arinc_429 discreteWord4(const StageResults& r) {
  const auto& a = r.apFd;
  return DiscreteWord()
      .bit(11, r.data.discrete_inputs.ap_instinctive_disc)
      .bit(12, r.logic.ap_own_engaged)
      .bit(13, r.logic.fd_own_engaged)
      .bit(14, a.lateral_modes.land_active)
      .bit(16, a.land_2_capability)
      .bit(17, a.land_3_fail_passive_capability)
      .bit(18, a.land_3_fail_op_capability)
      .bit(19, r.logic.ap_inop)
      .bit(20, a.land_2_inop)
      .bit(21, a.land_3_fail_passive_inop)
      .bit(22, a.land_3_fail_op_inop)
      .bit(23, a.land_2_capacity)
      .bit(24, a.land_3_fail_passive_capacity)
      .bit(25, a.land_3_fail_op_capacity)
      .bit(26, r.logic.ils_tune_inhibit)
      .bit(27, a.fd_auto_disengage)
      .bit(28, a.ap_fd_mode_reversion)
      .bit(29, a.vs_target_not_held)
      .word();
}

/**
 * Discrete word 5: speed control (generated 4716–4736; A bus only). Bits 11–16 (Mach selected, DES submodes) and 27
 * (SPD window display) are documented in a320-simvars.md but packed as spares.
 *
 * | bit | meaning                                          |
 * |-----|--------------------------------------------------|
 * | 19  | managed speed (auto_spd_control_active)          |
 * | 20  | selected speed (manual_spd_control_active)       |
 * | 24  | pitch FD bars flashing                           |
 * | 28  | MCDU preset speed/Mach activated (FMS input; doc: "top of SPD synchro") |
 * | others | spare                                         |
 */
base_arinc_429 discreteWord5(const StageResults& r) {
  return DiscreteWord()
      .bit(19, r.apFd.auto_spd_control_active)
      .bit(20, r.apFd.manual_spd_control_active)
      .bit(24, r.apFd.pitch_fd_bars_flashing)
      .bit(28, r.data.fms_inputs.preset_spd_mach_activate)
      .word();
}

/** Discrete word 6 (generated 4879–4899; A bus only): every bit spare, SSM NormalOperation. */
base_arinc_429 discreteWord6(const StageResults& /*r*/) {
  return DiscreteWord().word();
}

/**
 * Discrete word 7: TCAS (generated 4904–4925; A bus only). The opposite FMGC reads bit 13 (TCAS mode sync).
 *
 * | bit | meaning                                     |
 * |-----|---------------------------------------------|
 * | 11  | AP/FD TCAS mode installed: constant true    |
 * | 12  | TCAS mode armed                             |
 * | 13  | TCAS mode active                            |
 * | 18  | TCAS large box on the FMA (longi_large_box_tcas) |
 * | 24  | TCAS RA inhibited message                   |
 * | 25  | TRK FPA deselected message                  |
 * | 27  | TCAS RA corrective                          |
 * | 28  | TCAS V/S target > 0 (nose up)               |
 * | 29  | TCAS V/S target < 0 (nose down)             |
 * | others | spare (14 "TCAS mode fault", 20 "soft GA inop" in a320-simvars.md) |
 */
base_arinc_429 discreteWord7(const StageResults& r) {
  const auto& a = r.apFd;
  return DiscreteWord()
      .bit(11, true)
      .bit(12, a.armed_modes.tcas_armed)
      .bit(13, a.longitudinal_modes.tcas_active)
      .bit(18, a.longi_large_box_tcas)
      .bit(24, a.tcas_ra_inhibited)
      .bit(25, a.trk_fpa_deselected)
      .bit(27, a.tcas_ra_corrective)
      .bit(28, a.tcas_vs_target > 0.0)
      .bit(29, a.tcas_vs_target < 0.0)
      .word();
}

/**
 * ATS discrete word: A/THR engagement and modes (generated 4766–4785, `rtb_y_gd`; A bus and B bus).
 *
 * | bit | meaning                                | bit | meaning                              |
 * |-----|----------------------------------------|-----|--------------------------------------|
 * | 13  | A/THR own engaged                      | 20  | RETARD mode                          |
 * | 14  | A/THR active                           | 21  | THR (N1) mode                        |
 * | 17  | A/THR instinctive disconnect (pb)      | 22  | spare ("THR EPR mode")               |
 * | 18  | SPEED/MACH mode                        | 23  | alpha floor                          |
 * | 19  | Mach control active (FMGC Mach latch)  | 24  | A/THR inop                           |
 * |     |                                        | 25  | A/THR limited                        |
 * | others | spare                               |     |                                      |
 */
base_arinc_429 atsDiscreteWord(const StageResults& r) {
  return DiscreteWord()
      .bit(13, r.logic.athr_own_engaged)
      .bit(14, r.athr.athr_active)
      .bit(17, r.data.discrete_inputs.athr_instinctive_disc)
      .bit(18, r.athr.speed_mach_mode_active)
      .bit(19, r.apFd.mach_control_active)
      .bit(20, r.athr.retard_mode_active)
      .bit(21, r.athr.thrust_mode_active)
      .bit(23, r.athr.alpha_floor_mode_active)
      .bit(24, r.logic.athr_inop)
      .bit(25, r.athr.athr_limited)
      .word();
}

/**
 * ATS FMA discrete word: the A/THR FMA column (generated 4786–4806; A bus only). One bit per FMA text, decoded from
 * athr.fma_mode / fma_message.
 *
 * | bit | FMA text      | when                     | bit | FMA text  | when                   |
 * |-----|---------------|--------------------------|-----|-----------|------------------------|
 * | 11  | TOGA          | MAN_TOGA                 | 18  | TOGA LK   | TOGA_LK                |
 * | 12  | MCT           | MAN_MCT or THR_MCT       | 19  | SPEED     | SPEED                  |
 * | 13  | FLX           | MAN_FLEX                 | 20  | MACH      | MACH                   |
 * | 14  | CLB           | THR_CLB                  | 21  | ASYM      | message LVR_ASYM       |
 * | 15  | THR           | MAN_THR or THR_LVR       | 22  | LVR CLB   | message LVR_CLB        |
 * | 16  | IDLE          | THR_IDLE                 | 23  | LVR MCT   | message LVR_MCT        |
 * | 17  | A.FLOOR       | A_FLOOR                  |     |           |                        |
 * | others | spare      |                          |     |           |                        |
 */
base_arinc_429 atsFmaDiscreteWord(const StageResults& r) {
  const athr_fma_mode mode = r.athr.fma_mode;
  const athr_fma_message message = r.athr.fma_message;
  return DiscreteWord()
      .bit(11, mode == athr_fma_mode::MAN_TOGA)
      .bit(12, mode == athr_fma_mode::MAN_MCT || mode == athr_fma_mode::THR_MCT)
      .bit(13, mode == athr_fma_mode::MAN_FLEX)
      .bit(14, mode == athr_fma_mode::THR_CLB)
      .bit(15, mode == athr_fma_mode::MAN_THR || mode == athr_fma_mode::THR_LVR)
      .bit(16, mode == athr_fma_mode::THR_IDLE)
      .bit(17, mode == athr_fma_mode::A_FLOOR)
      .bit(18, mode == athr_fma_mode::TOGA_LK)
      .bit(19, mode == athr_fma_mode::SPEED)
      .bit(20, mode == athr_fma_mode::MACH)
      .bit(21, message == athr_fma_message::LVR_ASYM)
      .bit(22, message == athr_fma_message::LVR_CLB)
      .bit(23, message == athr_fma_message::LVR_MCT)
      .word();
}

// ---------------------------------------------------------------------------------------------------------------------
// Buses
// ---------------------------------------------------------------------------------------------------------------------

/**
 * FMGC A bus (generated SSM decisions 4642–4714, 4757–4765 and 4900–4902, values
 * 4964–5052). Value words: SSM NormalOperation
 * unless a rule is given; the Data are float32 conversions of the double sources. Whole-word copies keep the source's
 * SSM.
 */
base_fmgc_a_bus fmgcABus(const StageResults& r) {
  const auto& fms = r.data.fms_inputs;
  const auto& fcu = r.data.bus_inputs.fcu_bus;
  const auto& a = r.apFd;
  const auto& lat = a.lateral_modes;
  const auto& ir = r.logic.ir_computation_data;
  const auto& adr = r.logic.adr_computation_data;
  const auto& ap = r.outerLoops.autopilot;
  const auto& fd = r.outerLoops.flight_director;
  base_fmgc_a_bus b{};

  // PFD speed target: NCD only when the FCU shows neither a speed nor a Mach (both words not normal) and the speed is
  // not managed (4643–4653).
  bool fcuSpdOrMachShown = fcu.selected_mach.SSM == kNormal || fcu.selected_spd_kts.SSM == kNormal;
  b.pfd_sel_spd_kts = word(normalIf(fcuSpdOrMachShown || a.auto_spd_control_active), a.pfd_spd_target_kts);
  // Memorised runway heading: normal when the ILS runway (course) word is normal or the ILS tuning is inhibited
  // (4654–4664); the value is the logic bus memo.
  b.runway_hdg_memorized_deg = word(
      normalIf(r.logic.ils_computation_data.runway_heading_deg.SSM == kNormal || r.logic.ils_tune_inhibit),
      r.logic.rwy_hdg_memo);
  // MCDU preset speed and Mach: normal when set (> 0) (4666–4682).
  b.preset_mach_from_mcdu = word(normalIf(fms.preset_mach > 0.0), fms.preset_mach);
  b.preset_speed_from_mcdu_kts = word(normalIf(fms.preset_spd_kts > 0.0), fms.preset_spd_kts);
  // FD commands (the FD bars): roll with any lateral mode except RWY and ROLL OUT, pitch with any longitudinal mode
  // except in ROLL OUT, yaw only in RWY, ALIGN or ROLL OUT (4684–4714).
  b.roll_fd_command = word(normalIf(a.any_lateral_mode_engaged && !lat.rwy_loc_submode_active &&
                                    !lat.rollout_submode_active),
                           fd.Phi_c_deg);
  b.pitch_fd_command = word(normalIf(a.any_longitudinal_mode_engaged && !lat.rollout_submode_active), fd.Theta_c_deg);
  b.yaw_fd_command = word(normalIf(lat.rwy_loc_submode_active || lat.align_submode_active ||
                                   lat.rollout_submode_active),
                          fd.Beta_c_deg);
  b.discrete_word_5 = discreteWord5(r);
  b.discrete_word_4 = discreteWord4(r);
  // Next FMS altitude constraint: normal only while it is the altitude target (alt_cstr_applicable, 4980–4988); the
  // value is always sent.
  b.fm_alt_constraint_ft = word(normalIf(a.alt_cstr_applicable), fms.next_alt_cstr_ft);
  // Air data relayed from the selected sources (whole words).
  b.altitude_ft = r.logic.altitude_indicated_ft;
  b.mach = adr.mach;
  b.cas_kts = adr.airspeed_computed_kn;
  // FLX TO temperature: normal when set (!= 0) (4757–4765).
  b.flx_to_temp_deg_c = word(normalIf(fms.flex_temp_deg_c != 0.0), fms.flex_temp_deg_c);
  b.ats_discrete_word = atsDiscreteWord(r);
  b.ats_fma_discrete_word = atsFmaDiscreteWord(r);
  b.discrete_word_3 = discreteWord3(r);
  b.discrete_word_1 = discreteWord1(r);
  b.discrete_word_2 = discreteWord2(r);
  b.discrete_word_6 = discreteWord6(r);
  // Speed synchro value: constant 0, SSM normal.
  b.synchro_spd_mach_value = word(kNormal, 0.0);
  // DES speed margins (FMS): normal when the FMS asks to show them (4642, 4900–4902).
  b.low_target_speed_margin_kts = word(normalIf(fms.show_speed_margins), fms.v_lower_margin_kts);
  b.high_target_speed_margin_kts = word(normalIf(fms.show_speed_margins), fms.v_upper_margin_kts);
  // AP commands to the flight controls ("voted"): roll command to both ailerons and spoilers, sideslip to the rudder,
  // nosewheel, pitch.
  b.delta_p_ail_voted_cmd_deg = word(kNormal, ap.Phi_c_deg);
  b.delta_p_splr_voted_cmd_deg = word(kNormal, ap.Phi_c_deg);
  b.delta_r_voted_cmd_deg = word(kNormal, ap.Beta_c_deg);
  b.delta_nosewheel_voted_cmd_deg = word(kNormal, r.outerLoops.Nosewheel_c);
  b.delta_q_voted_cmd_deg = word(kNormal, ap.Theta_c_deg);
  // Inertial data relayed from the selected IR (whole words).
  b.track_deg = ir.track_angle_magnetic_deg;
  b.heading_deg = ir.heading_magnetic_deg;
  b.fpa_deg = ir.flight_path_angle_deg;
  b.n1_command_percent = word(kNormal, r.athr.n1_c_percent);
  b.vertical_speed_ft_min = ir.inertial_vertical_speed_ft_s;  // the IR word, whatever its unit name
  b.discrete_word_7 = discreteWord7(r);
  return b;
}

/**
 * FMGC B bus (generated 5053–5106): weights, CG, radio height, the AP commands and copies of discrete words 1–4 and the
 * ATS word. FM weight and CG, the approach speed and the N1 words use a second NormalOperation constant
 * (EnumeratedConstant1_Value_d), same value.
 */
base_fmgc_b_bus fmgcBBus(const StageResults& r) {
  const auto& fms = r.data.fms_inputs;
  const auto& fac = r.logic.chosen_fac_bus;
  const auto& ap = r.outerLoops.autopilot;
  base_fmgc_b_bus b{};
  b.fac_weight_lbs = fac.total_weight_lbs;
  b.fm_weight_lbs = word(kNormal, fms.fms_weight_lbs);
  b.fac_cg_percent = fac.center_of_gravity_pos_percent;
  b.fm_cg_percent = word(kNormal, fms.fms_cg_percent);
  b.fg_radio_height_ft = r.logic.ra_computation_data.radio_height_ft;
  b.discrete_word_4 = discreteWord4(r);
  b.ats_discrete_word = atsDiscreteWord(r);
  b.discrete_word_3 = discreteWord3(r);
  b.discrete_word_1 = discreteWord1(r);
  b.discrete_word_2 = discreteWord2(r);
  b.approach_spd_target_kn = word(kNormal, 0.0);  // constant 0 (not computed), SSM normal
  b.delta_p_ail_cmd_deg = word(kNormal, ap.Phi_c_deg);
  b.delta_p_splr_cmd_deg = word(kNormal, ap.Phi_c_deg);
  b.delta_r_cmd_deg = word(kNormal, ap.Beta_c_deg);
  b.delta_nose_wheel_cmd_deg = word(kNormal, r.outerLoops.Nosewheel_c);
  b.delta_q_cmd_deg = word(kNormal, ap.Theta_c_deg);
  b.n1_left_percent = word(kNormal, 0.0);   // constant 0, SSM normal
  b.n1_right_percent = word(kNormal, 0.0);  // constant 0, SSM normal
  return b;
}

/** Hardwired discretes (generated 4957–4963). */
base_fmgc_discrete_outputs discreteOutputs(const StageResults& r) {
  base_fmgc_discrete_outputs d{};
  d.athr_own_engaged = r.logic.athr_own_engaged;
  d.fd_own_engaged = r.logic.fd_own_engaged;
  d.ap_own_engaged = r.logic.ap_own_engaged;
  d.fcu_own_fail = false;   // constant
  d.fmgc_healthy = true;    // constant: the FMGC never reports itself unhealthy
  d.ils_test_inhibit = r.logic.ils_tune_inhibit;
  d.stick_rudder_lock = r.logic.ap_own_engaged;
  return d;
}

/** FmgcComputer_Y.out (generated 4926–5106): the stage results copied unchanged, then the packed outputs. */
fmgc_outputs assemble(const StageResults& r) {
  fmgc_outputs out{};
  out.data = r.data;
  out.logic = r.logic;
  out.ap_fd_logic = r.apFd;
  out.ap_fd_outer_loops = r.outerLoops;
  out.athr = r.athr;
  out.discrete_outputs = discreteOutputs(r);
  out.bus_outputs.fmgc_a_bus = fmgcABus(r);
  out.bus_outputs.fmgc_b_bus = fmgcBBus(r);
  return out;
}

}  // namespace fmgc::readable::output
