// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FMGC input conditioning: sensor source selection and voting, the logic bus. Spec: docs/conditioning.md.
// Generated: FmgcComputer.cpp 695–1379 (master); copied to the logic bus at 3864–3971 (the ADR Mach/TAS/AoA, the IR
// and FAC words without an SSM consumer, the ILS frequency, fac_flap_slat_data_failure and tcas_failure are written to
// rtb_BusAssignment_jmp.logic directly at 781–1364). Init: confirm node at 522 (first running frame), the runway
// heading memory FmgcComputer_B.u_lyjj only in initialize() (5160).
//
// The generated temporaries are reused for unrelated signals (rtb_adrOwnInvalid ends up holding all_ir_valid,
// raOwnInvalid/raOppInvalid first hold "own/3 ADR valid", then the RA flags; rtb_irOwnInvalid ends up holding
// fac_rudder_control_failure, rtb_ir3Invalid both_fac_rudder_valid). None of them is reassigned between 1379 and the
// final copy at 3864, so every logic field is the value computed here.

#include "conditioning.h"

namespace fmgc::readable::conditioning {

const char* conditionName(int c) {
  static const char* names[] = {
      "adr_own_invalid", "adr_opp_invalid", "adr_3_invalid", "adr_sel_own", "adr_sel_3", "adr_sel_none", "alt_std",
      "alt_std_fcu_word_failed", "alt_corrected_1", "alt_corrected_2", "ir_own_invalid", "ir_opp_invalid",
      "ir_3_invalid", "ir_sel_own", "ir_sel_3", "ir_sel_none", "ra_both_ok", "ra_own_only", "ra_opp_only", "ra_none",
      "fac_sel_own", "fac_sel_opp", "fac_rudder_no_yaw_damper", "fac_rudder_no_trim", "fac_rudder_both_unhealthy",
      "on_ground", "in_flight_5s", "ground_engines_stopped", "flap_lever_0", "flap_lever_1", "flap_lever_2",
      "flap_lever_3", "flap_lever_4", "flap_lever_5", "ils_own_valid", "ils_opp_valid", "ils_sel_opp",
      "loc_backbeam", "tcas_available", "tcas_ra_inhibit", "tune_inhibit_rwy_loc", "tune_inhibit_land",
      "tune_inhibit_land_armed_low", "rwy_hdg_memo_updated"};
  static_assert(sizeof(names) / sizeof(names[0]) == kConditionCount, "one name per condition");
  return c >= 0 && c < kConditionCount ? names[c] : "?";
}

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::isNotFailed;
using blocks::a429::rawValue;
using blocks::a429::valueOr;

constexpr double kInFlightConfirmS = 5.0;   // not on ground this long: "in flight"            [ConfirmNode1_timeDelay]
constexpr float kRaBothFailedFt = 2500.0F;  // radio height when both RAs report a failure
constexpr float kTcasMinRaFt = 900.0F;      // TCAS mode available above this radio height  [CompareToConstant_const_n]
constexpr float kTuneInhibitRaFt = 700.0F;  // LAND armed at or below this RA inhibits the ILS tune [_const_o2]
constexpr float kRwyHdgDefaultDeg = 0.0F;   // runway heading memorised when the ILS course word is not normal
constexpr float kBackbeamGain = -1.0F;      // localizer deviation sign for a back beam      [Gain_Gain_f]
constexpr bool kRudderControlEnabled = true;  // a constant in the rudder control failure OR    [Constant_Value_a]

// Bits (1-based)
constexpr int kFcuEisWord2Std = 28;          // FCU EIS discrete word 2 (left: unit 1, right: unit 2): STD selected
constexpr int kFacWord2YawDamperOwn = 11;    // FAC discrete word 2: yaw damper (own channel) engaged
constexpr int kFacWord2RudderTrimOwn = 13;   //                      rudder trim (own channel) engaged
constexpr int kFacWord5FlapLever1 = 11;      // FAC discrete word 5: flap lever 1..5 = bits 11..15
constexpr int kFacWord5AllLgciuLost = 17;    //                      all LGCIUs lost
constexpr int kFacWord5LeftMainGear = 18;    //                      left main gear compressed
constexpr int kFacWord5RightMainGear = 19;   //                      right main gear compressed
constexpr int kFacWord5AllSfccLost = 22;     //                      all SFCCs lost
constexpr int kTcasWordStandby = 25;         // TCAS mode word ("sensitivity_level"): STBY
constexpr int kTcasWordTa = 26;              //                      sensitivity > 1 (TA)
constexpr int kTcasWordTaRa = 27;            //                      sensitivity > 2 (TA/RA)

// The bus used when no ADR / IR is valid: every word SSM failure warning (0), data 0 [FmgcComputer_P.Constant1_Value].
const base_adr_bus kNoAdr{};
const base_ir_bus kNoIr{};

bool failed(const base_arinc_429& w) {
  return !isNotFailed(w);
}
bool bitSet(const base_arinc_429& w, int n) {
  return bit(w, n) != 0U;
}

}  // namespace

bool adrValid(const base_adr_bus& adr) {
  return !(failed(adr.airspeed_computed_kn) || failed(adr.aoa_corrected_deg) || failed(adr.mach) ||
           failed(adr.altitude_standard_ft));
}

bool irValid(const base_ir_bus& ir) {
  return isNormal(ir.body_yaw_rate_deg_s) && isNormal(ir.body_lat_accel_g) && isNormal(ir.pitch_angle_deg) &&
         isNormal(ir.roll_angle_deg) && isNormal(ir.heading_true_deg) && isNormal(ir.track_angle_true_deg) &&
         isNormal(ir.inertial_vertical_speed_ft_s) && isNormal(ir.latitude_deg);
}

void Conditioning::update(const Inputs& in, base_fmgc_logic_outputs& out) {
  const auto& di = in.in.discrete_inputs;
  const auto& bus = in.in.bus_inputs;
  const auto& last = in.last;
  const double dt = in.in.time.dt;
  State& s = s_;
  for (bool& c : cond) {
    c = false;
  }
  out = base_fmgc_logic_outputs{};

  // --- ADR (695–720, 772–845): 2-of-3 failure vote over own / opposite / 3; the selected bus is the own ADR, else
  // ADR 3, else a zero bus. The opposite ADR only votes.
  const bool adrOwnOk = adrValid(bus.adr_own_bus);
  const bool adrOppOk = adrValid(bus.adr_opp_bus);
  const bool adr3Ok = adrValid(bus.adr_3_bus);
  out.double_adr_failure = twoOfThree(!adrOwnOk, !adrOppOk, !adr3Ok);
  out.all_adr_valid = adrOwnOk && adrOppOk && adr3Ok;
  out.adr_computation_data = adrOwnOk ? bus.adr_own_bus : (adr3Ok ? bus.adr_3_bus : kNoAdr);
  cond[CondAdrOwnInvalid] = !adrOwnOk;
  cond[CondAdrOppInvalid] = !adrOppOk;
  cond[CondAdr3Invalid] = !adr3Ok;
  cond[adrOwnOk ? CondAdrSelOwn : (adr3Ok ? CondAdrSel3 : CondAdrSelNone)] = true;

  // --- Indicated altitude (847–864): the selected ADR's standard altitude when this side's FCU EIS word 2 shows STD
  // or has failed, else its baro-corrected altitude for this side (1: captain, 2: first officer).
  const base_arinc_429& eisWord2 = di.is_unit_1 ? bus.fcu_bus.eis_discrete_word_2_left : bus.fcu_bus.eis_discrete_word_2_right;
  const bool stdSelected = bitSet(eisWord2, kFcuEisWord2Std);
  if (stdSelected || failed(eisWord2)) {
    out.altitude_indicated_ft = out.adr_computation_data.altitude_standard_ft;
    cond[stdSelected ? CondAltStd : CondAltStdFcuWordFailed] = true;
  } else if (di.is_unit_1) {
    out.altitude_indicated_ft = out.adr_computation_data.altitude_corrected_1_ft;
    cond[CondAltCorrected1] = true;
  } else {
    out.altitude_indicated_ft = out.adr_computation_data.altitude_corrected_2_ft;
    cond[CondAltCorrected2] = true;
  }

  // --- IR (721–771, 866–1074): same vote and selection; an IR word counts as invalid unless in normal operation.
  const bool irOwnOk = irValid(bus.ir_own_bus);
  const bool irOppOk = irValid(bus.ir_opp_bus);
  const bool ir3Ok = irValid(bus.ir_3_bus);
  out.double_ir_failure = twoOfThree(!irOwnOk, !irOppOk, !ir3Ok);
  out.all_ir_valid = irOwnOk && irOppOk && ir3Ok;
  out.ir_computation_data = irOwnOk ? bus.ir_own_bus : (ir3Ok ? bus.ir_3_bus : kNoIr);
  cond[CondIrOwnInvalid] = !irOwnOk;
  cond[CondIrOppInvalid] = !irOppOk;
  cond[CondIr3Invalid] = !ir3Ok;
  cond[irOwnOk ? CondIrSelOwn : (ir3Ok ? CondIrSel3 : CondIrSelNone)] = true;

  // --- RA (1076–1100): own unless it reports a failure, then the opposite; both failed: 2500 ft (with the own,
  // failed, SSM). NCD and FT count as not failed here, but both_ra_valid needs normal operation on both.
  const bool raOwnFailed = failed(bus.ra_own_bus.radio_height_ft);
  const bool raOppFailed = failed(bus.ra_opp_bus.radio_height_ft);
  if (!raOwnFailed) {
    out.ra_computation_data = bus.ra_own_bus;
    cond[raOppFailed ? CondRaOwnOnly : CondRaBothOk] = true;
  } else if (!raOppFailed) {
    out.ra_computation_data = bus.ra_opp_bus;
    cond[CondRaOppOnly] = true;
  } else {
    out.ra_computation_data.radio_height_ft.SSM = bus.ra_own_bus.radio_height_ft.SSM;
    out.ra_computation_data.radio_height_ft.Data = kRaBothFailedFt;
    cond[CondRaNone] = true;
  }
  out.dual_ra_failure = raOwnFailed && raOppFailed;
  out.both_ra_valid = isNormal(bus.ra_own_bus.radio_height_ft) && isNormal(bus.ra_opp_bus.radio_height_ft);
  // Consumers below read the raw value, whatever its SSM (NCD/FT data are used as is; 2500 ft when both failed).
  const float radioHeightFt = rawValue(out.ra_computation_data.radio_height_ft);

  // --- FAC (1101–1213): the own FAC's bus if the own FAC is healthy, else the opposite one's (healthy or not).
  out.chosen_fac_bus = di.fac_own_healthy ? bus.fac_own_bus : bus.fac_opp_bus;
  cond[di.fac_own_healthy ? CondFacSelOwn : CondFacSelOpp] = true;

  // Rudder control (1215–1241), from both FACs' discrete words 2 (bits read whatever the SSM or the FAC health):
  // failed unless a yaw damper and a rudder trim are engaged (in either FAC) and a FAC is healthy. "Both valid" needs
  // both FACs healthy, but still only one yaw damper and one rudder trim (the bits are ORed across the FACs).
  const bool yawDamper = bitSet(bus.fac_own_bus.discrete_word_2, kFacWord2YawDamperOwn) ||
                         bitSet(bus.fac_opp_bus.discrete_word_2, kFacWord2YawDamperOwn);
  const bool rudderTrim = bitSet(bus.fac_own_bus.discrete_word_2, kFacWord2RudderTrimOwn) ||
                          bitSet(bus.fac_opp_bus.discrete_word_2, kFacWord2RudderTrimOwn);
  const bool bothFacsUnhealthy = !di.fac_own_healthy && !di.fac_opp_healthy;
  out.fac_rudder_control_failure = !yawDamper || !rudderTrim || !kRudderControlEnabled || bothFacsUnhealthy;
  out.both_fac_rudder_valid = di.fac_opp_healthy && di.fac_own_healthy && yawDamper && rudderTrim;
  cond[CondFacRudderNoYawDamper] = !yawDamper;
  cond[CondFacRudderNoTrim] = !rudderTrim;
  cond[CondFacRudderBothUnhealthy] = bothFacsUnhealthy;

  // Speeds and weights (1242–1262): a word reporting a failure (NCD counts as valid).
  const base_fac_bus& fac = out.chosen_fac_bus;
  out.fac_speeds_failure = failed(fac.v_max_kn) || failed(fac.v_ls_kn);
  out.fac_weights_failure = failed(fac.v_man_kn) || failed(fac.center_of_gravity_pos_percent) ||
                            failed(fac.total_weight_lbs);

  // Landing gear (1263–1279), FAC discrete word 5: on ground with both main gears compressed, LGCIU data available
  // and the word not failed; "ground with engines stopped, or in flight for 5 s".
  const base_arinc_429& word5 = fac.discrete_word_5;
  const bool allLgciuLost = bitSet(word5, kFacWord5AllLgciuLost);
  out.on_ground = bitSet(word5, kFacWord5LeftMainGear) && bitSet(word5, kFacWord5RightMainGear) && !allLgciuLost &&
                  !failed(word5);
  out.fac_lg_data_failure = failed(word5) || allLgciuLost;
  const bool inFlight5s = s.inFlight5s.update(!out.on_ground, dt, true, kInFlightConfirmS);
  const bool groundEnginesStopped = di.eng_opp_stop && di.eng_own_stop && out.on_ground;
  out.gnd_eng_stop_flt_5s = groundEnginesStopped || inFlight5s;
  cond[CondOnGround] = out.on_ground;
  cond[CondInFlight5s] = inFlight5s;
  cond[CondGroundEnginesStopped] = groundEnginesStopped;

  // Flap lever (1280–1302): the first of bits 11..15 set gives positions 1..5; none set: 0. Flap/slat data (1304–1307).
  out.flap_slat_lever_position = 0;
  for (int pos = 1; pos <= 5; pos++) {
    if (bitSet(word5, kFacWord5FlapLever1 + pos - 1)) {
      out.flap_slat_lever_position = static_cast<int8_T>(pos);
      break;
    }
  }
  cond[CondFlapLever0 + out.flap_slat_lever_position] = true;
  out.fac_flap_slat_data_failure = bitSet(word5, kFacWord5AllSfccLost) || failed(word5);

  // --- Engines and FCU (1308–1313)
  out.one_engine_out = di.eng_opp_stop != di.eng_own_stop;
  out.engine_running = !di.eng_opp_stop || !di.eng_own_stop;
  out.fcu_failure = !di.fcu_opp_healthy && !di.fcu_own_healthy;

  // --- ILS (1314–1346): a receiver is valid when neither LOC nor G/S deviation reports a failure. The own ILS is used
  // unless invalid, then the opposite one (valid or not). The LOC deviation changes sign for a back beam (FMS).
  const bool ilsOppOk = !failed(bus.ils_opp_bus.glideslope_deviation_deg) &&
                        !failed(bus.ils_opp_bus.localizer_deviation_deg);
  const bool ilsOwnOk = !failed(bus.ils_own_bus.glideslope_deviation_deg) &&
                        !failed(bus.ils_own_bus.localizer_deviation_deg);
  out.ils_failure = !ilsOppOk && !ilsOwnOk;
  out.both_ils_valid = ilsOppOk && ilsOwnOk;
  out.ils_computation_data = ilsOwnOk ? bus.ils_own_bus : bus.ils_opp_bus;
  if (in.in.fms_inputs.backbeam_selected) {
    out.ils_computation_data.localizer_deviation_deg.Data *= kBackbeamGain;
  }
  cond[CondIlsOwnValid] = ilsOwnOk;
  cond[CondIlsOppValid] = ilsOppOk;
  cond[CondIlsSelOpp] = !ilsOwnOk;
  cond[CondLocBackbeam] = in.in.fms_inputs.backbeam_selected;

  // --- TCAS (1350–1364): mode word not failed, TA/RA selected (not STBY), above 900 ft RA.
  const base_arinc_429& tcasWord = bus.tcas_bus.sensitivity_level;
  const bool tcasTaRa = !failed(tcasWord) && !bitSet(tcasWord, kTcasWordStandby) && bitSet(tcasWord, kTcasWordTa) &&
                        bitSet(tcasWord, kTcasWordTaRa);
  out.tcas_mode_available = tcasTaRa && radioHeightFt > kTcasMinRaFt;
  out.tcas_failure = failed(tcasWord);
  cond[CondTcasAvailable] = out.tcas_mode_available;
  cond[CondTcasRaInhibit] = tcasTaRa && !out.tcas_mode_available;

  // --- ILS tune inhibit and runway heading memory (1365–1379), from the previous frame's modes: RWY LOC, LAND, or
  // LAND armed at or below 700 ft RA. Otherwise the memory follows the selected ILS runway heading (0 if not normal).
  const bool landArmedLow = last.armed_modes.land_armed && radioHeightFt <= kTuneInhibitRaFt;
  out.ils_tune_inhibit = last.lateral_modes.rwy_loc_submode_active || last.lateral_modes.land_active || landArmedLow;
  if (!out.ils_tune_inhibit) {
    s.rwyHdgMemo = valueOr(out.ils_computation_data.runway_heading_deg, kRwyHdgDefaultDeg);
  }
  out.rwy_hdg_memo = s.rwyHdgMemo;
  cond[CondTuneInhibitRwyLoc] = last.lateral_modes.rwy_loc_submode_active;
  cond[CondTuneInhibitLand] = last.lateral_modes.land_active;
  cond[CondTuneInhibitLandArmedLow] = landArmedLow;
  cond[CondRwyHdgMemoUpdated] = !out.ils_tune_inhibit;
}

}  // namespace fmgc::readable::conditioning
