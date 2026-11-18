// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's input conditioning (generated FacComputer.cpp 426-665). See sensors.h.

#include "sensors.h"

namespace fac::readable {

namespace {

namespace a429 = blocks::a429;

/** The configuration bits of the SFCC slat/flap system status word (BitfromLabel2..7, 494-505). */
constexpr int kSfccConf0Bit = 17, kSfccConf1Bit = 18, kSfccConf2Bit = 19, kSfccConf3Bit = 20, kSfccConfFullBit = 21;
constexpr int kSfccConf1FlapsBit = 26;  // with bit 18: CONF 1 (slats only); without: CONF 1+F
/** The SFCC actual position word bit compared with the slat-extended discrete (BitfromLabel_bit_m). */
constexpr int kSfccSlatBit = 12;
/** LGCIU discrete word 2: nose gear and left/right main gear compressed; word 3: left/right main gear downlocked. */
constexpr int kLgciuNoseBit = 12, kLgciuLeftMainBit = 13, kLgciuRightMainBit = 14;
constexpr int kLgciuGearDownBitA = 11, kLgciuGearDownBitB = 12;
/** The opposite FAC's discrete word 5 (see output_packing.cpp): CONF 1..FULL bits 11-15, LGCIU own valid 16, left and
 *  right main gear 18, 19, gear down 20, SFCC own valid 21. */
constexpr int kFacConf1Bit = 11, kFacLgciuValidBit = 16, kFacLeftMainBit = 18, kFacRightMainBit = 19,
              kFacGearDownBit = 20, kFacSfccValidBit = 21;

bool bit(const base_arinc_429& w, int n) { return a429::bit(w, n) != 0U; }

/** A flap or slat FPPU angle -> surface angle (deg) tables (FlapFPPUtoSurfaceAngle, SlatFPPUtoSurfaceAngle). */
constexpr double kFlapFppuDeg[12] = {0.0,    35.66,  69.32,  89.7,   105.29, 120.23,
                                     145.62, 168.35, 189.87, 210.69, 231.25, 251.97};
constexpr double kFlapSurfaceDeg[12] = {0.0, 0.0, 2.5, 5.0, 7.5, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0};
constexpr double kSlatFppuDeg[5] = {0.0, 66.83, 167.08, 272.27, 334.16};
constexpr double kSlatSurfaceDeg[5] = {0.0, 5.4, 13.5, 22.0, 27.0};

}  // namespace

void InputConditioning::reset() {
  noseGearAgree_.reset();
  sfccAgree_.reset();
  raOwn_.reset();
  raOpp_.reset();
  onGround_.reset();
}

void InputConditioning::updateGear(const fac_inputs& in) {
  const base_lgciu_bus& lgciu = in.bus_inputs.lgciu_own_bus;
  const base_arinc_429& oppWord5 = in.bus_inputs.fac_opp_bus.discrete_word_5;
  // 426-431: the own LGCIU is valid while its word 1 is not failed and its nose gear bit has not disagreed with the
  // nose gear discrete for 1 s (a falling-edge confirm: valid again as soon as they agree)
  const bool noseAgrees = in.discrete_inputs.nose_gear_pressed == bit(lgciu.discrete_word_2, kLgciuNoseBit);
  const bool agreed = noseGearAgree_.update(noseAgrees, in.time.dt, false, kLgciuDisagreeSeconds);
  const bool ownValid = a429::isNotFailed(lgciu.discrete_word_1) && agreed;
  // 441-466: the opposite FAC's copy is usable when it reports its own LGCIU valid (and the word is not failed)
  const bool oppValid = bit(oppWord5, kFacLgciuValidBit) && a429::isNotFailed(oppWord5);
  GearState& g = s_.gear;
  if (ownValid) {
    g.leftMainPressed = bit(lgciu.discrete_word_2, kLgciuLeftMainBit);
    g.rightMainPressed = bit(lgciu.discrete_word_2, kLgciuRightMainBit);
    g.mainGearOut = bit(lgciu.discrete_word_3, kLgciuGearDownBitA) || bit(lgciu.discrete_word_3, kLgciuGearDownBitB);
  } else if (oppValid) {
    g.leftMainPressed = bit(oppWord5, kFacLeftMainBit);
    g.rightMainPressed = bit(oppWord5, kFacRightMainBit);
    g.mainGearOut = bit(oppWord5, kFacGearDownBit);
  } else {
    g.leftMainPressed = g.rightMainPressed = g.mainGearOut = false;
  }
  g.lgciuOwnValid = ownValid;
  g.allLgciuLost = !ownValid && !oppValid;
}

void InputConditioning::updateSlatFlap(const fac_inputs& in) {
  const base_sfcc_bus& sfcc = in.bus_inputs.sfcc_own_bus;
  const base_fac_bus& opp = in.bus_inputs.fac_opp_bus;
  const base_arinc_429& status = sfcc.slat_flap_system_status_word;
  // 486-493: own SFCC valid while its status word is not failed and the slat-extended discrete has not agreed with the
  // actual position word bit 12 for 1 s. The wrapper derives the discrete as the inverse of that bit (when the word is
  // normal), so with a working SFCC they always differ.
  const bool discreteDiffers = in.discrete_inputs.slats_extended != bit(sfcc.slat_flap_actual_position_word, kSfccSlatBit);
  const bool consistent = sfccAgree_.update(discreteDiffers, in.time.dt, false, kSfccDisagreeSeconds);
  const bool ownValid = a429::isNotFailed(status) && consistent;
  const bool oppValid = bit(opp.discrete_word_5, kFacSfccValidBit) && a429::isNotFailed(opp.discrete_word_5);

  SlatFlapState& sf = s_.slatFlap;
  if (ownValid) {
    // 506-525: the configuration from the status word bits (first match wins)
    if (bit(status, kSfccConf0Bit)) {
      sf.configIndex = 0.0F;
    } else if (bit(status, kSfccConf1Bit) && bit(status, kSfccConf1FlapsBit)) {
      sf.configIndex = 1.0F;
    } else if (bit(status, kSfccConf1Bit) && !bit(status, kSfccConf1FlapsBit)) {
      sf.configIndex = 2.0F;
    } else if (bit(status, kSfccConf2Bit)) {
      sf.configIndex = 3.0F;
    } else if (bit(status, kSfccConf3Bit)) {
      sf.configIndex = 4.0F;
    } else if (bit(status, kSfccConfFullBit)) {
      sf.configIndex = 5.0F;
    } else {
      sf.configIndex = 0.0F;
    }
    sf.flapFppuDeg = sfcc.flap_actual_position_deg.Data;
    sf.slatFppuDeg = sfcc.slat_actual_position_deg.Data;
    sf.actualPositionWord = sfcc.slat_flap_actual_position_word.Data;
  } else if (oppValid) {
    // 526-544: the opposite FAC's configuration bits (CONF 1 .. FULL) and angles
    sf.configIndex = 0.0F;
    for (int k = 0; k < 5; k++) {
      if (bit(opp.discrete_word_5, kFacConf1Bit + k)) {
        sf.configIndex = static_cast<float>(k + 1);
        break;
      }
    }
    sf.flapFppuDeg = opp.fac_flap_angle_deg.Data;
    sf.slatFppuDeg = opp.fac_slat_angle_deg.Data;
    sf.actualPositionWord = opp.discrete_word_1.Data;
  } else if (s_.gear.mainGearOut) {
    // 544-555: no SFCC information with the gear down: assume CONF FULL
    sf.configIndex = kGearDownConfigIndex;
    sf.flapFppuDeg = kGearDownFlapAngle;
    sf.slatFppuDeg = kGearDownSlatAngle;
    sf.actualPositionWord = 0.0F;
  } else {
    sf.configIndex = sf.flapFppuDeg = sf.slatFppuDeg = sf.actualPositionWord = 0.0F;
  }
  sf.flapSurfaceDeg = blocks::lookup1(static_cast<double>(sf.flapFppuDeg), kFlapFppuDeg, kFlapSurfaceDeg);
  sf.slatSurfaceDeg = blocks::lookup1(static_cast<double>(sf.slatFppuDeg), kSlatFppuDeg, kSlatSurfaceDeg);
  sf.sfccOwnValid = ownValid;
  sf.allSfccLost = !ownValid && !oppValid;
}

void InputConditioning::updateAirData(const base_fac_bus_inputs& b) {
  // 558-600: an ADR is invalid when its CAS or AoA word reports a failure (NCD counts as valid)
  auto invalid = [](const base_adr_bus& a) {
    return !a429::isNotFailed(a.airspeed_computed_kn) || !a429::isNotFailed(a.aoa_corrected_deg);
  };
  AirData& d = s_.adr;
  d.ownInvalid = invalid(b.adr_own_bus);
  d.oppInvalid = invalid(b.adr_opp_bus);
  d.adr3Invalid = invalid(b.adr_3_bus);
  const base_adr_bus* a = !d.ownInvalid ? &b.adr_own_bus : (!d.adr3Invalid ? &b.adr_3_bus : nullptr);
  if (a) {
    d.casKn = a->airspeed_computed_kn.Data;
    d.tasKn = a->airspeed_true_kn.Data;
    d.mach = a->mach.Data;
    d.alphaDeg = a->aoa_corrected_deg.Data;
    d.staticPressureHpa = a->corrected_average_static_pressure.Data;
    d.altitudeFt = a->altitude_standard_ft.Data;
  } else {
    d.casKn = d.tasKn = d.mach = d.alphaDeg = d.staticPressureHpa = d.altitudeFt = 0.0F;
  }
}

void InputConditioning::updateInertialData(const base_fac_bus_inputs& b) {
  // 567-638: an IR is invalid unless its body yaw rate and lateral acceleration words are both normal
  auto invalid = [](const base_ir_bus& r) {
    return !a429::isNormal(r.body_yaw_rate_deg_s) || !a429::isNormal(r.body_lat_accel_g);
  };
  InertialData& d = s_.ir;
  d.ownInvalid = invalid(b.ir_own_bus);
  d.oppInvalid = invalid(b.ir_opp_bus);
  d.ir3Invalid = invalid(b.ir_3_bus);
  const base_ir_bus* r = !d.ownInvalid ? &b.ir_own_bus : (!d.ir3Invalid ? &b.ir_3_bus : nullptr);
  if (r) {
    d.thetaDeg = r->pitch_angle_deg.Data;
    d.phiDeg = r->roll_angle_deg.Data;
    d.qDegS = r->body_pitch_rate_deg_s.Data;
    d.rDegS = r->body_yaw_rate_deg_s.Data;
    d.nxG = r->body_long_accel_g.Data;
    d.nyG = r->body_lat_accel_g.Data;
    d.nzG = r->body_normal_accel_g.Data;
    d.thetaDotDegS = r->pitch_att_rate_deg_s.Data;
    d.phiDotDegS = r->roll_att_rate_deg_s.Data;
    d.groundSpeedKn = r->ground_speed_kn.Data;
    d.verticalSpeed = r->inertial_vertical_speed_ft_s.Data;
  } else {
    d.thetaDeg = d.phiDeg = d.qDegS = d.rDegS = d.nxG = d.nyG = d.nzG = d.thetaDotDegS = d.phiDotDegS =
        d.groundSpeedKn = d.verticalSpeed = 0.0F;
  }
}

const Sensors& InputConditioning::update(const fac_inputs& in) {
  const base_fac_bus_inputs& b = in.bus_inputs;
  const base_fac_discrete_inputs& d = in.discrete_inputs;
  updateGear(in);
  updateSlatFlap(in);
  updateAirData(b);
  updateInertialData(b);

  // 640-655: radio height from the own FMGC unless its word has been failed for 0.6 s (a falling-edge confirm: back
  // as soon as it is not failed), else from the opposite FMGC (whatever its state). Both failed for 0.6 s: all RA
  // failed (the opposite FMGC's word is still passed on).
  const bool ownRa = raOwn_.update(a429::isNotFailed(b.fmgc_own_bus.fg_radio_height_ft), in.time.dt, false,
                                   kRaFailConfirmSeconds);
  const bool oppRa = raOpp_.update(a429::isNotFailed(b.fmgc_opp_bus.fg_radio_height_ft), in.time.dt, false,
                                   kRaFailConfirmSeconds);
  s_.radioHeight = ownRa ? b.fmgc_own_bus.fg_radio_height_ft : b.fmgc_opp_bus.fg_radio_height_ft;
  s_.allRaFailed = !ownRa && !oppRa;

  // 656-662: the own FMGC is followed when its AP is engaged (unless both APs are and this is FAC 2), or with no AP
  // engaged when its FM weight and CG words are normal
  s_.anyApEngaged = d.ap_own_engaged || d.ap_opp_engaged;
  const bool ownWeightAndCg = a429::isNormal(b.fmgc_own_bus.fm_weight_lbs) && a429::isNormal(b.fmgc_own_bus.fm_cg_percent);
  s_.fmgcOwnSelected = ((!s_.anyApEngaged && ownWeightAndCg) || d.ap_own_engaged) &&
                       (!d.ap_opp_engaged || !d.ap_own_engaged || d.is_unit_1);
  s_.fmgc = s_.fmgcOwnSelected ? &b.fmgc_own_bus : &b.fmgc_opp_bus;

  // 663-665: on ground once either main gear has been compressed for 0.5 s (in flight as soon as both are not)
  s_.onGround = onGround_.update(s_.gear.leftMainPressed || s_.gear.rightMainPressed, in.time.dt, true,
                                 kOnGroundConfirmSeconds);
  return s_;
}


}  // namespace fac::readable
