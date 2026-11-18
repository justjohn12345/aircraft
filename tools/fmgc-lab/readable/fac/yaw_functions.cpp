// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's yaw damper, rudder trim and rudder travel limiter (generated FacComputer.cpp 666-681, 1095-1219). See
// yaw_functions.h.

#include "yaw_functions.h"

namespace fac::readable {

namespace {

namespace a429 = blocks::a429;

/** ELAC discrete status word 1 bits read by the FAC (BitfromLabel_bit_c/_j/..., all 20 or 26). */
constexpr int kElacYawDamperBit = 20;  // with bit 26: the FAC takes the ELAC's yaw damper order
constexpr int kElacNormalLawBit = 26;

/** Rudder travel limit (deg) against CAS (kt) (uDLookupTable_tableData_j / _bp01Data_i). */
constexpr double kTravelLimitCasKn[7] = {100.0, 160.0, 200.0, 240.0, 320.0, 380.0, 400.0};
constexpr double kTravelLimitDeg[7] = {25.0, 25.0, 14.5, 8.8, 4.8, 3.4, 3.4};

bool bit(const base_arinc_429& w, int n) { return a429::bit(w, n) != 0U; }

}  // namespace

YawEngagement YawEngagement::compute(const base_fac_discrete_inputs& d) {
  // 666-681: each function engages when it can and this FAC has priority (FAC 1 always; FAC 2 when FAC 1's function
  // is not engaged)
  YawEngagement e{};
  e.yawDamperCanEngage = d.yaw_damper_has_hyd_press && d.fac_engaged_from_switch;
  e.yawDamperHasPriority = d.is_unit_1 || !d.yaw_damper_opp_engaged;
  e.yawDamperEngaged = e.yawDamperCanEngage && e.yawDamperHasPriority;
  e.rudderTrimCanEngage = d.rudder_trim_actuator_healthy && d.fac_engaged_from_switch;
  e.rudderTrimHasPriority = d.is_unit_1 || !d.rudder_trim_opp_engaged;
  e.rudderTrimEngaged = e.rudderTrimCanEngage && e.rudderTrimHasPriority;
  e.travelLimCanEngage = d.rudder_travel_lim_actuator_healthy && d.fac_engaged_from_switch;
  e.travelLimHasPriority = d.is_unit_1 || !d.rudder_travel_lim_opp_engaged;
  e.travelLimEngaged = e.travelLimCanEngage && e.travelLimHasPriority;
  return e;
}

void RudderTrim::reset() {
  restart();
  integrator_ = 0.0;
  resetButton_.previousInput = false;
}

void RudderTrim::restart() {
  resetLatch_.q = false;
  reload_ = true;
  resetButton_.reset();
  order_.reset();
}

double RudderTrim::update(const fac_inputs& in, const Sensors& s, bool engaged) {
  const base_fac_discrete_inputs& d = in.discrete_inputs;
  const base_fac_bus_inputs& b = in.bus_inputs;
  // 1095-1112: the RESET pushbutton (ignored with an AP engaged) latches a trim reset, cleared by either trim switch
  // or an AP engagement (clear wins)
  const bool resetPressed = resetButton_.update(d.rudder_trim_reset_button && !s.anyApEngaged, true);
  resetLatch_.update(resetPressed, d.rudder_trim_switch_left || d.rudder_trim_switch_right || s.anyApEngaged);

  // 1113-1146: trim rate. With an AP engaged: above 200 ft (the own FMGC's radio height value, whatever its SSM) the
  // ELAC yaw damper order x 1/15 (the first ELAC in normal law, healthy), limited to 1.7 deg/s; below, 0. Without: the
  // switches, left (positive) first.
  double rate;
  if (s.anyApEngaged) {
    if (b.fmgc_own_bus.fg_radio_height_ft.Data >= static_cast<float>(kAutoTrimMinRadioHeightFt)) {
      double yawDamperOrder;
      if (bit(b.elac_1_bus.discrete_status_word_1, kElacNormalLawBit) && d.elac_1_healthy) {
        yawDamperOrder = b.elac_1_bus.yaw_damper_command_deg.Data;
      } else if (bit(b.elac_2_bus.discrete_status_word_1, kElacNormalLawBit) && d.elac_2_healthy) {
        yawDamperOrder = b.elac_2_bus.yaw_damper_command_deg.Data;
      } else {
        yawDamperOrder = 0.0;
      }
      rate = kAutoTrimGain * yawDamperOrder;
    } else {
      rate = 0.0;
    }
    rate = blocks::limit(rate, -kAutoTrimRateLimitDegS, kAutoTrimRateLimitDegS);
  } else if (d.rudder_trim_switch_left) {
    rate = kTrimSwitchRateDegS;
  } else if (d.rudder_trim_switch_right) {
    rate = -kTrimSwitchRateDegS;
  } else {
    rate = 0.0;
  }
  const double increment = 1.0 * rate * in.time.dt;

  // 1147-1165: the integrator is reloaded while a reset is latched or the trim is not engaged (to the measured
  // position) and on the first frame after a restart; when engaged that reload value is 0 (a reset trims back to 0,
  // and so does a FAC restart). Limited to +-20 deg.
  reload_ = resetLatch_.q || !engaged || reload_;
  if (reload_) {
    integrator_ = (!engaged ? in.analog_inputs.rudder_trim_position_deg : 0.0) - increment;
  }
  integrator_ = blocks::limit(integrator_ + increment, -kTrimRangeDeg, kTrimRangeDeg);
  reload_ = false;  // icLoad is cleared at the end of the frame (1562); nothing reads it in between

  // 1166-1169: the order moves to the integrator at 2 deg/s; not engaged: the measured position
  return order_.update(integrator_, kTrimOrderRateDegS, kTrimOrderRateDegS, in.time.dt,
                       in.analog_inputs.rudder_trim_position_deg, !engaged);
}

void YawDamper::reset() {
  washout_.reset();
  order_.reset();
}

double YawDamper::update(const fac_inputs& in, const Sensors& s, bool engaged) {
  const base_fac_discrete_inputs& d = in.discrete_inputs;
  const base_fac_bus_inputs& b = in.bus_inputs;
  // 1170-1202: the ELAC's yaw damper order when an ELAC with bits 20 and 26 is healthy (ELAC 1 first), else the
  // alternate law: the washed-out yaw rate (high pass, 20 rad/s), gain 1, +-5 deg. The washout runs every frame.
  const bool elac1 = bit(b.elac_1_bus.discrete_status_word_1, kElacYawDamperBit) && d.elac_1_healthy &&
                     bit(b.elac_1_bus.discrete_status_word_1, kElacNormalLawBit);
  const bool elac2 = bit(b.elac_2_bus.discrete_status_word_1, kElacYawDamperBit) && d.elac_2_healthy &&
                     bit(b.elac_2_bus.discrete_status_word_1, kElacNormalLawBit);
  const double washedOutRate = washout_.update(static_cast<double>(s.ir.rDegS), kYawDamperWashoutC1, in.time.dt);
  double order;
  if (elac1 || elac2) {
    order = elac1 ? b.elac_1_bus.yaw_damper_command_deg.Data : b.elac_2_bus.yaw_damper_command_deg.Data;
  } else {
    order = blocks::limit(1.0 * washedOutRate, -kAlternateYawDamperLimitDeg, kAlternateYawDamperLimitDeg);
  }
  order = blocks::limit(order, -kYawDamperLimitDeg, kYawDamperLimitDeg);
  // 1203-1206: 40 deg/s; not engaged: the measured position
  return order_.update(order, kYawDamperRateDegS, kYawDamperRateDegS, in.time.dt,
                       in.analog_inputs.yaw_damper_position_deg, !engaged);
}

void TravelLimiter::reset() { order_.reset(); }

double TravelLimiter::update(const fac_inputs& in, const Sensors& s, bool engaged) {
  // 1207-1219: the travel limit for the CAS (ADR), 0..30 deg, at 1 deg/s; not engaged: the measured position
  const double limit = blocks::limit(blocks::lookup1(static_cast<double>(s.adr.casKn), kTravelLimitCasKn, kTravelLimitDeg),
                                     0.0, kTravelLimitMaxDeg);
  return order_.update(limit, kTravelLimitRateDegS, kTravelLimitRateDegS, in.time.dt,
                       in.analog_inputs.rudder_travel_lim_position_deg, !engaged);
}

}  // namespace fac::readable
