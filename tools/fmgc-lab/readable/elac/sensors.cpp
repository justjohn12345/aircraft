// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ELAC sensor conditioning (generated ElacComputer.cpp 428-801): ADR and IR voting, radio height, on ground, the
// flight / ground chart, hydraulic supply. Parameters: docs/params-ElacComputer.txt.

#include <cmath>

#include "elac.h"

namespace elac::readable {

namespace {

using blocks::a429::isNormal;

constexpr uint32_t kFw = static_cast<uint32_t>(SignStatusMatrix::FailureWarning);

bool adrInvalid(const base_adr_bus& b) {
  return b.mach.SSM == kFw || b.airspeed_computed_kn.SSM == kFw || b.airspeed_true_kn.SSM == kFw ||
         b.aoa_corrected_deg.SSM == kFw;
}

/** Note: the body pitch rate (q) is used but not monitored. */
bool irInvalid(const base_ir_bus& b) {
  return !isNormal(b.pitch_angle_deg) || !isNormal(b.roll_angle_deg) || !isNormal(b.body_yaw_rate_deg_s) ||
         !isNormal(b.body_long_accel_g) || !isNormal(b.body_lat_accel_g) || !isNormal(b.body_normal_accel_g) ||
         !isNormal(b.pitch_att_rate_deg_s) || !isNormal(b.roll_att_rate_deg_s);
}

/** Float average of the same word of two buses (`(a + b) / 2.0F`). */
template <class Bus>
float avg(const Bus& a, const Bus& b, base_arinc_429 Bus::*w) {
  return ((a.*w).Data + (b.*w).Data) / 2.0F;
}

}  // namespace

void ElacComputer::voteAirData(Frame& f) {
  const base_elac_bus_inputs& b = f.in->bus_inputs;
  const bool v1 = !adrInvalid(b.adr_1_bus), v2 = !adrInvalid(b.adr_2_bus), v3 = !adrInvalid(b.adr_3_bus);
  f.doubleAdrFault = (!v1 && !v2) || (!v1 && !v3) || (!v2 && !v3);
  f.tripleAdrFault = !v1 && !v2 && !v3;

  // the average of the two first valid ADRs, else the only valid one, else 0 (489-543)
  const base_adr_bus *x = nullptr, *y = nullptr;
  if (!v1 && v2 && v3) {
    x = &b.adr_2_bus, y = &b.adr_3_bus;
  } else if (v1 && !v2 && v3) {
    x = &b.adr_1_bus, y = &b.adr_3_bus;
  } else if (v1 && v2) {
    x = &b.adr_1_bus, y = &b.adr_2_bus;
  } else if (v1) {
    x = &b.adr_1_bus;
  } else if (v2) {
    x = &b.adr_2_bus;
  } else if (v3) {
    x = &b.adr_3_bus;
  }
  if (x && y) {
    f.vIas = avg(*x, *y, &base_adr_bus::airspeed_computed_kn);
    f.vTas = avg(*x, *y, &base_adr_bus::airspeed_true_kn);
    f.mach = avg(*x, *y, &base_adr_bus::mach);
    f.alpha = avg(*x, *y, &base_adr_bus::aoa_corrected_deg);
  } else if (x) {
    f.vIas = x->airspeed_computed_kn.Data;
    f.vTas = x->airspeed_true_kn.Data;
    f.mach = x->mach.Data;
    f.alpha = x->aoa_corrected_deg.Data;
  } else {
    f.vIas = f.vTas = f.mach = f.alpha = 0.0F;
  }
  f.alphaFiltered = sensors_.alphaFilter.update(static_cast<double>(f.alpha), 0.5, f.in->time.dt);  // (545-546)
}

void ElacComputer::voteInertialData(Frame& f) {
  const base_elac_bus_inputs& b = f.in->bus_inputs;
  const bool v1 = !irInvalid(b.ir_1_bus), v2 = !irInvalid(b.ir_2_bus), v3 = !irInvalid(b.ir_3_bus);
  f.doubleIrFault = (!v1 && !v2) || (!v1 && !v3) || (!v2 && !v3);
  f.tripleIrFault = !v1 && !v2 && !v3;

  // as the ADRs, but the pair order is 2+3, 1+2, 1+3 and the single order 1, 3, 2 (553-657)
  const base_ir_bus *x = nullptr, *y = nullptr;
  if (!v1 && v2 && v3) {
    x = &b.ir_2_bus, y = &b.ir_3_bus;
  } else if (v1 && v2 && !v3) {
    x = &b.ir_1_bus, y = &b.ir_2_bus;
  } else if (v1 && v3) {
    x = &b.ir_1_bus, y = &b.ir_3_bus;
  } else if (v1) {
    x = &b.ir_1_bus;
  } else if (v3) {
    x = &b.ir_3_bus;
  } else if (v2) {
    x = &b.ir_2_bus;
  }
  if (x && y) {
    f.theta = avg(*x, *y, &base_ir_bus::pitch_angle_deg);
    f.phi = avg(*x, *y, &base_ir_bus::roll_angle_deg);
    f.q = avg(*x, *y, &base_ir_bus::body_pitch_rate_deg_s);
    f.r = avg(*x, *y, &base_ir_bus::body_yaw_rate_deg_s);
    f.nx = avg(*x, *y, &base_ir_bus::body_long_accel_g);
    f.ny = avg(*x, *y, &base_ir_bus::body_lat_accel_g);
    f.nz = avg(*x, *y, &base_ir_bus::body_normal_accel_g);
    f.thetaDot = avg(*x, *y, &base_ir_bus::pitch_att_rate_deg_s);
    f.phiDot = avg(*x, *y, &base_ir_bus::roll_att_rate_deg_s);
  } else if (x) {
    f.theta = x->pitch_angle_deg.Data;
    f.phi = x->roll_angle_deg.Data;
    f.q = x->body_pitch_rate_deg_s.Data;
    f.r = x->body_yaw_rate_deg_s.Data;
    f.nx = x->body_long_accel_g.Data;
    f.ny = x->body_lat_accel_g.Data;
    f.nz = x->body_normal_accel_g.Data;
    f.thetaDot = x->pitch_att_rate_deg_s.Data;
    f.phiDot = x->roll_att_rate_deg_s.Data;
  } else {
    f.theta = f.phi = f.q = f.r = f.nx = f.ny = f.nz = f.thetaDot = f.phiDot = 0.0F;
  }
}

void ElacComputer::updateRadioHeight(Frame& f) {
  const elac_inputs& in = *f.in;
  const base_arinc_429& ra1 = in.bus_inputs.ra_1_bus.radio_height_ft;
  const base_arinc_429& ra2 = in.bus_inputs.ra_2_bus.radio_height_ft;
  const double dt = in.time.dt;
  const double now = in.time.simulation_time;

  // SFCC actual position word bit 23 (664-671): flaps in FULL (the time since it was last clear is configFullEventTime)
  const bool flapsFull1 = blocks::a429::bit(in.bus_inputs.sfcc_1_bus.slat_flap_actual_position_word, 23) != 0 &&
                          blocks::a429::isNotFailed(in.bus_inputs.sfcc_1_bus.slat_flap_actual_position_word);
  const bool flapsFull2 = blocks::a429::bit(in.bus_inputs.sfcc_2_bus.slat_flap_actual_position_word, 23) != 0 &&
                          blocks::a429::isNotFailed(in.bus_inputs.sfcc_2_bus.slat_flap_actual_position_word);

  // the two RAs disagree by more than 50 ft for 1 s (672-675)
  const bool disagree = sensors_.raDisagree.update(std::abs(ra1.Data - ra2.Data) > 50.0F, dt, true, 1.0);

  // coherence monitor (676-693): an RA above 50 ft at more than 200 kt CAS is rejected for good, but only with all
  // three ADRs failed, when the CAS is 0: never (finding L1)
  const bool raMonitorAdrCondition = f.tripleAdrFault;
  const bool implausible1 = sensors_.ra1Implausible.update(
      ra1.Data > 50.0F && isNormal(ra1) && f.vIas > 200.0F && raMonitorAdrCondition, dt, true, 1.0);
  const bool implausible2 = sensors_.ra2Implausible.update(
      ra2.Data > 50.0F && isNormal(ra2) && f.vIas > 200.0F && raMonitorAdrCondition, dt, true, 1.0);
  sensors_.ra1Rejected = implausible1 || sensors_.ra1Rejected;
  sensors_.ra2Rejected = implausible2 || sensors_.ra2Rejected;
  const bool ra1Invalid = ra1.SSM == kFw || sensors_.ra1Rejected;
  const bool ra2Invalid = ra2.SSM == kFw || sensors_.ra2Rejected;

  if (!sensors_.flapsFullTimeSet) {
    sensors_.flapsFullTime = now;
    sensors_.flapsFullTimeSet = true;
  }
  if (!flapsFull1 && !flapsFull2) sensors_.flapsFullTime = now;

  // the value (703-727): the average of both; while they disagree, the lower one once in FULL for 10 s, else 250 ft;
  // one RA: that one (250 ft above 180 kt with all ADRs failed: unreachable, L1); none: 250 ft
  if (!ra1Invalid && !ra2Invalid) {
    if (disagree) {
      f.radioHeight = now > sensors_.flapsFullTime + 10.0 ? std::fmin(ra1.Data, ra2.Data) : 250.0F;
    } else {
      f.radioHeight = (ra1.Data + ra2.Data) / 2.0F;
    }
  } else if (ra1Invalid != ra2Invalid) {
    if (f.vIas > 180.0F && raMonitorAdrCondition) {
      f.radioHeight = 250.0F;
    } else {
      f.radioHeight = ra2Invalid ? ra1.Data : ra2.Data;
    }
  } else {
    f.radioHeight = 250.0F;
  }
  f.dualRaFailure = ra1Invalid && ra2Invalid;
}

void ElacComputer::updateOnGround(Frame& f) {
  // (730-740): both LGCIUs report both main gears compressed, or one does below 50 ft RA, or both ground spoiler
  // discretes below 50 ft RA
  const base_elac_discrete_inputs& d = f.in->discrete_inputs;
  const bool below50ft = f.radioHeight < 50.0 && !f.dualRaFailure;
  const bool lgciu1 = d.lgciu_1_left_main_gear_pressed && d.lgciu_1_right_main_gear_pressed;
  const bool lgciu2 = d.lgciu_2_left_main_gear_pressed && d.lgciu_2_right_main_gear_pressed;
  const bool groundSpoilers = d.ground_spoilers_active_1 && d.ground_spoilers_active_2;
  f.onGround = (lgciu1 && lgciu2) || ((lgciu1 || lgciu2) && below50ft) || (below50ft && groundSpoilers);
  f.trackingMode = f.in->sim_data.slew_on || f.in->sim_data.pause_on || f.in->sim_data.tracking_mode_on_override;
}

void ElacComputer::updateFlightPhase(Frame& f) {
  // the pitch law's flight / ground chart (741-777): in flight when off the ground with more than 8 deg of pitch or
  // above 400 ft RA; back on the ground 5 s after touching down with less than 2.5 deg of pitch
  SensorState& s = sensors_;
  const double now = f.in->time.simulation_time;
  if (!s.flightChartActive) {
    s.flightChartActive = true;
    s.flightPhase = FlightPhase::Ground;
    s.inFlight = 0.0;
  } else if (s.flightPhase == FlightPhase::Flight) {
    if (f.onGround && f.theta < 2.5F) {
      s.onGroundTime = now;
      s.flightPhase = FlightPhase::FlightToGround;  // in_flight not written: stays 1
    } else {
      s.inFlight = 1.0;
    }
  } else if (s.flightPhase == FlightPhase::FlightToGround) {
    if (now - s.onGroundTime >= 5.0) {
      s.flightPhase = FlightPhase::Ground;
      s.inFlight = 0.0;
    } else if (!f.onGround || f.theta >= 2.5F) {
      s.onGroundTime = 0.0;
      s.flightPhase = FlightPhase::Flight;
      s.inFlight = 1.0;
    }
  } else if ((!f.onGround && f.theta > 8.0F) || f.radioHeight > 400.0F) {
    s.onGroundTime = 0.0;
    s.flightPhase = FlightPhase::Flight;
    s.inFlight = 1.0;
  } else {
    s.inFlight = 0.0;
  }
  f.inFlight = s.inFlight != 0.0;
}

void ElacComputer::updateHydraulics(Frame& f) {
  // (779-796): pressure above 1750 psi (down to 1450) and the system's pressure switch, confirmed 0.5 s
  const elac_inputs& in = *f.in;
  const double dt = in.time.dt;
  const bool yellowHigh = sensors_.yellowPressure.update(in.analog_inputs.yellow_hyd_pressure_psi, 1750.0, 1450.0);
  f.yellowAvail = sensors_.yellowConfirm.update(!in.discrete_inputs.yellow_low_pressure && yellowHigh, dt, true, 0.5);
  const bool blueHigh = sensors_.bluePressure.update(in.analog_inputs.blue_hyd_pressure_psi, 1750.0, 1450.0);
  f.blueAvail = sensors_.blueConfirm.update(!in.discrete_inputs.blue_low_pressure && blueHigh, dt, true, 0.5);
  const bool greenHigh = sensors_.greenPressure.update(in.analog_inputs.green_hyd_pressure_psi, 1750.0, 1450.0);
  f.greenAvail = sensors_.greenConfirm.update(!in.discrete_inputs.green_low_pressure && greenHigh, dt, true, 0.5);
}

}  // namespace elac::readable
