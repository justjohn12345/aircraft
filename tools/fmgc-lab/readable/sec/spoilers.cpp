// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SEC spoilers (generated SecComputer.cpp 721-987): speed brake inhibition, ground spoilers and partial lift dumping,
// the speed brake command, and the roll + speed brake + ground spoiler orders of the SEC's spoiler pairs.

#include <cmath>

#include "sec.h"

namespace sec::readable {

namespace {

using blocks::a429::bit;
using blocks::a429::isNormal;
using blocks::a429::isNotFailed;

bool on(const base_arinc_429& w, int n) { return bit(w, n) != 0; }
bool onValid(const base_arinc_429& w, int n) { return bit(w, n) != 0 && isNotFailed(w); }

// speed brake lever (clamped) -> speed brake deflection (deg, negative up)
constexpr double kLeverBp[5] = {0.0, 0.05, 0.051, 0.5, 1.0};
constexpr double kSpeedBrakeDeg[5] = {0.0, 0.0, -2.0, -25.0, -40.0};

/** One spoiler's order before limits (924, 940, 955, 970): its own order minus the part of the other one beyond -50. */
double spoilerOrder(double own, double other) {
  return std::fmax(own - (other - std::fmax(other, -50.0)), -50.0);
}

}  // namespace

void SecComputer::updateSpoilers(Frame& f) {
  const sec_inputs& in = *f.in;
  const base_sec_bus_inputs& b = in.bus_inputs;
  const base_sec_discrete_inputs& d = in.discrete_inputs;
  const base_sec_analog_inputs& a = in.analog_inputs;
  const double dt = in.time.dt;

  // speed brake inhibition (721-759): an ELAC's alpha protection (discrete word 2 bit 23), flaps FULL (SFCC bit 23),
  // the elevators lost (both ELACs report an elevator unavailable, or with both ELACs' pitch lost an elevator lost by
  // this SEC and the other), or a thrust lever at 35 deg (MCT) or more; kept until the lever is retracted for 10 s
  const bool alphaProt = onValid(b.elac_1_bus.discrete_status_word_2, 23) || onValid(b.elac_2_bus.discrete_status_word_2, 23);
  const bool flapsFull = onValid(b.sfcc_1_bus.slat_flap_actual_position_word, 23) ||
                         onValid(b.sfcc_2_bus.slat_flap_actual_position_word, 23);
  const bool elac1ElevatorLost =
      !on(b.elac_1_bus.discrete_status_word_1, 17) || !on(b.elac_1_bus.discrete_status_word_1, 18);
  const bool elac2ElevatorLost =
      !on(b.elac_2_bus.discrete_status_word_1, 17) || !on(b.elac_2_bus.discrete_status_word_1, 18);
  const bool elevatorsLost =
      (!f.elacsPitchLost && elac1ElevatorLost && elac2ElevatorLost) ||
      (f.elacsPitchLost && ((d.left_elev_not_avail_sec_opp && !f.leftElevAvail) ||
                            (d.right_elev_not_avail_sec_opp && !f.rightElevAvail)));
  const bool inhibit = alphaProt || flapsFull || elevatorsLost || (a.thr_lever_1_pos >= 35.0 || a.thr_lever_2_pos >= 35.0);
  const bool leverRetracted = s_.leverRetracted.update(a.spd_brk_lever_pos < 0.05, dt, true, 10.0);
  s_.speedBrakeInhibitLatch = blocks::SrLatch{s_.speedBrakeInhibitLatch}.update(inhibit, leverRetracted);
  f.speedBrakeInhibited = inhibit || s_.speedBrakeInhibitLatch;

  // ground spoilers (763-816): out at touchdown (both main gears), or above 72 kt wheel speed
  // after the wheels were below 23 kt on the ground (rejected take-off), with the lever armed or extended and both
  // thrust levers below 20 deg, or a reverser selected; partial lift dumping with one main gear compressed (or at
  // touchdown with the lever armed and both thrust levers at 25 deg or less)
  const bool touchdown = s_.touchdown.update(f.onGround, true);
  const bool wheelsSlow = touchdown || (a.wheel_speed_left < 23.0 && a.wheel_speed_right < 23.0);
  const bool liftOff = s_.liftOff.update(f.onGround, false);
  s_.wheelsSlowLatch = blocks::SrLatch{s_.wheelsSlowLatch}.update(wheelsSlow, liftOff);
  const base_arinc_429& l1 = b.lgciu_1_bus.discrete_word_2;
  const base_arinc_429& l2 = b.lgciu_2_bus.discrete_word_2;
  // both main gears: the left on LGCIU 1 (word 2 bit 13) and the right on LGCIU 2 (bit 14), twice (771-783)
  const bool bothGears = (on(l1, 13) && on(l2, 14)) || (on(l1, 13) && on(l2, 14));
  const bool mainGearsDown = s_.bothMainGears.update(bothGears, true);
  f.groundSpoilersArmed = a.spd_brk_lever_pos < -0.02;
  const bool extensionCondition =
      ((a.spd_brk_lever_pos > 0.05 || f.groundSpoilersArmed) && (a.thr_lever_1_pos < 20.0 && a.thr_lever_2_pos < 20.0)) ||
      ((a.thr_lever_1_pos < 0.0 && a.thr_lever_2_pos <= 35.0) || (a.thr_lever_1_pos <= 35.0 && a.thr_lever_2_pos < 0.0));
  s_.groundSpoilersOut =
      extensionCondition && (mainGearsDown || (a.wheel_speed_left >= 72.0 && a.wheel_speed_right >= 72.0 &&
                                               s_.wheelsSlowLatch) ||
                             s_.groundSpoilersOut);
  // one main gear: the left on LGCIU 1 (word 2 bit 13) or the right on LGCIU 2 (bit 14) (797-811, finding S4)
  const bool oneGear = on(l1, 13) || on(l2, 14);
  const bool oneGearDown = s_.oneMainGear.update(oneGear || oneGear, true);
  s_.partialLiftDumpingOneGear = extensionCondition && (oneGearDown || s_.partialLiftDumpingOneGear);
  s_.partialLiftDumpingArmed = f.groundSpoilersArmed && (a.thr_lever_1_pos <= 25.0 && a.thr_lever_2_pos <= 25.0) &&
                               (mainGearsDown || s_.partialLiftDumpingArmed);
  f.partialLiftDumping = !s_.groundSpoilersOut && (s_.partialLiftDumpingOneGear || s_.partialLiftDumpingArmed);

  // ground spoiler order: -50 deg out, -10 deg partial lift dumping, 20 deg/s (819-830)
  const double groundTarget = s_.groundSpoilersOut ? -50.0 : (f.partialLiftDumping ? -10.0 : 0.0);
  const double groundOrder = s_.groundSpoilerRate.update(groundTarget, 20.0, -20.0, dt, 0.0);
  const bool groundActive = s_.groundSpoilersOut || f.partialLiftDumping;

  // speed brakes (831-857): the lever, limited to half with an AP engaged (ELAC discrete word 2 bit 22), through the
  // deflection table, 5 deg/s; 0 while inhibited
  double speedBrake;
  if (f.speedBrakeInhibited) {
    speedBrake = 0.0;
  } else {
    double lever = on(b.elac_1_bus.discrete_status_word_2, 22) || on(b.elac_2_bus.discrete_status_word_2, 22) ? 0.5 : 1.0;
    if (a.spd_brk_lever_pos <= lever) {
      lever *= -1.0;
      if (a.spd_brk_lever_pos >= lever) lever = a.spd_brk_lever_pos;
    }
    speedBrake = blocks::lookup1(lever, kLeverBp, kSpeedBrakeDeg);
  }
  f.speedBrakeCommand = s_.speedBrakeRate.update(speedBrake, 5.0, -5.0, dt, 0.0);

  // roll order (858-883): ELAC 1's roll spoiler order, else ELAC 2's, else the SEC's own direct law (xi x 35 / 25),
  // +-35 deg; the ELAC's discrete word 1 bits 28 / 29 enable the roll function on SEC 1's / SEC 3's pairs
  const efcs::readable::LateralLawOutputs direct = lateralDirect_.step({dt, f.rollStick});
  double roll;
  bool pairsSec1, pairsSec3;
  if (isNormal(b.elac_1_bus.roll_spoiler_command_deg)) {
    roll = b.elac_1_bus.roll_spoiler_command_deg.Data;
    pairsSec1 = on(b.elac_1_bus.discrete_status_word_1, 28);
    pairsSec3 = on(b.elac_1_bus.discrete_status_word_1, 29);
  } else if (isNormal(b.elac_2_bus.roll_spoiler_command_deg)) {
    roll = b.elac_2_bus.roll_spoiler_command_deg.Data;
    pairsSec1 = on(b.elac_2_bus.discrete_status_word_1, 28);
    pairsSec3 = on(b.elac_2_bus.discrete_status_word_1, 29);
  } else {
    roll = direct.xi_deg * 35.0 / 25.0;
    pairsSec1 = pairsSec3 = true;
  }
  roll = std::fmax(std::fmin(roll, 35.0), -35.0);

  // per pair (884-909): roll and speed brake parts. SEC 1: pair 1 roll (when enabled) and speed brakes, pair 2 roll and
  // speed brakes; SEC 2: roll only; SEC 3: pair 2 roll (when enabled) and half the speed brakes
  double roll1, roll2, brake1, brake2;
  if (d.is_unit_1) {
    roll1 = pairsSec1 ? roll : 0.0;
    roll2 = roll;
    brake1 = brake2 = f.speedBrakeCommand;
  } else if (d.is_unit_2) {
    roll1 = roll;
    roll2 = brake1 = brake2 = 0.0;
  } else {
    roll1 = 0.0;
    roll2 = pairsSec3 ? roll : 0.0;
    brake1 = 0.0;
    brake2 = f.speedBrakeCommand / 2.0;
  }
  // the roll goes on the spoilers of the down-going wing side (911-919): rolling right (>= 0) raises the left ones
  double left1, right1, left2, right2;
  if (roll >= 0.0) {
    left1 = brake1 - roll1;
    right1 = brake1;
    left2 = brake2 - roll2;
    right2 = brake2;
  } else {
    left1 = brake1;
    right1 = brake1 + roll1;
    left2 = brake2;
    right2 = brake2 + roll2;
  }

  // orders (921-987): the ground spoiler order when active, else the sum moved to the other side beyond -50; -50..0
  // deg, 50 deg/s; a pair not available follows its measured positions
  const double u1 = blocks::limit(groundActive ? groundOrder : spoilerOrder(left1, right1), -50.0, 0.0);
  f.leftSpoiler1 = s_.leftSpoiler1.update(u1, 50.0, -50.0, dt, a.left_spoiler_1_pos_deg, !f.spoilerPair1Avail);
  const double u2 = blocks::limit(groundActive ? groundOrder : spoilerOrder(right1, left1), -50.0, 0.0);
  f.rightSpoiler1 = s_.rightSpoiler1.update(u2, 50.0, -50.0, dt, a.right_spoiler_1_pos_deg, !f.spoilerPair1Avail);
  const double u3 = blocks::limit(groundActive ? groundOrder : spoilerOrder(left2, right2), -50.0, 0.0);
  f.leftSpoiler2 = s_.leftSpoiler2.update(u3, 50.0, -50.0, dt, a.left_spoiler_2_pos_deg, !f.spoilerPair2Avail);
  const double u4 = blocks::limit(groundActive ? groundOrder : spoilerOrder(right2, left2), -50.0, 0.0);
  f.rightSpoiler2 = s_.rightSpoiler2.update(u4, 50.0, -50.0, dt, a.right_spoiler_2_pos_deg, !f.spoilerPair2Avail);
}

}  // namespace sec::readable
