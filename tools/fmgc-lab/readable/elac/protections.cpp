// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// ELAC protections (generated ElacComputer.cpp 1086-1238): alpha prot / alpha max, high speed protection, high angle
// of attack protection, the AP disconnect phases, AP disconnect and authorisation.

#include <cmath>

#include "elac.h"

namespace elac::readable {

namespace {

// alpha max / alpha prot (deg) against Mach (0, 0.5, 0.9, 1) and the flap handle index from the SFCC status word
// (CONF 0, 1, 1+F, 2, 3, FULL); u0 (Mach) varies fastest
constexpr double kMachBp[4] = {0.0, 0.5, 0.9, 1.0};
constexpr double kConfigBp[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
constexpr double kAlphaMax[24] = {8.7,  8.7,  6.4,  6.4,  13.6, 13.6, 13.6, 13.6, 13.6, 13.6, 13.6, 13.6,
                                  14.2, 14.2, 14.2, 14.2, 13.1, 13.1, 13.1, 13.1, 13.0, 13.0, 13.0, 13.0};
constexpr double kAlphaProt[24] = {6.5,  6.5,  4.6,  4.6,  11.7, 11.7, 11.7, 11.7, 11.7, 11.7, 11.7, 11.7,
                                   11.9, 11.9, 11.9, 11.9, 11.0, 11.0, 11.0, 11.0, 10.6, 10.6, 10.6, 10.6};
// high speed protection thresholds against the flight path angle (deg): VMO + 6 kt / MMO + 0.01 when diving
// (uDLookupTable1 / 2), and the AP disconnect Mach (uDLookupTable)
constexpr double kGammaBp[4] = {-4.0, -3.0, -1.0, 0.0};
constexpr double kHsProtVmo[4] = {350.0, 350.0, 356.0, 356.0};
constexpr double kHsProtMmo[4] = {0.82, 0.82, 0.83, 0.83};
constexpr double kApDiscMach[4] = {0.82, 0.82, 0.85, 0.85};

}  // namespace

/** The flap handle index from the SFCC 1 system status word bits 17-21 and 26 (generated MATLABFunction_o, 106-124). */
double flapHandleIndex(const base_arinc_429& status) {
  const auto on = [&](int n) { return blocks::a429::bit(status, n) != 0; };
  if (on(17)) return 0.0;
  if (on(18) && on(26)) return 1.0;
  if (on(18) && !on(26)) return 2.0;
  if (on(19)) return 3.0;
  if (on(20)) return 4.0;
  if (on(21)) return 5.0;
  return 0.0;
}

void ElacComputer::updateProtections(Frame& f) {
  const elac_inputs& in = *f.in;
  ProtectionState& p = protections_;
  const double dt = in.time.dt;
  const double now = in.time.simulation_time;

  // alpha max, alpha prot: rate limited 1 deg/s; alpha prot = alpha max for 5 s after lift-off (1086-1130)
  const double config = flapHandleIndex(in.bus_inputs.sfcc_1_bus.slat_flap_system_status_word);
  f.alphaMax = p.alphaMaxRate.update(blocks::lookup2(static_cast<double>(f.mach), config, kMachBp, kConfigBp, kAlphaMax),
                                     1.0, -1.0, dt);
  if (!p.liftOffTimeSet) {
    p.liftOffTime = now;
    p.liftOffTimeSet = true;
  }
  if (f.onGround || p.liftOffTime == 0.0) p.liftOffTime = now;
  const double alphaProtTable = p.alphaProtRate.update(
      blocks::lookup2(static_cast<double>(f.mach), config, kMachBp, kConfigBp, kAlphaProt), 1.0, -1.0, dt);
  f.alphaProt = now - p.liftOffTime <= 5.0 ? f.alphaMax : alphaProtTable;

  // high speed protection thresholds (1131-1137): VMO / MMO 350 kt / M 0.82 (low), 380 kt / M 0.88 (high), the Mach
  // limits as CAS at the current CAS / Mach ratio (GetIASforMach4)
  const double vIas = f.vIas, mach = f.mach;
  f.highSpeedProtLow = std::fmin(350.0, vIas * 0.82 / mach);
  f.highSpeedProtHigh = std::fmin(380.0, vIas * 0.88 / mach);

  // flight path angle estimate theta - alpha cos(phi) (1139), for the overspeed thresholds
  const double gamma = f.theta - std::cos(0.017453292519943295 * f.phi) * f.alphaFiltered;
  f.normalLawActive = f.activePitchLaw == pitch_efcs_law::NormalLaw || f.activeLateralLaw == lateral_efcs_law::NormalLaw;

  // high speed protection (1142-1151): set above VMO + 6 / MMO + 0.01 (+0 when diving more than 3 deg, finding L6),
  // without AP, in a normal law; reset below VMO / MMO (the low threshold)
  f.noApEngaged = in.discrete_inputs.ap_1_disengaged && in.discrete_inputs.ap_2_disengaged;
  if (f.noApEngaged && f.vIas > std::fmin(blocks::lookup1(gamma, kGammaBp, kHsProtVmo),
                                          vIas / mach * blocks::lookup1(gamma, kGammaBp, kHsProtMmo))) {
    p.highSpeedProt = f.normalLawActive || p.highSpeedProt;
  }
  p.highSpeedProt = f.vIas >= f.highSpeedProtLow && (f.noApEngaged && f.normalLawActive && p.highSpeedProt);

  // high AoA protection (1152-1165): set above alpha prot in flight, normal law, no AP, 10 s after power-up; held while
  // alpha >= alpha prot - 2 deg, or the stick is pulled back to 0.5, or above 200 ft RA; reset by pushing the stick
  // forward of -0.5, or keeping it forward of -1/32 for 0.5 s (unless alpha >= alpha max)
  if (!p.stickNotPulledTimeSet) {
    p.stickNotPulledTime = now;
    p.stickNotPulledTimeSet = true;
  }
  if (f.pitchStick >= -0.03125 || f.alphaFiltered >= f.alphaMax || p.stickNotPulledTime == 0.0) {
    p.stickNotPulledTime = now;
  }
  p.alphaProt = (!f.onGround && f.normalLawActive && f.noApEngaged && f.alphaFiltered > f.alphaProt &&
                 in.time.monotonic_time > 10.0) ||
                p.alphaProt;
  p.alphaProt = now - p.stickNotPulledTime <= 0.5 && f.pitchStick >= -0.5 &&
                (f.radioHeight >= 200.0F || f.pitchStick >= 0.5 || f.alphaFiltered >= f.alphaProt - 2.0) &&
                !f.onGround && f.normalLawActive && p.alphaProt;

  // AP disconnect phases (1166-1217): Landed -> Takeoff100ft (off the ground) -> Flying (above 100 ft RA) ->
  // Landing100ft (below 100 ft: the AP disconnects above alpha max) -> Landed (on the ground)
  int below100ft = 0;
  if (!p.apChartActive) {
    p.apChartActive = true;
    p.apPhase = ApPhase::Landed;
  } else {
    switch (p.apPhase) {
      case ApPhase::Flying:
        if (f.radioHeight < 100.0F) {
          p.apPhase = ApPhase::Landing100ft;
          below100ft = 1;
        } else if (f.onGround) {
          p.apPhase = ApPhase::Landed;
        }
        break;
      case ApPhase::Landed:
        if (!f.onGround) p.apPhase = ApPhase::Takeoff100ft;
        break;
      case ApPhase::Landing100ft:
        if (f.radioHeight > 100.0F) {
          p.apPhase = ApPhase::Flying;
        } else if (f.onGround) {
          p.apPhase = ApPhase::Landed;
        } else {
          below100ft = 1;
        }
        break;
      default:  // Takeoff100ft
        if (f.onGround) {
          p.apPhase = ApPhase::Landed;
        } else if (f.radioHeight > 100.0F) {
          p.apPhase = ApPhase::Flying;
        }
        break;
    }
  }

  // AP disconnect (1219-1234): alpha > alpha prot + 0.25 deg (alpha max below 100 ft in approach) in flight in a
  // normal law, 3 s above the overspeed limit (min(365 kt, MMO 0.82..0.85 + 0.01)) unless both laws are degraded, or a
  // protection active
  if (!p.speedBelowLimitTimeSet) {
    p.speedBelowLimitTime = now;
    p.speedBelowLimitTimeSet = true;
  }
  if (f.vIas <= std::fmin(365.0, vIas / mach * (blocks::lookup1(gamma, kGammaBp, kApDiscMach) + 0.01)) ||
      (f.activePitchLaw != pitch_efcs_law::NormalLaw && f.activeLateralLaw != lateral_efcs_law::NormalLaw) ||
      p.speedBelowLimitTime == 0.0) {
    p.speedBelowLimitTime = now;
  }
  f.apDisconnect = (!f.onGround &&
                    ((below100ft != 0 && f.alphaFiltered > f.alphaMax) || f.alphaFiltered > f.alphaProt + 0.25) &&
                    f.normalLawActive) ||
                   now - p.speedBelowLimitTime > 3.0 || p.highSpeedProt || p.alphaProt;

  // AP authorisation (1235-1238): sticks within half deflection, pedals within 0.4, pitch -13..25 deg, bank within 45
  // deg, the axes this ELAC has priority in available, no AP disconnect condition
  f.apAuthorised = std::abs(f.pitchStick) <= 0.5 && std::abs(f.rollStick) <= 0.5 &&
                   (std::abs(in.analog_inputs.rudder_pedal_pos) <= 0.4 &&
                    (f.theta <= 25.0F && f.theta >= -13.0F && std::abs(static_cast<double>(f.phi)) <= 45.0 &&
                     (!f.hasPriorityInPitch || f.canEngageInPitch) && (!f.hasPriorityInRoll || f.canEngageInRoll) &&
                     !f.apDisconnect));
}

}  // namespace elac::readable
