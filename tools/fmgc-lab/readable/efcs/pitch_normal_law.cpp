// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchNormalLaw (generated PitchNormalLaw.cpp 1162-2457). See pitch_normal_law.h for the structure. Line
// numbers in the comments are PitchNormalLaw.cpp's.

#include "pitch_normal_law.h"

#include <cmath>

namespace efcs::readable {

using namespace pitch_blocks;

namespace {

using FlareMode = PitchNormalLawState::FlareMode;
using Rotation = PitchNormalLawState::Rotation;

/** Elevator per unit of stick deflection on the ground and as the in-flight integrator's ground value (deg). */
constexpr double kStickToEtaDeg = -30.0;
/** Elevator travel (deg) of the in-flight law, the rotation law and the final command. */
constexpr double kEtaMinDeg = -30.0, kEtaMaxDeg = 17.0;
/** Load factor demand from the pitch attitude limits: 0.2 g per degree towards the limit. */
constexpr double kThetaLimitGain = 0.2;
constexpr double kThetaMinDeg = -15.0;

/** Speed schedules (V_tas kt) of the pitch attitude command channels: autopilot and flare (g per degree). */
constexpr double kAttitudeGainBp[7] = {0.0, 100.0, 150.0, 200.0, 250.0, 300.0, 400.0};
constexpr double kAutopilotGain[7] = {0.1, 0.1, 0.15, 0.2, 0.3, 0.5, 0.5};
constexpr double kFlareGain[7] = {0.1, 0.1, 0.1, 0.15, 0.2, 0.3, 0.3};

/** Spoiler term gain of the C* channels against radio height (ft): none below 50 ft, -30 from 100 ft. */
constexpr double kSpoilerGainBp[4] = {0.0, 50.0, 100.0, 200.0};
constexpr double kSpoilerGain[4] = {0.0, 0.0, -30.0, -30.0};

/** Integrator gain against V_ias (kt): 1 up to 163 kt, 0.5 at 243 kt, 0.3 from 344 kt (both integrators). */
constexpr double kIntegratorGainBp[5] = {0.0, 163.0, 243.0, 344.0, 400.0};
constexpr double kIntegratorGain[5] = {1.0, 1.0, 0.5, 0.3, 0.3};

/** Ground elevator limits of the flare law (fraction of 20 deg) against pitch rate (deg/s) x elevator (deg). */
constexpr double kGroundLimitQBp[7] = {-10.0, -2.0, -0.1, 0.0, 0.1, 2.0, 10.0};
constexpr double kGroundLimitEtaBp[5] = {-10.0, -3.0, 0.0, 3.0, 10.0};
constexpr double kGroundLimitUp[35] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0};
constexpr double kGroundLimitLo[35] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                       1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
constexpr double kFlareEtaLimitDeg = 20.0;

/** Tail strike protection: pitch rate demand reduction (fraction of the stick) against pitch (deg) x radio height. */
constexpr double kTailstrikeThetaBp[5] = {-1.0, 0.0, 6.0, 11.0, 15.0};
constexpr double kTailstrikeHeightBp[5] = {-1.0, 8.0, 18.0, 28.0, 40.0};
constexpr double kTailstrikeReduction[25] = {0.0, 0.0, 0.0, -0.75, -0.75, 0.0, 0.0, 0.0, -0.75, -0.75, 0.0, 0.0, 0.0,
                                             0.0, -0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
/** Rotation law pitch rate demand (deg/s) against stick. */
constexpr double kStickBp[3] = {-1.0, 0.0, 1.0};
constexpr double kPitchRateDemand[3] = {-9.0, 0.0, 9.0};

/** 0..1 blend weight (saturation after the blend rate limiters). */
double weight(double y) { return limit(y, 0.0, 1.0); }

}  // namespace

// ---------------------------------------------------------------------------------------------------------------------
// init / reset
// ---------------------------------------------------------------------------------------------------------------------

void PitchNormalLaw::init() {
  // 1162-1209: every Delay state to its initial condition (all 0) and both integrators to load on the next step. The
  // filters, rate limiters and charts keep their state (all zero after construction).
  s_.flareThetaCmd.reset();
  s_.cstar.init();
  s_.thetaMaxChannel.reset();
  s_.nzMaxChannel.reset();
  s_.autopilotChannel.reset();
  s_.flareChannel.reset();
  s_.normalChannel.reset();
  s_.nzMinChannel.reset();
  s_.thetaMinChannel.reset();
  s_.apBlend.reset();
  s_.alphaErrorRate.reset();
  s_.iasRate.reset();
  s_.aoaBlend.reset();
  s_.flareThetaLag.reset();
  s_.flareBlend.reset();
  s_.hspStick.reset();
  s_.hspTargetRate.reset();
  s_.hspBlend.reset();
  s_.qDemandRate.reset();
  s_.qErrorRate.reset();
  s_.flightIntegrator.requestLoad();
  s_.rotationIntegrator.requestLoad();
}

void PitchNormalLaw::reset() {
  // 1211-1306: init(), plus every filter / rate limiter back to "first frame", the charts inactive and the flare mode
  // outputs to 0. The integrator states themselves are not reset (they are reloaded on the next step).
  init();
  s_.thetaFilter.reset();
  s_.flareMode = FlareMode::None;
  s_.flareThetaCmdDeg = 0.0;
  s_.flareThetaCmdRateDegS = 0.0;
  s_.trimFreeze.reset();
  s_.autoTrimMode.reset();
  s_.flightPhase.reset();
  s_.inFlightBlend.reset();
  s_.rotation = Rotation::None;
  s_.rotationBlend.reset();
  s_.nzMax.reset();
  s_.nzMin.reset();
  s_.trimLimitLo.reset();
  s_.trimLimitUp.reset();
  s_.thetaMax.reset();
  s_.cstar.reset();
  s_.apThetaCmd.reset();
  s_.aoaStick.reset();
  s_.alphaFilter.reset();
  s_.attitudeWashout.reset();
  s_.flareThetaLimit.reset();
  s_.flareThetaRate.reset();
  s_.normalStick.reset();
  s_.rotationStick.reset();
  s_.qPredicted.reset();
  s_.eta.reset();
}

// ---------------------------------------------------------------------------------------------------------------------
// Flare mode (1388-1477, 1667-1670)
// ---------------------------------------------------------------------------------------------------------------------

PitchNormalLaw::FlareOutputs PitchNormalLaw::flareMode(const PitchNormalLawInputs& in) {
  // 1388-1399: filtered pitch attitude (0.05 rad/s). The debug switches (Theta_c debug value, manual flare triggers)
  // are constant: never in debug, never forced.
  const double thetaFiltered = s_.thetaFilter.update(in.Theta_deg, 0.05, in.time_dt);
  const double h = in.H_radio_ft;
  const bool backInFlight = in.in_flight == 1.0 && h > 50.0;

  // 1400-1477, chart c2:
  //   Ground   -> Flight          in flight above 50 ft
  //   Flight   -> Prepare         at 50 ft: the command starts at the filtered pitch attitude
  //   Prepare  -> Active Armed    at 50 ft (the next step): the command now decreases to -2 deg in 8 s from where it
  //                               started (rate (theta_f + 2) / 8, theta_f at least -2 deg)
  //   Armed    -> Active Reduce   at 30 ft: the command jumps to -2 deg (still at the same rate limit)
  //   Reduce   -> Ground          on ground
  //   any but Ground/Flight -> Flight  back above 50 ft in flight
  // in_flare is true in Active Armed and Active Reduce.
  bool inFlare = false;
  switch (s_.flareMode) {
    case FlareMode::None:
      s_.flareMode = FlareMode::Ground;
      s_.flareThetaCmdDeg = -2.0;
      s_.flareThetaCmdRateDegS = -1000.0;
      break;
    case FlareMode::ActiveArmed:
      if (h <= 30.0) {
        s_.flareMode = FlareMode::ActiveReduce;
        inFlare = true;
        s_.flareThetaCmdDeg = -2.0;
      } else if (backInFlight) {
        s_.flareMode = FlareMode::Flight;
      } else {
        inFlare = true;
      }
      break;
    case FlareMode::ActiveReduce:
      if (in.in_flight == 0.0) {
        s_.flareMode = FlareMode::Ground;
        s_.flareThetaCmdDeg = -2.0;
        s_.flareThetaCmdRateDegS = -1000.0;
      } else if (backInFlight) {
        s_.flareMode = FlareMode::Flight;
      } else {
        inFlare = true;
        s_.flareThetaCmdDeg = -2.0;
      }
      break;
    case FlareMode::Prepare:
      if (h <= 50.0) {
        s_.flareThetaCmdRateDegS = -(std::fmax(-2.0, thetaFiltered) + 2.0) / 8.0;
        s_.flareMode = FlareMode::ActiveArmed;
        inFlare = true;
      } else if (backInFlight) {
        s_.flareMode = FlareMode::Flight;
      }
      break;
    case FlareMode::Flight:
      if (h <= 50.0) {
        s_.flareThetaCmdRateDegS = -1000.0;
        s_.flareThetaCmdDeg = thetaFiltered;
        s_.flareMode = FlareMode::Prepare;
      }
      break;
    default:  // Ground
      if (backInFlight) {
        s_.flareMode = FlareMode::Flight;
      } else {
        s_.flareThetaCmdDeg = -2.0;
        s_.flareThetaCmdRateDegS = -1000.0;
      }
      break;
  }

  // 1667-1670: the command, rate limited both ways by |rate| upwards and by rate downwards (a negative rate). With the
  // -1000 deg/s rate outside the flare it follows the command at once.
  const double rate = s_.flareThetaCmdRateDegS;
  const double thetaCmd = s_.flareThetaCmd.update(s_.flareThetaCmdDeg, std::abs(rate), rate, in.time_dt);
  return {inFlare, thetaCmd};
}

// ---------------------------------------------------------------------------------------------------------------------
// Ground / flight blends (1620-1660) and trim limits (1671-1685)
// ---------------------------------------------------------------------------------------------------------------------

PitchNormalLaw::Blends PitchNormalLaw::groundFlightBlends(const PitchNormalLawInputs& in) {
  const double dt = in.time_dt;
  // 1620-1629: in flight, 0 -> 1 at 0.2 /s
  const double inFlight = s_.inFlightBlend.update(limit(in.in_flight, 0.0, 1.0), 0.2, -0.2, dt, 0.0);

  // 1630-1648, chart c6: take-off rotation. On while not fully in flight, above 70 kt with a thrust lever at 35 (FLX /
  // MCT) or more; off once fully in flight, above 400 ft RA, or below 70 kt with a thrust lever below 35.
  const bool leverAtFlex = in.thrust_lever_1_pos >= 35.0 || in.thrust_lever_2_pos >= 35.0;
  const bool leverBelowFlex = in.thrust_lever_1_pos < 35.0 || in.thrust_lever_2_pos < 35.0;
  bool rotating;
  if (s_.rotation == Rotation::None) {
    s_.rotation = Rotation::Off;
    rotating = false;
  } else if (s_.rotation == Rotation::Off) {
    rotating = inFlight < 1.0 && in.V_tas_kn > 70.0 && leverAtFlex;
    if (rotating) {
      s_.rotation = Rotation::On;
    }
  } else if (inFlight == 1.0 || in.H_radio_ft > 400.0 || (in.V_tas_kn < 70.0 && leverBelowFlex)) {
    s_.rotation = Rotation::Off;
    rotating = false;
  } else {
    rotating = true;
  }

  // 1650-1660: rotation 0 -> 1 at 0.5 /s (the generated 0..1 saturation of the chart output is a no-op)
  const double rotation = s_.rotationBlend.update(rotating ? 1.0 : 0.0, 0.5, -0.5, dt, 0.0);
  return {inFlight, rotation};
}

void PitchNormalLaw::trimLimits(const PitchNormalLawInputs& in, PitchLawOutputs& out) {
  // 1671-1685: THS limits -11 / +3.5 deg; while the high AoA (high speed) protection is active the lower (upper)
  // limit is frozen at the THS position it had when the protection became active.
  const double lo = s_.trimLimitLo.update(in.eta_trim_deg, in.high_aoa_prot_active);
  out.eta_trim_limit_lo = in.high_aoa_prot_active ? lo : -11.0;
  const double up = s_.trimLimitUp.update(in.eta_trim_deg, in.high_speed_prot_active);
  out.eta_trim_limit_up = in.high_speed_prot_active ? up : 3.5;
}

// ---------------------------------------------------------------------------------------------------------------------
// Pitch attitude protection channels (1686-1719 / 2268-2283, 2224-2289)
// ---------------------------------------------------------------------------------------------------------------------

double PitchNormalLaw::thetaMaxChannel(const PitchNormalLawInputs& in, const CStarTerms& t, double thetaGain) {
  // 1686-1696: maximum pitch attitude 30 deg (25 deg in CONF FULL, flap lever index 5), reduced by up to 5 deg as the
  // speed drops from VLS + 25 kt to VLS + 5 kt; rate limited 1 deg/s, starting at 30 deg.
  const double maxDeg = in.flaps_handle_index == 5.0 ? 25.0 : 30.0;
  const double lowSpeedReduction = std::fmin(5.0, std::fmax(0.0, 5.0 - (in.V_ias_kn - (in.VLS_kn + 5.0)) * 0.25));
  const double thetaMax = s_.thetaMax.update(maxDeg - lowSpeedReduction, 1.0, -1.0, in.time_dt, 30.0);
  // 1697-1714: load factor demand 0.2 g/deg towards the limit (-0.25 .. 1 g through the stick load demand table)
  const double toLimit = limit(kThetaLimitGain * thetaMax - thetaGain, -0.25, 1.0);
  return s_.thetaMaxChannel.update(t, t.cosRatio + loadDemand(toLimit), in.time_dt);
}

double PitchNormalLaw::thetaMinChannel(const CStarTerms& t, double thetaGain, double dt) {
  // 2230-2245: minimum pitch attitude -15 deg, the same way (-1 .. 0.25 g)
  const double toLimit = limit(kThetaLimitGain * kThetaMinDeg - thetaGain, -1.0, 0.25);
  return s_.thetaMinChannel.update(t, t.cosRatio + loadDemand(toLimit), dt);
}

// ---------------------------------------------------------------------------------------------------------------------
// Autopilot (1789-1869)
// ---------------------------------------------------------------------------------------------------------------------

double PitchNormalLaw::autopilotChannel(const PitchNormalLawInputs& in, const CStarTerms& t, const Attitude& a) {
  // 1802-1816: the FMGC pitch attitude command (rate limited 5 deg/s) -> load factor demand by a speed schedule
  const double thetaCmd = s_.apThetaCmd.update(in.ap_theta_c_deg, 5.0, -5.0, in.time_dt, 0.0);
  const double gain = lookup1(in.V_tas_kn, kAttitudeGainBp, kAutopilotGain);
  return s_.autopilotChannel.update(t, (thetaCmd - in.Theta_deg) * gain + a.cosRatio33, in.time_dt);
}

// ---------------------------------------------------------------------------------------------------------------------
// High angle of attack protection (1870-1918)
// ---------------------------------------------------------------------------------------------------------------------

double PitchNormalLaw::highAoaChannel(const PitchNormalLawInputs& in, double iasRate) {
  const double dt = in.time_dt;
  // 1870-1881: alpha error: stick (rate limited 1 /s) x (alpha max - alpha prot) above alpha prot, minus the filtered
  // alpha (1.6 rad/s) above alpha prot, minus a washed out (1 rad/s) attitude term: pitch above 22.5 deg, or bank
  // above 3 deg (/6)
  const double stick = s_.aoaStick.update(in.delta_eta_pos, 1.0, -1.0, dt, 0.0);
  const double alphaDemand = (in.alpha_max - in.alpha_prot) * stick;
  const double alphaAboveProt = s_.alphaFilter.update(in.alpha_deg, 1.6, dt) - in.alpha_prot;
  const double attitude =
      std::fmax(std::fmax(0.0, in.Theta_deg - 22.5), std::fmax(0.0, (std::abs(in.Phi_deg) - 3.0) / 6.0));
  const double error = (alphaDemand - alphaAboveProt) - s_.attitudeWashout.update(attitude, 1.0, dt);
  // 1882-1887: its rate, lagged (2 rad/s)
  const double errorRate = s_.alphaErrorRate.update(error, 2.0, dt);
  // 1910-1918: elevator rate (deg/s, +-4); the V_ias rate (1889-1894) is computed in step()
  const double cmd =
      (((-1.0 * errorRate + -1.2 * error) + -0.8 * iasRate) + in.qk_deg_s) + in.qk_dot_deg_s2;
  return limit(cmd, -4.0, 4.0);
}

// ---------------------------------------------------------------------------------------------------------------------
// Flare law (1921-2035)
// ---------------------------------------------------------------------------------------------------------------------

double PitchNormalLaw::flareChannel(const PitchNormalLawInputs& in, const CStarTerms& t, const Attitude& a,
                                    double flareThetaCmdDeg) {
  const double dt = in.time_dt;
  // 1921-1934: upper elevator rate limit: 20 deg/s; on the ground a pitch rate x elevator table (0 or 20)
  double upper = kFlareEtaLimitDeg;
  if (in.on_ground) {
    const double fraction = lookup2(in.qk_deg_s, in.eta_deg, kGroundLimitQBp, kGroundLimitEtaBp, kGroundLimitUp);
    upper = limit(fraction * kFlareEtaLimitDeg, 0.0, kFlareEtaLimitDeg);
  }

  // 1949-1958: pitch attitude command ceiling: 18 deg in flight, 9 deg on the ground (6 deg on the ground while
  // pitching up faster than 3 deg/s; exactly 3 deg/s, or a NaN rate, gives 18), rate limited 3 deg/s from 0
  int ceilingDeg;
  if (in.on_ground && in.qk_deg_s > 3.0) {
    ceilingDeg = 6;
  } else if (in.on_ground && in.qk_deg_s < 3.0) {
    ceilingDeg = 9;
  } else {
    ceilingDeg = 18;
  }
  const double ceiling = s_.flareThetaLimit.update(static_cast<double>(ceilingDeg), 3.0, -3.0, dt, 0.0);

  // 1960-1975: command = flare mode command + 20 deg x stick, -22 .. ceiling, lagged 5 rad/s, rate limited 12 deg/s
  const double raw = limit(20.0 * in.delta_eta_pos + flareThetaCmdDeg, -22.0, ceiling);
  const double thetaCmd = s_.flareThetaRate.update(s_.flareThetaLag.update(raw, 5.0, dt), 12.0, -12.0, dt, 0.0);

  // 1976-2013: C* channel on the pitch attitude error
  const double gain = lookup1(in.V_tas_kn, kAttitudeGainBp, kFlareGain);
  const double cmd = s_.flareChannel.update(t, (thetaCmd - in.Theta_deg) * gain + a.cosRatio33, dt);

  // 2015-2035: limited to [lower, upper]; lower -20, on the ground from a table (-20 or 0); a NaN command gives upper
  if (!(cmd <= upper)) {
    return upper;
  }
  double lower = -kFlareEtaLimitDeg;
  if (in.on_ground) {
    const double tableLower =
        lookup2(in.qk_deg_s, in.eta_deg, kGroundLimitQBp, kGroundLimitEtaBp, kGroundLimitLo) * lower;
    if (tableLower > 0.0) {
      lower = 0.0;
    } else if (tableLower >= lower) {
      lower = tableLower;
    }
  }
  return cmd < lower ? lower : cmd;
}

// ---------------------------------------------------------------------------------------------------------------------
// Normal law with the high speed protection (2063-2157)
// ---------------------------------------------------------------------------------------------------------------------

double PitchNormalLaw::normalChannel(const PitchNormalLawInputs& in, const CStarTerms& t, const Attitude& a,
                                     double iasRate) {
  const double dt = in.time_dt;
  // 2063-2067: stick (rate limited 4 /s) -> load factor increment (-2 .. +1.5 g)
  const double demand = loadDemand(s_.normalStick.update(in.delta_eta_pos, 4.0, -4.0, dt, 0.0));

  // 2068-2080: high speed protection speed target: the low threshold, raised towards the high one by nose down stick
  // (rate limited 0.5 /s), and its rate (lagged 2 rad/s)
  const double stick = s_.hspStick.update(in.delta_eta_pos, 0.5, -0.5, dt);
  double vTarget =
      std::fmax((in.high_speed_prot_low_kn - in.high_speed_prot_high_kn) * stick, 0.0) + in.high_speed_prot_low_kn;
  const double vTargetRate = s_.hspTargetRate.update(vTarget, 2.0, dt);

  // 2087-2112: while active: the stick demand (0 .. 1.5 g) plus a speed law (+-0.75 g)
  const double hspOn = in.high_speed_prot_active;
  double hspDemand;
  if (hspOn > 0.0) {
    const double qkDotTerm = -0.4 * in.qk_dot_deg_s2;
    const double qkTerm = -0.4 * in.qk_deg_s;
    vTarget -= in.V_ias_kn;
    const double speedLaw = ((((-0.6 * vTargetRate + -0.1 * vTarget) + 1.4 * iasRate) + qkTerm) + qkDotTerm) * 0.2;
    hspDemand = limit(demand, 0.0, 1.5) + limit(speedLaw, -0.75, 0.75);
  } else {
    hspDemand = 0.0;
  }

  // 2114-2128: blended in at 4 /s, out at 1/3 /s
  const double w = weight(s_.hspBlend.update(hspOn, 4.0, -0.33333333333333331, dt));
  return s_.normalChannel.update(t, ((1.0 - w) * demand + hspDemand * w) + a.cosRatio33, dt);
}

// ---------------------------------------------------------------------------------------------------------------------
// Ground law (2326-2397)
// ---------------------------------------------------------------------------------------------------------------------

double PitchNormalLaw::groundLaw(const PitchNormalLawInputs& in, const Blends& b, double stickEtaDeg) {
  const double dt = in.time_dt;
  // 2326-2342: rotation pitch rate demand from the stick (rate limited 0.5 /s, -9 .. 9 deg/s); tail strike
  // protection: on, nose up stick is reduced by up to 75 % at high pitch close to the ground
  const double stick = s_.rotationStick.update(in.delta_eta_pos, 0.5, -0.5, dt, 0.0);
  const double tailstrike = in.tailstrike_protection_on;
  const double reduction =
      lookup2(in.Theta_deg, in.H_radio_ft, kTailstrikeThetaBp, kTailstrikeHeightBp, kTailstrikeReduction);
  const double qDemand = lookup1(tailstrike * reduction * limit(stick, 0.0, 1.0) + stick, kStickBp, kPitchRateDemand);

  // 2343-2356: pitch rate law: PD on the pitch rate error, minus a feed-forward of the demand's rate, plus the error
  // of a predicted pitch rate (qk + 2.5 qk_dot, lagged 1 rad/s), plus 2 qk_dot
  const double qDemandRate = s_.qDemandRate.update(qDemand, dt);
  const double qError = in.qk_deg_s - qDemand;
  const double qErrorRate = s_.qErrorRate.update(0.2 * qError, dt);
  const double qPredicted = s_.qPredicted.update(2.5 * in.qk_dot_deg_s2 + in.qk_deg_s, 1.0, dt);
  const double cmd = (((4.0 * qError + qErrorRate) + -2.0 * qDemandRate) + (qPredicted - qDemand) * -1.5) +
                     2.0 * in.qk_dot_deg_s2;

  // 2357-2376: integrated while not in flight (x (1 - in-flight blend)); held at 0 with the stick at or forward of
  // neutral on the ground, outside the rotation, and in tracking mode
  const double increment = (1.0 - b.inFlight) * (cmd * lookup1(in.V_ias_kn, kIntegratorGainBp, kIntegratorGain)) * dt;
  const bool hold = (in.delta_eta_pos <= 0.0 && in.on_ground) || b.rotation == 0.0 || in.tracking_mode_on;
  const double rotationEta = s_.rotationIntegrator.update(increment, hold, 0.0, kEtaMinDeg, kEtaMaxDeg);

  // 2378-2396: plus the nose down stick (0..30 deg) on the ground; blended with the stick by the rotation weight
  const double groundStick = in.on_ground ? limit(stickEtaDeg, 0.0, 30.0) : 0.0;
  const double w = weight(b.rotation);
  return (1.0 - w) * stickEtaDeg + (rotationEta + groundStick) * w;
}

// ---------------------------------------------------------------------------------------------------------------------
// step
// ---------------------------------------------------------------------------------------------------------------------

PitchLawOutputs PitchNormalLaw::step(const PitchNormalLawInputs& in) {
  const double dt = in.time_dt;
  PitchLawOutputs out;

  const FlareOutputs flare = flareMode(in);                                                  // 1388-1477, 1667-1670
  const bool trimFrozen = s_.trimFreeze.update(flare.inFlare, in.nz_g, in.Phi_deg);         // 1479-1496

  // 1498-1512: attitude terms; the stick as an elevator deflection (ground law, integrator initial value)
  const double cosTheta = std::cos(kDegToRad * in.Theta_deg);
  Attitude att;
  att.cosRatio = cosTheta / std::cos(kDegToRad * in.Phi_deg);
  att.cosRatio33 = cosTheta / std::cos(kDegToRad * limit(in.Phi_deg, -33.0, 33.0));  // 1808-1815
  const double stickEtaDeg = kStickToEtaDeg * in.delta_eta_pos;

  s_.autoTrimMode.update(in.in_flight, in.tracking_mode_on, in.eta_trim_deg);               // 1513-1544 (unused)
  const FlightPhaseChart::Limits phase = s_.flightPhase.update(in.in_flight, in.flaps_handle_index);  // 1546-1618
  const Blends blends = groundFlightBlends(in);                                              // 1620-1660
  const double nzMax = s_.nzMax.update(phase.nzUp, 0.5, -0.5, dt, 2.0);                    // 1661-1663
  const double nzMin = s_.nzMin.update(phase.nzLo, 0.5, -0.5, dt, 0.0);                    // 1664-1666
  trimLimits(in, out);                                                                       // 1671-1685

  // shared C* terms (the spoiler gain is the radio height schedule, 1735 and copies)
  const double spoilerGain = lookup1(in.H_radio_ft, kSpoilerGainBp, kSpoilerGain);
  const CStarTerms t = s_.cstar.update(in.qk_deg_s, in.nz_g, att.cosRatio, in.V_tas_kn,
                                       std::fmin(in.spoilers_left_pos, in.spoilers_right_pos), spoilerGain, dt);
  // 1889-1894 / 2081-2086: V_ias rate, lagged 2 rad/s (high AoA protection and high speed protection)
  const double iasRate = s_.iasRate.update(in.V_ias_kn, 2.0, dt);

  // channels
  const double thetaGain = kThetaLimitGain * in.Theta_deg;  // 1697
  const double thetaMax = thetaMaxChannel(in, t, thetaGain);
  const double nzMaxCmd = s_.nzMaxChannel.update(t, nzMax, dt);    // 1744-1788, 2194-2215
  const double apCmd = autopilotChannel(in, t, att);
  const double aoaCmd = highAoaChannel(in, iasRate);
  const double flareCmd = flareChannel(in, t, att, flare.thetaCmdDeg);
  const double normalCmd = normalChannel(in, t, att, iasRate);
  const double nzMinCmd = s_.nzMinChannel.update(t, nzMin, dt);    // 2158-2209
  const double thetaMin = thetaMinChannel(t, thetaGain, dt);

  // blend weights: autopilot 2 /s both ways (1841-1869), high AoA protection in at 4 /s, out at 0.25 /s
  // (1897-1920), flare in at 1 /s, out at 0.2 /s (2036-2049)
  const double wAp = weight(s_.apBlend.update(in.any_ap_engaged, 2.0, -2.0, dt));
  const double wAoa = weight(s_.aoaBlend.update(in.high_aoa_prot_active, 4.0, -0.25, dt));
  const double wFlare = weight(s_.flareBlend.update(static_cast<double>(flare.inFlare), 1.0, -0.2, dt));
  const double law =
      apCmd * wAp + (1.0 - wAp) * (aoaCmd * wAoa + (1.0 - wAoa) * (flareCmd * wFlare + (1.0 - wFlare) * normalCmd));

  // 2222, 2290: the votes
  const double qCmd = median3(thetaMax, median3(nzMaxCmd, law, nzMinCmd), thetaMin);

  // 2291-2325: in-flight integrator; loaded with the elevator position (in flight) or the stick (on ground) while not
  // in flight at all, and in tracking mode
  const double increment = qCmd * lookup1(in.V_ias_kn, kIntegratorGainBp, kIntegratorGain) * timeStepGain(dt) * dt;
  const bool load = blends.inFlight == 0.0 || in.tracking_mode_on;
  const double initial = in.in_flight > 0.0 ? in.eta_deg : stickEtaDeg;
  const double flightEta = s_.flightIntegrator.update(increment, load, initial, kEtaMinDeg, kEtaMaxDeg);
  const double wFlight = weight(blends.inFlight);

  // 2326-2397: ground law, blended out by the in-flight weight
  const double groundEta = groundLaw(in, blends, stickEtaDeg);
  const double eta = limit(groundEta * (1.0 - wFlight) + flightEta * wFlight, kEtaMinDeg, kEtaMaxDeg);

  // 2405-2421: elevator rate limited 45 deg/s; auto trim
  out.eta_deg = s_.eta.update(eta, 45.0, -45.0, dt, 0.0);
  out.eta_trim_dot_deg_s = autoTrimRate(trimFrozen, out.eta_deg, phase);
  return out;
}

}  // namespace efcs::readable
