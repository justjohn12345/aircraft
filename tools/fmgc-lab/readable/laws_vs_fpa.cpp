// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// V/S and FPA laws. Generated: FmgcOuterLoops.cpp 3323–3473 (FPA) and 3792–3955 (V/S).
//
// Both laws have the same shape:
//   - the path command: the flight path angle error (FPA) or the V/S error turned into a flight path angle change
//     (V/S); the FD gets it as is, the AP 0.35 of it, limited to a pitch rate of 0.05 g (0.1 g during a level-off,
//     0.3 g in TCAS for V/S);
//   - two speed-on-pitch commands, towards VLS and towards VMAX (FD as is, AP 0.36 of it);
//   - speed protection: below VLS + 5 kt the lower of the path and VLS commands, above VMAX - 5 kt the higher of the
//     path and VMAX commands.
// Every block runs every frame, whichever law is active.

#include "laws.h"

namespace fmgc::readable::laws {

namespace {

constexpr double kKnotsToMs = 0.51444444444444448;
constexpr double kKnotsToMsShort = 0.5144;  // the speed laws' ktstomps gains use the rounded factor
constexpr double kFpmToMs = 0.00508;
constexpr double kRadToDeg = 57.295779513082323;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kG = 9.81;
constexpr double kOneOverG = 0.1019367991845056;

constexpr double kMinSpeedMs = 1;           // speeds limited to 1 m/s before dividing by them ...
constexpr double kMaxGroundSpeedMs = 1000;  // ... and to 1000 m/s (ground speed) ...
constexpr double kMaxTasMs = 2000;          // ... or 2000 m/s (TAS, V/S law)

// Speed-on-pitch (VLS / VMAX) laws
constexpr double kGroundSpeedToCas = 0.90350790290525129;  // GStoGS_CAS gain
constexpr double kGroundSpeedWashoutGain = 2;              // washout input gain, with C1 = 2 rad/s: d/dt of ground speed
constexpr double kGroundSpeedWashoutC1 = 2;
constexpr double kHighPassC1 = 3, kHighPassC2 = 0, kHighPassC3 = 3, kHighPassC4 = 1;  // 3s / (3s + 1)
constexpr double kLowPassC1 = 0.33333333333333331, kLowPassC2 = 0, kLowPassC3 = 1,
                 kLowPassC4 = 0.33333333333333331;  // (s/3) / (s + 1/3)
constexpr double kSpeedErrorGain = 0.33333333333333331;  // deg of pitch per kt of speed error ...
constexpr double kSpeedErrorLimitDeg = 4;                // ... limited to ±4°
constexpr double kSpeedProtectionMarginKn = 5;

// AP command fractions
constexpr double kApPathGain = 0.35;   // AP path command = 0.35 × FD command (VS_Gain / Gain_Gain_cb)
constexpr double kApSpeedGain = 0.36;  // AP speed-on-pitch command = 0.36 × FD command

// Pitch rate limits of the AP path command (g)
constexpr double kPitchRateLimitG = 0.05;
constexpr double kLevelOffPitchRateLimitG = 0.1;
constexpr double kTcasPitchRateLimitG = 0.3;

// Level-off: target set to 0 from a previous target above this (stock quirk: signed, so not from a descent target)
constexpr double kVsLevelOffFromFpm = 500;
constexpr double kFpaLevelOffFromDeg = 1;

/** Simulink Saturation block (NaN passes through). */
double saturate(double u, double lo, double hi) {
  if (u > hi) {
    return hi;
  }
  if (u < lo) {
    return lo;
  }
  return u;
}

/** Flight path angle (deg) from V/S and ground speed. */
double flightPathAngleDeg(const ap_laws_output& in) {
  double vGnd = saturate(kKnotsToMs * in.data.V_gnd_kn, kMinSpeedMs, kMaxGroundSpeedMs);
  return std::atan(kFpmToMs * in.data.H_dot_ft_min / vGnd) * kRadToDeg;
}

/**
 * Speed-on-pitch command (delta theta, deg) towards targetSpeedKn: the energy angle (flight path angle plus
 * acceleration in g) minus the flight path angle, plus the speed error / 3 limited to ±4°. Positive when too fast.
 */
double speedOnPitchCommand(const ap_laws_output& in, SpeedOnPitchFilters& f, double gammaDeg, double targetSpeedKn) {
  const double dt = in.time.dt;
  double phi = kDegToRad * in.data.Phi_deg;
  double groundAccel = f.groundSpeedRate.update(
      kGroundSpeedWashoutGain * (kGroundSpeedToCas * (kKnotsToMsShort * in.data.V_gnd_kn)), kGroundSpeedWashoutC1, dt);
  // g component along track due to attitude (pitch above the flight path in a bank, sideslip of the track)
  double attitudeDeg = kRadToDeg * ((kDegToRad * in.data.Theta_deg - kDegToRad * gammaDeg) * (1.0 - std::cos(phi)) +
                                    std::sin(phi) * std::sin(kDegToRad * in.data.Psi_magnetic_track_deg -
                                                             kDegToRad * in.data.Psi_magnetic_deg));
  double inertial = f.inertialAccelHighPass.update(groundAccel - kG * (kDegToRad * attitudeDeg), kHighPassC1, kHighPassC2,
                                              kHighPassC3, kHighPassC4, dt);
  double airspeed = f.airspeedRate.update(kKnotsToMsShort * in.data.V_ias_kn, kLowPassC1, kLowPassC2, kLowPassC3,
                                             kLowPassC4, dt);
  double accelG = (inertial + airspeed) * kOneOverG;
  double energyAngle = accelG + kDegToRad * gammaDeg;

  // The generated switches also offer limits of -0.93 / 0.6 × (0.7·accel + gamma) in deg; a constant selects ±4°.
  double speedError = (in.data.V_ias_kn - targetSpeedKn) * kSpeedErrorGain;
  double speedTerm = kSpeedErrorLimitDeg;
  if (speedError <= speedTerm) {
    speedTerm = -kSpeedErrorLimitDeg;
    if (speedError >= speedTerm) {
      speedTerm = speedError;
    }
  }
  return (kRadToDeg * energyAngle - gammaDeg) + speedTerm;
}

/**
 * Path command with VLS / VMAX speed protection: the path law's FD command and its AP command (already scaled and
 * rate limited), and the speed-on-pitch commands towards VLS and VMAX.
 */
LawCommand protectedPathCommand(const ap_laws_output& in, double pathFd, double pathAp, SpeedOnPitchFilters& vls,
                                SpeedOnPitchFilters& vmax) {
  double gammaDeg = flightPathAngleDeg(in);
  double vlsCmd = speedOnPitchCommand(in, vls, gammaDeg, blocks::vlsWithMargin(in.input.V_c_kn, in.data.VLS_kn));
  double vmaxCmd = speedOnPitchCommand(in, vmax, gammaDeg, in.data.VMAX_kn);
  blocks::ProtectedCommand p =
      blocks::speedProtection(in.input.V_c_kn, in.data.VLS_kn, in.data.V_ias_kn, in.data.VMAX_kn, pathFd, pathAp, vlsCmd,
                              kApSpeedGain * vlsCmd, vmaxCmd, kApSpeedGain * vmaxCmd, kSpeedProtectionMarginKn);
  return {p.fd, p.ap};
}

/** Updates the level-off detector of law `ownLaw` with this frame's law and target; returns whether levelling off. */
bool updateLevelOff(LevelOffDetector& d, double law, double target, double ownLaw, double fromTarget) {
  if (!d.primed) {
    d.previousLaw = law;
    d.previousTarget = target;
    d.primed = true;
  }
  d.active = ((law == ownLaw) && (d.previousLaw != ownLaw) && (target == 0.0)) ||
             ((target == 0.0) && (d.previousTarget > fromTarget)) || ((target == 0.0) && (law == ownLaw) && d.active);
  d.previousLaw = law;
  d.previousTarget = target;
  return d.active;
}

/** Largest pitch command change (deg) for a pitch rate of rateG g at the current TAS. */
double pitchRateLimitDeg(const ap_laws_output& in, double rateG) {
  double gOverV = kG / (in.data.V_tas_kn * kKnotsToMs);
  return gOverV * rateG * kRadToDeg;
}

}  // namespace

void VerticalLaws::resetVsFpa() {
  vsFpa_.fpaVls.reset();
  vsFpa_.fpaVmax.reset();
  vsFpa_.fpaLevelOff.reset();
  vsFpa_.vsVls.reset();
  vsFpa_.vsVmax.reset();
  vsFpa_.vsLevelOff.reset();
}

void VerticalLaws::computeVsFpa(const ap_laws_output& in, LawResults& out) {
  const double law = in.input.vertical_law;

  // ---- FPA: command = FPA target - current flight path angle
  {
    double fpaError = in.input.FPA_c_deg - flightPathAngleDeg(in);
    bool levelOff = updateLevelOff(vsFpa_.fpaLevelOff, law, in.input.FPA_c_deg, Fpa, kFpaLevelOffFromDeg);
    double limit = pitchRateLimitDeg(in, levelOff ? kLevelOffPitchRateLimitG : kPitchRateLimitG);
    double ap = std::fmax(-limit, std::fmin(limit, kApPathGain * fpaError));
    out.cmd[Fpa] = protectedPathCommand(in, fpaError, ap, vsFpa_.fpaVls, vsFpa_.fpaVmax);
  }

  // ---- V/S: command = flight path angle change for the V/S error, asin(dV/S / TAS)
  {
    double vTas = saturate(kKnotsToMs * in.data.V_tas_kn, kMinSpeedMs, kMaxTasMs);
    double sinGamma = (in.input.H_dot_c_fpm - in.data.H_dot_ft_min) * kFpmToMs / vTas;
    if (sinGamma > 1.0) {
      sinGamma = 1.0;
    } else if (sinGamma < -1.0) {
      sinGamma = -1.0;
    }
    double vsError = kRadToDeg * std::asin(sinGamma);
    bool levelOff = updateLevelOff(vsFpa_.vsLevelOff, law, in.input.H_dot_c_fpm, Vs, kVsLevelOffFromFpm);
    double rateG = in.input.TCAS_mode_active ? kTcasPitchRateLimitG
                                             : (levelOff ? kLevelOffPitchRateLimitG : kPitchRateLimitG);
    double limit = pitchRateLimitDeg(in, rateG);
    double ap = std::fmax(-limit, std::fmin(limit, kApPathGain * vsError));
    out.cmd[Vs] = protectedPathCommand(in, vsError, ap, vsFpa_.vsVls, vsFpa_.vsVmax);
  }
}

}  // namespace fmgc::readable::laws
