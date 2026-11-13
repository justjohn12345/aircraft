// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// SPD_MACH (speed on pitch: CLB, DES, OP CLB/DES, EXP CLB/DES) and SRS (takeoff / go-around speed reference) laws.
// Generated: FmgcOuterLoops.cpp 3557–3655 (SPD_MACH), 3956–4069 (SRS), FD switch 4095, AP commands 4456–4476.
//
// Both laws are instances of the same speed-on-pitch subsystem. The pitch command (delta theta) is the current energy
// rate expressed as a pitch change (acceleration in g plus flight path angle, minus the flight path angle) plus a
// limited speed error term, so pitch moves to stop the acceleration and to bring the speed back to the target:
//
//   delta theta = deg(dV/dt / g + gamma) - gamma_deg + limit(speed error / 3 kt per deg)
//
// The speed error term is limited by the energy rate (so pitch does not trade more energy than available) and, towards
// the altitude target, the command is never less than a 120 ft/min climb (or descent) towards it. When climbing
// towards the target with a decaying total energy and the speed at or below the target (+1.5 kt), the speed error
// term and the minimum V/S are dropped: the law only holds the current speed.
//
// SPD_MACH: speed target = median(VLS, V_c, VMAX), altitude target = H_c.
// SRS: speed target = V_c (the FMGC sets V2 + 10 or the go-around speed), altitude target 50000 ft (always climbing),
// and the command is the median of the speed command, the pitch change to 18 deg theta and the flight path change for
// a 120 ft/min climb: pitch limited to 18 deg unless that gives less than 120 ft/min.

#include "laws.h"

namespace fmgc::readable::laws {

namespace {

constexpr double kKnotsToMs = 0.51444444444444448;
constexpr double kKnotsToMsAirspeed = 0.5144;     // used for the acceleration estimate (slightly rounded)
constexpr double kFpmToMs = 0.0050800000000000003;
constexpr double kRadToDeg = 57.295779513082323;
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kG = 9.81;                        // m/s²
constexpr double kOneOverG = 0.1019367991845056;   // 1/9.81, s²/m

// Speeds used for flight path angles (m/s): ground speed limited to [1, 1000], TAS to [1, 2000].
constexpr double kMinSpeedMs = 1;
constexpr double kMaxGroundSpeedMs = 1000;
constexpr double kMaxTasMs = 2000;

// Acceleration estimate: complementary filter of the inertial acceleration (ground speed rate corrected for attitude)
// and the airspeed rate.
constexpr double kGroundSpeedToCas = 0.90350790290525129;  // ground speed rate scaled towards a CAS rate
constexpr double kGroundSpeedRateGain = 2;                 // 2·s/(s+2) = s/(1+s/2): rate with a 0.5 s lag
constexpr double kGroundSpeedRateC1 = 2;                   // rad/s
constexpr double kInertialHighPassC1 = 3, kInertialHighPassC2 = 0;  // 3s/(3s+1)
constexpr double kInertialHighPassC3 = 3, kInertialHighPassC4 = 1;
constexpr double kAirspeedRateC1 = 0.33333333333333331, kAirspeedRateC2 = 0;  // (s/3)/(s+1/3) = s/(3s+1)
constexpr double kAirspeedRateC3 = 1, kAirspeedRateC4 = 0.33333333333333331;

// Energy rate used to limit the speed error term: deg(0.7·dV/dt/g + gamma).
constexpr double kEnergyAccelWeight = 0.7;

// Speed error term: 3 kt per degree of pitch.
constexpr double kSpeedErrorGainDegPerKt = 0.33333333333333331;
// Hold-speed condition: speed error below 1.5 kt (0.5 deg).
constexpr double kHoldSpeedMaxErrorDeg = 0.5;
// Speed error term limits. Upper: +4 deg when the altitude target is above, else -0.93 x energy rate. Lower: the
// higher of -4 deg and -0.93 x energy rate when the target is above, else 0.6 x energy rate.
constexpr double kSpeedTermMaxClimbDeg = 4;
constexpr double kSpeedTermMinClimbDeg = -4;
constexpr double kSpeedTermEnergyGain = -0.93;
constexpr double kSpeedTermDescentLowerEnergyGain = 0.6;

// Minimum vertical speed towards the altitude target.
constexpr double kMinVsTowardsTargetFpm = 120;

// AP command: a quarter of the FD command, then the 0.15 g V/S limiter (blocks::vsLimit).
constexpr double kApGain = 0.25;

// SRS
constexpr double kSrsAltitudeTargetFt = 50000;  // always "target above": climb
constexpr double kSrsMaxThetaDeg = 18;          // pitch limit
constexpr double kSrsMinClimbFpm = 120;         // minimum climb rate
constexpr double kSrsApMinClimbGain = 0.35;     // AP share of the minimum-climb flight path change
constexpr double kSrsApMaxCommandG = 0.5;       // AP command limited to ±0.5 g equivalent (g/V rad)

/** Saturation block: hi if x > hi, else lo if x < lo, else x (NaN passes). */
inline double saturate(double x, double lo, double hi) {
  if (x > hi) {
    return hi;
  }
  if (x < lo) {
    return lo;
  }
  return x;
}

struct SpeedOnPitch {
  double altitudeErrorFt;  // altitude target - indicated altitude
  bool holdSpeed;          // speed error term and minimum V/S dropped
  double pitchCmdDeg;      // energy rate + limited speed error term
  double minVsCmdDeg;      // flight path change for 120 ft/min towards the altitude target
  double fdCmdDeg;         // pitchCmdDeg, not less than minVsCmdDeg towards the target (unless holdSpeed)
};

/** One speed-on-pitch instance (the generated code computes the identical subsystem for SPD_MACH and SRS). */
SpeedOnPitch speedOnPitch(const ap_laws_output& in, SpeedOnPitchFilters& f, double altitudeTargetFt,
                          double speedTargetKn, double law) {
  const double dt = in.time.dt;
  const auto& d = in.data;
  SpeedOnPitch out{};
  out.altitudeErrorFt = altitudeTargetFt - d.H_ind_ft;

  // flight path angle from V/S and ground speed
  double vGndMs = saturate(kKnotsToMs * d.V_gnd_kn, kMinSpeedMs, kMaxGroundSpeedMs);
  double gammaDeg = std::atan(kFpmToMs * d.H_dot_ft_min / vGndMs) * kRadToDeg;
  double gammaRad = kDegToRad * gammaDeg;
  double phiRad = kDegToRad * d.Phi_deg;

  // acceleration estimate (in g): inertial acceleration from the ground speed rate minus the gravity term of the
  // attitude correction (theta - gamma in a bank, drift), high-passed, plus the lagged airspeed rate
  double groundSpeedRate = f.groundSpeedRate.update(
      kGroundSpeedRateGain * (kGroundSpeedToCas * (kKnotsToMsAirspeed * d.V_gnd_kn)), kGroundSpeedRateC1, dt);
  double attitudeCorrectionRad =
      (kDegToRad * d.Theta_deg - kDegToRad * (kRadToDeg * std::atan(kFpmToMs * d.H_dot_ft_min / vGndMs))) *
          (1.0 - std::cos(phiRad)) +
      std::sin(phiRad) * std::sin(kDegToRad * d.Psi_magnetic_track_deg - kDegToRad * d.Psi_magnetic_deg);
  double inertialAccel = f.inertialAccelHighPass.update(
      groundSpeedRate - kG * (kDegToRad * (kRadToDeg * attitudeCorrectionRad)), kInertialHighPassC1,
      kInertialHighPassC2, kInertialHighPassC3, kInertialHighPassC4, dt);
  double airspeedRate = f.airspeedRate.update(kKnotsToMsAirspeed * d.V_ias_kn, kAirspeedRateC1, kAirspeedRateC2,
                                              kAirspeedRateC3, kAirspeedRateC4, dt);
  double accelG = (inertialAccel + airspeedRate) * kOneOverG;

  double energyRateDeg = (kEnergyAccelWeight * accelG + gammaRad) * kRadToDeg;
  double speedErrorDeg = (d.V_ias_kn - speedTargetKn) * kSpeedErrorGainDegPerKt;
  bool targetAbove = out.altitudeErrorFt > 0.0;
  out.holdSpeed = targetAbove && energyRateDeg < 0.0 && speedErrorDeg < kHoldSpeedMaxErrorDeg &&
                  in.input.vertical_law == law;

  // speed error term, limited by the energy rate
  double speedTermDeg;
  if (out.holdSpeed) {
    speedTermDeg = 0.0;
  } else {
    speedTermDeg = targetAbove ? kSpeedTermMaxClimbDeg : kSpeedTermEnergyGain * energyRateDeg;
    if (speedErrorDeg <= speedTermDeg) {
      double lower = targetAbove ? std::fmax(kSpeedTermMinClimbDeg, kSpeedTermEnergyGain * energyRateDeg)
                                 : kSpeedTermDescentLowerEnergyGain * energyRateDeg;
      speedTermDeg = speedErrorDeg >= lower ? speedErrorDeg : lower;
    }
  }
  double totalEnergyRateRad = accelG + gammaRad;
  out.pitchCmdDeg = (kRadToDeg * totalEnergyRateRad - gammaDeg) + speedTermDeg;

  // flight path change to reach 120 ft/min towards the altitude target
  double vTasMs = saturate(kKnotsToMs * d.V_tas_kn, kMinSpeedMs, kMaxTasMs);
  double towardsTarget = out.altitudeErrorFt < 0.0 ? -1.0 : (targetAbove ? 1.0 : 0.0);
  double sinGamma = saturate((towardsTarget * kMinVsTowardsTargetFpm - d.H_dot_ft_min) * kFpmToMs / vTasMs, -1.0, 1.0);
  out.minVsCmdDeg = kRadToDeg * std::asin(sinGamma);

  if (out.holdSpeed) {
    out.fdCmdDeg = out.pitchCmdDeg;
  } else if (targetAbove) {
    out.fdCmdDeg = std::fmax(out.pitchCmdDeg, out.minVsCmdDeg);
  } else {
    out.fdCmdDeg = std::fmin(out.pitchCmdDeg, out.minVsCmdDeg);
  }
  return out;
}

}  // namespace

void VerticalLaws::resetSpeed() {
  speed_.spdMach.reset();
  speed_.srs.reset();
}

void VerticalLaws::computeSpeed(const ap_laws_output& in, LawResults& out) {
  const auto& d = in.data;

  // ---- SPD_MACH: speed target within VLS..VMAX, altitude target H_c
  double speedTargetKn = blocks::median3(d.VLS_kn, in.input.V_c_kn, d.VMAX_kn);
  SpeedOnPitch spd = speedOnPitch(in, speed_.spdMach, in.input.H_c_ft, speedTargetKn, SpdMach);
  out.cmd[SpdMach].fd = spd.fdCmdDeg;
  out.cmd[SpdMach].ap = blocks::vsLimit(kApGain * spd.fdCmdDeg, d.V_tas_kn);

  // ---- SRS: speed target V_c, always climbing; median with the 18° pitch limit and the 120 ft/min minimum climb
  SpeedOnPitch srs = speedOnPitch(in, speed_.srs, kSrsAltitudeTargetFt, in.input.V_c_kn, Srs);
  double toMaxThetaDeg = kSrsMaxThetaDeg - d.Theta_deg;
  double vTasMs = saturate(kKnotsToMs * d.V_tas_kn, kMinSpeedMs, kMaxTasMs);
  double sinGamma = saturate((kSrsMinClimbFpm - d.H_dot_ft_min) * kFpmToMs / vTasMs, -1.0, 1.0);
  double minClimbCmdDeg = kRadToDeg * std::asin(sinGamma);
  out.cmd[Srs].fd = blocks::median3(toMaxThetaDeg, srs.fdCmdDeg, minClimbCmdDeg);

  double apSrs = blocks::median3(toMaxThetaDeg, kApGain * srs.fdCmdDeg, kSrsApMinClimbGain * minClimbCmdDeg);
  double apLimitDeg = kG / (d.V_tas_kn * kKnotsToMs) * kSrsApMaxCommandG * kRadToDeg;
  out.cmd[Srs].ap = std::fmax(-apLimitDeg, std::fmin(apLimitDeg, apSrs));
}

}  // namespace fmgc::readable::laws
