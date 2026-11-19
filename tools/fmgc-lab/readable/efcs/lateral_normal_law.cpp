// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable LateralNormalLaw. Line numbers: generated LateralNormalLaw.cpp. Parameters: docs/params-EfcsLaws.txt.

#include "lateral_normal_law.h"

#include <cmath>

namespace efcs::readable {

namespace {

constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.295779513082323;

// BankAngleProtection tables (360-368): roll rate (deg/s) fed back against the bank angle (deg)
constexpr double kBankProtBp[9] = {-90.0, -71.0, -66.0, -33.0, 0.0, 33.0, 66.0, 71.0, 90.0};  // normal
constexpr double kBankProtHighAoaBp[9] = {-90.0, -66.0, -45.0, -33.0, 0.0, 33.0, 45.0, 66.0, 90.0};
constexpr double kBankProtTable[9] = {20.0, 20.0, 15.0, 0.0, 0.0, 0.0, -15.0, -20.0, -20.0};
constexpr double kBankProtHighSpeedBp[5] = {-50.0, -40.0, 0.0, 40.0, 50.0};
constexpr double kBankProtHighSpeedTable[5] = {20.0, 15.0, 0.0, -15.0, -20.0};
// ScheduledGain (416-417): roll attitude law gain against the frame time
constexpr double kDtBp[5] = {0.0, 0.06, 0.1, 0.2, 1.0};
constexpr double kDtGain[5] = {1.1, 1.0, 0.6, 0.3, 0.1};
// ScheduledGain1 (472-473): ground yaw rate feedback gain against CAS
constexpr double kGroundYawBp[4] = {0.0, 40.0, 100.0, 180.0};
constexpr double kGroundYawGain[4] = {0.0, 0.0, 1.4, 1.4};

/**
 * Rudder pedal feed-forward gain against max(CAS, 60) kt (260-289): a MATLAB interp1 of [0 120 150 380] ->
 * [-15 -15 -15 -2], 0 above 380 kt. The specialised code returns the exact table value at r == 0 / r == 1 and -15 when
 * both ends are -15, so any interval holding v gives the same result.
 */
double pedalGain(double v) {
  static const int16_t bp[4] = {0, 120, 150, 380};
  static const int8_t gain[4] = {-15, -15, -15, -2};
  if (!(v <= 380.0)) return 0.0;
  int i = 1;  // 1-based right end of the interval [bp[i - 1], bp[i]]
  while (i < 3 && v >= bp[i]) i++;
  const double r = (v - static_cast<double>(bp[i - 1])) / static_cast<double>(bp[i] - bp[i - 1]);
  if (r == 0.0) return -15.0;
  if (r == 1.0) return gain[i];
  if (gain[i] == -15) return -15.0;
  return (1.0 - r) * -15.0 + r * static_cast<double>(gain[i]);
}

/**
 * Roll mode natural frequency omega_0 (rad/s) against CAS (322-353): interp1 of [-1 0 120 320 400] -> [1 1 1.2 2 2],
 * clamped outside. The binary search is the generated one (it decides the interval for NaN).
 */
double rollFrequency(double vIas) {
  static const int16_t bp[5] = {-1, 0, 120, 320, 400};
  static const double value[5] = {1.0, 1.0, 1.2, 2.0, 2.0};
  if (vIas > 400.0) return 2.0;
  if (vIas < -1.0) return 1.0;
  int high = 5, low = 0, lowPlus1 = 2;
  while (high > lowPlus1) {
    const int mid = ((low + high) + 1) >> 1;
    if (vIas >= bp[mid - 1]) {
      low = mid - 1;
      lowPlus1 = mid + 1;
    } else {
      high = mid;
    }
  }
  const double r = (vIas - static_cast<double>(bp[low])) / static_cast<double>(bp[low + 1] - bp[low]);
  if (r == 0.0) return value[low];
  if (r == 1.0) return value[low + 1];
  if (value[low + 1] == value[low]) return value[low];
  return (1.0 - r) * value[low] + value[low + 1] * r;
}

}  // namespace

void LateralNormalLaw::init() {
  previousXiCommand_ = 0.0;
  previousRollRateDemand_ = 0.0;
  reloadBankCommand_ = true;
}

void LateralNormalLaw::reset() {
  init();
  modeChartActive_ = false;
  mode_ = Mode::None;
  inFlightRate_.reset();
  pedalFeedForwardRate_.reset();
  apEngagedRate_.reset();
  xiOrderRate_.reset();
  airborneRate_.reset();
}

int LateralNormalLaw::updateMode(const LateralNormalLawInputs& in) {
  if (!modeChartActive_) {
    modeChartActive_ = true;
    mode_ = Mode::Ground;
    return 0;
  }
  if (mode_ == Mode::Flight) {
    if (in.on_ground) {
      mode_ = Mode::Ground;
      return 0;
    }
    return 1;
  }
  // ground mode: to flight with pitch above 8 deg off the ground, or above 400 ft RA
  if ((!in.on_ground && in.Theta_deg > 8.0) || in.H_radio_ft > 400.0) {
    mode_ = Mode::Flight;
    return 1;
  }
  return 0;
}

double LateralNormalLaw::bankAngleCommand(const LateralNormalLawInputs& in, double inFlightBlend, double omega0) {
  // roll rate demand: 15 deg/s at full stick, plus its derivative / omega_0 (356-358)
  const double demand = 15.0 * in.delta_xi_pos;
  const double demandRate = (demand - previousRollRateDemand_) / in.time_dt;
  previousRollRateDemand_ = demand;  // stored at the very end in the generated code (489); not read in between

  // bank angle protection: a rate back towards 33 deg (0 with high speed protection, 45 with high AoA) (359-368)
  double rate;
  if (in.high_speed_prot_active) {
    rate = blocks::lookup1(in.Phi_deg, kBankProtHighSpeedBp, kBankProtHighSpeedTable);
  } else if (in.high_aoa_prot_active) {
    rate = blocks::lookup1(in.Phi_deg, kBankProtHighAoaBp, kBankProtTable);
  } else {
    rate = blocks::lookup1(in.Phi_deg, kBankProtBp, kBankProtTable);
  }
  rate += 1.0 / omega0 * demandRate + demand;
  rate = blocks::limit(rate, -15.0, 15.0);  // (371-375)

  // while the previous aileron command is saturated (+-25 deg), the demand cannot exceed the actual roll rate in that
  // direction (377-383)
  double upper = 15.0, lower = -15.0;
  if (previousXiCommand_ >= 25.0) {
    lower = in.pk_deg_s;
  } else if (previousXiCommand_ <= -25.0) {
    upper = in.pk_deg_s;
  }
  const double increment = std::fmin(upper, std::fmax(lower, rate * inFlightBlend)) * 1.0 * in.time_dt;

  // the integrator follows the bank angle on the ground, in tracking mode and with an AP engaged (388-402)
  const bool follow = inFlightBlend == 0.0 || in.tracking_mode_on || in.any_ap_engaged;
  reloadBankCommand_ = follow || reloadBankCommand_;
  if (reloadBankCommand_) bankCommand_ = in.Phi_deg - increment;
  bankCommand_ = blocks::limit(increment + bankCommand_, -67.0, 67.0);

  // with an AP engaged, its bank command (the actual bank in tracking mode) (404-412)
  if (in.any_ap_engaged) return in.tracking_mode_on ? in.Phi_deg : in.ap_phi_c_deg;
  return bankCommand_;
}

LateralLawOutputs LateralNormalLaw::step(const LateralNormalLawInputs& in) {
  LateralLawOutputs out;

  // flight / ground blend, 0.5 s (230-258)
  const double inFlightBlend = inFlightRate_.update(updateMode(in), 2.0, -2.0, in.time_dt, 0.0);
  const double directXi = -25.0 * in.delta_xi_pos;  // (259)

  // rudder pedal feed-forward (260-294): computed and rate limited, but the output is overwritten before use
  {
    const double v = std::fmax(in.V_ias_kn, 60.0);
    const double gain = pedalGain(v);
    const double vMs = v * 0.5144;
    pedalFeedForwardRate_.update(0.814 / std::sqrt(1.3734E+6 / (149.45000000000002 * (vMs * vMs))) *
                                     (gain * in.delta_zeta_pos),
                                 5.0, -5.0, in.time_dt, 0.0);
  }

  // normal law weight: in flight or AP engaged (295-316); the direct law takes the rest
  const double apBlend = apEngagedRate_.update(static_cast<double>(in.any_ap_engaged), 2.0, -2.0, in.time_dt, 0.0);
  const double lawWeight = blocks::limit(blocks::limit(inFlightBlend + apBlend, 0.0, 1.0), 0.0, 1.0);
  const double directPart = (1.0 - lawWeight) * directXi;

  // roll model at the current dynamic pressure (317-355): q = 0.6125 v^2 (v = CAS >= 80 kt, m/s), roll control power
  // L_xi, roll damping L_p
  const double vMs = std::fmax(in.V_ias_kn, 80.0) * 0.5144;
  const double dynamicPressure = vMs * vMs * 0.6125;
  const double qSb = dynamicPressure * 122.0 * 17.9;
  const double lXi = qSb * -0.090320788790706555 / 1.0E+6;
  const double omega0 = rollFrequency(in.V_ias_kn);
  const double kPhi = -(omega0 * omega0) / lXi;

  const double phiCommand = bankAngleCommand(in, inFlightBlend, omega0);

  // roll attitude law (414-433): aileron command from bank error and roll rate, gain scheduled with dt
  const double dtGain = blocks::lookup1(in.time_dt, kDtBp, kDtGain);
  const double lP = dynamicPressure / vMs * 122.0 * 320.40999999999997;
  previousXiCommand_ = ((-(lP * -0.487 / 1.0E+6 + 1.414 * omega0) / lXi * (kDegToRad * in.pk_deg_s) +
                         kPhi * (kDegToRad * in.Phi_deg)) +
                        kDegToRad * phiCommand * -kPhi) *
                       dtGain * kRadToDeg;
  const double xiCommand = blocks::limit(previousXiCommand_, -25.0, 25.0);
  out.xi_deg = xiOrderRate_.update(xiCommand * lawWeight + directPart, 50.0, -50.0, in.time_dt, 0.0);

  // yaw (434-488): turn coordination / yaw damping in flight: the yaw rate minus the turn rate of the commanded bank
  // (g sin(phi_c) cos(theta) / TAS, TAS 100-1000 kt)
  const double sinPhiCommand = std::sin(kDegToRad * phiCommand);
  const double cosTheta = std::cos(kDegToRad * in.Theta_deg);
  const double vTas = blocks::limit(in.V_tas_kn, 100.0, 1000.0);
  const double yawRateError =
      in.r_deg_s - 9.81 * sinPhiCommand * cosTheta / (0.51444444444444448 * vTas) * kRadToDeg;
  double yawDamper = -(std::sqrt(qSb * 0.0035255650890285459 / 4.47E+6) * 1.414 +
                       lP * -0.10100220381291185 / 4.47E+6) /
                     (qSb * -0.018936822384138473 / 4.47E+6) * (kDegToRad * yawRateError) * kRadToDeg;
  yawDamper = blocks::limit(yawDamper, -25.0, 25.0);

  // airborne blend, 0.5 s; on the ground a yaw rate feedback (gain 0 below 40 kt, 1.4 from 100 kt)
  const double airborne = blocks::limit(
      airborneRate_.update(static_cast<double>(!in.on_ground), 2.0, -2.0, in.time_dt, 0.0), 0.0, 1.0);
  const double flightYaw = yawDamper * airborne;
  const double groundYawRate =
      blocks::limit(blocks::lookup1(in.V_ias_kn, kGroundYawBp, kGroundYawGain) * in.r_deg_s, -25.0, 25.0);
  const double groundYaw = (1.0 - airborne) * groundYawRate;
  const double apBeta = in.any_ap_engaged ? in.ap_beta_c_deg : 0.0;
  out.zeta_deg = (flightYaw + groundYaw) + apBeta;

  reloadBankCommand_ = false;
  return out;
}

}  // namespace efcs::readable
