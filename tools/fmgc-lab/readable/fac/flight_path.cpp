// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's sideslip estimate and target, flight path angles, low energy and pitch-pitch warnings (generated
// FacComputer.cpp 690-724, 976-1100). See flight_path.h.

#include "flight_path.h"

#include <cmath>

namespace fac::readable {

namespace {

namespace a429 = blocks::a429;

constexpr double kRadToDeg = 57.295779513082323;
constexpr double kKtToMs = 0.5144;
constexpr double kFtMinToMs = 0.00508;

/** FMGC discrete word 1 bits: SRS take-off 15, SRS go-around 16, TOGA lever for go-around 29; ATS word: alpha floor 23. */
constexpr int kFmgcSrsTakeoffBit = 15, kFmgcSrsGoAroundBit = 16, kFmgcTogaLeverBit = 29, kFmgcAlphaFloorBit = 23;
/** ELAC discrete status word 1: bits 20 and 26 (see yaw_functions.cpp). */
constexpr int kElacYawDamperBit = 20, kElacNormalLawBit = 26;

/** Low energy alpha threshold terms: flight path angle (deg) -> deg; deceleration term (gamma_t - gamma_a, deg) ->
 *  deg per unit of the flap gain; flap gain and base threshold against the flap surface angle (deg). */
constexpr double kFpaDeg[4] = {-10.0, -3.0, 3.0, 10.0};
constexpr double kFpaTermDeg[4] = {-3.0, 0.0, 0.0, 3.0};
constexpr double kDecelDeg[4] = {-2.0, -1.0, -0.5, 0.0};
constexpr double kDecelTerm[4] = {-0.66666666666666663, -0.33333333333333331, -0.26666666666666666, 0.0};
constexpr double kFlapSurfaceDeg[4] = {15.0, 20.0, 35.0, 40.0};
constexpr double kDecelFlapGain[4] = {1.9, 1.9, 2.5, 2.5};
constexpr double kBaseAlphaDeg[4] = {10.7, 9.8, 9.5, 9.5};

bool bit(const base_arinc_429& w, int n) { return a429::bit(w, n) != 0U; }

}  // namespace

void FlightPath::reset() {
  sideslip_.reset();
  n1RightMinusLeft_.reset();
  betaAlpha_.reset();
  n1LeftMinusRight_.reset();
  gammaA_.reset();
  nz_.reset();
  gammaT_.reset();
  inertialFpa_.reset();
  deceleration_.reset();
  raNcd_.reset();
  goAround_.reset();
  lowEnergyConfirm_.reset();
  lowEnergyHold_.reset();
}

void FlightPath::updateSideslip(const fac_inputs& in, const Sensors& s) {
  const double dt = in.time.dt;
  // 690-698: sideslip that gives the measured lateral load factor: ny g / (q S / m x CY_beta), with q from CAS (m/s),
  // S = 122 m2, m = 70 t, CY_beta = -0.62 /rad; 0 below 30 kt
  const double v = s.adr.casKn * kKtToMs;
  double beta = 0.0;
  if (s.adr.casKn >= kSideslipMinCasKn) {
    beta = s.ir.nyG * 9.81 / (v * v * 0.6125 * 122.0 / (70000.0 * v) * -0.62 * v) * 180.0 / 3.1415926535897931;
  }
  out_.estimatedBetaDeg = sideslip_.update(beta, kSideslipLagC1, dt);

  // 699-720: sideslip target for an engine failure, from the (lagged) N1 difference of the own FMGC's N1 words
  const base_fmgc_b_bus& fmgc = in.bus_inputs.fmgc_own_bus;
  const float n1RightMinusLeft = n1RightMinusLeft_.update(
      fmgc.n1_right_percent.Data - fmgc.n1_left_percent.Data, kBetaTargetLagC1, dt);
  const double alpha = betaAlpha_.update(blocks::limit(s.adr.alphaDeg, -5.0, 5.0), kBetaTargetLagC1, dt);
  const float n1LeftMinusRight = n1LeftMinusRight_.update(
      fmgc.n1_left_percent.Data - fmgc.n1_right_percent.Data, kBetaTargetLagC1, dt);
  const double cas = blocks::limit(s.adr.casKn, 1.0, 1000.0);
  out_.betaTargetDeg =
      (alpha * n1LeftMinusRight * kBetaTargetAlphaN1Gain + kBetaTargetN1Gain * n1RightMinusLeft) / cas / cas * kRadToDeg;
}

void FlightPath::updateWarnings(const fac_inputs& in, const Sensors& s, double alphaFilteredDeg, double vls) {
  const double dt = in.time.dt;
  const base_fac_discrete_inputs& d = in.discrete_inputs;
  const base_fac_bus_inputs& b = in.bus_inputs;
  const base_fmgc_b_bus& fmgc = *s.fmgc;

  // 1031-1043: pitch-pitch: tail strike risk on the ground and just after lift-off / in the flare
  const float ra = a429::rawValue(s.radioHeight);
  const bool raNcd = raNcd_.update(s.radioHeight.SSM == static_cast<uint32_t>(SignStatusMatrix::NoComputedData), dt,
                                   true, kRaNcdConfirmSeconds);
  out_.pitchPitch = (static_cast<double>(s.ir.thetaDeg) + s.ir.thetaDotDegS > kPitchPitchThresholdDeg) &&
                    (!d.ap_opp_engaged && !d.ap_own_engaged) && !bit(fmgc.discrete_word_1, kFmgcTogaLeverBit) &&
                    (ra <= kPitchPitchMaxRaFt && !s.allRaFailed && !raNcd);

  // 1044-1058: gamma_a = pitch - alpha cos(roll) (pitch for alpha on the ground); gamma_t = nx (deg) - nz alpha,
  // nz - 1 lagged and limited to +-0.8 g
  const double alphaOrPitch = s.onGround ? s.ir.thetaDeg : s.adr.alphaDeg;
  out_.gammaADeg =
      gammaA_.update(s.ir.thetaDeg - std::cos(0.017453292519943295 * s.ir.phiDeg) * alphaOrPitch, 1.0, dt);
  const double nzMinusOne = blocks::limit(nz_.update(s.ir.nzG + -1.0, 1.0, dt), -0.8, 0.8);
  out_.gammaTDeg = gammaT_.update(kRadToDeg * s.ir.nxG - (nzMinusOne + 1.0) * alphaOrPitch, 1.0, dt);

  // 1061-1083: low energy alpha threshold: base (flaps) + flight path angle term + deceleration term (flaps gain,
  // -5..0 deg). The inertial flight path angle is V/S over ground speed (as an angle, not an arctangent).
  const double groundSpeed = blocks::limit(s.ir.groundSpeedKn, 1.0, 500.0);
  const double fpa = inertialFpa_.update(kRadToDeg * (kFtMinToMs * s.ir.verticalSpeed / (kKtToMs * groundSpeed)), 0.5, dt);
  const double decel = deceleration_.update(out_.gammaTDeg - out_.gammaADeg, 1.0, dt);
  const double flaps = s.slatFlap.flapSurfaceDeg;
  const double decelTerm = blocks::limit(
      blocks::lookup1(decel, kDecelDeg, kDecelTerm) * blocks::lookup1(flaps, kFlapSurfaceDeg, kDecelFlapGain), -5.0, 0.0);
  const double alphaThreshold =
      (blocks::lookup1(fpa, kFpaDeg, kFpaTermDeg) + decelTerm) + blocks::lookup1(flaps, kFlapSurfaceDeg, kBaseAlphaDeg);

  // 1084-1100: low energy warning
  const bool alphaFloor = bit(fmgc.ats_discrete_word, kFmgcAlphaFloorBit);
  const bool goAroundInhibit =
      goAround_.update(bit(fmgc.discrete_word_1, kFmgcSrsGoAroundBit), dt, true, false, kGoAroundInhibitSeconds);
  const bool srsTakeoff = bit(fmgc.discrete_word_1, kFmgcSrsTakeoffBit);
  auto elacNormal = [](const base_elac_out_bus& e, bool healthy) {
    return bit(e.discrete_status_word_1, kElacYawDamperBit) && healthy && bit(e.discrete_status_word_1, kElacNormalLawBit);
  };
  const bool elac1 = elacNormal(b.elac_1_bus, d.elac_1_healthy);
  const bool elac2 = elacNormal(b.elac_2_bus, d.elac_2_healthy);
  const double cas = s.adr.casKn;
  const bool slow = ((alphaThreshold < alphaFilteredDeg) && (vls > cas)) || (vls + -kLowEnergyVlsMarginKn > cas);
  const bool condition = slow && !s.onGround && (s.slatFlap.configIndex >= kLowEnergyMinConfig) &&
                         (ra > kLowEnergyMinRaFt) && (ra < kLowEnergyMaxRaFt) && !alphaFloor && !goAroundInhibit &&
                         (!srsTakeoff && (elac1 || elac2));
  const bool confirmed = lowEnergyConfirm_.update(condition, dt, true, kLowEnergyConfirmSeconds);
  const bool held = lowEnergyHold_.update(confirmed, dt, true, true, kLowEnergyHoldSeconds);
  out_.lowEnergy = !s.allRaFailed && (confirmed || held);
}

}  // namespace fac::readable
