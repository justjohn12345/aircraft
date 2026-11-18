// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The rest of the FAC's flight envelope function (generated FacComputer.cpp 690-724, 976-1100):
//
//   estimated sideslip   from the lateral load factor and CAS (side force derivative of a 70 t aircraft), lag 4 rad/s
//   sideslip target      the engine-out "beta target": alpha x N1 difference and N1 difference terms over CAS^2; sent
//                        with SSM NCD (never shown), and the FMGC sends constant 0 N1 words (finding F12)
//   gamma_a              air path angle: pitch - alpha x cos(roll) (pitch on the ground), lag 1 rad/s
//   gamma_t              total (energy) path angle: long. load factor (deg) - nz x alpha, lag 1 rad/s
//   low energy warning   "SPEED SPEED SPEED": in CONF 2 or more between 100 and 2000 ft RA, in flight, an ELAC in
//                        normal law, no alpha floor, not in SRS take-off or the first 8 s of SRS go-around, when the
//                        speed is below VLS - 10 kt, or below VLS with the filtered alpha above a threshold that falls
//                        with the deceleration (gamma_t - gamma_a) and rises with the flight path angle; confirmed 0.5
//                        s, then held at least 3 s
//   pitch-pitch warning  "PITCH PITCH": pitch + pitch rate above 10 deg below 20 ft RA, no AP, no TOGA go-around
//                        lever setting, radio height valid (not NCD for 1 s)

#pragma once

#include "FacComputer_types.h"
#include "fac_blocks.h"
#include "sensors.h"

namespace fac::readable {

/** Estimated sideslip: valid from 30 kt CAS; lag (rad/s). */
constexpr float kSideslipMinCasKn = 30.0F;
constexpr double kSideslipLagC1 = 4.0;
/** Sideslip target: N1 difference and alpha lags (rad/s), gains (float N1 gain as in the model). */
constexpr double kBetaTargetLagC1 = 0.5;
constexpr double kBetaTargetAlphaN1Gain = 0.0302;
constexpr float kBetaTargetN1Gain = 0.1101F;
/** Low energy: radio height window (ft), configuration (index 3 = CONF 2), VLS margin (kt), confirm and hold (s). */
constexpr float kLowEnergyMinRaFt = 100.0F, kLowEnergyMaxRaFt = 2000.0F;
constexpr float kLowEnergyMinConfig = 3.0F;
constexpr double kLowEnergyVlsMarginKn = 10.0;
constexpr double kLowEnergyConfirmSeconds = 0.5, kLowEnergyHoldSeconds = 3.0;
constexpr double kGoAroundInhibitSeconds = 8.0;
/** Pitch-pitch: pitch + pitch rate threshold (deg), radio height (ft), RA NCD confirm (s). */
constexpr double kPitchPitchThresholdDeg = 10.0;
constexpr float kPitchPitchMaxRaFt = 20.0F;
constexpr double kRaNcdConfirmSeconds = 1.0;

struct FlightPathOutputs {
  double estimatedBetaDeg, betaTargetDeg, gammaADeg, gammaTDeg;
  bool lowEnergy, pitchPitch;
};

class FlightPath {
 public:
  void reset();

  /** Sideslip estimate and target (generated 690-724). */
  void updateSideslip(const fac_inputs& in, const Sensors& s);

  /** Flight path angles and the two warnings (976-1100); needs VLS and the filtered alpha. */
  void updateWarnings(const fac_inputs& in, const Sensors& s, double alphaFilteredDeg, double vls);

  const FlightPathOutputs& outputs() const { return out_; }

  // state access for the checker
  const blocks::LagFilter<double>& sideslipLag() const { return sideslip_; }
  const blocks::LagFilter<float>& n1DiffRightLag() const { return n1RightMinusLeft_; }
  const blocks::LagFilter<double>& betaAlphaLag() const { return betaAlpha_; }
  const blocks::LagFilter<float>& n1DiffLeftLag() const { return n1LeftMinusRight_; }
  const blocks::LagFilter<double>& gammaALag() const { return gammaA_; }
  const blocks::LagFilter<double>& nzLag() const { return nz_; }
  const blocks::LagFilter<double>& gammaTLag() const { return gammaT_; }
  const blocks::LagFilter<double>& inertialFpaLag() const { return inertialFpa_; }
  const blocks::LagFilter<double>& decelerationLag() const { return deceleration_; }
  const blocks::ConfirmNode& raNcdConfirm() const { return raNcd_; }
  const blocks::Monostable& goAroundInhibit() const { return goAround_; }
  const blocks::ConfirmNode& lowEnergyConfirm() const { return lowEnergyConfirm_; }
  const blocks::Monostable& lowEnergyHold() const { return lowEnergyHold_; }

 private:
  blocks::LagFilter<double> sideslip_;          // sf_LagFilter_d
  blocks::LagFilter<float> n1RightMinusLeft_;   // sf_LagFilter_k
  blocks::LagFilter<double> betaAlpha_;         // sf_LagFilter_f
  blocks::LagFilter<float> n1LeftMinusRight_;   // sf_LagFilter_d5
  blocks::LagFilter<double> gammaA_;            // sf_LagFilter_m
  blocks::LagFilter<double> nz_;                // sf_LagFilter_c4
  blocks::LagFilter<double> gammaT_;            // sf_LagFilter_mt
  blocks::LagFilter<double> inertialFpa_;       // sf_LagFilter_fi
  blocks::LagFilter<double> deceleration_;      // sf_LagFilter_e
  blocks::ConfirmNode raNcd_;                   // sf_MATLABFunction_o
  blocks::Monostable goAround_;                 // sf_MATLABFunction_ns
  blocks::ConfirmNode lowEnergyConfirm_;        // sf_MATLABFunction_n
  blocks::Monostable lowEnergyHold_;            // sf_MATLABFunction_a
  FlightPathOutputs out_{};
};

}  // namespace fac::readable
