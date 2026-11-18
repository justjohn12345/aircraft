// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FAC's alpha floor detection (generated FacComputer.cpp 722-831). See alpha_floor.h.

#include "alpha_floor.h"

#include <cmath>

namespace fac::readable {

namespace {

namespace a429 = blocks::a429;

/** ELAC discrete status word 2 bit 23: high angle of attack protection active; word 1 bit 26: normal law. */
constexpr int kElacHighAoaProtBit = 23;
constexpr int kElacNormalLawBit = 26;

/** Alpha floor threshold (deg): rows CONF 0 .. FULL (config index 0..5), columns Mach 0, 0.5, 0.9, 1. */
constexpr double kMachBreakpoints[4] = {0.0, 0.5, 0.9, 1.0};
constexpr double kConfigBreakpoints[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
constexpr double kAlphaFloorDeg[24] = {
    7.6,  7.6,  6.03, 6.03,  // CONF 0
    12.5, 12.5, 12.5, 12.5,  // CONF 1
    12.7, 12.7, 12.7, 12.7,  // CONF 1+F
    13.1, 13.1, 13.1, 13.1,  // CONF 2
    12.1, 12.1, 12.1, 12.1,  // CONF 3
    11.8, 11.8, 11.8, 11.8,  // CONF FULL
};

}  // namespace

void AlphaFloor::reset() {
  restart();
  alphaFiltered_ = 0.0;
}

void AlphaFloor::restart() {
  alphaFilter_.reset();
  threshold_.reset();
  speedRate_.reset();
  previousHalfCas_ = 0.0;
  chartActive_ = false;
  phase_ = AlphaFloorPhase::None;
  latched_ = false;
}

double AlphaFloor::update(const fac_inputs& in, const Sensors& s) {
  const base_fac_bus_inputs& b = in.bus_inputs;
  const double dt = in.time.dt;

  // 722-724: alpha, lag 0.5 rad/s
  alphaFiltered_ = alphaFilter_.update(static_cast<double>(s.adr.alphaDeg), kAlphaFilterC1, dt);

  // 725-735: either ELAC's high angle of attack protection active; the threshold for Mach and configuration at 1 deg/s
  const bool highAoaProt = a429::bit(b.elac_1_bus.discrete_status_word_2, kElacHighAoaProtBit) != 0U ||
                           a429::bit(b.elac_2_bus.discrete_status_word_2, kElacHighAoaProtBit) != 0U;
  const double threshold = threshold_.update(
      blocks::lookup2(static_cast<double>(s.adr.mach), static_cast<double>(s.slatFlap.configIndex), kMachBreakpoints,
                      kConfigBreakpoints, kAlphaFloorDeg),
      kAlphaThresholdRateDegS, kAlphaThresholdRateDegS, dt);

  // 736-739: the rate of half the CAS (a discrete derivative of 0.5 x CAS, so kt/s / 2), lag 0.4 rad/s
  const double halfCas = 0.5 * s.adr.casKn;
  const double speedRate = speedRate_.update((halfCas - previousHalfCas_) / dt, kSpeedRateFilterC1, dt);
  previousHalfCas_ = halfCas;  // Delay_DSTATE, written at the end of the frame (1561); nothing reads it in between

  // 740-799: flight phase. The radio height is the own FMGC's word (200 ft when failed, not the selected one).
  const double ra = a429::isNotFailed(b.fmgc_own_bus.fg_radio_height_ft) ? b.fmgc_own_bus.fg_radio_height_ft.Data
                                                                         : kRaFailedDefaultFt;
  bool inhibited;
  if (!chartActive_) {
    chartActive_ = true;
    phase_ = AlphaFloorPhase::Landed;
    inhibited = true;
  } else {
    switch (phase_) {
      case AlphaFloorPhase::Flying:
        if (ra < kAlphaFloorInhibitRaFt) {
          phase_ = AlphaFloorPhase::Landing100ft;
        } else if (s.onGround) {
          phase_ = AlphaFloorPhase::Landed;
        }
        inhibited = phase_ != AlphaFloorPhase::Flying;
        break;
      case AlphaFloorPhase::Landed:
        if (!s.onGround) phase_ = AlphaFloorPhase::Takeoff100ft;
        inhibited = phase_ == AlphaFloorPhase::Landed;
        break;
      case AlphaFloorPhase::Landing100ft:
        if (ra > kAlphaFloorInhibitRaFt) {
          phase_ = AlphaFloorPhase::Flying;
        } else if (s.onGround) {
          phase_ = AlphaFloorPhase::Landed;
        }
        inhibited = phase_ != AlphaFloorPhase::Flying;
        break;
      default:  // Takeoff100ft (and an unknown state, as generated)
        if (s.onGround) {
          phase_ = AlphaFloorPhase::Landed;
        } else if (ra > kAlphaFloorInhibitRaFt) {
          phase_ = AlphaFloorPhase::Flying;
        }
        inhibited = phase_ == AlphaFloorPhase::Landed;
        break;
    }
  }

  // 800-831: the latch
  auto normalLaw = [](const base_arinc_429& w) {
    return a429::isNotFailed(w) && a429::bit(w, kElacNormalLawBit) != 0U;
  };
  const bool anyNormalLaw = normalLaw(b.elac_1_bus.discrete_status_word_1) || normalLaw(b.elac_2_bus.discrete_status_word_1);
  bool set = false;
  if (!inhibited && s.adr.mach < kAlphaFloorMaxMach) {
    // anticipation: the threshold lowered by the deceleration term, in CONF 3 and FULL only (else limited to 0)
    const double lowest = s.slatFlap.configIndex >= 4.0F ? -kMaxAnticipationDeg : 0.0;
    set = alphaFiltered_ > threshold + std::fmin(std::fmax(speedRate, lowest), 0.0) && anyNormalLaw;
  }
  if (set) {
    latched_ = true;
  } else if (inhibited || !highAoaProt || !anyNormalLaw) {
    latched_ = false;
  }
  return alphaFiltered_;
}

}  // namespace fac::readable
