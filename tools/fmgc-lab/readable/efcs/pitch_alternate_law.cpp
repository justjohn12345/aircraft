// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchAlternateLaw (generated PitchAlternateLaw.cpp 393-899). See pitch_alternate_law.h. Line numbers in the
// comments are PitchAlternateLaw.cpp's.

#include "pitch_alternate_law.h"

#include <cmath>

namespace efcs::readable {

using namespace pitch_blocks;

namespace {

/** Low speed stability: target speed (kt) by flap lever index (a VLS-like table). */
constexpr double kFlapsBp[6] = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
constexpr double kLowSpeedKn[6] = {180.0, 145.0, 130.0, 120.0, 120.0, 115.0};
/** Integrator gain by flap lever index. */
constexpr double kFlapsGain[6] = {0.3, 0.5, 0.8, 1.0, 1.0, 1.0};

}  // namespace

void PitchAlternateLaw::init() {
  // 393-405: the Delay states (all 0) and the integrator to load on the next step
  s_.cstar.init();
  s_.nzMaxChannel.reset();
  s_.stickChannel.reset();
  s_.nzMinChannel.reset();
  s_.integrator.requestLoad();
}

void PitchAlternateLaw::reset() {
  // 407-439: init() plus every filter / rate limiter / chart (the integrator state is reloaded, not reset)
  init();
  s_.trimFreeze.reset();
  s_.flightPhase.reset();
  s_.nzMax.reset();
  s_.cstar.reset();
  s_.stick.reset();
  s_.nzMin.reset();
  s_.eta.reset();
  s_.autoTrimMode.reset();
}

PitchLawOutputs PitchAlternateLaw::step(const PitchAlternateLawInputs& in) {
  const double dt = in.time_dt;
  PitchLawOutputs out;

  // 477-495: auto trim freeze (the in-flare inhibit of the normal law is constant false here)
  const bool trimFrozen = s_.trimFreeze.update(false, in.nz_g, in.Phi_deg);

  // 497-504: attitude terms
  const double cosTheta = std::cos(kDegToRad * in.Theta_deg);
  const double cosRatio = cosTheta / std::cos(kDegToRad * in.Phi_deg);
  const double cosRatio33 = cosTheta / std::cos(kDegToRad * limit(in.Phi_deg, -33.0, 33.0));  // 664-686

  // 515-587: flight phase -> trim rate and load factor limits (rate limited 0.5 g/s, 589-592 / 731-734)
  const FlightPhaseChart::Limits phase = s_.flightPhase.update(in.in_flight, in.flaps_handle_index);
  const double nzMax = s_.nzMax.update(phase.nzUp, 0.5, -0.5, dt, 2.0);
  const double nzMin = s_.nzMin.update(phase.nzLo, 0.5, -0.5, dt, 0.0);

  // shared C* terms: the spoiler term has a fixed -30 gain (758, 767, 782)
  const CStarTerms t = s_.cstar.update(in.qk_deg_s, in.nz_g, cosRatio, in.V_tas_kn,
                                       std::fmin(in.spoilers_left_pos, in.spoilers_right_pos), -30.0, dt);

  // 639-686: stick (rate limited 1 /s) load factor demand, plus the speed stabilities when available (else 0): low
  // speed, 0.04 g/kt below the flap table speed (0 .. -0.5 g); high speed, 0.1 g/kt above min(350 kt, 0.82 x
  // V_ias / mach) (0 .. 0.5 g)
  const double stick = s_.stick.update(in.delta_eta_pos, 1.0, -1.0, dt, 0.0);
  double lowSpeed = 0.0;
  if (in.stabilities_available) {
    lowSpeed = -0.04 * (lookup1(in.flaps_handle_index, kFlapsBp, kLowSpeedKn) - in.V_ias_kn);
  }
  double highSpeed = in.V_ias_kn * 0.82 / in.mach;
  if (in.stabilities_available) {
    highSpeed = -0.1 * (std::fmin(350.0, highSpeed) - in.V_ias_kn);
  } else {
    highSpeed = 0.0;
  }
  const double stickDemand =
      ((loadDemand(stick) + limit(lowSpeed, -0.5, 0.0)) + limit(highSpeed, 0.0, 0.5)) + cosRatio33;

  // the three channels and the vote (594-810)
  const double nzMaxCmd = s_.nzMaxChannel.update(t, nzMax, dt);
  const double stickCmd = s_.stickChannel.update(t, stickDemand, dt);
  const double nzMinCmd = s_.nzMinChannel.update(t, nzMin, dt);
  const double qCmd = median3(nzMaxCmd, stickCmd, nzMinCmd) * lookup1(in.flaps_handle_index, kFlapsBp, kFlapsGain);

  // 811-834: integrator (limits -30 .. 17 deg), loaded with the elevator position (in flight) or the stick (on
  // ground) while not in flight and in tracking mode
  const double increment = qCmd * timeStepGain(dt) * dt;
  const bool load = in.in_flight == 0.0 || in.tracking_mode_on;
  const double initial = in.in_flight > 0.0 ? in.eta_deg : -30.0 * in.delta_eta_pos;
  const double integrated = s_.integrator.update(increment, load, initial, -30.0, 17.0);

  // 836-855: auto trim from the integrator (not the rate limited output); elevator rate limited 45 deg/s; fixed THS
  // limits
  out.eta_trim_dot_deg_s = autoTrimRate(trimFrozen, integrated, phase);
  out.eta_deg = s_.eta.update(integrated, 45.0, -45.0, dt, 0.0);
  out.eta_trim_limit_up = 3.5;
  out.eta_trim_limit_lo = -11.0;

  s_.autoTrimMode.update(in.in_flight, in.tracking_mode_on, in.eta_trim_deg);  // 856-887 (unused)
  return out;
}

}  // namespace efcs::readable
