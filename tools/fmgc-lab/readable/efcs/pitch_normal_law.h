// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable PitchNormalLaw (generated fbw-a32nx/src/wasm/fbw_a320/src/model/PitchNormalLaw.cpp), stepped by the ELAC
// (LawMDLOBJ5). Bit-exact with the generated law (src/check_pitch_laws.cpp).
//
// The law computes an elevator deflection (deg) and an auto trim rate:
//
//   in flight   the elevator integrates a pitch rate (deg/s) chosen by two median-of-three votes over C* channels
//               (pitch_law_blocks.h), each turning a load factor demand into an elevator rate:
//                 vote 1 = median(nz max, blended law, nz min)
//                 vote 2 = median(theta max, vote 1, theta min)
//               where "blended law" is, by smooth blends, the autopilot (pitch attitude command), else the high angle
//               of attack protection (an alpha law, not a C* channel), else the flare law (pitch attitude command
//               from the flare mode), else the normal law (stick load factor demand, with the high speed protection
//               blended in)
//   on ground   the stick deflection directly (-30 deg x stick), blended into a pitch rate law during the take-off
//               rotation (with tail strike protection), and into the in-flight law over 5 s after lift-off
//   auto trim   the THS moves at eta / 13 deg/s (limited by flight phase), frozen in flare, outside 0.5..1.25 g and
//               beyond 30 deg of bank; its limits freeze at the THS position while a protection is active
//
// Inputs from the ELAC (ElacComputer.cpp 1435-1446): VLS and both spoiler positions are constant 0.

#pragma once

#include "pitch_law_blocks.h"
#include "pitch_laws.h"

namespace efcs::readable {

/** Every memory of the normal law, named. Generated names in the comments; the checker pairs them all. */
struct PitchNormalLawState {
  using LagFilter = pitch_blocks::LagFilter<double>;
  using WashoutFilter = pitch_blocks::WashoutFilter;
  using RateLimiterInit = pitch_blocks::RateLimiterInit;
  using DelayRateLimiter = pitch_blocks::DelayRateLimiter;
  using DiscreteDerivative = pitch_blocks::DiscreteDerivative;
  using LaggedDerivative = pitch_blocks::LaggedDerivative;
  using CStarChannel = pitch_blocks::CStarChannel;

  /** Flare mode chart c2 (states in generated numbering) and its outputs, which are memories (PitchNormalLaw_B). */
  enum class FlareMode : uint8_t { None = 0, ActiveArmed = 1, ActiveReduce = 2, Prepare = 3, Flight = 4, Ground = 5 };
  /** Take-off rotation chart c6. */
  enum class Rotation : uint8_t { None = 0, Off = 1, On = 2 };

  // flare mode
  LagFilter thetaFilter;                // sf_LagFilter: pitch attitude, 0.05 rad/s
  FlareMode flareMode = FlareMode::None;  // is_active_c2 / is_c2
  double flareThetaCmdDeg = 0.0;        // PitchNormalLaw_B.flare_Theta_c_deg
  double flareThetaCmdRateDegS = 0.0;   // PitchNormalLaw_B.flare_Theta_c_rate_deg_s
  DelayRateLimiter flareThetaCmd;       // Delay_DSTATE_e (rate limited flare_Theta_c_deg)

  // auto trim, flight phase, blends
  pitch_blocks::TrimFreezeChart trimFreeze;     // c9
  pitch_blocks::AutoTrimModeChart autoTrimMode;  // c8 (unused)
  pitch_blocks::FlightPhaseChart flightPhase;    // c7
  RateLimiterInit inFlightBlend;        // sf_RateLimiter: in_flight 0 -> 1 over 5 s
  Rotation rotation = Rotation::None;   // is_active_c6 / is_c6
  RateLimiterInit rotationBlend;        // sf_RateLimiter_p
  RateLimiterInit nzMax;                // sf_RateLimiter_c (flight phase nz upper limit, 0.5 g/s)
  RateLimiterInit nzMin;                // sf_RateLimiter_n
  pitch_blocks::HoldWhile trimLimitLo;  // sf_eta_trim_limit_lofreeze
  pitch_blocks::HoldWhile trimLimitUp;  // sf_eta_trim_limit_upfreeze

  // C* channels: the shared filters (7 generated copies each) and each channel's D term
  pitch_blocks::CStarFilters cstar;
  CStarChannel thetaMaxChannel;  // Delay_DSTATE_n
  CStarChannel nzMaxChannel;     // Delay_DSTATE_k
  CStarChannel autopilotChannel;  // Delay_DSTATE_en
  CStarChannel flareChannel;     // Delay_DSTATE_jv
  CStarChannel normalChannel;    // Delay_DSTATE_jh
  CStarChannel nzMinChannel;     // Delay_DSTATE_gz
  CStarChannel thetaMinChannel;  // Delay_DSTATE_ds

  // pitch attitude protection
  RateLimiterInit thetaMax;  // sf_RateLimiter_o (starts at 30 deg)

  // autopilot
  RateLimiterInit apThetaCmd;   // sf_RateLimiter_i
  DelayRateLimiter apBlend;     // Delay_DSTATE_g

  // high angle of attack protection
  RateLimiterInit aoaStick;             // sf_RateLimiter_nx
  LagFilter alphaFilter;                // sf_LagFilter_mf
  WashoutFilter attitudeWashout;        // sf_WashoutFilter_h
  LaggedDerivative alphaErrorRate;      // Subsystem1: Delay_DSTATE_f, Delay_DSTATE_g5, Delay1_DSTATE
  // Subsystem3 (Delay_DSTATE_j, _ca, Delay1_DSTATE_i) and Subsystem (Delay_DSTATE_m, _k2, Delay1_DSTATE_ns): the same
  // V_ias rate, two copies
  LaggedDerivative iasRate;
  DelayRateLimiter aoaBlend;            // Delay_DSTATE_e1

  // flare law
  RateLimiterInit flareThetaLimit;      // sf_RateLimiter_ck
  pitch_blocks::TustinLag flareThetaLag;  // LagFilter3: Delay_DSTATE_o, Delay1_DSTATE_n
  RateLimiterInit flareThetaRate;       // sf_RateLimiter_e
  DelayRateLimiter flareBlend;          // Delay_DSTATE_dv

  // normal law and high speed protection
  RateLimiterInit normalStick;          // sf_RateLimiter_c2
  DelayRateLimiter hspStick;            // Delay_DSTATE_b5
  LaggedDerivative hspTargetRate;       // Subsystem2: Delay_DSTATE_ku, _gl, Delay1_DSTATE_l
  DelayRateLimiter hspBlend;            // Delay_DSTATE_mz

  // in-flight integrator
  pitch_blocks::Integrator flightIntegrator;  // Delay_DSTATE_o3, icLoad

  // ground: stick, rotation law
  RateLimiterInit rotationStick;        // sf_RateLimiter_ct
  DiscreteDerivative qDemandRate;       // Delay_DSTATE_ej
  DiscreteDerivative qErrorRate;        // Delay_DSTATE_e4
  LagFilter qPredicted;                 // sf_LagFilter_f
  pitch_blocks::Integrator rotationIntegrator;  // Delay_DSTATE_cl, icLoad_p

  // output
  RateLimiterInit eta;  // sf_RateLimiter_b
};

class PitchNormalLaw {
 public:
  /** PitchNormalLaw::init(): called once by ElacComputer::initialize(). */
  void init();
  /** PitchNormalLaw::reset(): called on every ELAC restart (first running frame). */
  void reset();
  PitchLawOutputs step(const PitchNormalLawInputs& in);

  const PitchNormalLawState& state() const { return s_; }

 private:
  struct FlareOutputs {
    bool inFlare;
    double thetaCmdDeg;  // rate limited flare pitch attitude command
  };
  struct Attitude {
    double cosRatio;    // cos(theta)/cos(phi)
    double cosRatio33;  // cos(theta)/cos(phi limited to 33 deg)
  };
  struct Blends {
    double inFlight;  // 0 on ground .. 1 in flight (rate limited, 5 s)
    double rotation;  // 0 .. 1 during the take-off rotation (2 s)
  };

  FlareOutputs flareMode(const PitchNormalLawInputs& in);
  Blends groundFlightBlends(const PitchNormalLawInputs& in);
  void trimLimits(const PitchNormalLawInputs& in, PitchLawOutputs& out);
  double thetaMaxChannel(const PitchNormalLawInputs& in, const pitch_blocks::CStarTerms& t, double thetaGain);
  double thetaMinChannel(const pitch_blocks::CStarTerms& t, double thetaGain, double dt);
  double autopilotChannel(const PitchNormalLawInputs& in, const pitch_blocks::CStarTerms& t, const Attitude& a);
  double highAoaChannel(const PitchNormalLawInputs& in, double iasRate);
  double flareChannel(const PitchNormalLawInputs& in, const pitch_blocks::CStarTerms& t, const Attitude& a,
                      double flareThetaCmdDeg);
  double normalChannel(const PitchNormalLawInputs& in, const pitch_blocks::CStarTerms& t, const Attitude& a,
                       double iasRate);
  double groundLaw(const PitchNormalLawInputs& in, const Blends& b, double stickEtaDeg);

  PitchNormalLawState s_;
};

}  // namespace efcs::readable
