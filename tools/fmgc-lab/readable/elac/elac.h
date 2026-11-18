// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable ELAC (generated fbw-a32nx/src/wasm/fbw_a320/src/model/ElacComputer.cpp, 1945 lines, with the laws in
// readable/efcs/). Same interface as the model: initialize() once, then step() every frame; computer_running false
// (the wrapper's ELAC health) holds every output and memory, and the first running frame afterwards restarts it
// (generated 371-426). Bit-exact against the generated model: src/check_elac.cpp. Spec: docs/elac.md.
//
// Files: sensors.cpp (ADR / IR / RA voting, on ground, flight / ground chart, hydraulics), engagement.cpp (sidestick
// priority, surface availability, pitch / roll engagement, law capability and active laws), protections.cpp (alpha and
// high speed protections, abnormal attitude, AP disconnect and authorisation), orders.cpp (lateral and pitch orders:
// the laws, aileron droop, THS), output_packing.cpp (discrete words, bus, logic copies).

#pragma once

#include <cstdint>

#include "ElacComputer_types.h"
#include "efcs_blocks.h"
#include "lateral_direct_law.h"
#include "lateral_normal_law.h"
#include "pitch_alternate_law.h"
#include "pitch_direct_law.h"
#include "pitch_normal_law.h"

namespace elac::readable {

namespace blocks = efcs::blocks;

/** The flap handle index 0..5 from an SFCC system status word (generated MATLABFunction_o, 106-124). */
double flapHandleIndex(const base_arinc_429& status);

/** Results of one frame passed between the stages (the generated rtb_* temporaries, named). */
struct Frame {
  const elac_inputs* in = nullptr;

  // sensors.cpp
  bool doubleAdrFault = false, tripleAdrFault = false, doubleIrFault = false, tripleIrFault = false;
  float vIas = 0, vTas = 0, mach = 0, alpha = 0;  // voted ADR data
  double alphaFiltered = 0;                       // alpha, lag 0.5 rad/s (rtb_Y)
  float theta = 0, phi = 0, q = 0, r = 0, nx = 0, ny = 0, nz = 0, thetaDot = 0, phiDot = 0;  // voted IR data
  float radioHeight = 0;                          // rtb_raComputationValue
  bool dualRaFailure = false;
  bool onGround = false;                          // rtb_ra1Invalid (after 735)
  bool trackingMode = false;                      // slew, pause or external override
  bool inFlight = false;                          // the flight / ground chart (in_flight != 0)
  bool yellowAvail = false, blueAvail = false, greenAvail = false;

  // engagement.cpp
  bool leftStickLocked = false, rightStickLocked = false;  // rtb_OR4, rtb_AND2 (after 825)
  double rollStick = 0, pitchStick = 0;           // u0, u0_0: both sidesticks, +-1
  bool leftElevAvail = false, rightElevAvail = false, thsAvail = false;
  bool canEngageInPitch = false, hasPriorityInPitch = false, isEngagedInPitch = false;
  bool leftAileronAvail = false, rightAileronAvail = false;
  bool canEngageInRoll = false, hasPriorityInRoll = false, isEngagedInRoll = false;
  bool leftAileronCrossCommand = false, rightAileronCrossCommand = false;
  bool abnormalCondition = false;                 // rtb_OR1_me (after 960)
  lateral_efcs_law lateralLawCapability = lateral_efcs_law::NormalLaw;
  pitch_efcs_law pitchLawCapability = pitch_efcs_law::NormalLaw;
  lateral_efcs_law activeLateralLaw = lateral_efcs_law::None;
  pitch_efcs_law activePitchLaw = pitch_efcs_law::None;
  bool thsGroundSetting = false;                  // rtb_OR1_i (after 1138)
  bool thsActiveCommanded = false;                // rtb_y_p (after 1084)

  // protections.cpp
  double alphaMax = 0, alphaProt = 0;             // rtb_Switch3_p, rtb_handleIndex
  double highSpeedProtLow = 0, highSpeedProtHigh = 0;
  bool normalLawActive = false;                   // pitch or roll normal law (rtb_OR_eo, 1140)
  bool noApEngaged = false;                       // both AP discretes disengaged
  bool apDisconnect = false, apAuthorised = false;

  // orders.cpp
  bool ap1Control = false, ap2Control = false;
  bool aileronDroop = false, aileronAntiDroop = false;
  double leftAileronCommand = 0, rightAileronCommand = 0, rollSpoilerCommand = 0, yawDamperCommand = 0;
  double elevatorCommand = 0, thsCommand = 0;
  bool doublePressurization = false, leftDampingInhibited = false, rightDampingInhibited = false;
  double elevatorOrder = 0;                       // the own command, or the opposite ELAC's when not engaged
  bool useOppElevatorCommand = false;
  bool battPowerSupply = false;
};

class ElacComputer {
 public:
  /** The flight / ground chart (is_c30_ElacComputer). */
  enum class FlightPhase : uint8_t { None = 0, Flight = 1, FlightToGround = 2, Ground = 3 };
  /** The AP disconnect phase chart (is_c28_ElacComputer). */
  enum class ApPhase : uint8_t { None = 0, Flying = 1, Landed = 2, Landing100ft = 3, Takeoff100ft = 4 };

  /** A complete reset (the generated initialize() is not one: see docs/elac.md). */
  void initialize();
  const elac_outputs& step(const elac_inputs& in);
  const elac_outputs& outputs() const { return out_; }

  // ---- state, for the differential checker (generated DWork names in docs/elac.md and src/elac_state.h) ----
  struct SensorState {
    blocks::LagFilter<double> alphaFilter;          // sf_LagFilter_a
    blocks::ConfirmNode raDisagree;                 // sf_MATLABFunction_jz
    blocks::ConfirmNode ra1Implausible;             // sf_MATLABFunction_lf
    blocks::ConfirmNode ra2Implausible;             // sf_MATLABFunction_jl
    bool ra1Rejected = false, ra2Rejected = false;  // ra1/ra2CoherenceRejected
    double flapsFullTime = 0;                       // configFullEventTime
    bool flapsFullTimeSet = false;                  // configFullEventTime_not_empty
    bool flightChartActive = false;                 // is_active_c30
    FlightPhase flightPhase = FlightPhase::None;    // is_c30
    double inFlight = 0;                            // ElacComputer_B.in_flight
    double onGroundTime = 0;                        // on_ground_time
    blocks::Hysteresis yellowPressure, bluePressure, greenPressure;  // sf_MATLABFunction_jg, _mi, _br
    blocks::ConfirmNode yellowConfirm, blueConfirm, greenConfirm;    // sf_MATLABFunction_cj, _g2, _gfx
  };
  struct EngagementState {
    blocks::EdgeDetector captTakeover, foTakeover;  // sf_MATLABFunction_g4, _nu
    bool leftStickDisabled = false, rightStickDisabled = false;   // pLeft/RightStickDisabled
    blocks::ConfirmNode leftLockConfirm, rightLockConfirm;        // sf_MATLABFunction_j2, _g24
    bool leftLockedPrev = false, rightLockedPrev = false;         // Delay_DSTATE_cc, Delay1_DSTATE
    blocks::ConfirmNode slewConfirm;                // sf_MATLABFunction_nb
    bool abnormalWasActive = false;                 // abnormalConditionWasActive
    blocks::EdgeDetector landing;                   // sf_MATLABFunction_l0 (falling edge of in flight)
    bool thsGroundLatch = false;                    // Memory_PreviousInput
  };
  struct ProtectionState {
    blocks::RateLimiter alphaMaxRate, alphaProtRate;  // sf_RateLimiter_n, _m
    double liftOffTime = 0;                         // eventTime_p
    bool liftOffTimeSet = false;                    // eventTime_not_empty_i
    bool highSpeedProt = false;                     // sProtActive
    double stickNotPulledTime = 0;                  // resetEventTime
    bool stickNotPulledTimeSet = false;             // resetEventTime_not_empty
    bool alphaProt = false;                         // sProtActive_f
    bool apChartActive = false;                     // is_active_c28
    ApPhase apPhase = ApPhase::None;                // is_c28
    double speedBelowLimitTime = 0;                 // eventTime
    bool speedBelowLimitTimeSet = false;            // eventTime_not_empty
  };
  struct OrderState {
    blocks::RateLimiterInit droop, antiDroop;       // sf_RateLimiter, sf_RateLimiter_b
    blocks::RateLimiterFrom leftAileron, rightAileron;  // sf_RateLimiter_p, _a
    blocks::LagFilter<double> pitchAcceleration;    // sf_LagFilter
    double previousThetaDot = 0;                    // Delay_DSTATE
    double thsOrder = 0;                            // Delay_DSTATE_b
    double thsIntegrator = 0;                       // Delay_DSTATE_c
    bool reloadThs = false;                         // icLoad
    blocks::ConfirmNode battPower;                  // sf_MATLABFunction_fb
  };

  bool running() const { return running_; }
  const SensorState& sensorState() const { return sensors_; }
  const EngagementState& engagementState() const { return engagement_; }
  const ProtectionState& protectionState() const { return protections_; }
  const OrderState& orderState() const { return orders_; }
  const efcs::readable::LateralNormalLaw& lateralNormalLaw() const { return lateralNormal_; }
  const efcs::readable::LateralDirectLaw& lateralDirectLaw() const { return lateralDirect_; }
  const efcs::readable::PitchNormalLaw& pitchNormalLaw() const { return pitchNormal_; }
  const efcs::readable::PitchAlternateLaw& pitchAlternateLaw() const { return pitchAlternate_; }
  const efcs::readable::PitchDirectLaw& pitchDirectLaw() const { return pitchDirect_; }

 private:
  void restart();
  // sensors.cpp
  void voteAirData(Frame& f);
  void voteInertialData(Frame& f);
  void updateRadioHeight(Frame& f);
  void updateOnGround(Frame& f);
  void updateFlightPhase(Frame& f);
  void updateHydraulics(Frame& f);
  // engagement.cpp
  void updateSidestickPriority(Frame& f);
  void updateEngagement(Frame& f);
  void updateLawCapability(Frame& f);
  void updateActiveLaws(Frame& f);
  void updateThsEngagement(Frame& f);
  // protections.cpp
  void updateProtections(Frame& f);
  // orders.cpp
  void updateLateralOrders(Frame& f);
  void updatePitchOrders(Frame& f);
  // output_packing.cpp
  void packOutputs(const Frame& f);

  bool running_ = false;  // Runtime_MODE
  SensorState sensors_;
  EngagementState engagement_;
  ProtectionState protections_;
  OrderState orders_;
  efcs::readable::LateralNormalLaw lateralNormal_;    // LawMDLOBJ2
  efcs::readable::LateralDirectLaw lateralDirect_;    // LawMDLOBJ1
  efcs::readable::PitchNormalLaw pitchNormal_;        // LawMDLOBJ5
  efcs::readable::PitchAlternateLaw pitchAlternate_;  // LawMDLOBJ3
  efcs::readable::PitchDirectLaw pitchDirect_;        // LawMDLOBJ4
  elac_inputs in_{};
  elac_outputs out_{};
};

}  // namespace elac::readable
