// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable SEC (generated fbw-a32nx/src/wasm/fbw_a320/src/model/SecComputer.cpp, 1439 lines; laws in
// readable/efcs/). Same interface as the model: initialize() once, then step() every frame; computer_running false
// holds every output and memory, and the first running frame afterwards restarts it (generated 232-280). Bit-exact
// against the generated model: src/check_sec.cpp. Spec: docs/sec.md.
//
// Files: sec.cpp (ADR / IR selection, LGCIU monitoring, on ground, flight / ground chart, hydraulics, sidestick
// priority, engagement, pitch law capability; step and restart), spoilers.cpp (speed brakes, ground spoilers, partial
// lift dumping, roll spoilers), pitch.cpp (the pitch laws, elevator and THS orders), output_packing.cpp.

#pragma once

#include <cstdint>

#include "SecComputer_types.h"
#include "efcs_blocks.h"
#include "lateral_direct_law.h"
#include "pitch_alternate_law.h"
#include "pitch_direct_law.h"

namespace sec::readable {

namespace blocks = efcs::blocks;

/** Results of one frame passed between the stages (the generated rtb_* temporaries, named). */
struct Frame {
  const sec_inputs* in = nullptr;

  // sec.cpp
  bool singleAdrFault = false, doubleAdrFault = false, singleIrFault = false, doubleIrFault = false;
  float vIas = 0, vTas = 0, mach = 0, alpha = 0;
  float theta = 0, phi = 0, q = 0, r = 0, nx = 0, ny = 0, nz = 0, thetaDot = 0, phiDot = 0;
  bool gearNotUplocked = false;       // rtb_AND11: any_landing_gear_not_uplocked
  bool lgciuFault = false;            // rtb_OR1 (after 476): lgciu_uplock_disagree_or_fault
  bool onGround = false;              // rtb_OR6 (after 490)
  bool trackingMode = false;
  bool inFlight = false;
  bool yellowAvail = false, blueAvail = false, greenAvail = false;
  bool leftStickLocked = false, rightStickLocked = false;
  double rollStick = 0, pitchStick = 0;
  bool leftElevAvail = false, rightElevAvail = false, thsAvail = false;
  bool canEngageInPitch = false, hasPriorityInPitch = false, isEngagedInPitch = false;
  bool spoilerPair1Avail = false, spoilerPair2Avail = false, isEngagedInRoll = false;
  bool abnormalCondition = false;
  pitch_efcs_law pitchLawCapability = pitch_efcs_law::AlternateLaw1;
  pitch_efcs_law activePitchLaw = pitch_efcs_law::None;
  bool elacsPitchLost = false;        // both ELACs report their pitch axis lost
  bool thsGroundSetting = false, thsActiveCommanded = false;

  // spoilers.cpp
  bool speedBrakeInhibited = false, groundSpoilersArmed = false, partialLiftDumping = false;
  double speedBrakeCommand = 0;
  double leftSpoiler1 = 0, rightSpoiler1 = 0, leftSpoiler2 = 0, rightSpoiler2 = 0;

  // pitch.cpp
  double elevatorCommand = 0, elevatorOrder = 0, thsCommand = 0;
  bool useElacElevatorCommand = false;
  bool battPowerSupply = false;
};

class SecComputer {
 public:
  enum class GroundPhase : uint8_t { None = 0, InAir = 1, OnGround = 2 };                 // is_c8_SecComputer
  enum class FlightPhase : uint8_t { None = 0, Flight = 1, FlightToGround = 2, Ground = 3 };  // is_c30_SecComputer

  /** A complete reset (the generated initialize() is not one). */
  void initialize();
  const sec_outputs& step(const sec_inputs& in);
  const sec_outputs& outputs() const { return out_; }

  struct State {
    // sec.cpp
    blocks::ConfirmNode lgciu2UplockConfirm;   // sf_MATLABFunction_jk
    blocks::ConfirmNode lgciu1UplockConfirm;   // sf_MATLABFunction_dw
    bool groundChartActive = false;            // is_active_c8
    GroundPhase groundPhase = GroundPhase::None;  // is_c8
    bool flightChartActive = false;            // is_active_c30
    FlightPhase flightPhase = FlightPhase::None;  // is_c30
    double inFlight = 0;                       // SecComputer_B.in_flight
    double onGroundTime = 0;                   // on_ground_time
    blocks::ConfirmNode yellowConfirm, blueConfirm, greenConfirm;  // sf_MATLABFunction_ndv, _gf, _h
    blocks::EdgeDetector captTakeover, foTakeover;                 // sf_MATLABFunction_g4b, _nu
    bool leftStickDisabled = false, rightStickDisabled = false;    // pLeft/RightStickDisabled
    blocks::ConfirmNode leftLockConfirm, rightLockConfirm;         // sf_MATLABFunction_j2, _g24
    bool leftLockedPrev = false, rightLockedPrev = false;          // Delay_DSTATE_c, Delay1_DSTATE
    blocks::ConfirmNode slewConfirm;           // sf_MATLABFunction_k4
    bool abnormalWasActive = false;            // abnormalConditionWasActive
    blocks::EdgeDetector landing;              // sf_MATLABFunction_b4 (falling edge of in flight)
    bool thsGroundLatch = false;               // Memory_PreviousInput
    // spoilers.cpp
    blocks::ConfirmNode leverRetracted;        // sf_MATLABFunction_fh
    bool speedBrakeInhibitLatch = false;       // Memory_PreviousInput_f
    blocks::EdgeDetector touchdown;            // sf_MATLABFunction_nd (rising edge of on ground)
    blocks::EdgeDetector liftOff;              // sf_MATLABFunction_n (falling edge of on ground)
    bool wheelsSlowLatch = false;              // Memory_PreviousInput_n
    blocks::EdgeDetector bothMainGears;        // sf_MATLABFunction_a
    bool groundSpoilersOut = false;            // Delay1_DSTATE_i
    blocks::EdgeDetector oneMainGear;          // sf_MATLABFunction_e3
    bool partialLiftDumpingOneGear = false;    // Delay_DSTATE_n
    bool partialLiftDumpingArmed = false;      // Delay2_DSTATE
    blocks::RateLimiterInit groundSpoilerRate;  // sf_RateLimiter_c
    blocks::RateLimiterInit speedBrakeRate;    // sf_RateLimiter
    blocks::RateLimiterFrom leftSpoiler1, rightSpoiler1, leftSpoiler2, rightSpoiler2;  // sf_RateLimiter_b, _a, _k, _b4
    // pitch.cpp
    double thsOrder = 0;                       // Delay_DSTATE
    double thsIntegrator = 0;                  // Delay_DSTATE_l
    bool reloadThs = false;                    // icLoad
    blocks::ConfirmNode battPower;             // sf_MATLABFunction_i
  };

  bool running() const { return running_; }
  const State& state() const { return s_; }
  const efcs::readable::LateralDirectLaw& lateralDirectLaw() const { return lateralDirect_; }
  const efcs::readable::PitchAlternateLaw& pitchAlternateLaw() const { return pitchAlternate_; }
  const efcs::readable::PitchDirectLaw& pitchDirectLaw() const { return pitchDirect_; }

 private:
  void restart();
  // sec.cpp
  void selectAirData(Frame& f);
  void monitorLandingGear(Frame& f);
  void updateFlightPhase(Frame& f);
  void updateEngagement(Frame& f);
  void updateLawCapability(Frame& f);
  // spoilers.cpp
  void updateSpoilers(Frame& f);
  // pitch.cpp
  void updatePitchOrders(Frame& f);
  // output_packing.cpp
  void packOutputs(const Frame& f);

  bool running_ = false;  // Runtime_MODE
  State s_;
  efcs::readable::LateralDirectLaw lateralDirect_;    // LawMDLOBJ1
  efcs::readable::PitchAlternateLaw pitchAlternate_;  // LawMDLOBJ2
  efcs::readable::PitchDirectLaw pitchDirect_;        // LawMDLOBJ3
  sec_inputs in_{};
  sec_outputs out_{};
};

/** The flap handle index 0..5 from an SFCC system status word (generated 988-1014). */
double flapHandleIndex(const base_arinc_429& status);

}  // namespace sec::readable
