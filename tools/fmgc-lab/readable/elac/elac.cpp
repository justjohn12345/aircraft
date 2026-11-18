// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable ELAC (generated ElacComputer::step / initialize, ElacComputer.cpp 260-1931). See elac.h.

#include "elac.h"

namespace elac::readable {

void ElacComputer::initialize() {
  running_ = false;
  sensors_ = SensorState{};
  engagement_ = EngagementState{};
  protections_ = ProtectionState{};
  orders_ = OrderState{};
  orders_.reloadThs = true;
  lateralNormal_ = {};
  lateralDirect_ = {};
  pitchNormal_ = {};
  pitchAlternate_ = {};
  pitchDirect_ = {};
  // the generated initialize() (1919-1931) calls init() on the lateral normal, pitch normal and alternate laws
  lateralNormal_.init();
  pitchNormal_.init();
  pitchAlternate_.init();
  in_ = elac_inputs{};
  out_ = elac_outputs{};
}

void ElacComputer::restart() {
  // (372-425): every memory back to its initial state, except the THS integrator (reloaded instead, icLoad), the
  // flight / ground chart's touchdown time and the event times (reloaded when their "set" flags are cleared)
  SensorState& s = sensors_;
  s.alphaFilter.reset();
  s.raDisagree.reset();
  s.ra1Implausible.reset();
  s.ra2Implausible.reset();
  s.ra1Rejected = s.ra2Rejected = false;
  s.flapsFullTimeSet = false;
  s.inFlight = 0.0;
  s.onGroundTime = 0.0;
  s.flightChartActive = false;
  s.flightPhase = FlightPhase::None;
  s.yellowPressure.reset();
  s.bluePressure.reset();
  s.greenPressure.reset();
  s.yellowConfirm.reset();
  s.blueConfirm.reset();
  s.greenConfirm.reset();

  EngagementState& e = engagement_;
  e.captTakeover.reset();
  e.foTakeover.reset();
  e.leftStickDisabled = e.rightStickDisabled = false;
  e.leftLockConfirm.reset();
  e.rightLockConfirm.reset();
  e.leftLockedPrev = e.rightLockedPrev = false;
  e.slewConfirm.reset();
  e.abnormalWasActive = false;
  e.landing.reset();
  e.thsGroundLatch = false;

  ProtectionState& p = protections_;
  p.alphaMaxRate.reset();
  p.alphaProtRate.reset();
  p.liftOffTimeSet = false;
  p.highSpeedProt = false;
  p.stickNotPulledTimeSet = false;
  p.alphaProt = false;
  p.apChartActive = false;
  p.apPhase = ApPhase::None;
  p.speedBelowLimitTimeSet = false;

  OrderState& o = orders_;
  o.droop.reset();
  o.antiDroop.reset();
  o.leftAileron.reset();
  o.rightAileron.reset();
  o.pitchAcceleration.reset();
  o.previousThetaDot = 0.0;
  o.thsOrder = 0.0;
  o.reloadThs = true;
  o.battPower.reset();

  lateralNormal_.reset();
  lateralDirect_.reset();
  pitchNormal_.reset();
  pitchAlternate_.reset();
  pitchDirect_.reset();
}

const elac_outputs& ElacComputer::step(const elac_inputs& in) {
  if (!in.sim_data.computer_running) {
    running_ = false;  // (1914-1916) outputs and memories held
    return out_;
  }
  if (!running_) {
    restart();
    running_ = true;
  }
  in_ = in;
  Frame f;
  f.in = &in_;

  voteAirData(f);
  voteInertialData(f);
  updateRadioHeight(f);
  updateOnGround(f);
  updateFlightPhase(f);
  updateHydraulics(f);
  updateSidestickPriority(f);
  updateEngagement(f);
  updateLawCapability(f);
  updateActiveLaws(f);
  updateThsEngagement(f);
  updateProtections(f);
  updateLateralOrders(f);
  updatePitchOrders(f);
  packOutputs(f);
  return out_;
}

}  // namespace elac::readable
