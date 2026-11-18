// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable FAC (generated FacComputer::step / initialize, FacComputer.cpp 264-1587). See fac.h.

#include "fac.h"

#include "output_packing.h"

namespace fac::readable {

void FacComputer::initialize() {
  running_ = false;
  sensors_.reset();
  alphaFloor_.reset();
  speeds_.reset();
  flightPath_.reset();
  rudderTrim_ = RudderTrim{};
  yawDamper_.reset();
  travelLimiter_.reset();
  in_ = fac_inputs{};
  out_ = fac_outputs{};
}

void FacComputer::restart() {
  sensors_.reset();
  alphaFloor_.restart();
  speeds_.reset();
  flightPath_.reset();
  rudderTrim_.restart();
  yawDamper_.reset();
  travelLimiter_.reset();
}

const fac_outputs& FacComputer::step(const fac_inputs& in) {
  if (!in.sim_data.computer_running) {
    running_ = false;  // outputs held
    return out_;
  }
  if (!running_) {
    restart();
    running_ = true;
  }
  in_ = in;

  const Sensors& s = sensors_.update(in_);
  FrameResults r{};
  r.in = &in_;
  r.sensors = &s;
  r.engagement = YawEngagement::compute(in_.discrete_inputs);
  flightPath_.updateSideslip(in_, s);
  r.alphaFilteredDeg = alphaFloor_.update(in_, s);
  r.alphaFloor = alphaFloor_.active();
  r.speeds = &speeds_.update(in_, s, r.alphaFilteredDeg);
  flightPath_.updateWarnings(in_, s, r.alphaFilteredDeg, r.speeds->vls);
  r.flightPath = &flightPath_.outputs();
  r.rudderTrimOrder = rudderTrim_.update(in_, s, r.engagement.rudderTrimEngaged);
  r.yawDamperOrder = yawDamper_.update(in_, s, r.engagement.yawDamperEngaged);
  r.travelLimitOrder = travelLimiter_.update(in_, s, r.engagement.travelLimEngaged);
  assembleOutputs(r, out_);
  return out_;
}

}  // namespace fac::readable
