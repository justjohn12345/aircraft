// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The readable FADEC (generated FadecComputer::step / initialize, FadecComputer.cpp 93-463). See fadec.h.

#include "fadec.h"

#include "output_packing.h"

namespace fadec::readable {

void FadecComputer::initialize() {
  athr_.reset();
  flex_.reset();
  onGround_.reset();
  n1_.reset();
  out_ = athr_out{};
}

const athr_out& FadecComputer::step(const athr_in& in) {
  const AtsBits bits = AtsBits::read(in.fcu_input);
  athr_.update(in, bits);
  const bool flexActive = flex_.update(in);
  // 135-136: "time since touchdown" is the time on the ground (0 in flight)
  const double timeOnGround = onGround_.update(in.time.simulation_time, in.data.on_ground);

  const N1Rating rating = n1ForLever(in, flexActive);
  const ThrustLimit limit = thrustLimit(in, flexActive, bits.alphaFloor);
  const double n1Command = n1_.command(in, bits, athr_, rating.n1);
  const double throttle = n1_.throttle(in, n1Command, rating.reverse);

  out_.fadec_bus_output =
      packEcuBus(in, {rating.n1, flexActive, limit, athr_.controlActive(), bits.alphaFloor, n1Command, athr_.thrustLock()});
  // 415-452
  out_.fcu_input = in.fcu_input;
  out_.time = in.time;
  out_.data = in.data;
  out_.data_computed.TLA_in_active_range = in.input.TLA_deg <= kTlaActiveRangeMax;
  out_.data_computed.is_FLX_active = flexActive;
  out_.data_computed.ATHR_disabled = athr_.disabled();
  out_.data_computed.time_since_touchdown = timeOnGround;
  out_.input = in.input;

  athr_output& o = out_.output;
  o.sim_throttle_lever_pos = throttle;
  o.sim_thrust_mode = simThrustMode(in.input.TLA_deg);
  o.N1_TLA_percent = rating.n1;
  o.is_in_reverse = rating.reverse;
  o.thrust_limit_type = limit.type;
  o.thrust_limit_percent = limit.percent;
  o.N1_c_percent = n1Command;
  o.athr_control_active = athr_.controlActive();
  o.memo_thrust_active = athr_.thrustLock();
  return out_;
}

}  // namespace fadec::readable
