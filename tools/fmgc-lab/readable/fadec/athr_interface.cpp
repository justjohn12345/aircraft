// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FADEC's A/THR interface (generated FadecComputer.cpp 114-124, 138-150). See athr_interface.h.

#include "athr_interface.h"

namespace fadec::readable {

AtsBits AtsBits::read(const base_fcu_bus& fcu) {
  const base_arinc_429& w = fcu.ats_discrete_word;
  return {blocks::a429::bit(w, 13) != 0U, blocks::a429::bit(w, 14) != 0U, blocks::a429::bit(w, 23) != 0U};
}

bool outsideClAndMctDetents(double tla) {
  return (tla > kClDetentHigh || tla < kClDetentLow) && (tla > kMctDetentHigh || tla < kMctDetentLow);
}

void AthrInterface::reset() {
  disconnectHeld_.reset();
  disabled_.reset();
  engageEdge_.reset();
  control_.reset();
  lostEdge_.reset();
  thrustLock_.reset();
  engaged_ = false;
}

void AthrInterface::update(const athr_in& in, const AtsBits& bits) {
  const bool disconnect = in.input.ATHR_disconnect;

  // 114-118: A/THR disabled after a 15 s instinctive disconnect
  const double held = disconnectHeld_.update(in.time.simulation_time, disconnect);
  disabled_.update(held >= kAthrDisableHoldSeconds, in.input.ATHR_reset_disable);

  // 119-124: engaged = ATS bit 13 and a valid N1 command
  engaged_ = bits.engaged && blocks::a429::isNormal(in.fcu_input.n1_cmd_percent);
  const bool engageEdge = engageEdge_.update(engaged_, /*risingEdge*/ true);

  // 139-145: A/THR control active, and the edge when it is lost
  control_.update(engageEdge, disconnect || disabled_.q || !bits.engaged);
  const bool lost = lostEdge_.update(control_.qbar, /*risingEdge*/ true);

  // 146-150: thrust lock
  const bool outside = outsideClAndMctDetents(in.input.TLA_deg);
  thrustLock_.update(lost && !outside, outside || control_.q);
}

}  // namespace fadec::readable
