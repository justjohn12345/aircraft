// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// N1 command and sim throttle (generated FadecComputer.cpp 247-323, 423-428). See n1_control.h.

#include "n1_control.h"

#include <cmath>

#include "thrust_lever.h"

namespace fadec::readable {

void N1Control::reset() {
  frozenN1_ = 0.0;
  frozenN1Primed_ = false;
  trim_.reset();
  forward_.reset();
  reverse_.reset();
}

double N1Control::command(const athr_in& in, const AtsBits& bits, const AthrInterface& athr, double n1ForLever) {
  const double n1Actual = in.data.engine_N1_percent;
  // 253-260: the N1 to hold under thrust lock, frozen while the lock is on
  if (!frozenN1Primed_) {
    frozenN1_ = n1Actual;
    frozenN1Primed_ = true;
  }
  if (!athr.thrustLock()) {
    frozenN1_ = n1Actual;
  }

  const bool inRange = in.input.TLA_deg <= kTlaActiveRangeMax || bits.alphaFloor;
  if (inRange && bits.active && athr.engaged() && athr.controlActive()) {
    // 262-277: the FMGC's command (A429ValueOrDefault, default 0) within the limits
    const float fmgcCommand = blocks::a429::valueOr(in.fcu_input.n1_cmd_percent, 0.0F);
    double upper, lower;
    if (bits.engaged && bits.alphaFloor && !in.data.on_ground) {
      upper = in.input.thrust_limit_TOGA_percent;
      lower = n1ForLever;
    } else {
      upper = n1ForLever;
      lower = in.input.thrust_limit_IDLE_percent;
    }
    if (fmgcCommand > upper) {
      return static_cast<float>(upper);
    }
    if (fmgcCommand < lower) {
      return static_cast<float>(lower);
    }
    return fmgcCommand;
  }
  // 278-281
  return athr.thrustLock() ? frozenN1_ : n1ForLever;
}

double N1Control::throttle(const athr_in& in, double n1Command, bool reverse) {
  const double dt = in.time.dt;
  // 284-297: trim on the N1 error
  double error = n1Command - in.data.engine_N1_percent;
  if (std::abs(error) > kTrimResetError) {
    trim_.reset();
    error = 0.0;
  }
  trim_.update(error, kTrimGain, dt, -kTrimLimit, kTrimLimit);

  // 299-323: servo the sim's commanded N1
  const double throttleError = (n1Command + trim_.state) - in.data.commanded_engine_N1_percent;
  if (reverse) {
    forward_.reset();
  }
  forward_.update(throttleError, kThrottleGain, dt, 0.0, kThrottleLimit);
  if (!reverse) {
    reverse_.reset();
  }
  reverse_.update(throttleError, kReverseThrottleGain, dt, -kThrottleLimit, 0.0);
  // 423-428
  return reverse ? reverse_.state : forward_.state;
}

}  // namespace fadec::readable
