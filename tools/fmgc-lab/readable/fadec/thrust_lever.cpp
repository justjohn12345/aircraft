// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Thrust lever angle to thrust (generated FadecComputer.cpp 125-134, 151-246, 429-445). See thrust_lever.h.

#include "thrust_lever.h"

#include <cmath>

#include "fadec_blocks.h"

namespace fadec::readable {

bool FlexSelection::update(const athr_in& in) {
  // 125-128: flex temperature word normal and above TAT (A429ValueOrDefault default 0)
  const base_arinc_429& flexWord = in.fcu_input.fcu_flex_to_temp_deg_c;
  const bool flexValid = blocks::a429::isNormal(flexWord) && blocks::a429::valueOr(flexWord, 0.0F) > in.data.TAT_degC;
  const bool onGround = in.data.on_ground;
  const double tla = in.input.TLA_deg;
  // 129-132: set at exactly 35 on the ground, cleared at exactly 25 or 45
  latch_ = (flexValid && onGround && tla == kTlaFlexMct) || latch_;
  latch_ = latch_ && tla != kTlaClimb && tla != kTlaToga;
  // 133-134
  return (flexValid && onGround) || (!onGround && latch_);
}

N1Rating n1ForLever(const athr_in& in, bool flexActive) {
  const athr_input& lim = in.input;
  double tla = in.input.TLA_deg;
  if (!in.data.on_ground) {
    tla = std::fmax(0.0, tla);  // no reverse in flight
  }
  const bool reverse = tla < 0.0;

  double tlaBegin, tlaEnd, n1Begin, n1End;
  if (tla >= 0.0) {
    const double flexOrMct = flexActive ? lim.thrust_limit_FLEX_percent : lim.thrust_limit_MCT_percent;
    if (tla <= kTlaClimb) {
      tlaBegin = kTlaIdle, n1Begin = lim.thrust_limit_IDLE_percent;
      tlaEnd = kTlaClimb, n1End = lim.thrust_limit_CLB_percent;
    } else if (tla <= kTlaFlexMct) {
      tlaBegin = kTlaClimb, n1Begin = lim.thrust_limit_CLB_percent;
      tlaEnd = kTlaFlexMct, n1End = flexOrMct;
    } else {
      tlaBegin = kTlaFlexMct, n1Begin = flexOrMct;
      tlaEnd = kTlaToga, n1End = lim.thrust_limit_TOGA_percent;
    }
  } else {
    // reverse, or NaN
    tla = std::fmax(std::abs(tla), kTlaReverseIdle);
    tlaBegin = kTlaReverseIdle, n1Begin = std::abs(lim.thrust_limit_IDLE_percent + 1.0);
    tlaEnd = kTlaReverseMax, n1End = std::abs(lim.thrust_limit_REV_percent);
  }
  // 191-192
  return {(n1End - n1Begin) / (tlaEnd - tlaBegin) * (tla - tlaBegin) + n1Begin, reverse};
}

ThrustLimit thrustLimit(const athr_in& in, bool flexActive, bool alphaFloor) {
  using T = athr_thrust_limit_type;
  const athr_input& lim = in.input;
  double tla = in.input.TLA_deg;
  if (!in.data.on_ground) {
    tla = std::fmax(0.0, tla);
  }
  const ThrustLimit toga{T::TOGA, lim.thrust_limit_TOGA_percent};
  const ThrustLimit flex{T::FLEX, lim.thrust_limit_FLEX_percent};
  const ThrustLimit none{T::NONE, 0.0};
  const ThrustLimit reverse{T::REVERSE, lim.thrust_limit_REV_percent};

  if (!in.data.on_ground || !in.data.is_engine_operative) {
    // 200-224
    if (alphaFloor || tla > kTlaFlexMct) {
      return toga;
    }
    if (tla > kTlaClimb) {
      return flexActive ? flex : ThrustLimit{T::MCT, lim.thrust_limit_MCT_percent};
    }
    if (tla >= 0.0) {
      return {T::CLB, lim.thrust_limit_CLB_percent};
    }
    return tla < 0.0 ? reverse : none;
  }
  // 225-246: on the ground, engine operative
  if (tla >= 0.0) {
    return !flexActive || tla > kTlaFlexMct ? toga : flex;
  }
  return tla < 0.0 ? reverse : none;
}

double simThrustMode(double tla) {
  if (tla < 0.0) return 1.0;
  if (tla == kTlaIdle) return 2.0;
  if (tla > kTlaIdle && tla < kTlaClimb) return 3.0;
  if (tla >= kTlaClimb && tla < kTlaFlexMct) return 4.0;
  if (tla >= kTlaFlexMct && tla < kTlaToga) return 5.0;
  if (tla == kTlaToga) return 6.0;
  return 0.0;
}

}  // namespace fadec::readable
