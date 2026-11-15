// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "tcas.h"

#include <cmath>
#include <cstdint>

namespace fmgc::readable::tcas {

namespace {

using blocks::a429::bit;
using blocks::a429::valueOr;

constexpr double kFlagS = 10;  // the three FMA flags last 10 s (MTrigNode, rising edge, not retriggerable)

// TCAS vertical resolution advisory word
constexpr int kRaWordDown = 17;          // RA sense: down (the V/S target is negative)
constexpr int kRaWordCombined1 = 18;     // combined control bits 18–20; (19 and 20 without 18) = preventive
constexpr int kRaWordCombined2 = 19;
constexpr int kRaWordCombined3 = 20;
constexpr uint32_t kRaWordRateMask = 0xFC00;  // bits 11–16: RA vertical rate in 100 fpm
constexpr int kRaWordRateShift = 10;
constexpr int kRaRateMarginFpm = 200;    // added to a non-zero RA rate

constexpr int kFcuWord1HdgVsSelected = 24;

/** The word's value rounded and saturated to uint32, as the generated code does before masking it. */
uint32_t wordBits(const base_arinc_429& w) {
  float r = std::round(w.Data);
  return r < 4.2949673E+9F ? (r >= 0.0F ? static_cast<uint32_t>(r) : 0U) : UINT32_MAX;
}

}  // namespace

void TcasLogic::update(const Inputs& in, Outputs& out) {
  const auto& ra = in.in.bus_inputs.tcas_bus.vertical_resolution_advisory;
  const auto& lastModes = in.last.longitudinal_modes;
  const auto& lastArmed = in.last.armed_modes;
  State& s = s_;

  // Corrective unless the combined control is "preventive" (19 and 20 without 18). Also true without an RA.
  out.raCorrective = bit(ra, kRaWordCombined1) != 0 || bit(ra, kRaWordCombined2) == 0 || bit(ra, kRaWordCombined3) == 0;

  const uint32_t rate = ((wordBits(ra) & kRaWordRateMask) >> kRaWordRateShift) * 100U;
  const int32_t absVsTarget = rate != 0U ? static_cast<int32_t>(rate) + kRaRateMarginFpm : 0;
  if (out.raCorrective) {
    out.vsTargetFpm = bit(ra, kRaWordDown) != 0 ? -static_cast<double>(absVsTarget) : absVsTarget;
  } else {
    out.vsTargetFpm = valueOr(in.logic.adr_computation_data.vertical_speed_ft_min, 0.0F);
  }

  out.raInhibited =
      s.raInhibited10s.update(lastModes.tcas_active && !in.logic.tcas_mode_available, in.dt, true, false, kFlagS);

  // HDG-V/S selected during TCAS mode; once TCAS mode ends, a 10 s "TRK-FPA deselected" flag, and the latch resets on
  // the next frame.
  const bool hdgVsPushed = s.hdgVsSelected.update(
      bit(in.in.bus_inputs.fcu_bus.fcu_discrete_word_1, kFcuWord1HdgVsSelected) != 0, true);
  const bool latched = s.hdgVsDuringTcas.update(lastModes.tcas_active && hdgVsPushed, s.tcasEndedAfterHdgVs);
  s.tcasEndedAfterHdgVs = latched && !lastModes.tcas_active;
  out.trkFpaDeselected = s.trkFpaDeselected10s.update(s.tcasEndedAfterHdgVs, in.dt, true, false, kFlagS);

  out.longiLargeBox = s.largeBox10s.update(
      lastModes.tcas_active && (lastArmed.clb_armed || lastArmed.des_armed || lastArmed.final_des_armed ||
                                lastArmed.glide_armed || lastArmed.alt_acq_armed),
      in.dt, true, false, kFlagS);
}

}  // namespace fmgc::readable::tcas
