// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Thrust lever angle (TLA) to thrust (generated FadecComputer.cpp 125-134, 137, 151-246, 429-445): FLX selection, the
// N1 rating for the lever position (N1_TLA), the thrust limit type shown on the EWD, and the sim thrust mode.
//
// TLA detents (deg): reverse max -20, reverse idle -6 (the reverse curve starts at 6), IDLE 0, CL 25, FLX/MCT 35,
// TOGA 45. The thrust limits (N1 %) come from the sim (L:A32NX_AUTOTHRUST_THRUST_LIMIT_*, REV = TOGA x 0.813).

#pragma once

#include "FadecComputer_types.h"

namespace fadec::readable {

constexpr double kTlaIdle = 0.0, kTlaClimb = 25.0, kTlaFlexMct = 35.0, kTlaToga = 45.0;
/** Reverse curve: from |TLA| 6 (IDLE N1 + 1, absolute value) to |TLA| 20 (REV limit, absolute value). */
constexpr double kTlaReverseIdle = 6.0, kTlaReverseMax = 20.0;
/** A/THR active range upper end (CompareToConstant1_const): TLA_in_active_range = TLA <= 35. */
constexpr double kTlaActiveRangeMax = 35.0;

/**
 * FLX latch (125-134). Flex is "valid" when the FCU flex temperature word is normal and its value is above TAT.
 * On the ground FLX is active whenever flex is valid, whatever the lever. The latch is set on the ground with flex
 * valid and the lever exactly at 35 (the FLX/MCT detent), and cleared when the lever is exactly at 25 (CL) or 45
 * (TOGA); in flight only the latch counts. A lever that skips the exact detent values keeps it (finding E4).
 */
class FlexSelection {
 public:
  void reset() { latch_ = false; }
  /** Returns is_FLX_active. */
  bool update(const athr_in& in);
  bool latch() const { return latch_; }

 private:
  bool latch_ = false;  // DWork.latch
};

struct N1Rating {
  double n1;     // N1_TLA_percent (also the bus N1 ref)
  bool reverse;  // is_in_reverse
};

/**
 * N1 for the lever position (151-192), piecewise linear between the limits at the detents:
 *   0-25 IDLE -> CLB, 25-35 CLB -> FLX (when flexActive) or MCT, above 35 FLX/MCT -> TOGA (not limited at 45: the line
 *   goes on above 45); reverse (TLA < 0, on the ground only) |IDLE + 1| at 6 -> |REV| at 20, |TLA| below 6 counts as 6.
 * In flight the TLA is limited to >= 0 (no reverse). A NaN TLA takes the reverse line at 6 without `reverse`.
 */
N1Rating n1ForLever(const athr_in& in, bool flexActive);

struct ThrustLimit {
  athr_thrust_limit_type type;
  double percent;
};

/**
 * Thrust limit type and value (193-246).
 * In flight, or on the ground with the engine not operative: alpha floor -> TOGA; TLA > 35 TOGA; > 25 MCT or FLEX;
 * >= 0 CLB; < 0 REVERSE (ground only); NaN NONE (0 %).
 * On the ground with the engine operative: TLA >= 0: TOGA when FLX is not active or TLA > 35, else FLEX; < 0 REVERSE;
 * NaN NONE. So on the ground the limit is never CLB or MCT, and alpha floor is ignored.
 */
ThrustLimit thrustLimit(const athr_in& in, bool flexActive, bool alphaFloor);

/** sim_thrust_mode (429-445) from the raw TLA: 1 reverse, 2 idle (exactly 0), 3 (0, 25), 4 [25, 35), 5 [35, 45),
 *  6 exactly 45, 0 above 45 or NaN. */
double simThrustMode(double tla);

}  // namespace fadec::readable
