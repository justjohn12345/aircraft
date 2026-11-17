// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The FADEC's A/THR interface (generated FadecComputer.cpp 114-124, 138-150): A/THR disabled after a 15 s instinctive
// disconnect, "A/THR control active" (the FADEC follows the FMGC's N1 command) and the thrust lock (THR LK).
//
// FMGC inputs, all from the FCU bus (the priority FMGC's words, passed through by the FCU; ARINC bits, 1-based):
//   ATS discrete word bit 13   A/THR engaged          (FMGC athr_own_engaged)
//   ATS discrete word bit 14   A/THR active           (FMGC athr_active)
//   ATS discrete word bit 23   alpha floor            (FMGC alpha_floor_mode_active, includes TOGA LK)
//   N1 command word            SSM normal operation is part of "engaged"; its value is the N1 command
// The bits are read without looking at the ATS word's SSM.

#pragma once

#include "FadecComputer_types.h"
#include "fadec_blocks.h"

namespace fadec::readable {

/** Instinctive disconnect hold time that disables A/THR until reset (CompareToConstant_const, s). */
constexpr double kAthrDisableHoldSeconds = 15.0;

/** Thrust lever detent bands for the thrust lock (deg TLA): CL 24-26, FLX/MCT 34-36, both ends included. */
constexpr double kClDetentLow = 24.0, kClDetentHigh = 26.0;
constexpr double kMctDetentLow = 34.0, kMctDetentHigh = 36.0;

/** The A/THR-related bits of the ATS discrete word (BitfromLabel*_bit). */
struct AtsBits {
  bool engaged;      // bit 13
  bool active;       // bit 14
  bool alphaFloor;   // bit 23

  static AtsBits read(const base_fcu_bus& fcu);
};

/** True when the thrust lever is outside both detent bands (146-147). NaN counts as inside. */
bool outsideClAndMctDetents(double tla);

class AthrInterface {
 public:
  void reset();

  /**
   * One frame, in the generated order:
   *  1. A/THR disabled (114-118): latched when an instinctive disconnect input has been held >= 15 s of simulation time;
   *     reset only by ATHR_reset_disable (reset wins).
   *  2. Engaged (119-124): ATS bit 13 and the N1 command word normal; `engageEdge` = its rising edge (the first frame
   *     after power-up does not count as an edge).
   *  3. A/THR control active (139-143): set on the engage edge; reset (wins) by an instinctive disconnect, A/THR
   *     disabled, or ATS bit 13 false. ATS bit 14 (A/THR active) is not part of it.
   *  4. Thrust lock (144-150): set when control active is lost (rising edge of the flip-flop's second output) with the
   *     thrust lever in the CL or FLX/MCT band; reset (wins) when the lever is outside both bands, or by control active.
   *     Any loss of control active counts, the instinctive disconnect included (finding E1).
   */
  void update(const athr_in& in, const AtsBits& bits);

  bool disabled() const { return disabled_.q; }
  bool engaged() const { return engaged_; }
  bool controlActive() const { return control_.q; }
  bool thrustLock() const { return thrustLock_.q; }

  // state access for the checker
  const blocks::TimeSinceCondition& disconnectTimer() const { return disconnectHeld_; }
  const blocks::EdgeDetector& engageEdge() const { return engageEdge_; }
  const blocks::EdgeDetector& lostEdge() const { return lostEdge_; }

 private:
  blocks::TimeSinceCondition disconnectHeld_;  // sf_TimeSinceCondition
  blocks::SrFlipFlop disabled_;                // Memory_PreviousInput, Logic_table
  blocks::EdgeDetector engageEdge_;            // sf_MATLABFunction_f (rising)
  blocks::SrFlipFlop control_;                 // Memory_PreviousInput_p, Logic_table_n
  blocks::EdgeDetector lostEdge_;              // sf_MATLABFunction_b (rising edge of the second output)
  blocks::SrFlipFlop thrustLock_;              // Memory_PreviousInput_j, Logic_table_h
  bool engaged_ = false;
};

}  // namespace fadec::readable
