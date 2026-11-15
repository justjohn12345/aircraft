// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Readable rewrite of the FMGC output packing (Phase 5e): everything from the computed stage results to
// `FmgcComputer_Y.out` (generated FmgcComputer.cpp 4642–4925 packing, 4926–5106 assembly). Stock behaviour: must
// reproduce the generated FmgcComputer exactly (checked by src/check_output.cpp). Spec and word/bit tables:
// docs/output-buses.md.
//
// The packing has no state of its own: every output field is a function of this frame's stage results
// (inputs, logic bus, ap_fd_logic bus, outer-loop commands, A/THR outputs) and parameters. It only runs when the
// computer is running; otherwise the generated step() leaves FmgcComputer_Y.out as it was (the last running frame's
// outputs, or out_Y0 before the first one).
//
// Conventions:
//   - bit numbers are ARINC 429 bit numbers, 1-based, as every consumer reads them (Arinc429Utils::bitFromValue:
//     `(uint32)Data >> (bit - 1)`, the FMGC's own BitfromLabel). Bits 1–8 are the label, 9–10 the SDI, 11–29 the data
//     field, 30–31 the SSM, 32 parity. The generated packer (`MATLABFunction_gy`) shifts element i of a 19-element
//     bool vector by i + 10, so element i is ARINC bit i + 11: every discrete word uses exactly the data field.
//   - a word is {SSM, Data}: SSM is a SignStatusMatrix value as uint32, Data a float32 (for discrete words, the
//     packed bit pattern converted to float, exact since the pattern spans 19 bits).

#pragma once

#include <cstdint>

#include "FmgcComputer_types.h"

namespace fmgc::readable::output {

/** This frame's results of the stages before the packing (generated rtb_BusAssignment_jmp, outer-loop outputs). */
struct StageResults {
  const fmgc_inputs& data;                      // the step's inputs, unchanged (rtb_BusAssignment_jmp.data)
  const base_fmgc_logic_outputs& logic;         // input conditioning + engagement (logic bus)
  const base_fmgc_ap_fd_logic_outputs& apFd;    // modes, targets, flags (ap_fd_logic bus, this frame)
  const ap_raw_output& outerLoops;              // FmgcOuterLoops commands (FD, AP, flare law)
  const base_fmgc_athr_outputs& athr;           // A/THR modes, FMA, N1 command
};

// FMGC A bus discrete words (also copied to the B bus where noted). One function per word; bit tables in the .cpp.
base_arinc_429 discreteWord1(const StageResults& r);    // vertical modes (A and B)
base_arinc_429 discreteWord2(const StageResults& r);    // lateral modes (A and B)
base_arinc_429 discreteWord3(const StageResults& r);    // armed modes (A and B)
base_arinc_429 discreteWord4(const StageResults& r);    // AP/FD engagement, LAND capability (A and B)
base_arinc_429 discreteWord5(const StageResults& r);    // speed control (A only)
base_arinc_429 discreteWord6(const StageResults& r);    // all spare (A only)
base_arinc_429 discreteWord7(const StageResults& r);    // TCAS (A only)
base_arinc_429 atsDiscreteWord(const StageResults& r);  // A/THR engagement and modes (A and B)
base_arinc_429 atsFmaDiscreteWord(const StageResults& r);  // A/THR FMA column 1 (A only)

/** FMGC A bus: to the opposite FMGC (its fmgc_opp_bus), the FCU, the DMCs/FWCs (through L:vars). */
base_fmgc_a_bus fmgcABus(const StageResults& r);
/** FMGC B bus: to the ELACs/SECs (fmgc_1_bus / fmgc_2_bus). */
base_fmgc_b_bus fmgcBBus(const StageResults& r);
/** The hardwired discrete outputs. */
base_fmgc_discrete_outputs discreteOutputs(const StageResults& r);

/** The whole `FmgcComputer_Y.out` of a running frame. */
fmgc_outputs assemble(const StageResults& r);

}  // namespace fmgc::readable::output
