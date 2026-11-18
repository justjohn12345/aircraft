// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FAC shadow mode (personal build, ini [FAC] SHADOW_ENABLED): the readable FAC (tools/fmgc-lab/readable/fac, a
// hand-written rewrite of the generated FacComputer) runs alongside each generated one (FAC 1 and 2) on exactly the same
// inputs every frame the generated model steps, and every leaf of fac_outputs is compared by bit pattern
// (ShadowChecker), two NaNs being equal whatever their sign and payload. The generated outputs stay the only ones the
// aircraft uses. Design, log format and offline reproduction: tools/fmgc-lab/docs/shadow.md.

#pragma once

#include <memory>

#include "FacComputer_types.h"
#include "ShadowChecker.h"

namespace fac::readable {
class FacComputer;
}

class FacShadow {
 public:
  /**
   * unit: 1 or 2, for the log only. Construct together with the generated model, before its first step: the readable FAC
   * starts from initialize(), as the generated one does (fac/Fac.cpp initialize()s it at construction).
   */
  explicit FacShadow(int unit);
  ~FacShadow();
  FacShadow(const FacShadow&) = delete;
  FacShadow& operator=(const FacShadow&) = delete;

  /**
   * One frame: steps the readable FAC with `in` (the inputs the generated model just stepped with, computer_running
   * included) and compares its outputs with `generated` (the generated model's outputs after that step).
   */
  void step(const fac_inputs& in, const fac_outputs& generated);

  shadow::ShadowChecker& checker() { return checker_; }
  const shadow::ShadowChecker& checker() const { return checker_; }
  /** The readable computer (tests and diagnostics). */
  const fac::readable::FacComputer& readable() const { return *readable_; }

  /** The log of both FAC shadows: <fdr name>.fac-shadow.log in `directory`. */
  static std::unique_ptr<shadow::ShadowLog> makeLog(const std::string& directory);

 private:
  std::unique_ptr<fac::readable::FacComputer> readable_;
  shadow::ShadowChecker checker_;
};
