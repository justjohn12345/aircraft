// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FCU shadow mode (personal build, ini [FCU] SHADOW_ENABLED): the readable FCU (tools/fmgc-lab/readable/fcu, a
// hand-written rewrite of the generated FcuComputer) runs alongside the generated one on exactly the same inputs every
// frame, and every leaf of fcu_outputs is compared by bit pattern (ShadowChecker). The generated outputs stay the only
// ones the aircraft uses. Design, log format and offline reproduction: tools/fmgc-lab/docs/shadow.md.

#pragma once

#include <memory>

#include "FcuComputer_types.h"
#include "ShadowChecker.h"

namespace fcu::readable {
class FcuComputer;
}

class FcuShadow {
 public:
  /** Construct together with the generated model, before its first step (the readable FCU is initialize()d here). */
  FcuShadow();
  ~FcuShadow();
  FcuShadow(const FcuShadow&) = delete;
  FcuShadow& operator=(const FcuShadow&) = delete;

  /**
   * One frame: steps the readable FCU with `in` (the inputs the generated model just stepped with, computer_running
   * included) and compares its outputs with `generated` (the generated model's outputs after that step).
   */
  void step(const fcu_inputs& in, const fcu_outputs& generated);

  shadow::ShadowChecker& checker() { return checker_; }
  const shadow::ShadowChecker& checker() const { return checker_; }
  /** The readable computer (tests and diagnostics). */
  const fcu::readable::FcuComputer& readable() const { return *readable_; }

  /** The log of the FCU shadow: <fdr name>.fcu-shadow.log in `directory`. */
  static std::unique_ptr<shadow::ShadowLog> makeLog(const std::string& directory);

 private:
  std::unique_ptr<fcu::readable::FcuComputer> readable_;
  shadow::ShadowChecker checker_;
};
