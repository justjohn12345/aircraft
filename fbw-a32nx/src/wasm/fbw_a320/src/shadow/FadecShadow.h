// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FADEC shadow mode (personal build, ini [FADEC] SHADOW_ENABLED): the readable FADEC (tools/fmgc-lab/readable/fadec, a
// hand-written rewrite of the generated FadecComputer) runs alongside each generated one (engine 1 and 2) on exactly the
// same inputs every frame, and every leaf of athr_out is compared by bit pattern (ShadowChecker). The generated outputs
// stay the only ones the aircraft uses. Design, log format and offline reproduction: tools/fmgc-lab/docs/shadow.md.

#pragma once

#include <memory>

#include "FadecComputer_types.h"
#include "ShadowChecker.h"

namespace fadec::readable {
class FadecComputer;
}

class FadecShadow {
 public:
  /**
   * engine: 1 or 2, for the log only. Construct before the generated model's first step: the readable FADEC starts from
   * initialize(), which equals a generated FadecComputer that was only constructed (the wrapper never calls its
   * initialize(); every initial condition is 0, see docs/fadec.md).
   */
  explicit FadecShadow(int engine);
  ~FadecShadow();
  FadecShadow(const FadecShadow&) = delete;
  FadecShadow& operator=(const FadecShadow&) = delete;

  /** One frame: steps the readable FADEC with `in` and compares its outputs with `generated` (after that step). */
  void step(const athr_in& in, const athr_out& generated);

  shadow::ShadowChecker& checker() { return checker_; }
  const shadow::ShadowChecker& checker() const { return checker_; }
  /** The readable computer (tests and diagnostics). */
  const fadec::readable::FadecComputer& readable() const { return *readable_; }

  /** The log of both FADEC shadows: <fdr name>.fadec-shadow.log in `directory`. */
  static std::unique_ptr<shadow::ShadowLog> makeLog(const std::string& directory);

 private:
  std::unique_ptr<fadec::readable::FadecComputer> readable_;
  shadow::ShadowChecker checker_;
};
