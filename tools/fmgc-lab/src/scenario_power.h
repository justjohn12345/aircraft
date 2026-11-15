// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Synthetic FMGC power / health episodes injected on top of a replay (applied last, after the other scenarios and the
// fuzzer), for the whole-computer differential check (src/check_fmgc.cpp): the recordings run the computer
// continuously, so the restart path (generated 452–692: every latch, filter and delay reset, the block-output memories
// kept) is only exercised on frame 0. Episodes, one every 20–240 s (and optionally a late power-up at the start):
//   - computer_running false for 1 frame, 2–10 frames, 0.2–5 s or 5–60 s (an in-flight restart, the outputs held);
//   - power flicker: computer_running toggled every 1–5 frames for 0.5–3 s;
//   - opposite FMGC lost for 1–60 s: fmgc_opp_healthy false, its A bus words dead (SSM failure warning or NCD, data 0)
//     and its AP / FD / A/THR engaged discretes false, optionally with this FMGC restarting too (common power bus);
//   - opposite FMGC A bus words dead with fmgc_opp_healthy still true (wiring), 1–30 s;
//   - the A/THR / AP instinctive disconnect held across a restart (the A/THR inhibit latch must start again).
// Physically inconsistent with the rest of the recording, which is fine: the generated model is the oracle.

#pragma once

#include <cstdint>
#include <random>

#include "FmgcComputer_types.h"

struct PowerScenario {
  enum Kind : int { OffOneFrame = 0, OffFrames, OffShort, OffLong, Flicker, OppLost, OppBusDead, DiscHeld, kKinds };

  std::mt19937_64 rng;
  double t = 0, nextAt = 0, te = 0, len = 0;
  int kind = -1;
  long framesLeft = 0, framesIn = 0;
  int flickerPeriod = 1;
  bool alsoOwnRestart = false;
  double ownOffAt = 0, ownOffLen = 0;
  uint32_t deadSsm = 0;
  double lateStart = 0;  // computer off for the first lateStart seconds
  long episodes = 0;
  long count[kKinds]{};
  long offFrames = 0;

  explicit PowerScenario(uint64_t seed) : rng(seed) {
    lateStart = chance(0.3) ? u(0.1, 20) : 0;
    nextAt = lateStart + u(5, 120);
  }

  double u(double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(rng); }
  bool chance(double p) { return u(0, 1) < p; }
  int irand(int n) { return static_cast<int>(rng() % static_cast<uint64_t>(n)); }

  static void deadBus(base_fmgc_a_bus& b, uint32_t ssm) {
    // every word of the opposite FMGC's A bus: the struct is a sequence of base_arinc_429
    static_assert(sizeof(base_fmgc_a_bus) % sizeof(base_arinc_429) == 0, "A bus is a sequence of ARINC words");
    auto* w = reinterpret_cast<base_arinc_429*>(&b);
    for (size_t i = 0; i < sizeof(base_fmgc_a_bus) / sizeof(base_arinc_429); i++) {
      w[i].SSM = ssm;
      w[i].Data = 0.0F;
    }
  }

  void start() {
    kind = irand(kKinds);
    count[kind]++;
    episodes++;
    te = 0;
    framesIn = 0;
    switch (kind) {
      case OffOneFrame: framesLeft = 1; len = 1e9; break;
      case OffFrames: framesLeft = 2 + irand(9); len = 1e9; break;
      case OffShort: framesLeft = -1; len = u(0.2, 5); break;
      case OffLong: framesLeft = -1; len = u(5, 60); break;
      case Flicker: framesLeft = -1; len = u(0.5, 3); flickerPeriod = 1 + irand(5); break;
      case OppLost:
        framesLeft = -1;
        len = u(1, 60);
        deadSsm = chance(0.5) ? 0U : 1U;  // FW or NCD
        alsoOwnRestart = chance(0.4);
        ownOffAt = u(0, len);
        ownOffLen = chance(0.5) ? u(0.02, 0.5) : u(0.5, 10);
        break;
      case OppBusDead: framesLeft = -1; len = u(1, 30); deadSsm = chance(0.5) ? 0U : 1U; break;
      case DiscHeld:
        framesLeft = -1;
        len = u(2, 25);
        ownOffAt = u(0.5, len - 0.5);
        ownOffLen = u(0.02, 3);
        break;
      default: break;
    }
  }

  void apply(fmgc_inputs& in, double dt) {
    t += dt;
    bool off = false;
    if (t <= lateStart) {
      off = true;
    } else if (kind < 0 && t >= nextAt) {
      start();
    }
    if (kind >= 0) {
      te += dt;
      framesIn++;
      switch (kind) {
        case OffOneFrame:
        case OffFrames: off = true; break;
        case OffShort:
        case OffLong: off = te <= len; break;
        case Flicker: off = ((framesIn - 1) / flickerPeriod) % 2 == 0; break;
        case OppLost:
          in.discrete_inputs.fmgc_opp_healthy = false;
          in.discrete_inputs.ap_opp_engaged = false;
          in.discrete_inputs.fd_opp_engaged = false;
          in.discrete_inputs.athr_opp_engaged = false;
          deadBus(in.bus_inputs.fmgc_opp_bus, deadSsm);
          off = alsoOwnRestart && te >= ownOffAt && te < ownOffAt + ownOffLen;
          break;
        case OppBusDead: deadBus(in.bus_inputs.fmgc_opp_bus, deadSsm); break;
        case DiscHeld:
          in.discrete_inputs.athr_instinctive_disc = true;
          in.discrete_inputs.ap_instinctive_disc = te < 1.0;
          off = te >= ownOffAt && te < ownOffAt + ownOffLen;
          break;
        default: break;
      }
      bool done = framesLeft > 0 ? framesIn >= framesLeft : te >= len;
      if (done) {
        kind = -1;
        nextAt = t + u(20, 240);
      }
    }
    if (off) {
      in.sim_data.computer_running = false;
      offFrames++;
    }
  }
};
