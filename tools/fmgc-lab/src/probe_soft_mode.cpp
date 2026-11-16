// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Experiment for findings C13: the FCU's "SPD/MACH value changed" bit (FCU discrete word 1 bit 15) also pulses when
// its SPD/MACH memory follows the actual Mach in managed speed, and the FMGC resets the ALT soft mode on it.
//
// Replays a recording through the generated model twice: as recorded, and with word 1 bit 15 cleared while the FMGC
// was in managed speed (previous frame) and no SPD knob push/pull or SPD/MACH pb is on the FCU bus (the knob turns are
// not on the bus; check the recording had none in cruise). Prints the ALT soft mode engagements and drops of both runs.
// Open loop: the recorded aircraft does not respond to the different A/THR gain.
// Usage: fmgc-probe-soft-mode <file.fdr>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include "FmgcComputer.h"
#include "replay_core.h"

namespace {

constexpr uint32_t bit(int arincBit) {
  return 1U << (arincBit - 1);
}

struct Sample {
  double t, mach, hInd;
  bool soft, managed, bit15;
};

std::vector<Sample> run(const char* path, bool mask, int& masked) {
  auto computer = std::make_unique<FmgcComputer>();
  computer->initialize();
  Replayer replayer(path, {}, *computer);
  if (!replayer.ok()) {
    std::fprintf(stderr, "error: %s\n", replayer.error().c_str());
    std::exit(2);
  }
  bool managedPrev = false;
  masked = 0;
  replayer.beforeStep = [&]() {
    if (!mask || !managedPrev) {
      return;
    }
    auto& w1 = replayer.mutableInputs().bus_inputs.fcu_bus.fcu_discrete_word_1;
    auto v = static_cast<uint32_t>(w1.Data);
    const bool pilot = v & (bit(11) | bit(12) | bit(21));  // SPD pushed, pulled, SPD/MACH pb
    if ((v & bit(15)) && !pilot) {
      w1.Data = static_cast<float>(v & ~bit(15));
      masked++;
    }
  };
  std::vector<Sample> out;
  Replayer::Frame f;
  while (replayer.next(f)) {
    const fmgc_outputs& o = computer->getExternalOutputs().out;
    managedPrev = o.ap_fd_logic.auto_spd_control_active;
    const auto w1 = static_cast<uint32_t>(o.data.bus_inputs.fcu_bus.fcu_discrete_word_1.Data);
    out.push_back({f.fdr->base.simulation_time_s, o.logic.adr_computation_data.mach.Data,
                   o.logic.altitude_indicated_ft.Data, o.ap_fd_logic.alt_soft_mode_active, managedPrev,
                   (w1 & bit(15)) != 0});
  }
  return out;
}

void report(const char* name, const std::vector<Sample>& s) {
  double on = 0;
  int drops = 0;
  std::printf("%s:\n", name);
  for (size_t i = 1; i < s.size(); i++) {
    if (s[i].soft) {
      on += s[i].t - s[i - 1].t;
    }
    if (s[i].soft != s[i - 1].soft) {
      drops += !s[i].soft;
      std::printf("  t %7.1f soft %s  Mach %.4f  H %.0f\n", s[i].t, s[i].soft ? "ON " : "OFF", s[i].mach, s[i].hInd);
    }
  }
  std::printf("  soft mode active %.1f min, %d drops\n", on / 60, drops);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: fmgc-probe-soft-mode <file.fdr>\n");
    return 2;
  }
  int masked = 0, none = 0;
  report("as recorded", run(argv[1], false, none));
  auto fixed = run(argv[1], true, masked);
  report("bit 15 masked in managed speed", fixed);
  std::printf("frames with bit 15 masked: %d\n", masked);
  return 0;
}
