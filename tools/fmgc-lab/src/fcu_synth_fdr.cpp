// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Writes a personal-format (3209001) recording from any recording: every record is copied, and the FCU record is
// filled from the generated FcuComputer stepped with the synthetic drive of src/fcu_drive.h (FMGC 2 bus as recorded,
// pilot actions reconstructed from the recorded FCU bus; optional --script SEED / --panel SEED injectors). The FCU
// record holds the inputs of each recorded frame's step and the outputs after it, as FlightDataRecorder::writeFcu()
// does; unrecorded gaps are stepped as fcu-replay fills them. Used to test fcu-replay and the recorded-input path of
// fcu-check before real 3209001 recordings exist, (--corrupt FRAME) that fcu-replay reports a changed output, and
// (--from FRAME: frames before FRAME are stepped but not written, like a file split) fcu-replay --sync.
// Never write into the corpus.
//
// Usage: fcu-synth-fdr [--script SEED] [--panel SEED] [--corrupt FRAME] [--from FRAME] <in.fdr> <out.fdr>

#include <zlib.h>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "FcuComputer.h"
#include "fcu_drive.h"
#include "fcu_scenarios.h"

namespace {
template <typename T>
void put(gzFile f, const T& v) {
  gzwrite(f, &v, sizeof(T));
}
}  // namespace

int main(int argc, char** argv) {
  long scriptSeed = -1, panelSeed = -1, corrupt = -1, from = 0;
  std::string in, out;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--script" && i + 1 < argc) {
      scriptSeed = std::atol(argv[++i]);
    } else if (a == "--panel" && i + 1 < argc) {
      panelSeed = std::atol(argv[++i]);
    } else if (a == "--from" && i + 1 < argc) {
      from = std::atol(argv[++i]);
    } else if (a == "--corrupt" && i + 1 < argc) {
      corrupt = std::atol(argv[++i]);
    } else if (in.empty()) {
      in = a;
    } else if (out.empty()) {
      out = a;
    }
  }
  if (in.empty() || out.empty()) {
    std::fprintf(stderr,
                 "usage: fcu-synth-fdr [--script SEED] [--panel SEED] [--corrupt FRAME] [--from FRAME] <in.fdr> "
                 "<out.fdr>\n");
    return 2;
  }
  FcuInputDrive::Options opt;
  opt.synthetic = true;
  opt.reconstruct = true;
  FcuInputDrive drive(in, opt);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }
  auto computer = std::make_unique<FcuComputer>();
  computer->initialize();
  auto ext = std::make_unique<FcuComputer::ExternalInputs_FcuComputer_T>();
  drive.feedback = &computer->getExternalOutputs().out;
  std::unique_ptr<fcu_scenarios::ScriptScenario> script;
  if (scriptSeed >= 0) script = std::make_unique<fcu_scenarios::ScriptScenario>(scriptSeed);
  std::unique_ptr<fcu_scenarios::RandomPanelScenario> panel;
  if (panelSeed >= 0) panel = std::make_unique<fcu_scenarios::RandomPanelScenario>(panelSeed);

  gzFile f = gzopen(out.c_str(), "wb");
  if (!f) {
    std::fprintf(stderr, "error: cannot write %s\n", out.c_str());
    return 2;
  }
  put(f, FDR_INTERFACE_VERSION_FCU);
  long frames = 0;
  fcu_inputs lastWritten{};
  while (drive.next([&](fcu_inputs& s, const FcuInputDrive::StepInfo& info) {
    if (info.gapFill) {
      // as fcu-replay fills the gap: the last recorded inputs, events cleared
      fcu_inputs h = lastWritten;
      fcu_drive::clearEvents(h);
      h.time = s.time;
      s = h;
    } else {
      if (script) script->apply(s, s.time.dt);
      if (panel) panel->apply(s, s.time.dt);
      lastWritten = s;
    }
    ext->in = s;
    computer->setExternalInputs(ext.get());
    computer->step();
    if (info.gapFill || info.frame < from) return;
    const FdrFrame& fr = *info.fdr;
    put(f, fr.base);
    put(f, fr.specific);
    for (const auto& e : fr.elac) put(f, e.bus_outputs), put(f, e.discrete_outputs), put(f, e.analog_outputs);
    for (const auto& c : fr.sec) put(f, c.bus_outputs), put(f, c.discrete_outputs), put(f, c.analog_outputs);
    for (const auto& c : fr.fac) put(f, c.bus_outputs), put(f, c.discrete_outputs), put(f, c.analog_outputs);
    const auto& m = fr.fmgc1;
    put(f, m.logic), put(f, m.ap_fd_logic), put(f, m.ap_fd_outer_loops), put(f, m.athr), put(f, m.discrete_outputs);
    put(f, m.bus_outputs), put(f, m.bus_inputs), put(f, m.discrete_inputs), put(f, m.fms_inputs);
    put(f, fr.fadec[0].bus_outputs), put(f, fr.fadec[0].outputs);
    fcu_outputs o = computer->getExternalOutputs().out;
    if (info.frame == corrupt) {
      o.bus_outputs.selected_alt_ft.Data += 100.0F;
    }
    put(f, s), put(f, o.logic), put(f, o.discrete_outputs), put(f, o.bus_outputs);
    frames++;
  })) {
  }
  gzclose(f);
  std::printf("%s: %ld frames (interface %llu), %ld gap-fill steps; reconstruct %ld presses, %ld turns%s\n", out.c_str(),
              frames, static_cast<unsigned long long>(FDR_INTERFACE_VERSION_FCU), drive.gapSteps(),
              drive.reconstructedPresses, drive.reconstructedTurns, corrupt >= 0 ? "; one output corrupted" : "");
  return 0;
}
