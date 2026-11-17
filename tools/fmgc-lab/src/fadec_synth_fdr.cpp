// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Writes a personal-format 3209004 recording from any recording: every record is copied (missing FCU / VNAV parts are
// written as zeros), and the FADEC model records are filled from two generated FadecComputers stepped with the
// synthetic drive of src/fadec_drive.h (optional --script SEED / --athr SEED injectors). As in the sim, FADEC 1's stock
// record part (bus and output) is the same step's output. Unrecorded gaps are stepped as fadec-replay fills them.
// Used to test fadec-replay and the recorded-input path of fadec-check before real 3209004 recordings exist, and
// (--corrupt FRAME) that fadec-replay reports a changed output. Never write into the corpus.
//
// Usage: fadec-synth-fdr [--script SEED] [--athr SEED] [--corrupt FRAME] <in.fdr> <out.fdr>

#include <zlib.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "FadecComputer.h"
#include "fadec_drive.h"
#include "fadec_scenarios.h"

namespace {
template <typename T>
void put(gzFile f, const T& v) {
  gzwrite(f, &v, sizeof(T));
}
}  // namespace

int main(int argc, char** argv) {
  long scriptSeed = -1, athrSeed = -1, corrupt = -1;
  std::string in, out;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--script" && i + 1 < argc) {
      scriptSeed = std::atol(argv[++i]);
    } else if (a == "--athr" && i + 1 < argc) {
      athrSeed = std::atol(argv[++i]);
    } else if (a == "--corrupt" && i + 1 < argc) {
      corrupt = std::atol(argv[++i]);
    } else if (in.empty()) {
      in = a;
    } else if (out.empty()) {
      out = a;
    }
  }
  if (in.empty() || out.empty()) {
    std::fprintf(stderr, "usage: fadec-synth-fdr [--script SEED] [--athr SEED] [--corrupt FRAME] <in.fdr> <out.fdr>\n");
    return 2;
  }
  FadecInputDrive::Options opt;
  opt.synthetic = true;
  FadecInputDrive drive(in, opt);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }
  std::unique_ptr<FadecComputer> computer[2] = {std::make_unique<FadecComputer>(), std::make_unique<FadecComputer>()};
  for (int e = 0; e < 2; e++) {
    computer[e]->initialize();
    drive.feedback[e] = &computer[e]->getExternalOutputs().out;
  }
  auto ext = std::make_unique<FadecComputer::ExternalInputs_FadecComputer_T>();
  std::unique_ptr<fadec_scenarios::ScriptScenario> script;
  if (scriptSeed >= 0) script = std::make_unique<fadec_scenarios::ScriptScenario>(scriptSeed);
  std::unique_ptr<fadec_scenarios::AthrScenario> athr;
  if (athrSeed >= 0) athr = std::make_unique<fadec_scenarios::AthrScenario>(athrSeed);

  gzFile f = gzopen(out.c_str(), "wb");
  if (!f) {
    std::fprintf(stderr, "error: cannot write %s\n", out.c_str());
    return 2;
  }
  put(f, FDR_INTERFACE_VERSION_FADEC);
  long frames = 0;
  auto zeroFcu = std::make_unique<FcuRecord>();
  std::memset(zeroFcu.get(), 0, sizeof(FcuRecord));
  athr_in lastWritten[2]{};
  while (drive.next([&](athr_in (&s)[2], const FadecInputDrive::StepInfo& info) {
    if (info.gapFill) {
      // as fadec-replay fills the gap: the last recorded inputs held, the disconnect inputs cleared
      for (int e = 0; e < 2; e++) {
        athr_in h = lastWritten[e];
        h.input.ATHR_disconnect = false;
        h.input.ATHR_reset_disable = false;
        h.time = s[e].time;
        s[e] = h;
      }
    } else {
      if (script) script->apply(s, s[0].time.dt);
      if (athr) athr->apply(s, s[0].time.dt);
      lastWritten[0] = s[0];
      lastWritten[1] = s[1];
    }
    for (int e = 0; e < 2; e++) {
      ext->in = s[e];
      computer[e]->setExternalInputs(ext.get());
      computer[e]->step();
    }
    if (info.gapFill) return;
    const FdrFrame& fr = *info.fdr;
    put(f, fr.base);
    put(f, fr.specific);
    for (const auto& e : fr.elac) put(f, e.bus_outputs), put(f, e.discrete_outputs), put(f, e.analog_outputs);
    for (const auto& c : fr.sec) put(f, c.bus_outputs), put(f, c.discrete_outputs), put(f, c.analog_outputs);
    for (const auto& c : fr.fac) put(f, c.bus_outputs), put(f, c.discrete_outputs), put(f, c.analog_outputs);
    const auto& m = fr.fmgc1;
    put(f, m.logic), put(f, m.ap_fd_logic), put(f, m.ap_fd_outer_loops), put(f, m.athr), put(f, m.discrete_outputs);
    put(f, m.bus_outputs), put(f, m.bus_inputs), put(f, m.discrete_inputs), put(f, m.fms_inputs);
    athr_out o[2] = {computer[0]->getExternalOutputs().out, computer[1]->getExternalOutputs().out};
    put(f, o[0].fadec_bus_output), put(f, o[0].output);
    const FcuRecord& fcu = fr.hasFcu ? fr.fcu : *zeroFcu;
    put(f, fcu.inputs), put(f, fcu.logic), put(f, fcu.discrete_outputs), put(f, fcu.bus_outputs);
    FmsVnavRecord vnav{};
    double diag[FDR_FMS_VNAV_DIAG_COUNT]{};
    put(f, fr.hasFmsVnav ? fr.fmsVnav : vnav);
    gzwrite(f, fr.hasFmsVnavDiag ? fr.fmsVnavDiag : diag, sizeof(diag));
    if (info.frame == corrupt) {
      o[1].output.N1_c_percent += 1.0;
    }
    for (int e = 0; e < 2; e++) {
      // FlightDataRecorder::writeFadecModel: out.time, data, input, fcu_input (the inputs), data_computed, output, bus
      put(f, o[e].time), put(f, o[e].data), put(f, o[e].input), put(f, o[e].fcu_input);
      put(f, o[e].data_computed), put(f, o[e].output), put(f, o[e].fadec_bus_output);
    }
    frames++;
  })) {
  }
  gzclose(f);
  std::printf("%s: %ld frames (interface %llu), %ld gap-fill steps%s\n", out.c_str(), frames,
              static_cast<unsigned long long>(FDR_INTERFACE_VERSION_FADEC), drive.gapSteps(),
              corrupt >= 0 ? "; one output corrupted" : "");
  return 0;
}
