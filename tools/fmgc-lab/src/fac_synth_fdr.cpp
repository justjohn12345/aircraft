// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Writes a personal-format 3209005 recording from any recording: every record is copied (missing FCU / VNAV / FADEC
// model parts are written as zeros) and the FBW input blocks are added. The FAC blocks hold the inputs src/fac_drive.h
// rebuilds from the source record; two generated FacComputers are stepped with them as the wrapper would step them, and
// their outputs replace the recorded FAC outputs while the FAC is healthy (the wrapper's unhealthy outputs are kept).
// The ELAC and SEC blocks are zero except their time (the ELACs' monotonic time tells whether a FAC stepped).
// --outage FRAME LEN: FAC 1 does not step on frames FRAME .. FRAME + LEN - 1 (a power outage over 10 ms while healthy):
// its block repeats its last step's and its outputs are held, as in the sim. --corrupt FRAME changes one output on that
// frame (fac-replay must report it). Used to test the recorded-input path of fac-replay / fac-check before real
// 3209005 recordings exist. Never write into the corpus.
//
// Usage: fac-synth-fdr [--outage FRAME LEN] [--corrupt FRAME] <in.fdr> <out.fdr>

#include <xmmintrin.h>
#include <zlib.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "fac_drive.h"

namespace {
template <typename T>
void put(gzFile f, const T& v) {
  gzwrite(f, &v, sizeof(T));
}
}  // namespace

int main(int argc, char** argv) {
  long outageFrom = -1, outageLen = 0, corrupt = -1;
  std::string in, out;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--outage" && i + 2 < argc) {
      outageFrom = std::atol(argv[++i]);
      outageLen = std::atol(argv[++i]);
    } else if (a == "--corrupt" && i + 1 < argc) {
      corrupt = std::atol(argv[++i]);
    } else if (in.empty()) {
      in = a;
    } else if (out.empty()) {
      out = a;
    }
  }
  if (in.empty() || out.empty()) {
    std::fprintf(stderr, "usage: fac-synth-fdr [--outage FRAME LEN] [--corrupt FRAME] <in.fdr> <out.fdr>\n");
    return 2;
  }
  // as fac-replay and the sim: denormals flushed to zero (MXCSR FTZ and DAZ)
  _mm_setcsr(_mm_getcsr() | 0x8040);
  FacInputDrive::Options opt;
  opt.rebuild = true;
  FacInputDrive drive(in, opt);
  if (!drive.ok()) {
    std::fprintf(stderr, "error: %s\n", drive.error().c_str());
    return 2;
  }
  std::unique_ptr<FacComputer> computer[2] = {std::make_unique<FacComputer>(), std::make_unique<FacComputer>()};
  for (auto& c : computer) c->initialize();
  auto ext = std::make_unique<FacComputer::ExternalInputs_FacComputer_T>();

  gzFile f = gzopen(out.c_str(), "wb");
  if (!f) {
    std::fprintf(stderr, "error: cannot write %s\n", out.c_str());
    return 2;
  }
  put(f, FDR_INTERFACE_VERSION_FBW);
  long frames = 0, held = 0;
  auto zeroFcu = std::make_unique<FcuRecord>();
  std::memset(zeroFcu.get(), 0, sizeof(FcuRecord));
  FadecModelRecord zeroFadec[2];
  std::memset(static_cast<void*>(zeroFadec), 0, sizeof(zeroFadec));
  auto elac = std::make_unique<elac_inputs[]>(2);
  auto sec = std::make_unique<sec_inputs[]>(3);
  std::memset(static_cast<void*>(elac.get()), 0, 2 * sizeof(elac_inputs));
  std::memset(static_cast<void*>(sec.get()), 0, 3 * sizeof(sec_inputs));
  fac_inputs lastStep[2];
  std::memset(static_cast<void*>(lastStep), 0, sizeof(lastStep));

  while (drive.next([&](fac_inputs (&s)[2], const FacInputDrive::StepInfo& info) {
    const bool outage = info.frame >= outageFrom && info.frame < outageFrom + outageLen;
    held += outage;
    for (int i = 0; i < 2; i++) {
      if (i == 0 && outage) continue;  // FAC 1 not stepped: its inputs and outputs stay those of its last step
      lastStep[i] = s[i];
      ext->in = s[i];
      computer[i]->setExternalInputs(ext.get());
      computer[i]->step();
    }
    const FdrFrame& fr = *info.fdr;
    put(f, fr.base);
    put(f, fr.specific);
    for (const auto& e : fr.elac) put(f, e.bus_outputs), put(f, e.discrete_outputs), put(f, e.analog_outputs);
    for (const auto& c : fr.sec) put(f, c.bus_outputs), put(f, c.discrete_outputs), put(f, c.analog_outputs);
    for (int i = 0; i < 2; i++) {
      FacRecord r = fr.fac[i];
      if (info.healthy[i]) {
        const fac_outputs& o = computer[i]->getExternalOutputs().out;
        r.bus_outputs = o.bus_outputs;
        r.discrete_outputs = o.discrete_outputs;
        r.analog_outputs = o.analog_outputs;
        if (i == 1 && info.frame == corrupt) r.bus_outputs.v_ls_kn.Data += 1.0F;
      }
      put(f, r.bus_outputs), put(f, r.discrete_outputs), put(f, r.analog_outputs);
    }
    const auto& m = fr.fmgc1;
    put(f, m.logic), put(f, m.ap_fd_logic), put(f, m.ap_fd_outer_loops), put(f, m.athr), put(f, m.discrete_outputs);
    put(f, m.bus_outputs), put(f, m.bus_inputs), put(f, m.discrete_inputs), put(f, m.fms_inputs);
    put(f, fr.fadec[0].bus_outputs), put(f, fr.fadec[0].outputs);
    const FcuRecord& fcu = fr.hasFcu ? fr.fcu : *zeroFcu;
    put(f, fcu.inputs), put(f, fcu.logic), put(f, fcu.discrete_outputs), put(f, fcu.bus_outputs);
    FmsVnavRecord vnav{};
    double diag[FDR_FMS_VNAV_DIAG_COUNT]{};
    put(f, fr.hasFmsVnav ? fr.fmsVnav : vnav);
    gzwrite(f, fr.hasFmsVnavDiag ? fr.fmsVnavDiag : diag, sizeof(diag));
    for (int e = 0; e < 2; e++) {
      const FadecModelRecord& r = fr.hasFadecModel ? fr.fadecModel[e] : zeroFadec[e];
      put(f, r.in.time), put(f, r.in.data), put(f, r.in.input), put(f, r.in.fcu_input);
      put(f, r.data_computed), put(f, r.output), put(f, r.bus_outputs);
    }
    // FlightDataRecorder::writeFbwInputs: ELAC 1, 2, SEC 1, 2, 3, FAC 1, 2 (the ELACs and SECs: this frame's time)
    for (int k = 0; k < 2; k++) elac[k].time = s[1].time;
    for (int k = 0; k < 3; k++) sec[k].time = s[1].time;
    gzwrite(f, elac.get(), 2 * sizeof(elac_inputs));
    gzwrite(f, sec.get(), 3 * sizeof(sec_inputs));
    put(f, lastStep[0]), put(f, lastStep[1]);
    frames++;
  })) {
  }
  gzclose(f);
  std::printf("%s: %ld frames (interface %llu), FAC 1 held on %ld frames%s\n", out.c_str(), frames,
              static_cast<unsigned long long>(FDR_INTERFACE_VERSION_FBW), held, corrupt >= 0 ? "; one output corrupted" : "");
  return 0;
}
