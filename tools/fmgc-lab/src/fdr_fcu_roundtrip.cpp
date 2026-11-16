// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Round trip of the personal FDR format 3209001 (3200006 plus the FCU): writes a synthetic file field by field in the
// order of FlightDataRecorder::update() / writeFcu(), with marker values, and reads it back with FdrReader. The file
// is kept for checking fdr2csv on it.
// Usage: fdr-fcu-roundtrip <out.fdr>

#include <zlib.h>
#include <cstdio>
#include <cstring>
#include <memory>

#include "fdr.h"

namespace {

template <typename T>
void put(gzFile f, const T& v) {
  gzwrite(f, &v, sizeof(T));
}

void fill(FdrFrame& fr, int i) {
  std::memset(&fr, 0, sizeof(fr));
  fr.base.simulation_time_s = 100.0 + i;
  fr.fmgc1.fms_inputs.v_managed_kts = 250.0 + i;
  fr.fadec[0].outputs.N1_c_percent = 80.0 + i;  // last field before the FCU
  fr.fcu.inputs.time.dt = 0.25 + i;
  fr.fcu.inputs.discrete_inputs.afs_inputs.spd_knob.turns = static_cast<int8_T>(i - 1);
  fr.fcu.inputs.bus_inputs.fmgc_2_bus.mach.Data = 0.78F + 0.01F * i;
  fr.fcu.logic.afs.spd_mach_display_value = 0.5F + i;
  fr.fcu.discrete_outputs.afs_outputs.spd_mach_managed = i % 2;
  fr.fcu.bus_outputs.selected_spd_kts.Data = 123.5F + i;
  fr.fcu.bus_outputs.fcu_discrete_word_1.Data = static_cast<float>(1 << 14);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: fdr-fcu-roundtrip <out.fdr>\n");
    return 2;
  }
  constexpr int kFrames = 3;
  auto fr = std::make_unique<FdrFrame>();
  gzFile f = gzopen(argv[1], "wb");
  put(f, FDR_INTERFACE_VERSION_FCU);
  for (int i = 0; i < kFrames; i++) {
    fill(*fr, i);
    put(f, fr->base);
    put(f, fr->specific);
    for (const auto& e : fr->elac) {
      put(f, e.bus_outputs), put(f, e.discrete_outputs), put(f, e.analog_outputs);
    }
    for (const auto& s : fr->sec) {
      put(f, s.bus_outputs), put(f, s.discrete_outputs), put(f, s.analog_outputs);
    }
    for (const auto& c : fr->fac) {
      put(f, c.bus_outputs), put(f, c.discrete_outputs), put(f, c.analog_outputs);
    }
    const auto& m = fr->fmgc1;
    put(f, m.logic), put(f, m.ap_fd_logic), put(f, m.ap_fd_outer_loops), put(f, m.athr), put(f, m.discrete_outputs);
    put(f, m.bus_outputs), put(f, m.bus_inputs), put(f, m.discrete_inputs), put(f, m.fms_inputs);
    put(f, fr->fadec[0].bus_outputs), put(f, fr->fadec[0].outputs);
    put(f, fr->fcu.inputs), put(f, fr->fcu.logic), put(f, fr->fcu.discrete_outputs), put(f, fr->fcu.bus_outputs);
  }
  gzclose(f);

  FdrReader reader(argv[1]);
  if (!reader.ok() || !reader.hasFcu()) {
    std::fprintf(stderr, "FAIL: %s\n", reader.ok() ? "no FCU" : reader.error().c_str());
    return 1;
  }
  auto expected = std::make_unique<FdrFrame>();
  auto got = std::make_unique<FdrFrame>();
  int n = 0;
  while (reader.next(*got)) {
    fill(*expected, n);
    const bool same = got->hasFcu && got->base.simulation_time_s == expected->base.simulation_time_s &&
                      got->fmgc1.fms_inputs.v_managed_kts == expected->fmgc1.fms_inputs.v_managed_kts &&
                      got->fadec[0].outputs.N1_c_percent == expected->fadec[0].outputs.N1_c_percent &&
                      std::memcmp(&got->fcu, &expected->fcu, sizeof(FcuRecord)) == 0;
    if (!same) {
      std::fprintf(stderr, "FAIL: frame %d differs\n", n);
      return 1;
    }
    n++;
  }
  std::printf("%s: %d frames of interface %llu read back identical (FCU record %zu bytes)\n", n == kFrames ? "OK" : "FAIL",
              n, static_cast<unsigned long long>(reader.version()), sizeof(FcuRecord));
  return n == kFrames ? 0 : 1;
}
