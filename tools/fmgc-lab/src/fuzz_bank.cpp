// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Builds the fuzzer's snapshot bank (src/fuzz_bank.h) from flight data recordings: replays each recording through the
// generated model (and through the stateful oracles, e.g. the whole-computer differential once it exists), and every
// --interval seconds of recording time stores the model state before the step plus the next --window seconds of step
// inputs, decimated by --decimation. A snapshot at the first frame of every recording (fresh model, power-up) is always
// included.
//
// usage: fmgc-fuzz-bank [--interval S] [--window S] [--decimation N] --out FILE file.fdr...

#include "fuzz_oracle.h"

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "fuzz_bank.h"
#include "replay_core.h"

using namespace fmgc::fuzz;

namespace {

struct OpenWindow {
  BankSnapshot snap;
  std::vector<uint8_t> state;  // FmgcComputer bytes (the class is not copyable, but has no pointers)
  std::vector<uint8_t> oracle;
  std::vector<fmgc_inputs> frames;
  fmgc_inputs pending{};
  uint32_t pendingCount = 0;
  double pendingDt = 0;
  double endTime = 0;
};

/** A copy of the model made from the snapshot bytes, stepped alongside the replay for 2000 steps (restore check). */
struct Shadow {
  std::unique_ptr<FmgcComputer> model;
  long remaining = 2000;
};

void writeAt(FILE* f, uint64_t& pos, const void* data, size_t size) {
  std::fwrite(data, 1, size, f);
  pos += size;
}

void align(FILE* f, uint64_t& pos) {
  static const char zeros[64] = {};
  size_t pad = (64 - pos % 64) % 64;
  writeAt(f, pos, zeros, pad);
}

}  // namespace

int main(int argc, char** argv) {
  double interval = 120, window = 30;
  uint32_t decimation = 2;
  std::string outPath;
  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--interval" && i + 1 < argc) {
      interval = std::atof(argv[++i]);
    } else if (a == "--window" && i + 1 < argc) {
      window = std::atof(argv[++i]);
    } else if (a == "--decimation" && i + 1 < argc) {
      decimation = static_cast<uint32_t>(std::atoi(argv[++i]));
    } else if (a == "--out" && i + 1 < argc) {
      outPath = argv[++i];
    } else {
      files.push_back(a);
    }
  }
  if (outPath.empty() || files.empty() || decimation < 1) {
    std::fprintf(stderr, "usage: fmgc-fuzz-bank [--interval S] [--window S] [--decimation N] --out FILE file.fdr...\n");
    return 2;
  }

  OracleSet oracles({false, false, false, WholeComputerOracle::enabled()});
  FILE* f = std::fopen(outPath.c_str(), "wb");
  if (!f) {
    std::fprintf(stderr, "cannot write %s\n", outPath.c_str());
    return 2;
  }
  BankHeader header{};
  std::memcpy(header.magic, kBankMagic, 8);
  header.inputsSize = sizeof(fmgc_inputs);
  header.computerSize = sizeof(FmgcComputer);
  header.decimation = decimation;
  header.intervalS = interval;
  header.windowS = window;
  uint64_t pos = 0;
  writeAt(f, pos, &header, sizeof header);

  std::vector<BankSnapshot> table;
  std::vector<std::string> names;
  uint32_t oracleSize = 0;
  bool oracleSizeKnown = false;
  long totalFrames = 0;
  long verifiedSteps = 0, restoreMismatches = 0;

  auto flush = [&](OpenWindow& w) {
    if (w.pendingCount > 0) {
      w.pending.time.dt = w.pendingDt;
      w.frames.push_back(w.pending);
      w.pendingCount = 0;
      w.pendingDt = 0;
    }
    if (w.frames.size() < 16) {
      return;  // too short to be useful (end of recording)
    }
    align(f, pos);
    w.snap.stateOffset = pos;
    writeAt(f, pos, w.state.data(), w.state.size());
    align(f, pos);
    w.snap.oracleOffset = pos;
    if (!oracleSizeKnown) {
      oracleSize = static_cast<uint32_t>(w.oracle.size());
      oracleSizeKnown = true;
    }
    writeAt(f, pos, w.oracle.data(), w.oracle.size());
    align(f, pos);
    w.snap.framesOffset = pos;
    w.snap.frames = static_cast<uint32_t>(w.frames.size());
    writeAt(f, pos, w.frames.data(), w.frames.size() * sizeof(fmgc_inputs));
    totalFrames += static_cast<long>(w.frames.size());
    table.push_back(w.snap);
  };

  for (const std::string& path : files) {
    auto computer = std::make_unique<FmgcComputer>();
    computer->initialize();
    Replayer replayer(path, {}, *computer);
    if (!replayer.ok()) {
      std::fprintf(stderr, "skip %s: %s\n", path.c_str(), replayer.error().c_str());
      continue;
    }
    uint32_t rec = static_cast<uint32_t>(names.size());
    std::string base = path.substr(path.find_last_of('/') + 1);
    names.push_back(base);
    oracles.warmBegin();
    std::vector<std::unique_ptr<OpenWindow>> open;
    std::vector<std::unique_ptr<Shadow>> shadows;
    int64_t step = 0;
    double lastSnap = -1e18;
    long snapsHere = 0;

    replayer.beforeStep = [&]() {
      const fmgc_inputs& in = replayer.inputs();
      double t = in.time.simulation_time;
      if (step == 0 || t - lastSnap >= interval) {
        lastSnap = t;
        auto w = std::make_unique<OpenWindow>();
        w->snap = BankSnapshot{rec, 0, step, t, 0, 0, 0};
        w->state.assign(reinterpret_cast<const uint8_t*>(computer.get()), reinterpret_cast<const uint8_t*>(computer.get()) + sizeof(FmgcComputer));
        oracles.saveState(w->oracle);
        w->endTime = t + window;
        // restore check: a fresh object given these bytes must step exactly like the replayed one
        auto sh = std::make_unique<Shadow>();
        sh->model = std::make_unique<FmgcComputer>();
        std::memcpy(static_cast<void*>(sh->model.get()), w->state.data(), sizeof(FmgcComputer));
        shadows.push_back(std::move(sh));
        open.push_back(std::move(w));
        snapsHere++;
      }
      for (auto& w : open) {
        if (w->pendingCount == 0) {
          w->pendingDt = 0;
        }
        w->pending = in;
        w->pendingDt += in.time.dt;
        if (++w->pendingCount == decimation) {
          w->pending.time.dt = w->pendingDt;
          w->frames.push_back(w->pending);
          w->pendingCount = 0;
        }
      }
      // close finished windows
      for (size_t i = 0; i < open.size();) {
        if (t >= open[i]->endTime) {
          flush(*open[i]);
          open.erase(open.begin() + static_cast<long>(i));
        } else {
          i++;
        }
      }
    };
    replayer.afterStep = [&]() {
      oracles.warmStep(replayer.inputs(), computer->getExternalOutputs().out);
      for (size_t i = 0; i < shadows.size();) {
        Shadow& s = *shadows[i];
        s.model->FmgcComputer_U = computer->FmgcComputer_U;  // this step's inputs
        s.model->step();
        verifiedSteps++;
        // leaf by leaf (struct padding is copied from stack temporaries and differs between objects); hidden state
        // that the bytes failed to capture shows up in the outputs within the 2000 steps
        if (FieldTable<fmgc_outputs>::get().firstDifference(s.model->FmgcComputer_Y.out, computer->FmgcComputer_Y.out) >=
            0) {
          restoreMismatches++;
          s.remaining = 0;
        }
        if (--s.remaining <= 0) {
          shadows.erase(shadows.begin() + static_cast<long>(i));
        } else {
          i++;
        }
      }
      step++;
    };
    Replayer::Frame fr;
    while (replayer.next(fr)) {
    }
    for (auto& w : open) {
      flush(*w);
    }
    std::fprintf(stderr, "%s: %ld steps, %ld snapshots\n", base.c_str(), static_cast<long>(step), snapsHere);
  }

  align(f, pos);
  header.tableOffset = pos;
  writeAt(f, pos, table.data(), table.size() * sizeof(BankSnapshot));
  header.namesOffset = pos;
  for (const std::string& n : names) {
    char buf[128] = {};
    std::snprintf(buf, sizeof buf, "%s", n.c_str());
    writeAt(f, pos, buf, sizeof buf);
  }
  header.snapshots = static_cast<uint32_t>(table.size());
  header.recordings = static_cast<uint32_t>(names.size());
  header.oracleSize = oracleSize;
  std::fprintf(stderr, "restore check: %ld steps of models restored from snapshot bytes, %ld snapshots diverged%s\n",
               verifiedSteps, restoreMismatches, restoreMismatches ? " (SNAPSHOTS ARE NOT COMPLETE)" : "");
  std::fseek(f, 0, SEEK_SET);
  std::fwrite(&header, sizeof header, 1, f);
  std::fclose(f);
  std::fprintf(stderr, "bank %s: %u snapshots, %ld frames, %.1f MB, oracle state %u bytes\n", outPath.c_str(),
               header.snapshots, totalFrames, static_cast<double>(pos) / 1e6, oracleSize);
  return 0;
}
