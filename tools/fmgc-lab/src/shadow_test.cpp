// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Native driver and test of the sim's FMGC shadow checker (fbw-a32nx/src/wasm/fbw_a320/src/fmgc/FmgcShadow.{h,cpp}),
// the same source the wasm module compiles. The generated FmgcComputer is replayed from flight data recordings
// (src/replay_core.h) and, after every step, FmgcShadow gets exactly what the sim wrapper (fmgc/Fmgc.cpp) gives it: the
// inputs of that step and the generated outputs. The log is written in the sim's format (FmgcShadowLog).
//
//   fmgc-shadow-test [--unit 1|2] [--log FILE] [--time] file.fdr...
//       Shadow over each recording (fresh computers per file), log in the sim's format to FILE (appended; default
//       stdout), then one RESULT line per file. Exit status 1 if any mismatch.
//       --unit 2: FMGC 2 as far as a recording allows (it only holds FMGC 1's inputs): the same inputs with
//       discrete_inputs.is_unit_1 false, so the unit-2 paths run in both computers.
//       --time: per-frame cost of the generated step, the shadow step (readable step + comparison) and a bare readable
//       step on the same inputs (the difference is the comparison).
//       --power SEED: FMGC power / health episodes (computer_running off for a frame to a minute, flicker, opposite
//       FMGC lost; src/scenario_power.h) on top of the recording, so the restart paths run in both computers.
//       --no-shadow: generated model only (reference for --time and --stack).
//       --stack (wasm build only, scripts/build-shadow-wasm.sh): maximum stack depth over the runs.
//   fmgc-shadow-test --self-test [--log-dir DIR] file.fdr
//       1. the committed leaf table (FmgcOutputLeaves.gen.h) equals the leaf list of fields.gen.h (names, offsets,
//          sizes, kinds); 2. clean runs of both units report 0 mismatches; 3. one output bit flipped on one frame (a
//          float, a bool, an enum, an SSM) is reported as exactly one mismatching frame and one episode, with the field,
//          both values and the episode end in the log; 4. one readable input perturbed on one frame (dt x 1.5) is
//          reported from that frame on, including computed (non input-copy) outputs.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "FmgcComputer.h"
#include "FmgcShadow.h"
#include "fields.gen.h"
#include "readable/fmgc.h"
#include "replay_core.h"
#include "scenario_power.h"

using Clock = std::chrono::steady_clock;
using fmgc_shadow::kOutputLeafCount;
using fmgc_shadow::kOutputLeaves;
using fmgc_shadow::LeafKind;

namespace {

bool noShadow = false;  // --no-shadow: generated model only (stack and time reference)
long powerSeed = -1;    // --power SEED: FMGC power / health episodes (src/scenario_power.h) in both computers' inputs

#if defined(__wasm__)
// --stack (wasm only): maximum shadow-stack depth. wasm-ld places the stack between the data (__data_end) and the heap
// (__heap_base), growing down; it is painted before the runs and scanned after them.
extern "C" char __data_end;
extern "C" char __heap_base;
constexpr uint8_t kStackPaint = 0xa5;

uintptr_t paintStack() {
  volatile char probe = 0;
  const uintptr_t sp = reinterpret_cast<uintptr_t>(&probe);
  const uintptr_t low = (reinterpret_cast<uintptr_t>(&__data_end) + 15) & ~uintptr_t{15};
  std::memset(reinterpret_cast<void*>(low), kStackPaint, sp - 1024 - low);
  return low;
}

size_t stackUsed(uintptr_t low) {
  const auto* p = reinterpret_cast<const uint8_t*>(low);
  const uintptr_t high = reinterpret_cast<uintptr_t>(&__heap_base);
  while (reinterpret_cast<uintptr_t>(p) < high && *p == kStackPaint) {
    p++;
  }
  return high - reinterpret_cast<uintptr_t>(p);
}
#endif

std::string baseName(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

/** What one run does on top of the plain shadow: perturb the generated outputs or the readable inputs on one frame. */
struct Perturbation {
  uint64_t frame = 0;  // shadow frame (1-based), 0 = none
  int flipLeaf = -1;   // flip the lowest bit of this leaf in the generated outputs given to the shadow
  bool scaleDt = false;  // readable input: time.dt x 1.5
};

struct RunResult {
  bool ok = false;
  std::string error;
  uint64_t frames = 0, runningFrames = 0, mismatchFrames = 0, episodes = 0, firstFrame = 0;
  std::vector<uint32_t> leafMismatches;
  std::vector<bool> running;  // per shadow frame (index frame - 1)
  double genSeconds = 0, shadowSeconds = 0, bareSeconds = 0;
  double wallSeconds = 0;
  uint64_t steps = 0;
};

RunResult run(const std::string& path, int unit, const Perturbation& p, const std::string& logPath, bool timing,
              bool keepRunning = false) {
  RunResult res;
  const auto start = Clock::now();
  auto generated = std::make_unique<FmgcComputer>();
  generated->initialize();
  auto shadow = std::make_unique<FmgcShadow>(unit);
  FmgcShadowLog log("");
  log.setPath(logPath);
  std::unique_ptr<fmgc::readable::FmgcComputer> bare;
  if (timing) {
    bare = std::make_unique<fmgc::readable::FmgcComputer>();
    bare->initialize();
  }
  Replayer replayer(path, {}, *generated);
  if (!replayer.ok()) {
    res.error = replayer.error();
    return res;
  }
  auto outCopy = std::make_unique<fmgc_outputs>();
  auto inCopy = std::make_unique<fmgc_inputs>();
  Clock::time_point t0;
  FmgcShadow* units[1] = {shadow.get()};

  std::unique_ptr<PowerScenario> power;
  if (powerSeed >= 0) {
    power = std::make_unique<PowerScenario>(static_cast<uint64_t>(powerSeed));
  }
  replayer.beforeStep = [&]() {
    if (unit == 2) {
      replayer.mutableInputs().discrete_inputs.is_unit_1 = false;
    }
    if (power) {
      power->apply(replayer.mutableInputs(), replayer.inputs().time.dt);
    }
    if (timing) {
      t0 = Clock::now();
    }
  };
  replayer.afterStep = [&]() {
    if (timing) {
      res.genSeconds += std::chrono::duration<double>(Clock::now() - t0).count();
    }
    if (noShadow) {
      return;
    }
    const fmgc_inputs& in = replayer.inputs();
    const fmgc_outputs& out = generated->getExternalOutputs().out;
    const uint64_t frame = shadow->frames() + 1;
    if (keepRunning) {
      res.running.push_back(in.sim_data.computer_running);
    }
    if (frame == p.frame && (p.flipLeaf >= 0 || p.scaleDt)) {
      *outCopy = out;
      *inCopy = in;
      if (p.flipLeaf >= 0) {
        reinterpret_cast<uint8_t*>(outCopy.get())[kOutputLeaves[p.flipLeaf].offset] ^= 1;
      }
      if (p.scaleDt) {
        inCopy->time.dt *= 1.5;
      }
      shadow->step(*inCopy, *outCopy);
      return;
    }
    if (timing) {
      auto t1 = Clock::now();
      shadow->step(in, out);
      auto t2 = Clock::now();
      bare->step(in);
      res.shadowSeconds += std::chrono::duration<double>(t2 - t1).count();
      res.bareSeconds += std::chrono::duration<double>(Clock::now() - t2).count();
    } else {
      shadow->step(in, out);
    }
  };

  const std::string fdrName = baseName(path);
  Replayer::Frame fr;
  while (replayer.next(fr)) {
    // once per recorded frame, as the sim does after its flight data recorder update (gap-fill steps are reported
    // with the next recorded frame)
    log.update(units, 1, fdrName, fr.index);
  }
  log.writeSummary(units, 1);
  res.wallSeconds = std::chrono::duration<double>(Clock::now() - start).count();
  res.steps = static_cast<uint64_t>(replayer.frames() + replayer.gapSteps());
  res.ok = true;
  res.frames = shadow->frames();
  res.mismatchFrames = shadow->mismatchFrames();
  res.episodes = shadow->episodes();
  res.firstFrame = shadow->firstMismatchFrame();
  res.leafMismatches.resize(kOutputLeafCount);
  for (size_t i = 0; i < kOutputLeafCount; i++) {
    res.leafMismatches[i] = shadow->leafMismatches(i);
  }
  return res;
}

// ---- self-test -----------------------------------------------------------------------------------------------------------

int failures = 0;

void check(bool ok, const std::string& what) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what.c_str());
  failures += !ok;
}

std::string readFile(const std::string& path) {
  std::ifstream f(path);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

bool contains(const std::string& text, const std::string& what) {
  return text.find(what) != std::string::npos;
}

struct VisitedLeaf {
  std::string name;
  size_t offset;
  size_t size;
  FieldKind kind;
};

struct LeafCollector {
  std::vector<VisitedLeaf>& leaves;
  const void* base;
  template <typename T>
  void field(const char* prefix, const char* name, T& a, T& /*b*/, FieldKind kind) {
    leaves.push_back({std::string(prefix) + name,
                      static_cast<size_t>(reinterpret_cast<const char*>(&a) - static_cast<const char*>(base)), sizeof(T), kind});
  }
};

void checkLeafTable() {
  std::vector<VisitedLeaf> visited;
  auto a = std::make_unique<fmgc_outputs>();
  auto b = std::make_unique<fmgc_outputs>();
  LeafCollector c{visited, a.get()};
  visit(c, "", *a, *b);
  bool same = visited.size() == kOutputLeafCount;
  for (size_t i = 0; same && i < kOutputLeafCount; i++) {
    const auto& v = visited[i];
    const auto& l = kOutputLeaves[i];
    const LeafKind kind = v.kind == FieldKind::Bool    ? LeafKind::Bool
                          : v.kind == FieldKind::Float ? LeafKind::Float
                          : v.kind == FieldKind::Enum  ? LeafKind::Enum
                                                       : LeafKind::Int;
    if (v.name != l.name || v.offset != l.offset || v.size != l.size || kind != l.kind) {
      std::printf("  leaf %zu: fields.gen.h %s @%zu/%zu, FmgcOutputLeaves.gen.h %s @%u/%u\n", i, v.name.c_str(), v.offset, v.size,
                  l.name, static_cast<unsigned>(l.offset), static_cast<unsigned>(l.size));
      same = false;
    }
  }
  check(same, "leaf table: " + std::to_string(kOutputLeafCount) + " leaves of fmgc_outputs (" +
                  std::to_string(sizeof(fmgc_outputs)) + " bytes) match fields.gen.h");
}

int findLeaf(const char* name, LeafKind kind) {
  for (size_t i = 0; i < kOutputLeafCount; i++) {
    if (std::strcmp(kOutputLeaves[i].name, name) == 0) {
      return static_cast<int>(i);
    }
  }
  for (size_t i = 0; i < kOutputLeafCount; i++) {  // fallback: first computed leaf of that kind
    if (kOutputLeaves[i].kind == kind && std::strncmp(kOutputLeaves[i].name, "data.", 5) != 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int selfTest(const std::string& path, const std::string& logDir) {
  std::printf("INFO sizeof FmgcShadow %zu bytes (+ readable FmgcComputer %zu bytes, heap, allocated once); generated FmgcComputer %zu bytes\n",
              sizeof(FmgcShadow), sizeof(fmgc::readable::FmgcComputer), sizeof(FmgcComputer));
  checkLeafTable();

  // clean runs; pick the perturbation frame: a running frame 60 % into the recording
  RunResult clean = run(path, 1, {}, logDir + "/clean-unit1.log", false, true);
  if (!clean.ok) {
    std::printf("FAIL %s: %s\n", path.c_str(), clean.error.c_str());
    return 1;
  }
  check(clean.mismatchFrames == 0, "clean run unit 1: " + std::to_string(clean.frames) + " frames, " +
                                       std::to_string(clean.mismatchFrames) + " mismatching");
  RunResult clean2 = run(path, 2, {}, logDir + "/clean-unit2.log", false);
  check(clean2.ok && clean2.mismatchFrames == 0, "clean run unit 2: " + std::to_string(clean2.frames) + " frames, " +
                                                     std::to_string(clean2.mismatchFrames) + " mismatching");
  const std::string cleanLog = readFile(logDir + "/clean-unit1.log");
  check(contains(cleanLog, "FMGC_SHADOW_LOG version=1 leaves=1017") &&
            contains(cleanLog, "SUMMARY unit=1 frames=" + std::to_string(clean.frames) + " ") &&
            contains(cleanLog, "mismatch_frames=0 episodes=0") && !contains(cleanLog, "MISMATCH "),
        "clean log: header and summary, no MISMATCH record");

  uint64_t frame = clean.frames * 3 / 5 + 1;
  while (frame < clean.frames && !clean.running[frame - 1]) {
    frame++;
  }

  // one output bit flipped on one frame: exactly one mismatching frame and one episode
  const struct {
    const char* name;
    LeafKind kind;
  } flips[] = {{"bus_outputs.fmgc_a_bus.pitch_fd_command.Data", LeafKind::Float},
               {"discrete_outputs.ap_own_engaged", LeafKind::Bool},
               {"ap_fd_logic.active_lateral_law", LeafKind::Enum},
               {"bus_outputs.fmgc_a_bus.discrete_word_1.SSM", LeafKind::Int}};
  for (const auto& flip : flips) {
    const int leaf = findLeaf(flip.name, flip.kind);
    const std::string name = kOutputLeaves[leaf].name;
    const std::string logPath = logDir + "/flip-" + name + ".log";
    Perturbation p;
    p.frame = frame;
    p.flipLeaf = leaf;
    RunResult r = run(path, 1, p, logPath, false);
    const std::string text = readFile(logPath);
    const std::string f = std::to_string(frame);
    check(r.ok && r.mismatchFrames == 1 && r.episodes == 1 && r.firstFrame == frame && r.leafMismatches[leaf] == 1,
          "bit flip " + name + " at frame " + f + ": " + std::to_string(r.mismatchFrames) + " mismatching frame(s), " +
              std::to_string(r.episodes) + " episode(s), first at " + std::to_string(r.firstFrame));
    check(contains(text, "MISMATCH unit=1 episode=1 frame=" + f + " ") && contains(text, "fields=1 listed=1 fdr=" + baseName(path)) &&
              contains(text, "  FIELD name=" + name + " ") && contains(text, "  INPUTS dt=") &&
              contains(text, "END unit=1 episode=1 frame=" + std::to_string(frame + 1) + " length=1 ") &&
              contains(text, "SUMMARY unit=1 frames=" + std::to_string(clean.frames) + " ") &&
              contains(text, "mismatch_frames=1 episodes=1") &&
              contains(text, "  COUNT unit=1 name=" + name + " frames=1 first_frame=" + f + " "),
          "bit flip " + name + ": log has the MISMATCH record, FIELD, INPUTS, END, SUMMARY and COUNT lines");
  }

  // one readable input perturbed on one frame: reported from that frame, reaches computed outputs
  {
    const std::string logPath = logDir + "/input-dt.log";
    Perturbation p;
    p.frame = frame;
    p.scaleDt = true;
    RunResult r = run(path, 1, p, logPath, false);
    size_t computed = 0;
    for (size_t i = 0; i < kOutputLeafCount; i++) {
      computed += r.leafMismatches[i] != 0 && std::strncmp(kOutputLeaves[i].name, "data.", 5) != 0;
    }
    const std::string text = readFile(logPath);
    check(r.ok && r.firstFrame == frame && r.mismatchFrames >= 1 && computed > 0,
          "input dt x 1.5 at frame " + std::to_string(frame) + ": first mismatch at " + std::to_string(r.firstFrame) + ", " +
              std::to_string(r.mismatchFrames) + " mismatching frames, " + std::to_string(r.episodes) + " episodes, " +
              std::to_string(computed) + " computed output leaves differ");
    check(contains(text, "MISMATCH unit=1 episode=1 frame=" + std::to_string(frame) + " ") &&
              contains(text, "  FIELD name=data.time.dt kind=float class=value"),
          "input dt x 1.5: log has the MISMATCH record with data.time.dt");
  }

  std::printf("%s: %d failure(s)\n", failures ? "SELF-TEST FAILED" : "SELF-TEST PASSED", failures);
  return failures ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  int unit = 1;
  bool timing = false, self = false, stack = false;
  std::string logPath = "-", logDir = ".";
  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--unit" && i + 1 < argc) {
      unit = std::atoi(argv[++i]);
    } else if (a == "--log" && i + 1 < argc) {
      logPath = argv[++i];
    } else if (a == "--log-dir" && i + 1 < argc) {
      logDir = argv[++i];
    } else if (a == "--time") {
      timing = true;
    } else if (a == "--power" && i + 1 < argc) {
      powerSeed = std::atol(argv[++i]);
    } else if (a == "--no-shadow") {
      noShadow = true;
    } else if (a == "--stack") {
      stack = true;
    } else if (a == "--self-test") {
      self = true;
    } else if (!a.empty() && a[0] != '-') {
      files.push_back(a);
    } else {
      files.clear();
      break;
    }
  }
  if (files.empty() || (unit != 1 && unit != 2) || (self && files.size() != 1)) {
    std::fprintf(stderr,
                 "usage: fmgc-shadow-test [--unit 1|2] [--log FILE] [--time] [--power SEED] [--no-shadow] [--stack] file.fdr...\n"
                 "       fmgc-shadow-test --self-test [--log-dir DIR] file.fdr\n");
    return 2;
  }
  if (self) {
    return selfTest(files[0], logDir);
  }
#if defined(__wasm__)
  const uintptr_t stackLow = stack ? paintStack() : 0;
#else
  if (stack) {
    std::fprintf(stderr, "--stack: wasm builds only\n");
    return 2;
  }
#endif

  int status = 0;
  for (const std::string& path : files) {
    RunResult r = run(path, unit, {}, logPath, timing);
    if (!r.ok) {
      std::printf("RESULT file=%s unit=%d error=%s\n", baseName(path).c_str(), unit, r.error.c_str());
      status = 1;
      continue;
    }
    std::printf("RESULT file=%s unit=%d frames=%llu mismatch_frames=%llu episodes=%llu first_mismatch_frame=%llu", baseName(path).c_str(),
                unit, static_cast<unsigned long long>(r.frames), static_cast<unsigned long long>(r.mismatchFrames),
                static_cast<unsigned long long>(r.episodes), static_cast<unsigned long long>(r.firstFrame));
    if (timing) {
      const double n = static_cast<double>(r.steps);
      std::printf(" us_generated=%.3f us_shadow=%.3f us_readable_bare=%.3f us_compare=%.3f", r.genSeconds / n * 1e6,
                  r.shadowSeconds / n * 1e6, r.bareSeconds / n * 1e6, (r.shadowSeconds - r.bareSeconds) / n * 1e6);
    }
    std::printf(" steps=%llu wall_s=%.3f\n", static_cast<unsigned long long>(r.steps), r.wallSeconds);
    status |= r.mismatchFrames != 0;
  }
#if defined(__wasm__)
  if (stack) {
    std::printf("STACK max_depth_bytes=%zu stack_bytes=%zu%s\n", stackUsed(stackLow),
                static_cast<size_t>(&__heap_base - &__data_end), noShadow ? " (generated only)" : " (generated + shadow)");
  }
#endif
  return status;
}
