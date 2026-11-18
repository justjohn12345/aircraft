// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// Shared part of the native (and wasm) tests of the sim's FCU and FADEC shadow checkers (src/fcu_shadow_test.cpp,
// src/fadec_shadow_test.cpp; fbw_a320/src/shadow/): run results, perturbations, the self-test and the command line.
// Each test provides run(): its generated computer(s) replayed from a recording through the sim's FcuShadow /
// FadecShadow, log in the sim's format.
//
//   <computer>-shadow-test [--log FILE] [--time] [--synthetic] [--power SEED] [--no-shadow] [--stack] file.fdr...
//       Shadow over each recording (fresh computers per file), log in the sim's format to FILE (appended; default
//       stdout), then one RESULT line per file and unit. Exit status 1 if any mismatch.
//       --time: per-frame cost of the generated step, the shadow step (readable step + comparison) and a bare readable
//       step on the same inputs (the difference is the comparison). --synthetic: the drive's synthetic inputs even on
//       a recording with recorded model inputs. --power SEED (FCU only): power / health episodes (computer_running off).
//       --no-shadow: generated model only. --stack: maximum stack depth (wasm builds only).
//   <computer>-shadow-test --self-test [--log-dir DIR] file.fdr
//       1. the committed leaf table equals the lab's leaf list of the output struct (names, offsets, sizes, kinds);
//       2. a clean run reports 0 mismatches; 3. one output bit flipped on one frame (a float, a bool, an enum, an SSM)
//       is reported as exactly one mismatching frame and one episode with the MISMATCH, FIELD, INPUTS, END, SUMMARY and
//       COUNT lines; 4. one readable input perturbed on one frame (FCU: ALT knob +3 turns; FADEC: dt x 1.5) is reported
//       from that frame on, including computed (non input-copy) outputs.

#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "ShadowChecker.h"

namespace shadow_unit_test {

using Clock = std::chrono::steady_clock;

struct Options {
  bool timing = false;
  bool synthetic = false;
  bool noShadow = false;
  long powerSeed = -1;
};

/** What one run does on top of the plain shadow: perturb unit 1's generated outputs or readable inputs on one frame. */
struct Perturbation {
  uint64_t frame = 0;         // shadow frame (1-based), 0 = none
  int flipLeaf = -1;          // flip the lowest bit of this leaf in the generated outputs given to the shadow
  bool perturbInput = false;  // change one readable input (Computer::inputPerturbation)
};

struct UnitResult {
  uint64_t frames = 0, mismatchFrames = 0, episodes = 0, firstFrame = 0;
  std::vector<uint32_t> leafMismatches;
};

struct RunResult {
  bool ok = false;
  std::string error;
  std::vector<UnitResult> units;
  std::vector<bool> running;                                  // unit 1, per shadow frame (index frame - 1)
  double genSeconds = 0, shadowSeconds = 0, bareSeconds = 0;  // all units
  double wallSeconds = 0;
  uint64_t steps = 0;
  uint64_t version = 0;
  bool recordedInputs = false;
};

inline UnitResult unitResult(const shadow::ShadowChecker& c) {
  UnitResult u;
  u.frames = c.frames();
  u.mismatchFrames = c.mismatchFrames();
  u.episodes = c.episodes();
  u.firstFrame = c.firstMismatchFrame();
  u.leafMismatches.resize(c.leafCount());
  for (size_t i = 0; i < c.leafCount(); i++) {
    u.leafMismatches[i] = c.leafMismatches(i);
  }
  return u;
}

inline std::string baseName(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

/** A leaf as the lab's field visitor sees it (fcu_leaves.h), for the leaf table check. */
struct VisitedLeaf {
  std::string name;
  size_t offset;
  size_t size;
  shadow::LeafKind kind;
};

/** The computer under test. */
struct Computer {
  const char* name;  // "FCU" / "FADEC": log header <name>_SHADOW_LOG
  const char* program;
  const shadow::Leaf* leaves;
  size_t leafCount;
  size_t outputBytes;
  int units;
  bool hasPower;
  // the four leaves the self-test flips: a float, a bool, an enum, an SSM
  const char* flips[4];
  // the readable input run() changes on the perturbation frame, and the output leaf that copies it
  const char* inputPerturbation;
  const char* perturbedCopy;
  std::vector<VisitedLeaf> (*visitedLeaves)();
  RunResult (*run)(const std::string& path, const Options& opt, const Perturbation& p, const std::string& logPath, bool keepRunning);
};

inline int failures = 0;

inline void check(bool ok, const std::string& what) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what.c_str());
  failures += !ok;
}

inline std::string readFile(const std::string& path) {
  std::ifstream f(path);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}

inline bool contains(const std::string& text, const std::string& what) {
  return text.find(what) != std::string::npos;
}

inline int findLeaf(const Computer& c, const char* name) {
  for (size_t i = 0; i < c.leafCount; i++) {
    if (std::strcmp(c.leaves[i].name, name) == 0) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

inline bool isInputCopy(const char* name) {
  // leaves that copy the inputs (FCU data.*, FADEC time/data/input/fcu_input)
  for (const char* p : {"data.", "time.", "input.", "fcu_input."}) {
    if (std::strncmp(name, p, std::strlen(p)) == 0) {
      return true;
    }
  }
  return false;
}

inline int selfTest(const Computer& c, const std::string& path, const std::string& logDir) {
  const std::string unitLabel = c.units == 1 ? "" : " unit 1";
  // 1. leaf table
  {
    const std::vector<VisitedLeaf> visited = c.visitedLeaves();
    bool same = visited.size() == c.leafCount;
    for (size_t i = 0; same && i < c.leafCount; i++) {
      const auto& v = visited[i];
      const auto& l = c.leaves[i];
      if (v.name != l.name || v.offset != l.offset || v.size != l.size || v.kind != l.kind) {
        std::printf("  leaf %zu: lab %s @%zu/%zu, committed table %s @%u/%u\n", i, v.name.c_str(), v.offset, v.size, l.name,
                    static_cast<unsigned>(l.offset), static_cast<unsigned>(l.size));
        same = false;
      }
    }
    check(same, std::string("leaf table: ") + std::to_string(c.leafCount) + " leaves (" + std::to_string(c.outputBytes) +
                    " bytes) match the lab's field visitor");
  }

  // 2. clean run; pick the perturbation frame: a running frame 60 % into the recording
  const Options opt;
  RunResult clean = c.run(path, opt, {}, logDir + "/clean.log", true);
  if (!clean.ok) {
    std::printf("FAIL %s: %s\n", path.c_str(), clean.error.c_str());
    return 1;
  }
  uint64_t cleanMismatches = 0;
  for (const UnitResult& u : clean.units) {
    cleanMismatches += u.mismatchFrames;
  }
  const uint64_t frames = clean.units[0].frames;
  check(cleanMismatches == 0, "clean run (" + std::to_string(clean.units.size()) + " unit(s)): " + std::to_string(frames) +
                                  " frames per unit, " + std::to_string(cleanMismatches) + " mismatching");
  const std::string cleanLog = readFile(logDir + "/clean.log");
  check(contains(cleanLog, std::string(c.name) + "_SHADOW_LOG version=1 leaves=" + std::to_string(c.leafCount) + " ") &&
            contains(cleanLog, "SUMMARY unit=1 frames=" + std::to_string(frames) + " ") &&
            (c.units == 1 || contains(cleanLog, "SUMMARY unit=2 frames=" + std::to_string(frames) + " ")) &&
            contains(cleanLog, "mismatch_frames=0 episodes=0") && !contains(cleanLog, "MISMATCH "),
        "clean log: header and summary per unit, no MISMATCH record");

  uint64_t frame = frames * 3 / 5 + 1;
  while (frame < frames && !clean.running[frame - 1]) {
    frame++;
  }
  const std::string f = std::to_string(frame);

  // 3. one output bit flipped on one frame: exactly one mismatching frame and one episode
  for (const char* flip : c.flips) {
    const int leaf = findLeaf(c, flip);
    if (leaf < 0) {
      check(false, std::string("bit flip: no leaf ") + flip);
      continue;
    }
    const std::string name = flip;
    const std::string logPath = logDir + "/flip-" + name + ".log";
    Perturbation p;
    p.frame = frame;
    p.flipLeaf = leaf;
    RunResult r = c.run(path, opt, p, logPath, false);
    const UnitResult& u = r.units[0];
    bool othersClean = true;
    for (size_t k = 1; k < r.units.size(); k++) {
      othersClean = othersClean && r.units[k].mismatchFrames == 0;
    }
    const std::string text = readFile(logPath);
    check(r.ok && u.mismatchFrames == 1 && u.episodes == 1 && u.firstFrame == frame && u.leafMismatches[leaf] == 1 && othersClean,
          "bit flip " + name + unitLabel + " at frame " + f + ": " + std::to_string(u.mismatchFrames) + " mismatching frame(s), " +
              std::to_string(u.episodes) + " episode(s), first at " + std::to_string(u.firstFrame) +
              (c.units > 1 ? (othersClean ? ", unit 2 clean" : ", unit 2 MISMATCHES") : ""));
    check(contains(text, "MISMATCH unit=1 episode=1 frame=" + f + " ") && contains(text, "fields=1 listed=1 fdr=" + baseName(path)) &&
              contains(text, "  FIELD name=" + name + " ") && contains(text, "  INPUTS dt=") &&
              contains(text, "END unit=1 episode=1 frame=" + std::to_string(frame + 1) + " length=1 ") &&
              contains(text, "SUMMARY unit=1 frames=" + std::to_string(frames) + " ") && contains(text, "mismatch_frames=1 episodes=1") &&
              contains(text, "  COUNT unit=1 name=" + name + " frames=1 first_frame=" + f + " "),
          "bit flip " + name + ": log has the MISMATCH record, FIELD, INPUTS, END, SUMMARY and COUNT lines");
  }

  // 4. one readable input perturbed on one frame: reported from that frame, reaches computed outputs
  {
    const std::string logPath = logDir + "/input.log";
    Perturbation p;
    p.frame = frame;
    p.perturbInput = true;
    RunResult r = c.run(path, opt, p, logPath, false);
    const UnitResult& u = r.units[0];
    size_t computed = 0;
    std::string computedNames;
    for (size_t i = 0; i < c.leafCount; i++) {
      if (u.leafMismatches[i] != 0 && !isInputCopy(c.leaves[i].name)) {
        computed++;
        if (computed <= 3) {
          computedNames += std::string(computedNames.empty() ? "" : ", ") + c.leaves[i].name;
        }
      }
    }
    const std::string text = readFile(logPath);
    check(r.ok && u.firstFrame == frame && u.mismatchFrames >= 1 && computed > 0,
          std::string("input ") + c.inputPerturbation + " at frame " + f + ": first mismatch at " + std::to_string(u.firstFrame) + ", " +
              std::to_string(u.mismatchFrames) + " mismatching frames, " + std::to_string(u.episodes) + " episodes, " +
              std::to_string(computed) + " computed output leaves differ (" + computedNames + ")");
    check(contains(text, "MISMATCH unit=1 episode=1 frame=" + f + " ") &&
              contains(text, std::string("  FIELD name=") + c.perturbedCopy + " "),
          std::string("input ") + c.inputPerturbation + ": log has the MISMATCH record with " + c.perturbedCopy);
  }

  std::printf("%s: %d failure(s)\n", failures ? "SELF-TEST FAILED" : "SELF-TEST PASSED", failures);
  return failures ? 1 : 0;
}

inline int main(const Computer& c, int argc, char** argv) {
  Options opt;
  bool self = false, stack = false;
  std::string logPath = "-", logDir = ".";
  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a == "--log" && i + 1 < argc) {
      logPath = argv[++i];
    } else if (a == "--log-dir" && i + 1 < argc) {
      logDir = argv[++i];
    } else if (a == "--time") {
      opt.timing = true;
    } else if (a == "--synthetic") {
      opt.synthetic = true;
    } else if (a == "--power" && i + 1 < argc && c.hasPower) {
      opt.powerSeed = std::atol(argv[++i]);
    } else if (a == "--no-shadow") {
      opt.noShadow = true;
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
  if (files.empty() || (self && files.size() != 1)) {
    std::fprintf(stderr,
                 "usage: %s [--log FILE] [--time] [--synthetic]%s [--no-shadow] [--stack] file.fdr...\n"
                 "       %s --self-test [--log-dir DIR] file.fdr\n",
                 c.program, c.hasPower ? " [--power SEED]" : "", c.program);
    return 2;
  }
  if (self) {
    return selfTest(c, files[0], logDir);
  }
  if (stack) {
    std::fprintf(stderr, "--stack: wasm builds of fmgc-shadow-test only\n");
    return 2;
  }

  int status = 0;
  for (const std::string& path : files) {
    RunResult r = c.run(path, opt, {}, logPath, false);
    if (!r.ok) {
      std::printf("RESULT file=%s error=%s\n", baseName(path).c_str(), r.error.c_str());
      status = 1;
      continue;
    }
    for (size_t k = 0; k < r.units.size(); k++) {
      const UnitResult& u = r.units[k];
      std::printf(
          "RESULT file=%s unit=%zu interface=%llu inputs=%s frames=%llu mismatch_frames=%llu episodes=%llu first_mismatch_frame=%llu",
          baseName(path).c_str(), k + 1, static_cast<unsigned long long>(r.version), r.recordedInputs ? "recorded" : "synthetic",
          static_cast<unsigned long long>(u.frames), static_cast<unsigned long long>(u.mismatchFrames),
          static_cast<unsigned long long>(u.episodes), static_cast<unsigned long long>(u.firstFrame));
      if (opt.timing) {
        // per unit and frame
        const double n = static_cast<double>(r.steps) * static_cast<double>(r.units.size());
        std::printf(" us_generated=%.3f us_shadow=%.3f us_readable_bare=%.3f us_compare=%.3f", r.genSeconds / n * 1e6,
                    r.shadowSeconds / n * 1e6, r.bareSeconds / n * 1e6, (r.shadowSeconds - r.bareSeconds) / n * 1e6);
      }
      std::printf(" steps=%llu wall_s=%.3f\n", static_cast<unsigned long long>(r.steps), r.wallSeconds);
      status |= u.mismatchFrames != 0;
    }
  }
  return status;
}

}  // namespace shadow_unit_test
