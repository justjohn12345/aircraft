// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// FMGC shadow mode (personal build, ini [FMGC] SHADOW_ENABLED): the readable FMGC (tools/fmgc-lab/readable, a
// hand-written rewrite of the generated FmgcComputer) runs alongside the generated one on exactly the same inputs every
// frame, and every output leaf is compared by bit pattern. The generated outputs stay the only ones the aircraft uses;
// the readable outputs are only compared. Design, log format and offline reproduction: tools/fmgc-lab/docs/shadow.md.
//
// Plain C++ (no MSFS SDK), so the lab builds and tests it natively (tools/fmgc-lab/src/shadow_test.cpp).
//
// After a mismatch both computers keep running (free-running, no re-sync); see shadow.md for the trade-off. Mismatches
// are grouped in episodes (consecutive mismatching frames); the first kMaxEvents episodes keep their first frame's
// details for the log, every output leaf keeps a mismatch count. Memory is fixed at construction; step() allocates
// nothing and does no I/O.

#pragma once

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "FmgcComputer_types.h"
#include "FmgcOutputLeaves.gen.h"

namespace fmgc::readable {
class FmgcComputer;
}

class FmgcShadow {
 public:
  /** Mismatch episodes whose details are kept (later episodes are only counted). */
  static constexpr int kMaxEvents = 16;
  /** Differing fields listed per episode (the total number is always kept). */
  static constexpr int kMaxEventFields = 24;

  /** unit: 1 or 2 (FMGC 1 / FMGC 2), for the log only. Construct together with the generated model, before its first step. */
  explicit FmgcShadow(int unit);
  ~FmgcShadow();
  FmgcShadow(const FmgcShadow&) = delete;
  FmgcShadow& operator=(const FmgcShadow&) = delete;

  /**
   * One frame: steps the readable computer with `in` (the inputs the generated model just stepped with, computer_running
   * included) and compares its outputs with `generated` (the generated model's outputs after that step).
   */
  void step(const fmgc_inputs& in, const fmgc_outputs& generated);

  int unit() const { return unit_; }
  /** Frames stepped (= generated steps since construction). */
  uint64_t frames() const { return frames_; }
  /** Frames where at least one output leaf differed. */
  uint64_t mismatchFrames() const { return mismatchFrames_; }
  uint64_t episodes() const { return episodes_; }
  /** Frame (1-based, see frames()) and sim time of the first mismatch; 0 if none. */
  uint64_t firstMismatchFrame() const { return firstMismatchFrame_; }
  double firstMismatchSimTime() const { return firstMismatchSimTime_; }
  /** Number of differing leaves on the first mismatching frame, and the first of them. */
  uint32_t firstMismatchFieldCount() const { return firstMismatchFieldCount_; }
  const char* firstMismatchFieldName() const;
  /** Mismatching frames of one output leaf (index in fmgc_shadow::kOutputLeaves). */
  uint32_t leafMismatches(size_t leaf) const { return leafMismatches_[leaf]; }

  /**
   * Writes the log records not written yet (episode starts with their fields and inputs, episode ends) to `f`; `context`
   * (e.g. "fdr=... fdr_record=...") is appended to each record. Returns the number of records written. Call it every
   * frame after step() so the context is the one of the frame the records belong to.
   */
  int writeNewEvents(std::FILE* f, const char* context);
  /** True when writeNewEvents() has something to write. */
  bool hasNewEvents() const { return pendingStarts_ > 0 || pendingEnds_ > 0; }
  /** The end-of-session summary: totals and the mismatch count of every leaf that ever differed. */
  void writeSummary(std::FILE* f) const;

  /** The readable computer (tests and diagnostics). */
  const fmgc::readable::FmgcComputer& readable() const { return *readable_; }
  /**
   * The readable computer's outputs of the last step() (zero before the first): what the aircraft uses with [FMGC]
   * READABLE_PRIMARY. Out of line, so Fmgc.cpp needs no readable include.
   */
  const fmgc_outputs& readableOutputs() const;

 private:
  struct FieldValue {
    uint16_t leaf;
    uint8_t generated[8];
    uint8_t readable[8];
  };

  struct Event {
    uint64_t episode;  // 1-based
    uint64_t frame;
    uint64_t endFrame;  // first matching frame after the episode, 0 while it lasts
    base_time time;
    base_sim_data simData;
    uint32_t fieldCount;
    uint32_t listed;
    FieldValue fields[kMaxEventFields];
    bool startWritten;
    bool endWritten;
  };

  bool differs(const fmgc_outputs& a, const fmgc_outputs& b) const;
  void recordMismatch(const fmgc_inputs& in, const fmgc_outputs& generated, const fmgc_outputs& readable);

  static constexpr size_t kOutputWords = (sizeof(fmgc_outputs) + 7) / 8;

  const int unit_;
  std::unique_ptr<fmgc::readable::FmgcComputer> readable_;
  uint64_t mask_[kOutputWords];  // bytes that belong to a leaf (padding excluded)

  uint64_t frames_ = 0;
  uint64_t runningFrames_ = 0;
  uint64_t restarts_ = 0;
  bool wasRunning_ = false;
  // frames with sim_data inputs the flight data recorder does not record (offline replays assume false)
  uint64_t slewFrames_ = 0;
  uint64_t pauseFrames_ = 0;
  uint64_t trackingOverrideFrames_ = 0;
  uint64_t tailstrikeProtectionFrames_ = 0;

  uint64_t mismatchFrames_ = 0;
  uint64_t episodes_ = 0;
  uint64_t nanOnlyFrames_ = 0;  // every differing leaf: NaN in both, different payload
  uint64_t signedZeroLeafFrames_ = 0;  // leaf-frames +0 vs -0
  uint64_t firstMismatchFrame_ = 0;
  double firstMismatchSimTime_ = 0;
  uint32_t firstMismatchFieldCount_ = 0;
  int firstMismatchLeaf_ = -1;
  bool inEpisode_ = false;
  uint64_t episodeStart_ = 0;
  uint64_t longestEpisode_ = 0;
  int currentEvent_ = -1;  // index in events_ of the running episode, -1 if not kept

  int eventCount_ = 0;
  int pendingStarts_ = 0;
  int pendingEnds_ = 0;
  Event events_[kMaxEvents];

  uint32_t leafMismatches_[fmgc_shadow::kOutputLeafCount];
  uint64_t leafFirstFrame_[fmgc_shadow::kOutputLeafCount];
  double leafFirstSimTime_[fmgc_shadow::kOutputLeafCount];
};

/**
 * The shadow log file: one per session, named after the session's first flight data recorder file
 * (<fdr name>.fmgc-shadow.log, e.g. 2026-11-15-10-00-00.fmgc-shadow.log), or after the UTC time when no recording is
 * running. Opened in append mode only when there is something to write (episodes, the summary).
 */
class FmgcShadowLog {
 public:
  /** directory: prefix of the log path, e.g. "\\work\\" in the sim. */
  explicit FmgcShadowLog(std::string directory) : directory_(std::move(directory)) {}

  /**
   * Once per frame, after the flight data recorder update: writes the new records of each unit (rare) and prints a
   * console line at each unit's first mismatch. fdrFile: the recorder's current file (any directory prefix is dropped;
   * empty when not recording); fdrRecord: index of this frame's record in it, -1 if this frame was not recorded.
   */
  void update(FmgcShadow* const units[], int count, const std::string& fdrFile, long fdrRecord);

  /** End of session: writes the summary of each unit to the log and prints one console line. */
  void writeSummary(FmgcShadow* const units[], int count);

  /** The log path (empty until known). */
  const std::string& path() const { return path_; }
  /** Uses this path instead of the session name, "-" for stdout (native tests). */
  void setPath(std::string path) { path_ = std::move(path); }
  /** The readable FMGC is the primary ([FMGC] READABLE_PRIMARY): the header says primary=readable. */
  void setReadablePrimary(bool readablePrimary) { readablePrimary_ = readablePrimary; }

 private:
  std::FILE* open();

  std::string directory_;
  std::string path_;
  bool headerWritten_ = false;
  bool readablePrimary_ = false;
  bool firstReported_[2] = {false, false};
  std::string sessionFdr_;
};
