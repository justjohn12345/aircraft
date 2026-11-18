// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

// The comparison core of the FCU, FADEC and FAC shadow modes (personal build; FcuShadow.h, FadecShadow.h, FacShadow.h):
// one output struct of a generated computer against the same struct of its readable rewrite (tools/fmgc-lab/readable/
// fcu, readable/fadec, readable/fac), every leaf compared by bit pattern, every frame (optionally with two NaNs equal
// whatever their sign and payload: the FAC). It is the FMGC shadow checker's design (fmgc/FmgcShadow.{h,cpp},
// which keeps its own copy of this logic) made independent of the output type: the leaf table and the output size are
// constructor arguments. Design, log format and offline reproduction: tools/fmgc-lab/docs/shadow.md.
//
// Plain C++ (no MSFS SDK), so the lab builds and tests it natively (tools/fmgc-lab/src/shadow_fcu_fadec_test.cpp).
//
// After a mismatch both computers keep running (free-running, no re-sync). Mismatches are grouped in episodes
// (consecutive mismatching frames); the first kMaxEvents episodes keep their first frame's details for the log, every
// output leaf keeps a mismatch count. Memory is allocated once at construction; step() allocates nothing and does no
// I/O.

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

namespace shadow {

enum class LeafKind : uint8_t { Bool, Float, Int, Enum };

/** One scalar leaf of an output struct (generated tables: FcuOutputLeaves.gen.h, FadecOutputLeaves.gen.h). */
struct Leaf {
  const char* name;
  uint32_t offset;
  uint8_t size;
  LeafKind kind;
};

/** The inputs of a frame a mismatch record keeps: time, and the sim flags a flight data recording does not hold. */
struct FrameInputs {
  double dt = 0;
  double simulationTime = 0;
  double monotonicTime = 0;
  // base_sim_data (FCU; the FADEC has none: the checker is then constructed with hasSimData false and these are unused)
  bool slewOn = false;
  bool pauseOn = false;
  bool trackingModeOnOverride = false;
  bool tailstrikeProtectionOn = false;
  bool computerRunning = true;
};

class ShadowChecker {
 public:
  /** Mismatch episodes whose details are kept (later episodes are only counted). */
  static constexpr int kMaxEvents = 16;
  /** Differing fields listed per episode (the total number is always kept). */
  static constexpr int kMaxEventFields = 24;

  /**
   * unit: 1-based unit number for the log (FADEC: engine). leaves/leafCount: the output struct's leaf table (static
   * storage), outputBytes: its sizeof. hasSimData: the inputs have time.monotonic_time and base_sim_data (FCU, FAC).
   * nanEqual: two NaNs in a float leaf match whatever their sign and payload (the FAC: x86 and wasm take a NaN result
   * from whichever operand comes first in a commutative operation, which the generated and readable code order
   * differently; no FAC output depends on a NaN's sign or payload). Such frames are counted, not reported.
   */
  ShadowChecker(int unit, const Leaf* leaves, size_t leafCount, size_t outputBytes, bool hasSimData, bool nanEqual = false);
  ShadowChecker(const ShadowChecker&) = delete;
  ShadowChecker& operator=(const ShadowChecker&) = delete;

  /** One frame: compares `readable` with `generated` (both outputBytes long) after both computers stepped on `in`. */
  void step(const void* generated, const void* readable, const FrameInputs& in);

  int unit() const { return unit_; }
  /** Frames stepped. */
  uint64_t frames() const { return frames_; }
  /** Frames where at least one output leaf differed. */
  uint64_t mismatchFrames() const { return mismatchFrames_; }
  /** With nanEqual: frames that matched only because two NaNs were taken as equal. */
  uint64_t nanEqualFrames() const { return nanEqualFrames_; }
  uint64_t episodes() const { return episodes_; }
  /** Frame (1-based, see frames()) and sim time of the first mismatch; 0 if none. */
  uint64_t firstMismatchFrame() const { return firstMismatchFrame_; }
  double firstMismatchSimTime() const { return firstMismatchSimTime_; }
  /** Number of differing leaves on the first mismatching frame, and the first of them. */
  uint32_t firstMismatchFieldCount() const { return firstMismatchFieldCount_; }
  const char* firstMismatchFieldName() const;
  /** Mismatching frames of one output leaf (index in the leaf table). */
  uint32_t leafMismatches(size_t leaf) const { return leafMismatches_[leaf]; }
  size_t leafCount() const { return leafCount_; }
  const Leaf& leaf(size_t i) const { return leaves_[i]; }
  size_t outputBytes() const { return outputBytes_; }

  /**
   * Writes the log records not written yet (episode starts with their fields and inputs, episode ends) to `f`; `context`
   * (e.g. "fdr=... fdr_record=...") is appended to each record. Returns the number of records written.
   */
  int writeNewEvents(std::FILE* f, const char* context);
  /** True when writeNewEvents() has something to write. */
  bool hasNewEvents() const { return pendingStarts_ > 0 || pendingEnds_ > 0; }
  /** The end-of-session summary: totals and the mismatch count of every leaf that ever differed. */
  void writeSummary(std::FILE* f) const;

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
    FrameInputs inputs;
    uint32_t fieldCount;
    uint32_t listed;
    FieldValue fields[kMaxEventFields];
    bool startWritten;
    bool endWritten;
  };

  bool differs(const uint8_t* a, const uint8_t* b) const;
  /** Leaf i differs (by bit pattern; with nanEqual, two NaNs never differ). */
  bool leafDiffers(size_t i, const uint8_t* a, const uint8_t* b) const;
  void recordMismatch(const uint8_t* generated, const uint8_t* readable, const FrameInputs& in);

  const int unit_;
  const Leaf* const leaves_;
  const size_t leafCount_;
  const size_t outputBytes_;
  const bool hasSimData_;
  const bool nanEqual_;
  std::unique_ptr<uint64_t[]> mask_;  // bytes that belong to a leaf (padding excluded), (outputBytes + 7) / 8 words

  uint64_t frames_ = 0;
  uint64_t runningFrames_ = 0;
  uint64_t restarts_ = 0;
  bool wasRunning_ = false;
  uint64_t slewFrames_ = 0;
  uint64_t pauseFrames_ = 0;
  uint64_t trackingOverrideFrames_ = 0;
  uint64_t tailstrikeProtectionFrames_ = 0;

  uint64_t mismatchFrames_ = 0;
  uint64_t episodes_ = 0;
  uint64_t nanOnlyFrames_ = 0;         // every differing leaf: NaN in both, different payload
  uint64_t nanEqualFrames_ = 0;        // nanEqual: frames that differed only in NaN bits
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
  std::unique_ptr<Event[]> events_;

  std::unique_ptr<uint32_t[]> leafMismatches_;
  std::unique_ptr<uint64_t[]> leafFirstFrame_;
  std::unique_ptr<double[]> leafFirstSimTime_;
};

/**
 * The shadow log file of one computer type: one per session, named after the session's first flight data recorder file
 * (<fdr name><suffix>, e.g. 2026-11-15-10-00-00.fcu-shadow.log), or after the UTC time when no recording is running.
 * Opened in append mode only when there is something to write (episodes, the summary).
 */
class ShadowLog {
 public:
  /**
   * directory: prefix of the log path ("\\work\\" in the sim). name: the computer in the log and on the console ("FCU",
   * "FADEC"). suffix: ".fcu-shadow.log". description: the header comment (what is compared with what).
   */
  ShadowLog(std::string directory, std::string name, std::string suffix, std::string description)
      : directory_(std::move(directory)), name_(std::move(name)), suffix_(std::move(suffix)), description_(std::move(description)) {}

  /**
   * Once per frame, after the flight data recorder update: writes the new records of each unit (rare) and prints a
   * console line at each unit's first mismatch. fdrFile: the recorder's current file (any directory prefix is dropped;
   * empty when not recording); fdrRecord: index of this frame's record in it, -1 if this frame was not recorded.
   */
  void update(ShadowChecker* const units[], int count, const std::string& fdrFile, long fdrRecord);

  /** End of session: writes the summary of each unit to the log and prints one console line. */
  void writeSummary(ShadowChecker* const units[], int count);

  /** The log path (empty until known). */
  const std::string& path() const { return path_; }
  /** Uses this path instead of the session name, "-" for stdout (native tests). */
  void setPath(std::string path) { path_ = std::move(path); }

 private:
  static constexpr int kMaxUnits = 4;

  std::FILE* open(const ShadowChecker* first);
  std::string unitName(const ShadowChecker& unit, int count) const;

  std::string directory_;
  std::string name_;
  std::string suffix_;
  std::string description_;
  std::string path_;
  bool headerWritten_ = false;
  bool firstReported_[kMaxUnits] = {};
  std::string sessionFdr_;
};

}  // namespace shadow
