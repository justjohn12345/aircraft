// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "ShadowChecker.h"

#include <cmath>
#include <cstring>
#include <ctime>

namespace shadow {

namespace {

const char* kindName(LeafKind kind) {
  switch (kind) {
    case LeafKind::Bool:
      return "bool";
    case LeafKind::Float:
      return "float";
    case LeafKind::Int:
      return "int";
    case LeafKind::Enum:
      return "enum";
  }
  return "?";
}

double floatValue(const Leaf& leaf, const uint8_t* bytes) {
  if (leaf.size == 4) {
    float v;
    std::memcpy(&v, bytes, 4);
    return v;
  }
  double v;
  std::memcpy(&v, bytes, 8);
  return v;
}

/** "value": the values differ; "nan-payload": both NaN, different bits; "zero-sign": +0 vs -0. */
const char* mismatchClass(const Leaf& leaf, const uint8_t* generated, const uint8_t* readable) {
  if (leaf.kind != LeafKind::Float) {
    return "value";
  }
  const double g = floatValue(leaf, generated);
  const double r = floatValue(leaf, readable);
  if (std::isnan(g) && std::isnan(r)) {
    return "nan-payload";
  }
  if (g == 0 && r == 0) {
    return "zero-sign";
  }
  return "value";
}

void formatValue(const Leaf& leaf, const uint8_t* bytes, char* buf, size_t size) {
  if (leaf.kind == LeafKind::Float) {
    std::snprintf(buf, size, leaf.size == 4 ? "%.9g" : "%.17g", floatValue(leaf, bytes));
  } else if (leaf.kind == LeafKind::Bool) {
    std::snprintf(buf, size, "%u", static_cast<unsigned>(bytes[0]));
  } else if (leaf.size == 4) {
    int32_t v;
    std::memcpy(&v, bytes, 4);
    std::snprintf(buf, size, "%ld", static_cast<long>(v));
  } else {
    std::snprintf(buf, size, "%d", static_cast<int>(static_cast<int8_t>(bytes[0])));
  }
}

void formatBits(const Leaf& leaf, const uint8_t* bytes, char* buf, size_t size) {
  uint64_t bits = 0;
  std::memcpy(&bits, bytes, leaf.size);  // little endian on both x86-64 and wasm32
  std::snprintf(buf, size, "0x%0*llx", leaf.size * 2, static_cast<unsigned long long>(bits));
}

unsigned long long ull(uint64_t v) {
  return static_cast<unsigned long long>(v);
}

std::string utcStamp() {
  std::time_t now = std::time(nullptr);
  char buf[32] = "unknown";
  if (const std::tm* t = std::gmtime(&now)) {
    std::strftime(buf, sizeof buf, "%Y-%m-%d-%H-%M-%S", t);
  }
  return buf;
}

std::string baseName(const std::string& path) {
  const size_t slash = path.find_last_of("\\/");
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

}  // namespace

// ---- ShadowChecker -----------------------------------------------------------------------------------------------------

ShadowChecker::ShadowChecker(int unit, const Leaf* leaves, size_t leafCount, size_t outputBytes, bool hasSimData, bool nanEqual)
    : unit_(unit),
      leaves_(leaves),
      leafCount_(leafCount),
      outputBytes_(outputBytes),
      hasSimData_(hasSimData),
      nanEqual_(nanEqual),
      mask_(new uint64_t[(outputBytes + 7) / 8]()),
      events_(new Event[kMaxEvents]()),
      leafMismatches_(new uint32_t[leafCount]()),
      leafFirstFrame_(new uint64_t[leafCount]()),
      leafFirstSimTime_(new double[leafCount]()) {
  auto* mask = reinterpret_cast<uint8_t*>(mask_.get());
  for (size_t i = 0; i < leafCount_; i++) {
    std::memset(mask + leaves_[i].offset, 0xff, leaves_[i].size);
  }
}

const char* ShadowChecker::firstMismatchFieldName() const {
  return firstMismatchLeaf_ >= 0 ? leaves_[firstMismatchLeaf_].name : "";
}

bool ShadowChecker::differs(const uint8_t* a, const uint8_t* b) const {
  // Whole-struct XOR under the leaf mask (padding bytes are not compared), no branches.
  uint64_t acc = 0;
  const size_t fullWords = outputBytes_ / 8;
  for (size_t i = 0; i < fullWords; i++) {
    uint64_t x, y;
    std::memcpy(&x, a + 8 * i, 8);
    std::memcpy(&y, b + 8 * i, 8);
    acc |= (x ^ y) & mask_[i];
  }
  if (outputBytes_ % 8 != 0) {
    uint64_t x = 0, y = 0;
    std::memcpy(&x, a + 8 * fullWords, outputBytes_ % 8);
    std::memcpy(&y, b + 8 * fullWords, outputBytes_ % 8);
    acc |= (x ^ y) & mask_[fullWords];
  }
  return acc != 0;
}

bool ShadowChecker::leafDiffers(size_t i, const uint8_t* a, const uint8_t* b) const {
  const Leaf& leaf = leaves_[i];
  if (std::memcmp(a + leaf.offset, b + leaf.offset, leaf.size) == 0) {
    return false;
  }
  return !(nanEqual_ && leaf.kind == LeafKind::Float && std::isnan(floatValue(leaf, a + leaf.offset)) &&
           std::isnan(floatValue(leaf, b + leaf.offset)));
}

void ShadowChecker::step(const void* generated, const void* readable, const FrameInputs& in) {
  frames_++;
  const bool running = in.computerRunning;
  runningFrames_ += running;
  restarts_ += running && !wasRunning_;
  wasRunning_ = running;
  slewFrames_ += in.slewOn;
  pauseFrames_ += in.pauseOn;
  trackingOverrideFrames_ += in.trackingModeOnOverride;
  tailstrikeProtectionFrames_ += in.tailstrikeProtectionOn;

  const auto* g = static_cast<const uint8_t*>(generated);
  const auto* r = static_cast<const uint8_t*>(readable);
  bool mismatch = differs(g, r);
  if (mismatch && nanEqual_) {
    // the bytes differ somewhere: a mismatch unless every differing leaf holds a NaN in both
    mismatch = false;
    for (size_t i = 0; i < leafCount_ && !mismatch; i++) {
      mismatch = leafDiffers(i, g, r);
    }
    nanEqualFrames_ += !mismatch;
  }
  if (mismatch) {
    recordMismatch(g, r, in);
  } else if (inEpisode_) {
    inEpisode_ = false;
    const uint64_t length = frames_ - episodeStart_;
    longestEpisode_ = length > longestEpisode_ ? length : longestEpisode_;
    if (currentEvent_ >= 0) {
      events_[currentEvent_].endFrame = frames_;
      pendingEnds_++;
      currentEvent_ = -1;
    }
  }
}

void ShadowChecker::recordMismatch(const uint8_t* pg, const uint8_t* pr, const FrameInputs& in) {
  mismatchFrames_++;
  Event* event = nullptr;
  if (!inEpisode_) {
    inEpisode_ = true;
    episodeStart_ = frames_;
    episodes_++;
    if (eventCount_ < kMaxEvents) {
      currentEvent_ = eventCount_++;
      event = &events_[currentEvent_];
      event->episode = episodes_;
      event->frame = frames_;
      event->endFrame = 0;
      event->inputs = in;
      event->fieldCount = 0;
      event->listed = 0;
      event->startWritten = false;
      event->endWritten = false;
      pendingStarts_++;
    }
  }

  uint32_t fields = 0;
  bool nanOnly = true;
  for (size_t i = 0; i < leafCount_; i++) {
    const Leaf& leaf = leaves_[i];
    if (!leafDiffers(i, pg, pr)) {
      continue;
    }
    fields++;
    const char* cls = mismatchClass(leaf, pg + leaf.offset, pr + leaf.offset);
    nanOnly = nanOnly && std::strcmp(cls, "nan-payload") == 0;
    signedZeroLeafFrames_ += std::strcmp(cls, "zero-sign") == 0;
    if (leafMismatches_[i]++ == 0) {
      leafFirstFrame_[i] = frames_;
      leafFirstSimTime_[i] = in.simulationTime;
    }
    if (event && event->listed < kMaxEventFields) {
      FieldValue& f = event->fields[event->listed++];
      f.leaf = static_cast<uint16_t>(i);
      std::memcpy(f.generated, pg + leaf.offset, leaf.size);
      std::memcpy(f.readable, pr + leaf.offset, leaf.size);
    }
    if (firstMismatchFrame_ == 0 && firstMismatchLeaf_ < 0) {
      firstMismatchLeaf_ = static_cast<int>(i);
    }
  }
  nanOnlyFrames_ += nanOnly;
  if (event) {
    event->fieldCount = fields;
  }
  if (firstMismatchFrame_ == 0) {
    firstMismatchFrame_ = frames_;
    firstMismatchSimTime_ = in.simulationTime;
    firstMismatchFieldCount_ = fields;
  }
}

int ShadowChecker::writeNewEvents(std::FILE* f, const char* context) {
  int written = 0;
  for (int k = 0; k < eventCount_ && (pendingStarts_ > 0 || pendingEnds_ > 0); k++) {
    Event& e = events_[k];
    if (!e.startWritten) {
      std::fprintf(f, "MISMATCH unit=%d episode=%llu frame=%llu sim_time=%.6f", unit_, ull(e.episode), ull(e.frame),
                   e.inputs.simulationTime);
      if (hasSimData_) {
        std::fprintf(f, " running=%d", e.inputs.computerRunning ? 1 : 0);
      }
      std::fprintf(f, " fields=%lu listed=%lu %s\n", static_cast<unsigned long>(e.fieldCount), static_cast<unsigned long>(e.listed),
                   context);
      for (uint32_t j = 0; j < e.listed; j++) {
        const FieldValue& fv = e.fields[j];
        const Leaf& leaf = leaves_[fv.leaf];
        char g[40], r[40], gb[24], rb[24];
        formatValue(leaf, fv.generated, g, sizeof g);
        formatValue(leaf, fv.readable, r, sizeof r);
        formatBits(leaf, fv.generated, gb, sizeof gb);
        formatBits(leaf, fv.readable, rb, sizeof rb);
        std::fprintf(f, "  FIELD name=%s kind=%s class=%s generated=%s readable=%s generated_bits=%s readable_bits=%s\n", leaf.name,
                     kindName(leaf.kind), mismatchClass(leaf, fv.generated, fv.readable), g, r, gb, rb);
      }
      const FrameInputs& in = e.inputs;
      if (hasSimData_) {
        std::fprintf(f,
                     "  INPUTS dt=%.17g simulation_time=%.17g monotonic_time=%.17g slew_on=%d pause_on=%d tracking_mode_on_override=%d "
                     "tailstrike_protection_on=%d computer_running=%d\n",
                     in.dt, in.simulationTime, in.monotonicTime, in.slewOn, in.pauseOn, in.trackingModeOnOverride,
                     in.tailstrikeProtectionOn, in.computerRunning);
      } else {
        std::fprintf(f, "  INPUTS dt=%.17g simulation_time=%.17g\n", in.dt, in.simulationTime);
      }
      e.startWritten = true;
      pendingStarts_--;
      written++;
    }
    if (e.endFrame != 0 && !e.endWritten) {
      std::fprintf(f, "END unit=%d episode=%llu frame=%llu length=%llu %s\n", unit_, ull(e.episode), ull(e.endFrame),
                   ull(e.endFrame - e.frame), context);
      e.endWritten = true;
      pendingEnds_--;
      written++;
    }
  }
  return written;
}

void ShadowChecker::writeSummary(std::FILE* f) const {
  const uint64_t ongoing = inEpisode_ ? frames_ + 1 - episodeStart_ : 0;
  std::fprintf(f, "SUMMARY unit=%d frames=%llu ", unit_, ull(frames_));
  if (hasSimData_) {
    std::fprintf(f, "running_frames=%llu restarts=%llu ", ull(runningFrames_), ull(restarts_));
  }
  std::fprintf(f,
               "mismatch_frames=%llu episodes=%llu episodes_logged=%d longest_episode=%llu in_episode=%d first_mismatch_frame=%llu "
               "first_mismatch_sim_time=%.6f nan_only_frames=%llu zero_sign_leaf_frames=%llu\n",
               ull(mismatchFrames_), ull(episodes_), eventCount_, ull(ongoing > longestEpisode_ ? ongoing : longestEpisode_),
               inEpisode_ ? 1 : 0, ull(firstMismatchFrame_), firstMismatchSimTime_, ull(nanOnlyFrames_), ull(signedZeroLeafFrames_));
  if (nanEqual_) {
    std::fprintf(f, "NAN_EQUAL unit=%d frames=%llu\n", unit_, ull(nanEqualFrames_));
  }
  if (hasSimData_) {
    std::fprintf(f,
                 "UNRECORDED unit=%d slew_on_frames=%llu pause_on_frames=%llu tracking_mode_on_override_frames=%llu "
                 "tailstrike_protection_on_frames=%llu\n",
                 unit_, ull(slewFrames_), ull(pauseFrames_), ull(trackingOverrideFrames_), ull(tailstrikeProtectionFrames_));
  }
  for (size_t i = 0; i < leafCount_; i++) {
    if (leafMismatches_[i] != 0) {
      std::fprintf(f, "  COUNT unit=%d name=%s frames=%lu first_frame=%llu first_sim_time=%.6f\n", unit_, leaves_[i].name,
                   static_cast<unsigned long>(leafMismatches_[i]), ull(leafFirstFrame_[i]), leafFirstSimTime_[i]);
    }
  }
}

// ---- ShadowLog ---------------------------------------------------------------------------------------------------------

std::string ShadowLog::unitName(const ShadowChecker& unit, int count) const {
  return count == 1 ? name_ : name_ + " " + std::to_string(unit.unit());
}

std::FILE* ShadowLog::open(const ShadowChecker* first) {
  if (path_.empty()) {
    path_ = directory_ + utcStamp() + suffix_;
  }
  std::FILE* f = path_ == "-" ? stdout : std::fopen(path_.c_str(), "a");
  if (!f) {
    std::printf("WASM: %s SHADOW: cannot open %s, writing the log to the console\n", name_.c_str(), path_.c_str());
    f = stdout;
  }
  if (!headerWritten_) {
    headerWritten_ = true;
    std::fprintf(f,
                 "# %s shadow log: %s, every output leaf compared\n"
                 "# bit-exact every frame. Format and offline reproduction: tools/fmgc-lab/docs/shadow.md\n",
                 name_.c_str(), description_.c_str());
    std::fprintf(f, "%s_SHADOW_LOG version=1 leaves=%lu output_bytes=%lu session_fdr=%s created_utc=%s\n", name_.c_str(),
                 static_cast<unsigned long>(first ? first->leafCount() : 0), static_cast<unsigned long>(first ? first->outputBytes() : 0),
                 sessionFdr_.empty() ? "none" : sessionFdr_.c_str(), utcStamp().c_str());
  }
  return f;
}

void ShadowLog::update(ShadowChecker* const units[], int count, const std::string& fdrFile, long fdrRecord) {
  const std::string fdrName = baseName(fdrFile);
  if (sessionFdr_.empty() && !fdrName.empty()) {
    sessionFdr_ = fdrName;
    if (path_.empty()) {
      // named after the session's first recording: 2026-11-15-10-00-00.fdr -> 2026-11-15-10-00-00<suffix>
      path_ = directory_ + fdrName.substr(0, fdrName.rfind('.')) + suffix_;
    }
  }
  const ShadowChecker* first = nullptr;
  bool any = false;
  for (int i = 0; i < count; i++) {
    first = first ? first : units[i];
    any = any || (units[i] && units[i]->hasNewEvents());
  }
  if (!any) {
    return;
  }

  char context[192];
  char record[24] = "none";
  if (fdrRecord >= 0) {
    std::snprintf(record, sizeof record, "%ld", fdrRecord);
  }
  std::snprintf(context, sizeof context, "fdr=%s fdr_record=%s utc=%s", fdrName.empty() ? "none" : fdrName.c_str(), record,
                utcStamp().c_str());
  std::FILE* f = open(first);
  for (int i = 0; i < count; i++) {
    ShadowChecker* u = units[i];
    if (!u || !u->hasNewEvents()) {
      continue;
    }
    if (i < kMaxUnits && !firstReported_[i] && u->firstMismatchFrame() != 0) {
      firstReported_[i] = true;
      std::printf("WASM: %s SHADOW: first mismatch at frame %llu, sim time %.3f s: %lu output fields differ (%s); %s; log %s\n",
                  unitName(*u, count).c_str(), ull(u->firstMismatchFrame()), u->firstMismatchSimTime(),
                  static_cast<unsigned long>(u->firstMismatchFieldCount()), u->firstMismatchFieldName(), context, path_.c_str());
    }
    u->writeNewEvents(f, context);
  }
  if (f != stdout) {
    std::fclose(f);
  }
}

void ShadowLog::writeSummary(ShadowChecker* const units[], int count) {
  uint64_t frames = 0;
  const ShadowChecker* first = nullptr;
  for (int i = 0; i < count; i++) {
    frames += units[i] ? units[i]->frames() : 0;
    first = first ? first : units[i];
  }
  if (frames == 0) {
    return;  // no flight (e.g. the module was unloaded from the menu)
  }
  std::FILE* f = open(first);
  std::fprintf(f, "SESSION_END utc=%s\n", utcStamp().c_str());
  std::string line = "WASM: " + name_ + " SHADOW:";
  for (int i = 0; i < count; i++) {
    ShadowChecker* u = units[i];
    if (!u) {
      continue;
    }
    u->writeSummary(f);
    char buf[160];
    std::snprintf(buf, sizeof buf, " %s %llu frames, %llu mismatching (%llu episodes);", unitName(*u, count).c_str(), ull(u->frames()),
                  ull(u->mismatchFrames()), ull(u->episodes()));
    line += buf;
  }
  if (f != stdout) {
    std::fclose(f);
  }
  std::printf("%s log %s\n", line.c_str(), path_.c_str());
}

}  // namespace shadow
