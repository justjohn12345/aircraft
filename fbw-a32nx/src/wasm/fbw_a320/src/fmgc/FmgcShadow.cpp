// Copyright (c) 2026 FlyByWire Simulations
//
// SPDX-License-Identifier: GPL-3.0

#include "FmgcShadow.h"

#include <cmath>
#include <cstring>
#include <ctime>

// The readable FMGC, compiled from tools/fmgc-lab/readable (include path: tools/fmgc-lab). Qualified with its folder so
// it can never resolve to this folder's Fmgc.h on a case-insensitive file system.
#include "readable/fmgc.h"

using fmgc_shadow::kOutputLeafCount;
using fmgc_shadow::kOutputLeaves;
using fmgc_shadow::Leaf;
using fmgc_shadow::LeafKind;

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

// ---- FmgcShadow --------------------------------------------------------------------------------------------------------

FmgcShadow::FmgcShadow(int unit) : unit_(unit), readable_(std::make_unique<fmgc::readable::FmgcComputer>()) {
  readable_->initialize();
  std::memset(mask_, 0, sizeof mask_);
  auto* mask = reinterpret_cast<uint8_t*>(mask_);
  for (const Leaf& leaf : kOutputLeaves) {
    std::memset(mask + leaf.offset, 0xff, leaf.size);
  }
  std::memset(events_, 0, sizeof events_);
  std::memset(leafMismatches_, 0, sizeof leafMismatches_);
  std::memset(leafFirstFrame_, 0, sizeof leafFirstFrame_);
  std::memset(leafFirstSimTime_, 0, sizeof leafFirstSimTime_);
}

FmgcShadow::~FmgcShadow() = default;

const char* FmgcShadow::firstMismatchFieldName() const {
  return firstMismatchLeaf_ >= 0 ? kOutputLeaves[firstMismatchLeaf_].name : "";
}

bool FmgcShadow::differs(const fmgc_outputs& a, const fmgc_outputs& b) const {
  // Whole-struct XOR under the leaf mask (padding bytes are not compared): ~480 words, no branches.
  const auto* pa = reinterpret_cast<const uint8_t*>(&a);
  const auto* pb = reinterpret_cast<const uint8_t*>(&b);
  uint64_t acc = 0;
  constexpr size_t kFullWords = sizeof(fmgc_outputs) / 8;
  for (size_t i = 0; i < kFullWords; i++) {
    uint64_t x, y;
    std::memcpy(&x, pa + 8 * i, 8);
    std::memcpy(&y, pb + 8 * i, 8);
    acc |= (x ^ y) & mask_[i];
  }
  if constexpr (kFullWords < kOutputWords) {
    uint64_t x = 0, y = 0;
    std::memcpy(&x, pa + 8 * kFullWords, sizeof(fmgc_outputs) - 8 * kFullWords);
    std::memcpy(&y, pb + 8 * kFullWords, sizeof(fmgc_outputs) - 8 * kFullWords);
    acc |= (x ^ y) & mask_[kFullWords];
  }
  return acc != 0;
}

void FmgcShadow::step(const fmgc_inputs& in, const fmgc_outputs& generated) {
  frames_++;
  const bool running = in.sim_data.computer_running;
  runningFrames_ += running;
  restarts_ += running && !wasRunning_;
  wasRunning_ = running;
  slewFrames_ += in.sim_data.slew_on != 0;
  pauseFrames_ += in.sim_data.pause_on != 0;
  trackingOverrideFrames_ += in.sim_data.tracking_mode_on_override != 0;
  tailstrikeProtectionFrames_ += in.sim_data.tailstrike_protection_on != 0;

  const fmgc_outputs& readable = readable_->step(in);
  if (differs(generated, readable)) {
    recordMismatch(in, generated, readable);
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

void FmgcShadow::recordMismatch(const fmgc_inputs& in, const fmgc_outputs& generated, const fmgc_outputs& readable) {
  mismatchFrames_++;
  const bool newEpisode = !inEpisode_;
  Event* event = nullptr;
  if (newEpisode) {
    inEpisode_ = true;
    episodeStart_ = frames_;
    episodes_++;
    if (eventCount_ < kMaxEvents) {
      currentEvent_ = eventCount_++;
      event = &events_[currentEvent_];
      event->episode = episodes_;
      event->frame = frames_;
      event->endFrame = 0;
      event->time = in.time;
      event->simData = in.sim_data;
      event->fieldCount = 0;
      event->listed = 0;
      event->startWritten = false;
      event->endWritten = false;
      pendingStarts_++;
    }
  }

  const auto* pg = reinterpret_cast<const uint8_t*>(&generated);
  const auto* pr = reinterpret_cast<const uint8_t*>(&readable);
  uint32_t fields = 0;
  bool nanOnly = true;
  for (size_t i = 0; i < kOutputLeafCount; i++) {
    const Leaf& leaf = kOutputLeaves[i];
    if (std::memcmp(pg + leaf.offset, pr + leaf.offset, leaf.size) == 0) {
      continue;
    }
    fields++;
    const char* cls = mismatchClass(leaf, pg + leaf.offset, pr + leaf.offset);
    nanOnly = nanOnly && std::strcmp(cls, "nan-payload") == 0;
    signedZeroLeafFrames_ += std::strcmp(cls, "zero-sign") == 0;
    if (leafMismatches_[i]++ == 0) {
      leafFirstFrame_[i] = frames_;
      leafFirstSimTime_[i] = in.time.simulation_time;
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
    firstMismatchSimTime_ = in.time.simulation_time;
    firstMismatchFieldCount_ = fields;
  }
}

int FmgcShadow::writeNewEvents(std::FILE* f, const char* context) {
  int written = 0;
  for (int k = 0; k < eventCount_ && (pendingStarts_ > 0 || pendingEnds_ > 0); k++) {
    Event& e = events_[k];
    if (!e.startWritten) {
      std::fprintf(f, "MISMATCH unit=%d episode=%llu frame=%llu sim_time=%.6f running=%d fields=%lu listed=%lu %s\n", unit_,
                   ull(e.episode), ull(e.frame), e.time.simulation_time, e.simData.computer_running ? 1 : 0,
                   static_cast<unsigned long>(e.fieldCount), static_cast<unsigned long>(e.listed), context);
      for (uint32_t j = 0; j < e.listed; j++) {
        const FieldValue& fv = e.fields[j];
        const Leaf& leaf = kOutputLeaves[fv.leaf];
        char g[40], r[40], gb[24], rb[24];
        formatValue(leaf, fv.generated, g, sizeof g);
        formatValue(leaf, fv.readable, r, sizeof r);
        formatBits(leaf, fv.generated, gb, sizeof gb);
        formatBits(leaf, fv.readable, rb, sizeof rb);
        std::fprintf(f, "  FIELD name=%s kind=%s class=%s generated=%s readable=%s generated_bits=%s readable_bits=%s\n", leaf.name,
                     kindName(leaf.kind), mismatchClass(leaf, fv.generated, fv.readable), g, r, gb, rb);
      }
      std::fprintf(f,
                   "  INPUTS dt=%.17g simulation_time=%.17g monotonic_time=%.17g slew_on=%d pause_on=%d tracking_mode_on_override=%d "
                   "tailstrike_protection_on=%d computer_running=%d\n",
                   e.time.dt, e.time.simulation_time, e.time.monotonic_time, e.simData.slew_on, e.simData.pause_on,
                   e.simData.tracking_mode_on_override, e.simData.tailstrike_protection_on, e.simData.computer_running);
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

void FmgcShadow::writeSummary(std::FILE* f) const {
  const uint64_t ongoing = inEpisode_ ? frames_ + 1 - episodeStart_ : 0;
  std::fprintf(f,
               "SUMMARY unit=%d frames=%llu running_frames=%llu restarts=%llu mismatch_frames=%llu episodes=%llu "
               "episodes_logged=%d longest_episode=%llu in_episode=%d first_mismatch_frame=%llu first_mismatch_sim_time=%.6f "
               "nan_only_frames=%llu zero_sign_leaf_frames=%llu\n",
               unit_, ull(frames_), ull(runningFrames_), ull(restarts_), ull(mismatchFrames_), ull(episodes_), eventCount_,
               ull(ongoing > longestEpisode_ ? ongoing : longestEpisode_), inEpisode_ ? 1 : 0, ull(firstMismatchFrame_),
               firstMismatchSimTime_, ull(nanOnlyFrames_), ull(signedZeroLeafFrames_));
  std::fprintf(f,
               "UNRECORDED unit=%d slew_on_frames=%llu pause_on_frames=%llu tracking_mode_on_override_frames=%llu "
               "tailstrike_protection_on_frames=%llu\n",
               unit_, ull(slewFrames_), ull(pauseFrames_), ull(trackingOverrideFrames_), ull(tailstrikeProtectionFrames_));
  for (size_t i = 0; i < kOutputLeafCount; i++) {
    if (leafMismatches_[i] != 0) {
      std::fprintf(f, "  COUNT unit=%d name=%s frames=%lu first_frame=%llu first_sim_time=%.6f\n", unit_, kOutputLeaves[i].name,
                   static_cast<unsigned long>(leafMismatches_[i]), ull(leafFirstFrame_[i]), leafFirstSimTime_[i]);
    }
  }
}

// ---- FmgcShadowLog -----------------------------------------------------------------------------------------------------

std::FILE* FmgcShadowLog::open() {
  if (path_.empty()) {
    path_ = directory_ + utcStamp() + ".fmgc-shadow.log";
  }
  std::FILE* f = path_ == "-" ? stdout : std::fopen(path_.c_str(), "a");
  if (!f) {
    std::printf("WASM: FMGC SHADOW: cannot open %s, writing the log to the console\n", path_.c_str());
    return stdout;
  }
  if (!headerWritten_) {
    headerWritten_ = true;
    std::fprintf(f,
                 "# FMGC shadow log: readable FMGC (tools/fmgc-lab/readable) vs generated FmgcComputer, every output leaf compared\n"
                 "# bit-exact every frame. Format and offline reproduction: tools/fmgc-lab/docs/shadow.md\n");
    std::fprintf(f, "FMGC_SHADOW_LOG version=1 leaves=%lu output_bytes=%lu session_fdr=%s created_utc=%s\n",
                 static_cast<unsigned long>(kOutputLeafCount), static_cast<unsigned long>(sizeof(fmgc_outputs)),
                 sessionFdr_.empty() ? "none" : sessionFdr_.c_str(), utcStamp().c_str());
  }
  return f;
}

void FmgcShadowLog::update(FmgcShadow* const units[], int count, const std::string& fdrFile, long fdrRecord) {
  const std::string fdrName = baseName(fdrFile);
  if (sessionFdr_.empty() && !fdrName.empty()) {
    sessionFdr_ = fdrName;
    if (path_.empty()) {
      // named after the session's first recording: 2026-11-15-10-00-00.fdr -> 2026-11-15-10-00-00.fmgc-shadow.log
      path_ = directory_ + fdrName.substr(0, fdrName.rfind('.')) + ".fmgc-shadow.log";
    }
  }
  bool any = false;
  for (int i = 0; i < count; i++) {
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
  std::FILE* f = open();
  for (int i = 0; i < count; i++) {
    FmgcShadow* u = units[i];
    if (!u || !u->hasNewEvents()) {
      continue;
    }
    if (i < 2 && !firstReported_[i] && u->firstMismatchFrame() != 0) {
      firstReported_[i] = true;
      std::printf("WASM: FMGC %d SHADOW: first mismatch at frame %llu, sim time %.3f s: %lu output fields differ (%s); %s; log %s\n",
                  u->unit(), ull(u->firstMismatchFrame()), u->firstMismatchSimTime(),
                  static_cast<unsigned long>(u->firstMismatchFieldCount()), u->firstMismatchFieldName(), context, path_.c_str());
    }
    u->writeNewEvents(f, context);
  }
  if (f != stdout) {
    std::fclose(f);
  }
}

void FmgcShadowLog::writeSummary(FmgcShadow* const units[], int count) {
  uint64_t frames = 0;
  for (int i = 0; i < count; i++) {
    frames += units[i] ? units[i]->frames() : 0;
  }
  if (frames == 0) {
    return;  // no flight (e.g. the module was unloaded from the menu)
  }
  std::FILE* f = open();
  std::fprintf(f, "SESSION_END utc=%s\n", utcStamp().c_str());
  std::string line = "WASM: FMGC SHADOW:";
  for (int i = 0; i < count; i++) {
    FmgcShadow* u = units[i];
    if (!u) {
      continue;
    }
    u->writeSummary(f);
    char buf[160];
    std::snprintf(buf, sizeof buf, " FMGC %d %llu frames, %llu mismatching (%llu episodes);", u->unit(), ull(u->frames()),
                  ull(u->mismatchFrames()), ull(u->episodes()));
    line += buf;
  }
  if (f != stdout) {
    std::fclose(f);
  }
  std::printf("%s log %s\n", line.c_str(), path_.c_str());
}
