#pragma once
#include "types.h"
#include <string>
#include <vector>

// Grim Reaper 2.0, Phase 5: the user's pulls. History is every pull (newest
// last, capped), Keep is the curated library: kept pulls are never trimmed,
// kept dead machines included. One small JSON file, written atomically (temp
// file + rename), so an interrupted save leaves the previous file intact. A file
// that cannot be read is moved aside to `<path>.corrupt` and the library starts
// empty rather than refusing to open.

struct GrimPullEntry {
  u64 pull = 0;         // pull number, counts up forever
  u64 seed = 0;
  u64 genome_hash = 0;
  std::string genome;   // canonical genome text (grim_genome_serialize)
  u32 families = 0;     // settings the pull was made with
  u32 intensity = 0;
  bool rot = false;
  bool dead = false;
  std::string reason;   // death gate id, empty while alive
  std::string headline; // plain-words cause of death
  double seconds = 0.0; // how long it lived (or was watched)
  bool kept = false;
  bool mercy = false;   // replaced silently by Mercy; counted, not shown in History
  std::string note;
};

struct GrimLibraryStats {
  u64 pulls = 0, deaths = 0, kept = 0, mercy_rerolls = 0;
  double longest_alive_seconds = 0.0;
  u64 longest_alive_pull = 0;
};

class GrimLibrary {
public:
  static constexpr size_t kMaxHistory = 200; // un-kept pulls retained

  // Adds a pull and returns its number. Trims old un-kept pulls.
  u64 add(GrimPullEntry entry);
  // The machine's outcome becomes known later (death watch).
  bool update(u64 pull, bool dead, const std::string &reason, const std::string &headline,
              double seconds);
  bool set_kept(u64 pull, bool kept);
  bool mark_mercy(u64 pull);
  const GrimPullEntry *find(u64 pull) const;
  const std::vector<GrimPullEntry> &entries() const { return entries_; }
  GrimLibraryStats stats() const;
  u64 next_pull() const { return next_pull_; }

  bool load(const std::string &path, std::string &err);
  bool save(const std::string &path, std::string &err) const;

private:
  std::vector<GrimPullEntry> entries_;
  u64 next_pull_ = 1;
  u64 total_pulls_ = 0, total_deaths_ = 0; // session-independent, survive trimming
};
