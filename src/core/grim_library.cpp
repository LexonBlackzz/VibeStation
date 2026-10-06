#include "grim_library.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

u64 GrimLibrary::add(GrimPullEntry entry) {
  entry.pull = next_pull_++;
  ++total_pulls_;
  if (entry.dead) {
    ++total_deaths_;
  }
  entries_.push_back(std::move(entry));
  // Trim the oldest un-kept pulls beyond the history cap.
  size_t unkept = 0;
  for (const GrimPullEntry &e : entries_) {
    unkept += e.kept ? 0u : 1u;
  }
  for (auto it = entries_.begin(); it != entries_.end() && unkept > kMaxHistory;) {
    if (!it->kept) {
      it = entries_.erase(it);
      --unkept;
    } else {
      ++it;
    }
  }
  return next_pull_ - 1u;
}

bool GrimLibrary::update(u64 pull, bool dead, const std::string &reason,
                         const std::string &headline, double seconds) {
  for (GrimPullEntry &e : entries_) {
    if (e.pull == pull) {
      if (dead && !e.dead) {
        ++total_deaths_;
      }
      e.dead = dead;
      e.reason = reason;
      e.headline = headline;
      e.seconds = seconds;
      return true;
    }
  }
  return false;
}

bool GrimLibrary::set_kept(u64 pull, bool kept) {
  for (GrimPullEntry &e : entries_) {
    if (e.pull == pull) {
      e.kept = kept;
      return true;
    }
  }
  return false;
}

bool GrimLibrary::mark_mercy(u64 pull) {
  for (GrimPullEntry &e : entries_) {
    if (e.pull == pull) {
      e.mercy = true;
      return true;
    }
  }
  return false;
}

const GrimPullEntry *GrimLibrary::find(u64 pull) const {
  for (const GrimPullEntry &e : entries_) {
    if (e.pull == pull) {
      return &e;
    }
  }
  return nullptr;
}

GrimLibraryStats GrimLibrary::stats() const {
  GrimLibraryStats s;
  s.pulls = total_pulls_;
  s.deaths = total_deaths_;
  for (const GrimPullEntry &e : entries_) {
    s.kept += e.kept ? 1u : 0u;
    s.mercy_rerolls += e.mercy ? 1u : 0u;
    if (!e.dead && e.seconds > s.longest_alive_seconds) {
      s.longest_alive_seconds = e.seconds;
      s.longest_alive_pull = e.pull;
    }
  }
  return s;
}

namespace {
std::string hex64(u64 v) {
  char b[24];
  std::snprintf(b, sizeof(b), "0x%016llX", static_cast<unsigned long long>(v));
  return b;
}
bool parse_hex64(const json &j, u64 &out) {
  if (!j.is_string()) {
    return false;
  }
  const std::string s = j.get<std::string>();
  char *end = nullptr;
  out = std::strtoull(s.c_str(), &end, 16);
  return s.size() > 2 && end != nullptr && *end == 0;
}
} // namespace

bool GrimLibrary::save(const std::string &path, std::string &err) const {
  json doc;
  doc["format"] = 1;
  doc["next_pull"] = next_pull_;
  doc["total_pulls"] = total_pulls_;
  doc["total_deaths"] = total_deaths_;
  json list = json::array();
  for (const GrimPullEntry &e : entries_) {
    json j;
    j["pull"] = e.pull;
    j["seed"] = hex64(e.seed);
    j["genome_hash"] = hex64(e.genome_hash);
    j["genome"] = e.genome;
    j["families"] = e.families;
    j["intensity"] = e.intensity;
    j["rot"] = e.rot;
    j["dead"] = e.dead;
    j["reason"] = e.reason;
    j["headline"] = e.headline;
    j["seconds"] = e.seconds;
    j["kept"] = e.kept;
    j["mercy"] = e.mercy;
    j["note"] = e.note;
    list.push_back(std::move(j));
  }
  doc["pulls"] = std::move(list);

  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
      err = "cannot write " + tmp;
      return false;
    }
    out << doc.dump(1) << '\n';
    out.flush();
    if (!out) {
      err = "write failed: " + tmp;
      return false;
    }
  }
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec); // replaces an existing file
  if (ec) {
    std::filesystem::remove(path, ec);
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
      err = "cannot replace " + path + ": " + ec.message();
      return false;
    }
  }
  return true;
}

bool GrimLibrary::load(const std::string &path, std::string &err) {
  *this = GrimLibrary{};
  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    return true; // first run: empty library
  }
  std::ifstream in(path, std::ios::binary);
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const json doc = json::parse(text, nullptr, false);
  const auto fail = [&](const char *why) {
    in.close();
    std::filesystem::rename(path, path + ".corrupt", ec);
    err = std::string("grim library unreadable (") + why + "); moved to .corrupt, starting empty";
    *this = GrimLibrary{};
    return false;
  };
  if (doc.is_discarded() || !doc.is_object() || !doc.contains("pulls") ||
      !doc["pulls"].is_array()) {
    return fail("not a library file");
  }
  GrimLibrary lib;
  lib.next_pull_ = doc.value("next_pull", u64{1});
  lib.total_pulls_ = doc.value("total_pulls", u64{0});
  lib.total_deaths_ = doc.value("total_deaths", u64{0});
  for (const json &j : doc["pulls"]) {
    GrimPullEntry e;
    if (!j.is_object() || !parse_hex64(j.value("seed", json()), e.seed) ||
        !parse_hex64(j.value("genome_hash", json()), e.genome_hash) ||
        !j.value("genome", json()).is_string()) {
      return fail("bad pull entry");
    }
    e.pull = j.value("pull", u64{0});
    e.genome = j["genome"].get<std::string>();
    e.families = j.value("families", 0u);
    e.intensity = j.value("intensity", 0u);
    e.rot = j.value("rot", false);
    e.dead = j.value("dead", false);
    e.reason = j.value("reason", std::string());
    e.headline = j.value("headline", std::string());
    e.seconds = j.value("seconds", 0.0);
    e.kept = j.value("kept", false);
    e.mercy = j.value("mercy", false);
    e.note = j.value("note", std::string());
    lib.next_pull_ = std::max(lib.next_pull_, e.pull + 1u);
    lib.entries_.push_back(std::move(e));
  }
  *this = std::move(lib);
  return true;
}
