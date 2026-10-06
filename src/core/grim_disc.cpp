#include "grim_disc.h"
#include "grim_genome.h" // GrimRng, grim_mix64
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <map>
#include <set>

namespace {
constexpr size_t kUser = 2048;
constexpr int kMaxDirectories = 4096;
constexpr int kMaxDirectorySectors = 64;

u32 le32(const u8 *p) {
  return static_cast<u32>(p[0]) | (static_cast<u32>(p[1]) << 8) | (static_cast<u32>(p[2]) << 16) |
         (static_cast<u32>(p[3]) << 24);
}

int sectors_for(u32 bytes) { return static_cast<int>((bytes + kUser - 1u) / kUser); }

std::string upper(std::string s) {
  for (char &c : s) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  return s;
}

// "SLUS_005.94;1" -> "SLUS_005.94"
std::string strip_version(const std::string &name) {
  const size_t semi = name.find(';');
  return semi == std::string::npos ? name : name.substr(0, semi);
}
} // namespace

GrimDiscReaperConfig g_grim_disc_cli;

bool grim_disc_parse(const std::string &spec, GrimDiscReaperConfig &out, std::string &err) {
  GrimDiscReaperConfig c;
  c.enabled = true;
  size_t pos = 0;
  while (pos <= spec.size()) {
    const size_t end = std::min(spec.find(',', pos), spec.size());
    const std::string item = spec.substr(pos, end - pos);
    pos = end + 1;
    if (item.empty()) {
      continue;
    }
    const size_t eq = item.find('=');
    if (eq == std::string::npos) {
      err = "expected key=value, got '" + item + "'";
      return false;
    }
    const std::string k = item.substr(0, eq), v = item.substr(eq + 1);
    if (k == "pct") {
      c.percent = static_cast<float>(std::atof(v.c_str()));
    } else if (k == "seed") {
      c.seed = std::strtoull(v.c_str(), nullptr, 10);
    } else if (k == "every") {
      c.every = static_cast<u32>(std::strtoul(v.c_str(), nullptr, 10));
    } else if (k == "risky") {
      c.skip_risky = v == "0";
    } else if (k == "start") {
      c.start_frame = static_cast<u32>(std::strtoul(v.c_str(), nullptr, 10));
    } else if (k == "value") {
      c.engine.value = static_cast<u8>(std::strtoul(v.c_str(), nullptr, 16));
    } else if (k == "match") {
      c.engine.match = static_cast<u8>(std::strtoul(v.c_str(), nullptr, 16));
    } else if (k == "offset") {
      c.engine.offset = static_cast<s32>(std::strtol(v.c_str(), nullptr, 10));
    } else if (k == "engine") {
      if (!grim_byte_op_from_key(v, c.engine.op)) {
        err = "unknown engine '" + v + "'";
        return false;
      }
    } else if (k == "targets") {
      c.targets = 0;
      if (v.find("files") != std::string::npos) c.targets |= kGrimDiscFiles;
      if (v.find("movies") != std::string::npos) c.targets |= kGrimDiscMovies;
      if (v.find("xa") != std::string::npos) c.targets |= kGrimDiscXaAudio;
      if (v.find("streams") != std::string::npos) c.targets |= kGrimDiscMovies | kGrimDiscXaAudio;
      if (v.find("audio") != std::string::npos) c.targets |= kGrimDiscCdAudio;
      if (v.find("boot") != std::string::npos) c.targets |= kGrimDiscBootCode;
    } else {
      err = "unknown key '" + k + "'";
      return false;
    }
  }
  out = c;
  return true;
}

bool GrimDiscLayout::is_system(int lba) const {
  for (const auto &r : system) {
    if (lba >= r.first && lba < r.second) {
      return true;
    }
  }
  return false;
}

GrimDiscLayout grim_disc_scan(const std::function<bool(int lba, u8 *user)> &read_user) {
  GrimDiscLayout out;
  out.system.push_back({0, 16}); // system area (licence data on a PS1 disc)
  u8 sec[kUser];
  // Volume descriptors from LBA 16 to the terminator.
  int lba = 16;
  bool pvd = false;
  u8 primary[kUser] = {};
  for (; lba < 64; ++lba) {
    if (!read_user(lba, sec) || std::string(reinterpret_cast<const char *>(sec + 1), 5) != "CD001") {
      break;
    }
    if (sec[0] == 1 && !pvd) {
      std::copy(sec, sec + kUser, primary);
      pvd = true;
    }
    if (sec[0] == 255) {
      ++lba;
      break;
    }
  }
  out.system.push_back({16, std::max(lba, 17)});
  if (!pvd) {
    out.system.push_back({16, 24}); // not ISO9660: keep the usual metadata area anyway
    return out;
  }
  out.valid = true;

  // Path tables (type L, optional L, type M, optional M).
  const int pt_sectors = sectors_for(le32(primary + 132));
  for (int off : {140, 144}) {
    const u32 at = le32(primary + off);
    if (at != 0) out.system.push_back({static_cast<int>(at), static_cast<int>(at) + pt_sectors});
  }
  for (int off : {148, 152}) {
    const u8 *p = primary + off; // big-endian copies
    const u32 at = (static_cast<u32>(p[0]) << 24) | (static_cast<u32>(p[1]) << 16) |
                   (static_cast<u32>(p[2]) << 8) | static_cast<u32>(p[3]);
    if (at != 0) out.system.push_back({static_cast<int>(at), static_cast<int>(at) + pt_sectors});
  }

  // Walk the directory tree from the root record (PVD offset 156).
  struct Dir {
    u32 lba, size;
    std::string path;
  };
  std::map<std::string, std::pair<u32, u32>> files; // "DIR\\NAME.EXT" -> (lba, size)
  std::deque<Dir> queue{{le32(primary + 156 + 2), le32(primary + 156 + 10), ""}};
  std::set<u32> seen;
  int dirs = 0;
  while (!queue.empty() && dirs < kMaxDirectories) {
    const Dir d = queue.front();
    queue.pop_front();
    if (d.lba == 0 || !seen.insert(d.lba).second) {
      continue;
    }
    ++dirs;
    const int n = std::min(sectors_for(d.size), kMaxDirectorySectors);
    out.system.push_back({static_cast<int>(d.lba), static_cast<int>(d.lba) + std::max(n, 1)});
    for (int k = 0; k < n; ++k) {
      if (!read_user(static_cast<int>(d.lba) + k, sec)) {
        break;
      }
      for (size_t pos = 0; pos + 33 < kUser;) {
        const u8 len = sec[pos];
        if (len == 0) {
          break; // records never cross a sector; the rest is padding
        }
        if (pos + len > kUser || len < 34) {
          break;
        }
        const u8 *r = sec + pos;
        const u8 name_len = r[32];
        if (33u + name_len <= len) {
          const std::string name(reinterpret_cast<const char *>(r + 33), name_len);
          const bool self_or_parent = name_len == 1 && (r[33] == 0 || r[33] == 1);
          if (!self_or_parent) {
            const std::string path = d.path.empty() ? upper(strip_version(name))
                                                    : d.path + "\\" + upper(strip_version(name));
            if (r[25] & 2u) {
              queue.push_back({le32(r + 2), le32(r + 10), path});
            } else {
              files[path] = {le32(r + 2), le32(r + 10)};
              ++out.files;
            }
          }
        }
        pos += len;
      }
    }
  }

  // SYSTEM.CNF names the boot executable ("BOOT = cdrom:\SLUS_005.94;1"); without
  // it the BIOS boots PSX.EXE.
  std::string boot = "PSX.EXE";
  const auto cnf = files.find("SYSTEM.CNF");
  if (cnf != files.end()) {
    const int cnf_sectors = std::max(1, std::min(sectors_for(cnf->second.second), 4));
    out.system.push_back({static_cast<int>(cnf->second.first), static_cast<int>(cnf->second.first) + cnf_sectors});
    std::string text;
    for (int k = 0; k < cnf_sectors; ++k) {
      if (read_user(static_cast<int>(cnf->second.first) + k, sec)) {
        text.append(reinterpret_cast<const char *>(sec), kUser);
      }
    }
    text.resize(std::min<size_t>(text.size(), cnf->second.second));
    const std::string up = upper(text);
    const size_t key = up.find("BOOT");
    const size_t eq = key == std::string::npos ? std::string::npos : up.find('=', key);
    if (eq != std::string::npos) {
      size_t a = up.find_first_not_of(" \t", eq + 1);
      size_t b = a == std::string::npos ? a : up.find_first_of(" \t\r\n", a);
      std::string v = a == std::string::npos ? std::string() : up.substr(a, b == std::string::npos ? b : b - a);
      if (v.rfind("CDROM:", 0) == 0) {
        v = v.substr(6);
      }
      while (!v.empty() && (v[0] == '\\' || v[0] == '/')) {
        v.erase(0, 1);
      }
      std::replace(v.begin(), v.end(), '/', '\\');
      if (!v.empty()) {
        boot = strip_version(v);
      }
    }
  }
  const auto exe = files.find(boot);
  if (exe != files.end()) {
    out.boot = {static_cast<int>(exe->second.first),
                static_cast<int>(exe->second.first) + std::max(1, sectors_for(exe->second.second))};
    out.boot_name = boot;
  }
  return out;
}

bool grim_disc_sector_is_risky(const u8 *data, size_t len) {
  // Code: in MIPS code most non-zero words are lui/addiu/lw/sw/jal/jr ra/branches; in
  // other data those few opcodes cover about an eighth of the words.
  size_t words = 0, code = 0;
  for (size_t i = 0; i + 4 <= len; i += 4) {
    const u32 w = le32(data + i);
    if (w == 0) {
      continue;
    }
    ++words;
    const u32 op = w >> 26;
    const bool special = op == 0 && (w & 0xFC00003Fu) == 0x00000021u; // addu (move)
    code += (op == 0x0F || op == 0x09 || op == 0x23 || op == 0x2B || op == 0x03 || op == 0x04 ||
             op == 0x05 || w == 0x03E00008u || special)
                ? 1u
                : 0u;
  }
  if (words >= 64 && code * 100 >= words * 45) {
    return true;
  }
  // Packed: compressed or encrypted data has close to 8 bits of entropy per byte.
  u32 hist[256] = {};
  for (size_t i = 0; i < len; ++i) {
    ++hist[data[i]];
  }
  double bits = 0.0;
  for (u32 h : hist) {
    if (h != 0) {
      const double p = static_cast<double>(h) / static_cast<double>(len);
      bits -= p * std::log2(p);
    }
  }
  return bits > 7.6;
}

size_t grim_disc_corrupt_sector(u8 *raw, size_t sector_size, bool audio_track, int lba,
                                const GrimDiscReaperConfig &cfg, const GrimDiscLayout &layout) {
  if (!cfg.enabled || raw == nullptr) {
    return 0;
  }
  size_t begin = 0, len = 0;
  if (audio_track) {
    if ((cfg.targets & kGrimDiscCdAudio) == 0) {
      return 0;
    }
    len = sector_size;
  } else {
    if (layout.is_system(lba)) {
      return 0;
    }
    u32 kind = kGrimDiscFiles;
    if (sector_size == kUser) {
      len = kUser;
    } else if (sector_size >= 2352) {
      const u8 mode = raw[15];
      if (mode == 2) {
        const u8 submode = raw[18];
        const bool form2 = (submode & 0x20u) != 0;
        begin = 24;
        len = form2 ? 2324 : kUser;
        // XA audio: Form 2 with the audio submode bit. Other Form 2 data is streamed
        // (usually video), and is treated as a movie.
        kind = form2 ? ((submode & 0x04u) != 0 ? kGrimDiscXaAudio : kGrimDiscMovies) : kGrimDiscFiles;
      } else if (mode == 1) {
        begin = 16;
        len = kUser;
      } else {
        return 0;
      }
    } else {
      return 0;
    }
    // STR movie frames (Form 1 or 2) start with a 32-byte header (0x0160, 0x8001, frame
    // number, sizes): the picture data after it is the movie, the header stays intact so
    // the player keeps going while the picture breaks.
    const u8 *user = raw + begin;
    if (kind != kGrimDiscXaAudio && user[0] == 0x60 && user[1] == 0x01 && user[2] == 0x01 && user[3] == 0x80) {
      kind = kGrimDiscMovies;
      // The first sector of a frame (index 0) also starts the bitstream with its own
      // 8-byte header (size, 0x3800, quantiser, version); decoders give up without it.
      const size_t keep = (user[4] | (user[5] << 8)) == 0 ? 40u : 32u;
      begin += keep;
      len -= keep;
    }
    if (layout.is_boot(lba)) {
      kind = kGrimDiscBootCode;
    }
    if ((cfg.targets & kind) == 0) {
      return 0;
    }
    if (kind == kGrimDiscFiles && cfg.skip_risky && grim_disc_sector_is_risky(raw + begin, len)) {
      return 0;
    }
  }

  u8 *d = raw + begin;
  // One stream per sector: the same sector is always hit the same way.
  GrimRng rng{grim_mix64(cfg.seed ^ (static_cast<u64>(static_cast<u32>(lba)) * 0x9E3779B97F4A7C15ull))};
  size_t changed = 0;
  const auto hit = [&](size_t off) {
    const u32 random = static_cast<u32>(rng.next());
    const u8 piped = cfg.engine.op == GrimByteOp::Pipe ? d[grim_byte_pipe_source(off, cfg.engine.offset, len)] : 0;
    const u8 now = grim_byte_apply(cfg.engine, d[off], random, piped);
    changed += now != d[off] ? 1u : 0u;
    d[off] = now;
  };
  if (cfg.every > 0) {
    const u64 global = static_cast<u64>(static_cast<u32>(lba)) * len;
    for (size_t off = static_cast<size_t>((cfg.every - global % cfg.every) % cfg.every); off < len;
         off += cfg.every) {
      hit(off);
    }
    return changed;
  }
  const double expected = std::max(0.0, static_cast<double>(cfg.percent)) / 100.0 * static_cast<double>(len);
  size_t n = static_cast<size_t>(std::floor(expected));
  if (static_cast<double>(rng.unit_q10()) / 1024.0 < expected - std::floor(expected)) {
    ++n;
  }
  for (size_t k = 0; k < n; ++k) {
    hit(static_cast<size_t>(rng.next() % len));
  }
  return changed;
}
