#include "grim_map.h"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

namespace {
constexpr u64 kFnvOffset = 14695981039346656037ull;
constexpr u64 kFnvPrime = 1099511628211ull;
constexpr u32 kIoSpuBegin = 0x1F801C00u, kIoSpuEnd = 0x1F802000u;

u64 fnv_bytes(u64 h, const void *data, size_t n) {
  const u8 *p = static_cast<const u8 *>(data);
  for (size_t i = 0; i < n; ++i) {
    h = (h ^ p[i]) * kFnvPrime;
  }
  return h;
}
u64 fnv_u64(u64 h, u64 v) {
  u8 b[8];
  for (int i = 0; i < 8; ++i) {
    b[i] = static_cast<u8>(v >> (i * 8));
  }
  return fnv_bytes(h, b, 8);
}
double cycles_to_ms(u64 c) {
  return static_cast<double>(c) * 1000.0 / static_cast<double>(psx::CPU_CLOCK_HZ);
}
} // namespace

const char *grim_word_class_name(GrimWordClass c) {
  switch (c) {
  case GrimWordClass::Code: return "code";
  case GrimWordClass::Data: return "data";
  case GrimWordClass::Unknown: return "unknown";
  default: return "unused";
  }
}

std::string grim_consumer_name(u8 mask) {
  if (mask == 0) {
    return "cpu";
  }
  std::string s;
  auto add = [&](u8 bit, const char *n) {
    if (mask & bit) {
      s += s.empty() ? "" : "+";
      s += n;
    }
  };
  add(kGrimConsumerSpu, "spu");
  add(kGrimConsumerGpu, "gpu");
  add(kGrimConsumerGte, "gte");
  add(kGrimConsumerMdec, "mdec");
  return s;
}

const char *grim_spu_sample_use_kind_name(GrimSpuSampleUseKind kind) {
  switch (kind) {
  case GrimSpuSampleUseKind::StartWrite: return "start_write";
  case GrimSpuSampleUseKind::RepeatWrite: return "repeat_write";
  case GrimSpuSampleUseKind::KeyOnStart: return "key_on_start";
  case GrimSpuSampleUseKind::KeyOnRepeat: return "key_on_repeat";
  }
  return "unknown";
}

u32 GrimBootMap::count(GrimWordClass c) const {
  u32 n = 0;
  for (const GrimMapWord &w : words) {
    n += w.cls == c ? 1u : 0u;
  }
  return n;
}

u32 GrimBootMap::dormant_words() const {
  u32 n = 0;
  for (const GrimMapWord &w : words) {
    n += w.cls == GrimWordClass::Unused &&
                 (w.flags & (kGrimReadDirect | kGrimReadViaRam)) != 0 ? 1u : 0u;
  }
  return n;
}

std::vector<std::pair<u32, u32>> grim_map_dormant_regions(const GrimBootMap &m) {
  std::vector<std::pair<u32, u32>> regions;
  u32 start = kGrimNoRomOffset;
  for (u32 i = 0; i <= m.rom_words(); ++i) {
    const bool dormant = i < m.rom_words() && m.words[i].cls == GrimWordClass::Unused &&
                         (m.words[i].flags & (kGrimReadDirect | kGrimReadViaRam)) != 0;
    if (dormant && start == kGrimNoRomOffset) {
      start = i * 4u;
    } else if (!dormant && start != kGrimNoRomOffset) {
      regions.emplace_back(start, i * 4u);
      start = kGrimNoRomOffset;
    }
  }
  return regions;
}

u64 GrimBootMap::hash() const {
  u64 h = kFnvOffset;
  h = fnv_u64(h, bios_hash);
  h = fnv_bytes(h, scenario.data(), scenario.size());
  h = fnv_u64(h, frames);
  h = fnv_u64(h, cycles);
  h = fnv_u64(h, last_new_code_cycle);
  h = fnv_u64(h, ram_exec_words);
  h = fnv_u64(h, ram_exec_known);
  for (const GrimMapWord &w : words) {
    h = fnv_u64(h, w.first_exec);
    h = fnv_u64(h, w.first_read);
    h = fnv_u64(h, (u64{static_cast<u8>(w.cls)} << 16) | (u64{w.consumer} << 8) | w.flags);
  }
  // Keep hashes of old Phase 3 maps unchanged. The optional Phase 4 event
  // extension is hashed when present, including unresolved events.
  if (!spu_sample_uses.empty()) {
    h = fnv_u64(h, spu_sample_uses.size());
    for (const GrimSpuSampleUse &u : spu_sample_uses) {
      h = fnv_u64(h, u.cycle);
      h = fnv_u64(h, u.spu_address);
      h = fnv_u64(h, u.rom_offset);
      h = fnv_u64(h, (u64{u.voice} << 8u) | static_cast<u8>(u.kind));
    }
  }
  return h;
}

std::vector<GrimMapRegion> grim_map_regions(const GrimBootMap &m) {
  std::vector<GrimMapRegion> out;
  const u32 n = m.rom_words();
  for (u32 i = 0; i < n; ++i) {
    const GrimMapWord &w = m.words[i];
    if (out.empty() || out.back().cls != w.cls || out.back().consumer != w.consumer ||
        out.back().end_word != i) {
      GrimMapRegion r;
      r.start_word = i;
      r.end_word = i;
      r.cls = w.cls;
      r.consumer = w.consumer;
      out.push_back(r);
    }
    GrimMapRegion &r = out.back();
    r.end_word = i + 1;
    const u64 touch = w.cls == GrimWordClass::Code ? w.first_exec
                                                   : std::min(w.first_exec, w.first_read);
    r.first_touch = std::min(r.first_touch, touch);
    r.via_ram += (w.flags & (kGrimExecViaRam | kGrimReadViaRam)) != 0 ? 1u : 0u;
  }
  return out;
}

// ---- files ----------------------------------------------------------------------------

namespace {
void put_u32(std::string &s, u32 v) {
  for (int i = 0; i < 4; ++i) {
    s.push_back(static_cast<char>(v >> (i * 8)));
  }
}
void put_u64(std::string &s, u64 v) {
  for (int i = 0; i < 8; ++i) {
    s.push_back(static_cast<char>(v >> (i * 8)));
  }
}
u64 get_le(const u8 *p, int n) {
  u64 v = 0;
  for (int i = 0; i < n; ++i) {
    v |= u64{p[i]} << (i * 8);
  }
  return v;
}
} // namespace

bool grim_map_save(const GrimBootMap &m, const std::string &path, std::string &err) {
  const std::string words_path = path + ".words";
  std::string bin = "GRMW";
  put_u32(bin, 1);
  put_u32(bin, m.rom_words());
  for (const GrimMapWord &w : m.words) {
    put_u64(bin, w.first_exec);
    put_u64(bin, w.first_read);
    bin.push_back(static_cast<char>(w.cls));
    bin.push_back(static_cast<char>(w.consumer));
    bin.push_back(static_cast<char>(w.flags));
    bin.push_back(0);
  }
  char buf[512];
  std::string js;
  std::snprintf(buf, sizeof(buf),
                "{\"format\":1,\"bios_hash\":\"0x%016llX\",\"bios_words\":%u,"
                "\"scenario\":\"%s\",\"frames\":%u,\"cycles\":%llu,"
                "\"last_new_code_cycle\":%llu,\n",
                static_cast<unsigned long long>(m.bios_hash), m.rom_words(), m.scenario.c_str(),
                m.frames, static_cast<unsigned long long>(m.cycles),
                static_cast<unsigned long long>(m.last_new_code_cycle));
  js += buf;
  std::snprintf(buf, sizeof(buf),
                "\"provenance\":{\"ram_exec_words\":%u,\"with_rom_origin\":%u,\"permille\":%u},\n"
                "\"counts\":{\"code\":%u,\"data\":%u,\"unused\":%u,\"unknown\":%u},\n"
                "\"regions\":[",
                m.ram_exec_words, m.ram_exec_known, m.provenance_permille(),
                m.count(GrimWordClass::Code), m.count(GrimWordClass::Data),
                m.count(GrimWordClass::Unused), m.count(GrimWordClass::Unknown));
  js += buf;
  bool first = true;
  for (const GrimMapRegion &r : grim_map_regions(m)) {
    std::snprintf(buf, sizeof(buf),
                  "%s\n{\"start\":\"0x%05X\",\"end\":\"0x%05X\",\"class\":\"%s\","
                  "\"consumer\":\"%s\",\"first_touch\":%lld,\"via_ram\":%u}",
                  first ? "" : ",", r.start_word * 4u, r.end_word * 4u,
                  grim_word_class_name(r.cls), grim_consumer_name(r.consumer).c_str(),
                  r.first_touch == kGrimNever ? -1ll : static_cast<long long>(r.first_touch),
                  r.via_ram);
    js += buf;
    first = false;
  }
  js += "\n]";
  if (!m.spu_sample_uses.empty()) {
    js += ",\n\"spu_sample_uses\":[";
    bool first_use = true;
    for (const GrimSpuSampleUse &u : m.spu_sample_uses) {
      std::snprintf(buf, sizeof(buf),
                    "%s\n{\"cycle\":%llu,\"voice\":%u,\"kind\":\"%s\","
                    "\"spu_address\":%u,\"rom_offset\":%lld}",
                    first_use ? "" : ",", static_cast<unsigned long long>(u.cycle),
                    static_cast<unsigned>(u.voice), grim_spu_sample_use_kind_name(u.kind),
                    u.spu_address, u.rom_offset == kGrimNoRomOffset
                                       ? -1ll : static_cast<long long>(u.rom_offset));
      js += buf;
      first_use = false;
    }
    js += "\n]";
  }
  js += "}\n";
  std::ofstream jo(path, std::ios::binary | std::ios::trunc);
  jo << js;
  std::ofstream bo(words_path, std::ios::binary | std::ios::trunc);
  bo.write(bin.data(), static_cast<std::streamsize>(bin.size()));
  if (!jo || !bo) {
    err = "cannot write " + path;
    return false;
  }
  return true;
}

bool grim_map_load(const std::string &path, GrimBootMap &out, std::string &err) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    err = "cannot open " + path;
    return false;
  }
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const nlohmann::json j = nlohmann::json::parse(text, nullptr, false);
  if (j.is_discarded() || !j.is_object() || !j.contains("format") ||
      !j.contains("bios_hash") || !j.contains("provenance")) {
    err = path + ": not a Grim map";
    return false;
  }
  GrimBootMap m;
  try {
    m.bios_hash = std::stoull(j["bios_hash"].get<std::string>(), nullptr, 16);
    m.scenario = j["scenario"].get<std::string>();
    m.frames = j["frames"].get<u32>();
    m.cycles = j["cycles"].get<u64>();
    m.last_new_code_cycle = j["last_new_code_cycle"].get<u64>();
    m.ram_exec_words = j["provenance"]["ram_exec_words"].get<u32>();
    m.ram_exec_known = j["provenance"]["with_rom_origin"].get<u32>();
    if (j.contains("spu_sample_uses")) {
      if (!j["spu_sample_uses"].is_array()) {
        throw std::runtime_error("SPU sample uses must be an array");
      }
      for (const auto &entry : j["spu_sample_uses"]) {
        GrimSpuSampleUse u;
        u.cycle = entry.at("cycle").get<u64>();
        const u32 voice = entry.at("voice").get<u32>();
        u.spu_address = entry.at("spu_address").get<u32>();
        const s64 origin = entry.at("rom_offset").get<s64>();
        const std::string kind = entry.at("kind").get<std::string>();
        if (voice >= 24u || u.spu_address >= 512u * 1024u || origin < -1 ||
            origin >= static_cast<s64>(j.at("bios_words").get<u32>()) * 4) {
          throw std::runtime_error("out-of-range SPU sample use");
        }
        u.voice = static_cast<u8>(voice);
        u.rom_offset = origin < 0 ? kGrimNoRomOffset : static_cast<u32>(origin);
        bool recognized = false;
        for (u32 i = 0; i < 4; ++i) {
          if (kind == grim_spu_sample_use_kind_name(static_cast<GrimSpuSampleUseKind>(i))) {
            u.kind = static_cast<GrimSpuSampleUseKind>(i);
            recognized = true;
            break;
          }
        }
        if (!recognized) {
          throw std::runtime_error("unknown SPU sample use kind");
        }
        m.spu_sample_uses.push_back(u);
      }
    }
  } catch (const std::exception &e) {
    err = path + ": bad header (" + e.what() + ")";
    return false;
  }
  std::ifstream bin(path + ".words", std::ios::binary);
  if (!bin.is_open()) {
    err = "cannot open " + path + ".words (keep the two files together)";
    return false;
  }
  const std::string data((std::istreambuf_iterator<char>(bin)), std::istreambuf_iterator<char>());
  if (data.size() < 12 || data.compare(0, 4, "GRMW") != 0) {
    err = "bad words file";
    return false;
  }
  const u8 *p = reinterpret_cast<const u8 *>(data.data());
  const u32 n = static_cast<u32>(get_le(p + 8, 4));
  if (get_le(p + 4, 4) != 1 || data.size() != 12u + size_t{n} * 20u) {
    err = "bad words file size";
    return false;
  }
  m.words.resize(n);
  for (u32 i = 0; i < n; ++i) {
    const u8 *q = p + 12 + size_t{i} * 20u;
    m.words[i].first_exec = get_le(q, 8);
    m.words[i].first_read = get_le(q + 8, 8);
    m.words[i].cls = static_cast<GrimWordClass>(q[16]);
    m.words[i].consumer = q[17];
    m.words[i].flags = q[18];
  }
  out = std::move(m);
  return true;
}

std::string grim_map_summary(const GrimBootMap &m, u32 min_bytes) {
  std::string s;
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "GRIM_MAP bios=0x%016llX scenario=%s frames=%u emulated=%.2fs "
                "last_new_code=%.1fms\n",
                static_cast<unsigned long long>(m.bios_hash), m.scenario.c_str(), m.frames,
                cycles_to_ms(m.cycles) / 1000.0, cycles_to_ms(m.last_new_code_cycle));
  s += buf;
  if (!m.spu_sample_uses.empty()) {
    u32 resolved = 0, key_on_resolved = 0, key_on_total = 0;
    for (const GrimSpuSampleUse &u : m.spu_sample_uses) {
      const bool known = u.rom_offset != kGrimNoRomOffset;
      resolved += known ? 1u : 0u;
      if (u.kind == GrimSpuSampleUseKind::KeyOnStart) {
        ++key_on_total;
        key_on_resolved += known ? 1u : 0u;
      }
    }
    std::snprintf(buf, sizeof(buf),
                  "SPU address provenance: %u/%u resolved events; %u/%u key-on starts "
                  "resolved (unresolved register writes are kept)\n",
                  resolved, static_cast<u32>(m.spu_sample_uses.size()),
                  key_on_resolved, key_on_total);
    s += buf;
  }
  const u32 total = m.rom_words();
  auto pct = [&](u32 n) { return total ? 100.0 * n / total : 0.0; };
  std::snprintf(buf, sizeof(buf),
                "words: code=%u (%.1f%%) data=%u (%.1f%%) unused=%u (%.1f%%) unknown=%u (%.1f%%) "
                "of %u\n",
                m.count(GrimWordClass::Code), pct(m.count(GrimWordClass::Code)),
                m.count(GrimWordClass::Data), pct(m.count(GrimWordClass::Data)),
                m.count(GrimWordClass::Unused), pct(m.count(GrimWordClass::Unused)),
                m.count(GrimWordClass::Unknown), pct(m.count(GrimWordClass::Unknown)), total);
  s += buf;
  const u32 copied_only = m.dormant_words();
  std::snprintf(buf, sizeof(buf),
                "  unused split: untouched=%u words (%.1fKB); dormant=%u words (%.1fKB) "
                "read/copied but never consumed. Serialized classes stay compatible.\n",
                m.count(GrimWordClass::Unused) - copied_only,
                (m.count(GrimWordClass::Unused) - copied_only) * 4.0 / 1024.0,
                copied_only, copied_only * 4.0 / 1024.0);
  s += buf;
  std::snprintf(buf, sizeof(buf),
                "provenance: %u of %u distinct executed RAM words have a ROM origin (%.1f%%)\n",
                m.ram_exec_known, m.ram_exec_words, m.provenance_permille() / 10.0);
  s += buf;
  s += "\ndormant ROM ranges (read/copied, never consumed):\n";
  for (const auto &range : grim_map_dormant_regions(m)) {
    if (range.second - range.first < min_bytes) continue;
    std::snprintf(buf, sizeof(buf), "  0x%05X-0x%05X %.1fKB\n", range.first, range.second,
                  (range.second - range.first) / 1024.0);
    s += buf;
  }
  // Overview per 32 KB of ROM.
  s += "\nper 32 KB block (words): rom_range          code    data  unused unknown  code_via_ram\n";
  for (u32 b = 0; b * 8192u < total; ++b) {
    u32 n[4] = {0, 0, 0, 0}, via = 0;
    for (u32 i = b * 8192u; i < std::min(total, (b + 1) * 8192u); ++i) {
      ++n[static_cast<size_t>(m.words[i].cls)];
      via += m.words[i].cls == GrimWordClass::Code && (m.words[i].flags & kGrimExecViaRam) ? 1u : 0u;
    }
    std::snprintf(buf, sizeof(buf), "                         0x%05X-0x%05X %7u %7u %7u %7u  %7u\n",
                  b * 0x8000u, (b + 1) * 0x8000u, n[1], n[2], n[0], n[3], via);
    s += buf;
  }
  s += "\nrom_range           rom_addr_range           class    size      consumer   "
       "first_touch   via_ram\n";
  u32 omitted = 0;
  for (const GrimMapRegion &r : grim_map_regions(m)) {
    const u32 words = r.end_word - r.start_word;
    if (words * 4u < min_bytes) {
      ++omitted;
      continue;
    }
    char touch[32];
    if (r.first_touch == kGrimNever) {
      std::snprintf(touch, sizeof(touch), "-");
    } else {
      std::snprintf(touch, sizeof(touch), "%.2fms", cycles_to_ms(r.first_touch));
    }
    std::snprintf(buf, sizeof(buf),
                  "0x%05X-0x%05X  0xBFC%05X-0xBFC%05X  %-8s %6.1fKB  %-9s  %-12s %5.1f%%\n",
                  r.start_word * 4u, r.end_word * 4u, r.start_word * 4u, r.end_word * 4u,
                  grim_word_class_name(r.cls), words * 4.0 / 1024.0,
                  grim_consumer_name(r.consumer).c_str(), touch,
                  words ? 100.0 * r.via_ram / words : 0.0);
    s += buf;
  }
  std::snprintf(buf, sizeof(buf), "(%u regions under %u bytes are not listed; --min-bytes 0 lists all)\n",
                omitted, min_bytes);
  s += buf;
  return s;
}

// ---- mapper --------------------------------------------------------------------------------

GrimWordClass grim_classify_word(u8 flags, u8 consumer) {
  const bool executed = (flags & (kGrimExecDirect | kGrimExecViaRam)) != 0;
  const bool read = (flags & (kGrimReadDirect | kGrimReadViaRam)) != 0;
  if (executed) {
    // Code that also went to a peripheral is a conflict, not code.
    const bool to_peripheral =
        (consumer & (kGrimConsumerSpu | kGrimConsumerGpu | kGrimConsumerMdec)) != 0;
    return to_peripheral ? GrimWordClass::Unknown : GrimWordClass::Code;
  }
  if (read && ((flags & kGrimUsed) != 0 || consumer != 0)) {
    return GrimWordClass::Data;
  }
  if (read) {
    return GrimWordClass::Unused; // loaded only to be moved (a copy loop): never used
  }
  return (flags & kGrimPartial) != 0 ? GrimWordClass::Unknown : GrimWordClass::Unused;
}

GrimBootMap grim_map_merge(const GrimBootMap &a, const GrimBootMap &b) {
  GrimBootMap m = a;
  m.scenario = a.scenario + "+" + b.scenario;
  m.frames = std::max(a.frames, b.frames);
  m.cycles = std::max(a.cycles, b.cycles);
  m.ram_exec_words = std::max(a.ram_exec_words, b.ram_exec_words);
  m.ram_exec_known = std::max(a.ram_exec_known, b.ram_exec_known);
  m.last_new_code_cycle = 0;
  for (size_t i = 0; i < m.words.size() && i < b.words.size(); ++i) {
    GrimMapWord &w = m.words[i];
    const GrimMapWord &o = b.words[i];
    w.first_exec = std::min(w.first_exec, o.first_exec);
    w.first_read = std::min(w.first_read, o.first_read);
    w.consumer |= o.consumer;
    w.flags |= o.flags;
    w.cls = grim_classify_word(w.flags, w.consumer);
    if (w.cls == GrimWordClass::Code) {
      m.last_new_code_cycle = std::max(m.last_new_code_cycle, w.first_exec);
    }
  }
  m.spu_sample_uses.insert(m.spu_sample_uses.end(), b.spu_sample_uses.begin(),
                           b.spu_sample_uses.end());
  auto less = [](const GrimSpuSampleUse &x, const GrimSpuSampleUse &y) {
    if (x.cycle != y.cycle) return x.cycle < y.cycle;
    if (x.voice != y.voice) return x.voice < y.voice;
    if (x.kind != y.kind) return x.kind < y.kind;
    if (x.spu_address != y.spu_address) return x.spu_address < y.spu_address;
    return x.rom_offset < y.rom_offset;
  };
  std::sort(m.spu_sample_uses.begin(), m.spu_sample_uses.end(), less);
  m.spu_sample_uses.erase(std::unique(m.spu_sample_uses.begin(), m.spu_sample_uses.end(),
                                      [&](const GrimSpuSampleUse &x, const GrimSpuSampleUse &y) {
                                        return !less(x, y) && !less(y, x);
                                      }), m.spu_sample_uses.end());
  return m;
}

GrimBootMapper::GrimBootMapper(u32 rom_bytes)
    : rom_bytes_(rom_bytes), ram_tag_(kRamBytes, 0u), spu_tag_(kSpuRamBytes, 0u),
      first_exec_(rom_bytes / 4u, kGrimNever),
      first_read_(rom_bytes / 4u, kGrimNever), consumer_(rom_bytes / 4u, 0),
      flags_(rom_bytes / 4u, 0), ram_exec_(kRamBytes / 4u, 0) {}

GrimBootMapper::Region GrimBootMapper::decode(u32 addr, u32 &idx) const {
  if (addr >= 0xC0000000u) {
    return kNone;
  }
  const u32 phys = addr & 0x1FFFFFFFu;
  if (phys < 0x00800000u) {
    idx = phys & (kRamBytes - 1u);
    return kRam;
  }
  if (phys >= psx::SCRATCHPAD_BASE && phys < psx::SCRATCHPAD_BASE + 0x400u) {
    idx = phys - psx::SCRATCHPAD_BASE;
    return kScratch;
  }
  if (phys >= 0x1F801000u && phys < 0x1F803000u) {
    idx = phys;
    return kIo;
  }
  if (phys >= psx::BIOS_BASE && phys - psx::BIOS_BASE < rom_bytes_) {
    idx = phys - psx::BIOS_BASE;
    return kRom;
  }
  return kNone;
}

u32 GrimBootMapper::get_tag(Region r, u32 idx) const {
  switch (r) {
  case kRam: return ram_tag_[idx & (kRamBytes - 1u)];
  case kScratch: return idx < scratch_tag_.size() ? scratch_tag_[idx] : 0u;
  case kRom: return idx < rom_bytes_ ? idx + 1u : 0u;
  default: return 0u;
  }
}

void GrimBootMapper::set_tag(Region r, u32 idx, u32 tag) {
  if (r == kRam) {
    ram_tag_[idx & (kRamBytes - 1u)] = tag;
  } else if (r == kScratch && idx < scratch_tag_.size()) {
    scratch_tag_[idx] = tag;
  }
}

void GrimBootMapper::begin_instruction(u64 cycle, u32 pc, u32 instr, const u32 *gpr, u32 sr) {
  cur_cycle_ = cycle;
  cur_pc_ = pc;
  cur_instr_ = instr;
  cur_sr_ = sr;
  cur_ea_ = gpr[(instr >> 21) & 31u] + static_cast<u32>(static_cast<s32>(static_cast<s16>(instr)));
  cur_value_ = gpr[(instr >> 16) & 31u];
}

void GrimBootMapper::note_origin(const u32 *tags, size_t n, u8 consumer, bool via_ram, u64 cycle) {
  u32 last_word = ~0u;
  for (size_t i = 0; i < n; ++i) {
    if (tags[i] == 0) {
      continue;
    }
    const u32 word = (tags[i] - 1u) >> 2;
    if (word >= first_read_.size() || word == last_word) {
      continue;
    }
    last_word = word;
    first_read_[word] = std::min(first_read_[word], cycle);
    consumer_[word] |= consumer;
    flags_[word] |= via_ram ? kGrimReadViaRam : kGrimReadDirect;
  }
}

void GrimBootMapper::note_use(const u32 *tags) {
  u32 last_word = ~0u;
  for (int i = 0; i < 4; ++i) {
    if (tags[i] == 0) {
      continue;
    }
    const u32 word = (tags[i] - 1u) >> 2;
    if (word < flags_.size() && word != last_word) {
      last_word = word;
      flags_[word] |= kGrimUsed;
    }
  }
}

void GrimBootMapper::note_exec(u32 pc) {
  u32 idx = 0;
  const Region r = decode(pc, idx);
  if (r == kRom) {
    const u32 w = idx >> 2;
    first_exec_[w] = std::min(first_exec_[w], cur_cycle_);
    flags_[w] |= kGrimExecDirect;
  } else if (r == kRam) {
    const u32 base = idx & ~3u;
    const u32 t0 = ram_tag_[base], t1 = ram_tag_[base + 1], t2 = ram_tag_[base + 2],
              t3 = ram_tag_[base + 3];
    u8 &seen = ram_exec_[base >> 2];
    if ((seen & 1u) == 0) {
      seen |= 1u;
      ++ram_exec_words_;
    }
    if (t0 != 0 && t1 == t0 + 1u && t2 == t0 + 2u && t3 == t0 + 3u && ((t0 - 1u) & 3u) == 0) {
      const u32 w = (t0 - 1u) >> 2;
      if (w < first_exec_.size()) {
        if ((seen & 2u) == 0) {
          seen |= 2u;
          ++ram_exec_known_;
        }
        first_exec_[w] = std::min(first_exec_[w], cur_cycle_);
        flags_[w] |= kGrimExecViaRam;
      }
    } else if ((t0 | t1 | t2 | t3) != 0) {
      const u32 tags[4] = {t0, t1, t2, t3};
      u32 last_word = ~0u;
      for (u32 t : tags) {
        if (t != 0 && ((t - 1u) >> 2) != last_word && ((t - 1u) >> 2) < flags_.size()) {
          last_word = (t - 1u) >> 2;
          flags_[last_word] |= kGrimPartial;
        }
      }
    }
  }
}

void GrimBootMapper::set_reg_tags(u32 reg, const u32 tags[4]) {
  if (reg != 0) {
    for (int i = 0; i < 4; ++i) {
      reg_tags_[reg][i] = tags[i];
    }
  }
}

u32 GrimBootMapper::spu_rom_origin(u32 addr) const {
  const u32 first = spu_tag_[addr & (kSpuRamBytes - 1u)];
  if (first == 0 || first - 1u + 16u > rom_bytes_) {
    return kGrimNoRomOffset;
  }
  // Resolve only a whole block, so one tagged byte in a buffer cannot falsely
  // identify a mixed or partially overwritten sample.
  for (u32 i = 1; i < 16u; ++i) {
    if (spu_tag_[(addr + i) & (kSpuRamBytes - 1u)] != first + i) {
      return kGrimNoRomOffset;
    }
  }
  return first - 1u;
}

void GrimBootMapper::note_spu_sample_use(u32 voice, u32 addr, GrimSpuSampleUseKind kind) {
  GrimSpuSampleUse u;
  u.cycle = cur_cycle_;
  u.voice = static_cast<u8>(voice);
  u.spu_address = addr & (kSpuRamBytes - 1u);
  u.rom_offset = spu_rom_origin(u.spu_address);
  u.kind = kind;
  spu_sample_uses_.push_back(u);
}

void GrimBootMapper::note_spu_write16(u32 offset, u16 value, const u32 *tags) {
  if (offset >= 0x400u) {
    return;
  }
  // Same register/transfer semantics as Spu::write16. No SPU hooks are needed:
  // this code runs only after an instruction in the existing discovery loop.
  spu_regs_[offset / 2u] = offset >= 0x188u && offset <= 0x18Eu ? 0u : value;
  if (offset == 0x1A6u) {
    spu_transfer_addr_ = static_cast<u32>(value) * 8u;
  } else if (offset == 0x1A8u) {
    spu_tag_[spu_transfer_addr_ & (kSpuRamBytes - 1u)] = tags[0];
    spu_tag_[(spu_transfer_addr_ + 1u) & (kSpuRamBytes - 1u)] = tags[1];
    spu_transfer_addr_ = (spu_transfer_addr_ + 2u) & (kSpuRamBytes - 1u);
  } else if (offset < 0x180u && (offset & 15u) == 6u) {
    note_spu_sample_use(offset / 16u, static_cast<u32>(value) * 8u,
                         GrimSpuSampleUseKind::StartWrite);
  } else if (offset < 0x180u && (offset & 15u) == 14u) {
    note_spu_sample_use(offset / 16u, static_cast<u32>(value) * 8u,
                         GrimSpuSampleUseKind::RepeatWrite);
  } else if (offset == 0x188u || offset == 0x18Au) {
    // These are key-on requests (time of the CPU write, before the SPU's next
    // sample-clock latch), and not new hooks into normal voice playback.
    if ((spu_regs_[0x1AAu / 2u] & 0x8000u) == 0u) {
      return;
    }
    const u32 base = offset == 0x188u ? 0u : 16u;
    const u32 mask = offset == 0x188u ? value : value & 255u;
    for (u32 bit = 0; bit < 16u && base + bit < 24u; ++bit) {
      if ((mask & (1u << bit)) == 0u) {
        continue;
      }
      const u32 voice = base + bit;
      const u32 start = static_cast<u32>(spu_regs_[(voice * 16u + 6u) / 2u]) * 8u;
      u32 repeat = static_cast<u32>(spu_regs_[(voice * 16u + 14u) / 2u]) * 8u;
      if (repeat == 0u) {
        repeat = start; // Spu::key_on_voice's zero-repeat fallback
      }
      note_spu_sample_use(voice, start, GrimSpuSampleUseKind::KeyOnStart);
      note_spu_sample_use(voice, repeat, GrimSpuSampleUseKind::KeyOnRepeat);
    }
  }
}

// Byte tags a load instruction brings in (out[lane], lane 0 = least significant).
// ok = false when the access is misaligned or outside tracked memory.
void GrimBootMapper::load_tags(u32 op, u32 ea, u32 rt, u32 out[4], bool &ok) {
  u32 idx = 0;
  const Region r = decode(ea, idx);
  ok = r == kRam || r == kScratch || r == kRom;
  for (int i = 0; i < 4; ++i) {
    out[i] = 0;
  }
  if (!ok) {
    return;
  }
  auto byte = [&](u32 i) { return get_tag(r, idx + i); };
  switch (op) {
  case 0x20: case 0x24: // LB LBU
    out[0] = byte(0);
    break;
  case 0x21: case 0x25: // LH LHU
    ok = (ea & 1u) == 0;
    out[0] = byte(0);
    out[1] = byte(1);
    break;
  case 0x23: case 0x32: // LW LWC2
    ok = (ea & 3u) == 0;
    for (u32 i = 0; i < 4; ++i) {
      out[i] = byte(i);
    }
    break;
  case 0x22: { // LWL: bytes [base .. ea] land in lanes [3-a .. 3]
    const u32 a = ea & 3u;
    for (u32 i = 0; i < 4; ++i) {
      out[i] = reg_tags_[rt][i];
    }
    for (u32 i = 0; i <= a; ++i) {
      out[3 - a + i] = get_tag(r, (idx & ~3u) + i);
    }
    break;
  }
  case 0x26: { // LWR: bytes [ea .. end of word] land in lanes [0 .. 3-a]
    const u32 a = ea & 3u;
    for (u32 i = 0; i < 4; ++i) {
      out[i] = reg_tags_[rt][i];
    }
    for (u32 j = 0; j + a <= 3u; ++j) {
      out[j] = byte(j);
    }
    break;
  }
  default:
    ok = false;
  }
}

void GrimBootMapper::commit_instruction() {
  const u32 instr = cur_instr_;
  const u32 op = instr >> 26;
  const u32 rs = (instr >> 21) & 31u, rt = (instr >> 16) & 31u, rd = (instr >> 11) & 31u;
  const u32 funct = instr & 63u;
  const u32 imm = instr & 0xFFFFu;
  note_exec(cur_pc_);

  int dest = -1;
  u32 dtags[4] = {0, 0, 0, 0}; // cleared unless the operation is an exact copy
  bool is_load = false;
  bool is_copy = false; // an exact register copy: the value is moved, not used
  auto copy_from = [&](u32 reg) {
    is_copy = true;
    for (int i = 0; i < 4; ++i) {
      dtags[i] = reg_tags_[reg][i];
    }
  };

  switch (op) {
  case 0:
    switch (funct) {
    case 0: case 2: case 3: case 4: case 6: case 7: case 0x10: case 0x12: case 0x20:
    case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
    case 0x2A: case 0x2B: case 9:
      dest = static_cast<int>(rd);
      if (funct == 0 && instr != 0 && ((instr >> 6) & 31u) == 0) {
        copy_from(rt); // sll rd, rt, 0
      } else if (funct == 0x20 || funct == 0x21 || funct == 0x25 || funct == 0x26) {
        if (rt == 0) {
          copy_from(rs); // move rd, rs
        } else if (rs == 0) {
          copy_from(rt);
        }
      } else if (funct == 0x23 && rt == 0) {
        copy_from(rs);
      }
      break;
    default:
      break;
    }
    break;
  case 1:
    if ((rt & 0x1E) == 0x10) {
      dest = 31;
    }
    break;
  case 3:
    dest = 31;
    break;
  case 8: case 9: case 0xA: case 0xB: case 0xC: case 0xD: case 0xE: case 0xF:
    dest = static_cast<int>(rt);
    if (imm == 0 && (op == 8 || op == 9 || op == 0xD || op == 0xE)) {
      copy_from(rs); // addiu/ori rt, rs, 0
    }
    break;
  case 0x10:
    if (rs == 0) {
      dest = static_cast<int>(rt); // MFC0
    }
    break;
  case 0x12:
    if (rs == 0 || rs == 2) {
      dest = static_cast<int>(rt); // MFC2 / CFC2
    } else if (rs == 4 || rs == 6) {
      note_origin(reg_tags_[rt].data(), 4, kGrimConsumerGte, true, cur_cycle_); // MTC2 / CTC2
    }
    break;
  case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26:
  case 0x32: {
    u32 tags[4];
    bool ok = false;
    load_tags(op, cur_ea_, rt, tags, ok);
    if (ok) {
      u32 idx = 0;
      const Region r = decode(cur_ea_, idx);
      // Only the bytes this load really brings in count as read.
      u32 used[4] = {0, 0, 0, 0};
      size_t nused = 0;
      switch (op) {
      case 0x20: case 0x24: used[nused++] = tags[0]; break;
      case 0x21: case 0x25: used[nused++] = tags[0]; used[nused++] = tags[1]; break;
      case 0x22: for (u32 i = 3 - (cur_ea_ & 3u); i < 4; ++i) used[nused++] = tags[i]; break;
      case 0x26: for (u32 i = 0; i + (cur_ea_ & 3u) <= 3u; ++i) used[nused++] = tags[i]; break;
      default: for (u32 i = 0; i < 4; ++i) used[nused++] = tags[i]; break;
      }
      note_origin(used, nused, op == 0x32 ? kGrimConsumerGte : 0, r != kRom, cur_cycle_);
      for (size_t i = 0; i < nused; ++i) {
        stats_.tagged_load_bytes_by_op[op] += used[i] != 0 ? 1u : 0u;
        if (r == kRom) {
          ++stats_.rom_load_pcs[cur_pc_];
        }
      }
      if (op != 0x32) {
        is_load = true;
        dest = static_cast<int>(rt);
        for (int i = 0; i < 4; ++i) {
          dtags[i] = tags[i];
        }
      }
    } else if (op != 0x32) {
      dest = static_cast<int>(rt); // faulting or I/O load: result carries no tag
      is_load = true;
    }
    break;
  }
  case 0x28: case 0x29: case 0x2A: case 0x2B: case 0x2E: case 0x3A: {
    u32 idx = 0;
    const Region r = decode(cur_ea_, idx);
    u32 src[4] = {0, 0, 0, 0};
    if (op != 0x3A) {
      for (int i = 0; i < 4; ++i) {
        src[i] = reg_tags_[rt][i];
      }
    }
    // (memory byte offset from the aligned/base index, lane) pairs written.
    struct Put { u32 off; u32 lane; };
    Put puts[4];
    size_t n = 0;
    bool aligned = true;
    u32 base = idx;
    const u32 a = cur_ea_ & 3u;
    switch (op) {
    case 0x28: puts[n++] = {0, 0}; break;
    case 0x29: aligned = (cur_ea_ & 1u) == 0; puts[n++] = {0, 0}; puts[n++] = {1, 1}; break;
    case 0x2B: case 0x3A:
      aligned = (cur_ea_ & 3u) == 0;
      for (u32 i = 0; i < 4; ++i) puts[n++] = {i, i};
      break;
    case 0x2A: // SWL: lanes [3-a .. 3] to bytes [base .. ea]
      base = idx & ~3u;
      for (u32 i = 0; i <= a; ++i) puts[n++] = {i, 3 - a + i};
      break;
    default: // SWR: lanes [0 .. 3-a] to bytes [ea .. end of word]
      for (u32 j = 0; j + a <= 3u; ++j) puts[n++] = {j, j};
      break;
    }
    if (!aligned) {
      break;
    }
    if (r == kRam || r == kScratch) {
      if ((cur_sr_ & 0x10000u) != 0u) {
        break; // cache isolation: the store goes to the I-cache, not to memory
      }
      u32 tagged = 0;
      for (size_t i = 0; i < n; ++i) {
        set_tag(r, base + puts[i].off, src[puts[i].lane]);
        tagged += src[puts[i].lane] != 0 ? 1u : 0u;
      }
      if (tagged != 0) {
        stats_.tagged_store_bytes_by_op[op] += tagged;
        ++stats_.tagged_store_pcs[cur_pc_];
      }
    } else if (r == kIo) {
      u8 consumer = 0;
      if (idx >= kIoSpuBegin && idx < kIoSpuEnd) {
        consumer = kGrimConsumerSpu;
      } else if (idx >= 0x1F801810u && idx < 0x1F801818u) {
        consumer = kGrimConsumerGpu;
      } else if (idx >= 0x1F801820u && idx < 0x1F801828u) {
        consumer = kGrimConsumerMdec;
      }
      u32 tags[4];
      size_t nt = 0;
      for (size_t i = 0; i < n; ++i) {
        tags[nt++] = src[puts[i].lane];
      }
      bool any = false;
      for (size_t i = 0; i < nt; ++i) {
        any = any || tags[i] != 0;
      }
      if (any) {
        ++stats_.tagged_stores_to_io;
        note_origin(tags, nt, consumer, true, cur_cycle_);
        u32 padded[4] = {0, 0, 0, 0};
        for (size_t i = 0; i < nt; ++i) {
          padded[i] = tags[i];
        }
        note_use(padded); // a device register consumed it
      }
      // System implements SPU accesses as halfword writes. Byte stores to this
      // range are unhandled and must not advance the FIFO's shadow pointer.
      if (idx >= kIoSpuBegin && idx < kIoSpuEnd && op != 0x28) {
        const u32 offset = idx - kIoSpuBegin;
        if (op == 0x29) {
          note_spu_write16(offset, static_cast<u16>(cur_value_), src);
        } else if (op == 0x2B || op == 0x2A || op == 0x2E) {
          const u32 write_offset = base - kIoSpuBegin;
          u32 value = cur_value_;
          u32 out_tags[4] = {0, 0, 0, 0};
          if (op == 0x2B) {
            for (u32 i = 0; i < 4u; ++i) out_tags[i] = src[i];
          } else {
            // SWL/SWR read-modify-write a full aligned word in the interpreter.
            const u32 aligned_offset = write_offset & ~3u;
            value = static_cast<u32>(spu_regs_[aligned_offset / 2u]) |
                    (static_cast<u32>(spu_regs_[aligned_offset / 2u + 1u]) << 16u);
            for (size_t i = 0; i < n; ++i) {
              const u32 lane = (base + puts[i].off) & 3u;
              const u32 mask = 255u << (lane * 8u);
              value = (value & ~mask) | (((cur_value_ >> (puts[i].lane * 8u)) & 255u)
                                         << (lane * 8u));
              out_tags[lane] = src[puts[i].lane];
            }
          }
          const u32 aligned_offset = write_offset & ~3u;
          note_spu_write16(aligned_offset, static_cast<u16>(value), out_tags);
          note_spu_write16(aligned_offset + 2u, static_cast<u16>(value >> 16u), out_tags + 2u);
        }
      }
    }
    break;
  }
  default:
    break;
  }

  // Registers the instruction consumes as values (a store's data register is
  // transport, and so is an exact register copy). Tagged data reaching here is
  // used by the program.
  if (!is_copy) {
    bool use_s = false, use_t = false;
    switch (op) {
    case 0:
      switch (funct) {
      case 0: case 2: case 3: use_t = true; break;
      case 8: case 9: case 0x11: case 0x13: use_s = true; break;
      case 4: case 6: case 7: case 0x18: case 0x19: case 0x1A: case 0x1B: case 0x20: case 0x21:
      case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27: case 0x2A: case 0x2B:
        use_s = use_t = true;
        break;
      default: break;
      }
      break;
    case 1: case 6: case 7: case 8: case 9: case 0xA: case 0xB: case 0xC: case 0xD: case 0xE:
      use_s = true;
      break;
    case 4: case 5: use_s = use_t = true; break;
    case 0x10: use_t = rs == 4; break;
    case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x28:
    case 0x29: case 0x2A: case 0x2B: case 0x2E: case 0x32: case 0x3A:
      use_s = true; // the base register
      break;
    default: break;
    }
    if (use_s) {
      note_use(reg_tags_[rs].data());
    }
    if (use_t) {
      note_use(reg_tags_[rt].data());
    }
  }

  // A load's result lands after the next instruction: apply the previous one
  // now, unless this instruction overwrites the same register.
  if (pend_reg_ >= 0) {
    if (pend_reg_ != dest) {
      set_reg_tags(static_cast<u32>(pend_reg_), pend_tags_.data());
    }
    pend_reg_ = -1;
  }
  if (dest > 0) {
    if (is_load) {
      pend_reg_ = dest;
      for (int i = 0; i < 4; ++i) {
        pend_tags_[i] = dtags[i];
      }
    } else {
      set_reg_tags(static_cast<u32>(dest), dtags);
    }
  }
}

void GrimBootMapper::note_dma(int channel, bool from_ram, u32 addr, s32 step, u32 words,
                              u64 cycle) {
  const u8 consumer = channel == 2   ? kGrimConsumerGpu
                      : channel == 4 ? kGrimConsumerSpu
                      : channel == 0 ? kGrimConsumerMdec
                                     : 0;
  u32 a = addr & (kRamBytes - 4u);
  for (u32 i = 0; i < words; ++i) {
    u32 *t = &ram_tag_[a];
    if (from_ram) {
      if ((t[0] | t[1] | t[2] | t[3]) != 0) {
        ++stats_.dma_tagged_words;
        note_origin(t, 4, consumer, true, cycle);
      }
      if (channel == 4) {
        for (u32 b = 0; b < 4u; ++b) {
          spu_tag_[(spu_transfer_addr_ + b) & (kSpuRamBytes - 1u)] = t[b];
        }
      }
    } else {
      t[0] = t[1] = t[2] = t[3] = 0;
      ++stats_.dma_words_to_ram;
    }
    if (channel == 4) {
      // Both SPU DMA directions advance the same transfer pointer.
      spu_transfer_addr_ = (spu_transfer_addr_ + 4u) & (kSpuRamBytes - 1u);
    }
    a = static_cast<u32>(static_cast<s32>(a) + step) & (kRamBytes - 4u);
  }
}

GrimCopyStats GrimBootMapper::stats_report() const {
  GrimCopyStats s = stats_;
  u32 start = ~0u;
  for (u32 w = 0; w <= kRamBytes / 4u; ++w) {
    const bool unknown = w < kRamBytes / 4u && ram_exec_[w] == 1u; // seen, never known
    if (unknown && start == ~0u) {
      start = w;
    } else if (!unknown && start != ~0u) {
      if (s.unknown_exec_ranges.size() < 64) {
        s.unknown_exec_ranges.emplace_back(start * 4u, w * 4u);
      }
      start = ~0u;
    }
  }
  return s;
}

GrimBootMap GrimBootMapper::finish(u64 bios_hash, const std::string &scenario, u32 frames,
                                   u64 cycles) const {
  GrimBootMap m;
  m.bios_hash = bios_hash;
  m.scenario = scenario;
  m.frames = frames;
  m.cycles = cycles;
  m.ram_exec_words = ram_exec_words_;
  m.ram_exec_known = ram_exec_known_;
  m.spu_sample_uses = spu_sample_uses_;
  m.words.resize(first_exec_.size());
  for (size_t i = 0; i < m.words.size(); ++i) {
    GrimMapWord &w = m.words[i];
    w.first_exec = first_exec_[i];
    w.first_read = first_read_[i];
    w.consumer = consumer_[i];
    w.flags = flags_[i];
    w.cls = grim_classify_word(w.flags, w.consumer);
    if (w.cls == GrimWordClass::Code) {
      m.last_new_code_cycle = std::max(m.last_new_code_cycle, w.first_exec);
    }
  }
  return m;
}
