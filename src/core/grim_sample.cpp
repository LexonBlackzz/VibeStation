#include "grim_sample.h"
#include "bios.h"
#include <algorithm>
#include <cstdio>
#include <numeric>

namespace {
u8 byte_at(const std::vector<u32> &words, u32 offset) {
  return static_cast<u8>(words[offset / 4u] >> ((offset & 3u) * 8u));
}
void put_byte(std::vector<u32> &words, u32 offset, u8 value) {
  const u32 shift = (offset & 3u) * 8u;
  words[offset / 4u] = (words[offset / 4u] & ~(255u << shift)) | (u32{value} << shift);
}
bool valid_block(const std::vector<u32> &words, u32 offset) {
  if (u64{offset} + 16u > u64{words.size()} * 4u) return false;
  const u8 h = byte_at(words, offset), f = byte_at(words, offset + 1u);
  return (h & 0x80u) == 0 && ((h >> 4u) & 7u) <= 4u && (h & 15u) <= 12u && f <= 7u;
}
bool zero_block(const std::vector<u32> &words, u32 offset) {
  for (u32 b = 0; b < 16; ++b) if (byte_at(words, offset + b) != 0) return false;
  return true;
}
bool nonzero_payload(const std::vector<u32> &words, u32 offset) {
  for (u32 b = 2; b < 16; ++b) if (byte_at(words, offset + b) != 0) return true;
  return false;
}
GrimAdpcmSample make_sample(const std::vector<u32> &words, u32 start, u32 end) {
  GrimAdpcmSample s;
  s.start_offset = start;
  s.end_offset = end;
  s.block_count = (end - start) / 16u;
  for (u32 off = start; off < end; off += 16u) {
    const u8 flags = byte_at(words, off + 1u);
    if (flags & 4u) s.loop_start_offsets.push_back(off);
    if (flags & 1u) s.loop_end_offset = off;
  }
  return s;
}
bool sample_valid(const GrimSampleContext &ctx, const GrimAdpcmSample &s) {
  return s.block_count != 0 && (s.start_offset & 7u) == 0 &&
         u64{s.start_offset} + u64{s.block_count} * 16u == s.end_offset &&
         u64{s.end_offset} <= u64{ctx.words.size()} * 4u;
}
bool applicable(GrimSampleMut kind, const GrimSampleContext &ctx, u32 index) {
  if (index >= ctx.samples.size() || !sample_valid(ctx, ctx.samples[index])) return false;
  const GrimAdpcmSample &s = ctx.samples[index];
  if (kind == GrimSampleMut::LoopEndRemove) return s.loop_end_offset != kGrimSampleAuto;
  if (kind == GrimSampleMut::LoopStartMove || kind == GrimSampleMut::LoopEndEarly ||
      kind == GrimSampleMut::BlockShuffle || kind == GrimSampleMut::BlockRepeat ||
      kind == GrimSampleMut::BlockReverse) return s.block_count >= 2;
  if (kind == GrimSampleMut::Transplant) return ctx.samples.size() >= 2;
  return kind < GrimSampleMut::Count;
}
void copy_payload(std::vector<u32> &dst, u32 dst_offset,
                  const std::vector<u32> &src, u32 src_offset) {
  for (u32 b = 2; b < 16; ++b) put_byte(dst, dst_offset + b, byte_at(src, src_offset + b));
}
std::vector<u32> choose_blocks(GrimRng &rng, u32 blocks, u32 count) {
  std::vector<u32> out(blocks);
  std::iota(out.begin(), out.end(), 0u);
  count = std::min(count, blocks);
  for (u32 i = 0; i < count; ++i) std::swap(out[i], out[rng.range(i, blocks - 1u)]);
  out.resize(count);
  return out;
}
} // namespace

std::vector<GrimAdpcmSample> grim_scan_adpcm(const std::vector<u32> &words, u32 min_blocks) {
  std::vector<GrimAdpcmSample> candidates;
  const u32 size = static_cast<u32>(words.size() * 4u);
  min_blocks = std::max(1u, min_blocks);
  // The two lanes reflect the SPU's eight-byte address units. A run never
  // changes phase: after its first block all block boundaries are 16 bytes apart.
  for (u32 lane : {0u, 8u}) {
    u32 start = kGrimSampleAuto, blocks = 0, nonzero = 0, leading_zero = 0;
    for (u32 off = lane; u64{off} + 16u <= size; off += 16u) {
      if (!valid_block(words, off)) {
        start = kGrimSampleAuto; blocks = nonzero = leading_zero = 0;
        continue;
      }
      if (start == kGrimSampleAuto) start = off;
      ++blocks;
      if (nonzero_payload(words, off)) ++nonzero;
      if (blocks == leading_zero + 1u && zero_block(words, off)) ++leading_zero;
      if (byte_at(words, off + 1u) & 1u) {
        // One zero preamble can intentionally reset the predictor; swallowing
        // an arbitrarily long zero-filled ROM gap would be a false boundary.
        const u32 trim = leading_zero > 1u ? leading_zero - 1u : 0u;
        if (blocks - trim >= min_blocks && nonzero >= 2u)
          candidates.push_back(make_sample(words, start + trim * 16u, off + 16u));
        start = kGrimSampleAuto; blocks = nonzero = leading_zero = 0;
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const GrimAdpcmSample &a,
                                                    const GrimAdpcmSample &b) {
    if (a.block_count != b.block_count) return a.block_count > b.block_count;
    return a.start_offset < b.start_offset;
  });
  std::vector<GrimAdpcmSample> result;
  for (const GrimAdpcmSample &s : candidates) {
    const bool overlap = std::any_of(result.begin(), result.end(), [&](const GrimAdpcmSample &p) {
      return s.start_offset < p.end_offset && p.start_offset < s.end_offset;
    });
    if (!overlap) result.push_back(s);
  }
  std::sort(result.begin(), result.end(), [](const GrimAdpcmSample &a, const GrimAdpcmSample &b) {
    return a.start_offset < b.start_offset;
  });
  return result;
}

double GrimSampleScanScore::precision() const {
  const u32 n = true_positive_words + false_positive_words;
  return n ? static_cast<double>(true_positive_words) / n : 0.0;
}
double GrimSampleScanScore::recall() const {
  const u32 n = true_positive_words + false_negative_words;
  return n ? static_cast<double>(true_positive_words) / n : 0.0;
}
GrimSampleScanScore grim_sample_score(const std::vector<GrimAdpcmSample> &samples,
                                    const GrimBootMap &map) {
  std::vector<u8> found(map.words.size(), 0);
  for (const GrimAdpcmSample &s : samples)
    for (u32 i = s.start_offset / 4u; i < s.end_offset / 4u && i < found.size(); ++i) found[i] = 1;
  GrimSampleScanScore score;
  for (size_t i = 0; i < found.size(); ++i) {
    const bool truth = (map.words[i].consumer & kGrimConsumerSpu) != 0;
    if (found[i] && truth) ++score.true_positive_words;
    else if (found[i]) ++score.false_positive_words;
    else if (truth) ++score.false_negative_words;
  }
  return score;
}

std::vector<GrimAdpcmSample> grim_sample_annotate(const std::vector<u32> &words,
                                               const std::vector<GrimAdpcmSample> &scanned,
                                               const GrimBootMap &map) {
  std::vector<u32> starts;
  for (const GrimSpuSampleUse &u : map.spu_sample_uses) {
    if (u.kind == GrimSpuSampleUseKind::KeyOnStart && u.rom_offset != kGrimNoRomOffset &&
        (u.rom_offset & 7u) == 0 && valid_block(words, u.rom_offset)) starts.push_back(u.rom_offset);
  }
  std::sort(starts.begin(), starts.end());
  starts.erase(std::unique(starts.begin(), starts.end()), starts.end());
  std::vector<GrimAdpcmSample> result;
  // A voice start is an actual playback boundary, and can point inside a
  // byte-only run. Follow that start to its own loop-end; tails may be shared.
  for (u32 start : starts) {
    for (u32 off = start; valid_block(words, off); off += 16u) {
      if (byte_at(words, off + 1u) & 1u) {
        GrimAdpcmSample s = make_sample(words, start, off + 16u);
        s.voice_start_confirmed = true;
        result.push_back(std::move(s));
        break;
      }
    }
  }
  // Preserve unplayed/unresolved byte candidates. A raw prefix before a known
  // voice boundary is retained only in scanner_samples, not called a sample.
  for (const GrimAdpcmSample &s : scanned) {
    const bool has_voice = std::any_of(result.begin(), result.end(), [&](const GrimAdpcmSample &p) {
      return p.start_offset >= s.start_offset && p.start_offset < s.end_offset;
    });
    if (!has_voice) result.push_back(s);
  }
  std::sort(result.begin(), result.end(), [](const GrimAdpcmSample &a, const GrimAdpcmSample &b) {
    return a.start_offset < b.start_offset;
  });
  for (GrimAdpcmSample &s : result) {
    for (u32 i = s.start_offset / 4u; i < s.end_offset / 4u && i < map.words.size(); ++i)
      if (map.words[i].consumer & kGrimConsumerSpu) ++s.spu_words;
    for (const GrimSpuSampleUse &u : map.spu_sample_uses) {
      const bool start_use = u.kind == GrimSpuSampleUseKind::StartWrite ||
                             u.kind == GrimSpuSampleUseKind::KeyOnStart;
      const bool match = start_use ? u.rom_offset == s.start_offset
                                 : u.rom_offset >= s.start_offset && u.rom_offset < s.end_offset;
      if (!match || u.rom_offset == kGrimNoRomOffset) continue;
      s.uses.push_back(GrimSampleVoiceUse{u.cycle, u.rom_offset, u.spu_address, u.voice,
                                        static_cast<u8>(u.kind)});
      s.voice_mask |= 1u << u.voice;
      s.first_use_cycle = std::min(s.first_use_cycle, u.cycle);
      s.last_use_cycle = std::max(s.last_use_cycle, u.cycle);
      if (!start_use && std::find(s.loop_start_offsets.begin(), s.loop_start_offsets.end(),
                                 u.rom_offset) == s.loop_start_offsets.end())
        s.loop_start_offsets.push_back(u.rom_offset);
    }
    std::sort(s.loop_start_offsets.begin(), s.loop_start_offsets.end());
  }
  return result;
}

bool GrimSampleContext::load(const std::string &bios_path, const std::string &map_path,
                             std::string &err) {
  Bios bios;
  if (!bios.load(bios_path)) { err = "cannot load BIOS " + bios_path; return false; }
  bios_hash = bios.image_hash();
  words.assign(bios.image_size() / 4u, 0);
  for (u32 i = 0; i < words.size(); ++i) bios.original_word(i * 4u, words[i]);
  map = GrimBootMap{};
  if (!map_path.empty()) {
    if (!grim_map_load(map_path, map, err)) return false;
    if (map.bios_hash != bios_hash || map.words.size() != words.size()) {
      err = "sample map does not match the stock BIOS image";
      return false;
    }
  }
  init();
  return true;
}
void GrimSampleContext::init() {
  scanner_samples = grim_scan_adpcm(words);
  samples = map.words.empty() ? scanner_samples : grim_sample_annotate(words, scanner_samples, map);
}

const char *grim_sample_mut_name(GrimSampleMut kind) {
  static const char *const names[] = {"filter_swap", "shift_change", "loop_start_move",
    "loop_end_remove", "loop_end_early", "block_shuffle", "block_repeat", "block_reverse",
    "transplant", "nibble_noise"};
  return static_cast<size_t>(kind) < sizeof(names) / sizeof(names[0])
           ? names[static_cast<size_t>(kind)] : "invalid";
}
bool grim_sample_header_gene(GrimSampleMut kind) {
  return kind <= GrimSampleMut::LoopEndEarly;
}

GrimGene grim_sample_generate(const GrimSampleContext &ctx, u64 seed, GrimSampleMut kind,
                             u32 count, u32 magnitude, u32 sample_index, u32 donor_index) {
  GrimGene gene = grim_default_gene(GrimGeneType::SpuSample);
  gene.seed = seed;
  count = std::max(1u, std::min(count, 256u));
  magnitude = std::max(1u, std::min(magnitude, 15u));
  gene.params[0] = static_cast<s32>(kind);
  gene.params[1] = static_cast<s32>(count);
  gene.params[2] = static_cast<s32>(magnitude);
  gene.params[3] = sample_index == kGrimSampleAuto ? -1 : static_cast<s32>(sample_index);
  gene.params[4] = donor_index == kGrimSampleAuto ? -1 : static_cast<s32>(donor_index);
  gene.params[5] = 0;
  if (kind >= GrimSampleMut::Count) return gene;
  std::vector<u32> candidates;
  for (u32 i = 0; i < ctx.samples.size(); ++i) {
    if (!applicable(kind, ctx, i)) continue;
    if (sample_index != kGrimSampleAuto && sample_index != i) continue;
    candidates.push_back(i);
  }
  // With a map, prefer confirmed bank data over byte-pattern false positives.
  if (sample_index == kGrimSampleAuto && !ctx.map.words.empty()) {
    std::vector<u32> confirmed;
    for (u32 i : candidates) if (ctx.samples[i].spu_words != 0) confirmed.push_back(i);
    if (!confirmed.empty()) candidates = std::move(confirmed);
  }
  if (candidates.empty()) return gene;
  GrimRng rng{seed};
  for (u32 attempt = 0; attempt < 128u; ++attempt) {
    u64 total_blocks = 0;
    for (u32 i : candidates) total_blocks += ctx.samples[i].block_count;
    u64 draw = rng.next() % total_blocks;
    u32 si = candidates.back();
    for (u32 i : candidates) {
      if (draw < ctx.samples[i].block_count) { si = i; break; }
      draw -= ctx.samples[i].block_count;
    }
    const GrimAdpcmSample &s = ctx.samples[si];
    std::vector<u32> changed = ctx.words;
    u32 di = kGrimSampleAuto;
    const auto blocks = choose_blocks(rng, s.block_count, count);
    switch (kind) {
    case GrimSampleMut::FilterSwap:
      for (u32 b : blocks) {
        const u32 off = s.start_offset + b * 16u;
        const u8 h = byte_at(changed, off);
        const u32 old = (h >> 4u) & 7u;
        const u32 filter = (old + rng.range(1u, 4u)) % 5u;
        put_byte(changed, off, static_cast<u8>((h & 0x8Fu) | (filter << 4u)));
      }
      break;
    case GrimSampleMut::ShiftChange:
      for (u32 b : blocks) {
        const u32 off = s.start_offset + b * 16u;
        const u8 h = byte_at(changed, off);
        const u32 delta = rng.range(1u, magnitude);
        const u32 shift = ((h & 15u) + ((rng.next() & 1u) ? delta : 16u - delta)) & 15u;
        put_byte(changed, off, static_cast<u8>((h & 0xF0u) | shift));
      }
      break;
    case GrimSampleMut::LoopStartMove: {
      const u32 dest = rng.range(0, s.block_count - 1u);
      for (u32 b = 0; b < s.block_count; ++b) {
        const u32 off = s.start_offset + b * 16u + 1u;
        const u8 flags = byte_at(changed, off);
        put_byte(changed, off, static_cast<u8>((flags & ~4u) | (b == dest ? 4u : 0u)));
      }
      break;
    }
    case GrimSampleMut::LoopEndRemove:
      for (u32 b = 0; b < s.block_count; ++b) {
        const u32 off = s.start_offset + b * 16u + 1u;
        put_byte(changed, off, static_cast<u8>(byte_at(changed, off) & ~1u));
      }
      break;
    case GrimSampleMut::LoopEndEarly: {
      const u32 off = s.start_offset + rng.range(0, s.block_count - 2u) * 16u + 1u;
      put_byte(changed, off, static_cast<u8>(byte_at(changed, off) | 1u));
      break;
    }
    case GrimSampleMut::BlockShuffle:
    case GrimSampleMut::BlockReverse: {
      const u32 n = std::min(s.block_count, std::max(2u, count));
      const u32 first = rng.range(0, s.block_count - n);
      std::vector<u32> order(n);
      std::iota(order.begin(), order.end(), 0u);
      if (kind == GrimSampleMut::BlockReverse) std::reverse(order.begin(), order.end());
      else {
        for (u32 i = n - 1u; i > 0; --i) std::swap(order[i], order[rng.range(0, i)]);
        bool identity = true;
        for (u32 i = 0; i < n; ++i) identity = identity && order[i] == i;
        if (identity) std::rotate(order.begin(), order.begin() + 1, order.end());
      }
      for (u32 i = 0; i < n; ++i)
        copy_payload(changed, s.start_offset + (first + i) * 16u,
                     ctx.words, s.start_offset + (first + order[i]) * 16u);
      break;
    }
    case GrimSampleMut::BlockRepeat: {
      const u32 source = rng.range(0, s.block_count - 1u);
      for (u32 b : blocks)
        copy_payload(changed, s.start_offset + b * 16u, ctx.words, s.start_offset + source * 16u);
      break;
    }
    case GrimSampleMut::Transplant: {
      std::vector<u32> donors;
      for (u32 i = 0; i < ctx.samples.size(); ++i) {
        if (i == si || !sample_valid(ctx, ctx.samples[i])) continue;
        if (donor_index != kGrimSampleAuto && donor_index != i) continue;
        if (donor_index == kGrimSampleAuto && !ctx.map.words.empty() &&
            ctx.samples[i].spu_words == 0) continue;
        donors.push_back(i);
      }
      if (donors.empty()) return gene;
      di = donors[rng.range(0, static_cast<u32>(donors.size()) - 1u)];
      const GrimAdpcmSample &donor = ctx.samples[di];
      const u32 source = rng.range(0, donor.block_count - 1u);
      for (u32 i = 0; i < blocks.size(); ++i)
        copy_payload(changed, s.start_offset + blocks[i] * 16u, ctx.words,
                     donor.start_offset + ((source + i) % donor.block_count) * 16u);
      break;
    }
    case GrimSampleMut::NibbleNoise:
      for (u32 b : blocks) {
        const u32 off = s.start_offset + b * 16u;
        const auto nibbles = choose_blocks(rng, 28u, std::min(28u, magnitude));
        for (u32 n : nibbles) {
          const u32 byte_off = off + 2u + n / 2u;
          const u8 mask = static_cast<u8>(rng.range(1u, 15u) << ((n & 1u) * 4u));
          put_byte(changed, byte_off, static_cast<u8>(byte_at(changed, byte_off) ^ mask));
        }
      }
      break;
    default: return gene;
    }
    gene.patches.clear();
    for (u32 i = s.start_offset / 4u; i < s.end_offset / 4u; ++i) {
      if (changed[i] != ctx.words[i])
        gene.patches.push_back(GrimRomPatch{i * 4u, ctx.words[i], changed[i], false});
    }
    if (gene.patches.empty()) continue;
    gene.params[3] = static_cast<s32>(si);
    gene.params[4] = di == kGrimSampleAuto ? -1 : static_cast<s32>(di);
    gene.params[6] = static_cast<s32>(s.start_offset & 15u);
    if (kind == GrimSampleMut::ShiftChange) {
      for (u32 b = 0; b < s.block_count; ++b) {
        const u32 off = s.start_offset + b * 16u;
        if (byte_at(changed, off) != byte_at(ctx.words, off) && (byte_at(changed, off) & 15u) >= 13u)
          gene.params[5] = 1;
      }
    }
    return gene;
  }
  return gene;
}

void grim_add_random_sample_genes(GrimGenome &genome, u64 seed, const GrimRandomParams &rp) {
  if (rp.sample == nullptr) return;
  const GrimSampleContext &ctx = *rp.sample;
  GrimRng rng{seed ^ 0x53414D5047454E31ull}; // "SAMPGEN1", independent stream
  const u32 n = rng.range(rp.sample_genes_min, std::max(rp.sample_genes_min, rp.sample_genes_max));
  for (u32 g = 0; g < n; ++g) {
    const auto kind = rp.sample_kind < 0 ? static_cast<GrimSampleMut>(
      rng.range(0, static_cast<u32>(GrimSampleMut::Count) - 1u)) : static_cast<GrimSampleMut>(rp.sample_kind);
    const u32 count = rng.range(1, std::max(1u, rp.sample_count_max));
    const u32 si = rp.sample_index < 0 ? kGrimSampleAuto : static_cast<u32>(rp.sample_index);
    GrimGene gene = grim_sample_generate(ctx, rng.next(), kind, count, rp.sample_magnitude, si);
    if (!gene.patches.empty()) genome.genes.push_back(std::move(gene));
  }
  if (grim_genome_has_rom(genome)) { genome.version = 2; genome.bios_hash = ctx.bios_hash; }
}

std::string grim_sample_summary(const GrimSampleContext &ctx) {
  std::string out;
  char b[300];
  std::snprintf(b, sizeof(b), "GRIM_SAMPLES bios=0x%016llX scanned=%zu samples=%zu\n",
                static_cast<unsigned long long>(ctx.bios_hash), ctx.scanner_samples.size(), ctx.samples.size());
  out += b;
  if (!ctx.map.words.empty()) {
    const auto score = grim_sample_score(ctx.scanner_samples, ctx.map);
    std::snprintf(b, sizeof(b), "GRIM_SAMPLE_SCAN tp=%u fp=%u fn=%u precision=%.4f recall=%.4f\n",
      score.true_positive_words, score.false_positive_words, score.false_negative_words,
      score.precision(), score.recall());
    out += b;
  }
  for (size_t i = 0; i < ctx.samples.size(); ++i) {
    const auto &s = ctx.samples[i];
    std::snprintf(b, sizeof(b), "sample %zu: rom=0x%05X-0x%05X blocks=%u loop_end=0x%05X "
                  "voices=0x%06X confirmed=%d spu_words=%u",
      i, s.start_offset, s.end_offset, s.block_count, s.loop_end_offset, s.voice_mask,
      s.voice_start_confirmed ? 1 : 0, s.spu_words);
    out += b;
    if (s.first_use_cycle != kGrimNever) {
      std::snprintf(b, sizeof(b), " first=%.3fms last=%.3fms",
        1000.0 * static_cast<double>(s.first_use_cycle) / psx::CPU_CLOCK_HZ,
        1000.0 * static_cast<double>(s.last_use_cycle) / psx::CPU_CLOCK_HZ);
      out += b;
    }
    out += " loop_start=";
    for (u32 off : s.loop_start_offsets) {
      std::snprintf(b, sizeof(b), "0x%05X,", off); out += b;
    }
    out += "\n";
    for (const auto &u : s.uses) {
      std::snprintf(b, sizeof(b), "  use voice=%u kind=%u rom=0x%05X spu=0x%05X cycle=%llu\n",
        u.voice, u.kind, u.rom_offset, u.spu_address, static_cast<unsigned long long>(u.cycle));
      out += b;
    }
  }
  return out;
}
std::string grim_sample_describe_gene(const GrimGene &g, const GrimSampleContext *ctx) {
  std::string out;
  char b[300];
  std::snprintf(b, sizeof(b), "spu_sample kind=%s patches=%zu blocks=%d magnitude=%d generated_sample=%d generated_donor=%d phase=%d%s\n",
    grim_sample_mut_name(static_cast<GrimSampleMut>(g.params[0])), g.patches.size(), g.params[1],
    g.params[2], g.params[3], g.params[4], g.params[6],
    g.params[5] ? " shift13-15=VibeStation-defined (not hardware-verified)" : "");
  out += b;
  if (ctx != nullptr && !g.patches.empty()) {
    // A map can refine/reorder the byte-only candidate list used during
    // generation. The persisted indices are explanatory parameters, while
    // resolved patch offsets remain authoritative across those contexts.
    for (size_t i = 0; i < ctx->samples.size(); ++i) {
      const auto &s = ctx->samples[i];
      if ((s.start_offset & 15u) != static_cast<u32>(g.params[6])) continue;
      const bool contains = std::all_of(g.patches.begin(), g.patches.end(), [&](const GrimRomPatch &p) {
        return p.offset >= s.start_offset && p.offset < s.end_offset;
      });
      if (!contains) continue;
      std::snprintf(b, sizeof(b), "  contextual sample %zu: ROM 0x%05X-0x%05X blocks=%u voices=0x%06X\n",
        i, s.start_offset, s.end_offset, s.block_count, s.voice_mask); out += b;
    }
  }
  for (const GrimRomPatch &p : g.patches) {
    std::snprintf(b, sizeof(b), "  0x%05X: 0x%08X -> 0x%08X\n", p.offset, p.original, p.mutated);
    out += b;
  }
  return out;
}
