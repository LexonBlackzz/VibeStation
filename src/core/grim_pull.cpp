#include "grim_pull.h"
#include "grim_rom.h"
#include "grim_sample.h"
#include <algorithm>
#include <cctype>
#include <cstdio>

GrimPullPlan grim_pull_plan(u32 intensity) {
  const u32 t = std::min(intensity, 100u);
  GrimPullPlan p;
  p.min_genes = 1u + t * 6u / 100u;
  p.max_genes = 2u + t * 9u / 100u;
  p.risk_q10 = t * 1024u / 100u;
  p.rom_early_ms = 600u - t * 6u;
  p.rom_curve = t < 20u ? 3u : t < 50u ? 2u : t < 80u ? 1u : 0u;
  p.rom_patches_max = 2u + t * 6u / 100u;
  p.rom_call_swap = t >= 90u;
  p.sample_ms = 100u + t * 4u;
  p.hw_avoid_code_permille = t <= 20u ? 1000u : (100u - t) * 1000u / 80u;
  p.risk_label = t < 20u ? "safe" : t < 45u ? "mild" : t < 70u ? "risky" : "lethal";
  return p;
}

std::string grim_pull_readout(u32 intensity) {
  const GrimPullPlan p = grim_pull_plan(intensity);
  char b[64];
  std::snprintf(b, sizeof(b), "%u-%u genes \xC2\xB7 %s", p.min_genes, p.max_genes, p.risk_label);
  return b;
}

u32 grim_pull_available_families(const GrimPullContext &ctx) {
  // SPU and GP0 filters and hardware faults need nothing but a running machine;
  // without a sound-bank scan, Audio is SPU filters only.
  u32 m = kGrimFamilyAudio | kGrimFamilyVisual | kGrimFamilyHardware;
  if (ctx.rom != nullptr) {
    m |= kGrimFamilyCode;
  }
  return m;
}

u32 grim_gene_family(GrimGeneType t) {
  switch (t) {
  case GrimGeneType::SpuSample:
  case GrimGeneType::SpuPitch:
  case GrimGeneType::SpuAdsr:
  case GrimGeneType::SpuVolume:
  case GrimGeneType::SpuAddress:
  case GrimGeneType::SpuKeyOn:
  case GrimGeneType::SpuNoise:
  case GrimGeneType::SpuPmon:
  case GrimGeneType::SpuReverb:
    return kGrimFamilyAudio;
  case GrimGeneType::RomCode: return kGrimFamilyCode;
  case GrimGeneType::HwRam:
  case GrimGeneType::HwVram:
  case GrimGeneType::HwSpuRam: return kGrimFamilyHardware;
  default: return kGrimFamilyVisual;
  }
}

namespace {
std::vector<u64> gene_keys(const GrimGenome &g) {
  std::vector<u64> keys;
  for (const GrimGene &gene : g.genes) {
    u64 k = 0;
    if (gene.type == GrimGeneType::SpuSample) {
      k = (1ull << 56) | (static_cast<u64>(gene.params[0]) << 16) |
          static_cast<u64>(static_cast<u32>(gene.params[3]) & 0xFFFFu);
    } else if (gene.type == GrimGeneType::RomCode) {
      const u32 off = gene.patches.empty() ? 0u : gene.patches.front().offset >> 13;
      k = (2ull << 56) | (static_cast<u64>(gene.params[0]) << 16) | off;
    } else if (grim_gene_is_hardware(gene.type)) {
      k = (4ull << 56) | (static_cast<u64>(gene.type) << 16) | static_cast<u64>(gene.params[0]);
    } else {
      k = (3ull << 56) | (static_cast<u64>(gene.type) << 8) |
          static_cast<u64>(gene.trigger.kind);
    }
    keys.push_back(k);
  }
  return keys;
}

const char *iface_title(GrimGeneType t) {
  switch (t) {
  case GrimGeneType::SpuPitch: return "Pitch warp";
  case GrimGeneType::SpuAdsr: return "Envelope rot";
  case GrimGeneType::SpuVolume: return "Volume drift";
  case GrimGeneType::SpuAddress: return "Sample address slip";
  case GrimGeneType::SpuKeyOn: return "Key-on stutter";
  case GrimGeneType::SpuNoise: return "Noise injection";
  case GrimGeneType::SpuPmon: return "Pitch modulation";
  case GrimGeneType::SpuReverb: return "Reverb abuse";
  case GrimGeneType::GpuVertex: return "Vertex drift";
  case GrimGeneType::GpuColor: return "Colour shift";
  case GrimGeneType::GpuFlags: return "Command flag flip";
  case GrimGeneType::GpuTexParam: return "Texture slip";
  case GrimGeneType::GpuState: return "Draw state glitch";
  case GrimGeneType::GpuFill: return "Fill glitch";
  default: return grim_gene_type_name(t);
  }
}

std::string trigger_text(const GrimTrigger &t, u32 fps) {
  char b[96];
  const auto sec = [fps](u32 f) { return static_cast<double>(f) / static_cast<double>(std::max(fps, 1u)); };
  switch (t.kind) {
  case GrimTriggerKind::Always: return "always on";
  case GrimTriggerKind::Window:
    std::snprintf(b, sizeof(b), "window %.1f-%.1f s", sec(t.start_frame), sec(t.end_frame));
    return b;
  case GrimTriggerKind::Rot:
    std::snprintf(b, sizeof(b), "rot %.0f-%.0f s", sec(t.start_frame), sec(t.end_frame));
    return b;
  case GrimTriggerKind::Intermittent:
    return t.period > 0 ? "pulsing" : "random hits";
  }
  return "";
}

std::string hw_title(const GrimGene &g) {
  static const char *const kRam[] = {"Stuck-high RAM bits", "Stuck-low RAM bits", "Flaky RAM cells",
                                     "RAM bursts", "Bad RAM column", "Thermal RAM drift", "Rowhammer"};
  static const char *const kVram[] = {"Stuck-high VRAM bits", "Stuck-low VRAM bits",
                                      "Flaky VRAM cells", "Dead VRAM line"};
  static const char *const kSpu[] = {"Stuck-high sound RAM bits", "Stuck-low sound RAM bits",
                                     "Flaky sound RAM cells", "Sound RAM bursts", "Bad sound RAM column"};
  const size_t mode = static_cast<size_t>(g.params[0]);
  if (g.type == GrimGeneType::HwRam) return mode < 7 ? kRam[mode] : "RAM fault";
  if (g.type == GrimGeneType::HwVram) return mode < 4 ? kVram[mode] : "VRAM fault";
  return mode < 5 ? kSpu[mode] : "Sound RAM fault";
}

std::string hw_detail(const GrimGene &g, u32 fps) {
  char b[160];
  if (g.type == GrimGeneType::HwVram) {
    std::snprintf(b, sizeof(b), "%d cell%s · rows %d-%d", g.params[1], g.params[1] == 1 ? "" : "s",
                  g.params[2], g.params[2] + g.params[3]);
  } else {
    const unsigned lo = static_cast<unsigned>(g.params[2]) * 1024u;
    std::snprintf(b, sizeof(b), "%d cell%s · 0x%05X-0x%05X", g.params[1], g.params[1] == 1 ? "" : "s", lo,
                  lo + static_cast<unsigned>(g.params[3]) * 1024u);
  }
  std::string s = b;
  s += " · " + trigger_text(g.trigger, fps);
  if (g.params[7] > 0) {
    s += " · bus-sensitive";
  }
  if (grim_hw_gene_is_critical(g)) {
    s += " · DANGEROUS";
  }
  return s;
}

std::string pretty(const char *snake) {
  std::string s = snake;
  std::replace(s.begin(), s.end(), '_', ' ');
  if (!s.empty()) {
    s[0] = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
  }
  return s;
}

GrimGenome generate_one(u64 seed, const GrimPullSettings &settings, const GrimPullContext &ctx,
                        const GrimPullPlan &plan) {
  GrimGenome out;
  const u32 enabled = settings.families & grim_pull_available_families(ctx);
  struct Slot {
    u32 family;
    u32 weight;
  };
  std::vector<Slot> slots;
  if (enabled & kGrimFamilyAudio) slots.push_back({kGrimFamilyAudio, 4});
  if (enabled & kGrimFamilyVisual) slots.push_back({kGrimFamilyVisual, 3});
  if (enabled & kGrimFamilyCode) slots.push_back({kGrimFamilyCode, 3});
  if (enabled & kGrimFamilyHardware) slots.push_back({kGrimFamilyHardware, 3});
  if (slots.empty()) {
    return out;
  }
  u32 total = 0;
  for (const Slot &s : slots) total += s.weight;

  GrimRng pick{seed ^ 0x5851F42D4C957F2Dull};
  const u32 n = pick.range(plan.min_genes, plan.max_genes);
  u32 count_spu = 0, count_gpu = 0, count_sample = 0, count_code = 0, count_hw = 0;
  for (u32 i = 0; i < n; ++i) {
    u32 w = pick.range(0, total - 1);
    for (const Slot &s : slots) {
      if (w < s.weight) {
        if (s.family == kGrimFamilyAudio) {
          // Half of the audio genes rot the sound bank itself, when it was scanned.
          (ctx.sample != nullptr && pick.range(0, 1) == 0 ? count_sample : count_spu)++;
        } else {
          (s.family == kGrimFamilyVisual ? count_gpu : s.family == kGrimFamilyCode ? count_code : count_hw)++;
        }
        break;
      }
      w -= s.weight;
    }
  }

  const auto add_interface = [&](u32 count, bool spu, u64 salt) {
    GrimRandomParams p;
    p.spu = spu;
    p.gpu = !spu;
    p.min_genes = p.max_genes = count;
    p.risk_q10 = plan.risk_q10;
    p.rot_only = settings.rot;
    const GrimGenome g = grim_random_genome(grim_mix64(seed ^ salt), p);
    out.genes.insert(out.genes.end(), g.genes.begin(), g.genes.end());
  };
  if (count_spu > 0) {
    add_interface(count_spu, true, 1u);
  }
  if (count_gpu > 0) {
    add_interface(count_gpu, false, 6u);
  }
  if (count_sample > 0) {
    GrimRandomParams p;
    p.spu = p.gpu = false;
    p.sample = ctx.sample;
    p.sample_genes_min = p.sample_genes_max = count_sample;
    p.sample_sizing = {GrimSampleSizeKind::Milliseconds, plan.sample_ms};
    const GrimGenome g = grim_random_genome(grim_mix64(seed ^ 2u), p);
    out.genes.insert(out.genes.end(), g.genes.begin(), g.genes.end());
  }
  if (count_code > 0) {
    GrimRandomParams p;
    p.spu = p.gpu = false;
    p.rom = ctx.rom;
    p.rom_genes_min = p.rom_genes_max = count_code;
    p.rom_patches_max = plan.rom_patches_max;
    p.rom_early_ms = plan.rom_early_ms;
    p.rom_curve = plan.rom_curve;
    p.rom_call_swap = plan.rom_call_swap;
    const GrimGenome g = grim_random_genome(grim_mix64(seed ^ 3u), p);
    out.genes.insert(out.genes.end(), g.genes.begin(), g.genes.end());
  }
  if (count_hw > 0) {
    GrimRandomParams p;
    p.spu = p.gpu = false;
    p.hw_genes_min = p.hw_genes_max = count_hw;
    p.risk_q10 = plan.risk_q10;
    if (ctx.rom != nullptr) {
      p.hw_avoid_ram = &ctx.rom->map.ram_exec_ranges;
      p.hw_avoid_permille = plan.hw_avoid_code_permille;
    }
    p.rot_only = settings.rot;
    const GrimGenome g = grim_random_genome(grim_mix64(seed ^ 5u), p);
    out.genes.insert(out.genes.end(), g.genes.begin(), g.genes.end());
  }
  // A ROM gene that found nothing to change is not a gene.
  out.genes.erase(std::remove_if(out.genes.begin(), out.genes.end(),
                                 [](const GrimGene &g) {
                                   return grim_gene_is_rom(g.type) && g.patches.empty();
                                 }),
                  out.genes.end());
  if (out.genes.empty() && (enabled & (kGrimFamilyAudio | kGrimFamilyVisual))) {
    add_interface(1, (enabled & kGrimFamilyAudio) != 0, 4u);
  }
  if (grim_genome_has_rom(out)) {
    out.version = 2;
    out.bios_hash = ctx.bios_hash;
  }
  return out;
}
} // namespace

void GrimPullHistory::note(const GrimGenome &genome) {
  recent_.push_back(gene_keys(genome));
  while (recent_.size() > kRemembered) {
    recent_.pop_front();
  }
}

u32 GrimPullHistory::similarity(const GrimGenome &genome) const {
  const std::vector<u64> keys = gene_keys(genome);
  u32 score = 0;
  u32 weight = static_cast<u32>(recent_.size());
  for (auto it = recent_.rbegin(); it != recent_.rend(); ++it, --weight) { // newest first
    for (u64 k : keys) {
      score += weight * static_cast<u32>(std::count(it->begin(), it->end(), k));
    }
  }
  return score;
}

GrimGenome grim_pull_generate(u64 seed, const GrimPullSettings &settings,
                              const GrimPullContext &ctx, const GrimPullHistory *history,
                              u32 *family_mask_used) {
  const GrimPullPlan plan = grim_pull_plan(settings.intensity);
  const u32 candidates = history != nullptr && history->size() > 0 ? 4u : 1u;
  GrimGenome best;
  u32 best_score = ~0u;
  for (u32 c = 0; c < candidates; ++c) {
    GrimGenome g = generate_one(c == 0 ? seed : grim_mix64(seed + c), settings, ctx, plan);
    const u32 score = history != nullptr ? history->similarity(g) : 0u;
    if (score < best_score) {
      best_score = score;
      best = std::move(g);
    }
  }
  if (family_mask_used != nullptr) {
    u32 m = 0;
    for (const GrimGene &g : best.genes) {
      m |= grim_gene_family(g.type);
    }
    *family_mask_used = m;
  }
  return best;
}

bool grim_pull_compatible(const GrimGenome &genome, u64 bios_hash, std::string &err) {
  if (grim_genome_has_rom(genome) && genome.bios_hash != bios_hash) {
    char b[160];
    std::snprintf(b, sizeof(b),
                  "This machine was made for another BIOS (0x%016llX); the loaded one is 0x%016llX.",
                  static_cast<unsigned long long>(genome.bios_hash),
                  static_cast<unsigned long long>(bios_hash));
    err = b;
    return false;
  }
  return true;
}

std::string grim_machine_id(const GrimGenome &genome) {
  char b[16];
  std::snprintf(b, sizeof(b), "#%04X", static_cast<unsigned>(grim_genome_hash(genome) & 0xFFFFu));
  return b;
}

std::vector<GrimGeneLine> grim_pull_describe(const GrimGenome &genome,
                                             const GrimPullContext &ctx) {
  std::vector<GrimGeneLine> lines;
  for (const GrimGene &g : genome.genes) {
    GrimGeneLine l;
    l.domain = grim_gene_family(g.type);
    l.tag = grim_gene_is_rom(g.type) ? "ROM" : grim_gene_is_hardware(g.type) ? "HW" : "IFACE";
    char b[128];
    if (g.type == GrimGeneType::RomCode) {
      l.title = pretty(grim_rom_mut_name(static_cast<GrimRomMut>(g.params[0])));
      std::snprintf(b, sizeof(b), "%zu word%s", g.patches.size(), g.patches.size() == 1 ? "" : "s");
      l.detail = b;
      if (ctx.rom != nullptr && !g.patches.empty()) {
        const u32 w = g.patches.front().offset / 4u;
        if (w < ctx.rom->map.words.size() && ctx.rom->map.words[w].first_exec != kGrimNever) {
          std::snprintf(b, sizeof(b), "%zu word%s, first runs at %.1f s", g.patches.size(),
                        g.patches.size() == 1 ? "" : "s",
                        grim_cycles_to_ms(ctx.rom->map.words[w].first_exec) / 1000.0);
          l.detail = b;
        }
      }
    } else if (g.type == GrimGeneType::SpuSample) {
      l.title = pretty(grim_sample_mut_name(static_cast<GrimSampleMut>(g.params[0])));
      std::snprintf(b, sizeof(b), "sound bank #%d \xC2\xB7 %zu word%s", g.params[3], g.patches.size(),
                    g.patches.size() == 1 ? "" : "s");
      l.detail = b;
    } else if (grim_gene_is_hardware(g.type)) {
      l.title = hw_title(g);
      l.detail = hw_detail(g, ctx.frame_rate);
    } else {
      l.title = iface_title(g.type);
      l.detail = trigger_text(g.trigger, ctx.frame_rate);
    }
    lines.push_back(std::move(l));
  }
  return lines;
}
