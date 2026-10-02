#include "grim_genome.h"
#include <algorithm>
#include <cstring>

// Grim Reaper 2.0, Phase 5.1: the Faulty Hardware Simulator.
//
// Failing memory is modelled as cell faults that are re-applied at scanline and frame
// boundaries, never on the access path: the recompiler reads and writes RAM directly, and
// the Interpreter and Recompiler must see the same memory at the same scheduler points.
// A stuck bit is therefore "held" every eighth scanline (about 33 times a frame); data a
// game writes into a bad cell is wrong until the next tick. Everything is integer math.

namespace {
enum Mode : s32 { kStuckHigh = 0, kStuckLow = 1, kFlip = 2, kBurst = 3, kColumn = 4, kDecay = 5, kHammer = 6 };
constexpr s32 kVramDeadLine = 3;
constexpr u32 kRowBytes = 1024;
constexpr u32 kRamBytes = 2u * 1024u * 1024u;
constexpr u32 kSpuBytes = 512u * 1024u;

u32 rnd(u64 seed, u32 i, u32 k) {
  return static_cast<u32>(grim_mix64(seed ^ (u64{i} << 24) ^ (u64{k} * 0x9E3779B97F4A7C15ull)));
}

// `bits` bit positions out of `width` (8 or 16), never empty.
u16 bit_mask(u64 seed, u32 i, u32 bits, u32 width) {
  u16 m = 0;
  for (u32 k = 0; k < bits; ++k) {
    m = static_cast<u16>(m | (1u << (rnd(seed, i, 100u + k) % width)));
  }
  return m != 0 ? m : u16{1};
}

// Probability in 1/100000: rate (permille) scaled by trigger magnitude and bus load.
u32 chance(const GrimGene &g, u32 magnitude, u32 bus_load_q10) {
  const u32 load = static_cast<u32>(g.params[7]);
  u32 scale = 1024u; // 1.0
  if (load > 0u) {
    const u32 l = load * 1024u / 1000u;
    scale = (1024u - l) + static_cast<u32>(u64{l} * bus_load_q10 * 4u / 1024u);
  }
  const u64 p = u64{static_cast<u32>(g.params[5])} * 100u * magnitude / 1024u * scale / 1024u;
  return static_cast<u32>(std::min<u64>(p, 100000u));
}
bool roll(u64 r, u32 salt, u32 p) { return grim_mix64(r ^ (u64{salt} << 32)) % 100000u < p; }

u64 row_hash(const u8 *p) {
  u64 h = 0xCBF29CE484222325ull;
  for (u32 i = 0; i < kRowBytes; i += 8) {
    u64 w;
    std::memcpy(&w, p + i, 8);
    h = (h ^ w) * 0x100000001B3ull;
  }
  return h;
}
} // namespace

bool grim_hw_gene_is_critical(const GrimGene &g) {
  if (g.type == GrimGeneType::HwRam) {
    const u32 lo = static_cast<u32>(g.params[2]) * 1024u;
    const u32 hi = lo + static_cast<u32>(g.params[3]) * 1024u;
    return lo < kGrimRamKernelBytes || hi > kRamBytes - kGrimRamStackBytes;
  }
  if (g.type == GrimGeneType::HwSpuRam) {
    return static_cast<u32>(g.params[2]) * 1024u < kGrimSpuRamReservedBytes;
  }
  return false;
}

void GrimGenomeRuntime::build_hw_cells() {
  hw_cells_.assign(hw_genes_.size(), {});
  hw_rows_.assign(hw_genes_.size(), HwRows{});
  for (size_t k = 0; k < hw_genes_.size(); ++k) {
    const GrimGene &g = genome_.genes[hw_genes_[k]];
    const s32 mode = g.params[0];
    const u32 cells = static_cast<u32>(g.params[1]);
    const u32 lo = static_cast<u32>(g.params[2]);
    const u32 span = static_cast<u32>(g.params[3]);
    const u32 bits = static_cast<u32>(g.params[4]);
    const u32 arg = static_cast<u32>(g.params[6]);
    std::vector<HwCell> &out = hw_cells_[k];
    if (g.type == GrimGeneType::HwVram) {
      for (u32 i = 0; i < cells; ++i) {
        const u32 row = std::min(lo + rnd(g.seed, i, 0) % span, 511u);
        const u32 col = mode == kVramDeadLine ? 0u : rnd(g.seed, i, 1) % 1024u;
        out.push_back({row * 1024u + col, mode == kVramDeadLine ? u16{0xFFFF} : bit_mask(g.seed, i, bits, 16)});
      }
      continue;
    }
    const u32 limit = g.type == GrimGeneType::HwRam ? kRamBytes : kSpuBytes;
    const u32 base = std::min(lo * 1024u, limit - 1024u);
    const u32 size = std::min(span * 1024u, limit - base);
    for (u32 i = 0; i < cells; ++i) {
      if (mode == kColumn) {
        const u64 where = u64{base} + u64{i} * arg;
        if (where < u64{base} + size) {
          out.push_back({static_cast<u32>(where), bit_mask(g.seed, 0, bits, 8)}); // the same bit in every row
        }
      } else {
        out.push_back({base + rnd(g.seed, i, 0) % size, bit_mask(g.seed, i, bits, 8)});
      }
    }
  }
}

void GrimGenomeRuntime::apply_hardware(GrimHwTarget &t, bool frame_tick, u32 bus_load_q10) {
  for (size_t k = 0; k < hw_genes_.size(); ++k) {
    const size_t gi = hw_genes_[k];
    const GrimGene &g = genome_.genes[gi];
    const u64 frame_draw = grim_mix64(g.seed ^ (u64{frame_} << 8));
    const u32 m = grim_trigger_magnitude(g.trigger, frame_, frame_draw);
    if (m == 0u) {
      continue;
    }
    const s32 mode = g.params[0];
    const std::vector<HwCell> &cells = hw_cells_[k];
    // Faults appear in activation order as the trigger ramps up (rot = wear).
    const size_t active = std::min<size_t>(cells.size(), (cells.size() * m + 1023u) / 1024u);
    const bool vram = g.type == GrimGeneType::HwVram;
    const bool ram = g.type == GrimGeneType::HwRam;
    const bool stuck = mode == kStuckHigh || mode == kStuckLow || mode == kColumn ||
                       (vram && mode == kVramDeadLine);
    u64 changed = 0;

    if (stuck) {
      for (size_t i = 0; i < active; ++i) {
        const HwCell &c = cells[i];
        if (vram) {
          if (mode == kVramDeadLine) {
            for (u32 x = 0; x < 1024u; ++x) {
              if (t.vram_view()[c.where + x] != 0u) {
                t.vram_put(c.where + x, 0u);
                ++changed;
              }
            }
            continue;
          }
          const u16 old = t.vram_view()[c.where];
          const u16 now = mode == kStuckLow ? static_cast<u16>(old & ~c.mask) : static_cast<u16>(old | c.mask);
          if (now != old) {
            t.vram_put(c.where, now);
            ++changed;
          }
          continue;
        }
        const bool low = mode == kStuckLow || (mode == kColumn && (g.seed & 1u) != 0u);
        const u8 old = (ram ? t.ram_view() : t.spu_view())[c.where];
        const u8 now = low ? static_cast<u8>(old & ~c.mask) : static_cast<u8>(old | c.mask);
        if (now != old) {
          if (ram) {
            t.ram_put(c.where, now);
          } else {
            t.spu_put(c.where, now);
          }
          ++changed;
        }
      }
      hits_[gi] += changed;
      continue;
    }
    if (!frame_tick) {
      continue; // transient faults happen once per frame
    }
    const u32 p = chance(g, m, bus_load_q10);
    if (mode == kFlip) {
      for (size_t i = 0; i < active; ++i) {
        if (!roll(frame_draw, static_cast<u32>(i), p)) {
          continue;
        }
        const HwCell &c = cells[i];
        if (vram) {
          t.vram_put(c.where, static_cast<u16>(t.vram_view()[c.where] ^ c.mask));
        } else if (ram) {
          t.ram_put(c.where, static_cast<u8>(t.ram_view()[c.where] ^ c.mask));
        } else {
          t.spu_put(c.where, static_cast<u8>(t.spu_view()[c.where] ^ c.mask));
        }
        ++changed;
      }
    } else if (mode == kBurst && !vram && roll(frame_draw, 7u, p)) {
      const u32 limit = ram ? kRamBytes : kSpuBytes;
      const u32 base = std::min(static_cast<u32>(g.params[2]) * 1024u, limit - 1024u);
      const u32 size = std::min(static_cast<u32>(g.params[3]) * 1024u, limit - base);
      const u32 start = base + rnd(g.seed, frame_, 9) % size;
      const u32 len = std::min<u32>(static_cast<u32>(g.params[1]), base + size - start);
      for (u32 i = 0; i < len; ++i) {
        const u8 v = static_cast<u8>(rnd(g.seed, frame_, 200u + i));
        if (ram) {
          t.ram_put(start + i, v);
        } else {
          t.spu_put(start + i, v);
        }
      }
      changed += len;
    } else if ((mode == kDecay || mode == kHammer) && ram) {
      const u32 base = std::min(static_cast<u32>(g.params[2]) * 1024u, kRamBytes - 1024u);
      const u32 size = std::min(static_cast<u32>(g.params[3]) * 1024u, kRamBytes - base);
      const u32 rows = size / kRowBytes;
      HwRows &st = hw_rows_[k];
      if (rows == 0u) {
        continue;
      }
      const u8 *view = t.ram_view();
      if (!st.seeded) {
        st.hash.assign(rows, 0);
        st.stable.assign(rows, 0);
        st.hot.assign(rows, 0);
        for (u32 r = 0; r < rows; ++r) {
          st.hash[r] = row_hash(view + base + r * kRowBytes);
        }
        st.seeded = true;
        continue;
      }
      for (u32 r = 0; r < rows; ++r) {
        const u64 h = row_hash(view + base + r * kRowBytes);
        const bool same = h == st.hash[r];
        st.stable[r] = same ? static_cast<u16>(std::min<u32>(st.stable[r] + 1u, 65535u)) : u16{0};
        st.hot[r] = same ? u16{0} : static_cast<u16>(std::min<u32>(st.hot[r] + 1u, 65535u));
        st.hash[r] = h;
      }
      u32 budget = static_cast<u32>(g.params[1]);
      const u32 arg = static_cast<u32>(g.params[6]);
      const u32 first = rnd(g.seed, frame_, 3) % rows; // rotate so low rows are not always first
      for (u32 n = 0; n < rows && budget > 0u; ++n) {
        const u32 r = (first + n) % rows;
        if (mode == kDecay ? st.stable[r] < arg : st.hot[r] < arg) {
          continue;
        }
        if (!roll(frame_draw, 1000u + r, p)) {
          continue;
        }
        u32 target_row = r;
        if (mode == kHammer) { // the damage lands in a neighbouring row
          target_row = (rnd(g.seed, frame_, r) & 1u) != 0u ? r + 1u : (r > 0u ? r - 1u : 1u);
          if (target_row >= rows) {
            continue;
          }
        }
        const u32 off = base + target_row * kRowBytes + rnd(g.seed, frame_, 500u + r) % kRowBytes;
        const u8 mask = static_cast<u8>(bit_mask(g.seed, r, static_cast<u32>(g.params[4]), 8));
        const u8 old = view[off];
        const u8 now = mode == kHammer ? static_cast<u8>(old ^ mask)
                       : (g.seed & 2u) != 0u ? static_cast<u8>(old | mask)
                                             : static_cast<u8>(old & ~mask);
        if (now != old) {
          t.ram_put(off, now);
          st.hash[target_row] = row_hash(view + base + target_row * kRowBytes);
          ++changed;
          --budget;
        }
      }
    }
    hits_[gi] += changed;
  }
}

// ---- random hardware genes ---------------------------------------------------------------------

void grim_add_random_hw_genes(GrimGenome &genome, u64 seed, const GrimRandomParams &rp) {
  GrimRng r{grim_mix64(seed ^ 0x48575F4641554C54ull)};
  const u32 count = r.range(rp.hw_genes_min, std::max(rp.hw_genes_min, rp.hw_genes_max));
  const u32 risk = rp.risk_q10;
  const u32 horizon = std::max(120u, rp.horizon_frames);
  for (u32 n = 0; n < count; ++n) {
    const u32 pick = r.range(0, 9);
    const GrimGeneType type =
        pick < 5 ? GrimGeneType::HwRam : pick < 8 ? GrimGeneType::HwVram : GrimGeneType::HwSpuRam;
    GrimGene g = grim_default_gene(type);
    g.seed = r.next();
    const bool critical = r.range(0, 999) < rp.hw_critical_permille;
    s32 &mode = g.params[0];
    if (type == GrimGeneType::HwRam) {
      static const s32 kModes[] = {kStuckHigh, kStuckHigh, kStuckLow, kStuckLow, kFlip, kFlip,
                                   kBurst,     kColumn,    kColumn,   kDecay,    kDecay, kHammer};
      mode = kModes[r.range(0, 11)];
    } else if (type == GrimGeneType::HwSpuRam) {
      mode = static_cast<s32>(r.range(0, 4));
    } else {
      mode = static_cast<s32>(r.range(0, 3));
    }
    g.params[1] = static_cast<s32>(2 + r.range(0, 6 + risk * 120 / 1024));
    g.params[4] = static_cast<s32>(1 + r.range(0, 1 + risk / 512));
    g.params[5] = static_cast<s32>(5 + r.range(0, 45 + risk * 250 / 1024));
    g.params[7] = r.range(0, 99) < 30 ? static_cast<s32>(r.range(300, 1000)) : 0;
    const u32 span_cap = 128 + risk * 900 / 1024;
    if (type == GrimGeneType::HwRam || type == GrimGeneType::HwSpuRam) {
      const bool ram = type == GrimGeneType::HwRam;
      const u32 total_kb = ram ? 2048u : 512u;
      const u32 floor_kb = ram ? kGrimRamKernelBytes / 1024u : kGrimSpuRamReservedBytes / 1024u;
      const u32 ceil_kb = ram ? total_kb - kGrimRamStackBytes / 1024u : total_kb;
      u32 span = std::min(16u + r.range(0, span_cap), ram ? 1024u : 256u);
      span = std::min(span, ceil_kb - floor_kb);
      u32 lo = floor_kb + r.range(0, ceil_kb - floor_kb - span);
      if (critical) { // the rare unlucky chip: kernel/stack (RAM) or decode buffers (SPU)
        if (ram) {
          lo = r.range(0, 1) == 1 ? total_kb - span : r.range(0, 63);
        } else {
          lo = 0;
        }
      }
      g.params[2] = static_cast<s32>(lo);
      g.params[3] = static_cast<s32>(span);
    } else {
      const u32 span = std::min(16u + r.range(0, 400u), 512u);
      g.params[2] = static_cast<s32>(r.range(0, 512u - span));
      g.params[3] = static_cast<s32>(span);
    }
    if (mode == kColumn && type != GrimGeneType::HwVram) {
      static const s32 kStrides[] = {16, 64, 256, 1024, 4096};
      g.params[6] = kStrides[r.range(0, 4)];
      g.params[1] = static_cast<s32>(8 + r.range(0, 56 + risk / 8));
    } else if (mode == kDecay && type == GrimGeneType::HwRam) {
      g.params[6] = static_cast<s32>(120 + r.range(0, 780));
      g.params[5] = static_cast<s32>(100 + r.range(0, 500));
    } else if (mode == kHammer && type == GrimGeneType::HwRam) {
      g.params[6] = static_cast<s32>(4 + r.range(0, 26));
      g.params[5] = static_cast<s32>(100 + r.range(0, 500));
    } else if (mode == kBurst && type != GrimGeneType::HwVram) {
      g.params[5] = static_cast<s32>(20 + r.range(0, 180));
    }
    // Wear is the point of failing hardware: ramps are the most common trigger.
    GrimTrigger &t = g.trigger;
    const u32 shape = r.range(0, 99);
    if (rp.rot_only || shape < 40) {
      t.kind = GrimTriggerKind::Rot;
      t.start_frame = 120u + r.range(0, horizon / 2u);
      t.end_frame = t.start_frame + std::max(300u, r.range(120, horizon));
    } else if (shape < 70) {
      t.kind = GrimTriggerKind::Always;
    } else if (shape < 85) {
      t.kind = GrimTriggerKind::Window;
      t.start_frame = r.range(0, horizon * 3u / 4u);
      t.end_frame = t.start_frame + r.range(120, std::max(240u, horizon / 2u));
    } else {
      t.kind = GrimTriggerKind::Intermittent;
      t.period = r.range(60, 600);
      t.duty = r.range(10, std::max(11u, t.period / 2u));
    }
    genome.genes.push_back(g);
  }
}
