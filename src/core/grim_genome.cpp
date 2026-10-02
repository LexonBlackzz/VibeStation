#include "grim_genome.h"
#include "bios.h"
#include "grim_rom.h"
#include "grim_sample.h"
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <nlohmann/json.hpp>

namespace {

// ---- schema -----------------------------------------------------------------

constexpr const char *kTypeNames[] = {
    "spu_pitch",  "spu_adsr",   "spu_volume",   "spu_address",  "spu_keyon",
    "spu_noise",  "spu_pmon",   "spu_reverb",   "gpu_vertex",   "gpu_color",
    "gpu_flags",  "gpu_texparam", "gpu_state",  "gpu_fill",  "rom_code", "spu_sample",
    "hw_ram",     "hw_vram",    "hw_spuram"};
static_assert(sizeof(kTypeNames) / sizeof(kTypeNames[0]) ==
                  static_cast<size_t>(GrimGeneType::Count),
              "type name table out of sync");

// Same order as GrimGeneType. Parameter meaning is documented in PROGRESS.md.
const std::vector<GrimParamSpec> kSchemas[] = {
    // spu_pitch: mode 0 multiply, 1 offset, 2 quantize to scale, 3 wobble
    {{"mode", 0, 3, 0},
     {"mul_q8", 16, 1024, 384},
     {"offset", -4096, 4096, 256},
     {"scale", 1, 4095, 0xAB5},
     {"depth", 0, 1024, 64},
     {"period", 1, 600, 120}},
    // spu_adsr: mode 0 infinite sustain, 1 instant release, 2 attack, 3 decay,
    //           4 sustain level, 5 release (2-5 flip random bits of that field)
    {{"mode", 0, 5, 0}},
    // spu_volume: mode 0 swap L/R, 1 invert sign, 2 clamp, 3 sweep
    {{"mode", 0, 3, 0},
     {"clamp", 0, 16383, 4096},
     {"depth", 0, 1024, 512},
     {"period", 1, 600, 120}},
    // spu_address: which 0 start, 1 repeat, 2 both; blocks in 8-byte units
    {{"which", 0, 2, 0}, {"blocks", -4096, 4096, 16}, {"random", 0, 1, 0}},
    // spu_keyon: mode 0 drop, 1 duplicate, 2 delay; keys 0 key-on, 1 key-off,
    //            2 both; prob permille per voice bit; delay in SPU samples
    {{"mode", 0, 2, 0},
     {"keys", 0, 2, 0},
     {"prob", 0, 1000, 300},
     {"delay", 1, 8192, 200}},
    // spu_noise / spu_pmon: piggyback 1 = also force the bits on every key-on
    {{"piggyback", 0, 1, 1}},
    {{"piggyback", 0, 1, 1}},
    // spu_reverb
    {{"cfg", 0, 1000, 200},
     {"base", -4096, 4096, 0},
     {"eon", 0, 1, 0},
     {"spucnt", 0, 1, 0}},
    // gpu_vertex: mode 0 fixed, 1 noise, 2 drift, 3 snap, 4 swap
    {{"mode", 0, 4, 1},
     {"amp", 0, 1023, 8},
     {"dx", -1023, 1023, 4},
     {"dy", -1023, 1023, 4},
     {"grid", 1, 256, 8},
     {"wrap", 0, 1, 0},
     {"prob", 0, 1000, 500}},
    // gpu_color: mode 0 channel permute, 1 invert, 2 gradient shuffle, 3 tint
    {{"mode", 0, 3, 0}, {"perm", 0, 6, 6}, {"tint", 0, 0xFFFFFF, 0x3F3F3F}},
    // gpu_flags: semi/raw 0 keep, 1 set, 2 clear, 3 toggle
    {{"semi", 0, 3, 3}, {"raw", 0, 3, 0}, {"prob", 0, 1000, 500}},
    // gpu_texparam: which 0 clut, 1 texpage, 2 both
    {{"which", 0, 2, 0},
     {"clut_mask", 0, 65535, 0x3F},
     {"tp_mask", 0, 65535, 0xF},
     {"prob", 0, 1000, 500}},
    // gpu_state: E1/E2/E6 flip masked bits; E3/E4/E5 jitter by amp pixels and
    //            drift by `drift` pixels per 60 frames
    {{"amp", 0, 255, 4},
     {"drift", -255, 255, 0},
     {"e1_mask", 0, 0x3FFF, 0x1F},
     {"e2_mask", 0, 0xFFFFF, 0x1F},
     {"e6_mask", 0, 3, 1}},
    // gpu_fill
    {{"color_mask", 0, 0xFFFFFF, 0x3F3F3F}, {"amp", 0, 511, 16}},
    // rom_code: generating parameters (the resolved patches are stored beside
    //           them). kind = GrimRomMut, early_ms = first-execution window that
    //           is never touched, curve = how strongly later code is preferred
    {{"kind", 0, 8, 0}, {"count", 1, 4096, 1}, {"early_ms", 0, 60000, 200}, {"curve", 0, 3, 2}},
    // spu_sample: resolved word patches; shift 13..15 is tagged separately.
    {{"kind", 0, 9, 0}, {"count", 1, 256, 1}, {"magnitude", 1, 15, 1},
     {"sample", -1, 32767, -1}, {"donor", -1, 32767, -1}, {"emulator_shift", 0, 1, 0},
     {"block_phase", 0, 8, 0}},
    // hw_ram: mode 0 stuck high, 1 stuck low, 2 flip, 3 burst, 4 bad column, 5 thermal
    //         decay, 6 rowhammer. cells = failing bytes (burst: bytes per burst, decay and
    //         hammer: rows per frame). Region = lo_kb..lo_kb+span_kb. rate = permille chance
    //         per event. arg = column stride in bytes, decay: frames a row must sit unchanged,
    //         hammer: frames a row must change in a row. load = permille sensitivity to bus load.
    {{"mode", 0, 6, 0}, {"cells", 1, 4096, 8}, {"lo_kb", 0, 2047, 256}, {"span_kb", 1, 2048, 64},
     {"bits", 1, 8, 1}, {"rate", 0, 1000, 50}, {"arg", 1, 65535, 64}, {"load", 0, 1000, 0}},
    // hw_vram: mode 0 stuck high, 1 stuck low, 2 flip, 3 dead line (row forced to black).
    //          Region = rows row_lo..row_lo+row_span; bits are of the 16-bit pixel.
    {{"mode", 0, 3, 0}, {"cells", 1, 4096, 8}, {"row_lo", 0, 511, 0}, {"row_span", 1, 512, 512},
     {"bits", 1, 16, 1}, {"rate", 0, 1000, 50}, {"arg", 1, 65535, 64}, {"load", 0, 1000, 0}},
    // hw_spuram: mode 0 stuck high, 1 stuck low, 2 flip, 3 burst, 4 bad column.
    {{"mode", 0, 4, 0}, {"cells", 1, 4096, 8}, {"lo_kb", 0, 511, 16}, {"span_kb", 1, 512, 128},
     {"bits", 1, 8, 1}, {"rate", 0, 1000, 50}, {"arg", 1, 65535, 64}, {"load", 0, 1000, 0}},
};
static_assert(sizeof(kSchemas) / sizeof(kSchemas[0]) ==
                  static_cast<size_t>(GrimGeneType::Count),
              "schema table out of sync");

bool is_spu_type(GrimGeneType t) { return t <= GrimGeneType::SpuReverb; }

const char *kTriggerNames[] = {"always", "window", "rot", "intermittent"};

// ---- small helpers -----------------------------------------------------------

constexpr u64 kFnvOffset = 14695981039346656037ull;
constexpr u64 kFnvPrime = 1099511628211ull;

u64 fnv1a(const std::string &s) {
  u64 h = kFnvOffset;
  for (const char c : s) {
    h = (h ^ static_cast<u8>(c)) * kFnvPrime;
  }
  return h;
}

// a + (b - a) * m / 1024 (m is a q10 magnitude).
s32 lerp_q10(s32 a, s32 b, u32 m) {
  return a + static_cast<s32>((static_cast<s64>(b) - a) * static_cast<s64>(m) / 1024);
}
s32 scale_q10(s32 v, u32 m) {
  return static_cast<s32>(static_cast<s64>(v) * static_cast<s64>(m) / 1024);
}
s32 clamp_s32(s32 v, s32 lo, s32 hi) { return v < lo ? lo : v > hi ? hi : v; }

// Bit i of `mask` survives when the mix of (r, i) falls under the magnitude:
// XOR masks scale by keeping fewer of their bits.
u32 scale_mask(u32 mask, u32 m, u64 r) {
  u32 out = 0;
  for (u32 i = 0; i < 32; ++i) {
    if (((mask >> i) & 1u) != 0u && (grim_mix64(r + i) & 1023u) < m) {
      out |= 1u << i;
    }
  }
  return out;
}

// True with probability m/1024, decided by (r, salt).
bool pick(u64 r, u32 salt, u32 m) { return (grim_mix64(r ^ (u64{salt} << 40)) & 1023u) < m; }

// Triangle wave in q10: -1024..1024 over `period` frames.
s32 triangle_q10(u32 frame, u32 period) {
  const u32 t = static_cast<u32>((u64{frame % period} * 4096u) / period);
  return t < 1024u ? static_cast<s32>(t)
         : t < 3072u ? static_cast<s32>(2048u - t)
                     : static_cast<s32>(t) - 4096;
}

s32 sext11(u32 v) { return static_cast<s32>((v & 0x7FFu) ^ 0x400u) - 0x400; }

// Voice bits of a 16-bit register half: low half = voices 0-15, high = 16-23.
u32 half_mask(u32 target, bool high) {
  return high ? ((target >> 16) & 0xFFu) : (target & 0xFFFFu);
}

// Quantizes an SPU pitch value to the nearest allowed semitone (mask bit k =
// semitone k of the octave, root = 0x1000).
u32 quantize_pitch(u32 pitch, u32 scale_mask_bits) {
  static const u32 kRatio[13] = {4096, 4340, 4598, 4871, 5161, 5468, 5793,
                                 6137, 6502, 6889, 7298, 7732, 8192};
  if (pitch == 0u || scale_mask_bits == 0u) {
    return pitch;
  }
  s32 octave = 0;
  u32 x = pitch;
  while (x >= 8192u) {
    x >>= 1;
    ++octave;
  }
  while (x < 4096u) {
    x <<= 1;
    --octave;
  }
  u32 best = 0;
  u32 best_dist = 0xFFFFFFFFu;
  for (u32 k = 0; k <= 12u; ++k) {
    if (((scale_mask_bits >> (k % 12u)) & 1u) == 0u) {
      continue;
    }
    const u32 d = x > kRatio[k] ? x - kRatio[k] : kRatio[k] - x;
    if (d < best_dist) {
      best_dist = d;
      best = kRatio[k];
    }
  }
  return octave >= 0 ? best << octave : best >> (-octave);
}

// ---- GP0 layout ---------------------------------------------------------------

constexpr u32 kClassPoly = 1, kClassLine = 2, kClassRect = 4;

struct Gp0Layout {
  u32 klass = 0; // kClass*
  bool textured = false;
  bool gouraud = false;
  size_t nv = 0, nc = 0, nuv = 0;
  size_t vw[4]{}, cw[4]{}, uw[4]{};
};

// Where the vertex, color and UV words of a drawing command sit. Returns false
// for anything else (fill, state, transfers, NOPs) or a short buffer.
bool gp0_layout(const u32 *w, size_t count, Gp0Layout &L) {
  const u32 op = w[0] >> 24;
  L = Gp0Layout{};
  if (op >= 0x20u && op <= 0x3Fu) {
    L.klass = kClassPoly;
    L.gouraud = (op & 0x10u) != 0u;
    L.textured = (op & 0x04u) != 0u;
    L.nv = (op & 0x08u) != 0u ? 4u : 3u;
    const size_t tex = L.textured ? 1u : 0u;
    if (L.gouraud) {
      const size_t stride = 2u + tex;
      L.nc = L.nv;
      for (size_t i = 0; i < L.nv; ++i) {
        L.cw[i] = i * stride;
        L.vw[i] = i * stride + 1u;
        L.uw[i] = i * stride + 2u;
      }
    } else {
      L.nc = 1;
      for (size_t i = 0; i < L.nv; ++i) {
        L.vw[i] = 1u + i * (1u + tex);
        L.uw[i] = 2u + i * 2u;
      }
    }
    L.nuv = L.textured ? L.nv : 0u;
    const size_t last = L.textured ? L.uw[L.nv - 1u] : L.vw[L.nv - 1u];
    return last < count;
  }
  if (op >= 0x40u && op <= 0x5Fu) {
    L.klass = kClassLine;
    L.gouraud = (op & 0x10u) != 0u;
    L.nv = 2;
    if (L.gouraud) {
      L.nc = 2;
      L.cw[0] = 0;
      L.vw[0] = 1;
      L.cw[1] = 2;
      L.vw[1] = 3;
      return count >= 4u;
    }
    L.nc = 1;
    L.vw[0] = 1;
    L.vw[1] = 2;
    return count >= 3u;
  }
  if (op >= 0x60u && op <= 0x7Fu) {
    L.klass = kClassRect;
    L.textured = (op & 0x04u) != 0u;
    L.nv = 1;
    L.nc = 1;
    L.vw[0] = 1;
    L.nuv = L.textured ? 1u : 0u;
    L.uw[0] = 2;
    return count >= (L.textured ? 3u : 2u);
  }
  return false;
}

u32 set_xy(u32 word, s32 x, s32 y, bool wrap) {
  if (wrap) {
    x = sext11(static_cast<u32>(x));
    y = sext11(static_cast<u32>(y));
  } else {
    x = clamp_s32(x, -1024, 1023);
    y = clamp_s32(y, -1024, 1023);
  }
  return (word & ~0x07FF07FFu) | (static_cast<u32>(x) & 0x7FFu) |
         ((static_cast<u32>(y) & 0x7FFu) << 16);
}

// Modes 0-3 of gpu_vertex on one vertex word. `salt` separates the vertices of
// one primitive so they get different noise.
u32 xform_vertex(const GrimGene &g, u32 word, u64 r, u32 salt, u32 m, u32 frame) {
  const auto &p = g.params;
  s32 x = sext11(word);
  s32 y = sext11(word >> 16);
  switch (p[0]) {
  case 0:
    x += scale_q10(p[2], m);
    y += scale_q10(p[3], m);
    break;
  case 1: {
    const u64 h = grim_mix64(r + salt);
    const u32 span = static_cast<u32>(p[1]) * 2u + 1u;
    x += scale_q10(static_cast<s32>((h & 0xFFFFu) % span) - p[1], m);
    y += scale_q10(static_cast<s32>(((h >> 16) & 0xFFFFu) % span) - p[1], m);
    break;
  }
  case 2:
    x += scale_q10(static_cast<s32>(static_cast<s64>(p[2]) * frame / 60), m);
    y += scale_q10(static_cast<s32>(static_cast<s64>(p[3]) * frame / 60), m);
    break;
  case 3: {
    const auto snap = [&](s32 v) {
      const s32 gsz = p[4];
      const s32 a = v + gsz / 2;
      s32 q = a / gsz;
      if (a < 0 && (a % gsz) != 0) {
        --q; // floor division for negative coordinates
      }
      return q * gsz;
    };
    x = lerp_q10(x, snap(x), m);
    y = lerp_q10(y, snap(y), m);
    break;
  }
  default:
    return word;
  }
  return set_xy(word, x, y, p[5] != 0);
}

// Modes 0, 1, 3 of gpu_color on one color word (low 24 bits; the top byte,
// which may hold the opcode, is preserved).
u32 xform_color(const GrimGene &g, u32 word, u64 r, u32 m) {
  static const u8 kPerm[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
                                 {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
  const auto &p = g.params;
  const u32 in[3] = {word & 0xFFu, (word >> 8) & 0xFFu, (word >> 16) & 0xFFu};
  u32 out[3] = {in[0], in[1], in[2]};
  if (p[0] == 0) {
    const u32 pi = p[1] < 6 ? static_cast<u32>(p[1]) : static_cast<u32>((r >> 8) % 6u);
    for (int k = 0; k < 3; ++k) {
      out[k] = in[kPerm[pi][k]];
    }
  } else if (p[0] == 1) {
    for (int k = 0; k < 3; ++k) {
      out[k] = 255u - in[k];
    }
  } else if (p[0] == 3) {
    for (int k = 0; k < 3; ++k) {
      out[k] = in[k] ^ ((static_cast<u32>(p[2]) >> (8 * k)) & 0xFFu);
    }
  }
  u32 res = word & 0xFF000000u;
  for (int k = 0; k < 3; ++k) {
    res |= static_cast<u32>(lerp_q10(static_cast<s32>(in[k]), static_cast<s32>(out[k]), m))
           << (8 * k);
  }
  return res;
}

bool is_polyline_terminator(u32 word) { return (word & 0xF000F000u) == 0x50005000u; }

// 0 keep, 1 set, 2 clear, 3 toggle applied to one bit.
u32 apply_bit_op(u32 word, u32 bit, s32 op) {
  switch (op) {
  case 1:
    return word | bit;
  case 2:
    return word & ~bit;
  case 3:
    return word ^ bit;
  default:
    return word;
  }
}

} // namespace

// ---- names / schema accessors ----------------------------------------------------

const char *grim_gene_type_name(GrimGeneType t) {
  return kTypeNames[static_cast<size_t>(t)];
}

const std::vector<GrimParamSpec> &grim_gene_schema(GrimGeneType t) {
  return kSchemas[static_cast<size_t>(t)];
}

u32 grim_gene_default_target(GrimGeneType t) {
  if (is_spu_type(t)) {
    return 0xFFFFFFu;
  }
  switch (t) {
  case GrimGeneType::RomCode:
  case GrimGeneType::SpuSample:
  case GrimGeneType::HwRam:
  case GrimGeneType::HwVram:
  case GrimGeneType::HwSpuRam:
    return 1u;
  case GrimGeneType::GpuState:
    return 0x3Fu;
  case GrimGeneType::GpuFill:
    return 1u;
  default:
    return 7u;
  }
}

GrimGene grim_default_gene(GrimGeneType t) {
  GrimGene g;
  g.type = t;
  g.target = grim_gene_default_target(t);
  const auto &schema = grim_gene_schema(t);
  for (size_t i = 0; i < schema.size(); ++i) {
    g.params[i] = schema[i].def;
  }
  return g;
}

// ---- trigger -------------------------------------------------------------------------

u32 grim_trigger_magnitude(const GrimTrigger &t, u32 frame, u64 rng_word) {
  switch (t.kind) {
  case GrimTriggerKind::Always:
    return 1024u;
  case GrimTriggerKind::Window:
    return frame >= t.start_frame && frame < t.end_frame ? 1024u : 0u;
  case GrimTriggerKind::Rot:
    if (frame <= t.start_frame) {
      return 0u;
    }
    if (frame >= t.end_frame) {
      return 1024u;
    }
    return static_cast<u32>(u64{frame - t.start_frame} * 1024u /
                            (t.end_frame - t.start_frame));
  case GrimTriggerKind::Intermittent:
    if (frame < t.start_frame || (t.end_frame != 0u && frame >= t.end_frame)) {
      return 0u;
    }
    if (t.period > 0u) {
      return ((frame - t.start_frame) % t.period) < t.duty ? 1024u : 0u;
    }
    return (rng_word % 1000u) < t.probability ? 1024u : 0u;
  }
  return 0u;
}

// ---- JSON ------------------------------------------------------------------------------

namespace {
using json = nlohmann::json;

bool fail(std::string &err, const std::string &msg) {
  err = msg;
  return false;
}

bool get_uint(const json &o, const char *key, u64 lo, u64 hi, u64 &out,
              const std::string &ctx, std::string &err) {
  const auto it = o.find(key);
  if (it == o.end()) {
    return fail(err, ctx + ": missing field \"" + key + "\"");
  }
  if (!(it->is_number_unsigned() || (it->is_number_integer() && it->get<s64>() >= 0))) {
    return fail(err, ctx + ": \"" + key + "\" must be a non-negative integer");
  }
  out = it->get<u64>();
  if (out < lo || out > hi) {
    return fail(err, ctx + ": \"" + key + "\" out of range [" + std::to_string(lo) + ", " +
                         std::to_string(hi) + "]");
  }
  return true;
}

// The object must have exactly these keys.
bool check_keys(const json &o, std::initializer_list<const char *> keys,
                const std::string &ctx, std::string &err) {
  if (!o.is_object()) {
    return fail(err, ctx + ": expected an object");
  }
  for (const char *k : keys) {
    if (o.find(k) == o.end()) {
      return fail(err, ctx + ": missing field \"" + k + "\"");
    }
  }
  for (auto it = o.begin(); it != o.end(); ++it) {
    bool known = false;
    for (const char *k : keys) {
      known = known || it.key() == k;
    }
    if (!known) {
      return fail(err, ctx + ": unknown field \"" + it.key() + "\"");
    }
  }
  return true;
}

bool parse_trigger(const json &j, GrimTrigger &t, const std::string &ctx, std::string &err) {
  if (!j.is_object()) {
    return fail(err, ctx + ": trigger must be an object");
  }
  const auto kit = j.find("kind");
  if (kit == j.end() || !kit->is_string()) {
    return fail(err, ctx + ": trigger.kind must be a string");
  }
  const std::string kind = kit->get<std::string>();
  const std::string c = ctx + " trigger";
  t = GrimTrigger{};
  u64 v = 0;
  if (kind == "always") {
    t.kind = GrimTriggerKind::Always;
    return check_keys(j, {"kind"}, c, err);
  }
  if (kind == "window" || kind == "rot") {
    t.kind = kind == "window" ? GrimTriggerKind::Window : GrimTriggerKind::Rot;
    if (!check_keys(j, {"kind", "start_frame", "end_frame"}, c, err) ||
        !get_uint(j, "start_frame", 0, 0xFFFFFFFFull, v, c, err)) {
      return false;
    }
    t.start_frame = static_cast<u32>(v);
    if (!get_uint(j, "end_frame", 1, 0xFFFFFFFFull, v, c, err)) {
      return false;
    }
    t.end_frame = static_cast<u32>(v);
    if (t.end_frame <= t.start_frame) {
      return fail(err, c + ": end_frame must be greater than start_frame");
    }
    return true;
  }
  if (kind == "intermittent") {
    t.kind = GrimTriggerKind::Intermittent;
    if (!check_keys(j, {"kind", "start_frame", "end_frame", "period", "duty", "probability"},
                    c, err)) {
      return false;
    }
    u64 start = 0, end = 0, period = 0, duty = 0, prob = 0;
    if (!get_uint(j, "start_frame", 0, 0xFFFFFFFFull, start, c, err) ||
        !get_uint(j, "end_frame", 0, 0xFFFFFFFFull, end, c, err) ||
        !get_uint(j, "period", 0, 0xFFFFFFFFull, period, c, err) ||
        !get_uint(j, "duty", 0, 0xFFFFFFFFull, duty, c, err) ||
        !get_uint(j, "probability", 0, 1000, prob, c, err)) {
      return false;
    }
    if (end != 0 && end <= start) {
      return fail(err, c + ": end_frame must be 0 (forever) or greater than start_frame");
    }
    if (period > 0) {
      if (duty < 1 || duty > period || prob != 0) {
        return fail(err, c + ": pattern needs 1 <= duty <= period and probability 0");
      }
    } else if (duty != 0 || prob < 1) {
      return fail(err, c + ": period 0 means probability mode: duty 0, probability 1..1000");
    }
    t.start_frame = static_cast<u32>(start);
    t.end_frame = static_cast<u32>(end);
    t.period = static_cast<u32>(period);
    t.duty = static_cast<u32>(duty);
    t.probability = static_cast<u32>(prob);
    return true;
  }
  return fail(err, ctx + ": unknown trigger kind \"" + kind + "\"");
}

bool parse_rom_gene(const json &j, size_t index, GrimGeneType type, GrimGene &g, std::string &err) {
  const std::string ctx = "gene " + std::to_string(index);
  const bool sized = type == GrimGeneType::SpuSample && j.is_object() && j.contains("sizing");
  if (!(sized ? check_keys(j, {"type", "seed", "params", "patches", "sizing"}, ctx, err)
              : check_keys(j, {"type", "seed", "params", "patches"}, ctx, err))) {
    return false;
  }
  g = GrimGene{};
  g.type = type;
  g.target = 1;
  u64 v = 0;
  if (!get_uint(j, "seed", 0, ~0ull, v, ctx, err)) {
    return false;
  }
  g.seed = v;
  const auto &schema = grim_gene_schema(g.type);
  const json &pj = j["params"];
  if (!pj.is_object() || pj.size() != schema.size()) {
    return fail(err, ctx + ": " + grim_gene_type_name(type) + " params must be an object with exactly " +
                         std::to_string(schema.size()) + " entries");
  }
  for (size_t i = 0; i < schema.size(); ++i) {
    const auto it = pj.find(schema[i].name);
    if (it == pj.end() || !it->is_number_integer()) {
      return fail(err, ctx + ": param \"" + schema[i].name + "\" must be an integer");
    }
    const s64 val = it->get<s64>();
    if (val < schema[i].lo || val > schema[i].hi) {
      return fail(err, ctx + ": param \"" + schema[i].name + "\" out of range");
    }
    g.params[i] = static_cast<s32>(val);
  }
  if (type == GrimGeneType::SpuSample && g.params[6] != 0 && g.params[6] != 8) {
    return fail(err, ctx + ": block_phase must be 0 or 8 (SPU addresses are 8-byte aligned)");
  }
  if (sized) {
    const json &sj = j["sizing"];
    const std::string sctx = ctx + " sizing";
    if (!grim_sample_window_gene(static_cast<GrimSampleMut>(g.params[0]))) {
      return fail(err, sctx + ": loop edits do not have a window");
    }
    if (sj.is_object() && sj.contains("milliseconds")) {
      if (!check_keys(sj, {"milliseconds"}, sctx, err) ||
          !get_uint(sj, "milliseconds", 1, 60000, v, sctx, err)) return false;
      g.sample_sizing = {GrimSampleSizeKind::Milliseconds, static_cast<u32>(v)};
    } else if (sj.is_object() && sj.contains("fraction_permille")) {
      if (!check_keys(sj, {"fraction_permille"}, sctx, err) ||
          !get_uint(sj, "fraction_permille", 1, 1000, v, sctx, err)) return false;
      g.sample_sizing = {GrimSampleSizeKind::FractionPermille, static_cast<u32>(v)};
    } else {
      return fail(err, sctx + ": expected milliseconds or fraction_permille");
    }
  }
  if (!j["patches"].is_array()) {
    return fail(err, ctx + ": patches must be an array");
  }
  for (size_t k = 0; k < j["patches"].size(); ++k) {
    const json &p = j["patches"][k];
    const std::string pctx = ctx + " patch " + std::to_string(k);
    if (!p.is_array() || p.size() != 4) {
      return fail(err, pctx + ": expected [offset, original, mutated, delay_slot]");
    }
    for (int f = 0; f < 4; ++f) {
      if (!(p[f].is_number_unsigned() || (p[f].is_number_integer() && p[f].get<s64>() >= 0))) {
        return fail(err, pctx + ": values must be non-negative integers");
      }
    }
    GrimRomPatch patch;
    const u64 off = p[0].get<u64>(), orig = p[1].get<u64>(), mut = p[2].get<u64>(),
              ds = p[3].get<u64>();
    if ((off & 3u) != 0u || off >= 0x100000u || orig > 0xFFFFFFFFull || mut > 0xFFFFFFFFull ||
        ds > 1u) {
      return fail(err, pctx + ": offset must be word aligned and inside the ROM, words 32 bit");
    }
    patch.offset = static_cast<u32>(off);
    patch.original = static_cast<u32>(orig);
    patch.mutated = static_cast<u32>(mut);
    patch.delay_slot = ds != 0u;
    if (type == GrimGeneType::SpuSample && patch.delay_slot) {
      return fail(err, pctx + ": sample patches cannot be instruction delay slots");
    }
    if (type == GrimGeneType::SpuSample) {
      const GrimSampleMut kind = static_cast<GrimSampleMut>(g.params[0]);
      const u32 changed = patch.original ^ patch.mutated;
      if (grim_sample_header_gene(kind)) {
        const u32 allowed = kind == GrimSampleMut::FilterSwap ? 0x70u
                            : kind == GrimSampleMut::ShiftChange ? 0xFu : 0x700u;
        if ((patch.offset & 15u) != static_cast<u32>(g.params[6]) || (changed & ~allowed) != 0u) {
          return fail(err, pctx + ": header gene must edit only its field at an aligned block header");
        }
      } else if ((patch.offset & 15u) == static_cast<u32>(g.params[6]) && (changed & 0xFFFFu) != 0u) {
        return fail(err, pctx + ": payload gene must preserve both ADPCM header bytes");
      }
      if (kind == GrimSampleMut::ShiftChange && (patch.mutated & 15u) >= 13u &&
          g.params[5] == 0) {
        return fail(err, pctx + ": shift 13..15 must carry emulator_shift=1");
      }
    }
    g.patches.push_back(patch);
  }
  if (type == GrimGeneType::SpuSample && g.params[0] != static_cast<s32>(GrimSampleMut::ShiftChange) &&
      g.params[5] != 0) {
    return fail(err, ctx + ": emulator_shift is only valid for a shift_change gene");
  }
  return true;
}

bool parse_gene(const json &j, size_t index, GrimGene &g, std::string &err) {
  if (j.is_object() && j.contains("type") && j["type"].is_string()) {
    const std::string type = j["type"].get<std::string>();
    if (type == "rom_code" || type == "spu_sample") {
      return parse_rom_gene(j, index, type == "rom_code" ? GrimGeneType::RomCode
                                                        : GrimGeneType::SpuSample, g, err);
    }
  }
  const std::string ctx = "gene " + std::to_string(index);
  if (!check_keys(j, {"type", "target", "seed", "trigger", "params"}, ctx, err)) {
    return false;
  }
  if (!j["type"].is_string()) {
    return fail(err, ctx + ": type must be a string");
  }
  const std::string name = j["type"].get<std::string>();
  size_t type_index = static_cast<size_t>(GrimGeneType::Count);
  for (size_t i = 0; i < static_cast<size_t>(GrimGeneType::Count); ++i) {
    if (name == kTypeNames[i]) {
      type_index = i;
    }
  }
  if (type_index == static_cast<size_t>(GrimGeneType::Count)) {
    return fail(err, ctx + ": unknown gene type \"" + name + "\"");
  }
  g = GrimGene{};
  g.type = static_cast<GrimGeneType>(type_index);
  u64 v = 0;
  if (!get_uint(j, "target", 0, 0xFFFFFFFFull, v, ctx, err)) {
    return false;
  }
  g.target = static_cast<u32>(v);
  const u32 allowed = is_spu_type(g.type) ? 0x1FFFFFFu
                      : g.type == GrimGeneType::GpuState ? 0x3Fu
                      : g.type == GrimGeneType::GpuFill  ? 1u
                                                          : 7u;
  if ((g.target & ~allowed) != 0u || g.target == 0u) {
    return fail(err, ctx + ": target must be a non-zero subset of mask " +
                         std::to_string(allowed));
  }
  if (!get_uint(j, "seed", 0, ~0ull, v, ctx, err)) {
    return false;
  }
  g.seed = v;
  if (!parse_trigger(j["trigger"], g.trigger, ctx, err)) {
    return false;
  }
  const auto &schema = grim_gene_schema(g.type);
  const json &pj = j["params"];
  if (!pj.is_object()) {
    return fail(err, ctx + ": params must be an object");
  }
  if (pj.size() != schema.size()) {
    return fail(err, ctx + ": " + name + " takes exactly " + std::to_string(schema.size()) +
                         " params");
  }
  for (size_t i = 0; i < schema.size(); ++i) {
    const auto it = pj.find(schema[i].name);
    if (it == pj.end()) {
      return fail(err, ctx + ": missing param \"" + schema[i].name + "\"");
    }
    if (!it->is_number_integer()) {
      return fail(err, ctx + ": param \"" + schema[i].name + "\" must be an integer");
    }
    const s64 val = it->get<s64>();
    if (val < schema[i].lo || val > schema[i].hi) {
      return fail(err, ctx + ": param \"" + schema[i].name + "\" out of range [" +
                           std::to_string(schema[i].lo) + ", " +
                           std::to_string(schema[i].hi) + "]");
    }
    g.params[i] = static_cast<s32>(val);
  }
  return true;
}
} // namespace

bool grim_genome_parse(const std::string &text, GrimGenome &out, std::string &err) {
  const json doc = json::parse(text, nullptr, false);
  if (doc.is_discarded()) {
    return fail(err, "not valid JSON");
  }
  if (!doc.is_object()) {
    return fail(err, "genome: expected an object");
  }
  u64 version = 0;
  if (!get_uint(doc, "version", 1, 2, version, "genome", err)) {
    return false;
  }
  if (version == 1 ? !check_keys(doc, {"version", "genes"}, "genome", err)
                   : !check_keys(doc, {"version", "bios_hash", "genes"}, "genome", err)) {
    return false;
  }
  if (!doc["genes"].is_array()) {
    return fail(err, "genome: genes must be an array");
  }
  GrimGenome g;
  g.version = static_cast<u32>(version);
  if (version == 2) {
    const json &bh = doc["bios_hash"];
    char *end = nullptr;
    const std::string text = bh.is_string() ? bh.get<std::string>() : "";
    g.bios_hash = std::strtoull(text.c_str(), &end, 16);
    if (text.size() < 3 || text.compare(0, 2, "0x") != 0 || end == nullptr || *end != 0) {
      return fail(err, "genome: bios_hash must be a hex string like \"0x1234abcd...\"");
    }
  }
  for (size_t i = 0; i < doc["genes"].size(); ++i) {
    GrimGene gene;
    if (!parse_gene(doc["genes"][i], i, gene, err)) {
      return false;
    }
    if (grim_gene_is_rom(gene.type) && version != 2) {
      return fail(err, "gene " + std::to_string(i) + ": " + grim_gene_type_name(gene.type) +
                           " genes need genome version 2");
    }
    g.genes.push_back(gene);
  }
  out = std::move(g);
  return true;
}

bool grim_genome_load(const std::string &path, GrimGenome &out, std::string &err) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    return fail(err, "cannot open " + path);
  }
  const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  return grim_genome_parse(text, out, err);
}

std::string grim_genome_serialize(const GrimGenome &g) {
  const bool v2 = g.version >= 2 || grim_genome_has_rom(g);
  std::string s = "{\"version\":" + std::string(v2 ? "2" : "1");
  if (v2) {
    char hb[40];
    std::snprintf(hb, sizeof(hb), ",\"bios_hash\":\"0x%016llX\"",
                  static_cast<unsigned long long>(g.bios_hash));
    s += hb;
  }
  s += ",\"genes\":[";
  for (size_t i = 0; i < g.genes.size(); ++i) {
    const GrimGene &gene = g.genes[i];
    const GrimTrigger &t = gene.trigger;
    s += i == 0 ? "\n" : ",\n";
    if (grim_gene_is_rom(gene.type)) {
      s += "{\"type\":\"" + std::string(grim_gene_type_name(gene.type)) + "\",\"seed\":" +
           std::to_string(gene.seed) + ",\"params\":{";
      const auto &rs = grim_gene_schema(gene.type);
      for (size_t k = 0; k < rs.size(); ++k) {
        s += (k ? ",\"" : "\"") + std::string(rs[k].name) + "\":" + std::to_string(gene.params[k]);
      }
      s += "}";
      if (gene.type == GrimGeneType::SpuSample && gene.sample_sizing.kind != GrimSampleSizeKind::Blocks) {
        const char *key = gene.sample_sizing.kind == GrimSampleSizeKind::Milliseconds
                            ? "milliseconds" : "fraction_permille";
        s += ",\"sizing\":{\"" + std::string(key) + "\":" +
             std::to_string(gene.sample_sizing.value) + "}";
      }
      s += ",\"patches\":[";
      for (size_t k = 0; k < gene.patches.size(); ++k) {
        const GrimRomPatch &p = gene.patches[k];
        s += (k ? ",[" : "[") + std::to_string(p.offset) + "," + std::to_string(p.original) +
             "," + std::to_string(p.mutated) + "," + (p.delay_slot ? "1" : "0") + "]";
      }
      s += "]}";
      continue;
    }
    s += "{\"type\":\"" + std::string(grim_gene_type_name(gene.type)) +
         "\",\"target\":" + std::to_string(gene.target) +
         ",\"seed\":" + std::to_string(gene.seed) + ",\"trigger\":{\"kind\":\"" +
         kTriggerNames[static_cast<size_t>(t.kind)] + "\"";
    if (t.kind == GrimTriggerKind::Window || t.kind == GrimTriggerKind::Rot) {
      s += ",\"start_frame\":" + std::to_string(t.start_frame) +
           ",\"end_frame\":" + std::to_string(t.end_frame);
    } else if (t.kind == GrimTriggerKind::Intermittent) {
      s += ",\"start_frame\":" + std::to_string(t.start_frame) +
           ",\"end_frame\":" + std::to_string(t.end_frame) +
           ",\"period\":" + std::to_string(t.period) +
           ",\"duty\":" + std::to_string(t.duty) +
           ",\"probability\":" + std::to_string(t.probability);
    }
    s += "},\"params\":{";
    const auto &schema = grim_gene_schema(gene.type);
    for (size_t k = 0; k < schema.size(); ++k) {
      s += (k ? ",\"" : "\"") + std::string(schema[k].name) +
           "\":" + std::to_string(gene.params[k]);
    }
    s += "}}";
  }
  s += g.genes.empty() ? "]}\n" : "\n]}\n";
  return s;
}

u64 grim_genome_hash(const GrimGenome &g) { return fnv1a(grim_genome_serialize(g)); }

// ---- random genome -----------------------------------------------------------------------

namespace {
bool categorical(const char *name) {
  static const char *const kNames[] = {"mode", "which", "keys", "semi", "raw", "perm",
                                       "piggyback", "wrap", "random", "eon", "spucnt",
                                       "scale"};
  for (const char *n : kNames) {
    if (std::string(n) == name) {
      return true;
    }
  }
  return false;
}
} // namespace

GrimGenome grim_random_genome(u64 seed, const GrimRandomParams &rp) {
  GrimRng rng{seed};
  GrimGenome genome;
  struct Weighted {
    GrimGeneType type;
    u32 weight;
  };
  std::vector<Weighted> pool;
  if (rp.spu) {
    pool.insert(pool.end(), {{GrimGeneType::SpuPitch, 3},
                             {GrimGeneType::SpuAdsr, 2},
                             {GrimGeneType::SpuVolume, 2},
                             {GrimGeneType::SpuAddress, 2},
                             {GrimGeneType::SpuKeyOn, 2},
                             {GrimGeneType::SpuNoise, 1},
                             {GrimGeneType::SpuPmon, 1},
                             {GrimGeneType::SpuReverb, 2}});
  }
  if (rp.gpu) {
    pool.insert(pool.end(), {{GrimGeneType::GpuVertex, 3},
                             {GrimGeneType::GpuColor, 3},
                             {GrimGeneType::GpuFlags, 2},
                             {GrimGeneType::GpuTexParam, 2},
                             {GrimGeneType::GpuState, 1},
                             {GrimGeneType::GpuFill, 1}});
  }
  if (pool.empty()) {
    if (rp.rom != nullptr && rp.rom_genes_max > 0) {
      grim_add_random_rom_genes(genome, seed, rp); // ROM-only genome
    }
    if (rp.sample != nullptr && rp.sample_genes_max > 0) {
      grim_add_random_sample_genes(genome, seed, rp);
    }
    if (rp.hw_genes_max > 0) {
      grim_add_random_hw_genes(genome, seed, rp);
    }
    return genome;
  }
  u32 total = 0;
  for (const Weighted &w : pool) {
    total += w.weight;
  }
  const u32 count = rng.range(std::max(1u, rp.min_genes), std::max(rp.min_genes, rp.max_genes));
  const u32 horizon = std::max(120u, rp.horizon_frames);
  for (u32 n = 0; n < count; ++n) {
    u32 pick_w = rng.range(0, total - 1);
    GrimGeneType type = pool.back().type;
    for (const Weighted &w : pool) {
      if (pick_w < w.weight) {
        type = w.type;
        break;
      }
      pick_w -= w.weight;
    }
    GrimGene g = grim_default_gene(type);
    g.seed = rng.next();
    // Target: usually everything, otherwise a random non-empty subset.
    if (rng.range(0, 99) >= 55) {
      const u32 full = grim_gene_default_target(type);
      u32 t = static_cast<u32>(rng.next()) & full;
      g.target = t != 0u ? t : (full & (0u - full));
      if (type == GrimGeneType::GpuFill) {
        g.target = 1u;
      }
    }
    // Parameters: categorical ones uniform, magnitudes pulled toward the
    // default by a squared random factor (survivable settings dominate).
    const auto &schema = grim_gene_schema(type);
    for (size_t i = 0; i < schema.size(); ++i) {
      const GrimParamSpec &spec = schema[i];
      if (categorical(spec.name)) {
        g.params[i] = rng.srange(spec.lo, spec.hi);
      } else {
        const u32 u = rng.unit_q10();
        u32 bias = (u * u) >> 10;
        bias += ((1024u - bias) * rp.risk_q10) >> 10;
        g.params[i] = lerp_q10(spec.def, rng.srange(spec.lo, spec.hi), bias);
      }
    }
    // Trigger shape.
    GrimTrigger &t = g.trigger;
    const u32 shape = rng.range(0, 99);
    if (shape < 15) {
      t.kind = GrimTriggerKind::Always;
    } else if (shape < 35) {
      t.kind = GrimTriggerKind::Window;
      t.start_frame = rng.range(0, horizon * 3u / 4u);
      t.end_frame = t.start_frame + rng.range(60, std::max(120u, horizon / 2u));
    } else if (shape < 70) {
      t.kind = GrimTriggerKind::Rot;
      t.start_frame = rng.range(0, horizon / 2u);
      t.end_frame = t.start_frame + rng.range(120, horizon);
    } else {
      t.kind = GrimTriggerKind::Intermittent;
      if (rng.range(0, 1) == 0) {
        t.period = rng.range(20, 300);
        t.duty = rng.range(1, std::max(1u, t.period * 3u / 4u));
      } else {
        t.probability = rng.range(50, 600);
      }
    }
    if (rp.rot_only && (t.kind != GrimTriggerKind::Rot || t.start_frame < 120u)) {
      t = GrimTrigger{};
      t.kind = GrimTriggerKind::Rot;
      t.start_frame = 120u + static_cast<u32>(g.seed % 300u); // healthy for 2-7 s
      t.end_frame = t.start_frame + std::max(300u, horizon);
    }
    genome.genes.push_back(g);
  }
  if (rp.rom != nullptr && rp.rom_genes_max > 0) {
    grim_add_random_rom_genes(genome, seed, rp);
  }
  if (rp.sample != nullptr && rp.sample_genes_max > 0) {
    grim_add_random_sample_genes(genome, seed, rp);
  }
  if (rp.hw_genes_max > 0) {
    grim_add_random_hw_genes(genome, seed, rp);
  }
  return genome;
}

// ---- runtime -----------------------------------------------------------------------------

GrimGenomeRuntime::GrimGenomeRuntime(GrimGenome genome) : genome_(std::move(genome)) {
  hash_ = grim_genome_hash(genome_);
  for (size_t i = 0; i < genome_.genes.size(); ++i) {
    if (grim_gene_is_rom(genome_.genes[i].type)) {
      continue; // applied to the ROM image, not to runtime traffic
    }
    if (grim_gene_is_hardware(genome_.genes[i].type)) {
      hw_genes_.push_back(i); // applied to memory at frame/scanline boundaries
      continue;
    }
    (is_spu_type(genome_.genes[i].type) ? spu_genes_ : gpu_genes_).push_back(i);
  }
  reset();
}

bool GrimGenomeRuntime::apply_rom(Bios &bios, std::string &err) {
  const u64 have = bios.image_hash();
  if (have != genome_.bios_hash) {
    char msg[160];
    std::snprintf(msg, sizeof(msg),
                  "ROM genes were made for BIOS 0x%016llX but the loaded BIOS is 0x%016llX",
                  static_cast<unsigned long long>(genome_.bios_hash),
                  static_cast<unsigned long long>(have));
    err = msg;
    return false;
  }
  for (size_t gi = 0; gi < genome_.genes.size(); ++gi) {
    const GrimGene &gene = genome_.genes[gi];
    if (!grim_gene_is_rom(gene.type)) {
      continue;
    }
    for (size_t k = 0; k < gene.patches.size(); ++k) {
      const GrimRomPatch &p = gene.patches[k];
      u32 word = 0;
      if (!bios.original_word(p.offset, word) || word != p.original) {
        char msg[200];
        std::snprintf(msg, sizeof(msg),
                      "gene %zu patch %zu at ROM 0x%05X: expected original word 0x%08X, the "
                      "BIOS has 0x%08X (wrong or modified BIOS)",
                      gi, k, p.offset, p.original, word);
        err = msg;
        return false;
      }
    }
  }
  for (size_t gi = 0; gi < genome_.genes.size(); ++gi) {
    if (!grim_gene_is_rom(genome_.genes[gi].type)) {
      continue;
    }
    for (const GrimRomPatch &p : genome_.genes[gi].patches) {
      bios.patch32(p.offset, p.mutated);
    }
  }
  return true;
}

void GrimGenomeRuntime::reset() {
  frame_ = 0;
  rng_.assign(genome_.genes.size(), GrimRng{});
  for (size_t i = 0; i < genome_.genes.size(); ++i) {
    rng_[i].state = genome_.genes[i].seed;
  }
  hits_.assign(genome_.genes.size(), 0);
  for (size_t i = 0; i < genome_.genes.size(); ++i) {
    hits_[i] = genome_.genes[i].patches.size(); // ROM genes: words patched at reset
  }
  delayed_.clear();
  shadow_.fill(0);
  build_hw_cells();
}

void GrimGenomeRuntime::queue_delayed(u64 due, u32 offset, u16 value) {
  constexpr size_t kMaxDelayed = 512;
  if (delayed_.size() >= kMaxDelayed) {
    return;
  }
  const auto pos = std::upper_bound(
      delayed_.begin(), delayed_.end(), due,
      [](u64 d, const Delayed &e) { return d < e.due; });
  delayed_.insert(pos, Delayed{due, offset, value});
}

size_t GrimGenomeRuntime::filter_spu_write(u32 offset, u16 value, u64 now_cycle,
                                           GrimSpuWrite *out) {
  WriteList cur;
  cur.push(offset, value);
  for (const size_t gi : spu_genes_) {
    WriteList next;
    for (size_t i = 0; i < cur.n; ++i) {
      apply_spu_gene(gi, cur.w[i], next, now_cycle);
    }
    cur = next;
  }
  for (size_t i = 0; i < cur.n; ++i) {
    out[i] = cur.w[i];
    if (cur.w[i].offset < shadow_.size() * 2u) {
      shadow_[cur.w[i].offset >> 1] = cur.w[i].value;
    }
  }
  return cur.n;
}

void GrimGenomeRuntime::apply_spu_gene(size_t gi, const GrimSpuWrite &in, WriteList &out,
                                       u64 now) {
  const GrimGene &g = genome_.genes[gi];
  const auto &p = g.params;
  const u32 off = in.offset;
  const u16 val = in.value;
  const bool voice_reg = off < 0x180u;
  const u32 voice = off >> 4;
  const u32 reg = off & 0xFu;
  const bool voice_in_target = voice_reg && voice < 24u && ((g.target >> voice) & 1u) != 0u;
  const bool is_kon = off == 0x188u || off == 0x18Au;
  const bool is_koff = off == 0x18Cu || off == 0x18Eu;
  const bool high_half = (off & 2u) != 0u; // for the paired 0x18x/0x19x registers

  // Decides whether this gene handles the write at all (no RNG use if not).
  bool handled = false;
  switch (g.type) {
  case GrimGeneType::SpuPitch:
    handled = voice_in_target && reg == 4u;
    break;
  case GrimGeneType::SpuAdsr:
    handled = voice_in_target && (reg == 8u || reg == 0xAu);
    break;
  case GrimGeneType::SpuVolume:
    handled = (voice_in_target && reg <= 2u && (reg & 1u) == 0u) ||
              (((g.target >> 24) & 1u) != 0u && (off == 0x180u || off == 0x182u));
    break;
  case GrimGeneType::SpuAddress:
    handled = voice_in_target && ((reg == 6u && p[0] != 1) || (reg == 0xEu && p[0] != 0));
    break;
  case GrimGeneType::SpuKeyOn:
    handled = ((is_kon && p[1] != 1) || (is_koff && p[1] != 0)) &&
              (val & half_mask(g.target, high_half)) != 0u;
    break;
  case GrimGeneType::SpuNoise:
    handled = off == 0x194u || off == 0x196u ||
              (p[0] != 0 && is_kon && (val & half_mask(g.target, high_half)) != 0u);
    break;
  case GrimGeneType::SpuPmon:
    handled = off == 0x190u || off == 0x192u ||
              (p[0] != 0 && is_kon && (val & half_mask(g.target, high_half)) != 0u);
    break;
  case GrimGeneType::SpuReverb:
    handled = (off >= 0x1C0u && off <= 0x1FEu && (off & 1u) == 0u && p[0] != 0) ||
              (off == 0x1A2u && p[1] != 0) || (p[2] != 0 && (off == 0x198u || off == 0x19Au)) ||
              (p[2] != 0 && is_kon && (val & half_mask(g.target, high_half)) != 0u) ||
              (off == 0x1AAu && p[3] != 0);
    break;
  default:
    break;
  }
  if (!handled) {
    out.push(off, val);
    return;
  }
  const u64 r = rng_[gi].next();
  const u32 m = grim_trigger_magnitude(g.trigger, frame_, r);
  if (m == 0u) {
    out.push(off, val);
    return;
  }

  u32 new_off = off;
  u32 v = val;
  const size_t n_before = out.n;
  const size_t queued_before = delayed_.size();
  switch (g.type) {
  case GrimGeneType::SpuPitch: {
    s32 np = static_cast<s32>(val);
    switch (p[0]) {
    case 0:
      np = static_cast<s32>(static_cast<s64>(val) * lerp_q10(256, p[1], m) / 256);
      break;
    case 1:
      np = static_cast<s32>(val) + scale_q10(p[2], m);
      break;
    case 2:
      np = lerp_q10(static_cast<s32>(val),
                    static_cast<s32>(quantize_pitch(val, static_cast<u32>(p[3]))), m);
      break;
    default: {
      const s64 wob = static_cast<s64>(val) * triangle_q10(frame_, static_cast<u32>(p[5])) *
                      scale_q10(p[4], m) / (1024 * 1024);
      np = static_cast<s32>(static_cast<s64>(val) + wob);
      break;
    }
    }
    v = static_cast<u32>(clamp_s32(np, 0, 0x3FFF));
    break;
  }
  case GrimGeneType::SpuAdsr: {
    const u32 mode = static_cast<u32>(p[0]);
    if (reg == 8u) { // low: attack 8-15, decay 4-7, sustain level 0-3
      if (mode == 0u) {
        v = (v & ~0xFu) | static_cast<u32>(lerp_q10(static_cast<s32>(v & 0xFu), 0xF, m));
      } else if (mode == 2u) {
        v ^= scale_mask(static_cast<u32>(r & 0xFF00u), m, r);
      } else if (mode == 3u) {
        v ^= scale_mask(static_cast<u32>(r & 0x00F0u), m, r);
      } else if (mode == 4u) {
        v ^= scale_mask(static_cast<u32>(r & 0x000Fu), m, r);
      }
    } else { // high: release shift 0-4, release mode 5, sustain 6-15
      if (mode == 0u) {
        const s32 shift = lerp_q10(static_cast<s32>((v >> 8) & 0x1Fu), 0x1F, m);
        v = (v & ~0xDF00u) | (static_cast<u32>(shift) << 8);
        if (pick(r, 1, m)) {
          v = (v | 0x4000u) & ~0x8000u; // decreasing, linear: slowest possible fade
        }
      } else if (mode == 1u) {
        const s32 shift = lerp_q10(static_cast<s32>(v & 0x1Fu), 0, m);
        v = (v & ~0x1Fu) | static_cast<u32>(shift);
        if (pick(r, 2, m)) {
          v &= ~0x20u; // linear release
        }
      } else if (mode == 5u) {
        v ^= scale_mask(static_cast<u32>((r >> 16) & 0x3Fu), m, r);
      }
    }
    break;
  }
  case GrimGeneType::SpuVolume: {
    if (p[0] == 0) { // swap L/R by redirecting the write
      if (pick(r, 3, m)) {
        new_off = off ^ 2u;
      }
      break;
    }
    if ((val & 0x8000u) != 0u) {
      break; // sweep-mode volume: leave alone
    }
    const s32 s = static_cast<s32>((val & 0x7FFFu) ^ 0x4000u) - 0x4000; // 15-bit signed
    s32 ns = s;
    if (p[0] == 1) {
      ns = lerp_q10(s, s == -0x4000 ? 0x3FFF : -s, m);
    } else if (p[0] == 2) {
      const s32 mag = s < 0 ? -s : s;
      if (mag > p[1]) {
        const s32 nm = lerp_q10(mag, p[1], m);
        ns = s < 0 ? -nm : nm;
      }
    } else {
      const s32 tri = triangle_q10(frame_, static_cast<u32>(p[3]));
      const s32 f = 1024 - scale_q10(p[2], m) * (tri + 1024) / 2048;
      ns = scale_q10(s, static_cast<u32>(f));
    }
    v = static_cast<u32>(ns) & 0x7FFFu;
    break;
  }
  case GrimGeneType::SpuAddress: {
    const s32 blocks = p[1];
    const s32 delta =
        p[2] != 0 ? static_cast<s32>(r % (static_cast<u64>(blocks < 0 ? -blocks : blocks) * 2u + 1u)) -
                        (blocks < 0 ? -blocks : blocks)
                  : blocks;
    v = static_cast<u32>(static_cast<s32>(val) + scale_q10(delta, m)) & 0xFFFFu;
    break;
  }
  case GrimGeneType::SpuKeyOn: {
    const u32 tmask = half_mask(g.target, high_half);
    const u32 chance = static_cast<u32>(scale_q10(p[2], m));
    u32 sel = 0;
    for (u32 b = 0; b < 16u; ++b) {
      if ((((val & tmask) >> b) & 1u) != 0u && (grim_mix64(r + b) % 1000u) < chance) {
        sel |= 1u << b;
      }
    }
    if (sel == 0u) {
      break;
    }
    const u64 due = now + static_cast<u64>(p[3]) * 768u;
    if (p[0] == 0) {
      v = val & ~sel;
    } else if (p[0] == 1) {
      queue_delayed(due, off, static_cast<u16>(sel));
    } else {
      v = val & ~sel;
      queue_delayed(due, off, static_cast<u16>(sel));
    }
    break;
  }
  case GrimGeneType::SpuNoise:
  case GrimGeneType::SpuPmon: {
    const bool noise = g.type == GrimGeneType::SpuNoise;
    const u32 base_reg = noise ? 0x194u : 0x190u;
    // The register half being written (direct write) or the half the key-on
    // belongs to (piggyback).
    const bool half_high = (off & 2u) != 0u;
    u32 tmask = half_mask(g.target, half_high);
    if (!noise && !half_high) {
      tmask &= ~1u; // voice 0 has no neighbour to modulate it
    }
    u32 sel = 0;
    const u32 limit = is_kon ? (val & tmask) : tmask;
    for (u32 b = 0; b < 16u; ++b) {
      if (((limit >> b) & 1u) != 0u && (grim_mix64(r + b) & 1023u) < m) {
        sel |= 1u << b;
      }
    }
    if (is_kon) {
      if (sel != 0u) {
        const u32 reg_off = base_reg + (half_high ? 2u : 0u);
        out.push(reg_off, static_cast<u16>(shadow_[reg_off >> 1] | sel));
      }
    } else {
      v = val | sel;
    }
    break;
  }
  case GrimGeneType::SpuReverb: {
    if (off >= 0x1C0u && off <= 0x1FEu) {
      if ((r % 1000u) < static_cast<u32>(scale_q10(p[0], m))) {
        v ^= scale_mask(static_cast<u32>((r >> 20) & 0x0FFFu), m, r);
      }
    } else if (off == 0x1A2u) {
      v = static_cast<u32>(static_cast<s32>(val) + scale_q10(p[1], m)) & 0xFFFFu;
    } else if (off == 0x198u || off == 0x19Au) {
      u32 sel = 0;
      const u32 tmask = half_mask(g.target, high_half);
      for (u32 b = 0; b < 16u; ++b) {
        if (((tmask >> b) & 1u) != 0u && (grim_mix64(r + b) & 1023u) < m) {
          sel |= 1u << b;
        }
      }
      v = val | sel;
    } else if (is_kon) {
      u32 sel = 0;
      const u32 tmask = val & half_mask(g.target, high_half);
      for (u32 b = 0; b < 16u; ++b) {
        if (((tmask >> b) & 1u) != 0u && (grim_mix64(r + b) & 1023u) < m) {
          sel |= 1u << b;
        }
      }
      if (sel != 0u) {
        const u32 reg_off = high_half ? 0x19Au : 0x198u;
        out.push(reg_off, static_cast<u16>(shadow_[reg_off >> 1] | sel));
      }
    } else if (off == 0x1AAu && pick(r, 4, m)) {
      v = val | 0x0080u; // SPUCNT reverb master enable
    }
    break;
  }
  default:
    break;
  }
  if (new_off != off || static_cast<u16>(v) != val || out.n != n_before || delayed_.size() != queued_before) {
    ++hits_[gi]; // changed the write, added a write, or queued a delayed one
  }
  out.push(new_off, static_cast<u16>(v));
}

// ---- GPU runtime ---------------------------------------------------------------------------

void GrimGenomeRuntime::filter_gp0(u32 *words, size_t count) {
  if (count == 0) {
    return;
  }
  const u32 original0 = words[0];
  for (const size_t gi : gpu_genes_) {
    apply_gpu_gene(gi, words, count);
  }
  // Belt and braces: whatever the genes did, the bits that decide how many
  // words follow (26-31) are unchanged; bits 24/25 only move on drawing
  // commands (semi-transparency), and bit 24 (raw texture) only on textured
  // ones. State commands keep their whole opcode byte.
  const u32 op = original0 >> 24;
  u32 keep = 0xFC000000u;
  if (op < 0x20u || op > 0x7Fu) {
    keep = 0xFF000000u;
  } else {
    const bool textured =
        ((op >= 0x20u && op <= 0x3Fu) || (op >= 0x60u && op <= 0x7Fu)) && (op & 0x04u) != 0u;
    if (!textured) {
      keep |= 0x01000000u;
    }
  }
  words[0] = (words[0] & ~keep) | (original0 & keep);
}

u32 GrimGenomeRuntime::filter_gp0_polyline_word(u32 word, bool gouraud, bool color_word) {
  (void)gouraud;
  if (gpu_genes_.empty() || is_polyline_terminator(word)) {
    return word;
  }
  for (const size_t gi : gpu_genes_) {
    apply_gpu_word_gene(gi, word, color_word);
  }
  return word;
}

void GrimGenomeRuntime::apply_gpu_word_gene(size_t gi, u32 &word, bool color_word) {
  const GrimGene &g = genome_.genes[gi];
  if ((g.target & kClassLine) == 0u) {
    return;
  }
  const bool vertex_gene = g.type == GrimGeneType::GpuVertex && g.params[0] != 4;
  const bool color_gene = g.type == GrimGeneType::GpuColor && g.params[0] != 2;
  if (!((vertex_gene && !color_word) || (color_gene && color_word))) {
    return;
  }
  const u64 r = rng_[gi].next();
  const u32 m = grim_trigger_magnitude(g.trigger, frame_, r);
  if (m == 0u) {
    return;
  }
  const u32 nw = vertex_gene ? xform_vertex(g, word, r, 0, m, frame_) : xform_color(g, word, r, m);
  if (nw != word && !is_polyline_terminator(nw)) { // never fake a terminator
    ++hits_[gi];
    word = nw;
  }
}

void GrimGenomeRuntime::apply_gpu_gene(size_t gi, u32 *w, size_t count) {
  const GrimGene &g = genome_.genes[gi];
  const auto &p = g.params;
  const u32 op = w[0] >> 24;
  Gp0Layout L;
  bool handled = false;
  switch (g.type) {
  case GrimGeneType::GpuVertex:
  case GrimGeneType::GpuColor:
  case GrimGeneType::GpuFlags:
  case GrimGeneType::GpuTexParam:
    handled = gp0_layout(w, count, L) && (g.target & L.klass) != 0u;
    if (handled && g.type == GrimGeneType::GpuTexParam) {
      handled = L.textured;
    }
    break;
  case GrimGeneType::GpuState:
    handled = op >= 0xE1u && op <= 0xE6u && ((g.target >> (op - 0xE1u)) & 1u) != 0u && count >= 1u;
    break;
  case GrimGeneType::GpuFill:
    handled = op == 0x02u && count >= 3u;
    break;
  default:
    break;
  }
  if (!handled) {
    return;
  }
  const u64 r = rng_[gi].next();
  const u32 m = grim_trigger_magnitude(g.trigger, frame_, r);
  if (m == 0u) {
    return;
  }
  bool changed = false;
  const auto set = [&](size_t i, u32 nv) {
    if (w[i] != nv) {
      w[i] = nv;
      changed = true;
    }
  };

  switch (g.type) {
  case GrimGeneType::GpuVertex: {
    if (p[0] == 4) { // swap two vertices' coordinates
      if (L.nv >= 2u && (r % 1000u) < static_cast<u32>(scale_q10(p[6], m))) {
        const size_t a = static_cast<size_t>((r >> 10) % L.nv);
        const size_t b = (a + 1u + static_cast<size_t>((r >> 20) % (L.nv - 1u))) % L.nv;
        const u32 wa = w[L.vw[a]];
        const u32 wb = w[L.vw[b]];
        set(L.vw[a], (wa & ~0x07FF07FFu) | (wb & 0x07FF07FFu));
        set(L.vw[b], (wb & ~0x07FF07FFu) | (wa & 0x07FF07FFu));
      }
      break;
    }
    for (size_t i = 0; i < L.nv; ++i) {
      set(L.vw[i], xform_vertex(g, w[L.vw[i]], r, static_cast<u32>(i), m, frame_));
    }
    break;
  }
  case GrimGeneType::GpuColor: {
    if (p[0] == 2) { // gradient shuffle: rotate colors among the vertices
      if (L.nc >= 2u) {
        u32 old[4];
        for (size_t i = 0; i < L.nc; ++i) {
          old[i] = w[L.cw[i]] & 0xFFFFFFu;
        }
        const size_t dir = (r & 1u) != 0u ? L.nc - 1u : 1u;
        for (size_t i = 0; i < L.nc; ++i) {
          const u32 src = old[(i + dir) % L.nc];
          u32 blended = 0;
          for (int k = 0; k < 3; ++k) {
            blended |= static_cast<u32>(lerp_q10(static_cast<s32>((old[i] >> (8 * k)) & 0xFFu),
                                                 static_cast<s32>((src >> (8 * k)) & 0xFFu), m))
                       << (8 * k);
          }
          set(L.cw[i], (w[L.cw[i]] & 0xFF000000u) | blended);
        }
      }
      break;
    }
    for (size_t i = 0; i < L.nc; ++i) {
      set(L.cw[i], xform_color(g, w[L.cw[i]], r + i, m));
    }
    break;
  }
  case GrimGeneType::GpuFlags: {
    if ((r % 1000u) >= static_cast<u32>(scale_q10(p[2], m))) {
      break;
    }
    u32 nw = apply_bit_op(w[0], 0x02000000u, p[0]);
    if (L.textured) {
      nw = apply_bit_op(nw, 0x01000000u, p[1]);
    }
    set(0, nw);
    break;
  }
  case GrimGeneType::GpuTexParam: {
    if ((r % 1000u) >= static_cast<u32>(scale_q10(p[3], m))) {
      break;
    }
    if (p[0] != 1 && L.nuv >= 1u) { // CLUT: upper half of the first UV word
      set(L.uw[0], w[L.uw[0]] ^ (scale_mask(static_cast<u32>(p[1]), m, r) << 16));
    }
    if (p[0] != 0 && L.nuv >= 2u) { // texpage: upper half of the second UV word
      set(L.uw[1], w[L.uw[1]] ^ (scale_mask(static_cast<u32>(p[2]), m, r >> 7) << 16));
    }
    break;
  }
  case GrimGeneType::GpuState: {
    const u32 v = w[0];
    const u32 span = static_cast<u32>(p[0]) * 2u + 1u;
    const s32 jx = scale_q10(static_cast<s32>((r & 0xFFFFu) % span) - p[0], m);
    const s32 jy = scale_q10(static_cast<s32>(((r >> 16) & 0xFFFFu) % span) - p[0], m);
    const s32 drift = scale_q10(static_cast<s32>(static_cast<s64>(p[1]) * frame_ / 60), m);
    switch (op) {
    case 0xE1:
      set(0, v ^ scale_mask(static_cast<u32>(p[2]), m, r));
      break;
    case 0xE2:
      set(0, v ^ scale_mask(static_cast<u32>(p[3]), m, r));
      break;
    case 0xE3:
    case 0xE4: {
      const s32 x = clamp_s32(static_cast<s32>(v & 0x3FFu) + jx + drift, 0, 1023);
      const s32 y = clamp_s32(static_cast<s32>((v >> 10) & 0x1FFu) + jy + drift, 0, 511);
      set(0, (v & ~0x7FFFFu) | static_cast<u32>(x) | (static_cast<u32>(y) << 10));
      break;
    }
    case 0xE5: {
      const s32 x = sext11(v) + jx + drift;
      const s32 y = sext11(v >> 11) + jy + drift;
      set(0, (v & ~0x3FFFFFu) | (static_cast<u32>(x) & 0x7FFu) |
                 ((static_cast<u32>(y) & 0x7FFu) << 11));
      break;
    }
    default: // E6
      set(0, v ^ scale_mask(static_cast<u32>(p[4]), m, r));
      break;
    }
    break;
  }
  case GrimGeneType::GpuFill: {
    set(0, w[0] ^ scale_mask(static_cast<u32>(p[0]), m, r));
    if (p[1] > 0) {
      const u32 span = static_cast<u32>(p[1]) * 2u + 1u;
      const auto jit = [&](u32 salt) {
        return scale_q10(static_cast<s32>(grim_mix64(r + salt) % span) - p[1], m);
      };
      const s32 x = static_cast<s32>(w[1] & 0x3FFu) + jit(1);
      const s32 y = static_cast<s32>((w[1] >> 16) & 0x1FFu) + jit(2);
      const s32 fw = clamp_s32(static_cast<s32>(w[2] & 0x3FFu) + jit(3), 0, 1023);
      const s32 fh = clamp_s32(static_cast<s32>((w[2] >> 16) & 0x1FFu) + jit(4), 0, 511);
      set(1, (w[1] & ~0x01FF03FFu) | (static_cast<u32>(x) & 0x3FFu) |
                 ((static_cast<u32>(y) & 0x1FFu) << 16));
      set(2, (w[2] & ~0x01FF03FFu) | static_cast<u32>(fw) | (static_cast<u32>(fh) << 16));
    }
    break;
  }
  default:
    break;
  }
  if (changed) {
    ++hits_[gi];
  }
}

namespace {
std::unique_ptr<GrimGenomeRuntime> g_gui_genome;
}

bool grim_gui_genome_load(const std::string &path, std::string &err) {
  GrimGenome genome;
  if (!grim_genome_load(path, genome, err)) {
    return false;
  }
  g_gui_genome = std::make_unique<GrimGenomeRuntime>(std::move(genome));
  return true;
}

GrimGenomeRuntime *grim_gui_genome() { return g_gui_genome.get(); }
