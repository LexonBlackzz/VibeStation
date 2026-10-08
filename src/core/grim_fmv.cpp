#include "grim_fmv.h"
#include <algorithm>
#include <array>

namespace {
constexpr u16 kEndOfBlock = 0xFE00u;
constexpr size_t kBlock = 64;

// The next per-macroblock random word (stateless mix, salted by the edit count).
u64 next_word(GrimFmvMacroblock &mb) { return grim_mix64(mb.r + ++mb.n); }

// True with probability chance/1024.
bool chance(u64 r, u32 chance) { return (r & 1023u) < chance; }

int sext10(u16 v) { return static_cast<int>((v & 0x3FFu) ^ 0x200u) - 0x200; }
} // namespace

void grim_fmv_begin(const GrimFmvKnobs &k, GrimRng &rng, GrimFmvMacroblock &mb) {
  const u64 r = rng.next();
  mb.r = grim_mix64(r);
  mb.n = 0;
  mb.hit = (k.targets & (kGrimFmvCoeffs | kGrimFmvBlocks | kGrimFmvPixels)) != 0u &&
           (r % 1000u) < k.rate;
}

u16 grim_fmv_coefficient(const GrimFmvKnobs &k, GrimFmvMacroblock &mb, u16 halfword) {
  if (!mb.hit || (k.targets & kGrimFmvCoeffs) == 0u || halfword == kEndOfBlock) {
    return halfword;
  }
  const u64 r = next_word(mb);
  if (!chance(r, k.strength / 2u)) {
    return halfword;
  }
  int level = sext10(halfword);
  const u64 r2 = r >> 10;
  switch (r2 % 4u) {
  case 0: { // flip low bits: more of them the stronger it is
    const u32 bits = 1u + k.strength * 9u / 1024u;
    level ^= static_cast<int>((r2 >> 2) & ((1u << bits) - 1u));
    break;
  }
  case 1: // amplify: ringing, oversharpened blocks
    level *= 2 + static_cast<int>((r2 >> 2) % (1u + k.strength / 128u));
    break;
  case 2: // invert
    level = -level;
    break;
  default: { // replace with noise
    const int span = 1 + static_cast<int>(k.strength / 2u);
    level = static_cast<int>((r2 >> 2) % static_cast<u64>(2 * span + 1)) - span;
    break;
  }
  }
  level = std::clamp(level, -512, 511);
  const u16 out = static_cast<u16>((halfword & 0xFC00u) | (static_cast<u32>(level) & 0x3FFu));
  return out == kEndOfBlock ? halfword : out; // never fake an end of block
}

size_t grim_fmv_quant(const GrimFmvKnobs &k, GrimRng &rng, u8 *table, size_t n) {
  if ((k.targets & kGrimFmvQuant) == 0u || k.strength == 0u) {
    return 0;
  }
  size_t changed = 0;
  for (size_t i = 0; i < n; ++i) {
    const u64 r = rng.next();
    if (!chance(r, k.strength / 2u)) {
      continue;
    }
    int q = table[i];
    const u64 r2 = r >> 10;
    if ((r2 & 1u) != 0u) { // scale up: coarse, posterised detail
      q *= 2 + static_cast<int>((r2 >> 1) % (1u + k.strength / 64u));
    } else { // scramble
      q ^= static_cast<int>((r2 >> 1) & 0xFFu);
    }
    const u8 nq = static_cast<u8>(std::clamp(q, 1, 255));
    if (nq != table[i]) {
      table[i] = nq;
      ++changed;
    }
  }
  return changed;
}

bool grim_fmv_blocks(const GrimFmvKnobs &k, GrimFmvMacroblock &mb, int *blocks, size_t count) {
  if (!mb.hit || (k.targets & kGrimFmvBlocks) == 0u || count == 0u) {
    return false;
  }
  const auto block = [&](size_t i) { return blocks + i * kBlock; };
  const u32 ops = 1u + k.strength * 3u / 1024u;
  bool changed = false;
  for (u32 op = 0; op < ops; ++op) {
    const u64 r = next_word(mb);
    const size_t a = static_cast<size_t>((r >> 8) % count);
    const size_t b = count > 1u ? (a + 1u + static_cast<size_t>((r >> 16) % (count - 1u))) % count : a;
    // Two-block edits need two blocks; a monochrome macroblock has one.
    const u32 kind = count > 1u ? static_cast<u32>(r % 5u) : 2u + static_cast<u32>(r % 3u);
    int *pa = block(a);
    switch (kind) {
    case 0: // swap (Cr with Cb turns the colours)
      std::swap_ranges(pa, pa + kBlock, block(b));
      break;
    case 1: // copy: a repeated tile
      std::copy(pa, pa + kBlock, block(b));
      break;
    case 2: { // flatten to one value: a dead square
      int sum = 0;
      for (size_t i = 0; i < kBlock; ++i) sum += pa[i];
      const int bias = static_cast<int>((r >> 24) % 129u) - 64;
      std::fill(pa, pa + kBlock, std::clamp(sum / static_cast<int>(kBlock) + bias, -128, 127));
      break;
    }
    case 3: // invert
      for (size_t i = 0; i < kBlock; ++i) pa[i] = std::clamp(-pa[i], -128, 127);
      break;
    default: { // shift rows and columns round: a torn tile
      std::array<int, kBlock> tmp{};
      const size_t dx = static_cast<size_t>((r >> 24) % 8u);
      const size_t dy = static_cast<size_t>((r >> 28) % 8u);
      for (size_t y = 0; y < 8u; ++y) {
        for (size_t x = 0; x < 8u; ++x) {
          tmp[((y + dy) & 7u) * 8u + ((x + dx) & 7u)] = pa[y * 8u + x];
        }
      }
      std::copy(tmp.begin(), tmp.end(), pa);
      break;
    }
    }
    changed = true;
  }
  return changed;
}

u32 grim_fmv_output(const GrimFmvKnobs &k, GrimFmvMacroblock &mb, u32 word) {
  if (!mb.hit || (k.targets & kGrimFmvPixels) == 0u) {
    return word;
  }
  const u64 r = next_word(mb);
  if (!chance(r, k.strength / 4u)) {
    return word;
  }
  // About a quarter of the bits, more of the high (visible) ones at high strength.
  const u32 noise = static_cast<u32>(r >> 10) & static_cast<u32>(r >> 32);
  const u32 keep_high = k.strength >= 512u ? 0xFFFFFFFFu : 0x3DEF3DEFu; // drop RGB15 MSBs when mild
  return word ^ (noise & keep_high);
}

bool grim_fmv_audio(const GrimFmvKnobs &k, GrimRng &rng, s16 *lr, size_t frames,
                    GrimFmvAudioMemory &memory) {
  if ((k.targets & kGrimFmvAudio) == 0u || frames == 0u) {
    return false;
  }
  const size_t n = frames * 2u;
  const std::vector<s16> clean(lr, lr + n);
  const u64 r = rng.next();
  const bool hit = (r % 1000u) < k.rate;
  if (hit) {
    GrimFmvMacroblock salt;
    salt.r = grim_mix64(r);
    const auto clip = [](s32 v) { return static_cast<s16>(std::clamp(v, -32768, 32767)); };
    // The damaged window: a slice of the sector, longer the stronger it is.
    const size_t len = std::max<size_t>(
        32u, frames * (128u + k.strength * 7u / 8u) / 1024u * (1u + next_word(salt) % 4u) / 4u);
    const size_t a = len >= frames ? 0u : static_cast<size_t>(next_word(salt) % (frames - len + 1u));
    const size_t b = std::min(frames, a + len);
    const u64 w = next_word(salt);
    switch (w % 7u) {
    case 0: { // stutter: the window loops its own first few milliseconds
      const size_t loop = 48u + static_cast<size_t>((w >> 8) % (64u + k.strength / 2u));
      for (size_t f = a + loop; f < b; ++f) {
        lr[f * 2u] = lr[(a + (f - a) % loop) * 2u];
        lr[f * 2u + 1u] = lr[(a + (f - a) % loop) * 2u + 1u];
      }
      break;
    }
    case 1: // skip back: the window replays the previous sector, like a jumping disc
      if (memory.last.size() == n) {
        std::copy(memory.last.begin() + static_cast<std::ptrdiff_t>(a * 2u),
                  memory.last.begin() + static_cast<std::ptrdiff_t>(b * 2u), lr + a * 2u);
        break;
      }
      [[fallthrough]];
    case 2: // reverse
      for (size_t i = a, j = b - 1u; i < j; ++i, --j) {
        std::swap(lr[i * 2u], lr[j * 2u]);
        std::swap(lr[i * 2u + 1u], lr[j * 2u + 1u]);
      }
      break;
    case 3: { // crush: fewer bits and a held, lower sample rate
      const u32 bits = 2u + k.strength * 10u / 1024u;
      const s32 step = 1 << std::min<u32>(bits, 14u);
      const size_t hold = 2u + static_cast<size_t>((w >> 8) % (1u + k.strength / 128u));
      for (size_t f = a; f < b; ++f) {
        const size_t src = a + (f - a) / hold * hold;
        for (size_t c = 0; c < 2u; ++c) {
          lr[f * 2u + c] = clip(lr[src * 2u + c] / step * step);
        }
      }
      break;
    }
    case 4: // dropout
      std::fill(lr + a * 2u, lr + b * 2u, s16{0});
      break;
    case 5: { // blast: overdriven into clipping
      const s32 gain = 2 + static_cast<s32>((w >> 8) % (1u + k.strength / 128u));
      for (size_t i = a * 2u; i < b * 2u; ++i) lr[i] = clip(lr[i] * gain);
      break;
    }
    default: { // static over the sound
      const s32 amp = 256 + static_cast<s32>(k.strength) * 16;
      for (size_t i = a * 2u; i < b * 2u; ++i) {
        lr[i] = clip(lr[i] + static_cast<s32>(grim_mix64(w + i) % static_cast<u64>(2 * amp + 1)) - amp);
      }
      break;
    }
    }
  }
  memory.last = clean;
  return hit;
}
