#pragma once
#include "types.h"
#include <array>
#include <vector>

// ── PGXP precision tracking ────────────────────────────────────────
// The GTE rounds projected vertices to whole pixels and the GPU only ever
// sees those integers. With PGXP enabled the GTE also keeps each
// projection's sub-pixel position and depth beside its screen-XY FIFO. When
// the game stores a screen-XY register to RAM with SWC2 (how PsyQ writes
// vertices into GPU primitives), the precise vertex is remembered for that
// RAM word together with the exact value stored. Precision also follows the
// data through CPU registers, plain word loads/stores and the scratchpad
// (MFC2/MTC2, LW/SW, LWC2: "memory mode"). A vertex the GPU later receives by
// DMA is only treated as precise if its source word still holds that value,
// so a match always means "this very vertex" and never a different point
// that happens to round to the same pixel. Vertices the CPU computes with
// arithmetic (sprite offsets, particles) are not tracked.
//
// Purely cosmetic state: never saved, and a miss simply means the vertex is
// drawn the original way.

class Pgxp {
public:
  static constexpr u32 kNoSource = 0xFFFFFFFFu;

  struct PreciseVertex {
    float x = 0.0f;
    float y = 0.0f;
    float w = 1.0f; // view-space depth, > 0
  };

  void clear() {
    fifo_ = {};
    gpr_ = {};
    ram_.clear();
  }

  // RTPS/RTPT pushed SXY2. `valid` is false when the projection had no
  // trustworthy sub-pixel position (clamped, divide overflow...).
  void push_screen_xy(s16 sx, s16 sy, bool valid, const PreciseVertex &v) {
    fifo_[0] = fifo_[1];
    fifo_[1] = fifo_[2];
    fifo_[2] = Slot{valid, pack(sx, sy), v};
  }

  // MTC2/LWC2 to SXYP pushes an integer-only vertex.
  void push_untracked_screen_xy() {
    fifo_[0] = fifo_[1];
    fifo_[1] = fifo_[2];
    fifo_[2] = Slot{};
  }

  // Precise position of screen-XY FIFO entry `index` (0..2 = SXY0..SXY2),
  // if it is tracked and still matches the register value `sx`/`sy`.
  bool screen_xy(int index, s16 sx, s16 sy, PreciseVertex &out) const {
    const Slot &slot = fifo_[static_cast<size_t>(index)];
    if (!slot.valid || slot.value != pack(sx, sy)) {
      return false;
    }
    out = slot.v;
    return true;
  }

  // ── Memory mode ──────────────────────────────────────────────────
  // Precise vertices also follow the data through CPU registers and plain
  // word loads/stores. Every shadow keeps the exact 32-bit value it belongs
  // to and is only used while the register/word still holds that value, so
  // unrelated writes never need hooks: they simply stop matching.

  // MFC2 rt <- GTE data `reg` (value already read from the GTE).
  void on_mfc2(u32 reg, u32 rt, u32 value) {
    if (rt == 0u || rt >= 32u) {
      return;
    }
    const Slot *slot = sxy_slot(reg);
    gpr_[rt] = (slot != nullptr && slot->valid && slot->value == value)
                   ? Word{true, value, slot->v}
                   : Word{};
  }

  // MTC2 GTE data `reg` <- rt; call after the GTE register was written.
  void on_mtc2(u32 reg, u32 rt, u32 value) {
    Slot *slot = sxy_slot(reg);
    if (slot == nullptr) {
      return;
    }
    const Word &src = gpr_[rt & 31u];
    *slot = (rt != 0u && src.valid && src.value == value)
                ? Slot{true, value, src.v}
                : Slot{false, value, {}};
  }

  // LWC2 GTE data `reg` <- [phys]; call after the GTE register was written.
  void on_lwc2(u32 reg, u32 phys, u32 value) {
    Slot *slot = sxy_slot(reg);
    if (slot == nullptr) {
      return;
    }
    PreciseVertex v;
    *slot = lookup(phys, value, v) ? Slot{true, value, v} : Slot{false, value, {}};
  }

  // LW rt <- [phys].
  void on_lw(u32 rt, u32 phys, u32 value) {
    if (rt == 0u || rt >= 32u) {
      return;
    }
    PreciseVertex v;
    gpr_[rt] = lookup(phys, value, v) ? Word{true, value, v} : Word{};
  }

  // SW [phys] <- rt.
  void on_sw(u32 rt, u32 phys, u32 value) {
    const Word &src = gpr_[rt & 31u];
    const bool tracked = rt != 0u && src.valid && src.value == value;
    Word *dst = shadow_word(phys, tracked);
    if (dst != nullptr) {
      *dst = tracked ? src : Word{};
    }
  }

  // SWC2 of GTE data register `reg` stored `value` at physical `phys`.
  void record_store(u32 phys, u32 reg, u32 value) {
    const Slot *slot = sxy_slot(reg);
    // The packed-value check rejects a shadow slot that drifted out of step
    // with the real register (e.g. PGXP enabled mid-frame).
    const bool tracked = slot != nullptr && slot->valid && slot->value == value;
    Word *dst = shadow_word(phys, tracked);
    if (dst != nullptr) {
      *dst = tracked ? Word{true, value, slot->v} : Word{};
    }
  }

  // Precise vertex for a word read from physical `phys` (a GPU vertex word
  // fetched by DMA, or an LW/LWC2 operand).
  bool lookup(u32 phys, u32 value, PreciseVertex &out) const {
    const Word *w = const_cast<Pgxp *>(this)->shadow_word(phys, false);
    if (w == nullptr || !w->valid || w->value != value) {
      return false;
    }
    out = w->v;
    return true;
  }

private:
  static constexpr u32 kRamMirrorEnd = 0x00800000u;
  static constexpr u32 kScratchpadBase = 0x1F800000u;
  static constexpr u32 kScratchpadSize = 0x400u;
  static constexpr size_t kRamWords = psx::RAM_SIZE / 4u;
  static constexpr size_t kShadowWords = kRamWords + kScratchpadSize / 4u;

  // Shadow entry for a physical address in main RAM (incl. mirrors) or the
  // scratchpad, or nullptr. The table is allocated on the first tracked
  // write (`allocate`); until then nothing can be tracked.
  struct Word;
  Word *shadow_word(u32 phys, bool allocate) {
    size_t index = 0;
    if (phys < kRamMirrorEnd) {
      index = (phys & (psx::RAM_SIZE - 1u)) >> 2;
    } else if (phys >= kScratchpadBase && phys < kScratchpadBase + kScratchpadSize) {
      index = kRamWords + ((phys - kScratchpadBase) >> 2);
    } else {
      return nullptr;
    }
    if (ram_.empty()) {
      if (!allocate) {
        return nullptr;
      }
      ram_.resize(kShadowWords);
    }
    return &ram_[index];
  }

  // SXY0..SXY2 (12..14); SXYP (15) aliases SXY2, the newest entry.
  struct Slot;
  Slot *sxy_slot(u32 reg) {
    return (reg >= 12u && reg <= 15u) ? &fifo_[(reg == 15u) ? 2u : (reg - 12u)]
                                      : nullptr;
  }
  const Slot *sxy_slot(u32 reg) const {
    return const_cast<Pgxp *>(this)->sxy_slot(reg);
  }

  static u32 pack(s16 sx, s16 sy) {
    return static_cast<u32>(static_cast<u16>(sx)) |
           (static_cast<u32>(static_cast<u16>(sy)) << 16);
  }

  struct Slot {
    bool valid = false;
    u32 value = 0;
    PreciseVertex v{};
  };
  struct Word {
    bool valid = false;
    u32 value = 0;
    PreciseVertex v{};
  };

  std::array<Slot, 3> fifo_{};
  std::array<Word, 32> gpr_{}; // per CPU register, memory mode
  std::vector<Word> ram_;      // one entry per RAM word, allocated on first use
};
