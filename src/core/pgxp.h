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
// RAM word together with the exact value stored. A vertex the GPU later
// receives by DMA is only treated as precise if its source word still holds
// that value, so a match always means "this very vertex" and never a
// different point that happens to round to the same pixel.
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

  // SWC2 of GTE data register `reg` stored `value` at physical `phys`.
  void record_store(u32 phys, u32 reg, u32 value) {
    if (phys >= kRamMirrorEnd) {
      return;
    }
    const size_t index = (phys & (psx::RAM_SIZE - 1u)) >> 2;
    const Slot *slot = nullptr;
    if (reg >= 12u && reg <= 15u) {
      slot = &fifo_[(reg == 15u) ? 2u : (reg - 12u)];
    }
    // The packed-value check rejects a shadow slot that drifted out of step
    // with the real register (e.g. PGXP enabled mid-frame).
    if (slot == nullptr || !slot->valid || slot->value != value) {
      if (!ram_.empty()) {
        ram_[index].valid = false;
      }
      return;
    }
    if (ram_.empty()) {
      ram_.resize(psx::RAM_SIZE / 4u);
    }
    ram_[index] = Word{true, value, slot->v};
  }

  // A GPU vertex word `value` arrived by DMA from physical `phys`.
  bool lookup(u32 phys, u32 value, PreciseVertex &out) const {
    if (phys == kNoSource || phys >= kRamMirrorEnd || ram_.empty()) {
      return false;
    }
    const Word &w = ram_[(phys & (psx::RAM_SIZE - 1u)) >> 2];
    if (!w.valid || w.value != value) {
      return false;
    }
    out = w.v;
    return true;
  }

private:
  static constexpr u32 kRamMirrorEnd = 0x00800000u;

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
  std::vector<Word> ram_; // one entry per RAM word, allocated on first use
};
