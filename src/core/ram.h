#pragma once
#include "types.h"
#include <array>

// ── Main RAM (2MB) + Scratchpad (1KB) ──────────────────────────────

class Ram {
public:
  Ram() {
    data_.fill(0);
    scratchpad_.fill(0);
  }

  void reset() {
    data_.fill(0);
    scratchpad_.fill(0);
  }

  // Main RAM access
  u8 read8(u32 offset) const;
  u16 read16(u32 offset) const;
  u32 read32(u32 offset) const;
  void write8(u32 offset, u8 val);
  void write16(u32 offset, u16 val);
  void write32(u32 offset, u32 val);

  // Scratchpad access (1KB, located at 0x1F800000)
  u8 scratch_read8(u32 offset) const;
  u16 scratch_read16(u32 offset) const;
  u32 scratch_read32(u32 offset) const;
  void scratch_write8(u32 offset, u8 val);
  void scratch_write16(u32 offset, u16 val);
  void scratch_write32(u32 offset, u32 val);

  // Direct pointer access (for DMA)
  u8 *data() { return data_.data(); }
  const u8 *data() const { return data_.data(); }
  u8 *scratch_data() { return scratchpad_.data(); }
  const u8 *scratch_data() const { return scratchpad_.data(); }

private:
  std::array<u8, psx::RAM_SIZE> data_;
  std::array<u8, psx::SCRATCHPAD_SIZE> scratchpad_;
};
