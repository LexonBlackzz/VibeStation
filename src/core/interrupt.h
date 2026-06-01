#pragma once
#include "types.h"

// ── Interrupt Controller ───────────────────────────────────────────
// I_STAT (0x1F801070): Interrupt status — set by hardware, ack by writing 0
// I_MASK (0x1F801074): Interrupt mask — enable/disable individual sources

class System; // Forward declaration

enum class Interrupt : u32 {
  VBlank = 0,    // Vertical blank
  GPU = 1,       // GPU ready
  CDROM = 2,     // CD-ROM
  DMA = 3,       // DMA transfer complete
  Timer0 = 4,    // Root counter 0
  Timer1 = 5,    // Root counter 1
  Timer2 = 6,    // Root counter 2
  SIO = 7,       // Controller/Memory card
  SPU = 9,       // SPU
  Lightpen = 10, // Lightpen (active low)
};

class InterruptController {
public:
  void init(System *sys) { sys_ = sys; }
  void reset();

  // Read registers
  u32 read(u32 offset) const;
  // Write registers
  void write(u32 offset, u32 value);

  // Fire an interrupt from a hardware source
  void request(Interrupt irq);

  // Check if any unmasked interrupt is pending
  bool pending() const { return (i_stat_ & i_mask_) != 0; }

  u32 stat() const { return i_stat_; }
  u32 mask() const { return i_mask_; }
  u64 request_count(Interrupt irq) const {
    return request_count_[static_cast<u32>(irq)];
  }

private:
  u32 read_reg_word(u32 base_offset) const;
  void write_reg_word(u32 base_offset, u32 value);

  System *sys_ = nullptr;
  u32 i_stat_ = 0;
  u32 i_mask_ = 0;
  u64 request_count_[11] = {};
};
