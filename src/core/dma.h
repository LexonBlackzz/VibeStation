#pragma once
#include "types.h"

// ── DMA Controller ─────────────────────────────────────────────────
// 7-channel DMA controller for the PS1.
// Channels: 0=MDECin, 1=MDECout, 2=GPU, 3=CDROM, 4=SPU, 5=PIO, 6=OTC (GPU list
// clear)

class System;

struct DmaChannel {
  u32 base_addr = 0;    // MADR: Base address
  u32 block_ctrl = 0;   // BCR: Block control
  u32 channel_ctrl = 0; // CHCR: Channel control
  u32 block_words_remaining = 0; // Internal progress for sync-mode block DMA.

  // Decoded from CHCR
  bool from_ram() const {
    return (channel_ctrl >> 0) & 1;
  } // 0=to RAM, 1=from RAM
  bool enabled() const { return (channel_ctrl >> 24) & 1; }
  bool trigger() const { return (channel_ctrl >> 28) & 1; }

  enum class SyncMode {
    Immediate = 0, // Transfer all at once
    Block = 1,     // Sync to DMA request
    LinkedList = 2 // Linked list mode (GPU)
  };

  SyncMode sync_mode() const {
    return static_cast<SyncMode>((channel_ctrl >> 9) & 0x3);
  }

  u16 block_size() const { return static_cast<u16>(block_ctrl & 0xFFFF); }
  u16 block_count() const { return static_cast<u16>(block_ctrl >> 16); }

  bool is_active() const {
    bool en = enabled();
    if (sync_mode() == SyncMode::Immediate) {
      return en && trigger();
    }
    return en;
  }
};

class DmaController {
public:
  struct TransferDebug {
    u32 base_addr = 0;
    u32 block_ctrl = 0;
    u32 channel_ctrl = 0;
    u32 transfer_words = 0;
    u32 first_addr = 0;
    u32 last_addr = 0;
    u64 cpu_cycle = 0;
    bool from_ram = false;
  };

  void init(System *sys) { sys_ = sys; }
  void reset();

  u32 read(u32 offset) const;
  void write(u32 offset, u32 value);

  void tick();
  const TransferDebug &last_debug(int channel) const {
    return last_debug_[channel & 0x7];
  }

private:
  System *sys_ = nullptr;
  DmaChannel channels_[7];
  TransferDebug last_debug_[7];
  bool mdec_out_reorder_active_ = false;
  u32 mdec_out_mb_base_addr_ = 0;
  u8 mdec_out_block_id_ = 0xFF;
  u32 mdec_out_word_index_in_block_ = 0;
  u32 mdec_out_last_linear_addr_ = 0;
  s32 mdec_out_last_step_ = 0;

  u32 dpcr_ = 0x07654321; // DMA control register (priority/enable)
  u32 dicr_ = 0;          // DMA interrupt register

  void recompute_dicr_master(bool request_irq_on_rise);
  void reset_mdec_out_reorder_state();
  u32 map_mdec_out_word_addr(u32 linear_addr, s32 step, u8 block_id, u8 depth);
  void execute_dma(int channel);
  void dma_block(int channel);
  void dma_linked_list(int channel);
  void transfer_complete(int channel);
  bool request_active(int channel) const;

  bool channel_enabled(int ch) const { return (dpcr_ >> (ch * 4 + 3)) & 1; }
};
