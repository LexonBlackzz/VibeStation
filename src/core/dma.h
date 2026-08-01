#pragma once
#include "types.h"
#include <array>
#include <vector>

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
  u16 raw_block_count() const { return static_cast<u16>(block_ctrl >> 16); }
  u32 block_count() const {
    const u16 count = raw_block_count();
    return count == 0 ? 0x10000u : count;
  }

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
    u32 id = 0;
    u32 base_addr = 0;
    u32 block_ctrl = 0;
    u32 channel_ctrl = 0;
    u32 transfer_words = 0;
    u32 first_addr = 0;
    u32 last_addr = 0;
    u64 cpu_cycle = 0;
    u64 cd_stream_generation = 0;
    s32 source_lba = -1;
    s32 cd_stream_start_lba = -1;
    s32 cd_next_read_lba = -1;
    u32 cd_buffer_state = 0;
    u32 cd_command_state = 0;
    bool from_ram = false;
  };
  struct RegisterWriteDebug {
    u32 madr_pc = 0;
    u32 bcr_pc = 0;
    u32 chcr_pc = 0;
    u64 madr_cycle = 0;
    u64 bcr_cycle = 0;
    u64 chcr_cycle = 0;
  };

  DmaController();
  ~DmaController();

  void init(System *sys) { sys_ = sys; }
  void reset();

  void save_state(std::vector<u8>& buf) const;
  void restore_state(const u8*& pos, size_t& remaining);

  u32 read(u32 offset) const;
  void write(u32 offset, u32 value);

  void tick();
  const TransferDebug &last_debug(int channel) const {
    return last_debug_[channel & 0x7];
  }
  u32 active_transfer_debug_id(int channel) const {
    return active_transfer_debug_id_[channel & 0x7];
  }
  const TransferDebug *transfer_debug(u32 id) const;
  const RegisterWriteDebug &last_register_write_debug(int channel) const {
    return register_write_debug_[channel & 0x7];
  }

private:
  System *sys_ = nullptr;
  DmaChannel channels_[7];
  TransferDebug last_debug_[7];
  // Spyro performs a very large number of short CD DMA slices before the
  // eventual fault. Keep a session-scale history so RAM provenance still
  // resolves the original transfer rather than falling back to the newest one.
  static constexpr size_t kTransferDebugHistorySize = 262144u;
  std::vector<TransferDebug> transfer_debug_history_;
  std::array<u32, 7> active_transfer_debug_id_{};
  u32 next_transfer_debug_id_ = 0;
  RegisterWriteDebug register_write_debug_[7];

  u32 dpcr_ = 0x07654321; // DMA control register (priority/enable)
  u32 dicr_ = 0;          // DMA interrupt register

  void recompute_dicr_master(bool request_irq_on_rise);
  void execute_dma(int channel);
  void dma_block(int channel, u32 max_words = 0xFFFFFFFFu);
  void dma_linked_list(int channel);
  void transfer_complete(int channel);
  bool request_active(int channel) const;
  u32 dma_ram_tick_cost(u32 word_count) const;
  u32 slice_words_for_channel(int channel) const;

  bool channel_enabled(int ch) const { return (dpcr_ >> (ch * 4 + 3)) & 1; }
  u8 channel_priority(int ch) const { return (dpcr_ >> (ch * 4)) & 0x7; }
};
