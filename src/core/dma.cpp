#include "dma.h"
#include "system.h"
#include <algorithm>
#include <chrono>

void DmaController::recompute_dicr_master(bool request_irq_on_rise) {
  const bool old_master = (dicr_ & 0x80000000u) != 0;
  const bool force_irq = ((dicr_ >> 15) & 1u) != 0;
  const bool master_enable = ((dicr_ >> 23) & 1u) != 0;
  const u32 channel_flags = (dicr_ >> 24) & 0x7Fu;
  const u32 channel_enables = (dicr_ >> 16) & 0x7Fu;
  const bool any_triggered = (channel_flags & channel_enables) != 0;
  const bool master = force_irq || (master_enable && any_triggered);

  if (master) {
    dicr_ |= 0x80000000u;
  } else {
    dicr_ &= ~0x80000000u;
  }

  if (request_irq_on_rise && !old_master && master) {
    sys_->irq().request(Interrupt::DMA);
  }
}

void DmaController::reset() {
  for (auto &ch : channels_) {
    ch = {};
  }
  for (auto &dbg : last_debug_) {
    dbg = {};
  }
  dpcr_ = 0x07654321;
  dicr_ = 0;
  reset_mdec_out_reorder_state();
}

void DmaController::reset_mdec_out_reorder_state() {
  mdec_out_reorder_active_ = false;
  mdec_out_mb_base_addr_ = 0;
  mdec_out_block_id_ = 0xFF;
  mdec_out_word_index_in_block_ = 0;
  mdec_out_last_linear_addr_ = 0;
  mdec_out_last_step_ = 0;
}

u32 DmaController::map_mdec_out_word_addr(u32 linear_addr, s32 step, u8 block_id,
                                          u8 depth) {
  const u32 addr = linear_addr & 0x001FFFFCu;
  if (g_mdec_debug_disable_dma1_reorder) {
    reset_mdec_out_reorder_state();
    return addr;
  }
  // Color MDEC output arrives as four 8x8 blocks per macroblock.
  // DMA1 must reorder those blocks into a 16x16 raster in RAM.
  if (step != 4 || block_id >= 4u || (depth != 2u && depth != 3u)) {
    reset_mdec_out_reorder_state();
    return addr;
  }

  if (mdec_out_reorder_active_) {
    const u32 expected =
        static_cast<u32>(static_cast<s32>(mdec_out_last_linear_addr_) +
                         mdec_out_last_step_) &
        0x001FFFFCu;
    const bool discontinuity =
        (step != mdec_out_last_step_) || (addr != expected);
    if (discontinuity) {
      reset_mdec_out_reorder_state();
    }
  }

  const u32 words_per_row_in_block = (depth == 2u) ? 6u : 4u;
  const u32 words_per_block = words_per_row_in_block * 8u;
  const u32 macroblock_row_words = words_per_row_in_block * 2u;

  if (!mdec_out_reorder_active_) {
    mdec_out_reorder_active_ = true;
    mdec_out_mb_base_addr_ = addr;
    mdec_out_block_id_ = block_id;
    mdec_out_word_index_in_block_ = 0;
  } else if (block_id != mdec_out_block_id_) {
    mdec_out_block_id_ = block_id;
    mdec_out_word_index_in_block_ = 0;
    if (block_id == 0u) {
      mdec_out_mb_base_addr_ = addr;
    }
  } else if (block_id == 0u && mdec_out_word_index_in_block_ == 0u) {
    mdec_out_mb_base_addr_ = addr;
  }

  const u32 word_index = mdec_out_word_index_in_block_++;
  if (mdec_out_word_index_in_block_ >= words_per_block) {
    mdec_out_word_index_in_block_ = 0;
  }

  const u32 row = word_index / words_per_row_in_block;
  const u32 col = word_index % words_per_row_in_block;
  const u32 block_x_words = (block_id & 0x1u) ? words_per_row_in_block : 0u;
  const u32 block_y_rows = (block_id >= 2u) ? 8u : 0u;
  const u32 offset_words =
      (block_y_rows + row) * macroblock_row_words + block_x_words + col;
  mdec_out_last_linear_addr_ = addr;
  mdec_out_last_step_ = step;
  return (mdec_out_mb_base_addr_ + offset_words * 4u) & 0x001FFFFCu;
}

u32 DmaController::read(u32 offset) const {
  // Channels are at 0x80-0xFF in the DMA register space
  // 0x1F801080 + channel * 0x10 + register
  if (offset < 0x70) {
    int channel = (offset >> 4) & 0x7;
    int reg = offset & 0xF;

    if (channel >= 7) {
      LOG_WARN("DMA: Read from invalid channel %d", channel);
      return 0;
    }

    const auto &ch = channels_[channel];
    switch (reg) {
    case 0x0:
      return ch.base_addr;
    case 0x4:
      return ch.block_ctrl;
    case 0x8:
      return ch.channel_ctrl;
    default:
      LOG_WARN("DMA: Unhandled channel %d reg 0x%X read", channel, reg);
      return 0;
    }
  }

  switch (offset) {
  case 0x70:
    return dpcr_;
  case 0x74:
    return dicr_;
  default:
    LOG_WARN("DMA: Unhandled read at offset 0x%X", offset);
    return 0;
  }
}

void DmaController::write(u32 offset, u32 value) {
  if (offset < 0x70) {
    int channel = (offset >> 4) & 0x7;
    int reg = offset & 0xF;

    if (channel >= 7) {
      LOG_WARN("DMA: Write to invalid channel %d", channel);
      return;
    }

    auto &ch = channels_[channel];
    switch (reg) {
    case 0x0:
      ch.base_addr = value & 0x00FFFFFF; // 24-bit address
      break;
    case 0x4:
      ch.block_ctrl = value;
      ch.block_words_remaining = 0;
      break;
    case 0x8:
      ch.block_words_remaining = 0;
      ch.channel_ctrl = value;
      if (g_trace_dma) {
        static u64 chcr_log_count = 0;
        if (trace_should_log(chcr_log_count, g_trace_burst_dma,
                             g_trace_stride_dma)) {
          LOG_DEBUG("DMA: CH%d CHCR=0x%08X (enabled=%d trigger=%d sync=%d "
                    "dpcr_en=%d)",
                    channel, ch.channel_ctrl, ch.enabled() ? 1 : 0,
                    ch.trigger() ? 1 : 0, static_cast<int>(ch.sync_mode()),
                    channel_enabled(channel) ? 1 : 0);
        }
      }
      if (ch.is_active() && channel_enabled(channel) && request_active(channel)) {
        execute_dma(channel);
      }
      break;
    default:
      LOG_WARN("DMA: Unhandled channel %d reg 0x%X write = 0x%08X", channel,
               reg, value);
      break;
    }
    return;
  }

  switch (offset) {
  case 0x70:
    dpcr_ = value;
    if (g_trace_dma) {
      static u64 dpcr_log_count = 0;
      if (trace_should_log(dpcr_log_count, g_trace_burst_dma,
                           g_trace_stride_dma)) {
        LOG_DEBUG("DMA: DPCR=0x%08X", dpcr_);
      }
    }
    break;
  case 0x74: {
    // DICR: bits 24-30 are W1C flags, bit31 is derived master-IRQ flag.
    const u32 old_flags = (dicr_ >> 24) & 0x7Fu;
    const u32 ack_flags = (value >> 24) & 0x7Fu;
    const u32 new_flags = old_flags & ~ack_flags;
    const u32 writable_control = value & 0x00FF803Fu; // 0-5,15,16-23
    const u32 old_master = dicr_ & 0x80000000u;
    dicr_ = old_master | writable_control | (new_flags << 24);
    recompute_dicr_master(true);
    break;
  }
  default:
    LOG_WARN("DMA: Unhandled write at offset 0x%X = 0x%08X", offset, value);
    break;
  }
}

void DmaController::execute_dma(int channel) {
  auto &ch = channels_[channel];
  if (g_trace_dma) {
    static u64 exec_log_count = 0;
    if (trace_should_log(exec_log_count, g_trace_burst_dma,
                         g_trace_stride_dma)) {
      LOG_DEBUG("DMA: Execute ch=%d madr=0x%08X bcr=0x%08X chcr=0x%08X sync=%d "
                "from_ram=%d",
                channel, ch.base_addr, ch.block_ctrl, ch.channel_ctrl,
                static_cast<int>(ch.sync_mode()), ch.from_ram() ? 1 : 0);
    }
  }

  switch (ch.sync_mode()) {
  case DmaChannel::SyncMode::Immediate:
    dma_block(channel);
    transfer_complete(channel);
    break;
  case DmaChannel::SyncMode::Block:
    dma_block(channel);
    if (ch.block_count() == 0 && ch.block_words_remaining == 0) {
      transfer_complete(channel);
    }
    break;
  case DmaChannel::SyncMode::LinkedList:
    dma_linked_list(channel);
    transfer_complete(channel);
    break;
  }
}

void DmaController::dma_block(int channel) {
  auto &ch = channels_[channel];

  u32 addr = ch.base_addr;
  u32 word_count = 0;

  if (ch.sync_mode() == DmaChannel::SyncMode::Immediate) {
    word_count = ch.block_size();
    if (word_count == 0)
      word_count = 0x10000;
  } else {
    if (ch.block_words_remaining == 0) {
      word_count = ch.block_size();
      if (word_count == 0) {
        word_count = 0x10000;
      }
      ch.block_words_remaining = word_count;
    }
    word_count = ch.block_words_remaining;
  }

  u32 transfer_words = word_count;

  s32 step;
  if ((ch.channel_ctrl >> 1) & 1) {
    step = -4;
  } else {
    step = 4;
  }

  bool from_ram = ch.from_ram();
  if (channel == 1 && from_ram) {
    reset_mdec_out_reorder_state();
  }
  auto &dbg = last_debug_[channel];
  dbg.base_addr = addr & 0x00FFFFFFu;
  dbg.block_ctrl = ch.block_ctrl;
  dbg.channel_ctrl = ch.channel_ctrl;
  dbg.transfer_words = transfer_words;
  dbg.cpu_cycle = sys_ ? sys_->cpu().cycle_count() : 0;
  dbg.from_ram = from_ram;
  dbg.first_addr = addr & 0x001FFFFCu;
  if (transfer_words != 0) {
    const s32 span = step * static_cast<s32>(transfer_words - 1);
    dbg.last_addr =
        static_cast<u32>(static_cast<s32>(dbg.first_addr) + span) & 0x001FFFFCu;
  } else {
    dbg.last_addr = dbg.first_addr;
  }

  if (!from_ram && ch.sync_mode() == DmaChannel::SyncMode::Block) {
    switch (channel) {
    case 1:
      transfer_words =
          std::min(transfer_words, sys_->mdec_dma_out_words_available());
      break;
    case 3:
      transfer_words = std::min(transfer_words, sys_->cdrom_dma_words_available());
      break;
    default:
      break;
    }
    if (transfer_words == 0) {
      return;
    }
  }

  if (!from_ram && channel == 1 && g_mdec_debug_upload_probe) {
    sys_->debug_note_mdec_dma_out_begin(addr & 0x001FFFFCu, transfer_words,
                                        sys_->mdec_dma_out_depth(),
                                        sys_->mdec_dma_out_block());
  }

  if (channel == 6) {
    // OTC (Ordering Table Clear): always writes to RAM, backwards.
    u32 current = addr;
    for (u32 i = 0; i < transfer_words; i++) {
      u32 val;
      if (i == transfer_words - 1) {
        val = 0x00FFFFFF; // End-of-list marker
      } else {
        val = (current - 4) & 0x001FFFFC;
      }
      sys_->write32(current, val);
      current -= 4;
    }
    if (ch.sync_mode() == DmaChannel::SyncMode::Block) {
      ch.base_addr = current & 0x00FFFFFF;
      if (ch.block_words_remaining > transfer_words) {
        ch.block_words_remaining -= transfer_words;
      } else {
        ch.block_words_remaining = 0;
      }
      if (ch.block_words_remaining == 0) {
        const u16 remaining = ch.block_count();
        const u16 next = (remaining > 0) ? static_cast<u16>(remaining - 1) : 0;
        ch.block_ctrl =
            (ch.block_ctrl & 0x0000FFFFu) | (static_cast<u32>(next) << 16);
      }
    }
    return;
  }

  for (u32 i = 0; i < transfer_words; i++) {
    u32 current_addr = addr & 0x001FFFFC;

    if (from_ram) {
      sys_->debug_begin_dma_bus_access(static_cast<u8>(channel));
      u32 data = sys_->read32(current_addr);
      sys_->debug_end_dma_bus_access();
      // Send to device
      switch (channel) {
      case 0: // MDEC in
        sys_->mdec_dma_write(data);
        break;
      case 2: // GPU
        sys_->gpu_gp0_dma(data, current_addr);
        break;
      case 4: // SPU
        sys_->spu_dma_write(data);
        break;
      default:
        LOG_WARN("DMA: Unhandled from-RAM channel %d", channel);
        break;
      }
    } else {
      u32 data = 0;
      u32 write_addr = current_addr;
      // Read from device
      switch (channel) {
      case 1: { // MDEC out
        const u32 macroblock_seq = sys_->mdec_dma_out_macroblock_seq();
        write_addr = map_mdec_out_word_addr(current_addr, step,
                                            sys_->mdec_dma_out_block(),
                                            sys_->mdec_dma_out_depth());
        data = sys_->mdec_dma_read();
        if (g_mdec_debug_upload_probe) {
          sys_->debug_note_mdec_dma_out_word(write_addr, data, macroblock_seq);
        }
        break;
      }
      case 2: // GPU (GPUREAD)
        data = sys_->gpu_read();
        break;
      case 3: // CDROM
        data = sys_->cdrom_dma_read();
        break;
      case 4: // SPU
        data = sys_->spu_dma_read();
        break;
      default:
        LOG_WARN("DMA: Unhandled to-RAM channel %d", channel);
        break;
      }
      sys_->debug_begin_dma_bus_access(static_cast<u8>(channel));
      sys_->write32(write_addr, data);
      sys_->debug_end_dma_bus_access();
    }

    addr = static_cast<u32>(static_cast<s32>(addr) + step);
  }

  ch.base_addr = addr & 0x00FFFFFF;
  if (ch.sync_mode() == DmaChannel::SyncMode::Block) {
    if (ch.block_words_remaining > transfer_words) {
      ch.block_words_remaining -= transfer_words;
    } else {
      ch.block_words_remaining = 0;
    }
    if (ch.block_words_remaining == 0) {
      const u16 remaining = ch.block_count();
      const u16 next = (remaining > 0) ? static_cast<u16>(remaining - 1) : 0;
      ch.block_ctrl =
          (ch.block_ctrl & 0x0000FFFFu) | (static_cast<u32>(next) << 16);
    }
  }
}

void DmaController::dma_linked_list(int channel) {
  // Only GPU (channel 2) uses linked list mode
  if (channel != 2) {
    LOG_WARN("DMA: Linked list mode on non-GPU channel %d", channel);
    return;
  }

  u32 addr = channels_[channel].base_addr & 0x001FFFFC;
  u32 safety = 0;

  while (safety < 0x100000) {
    sys_->debug_begin_dma_bus_access(static_cast<u8>(channel));
    u32 header = sys_->read32(addr);
    sys_->debug_end_dma_bus_access();
    u32 word_count = header >> 24;

    // Send words to GP0
    for (u32 i = 0; i < word_count; i++) {
      addr = (addr + 4) & 0x001FFFFC;
      sys_->debug_begin_dma_bus_access(static_cast<u8>(channel));
      u32 command = sys_->read32(addr);
      sys_->debug_end_dma_bus_access();
      sys_->gpu_gp0_dma(command, addr);
    }

    // Follow link to next node
    if (header & 0x00800000) {
      break; // End-of-list marker
    }

    addr = header & 0x001FFFFC;
    safety++;
  }

  if (safety >= 0x100000) {
    LOG_ERROR("DMA: Linked list loop detected!");
  }
}

void DmaController::transfer_complete(int channel) {
  auto &ch = channels_[channel];

  // Clear enable + trigger bits
  ch.channel_ctrl &= ~(1u << 24); // Disable
  ch.channel_ctrl &= ~(1u << 28); // Clear trigger
  ch.block_words_remaining = 0;
  // Keep MDEC-out reorder state across DMA1 transfer boundaries.
  // Some clients issue a sequence of DMA1 transfers while consuming one
  // continuous MDEC output stream; resetting here can desynchronize macroblock
  // base tracking when a transfer ends mid-macroblock.

  // Set completion flag in DICR and update IRQ state.
  dicr_ |= (1u << (24 + channel));
  recompute_dicr_master(true);
}

void DmaController::tick() {
  const bool profile_detailed = g_profile_detailed_timing;
  // Check if any channels need to start
  for (int i = 0; i < 7; i++) {
    if (channels_[i].is_active() && channel_enabled(i) && request_active(i)) {
      std::chrono::high_resolution_clock::time_point start{};
      if (profile_detailed) {
        start = std::chrono::high_resolution_clock::now();
      }
      execute_dma(i);
      if (profile_detailed && sys_) {
        const auto end = std::chrono::high_resolution_clock::now();
        sys_->add_dma_time(
            std::chrono::duration<double, std::milli>(end - start).count());
      }
    }
  }
}

bool DmaController::request_active(int channel) const {
  const DmaChannel &ch = channels_[channel];
  if (ch.sync_mode() == DmaChannel::SyncMode::Immediate) {
    return true;
  }
  if (ch.sync_mode() == DmaChannel::SyncMode::LinkedList) {
    return true;
  }

  switch (channel) {
  case 0:
    return sys_->mdec_dma_in_request();
  case 1:
    return sys_->mdec_dma_out_request();
  case 2:
    return sys_->gpu_dma_request();
  case 3:
    return sys_->cdrom_dma_request();
  case 4:
    return sys_->spu_dma_request();
  default:
    return true;
  }
}
