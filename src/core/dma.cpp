#include "dma.h"
#include "system.h"
#include <algorithm>
#include <chrono>

namespace {
constexpr u32 kStreamingDmaSliceWords = 96u;
constexpr u32 kDicrWriteMask = 0x00FF807Fu;
constexpr u32 kDicrResetMask = 0x7F000000u;

bool is_streaming_dma_channel(int channel) {
  return channel == 0 || channel == 1;
}

u32 sanitize_corrupt_chcr(int channel, u32 value) {
  constexpr u32 kChcrLegalMask =
      (1u << 0) | (1u << 1) | (1u << 8) | (0x3u << 9) |
      (0x7u << 16) | (0x7u << 20) | (1u << 24) | (1u << 28);
  u32 sanitized = value;

  const bool has_command_byte = (sanitized & 0xFF000000u) != 0;
  if (has_command_byte) {
    sanitized |= (1u << 24) | (1u << 28);
  }

  bool trigger = (sanitized & (1u << 28)) != 0;
  bool enabled = (sanitized & (1u << 24)) != 0;
  if (trigger && !enabled) {
    sanitized |= (1u << 24);
    enabled = true;
  }
  if (enabled && !trigger && (((sanitized >> 9) & 0x3u) == 0u)) {
    sanitized |= (1u << 28);
  }

  const u32 sync = (sanitized >> 9) & 0x3u;
  if (sync == 3u) {
    sanitized &= ~(0x3u << 9);
    sanitized |= ((channel == 2) ? 2u : 1u) << 9;
  }

  if (((sanitized >> 9) & 0x3u) == 2u && channel != 2) {
    sanitized &= ~(0x3u << 9);
    sanitized |= 1u << 9;
  }

  switch (channel) {
  case 0: // MDEC in: RAM -> device
  case 2: // GPU: linked-list and GP0 DMA are RAM -> device
    sanitized |= 1u;
    break;
  case 1: // MDEC out: device -> RAM
  case 3: // CDROM: device -> RAM
  case 6: // OTC: writes ordering table entries to RAM
    sanitized &= ~1u;
    break;
  default:
    break;
  }

  if (channel == 6) {
    sanitized |= 1u << 1;
    sanitized &= ~(0x3u << 9);
  }

  return sanitized & kChcrLegalMask;
}
} // namespace

u32 DmaController::slice_words_for_channel(int channel) const {
  if (!is_streaming_dma_channel(channel)) {
    return 0xFFFFFFFFu;
  }

  if (sys_ == nullptr) {
    return kStreamingDmaSliceWords;
  }

  // MDEC output is generated in macroblock-sized chunks. Let a DMA1 slice move
  // one complete ready macroblock so GPU uploads do not see a half-populated
  // staging area while still allowing the request line to drop between blocks.
  switch (sys_->mdec_dma_out_depth()) {
  case 2: // 24-bit color, 16x16x3 bytes
    return 192u;
  case 3: // 15-bit color, 16x16x2 bytes
    return 128u;
  case 1: // 8-bit mono, 16x16 bytes
    return 64u;
  case 0: // 4-bit mono, 16x16 / 2 bytes
    return 32u;
  default:
    return kStreamingDmaSliceWords;
  }
}

void DmaController::recompute_dicr_master(bool request_irq_on_rise) {
  const bool old_master = (dicr_ & 0x80000000u) != 0;
  const bool force_irq = ((dicr_ >> 15) & 1u) != 0;
  const bool master_enable = ((dicr_ >> 23) & 1u) != 0;
  const u32 channel_flags = (dicr_ >> 24) & 0x7Fu;
  const bool master = force_irq || (master_enable && channel_flags != 0);

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
      LOG_WARN("DMA: Unhandled channel %d reg 0x%X read pc=0x%08X cyc=%llu",
               channel, reg, sys_ ? sys_->cpu().pc() : 0,
               sys_ ? static_cast<unsigned long long>(sys_->cpu().cycle_count())
                    : 0ull);
      return 0xFFFFFFFFu;
    }
  }

  switch (offset) {
  case 0x70:
    return dpcr_;
  case 0x74:
    return dicr_;
  default:
    LOG_WARN("DMA: Unhandled read at offset 0x%X pc=0x%08X cyc=%llu", offset,
             sys_ ? sys_->cpu().pc() : 0,
             sys_ ? static_cast<unsigned long long>(sys_->cpu().cycle_count())
                  : 0ull);
    return 0xFFFFFFFFu;
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
      if (g_log_fmv_diagnostics && channel == 1) {
        LOG_WARN(
            "DMA: ch1 MADR write value=0x%08X bcr=0x%08X chcr=0x%08X cyc=%llu",
            ch.base_addr, ch.block_ctrl, ch.channel_ctrl,
            static_cast<unsigned long long>(sys_ ? sys_->cpu().cycle_count()
                                                 : 0ull));
      }
      break;
    case 0x4:
      ch.block_ctrl = value;
      ch.block_words_remaining = 0;
      if (g_log_fmv_diagnostics && channel == 1) {
        LOG_WARN(
            "DMA: ch1 BCR write value=0x%08X madr=0x%08X chcr=0x%08X cyc=%llu",
            value, ch.base_addr, ch.channel_ctrl,
            static_cast<unsigned long long>(sys_ ? sys_->cpu().cycle_count()
                                                 : 0ull));
      }
      break;
    case 0x8: {
      ch.block_words_remaining = 0;
      if (g_experimental_dma_command_sanitizer) {
        const u32 sanitized = sanitize_corrupt_chcr(channel, value);
        if (sanitized != value) {
          LOG_WARN("DMA: Sanitized CH%d CHCR 0x%08X -> 0x%08X",
                   channel, value, sanitized);
        }
        value = sanitized;
      }
      ch.channel_ctrl = value;
      if (g_log_fmv_diagnostics && channel == 1) {
        LOG_WARN(
            "DMA: ch1 CHCR write value=0x%08X madr=0x%08X bcr=0x%08X req=%u active=%u cyc=%llu",
            value, ch.base_addr, ch.block_ctrl, request_active(channel) ? 1u : 0u,
            ch.is_active() ? 1u : 0u,
            static_cast<unsigned long long>(sys_ ? sys_->cpu().cycle_count()
                                                 : 0ull));
      }
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
    }
    default:
      LOG_WARN("DMA: Unhandled channel %d reg 0x%X write = 0x%08X", channel,
               reg, value);
      break;
    }
    if (channel == 3 && g_log_fmv_diagnostics) {
      static u32 cdrom_dma_reg_write_logs = 0;
      CdRom &cd = sys_->cdrom();
      const bool fmv_sector = cd.mode() == 0xE0u || cd.dma_data_index() == 44 ||
                              sys_->cdrom_dma_words_available() == 504u;
      if (fmv_sector && cdrom_dma_reg_write_logs < 512u) {
        ++cdrom_dma_reg_write_logs;
        LOG_INFO(
            "DMA: CDROM reg write reg=0x%X val=0x%08X pc=0x%08X cyc=%llu "
            "madr=0x%08X bcr=0x%08X chcr=0x%08X active=%u dpcr_en=%u "
            "req_active=%u cd_idx=%d cd_avail=%u cd_size=%zu cd_ready=%u "
            "cd_req=%u cd_mode=0x%02X cd_lba=%d",
            reg, value, sys_ ? sys_->cpu().pc() : 0u,
            static_cast<unsigned long long>(sys_ ? sys_->cpu().cycle_count() : 0u),
            ch.base_addr, ch.block_ctrl, ch.channel_ctrl, ch.is_active() ? 1u : 0u,
            channel_enabled(channel) ? 1u : 0u, request_active(channel) ? 1u : 0u,
            cd.dma_data_index(), sys_->cdrom_dma_words_available(),
            cd.dma_buffer_size(), cd.sector_data_ready() ? 1u : 0u,
            cd.sector_data_request() ? 1u : 0u, static_cast<unsigned>(cd.mode()),
            cd.current_read_lba());
      }
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
    // DICR: bits 24-30 are W1C flags, bit31 is derived.
    const u32 old_dicr = dicr_;
    dicr_ = (dicr_ & ~kDicrWriteMask) | (value & kDicrWriteMask);
    dicr_ &= ~(value & kDicrResetMask);
    recompute_dicr_master(true);
    if (g_log_fmv_diagnostics) {
      static u32 dicr_write_log_count = 0;
      const u64 cycle = sys_ ? sys_->cpu().cycle_count() : 0ull;
      if (dicr_write_log_count < 256u || cycle >= 880000000ull) {
        ++dicr_write_log_count;
        LOG_WARN(
            "DMA: DICR write val=0x%08X old=0x%08X new=0x%08X pc=0x%08X cyc=%llu",
            value, old_dicr, dicr_, sys_ ? sys_->cpu().pc() : 0u,
            static_cast<unsigned long long>(cycle));
      }
    }
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
    dma_block(channel, slice_words_for_channel(channel));
    if (ch.block_words_remaining == 0) {
      transfer_complete(channel);
    }
    break;
  case DmaChannel::SyncMode::Block:
  {
    u32 words_budget = slice_words_for_channel(channel);
    u32 safety = 0;
    bool completed_request_transfer = false;
    while (ch.is_active() && channel_enabled(channel) && request_active(channel) &&
           (ch.block_count() != 0 || ch.block_words_remaining != 0) &&
           safety++ < 0x10000u) {
      const u16 before_count = ch.raw_block_count();
      const u32 before_remaining = ch.block_words_remaining;
      const u32 before_addr = ch.base_addr;
      const u32 max_words = (words_budget == 0xFFFFFFFFu) ? 0xFFFFFFFFu : words_budget;
      dma_block(channel, max_words);
      const u32 moved_words = last_debug_[channel].transfer_words;
      if (moved_words == 0) {
        break;
      }

      if (ch.raw_block_count() == 0u && ch.block_words_remaining == 0u) {
        completed_request_transfer = true;
        break;
      }

      if (words_budget != 0xFFFFFFFFu) {
        if (moved_words >= words_budget) {
          break;
        }
        words_budget -= moved_words;
      }

      if (before_count == ch.block_count() &&
          before_remaining == ch.block_words_remaining &&
          before_addr == ch.base_addr) {
        break;
      }
    }
    if (completed_request_transfer) {
      transfer_complete(channel);
    }
    break;
  }
  case DmaChannel::SyncMode::LinkedList:
    dma_linked_list(channel);
    transfer_complete(channel);
    break;
  }
}

u32 DmaController::dma_ram_tick_cost(u32 word_count) const {
  if (word_count == 0) {
    return 0;
  }

  // DuckStation models DMA RAM access as roughly one tick per word plus a
  // small row-reload overhead per 16-word chunk.
  return word_count + ((word_count + 15u) / 16u);
}

void DmaController::dma_block(int channel, u32 max_words) {
  auto &ch = channels_[channel];

  u32 addr = ch.base_addr;
  u32 word_count = 0;
  const bool immediate_mode =
      ch.sync_mode() == DmaChannel::SyncMode::Immediate;
  const bool block_mode = ch.sync_mode() == DmaChannel::SyncMode::Block;

  if (immediate_mode) {
    if (ch.block_words_remaining == 0) {
      word_count = ch.block_size();
      if (word_count == 0)
        word_count = 0x10000;
      ch.block_words_remaining = word_count;
    }
    word_count = ch.block_words_remaining;
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

  u32 transfer_words = std::min(word_count, max_words);

  s32 step;
  if ((ch.channel_ctrl >> 1) & 1) {
    step = -4;
  } else {
    step = 4;
  }

  bool from_ram = ch.from_ram();
  int cdrom_dma_active_lba = -1;
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

  if (!from_ram && (immediate_mode || block_mode)) {
    const u32 requested_words = transfer_words;
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
    if (transfer_words != requested_words) {
      dbg.transfer_words = transfer_words;
    }
    if (channel == 3 && g_log_fmv_diagnostics) {
      static u32 cdrom_dma_begin_logs = 0;
      static u32 cdrom_dma_whole_begin_logs = 0;
      static u32 cdrom_dma_e0_begin_logs = 0;
      static u32 cdrom_dma_c0_begin_logs = 0;
      const CdRom &cd = sys_->cdrom();
      const bool whole_sector = cd.read_whole_sector();
      const bool e0_sector = cd.mode() == 0xE0u;
      const bool c0_sector = cd.mode() == 0xC0u;
      cdrom_dma_active_lba = cd.active_data_lba();
      if (cdrom_dma_begin_logs < 256u ||
          (whole_sector && cdrom_dma_whole_begin_logs < 512u) ||
          (e0_sector && cdrom_dma_e0_begin_logs < 512u) ||
          (c0_sector && cdrom_dma_c0_begin_logs < 512u)) {
        ++cdrom_dma_begin_logs;
        if (whole_sector) {
          ++cdrom_dma_whole_begin_logs;
        }
        if (e0_sector) {
          ++cdrom_dma_e0_begin_logs;
        }
        if (c0_sector) {
          ++cdrom_dma_c0_begin_logs;
        }
        LOG_INFO(
            "DMA: CDROM begin madr=0x%08X requested=%u actual=%u slice=%u avail=%u "
            "idx=%d size=%zu ready=%u req=%u mode=0x%02X whole=%u lba=%d "
            "bcr=0x%08X chcr=0x%08X cyc=%llu",
            ch.base_addr & 0x00FFFFFFu, requested_words, transfer_words,
            max_words, sys_->cdrom_dma_words_available(), cd.dma_data_index(),
            cd.dma_buffer_size(), cd.sector_data_ready() ? 1u : 0u,
            cd.sector_data_request() ? 1u : 0u, static_cast<unsigned>(cd.mode()),
            cd.read_whole_sector() ? 1u : 0u, cdrom_dma_active_lba,
            ch.block_ctrl, ch.channel_ctrl,
            static_cast<unsigned long long>(sys_->cpu().cycle_count()));
      }
    }
    if (transfer_words == 0) {
      return;
    }
  }

  if (from_ram && channel == 0 && g_mdec_debug_upload_probe) {
    sys_->debug_note_mdec_dma_in_begin(addr & 0x001FFFFCu, transfer_words);
  }

  if (!from_ram && channel == 1 && g_mdec_debug_upload_probe) {
    sys_->debug_note_mdec_dma_out_begin(addr & 0x001FFFFCu, transfer_words,
                                        sys_->mdec_dma_out_depth(),
                                        sys_->mdec_dma_out_block());
  }

  u32 cdrom_dma_first_word = 0;
  u32 cdrom_dma_last_word = 0;
  u32 cdrom_dma_sample_words[8] = {};
  u32 cdrom_dma_sample_count = 0;
  bool cdrom_dma_saw_word = false;

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
    if (immediate_mode || block_mode) {
      ch.base_addr = current & 0x00FFFFFF;
      if (ch.block_words_remaining > transfer_words) {
        ch.block_words_remaining -= transfer_words;
      } else {
        ch.block_words_remaining = 0;
      }
      if (block_mode && ch.block_words_remaining == 0) {
        const u32 remaining = ch.block_count();
        const u16 next = (remaining > 0) ? static_cast<u16>(remaining - 1) : 0;
        ch.block_ctrl =
            (ch.block_ctrl & 0x0000FFFFu) | (static_cast<u32>(next) << 16);
      }
    }
    if (sys_ != nullptr) {
      sys_->add_cpu_cycle_penalty(dma_ram_tick_cost(transfer_words));
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
        if (g_mdec_debug_upload_probe) {
          sys_->debug_note_mdec_dma_in_word(current_addr, data);
        }
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
        if (g_log_fmv_diagnostics) {
          if (!cdrom_dma_saw_word) {
            cdrom_dma_first_word = data;
            cdrom_dma_saw_word = true;
          }
          cdrom_dma_last_word = data;
          if (cdrom_dma_sample_count < 8u) {
            cdrom_dma_sample_words[cdrom_dma_sample_count++] = data;
          }
        }
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
  if (immediate_mode || block_mode) {
    if (ch.block_words_remaining > transfer_words) {
      ch.block_words_remaining -= transfer_words;
    } else {
      ch.block_words_remaining = 0;
    }
    if (block_mode && ch.block_words_remaining == 0) {
      const u32 remaining = ch.block_count();
      const u16 next = (remaining > 0) ? static_cast<u16>(remaining - 1) : 0;
      ch.block_ctrl =
          (ch.block_ctrl & 0x0000FFFFu) | (static_cast<u32>(next) << 16);
    }
  }

  if (sys_ != nullptr) {
    sys_->add_cpu_cycle_penalty(dma_ram_tick_cost(transfer_words));
  }

  if (!from_ram && channel == 3 && g_log_fmv_diagnostics) {
    static u32 cdrom_dma_end_logs = 0;
    static u32 cdrom_dma_whole_end_logs = 0;
    static u32 cdrom_dma_e0_end_logs = 0;
    static u32 cdrom_dma_c0_end_logs = 0;
    const CdRom &cd = sys_->cdrom();
    const bool whole_sector = cd.read_whole_sector();
    const bool e0_sector = cd.mode() == 0xE0u;
    const bool c0_sector = cd.mode() == 0xC0u;
    if (cdrom_dma_end_logs < 256u ||
        (whole_sector && cdrom_dma_whole_end_logs < 512u) ||
        (e0_sector && cdrom_dma_e0_end_logs < 512u) ||
        (c0_sector && cdrom_dma_c0_end_logs < 512u)) {
      ++cdrom_dma_end_logs;
      if (whole_sector) {
        ++cdrom_dma_whole_end_logs;
      }
      if (e0_sector) {
        ++cdrom_dma_e0_end_logs;
      }
      if (c0_sector) {
        ++cdrom_dma_c0_end_logs;
      }
      LOG_INFO(
          "DMA: CDROM end base=0x%08X words=%u remaining_block=%u avail=%u "
          "idx=%d size=%zu ready=%u req=%u first=0x%08X last=0x%08X cyc=%llu",
          ch.base_addr & 0x00FFFFFFu, transfer_words, ch.block_words_remaining,
          sys_->cdrom_dma_words_available(), cd.dma_data_index(),
          cd.dma_buffer_size(), cd.sector_data_ready() ? 1u : 0u,
          cd.sector_data_request() ? 1u : 0u, cdrom_dma_first_word,
          cdrom_dma_last_word,
          static_cast<unsigned long long>(sys_->cpu().cycle_count()));
      if (e0_sector && cdrom_dma_sample_count > 0u) {
        LOG_INFO(
            "DMA: CDROM E0 sample words=%u w0=0x%08X w1=0x%08X w2=0x%08X "
            "w3=0x%08X w4=0x%08X w5=0x%08X w6=0x%08X w7=0x%08X",
            cdrom_dma_sample_count, cdrom_dma_sample_words[0],
            cdrom_dma_sample_words[1], cdrom_dma_sample_words[2],
            cdrom_dma_sample_words[3], cdrom_dma_sample_words[4],
            cdrom_dma_sample_words[5], cdrom_dma_sample_words[6],
            cdrom_dma_sample_words[7]);
      }
    }
    if (c0_sector && cdrom_dma_sample_count > 0u) {
      static u32 str_dma_log_count = 0;
      static int str_header_lba = -1;
      static u32 str_header_frame = 0;
      static u32 str_header_chunk = 0;
      static u32 str_header_chunks = 0;
      static bool str_header_pending = false;

      if (cdrom_dma_sample_words[0] == 0x80010160u &&
          cdrom_dma_sample_count >= 3u) {
        str_header_lba = cdrom_dma_active_lba;
        str_header_chunk = cdrom_dma_sample_words[1] & 0xFFFFu;
        str_header_chunks = cdrom_dma_sample_words[1] >> 16;
        str_header_frame = cdrom_dma_sample_words[2];
        str_header_pending = true;
        if (str_dma_log_count < 4096u) {
          ++str_dma_log_count;
          LOG_INFO(
              "DMA: STRDMA head lba=%d words=%u frame=%u chunk=%u/%u "
              "madr=0x%08X w3=0x%08X cyc=%llu",
              str_header_lba, transfer_words, str_header_frame,
              str_header_chunk, str_header_chunks, dbg.base_addr,
              cdrom_dma_sample_words[3],
              static_cast<unsigned long long>(sys_->cpu().cycle_count()));
        }
      } else if (str_header_pending && transfer_words >= 128u) {
        if (str_dma_log_count < 4096u) {
          ++str_dma_log_count;
          LOG_INFO(
              "DMA: STRDMA body lba=%d words=%u frame=%u chunk=%u/%u "
              "madr=0x%08X first=0x%08X last=0x%08X cyc=%llu",
              str_header_lba, transfer_words, str_header_frame,
              str_header_chunk, str_header_chunks, dbg.base_addr,
              cdrom_dma_first_word, cdrom_dma_last_word,
              static_cast<unsigned long long>(sys_->cpu().cycle_count()));
        }
        str_header_pending = false;
      }
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
  u32 transferred_words = 0;

  while (safety < 0x100000) {
    sys_->debug_begin_dma_bus_access(static_cast<u8>(channel));
    u32 header = sys_->read32(addr);
    sys_->debug_end_dma_bus_access();
    u32 word_count = header >> 24;
    const u32 packet_addr = addr;
    const u32 next_addr = header & 0x00FFFFFFu;
    transferred_words += 1;

    // Send words to GP0
    for (u32 i = 0; i < word_count; i++) {
      addr = (addr + 4) & 0x001FFFFC;
      sys_->debug_begin_dma_bus_access(static_cast<u8>(channel));
      u32 command = sys_->read32(addr);
      sys_->debug_end_dma_bus_access();
      sys_->gpu_gp0_dma(command, addr);
      transferred_words += 1;
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

  if (sys_ != nullptr) {
    sys_->add_cpu_cycle_penalty(dma_ram_tick_cost(transferred_words));
  }
}

void DmaController::transfer_complete(int channel) {
  auto &ch = channels_[channel];
  const u64 cycle = sys_ ? sys_->cpu().cycle_count() : 0ull;
  if (g_log_fmv_diagnostics &&
      (channel == 1 || cycle >= 880000000ull)) {
    LOG_WARN(
        "DMA: ch%d complete pre madr=0x%08X bcr=0x%08X chcr=0x%08X dicr=0x%08X cyc=%llu",
        channel, ch.base_addr, ch.block_ctrl, ch.channel_ctrl, dicr_,
        static_cast<unsigned long long>(cycle));
  }

  // Clear enable + trigger bits
  ch.channel_ctrl &= ~(1u << 24); // Disable
  ch.channel_ctrl &= ~(1u << 28); // Clear trigger
  ch.block_words_remaining = 0;
  // Keep MDEC-out reorder state across DMA1 transfer boundaries.
  // Some clients issue a sequence of DMA1 transfers while consuming one
  // continuous MDEC output stream; resetting here can desynchronize macroblock
  // base tracking when a transfer ends mid-macroblock.

  const bool master_enable = ((dicr_ >> 23) & 1u) != 0;
  const bool channel_irq_enable = ((dicr_ >> (16 + channel)) & 1u) != 0;
  if (master_enable && channel_irq_enable) {
    dicr_ |= (1u << (24 + channel));
    recompute_dicr_master(true);
  }
  if (g_log_fmv_diagnostics &&
      (channel == 1 || cycle >= 880000000ull)) {
    LOG_WARN(
        "DMA: ch%d complete post madr=0x%08X bcr=0x%08X chcr=0x%08X dicr=0x%08X cyc=%llu",
        channel, ch.base_addr, ch.block_ctrl, ch.channel_ctrl, dicr_,
        static_cast<unsigned long long>(cycle));
  }
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
