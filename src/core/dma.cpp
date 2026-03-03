#include "dma.h"
#include "system.h"
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
      break;
    case 0x8:
      ch.channel_ctrl = value;
      if (g_trace_dma) {
        static u64 chcr_log_count = 0;
        if (trace_should_log(chcr_log_count, g_trace_burst_dma,
                             g_trace_stride_dma)) {
          LOG_INFO("DMA: CH%d CHCR=0x%08X (enabled=%d trigger=%d sync=%d "
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
        LOG_INFO("DMA: DPCR=0x%08X", dpcr_);
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
      LOG_INFO("DMA: Execute ch=%d madr=0x%08X bcr=0x%08X chcr=0x%08X sync=%d "
               "from_ram=%d",
               channel, ch.base_addr, ch.block_ctrl, ch.channel_ctrl,
               static_cast<int>(ch.sync_mode()), ch.from_ram() ? 1 : 0);
    }
  }

  switch (ch.sync_mode()) {
  case DmaChannel::SyncMode::Immediate:
  case DmaChannel::SyncMode::Block:
    dma_block(channel);
    break;
  case DmaChannel::SyncMode::LinkedList:
    dma_linked_list(channel);
    break;
  }

  transfer_complete(channel);
}

void DmaController::dma_block(int channel) {
  auto &ch = channels_[channel];

  u32 addr = ch.base_addr;
  u32 word_count;

  if (ch.sync_mode() == DmaChannel::SyncMode::Immediate) {
    word_count = ch.block_size();
    if (word_count == 0)
      word_count = 0x10000;
  } else {
    word_count = ch.block_size() * ch.block_count();
  }

  s32 step;
  if ((ch.channel_ctrl >> 1) & 1) {
    step = -4;
  } else {
    step = 4;
  }

  bool from_ram = ch.from_ram();

  if (channel == 6) {
    // OTC (Ordering Table Clear): always writes to RAM, backwards.
    u32 current = addr;
    for (u32 i = 0; i < word_count; i++) {
      u32 val;
      if (i == word_count - 1) {
        val = 0x00FFFFFF; // End-of-list marker
      } else {
        val = (current - 4) & 0x001FFFFC;
      }
      sys_->write32(current, val);
      current -= 4;
    }
    return;
  }

  for (u32 i = 0; i < word_count; i++) {
    u32 current_addr = addr & 0x001FFFFC;

    if (from_ram) {
      u32 data = sys_->read32(current_addr);
      // Send to device
      switch (channel) {
      case 0: // MDEC in
        sys_->mdec_dma_write(data);
        break;
      case 2: // GPU
        sys_->gpu_gp0(data);
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
      // Read from device
      switch (channel) {
      case 1: // MDEC out
        data = sys_->mdec_dma_read();
        break;
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
      sys_->write32(current_addr, data);
    }

    addr = static_cast<u32>(static_cast<s32>(addr) + step);
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
    u32 header = sys_->read32(addr);
    u32 word_count = header >> 24;

    // Send words to GP0
    for (u32 i = 0; i < word_count; i++) {
      addr = (addr + 4) & 0x001FFFFC;
      u32 command = sys_->read32(addr);
      sys_->gpu_gp0(command);
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
