#include "interrupt.h"
#include <cstring>

namespace {
u64 g_irq_trace_counter = 0;
}

void InterruptController::reset() {
  i_stat_ = 0;
  i_mask_ = 0;
  line_state_ = 0;
  std::memset(request_count_, 0, sizeof(request_count_));
}

u32 InterruptController::read_reg_word(u32 base_offset) const {
  switch (base_offset) {
  case 0:
    return i_stat_;
  case 4:
    return i_mask_;
  default:
    LOG_WARN("IRQ: Unhandled read at offset 0x%X", base_offset);
    return 0;
  }
}

u32 InterruptController::read(u32 offset) const {
  const u32 base_offset = offset & ~0x3u;
  const u32 shift = (offset & 0x3u) * 8u;
  const u32 word = read_reg_word(base_offset);
  switch (offset & 0x3u) {
  case 0:
    return word;
  case 2:
    return (word >> 16) & 0xFFFFu;
  case 1:
  case 3:
    return (word >> shift) & 0xFFu;
  }
  return word;
}

void InterruptController::write_reg_word(u32 base_offset, u32 value) {
  switch (base_offset) {
  case 0:
    i_stat_ &= value;
    if (g_trace_irq && trace_should_log(g_irq_trace_counter, g_trace_burst_irq,
                                        g_trace_stride_irq)) {
      LOG_DEBUG("IRQ: ACK I_STAT with 0x%08X -> 0x%08X", value, i_stat_);
    }
    break;
  case 4:
    i_mask_ = value & 0x7FF;
    if (g_trace_irq && trace_should_log(g_irq_trace_counter, g_trace_burst_irq,
                                        g_trace_stride_irq)) {
      LOG_DEBUG("IRQ: I_MASK set to 0x%08X", i_mask_);
    }
    break;
  default:
    LOG_WARN("IRQ: Unhandled write at offset 0x%X = 0x%08X", base_offset, value);
    break;
  }
}

void InterruptController::write(u32 offset, u32 value) {
  const u32 base_offset = offset & ~0x3u;
  if ((offset & 0x3u) == 0u) {
    write_reg_word(base_offset, value);
    return;
  }

  const u32 shift = (offset & 0x3u) * 8u;
  const u32 width_bits = ((offset & 0x1u) == 0u) ? 16u : 8u;
  const u32 mask = ((1u << width_bits) - 1u) << shift;
  const u32 merged =
      (read_reg_word(base_offset) & ~mask) | ((value << shift) & mask);
  write_reg_word(base_offset, merged);
}

void InterruptController::request(Interrupt irq) {
  const u32 index = static_cast<u32>(irq);
  i_stat_ |= (1u << index);
  if (index < 11) {
    ++request_count_[index];
  }
  if (g_trace_irq && trace_should_log(g_irq_trace_counter, g_trace_burst_irq,
                                      g_trace_stride_irq)) {
    LOG_DEBUG("IRQ: request src=%u I_STAT=0x%08X I_MASK=0x%08X",
              static_cast<unsigned>(irq), i_stat_, i_mask_);
  }
}

void InterruptController::set_line_state(Interrupt irq, bool active) {
  const u32 index = static_cast<u32>(irq);
  if (index >= 11) {
    return;
  }

  const u32 bit = 1u << index;
  const u32 previous = line_state_;
  if (active) {
    line_state_ |= bit;
  } else {
    line_state_ &= ~bit;
  }

  if (line_state_ == previous) {
    return;
  }

  if (active) {
    request(irq);
  }
}
