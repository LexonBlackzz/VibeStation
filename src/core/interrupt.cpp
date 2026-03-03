#include "interrupt.h"
#include <cstring>

namespace {
u64 g_irq_trace_counter = 0;
}

void InterruptController::reset() {
  i_stat_ = 0;
  i_mask_ = 0;
  std::memset(request_count_, 0, sizeof(request_count_));
}

u32 InterruptController::read(u32 offset) const {
  switch (offset) {
  case 0:
    return i_stat_;
  case 4:
    return i_mask_;
  default:
    LOG_WARN("IRQ: Unhandled read at offset 0x%X", offset);
    return 0;
  }
}

void InterruptController::write(u32 offset, u32 value) {
  switch (offset) {
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
    LOG_WARN("IRQ: Unhandled write at offset 0x%X = 0x%08X", offset, value);
    break;
  }
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
