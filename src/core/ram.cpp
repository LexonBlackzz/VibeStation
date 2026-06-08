#include "ram.h"

namespace {
u64 g_ram_trace_counter = 0;

inline void ram_trace_event(const char *op, u32 offset, u32 value) {
  if (!g_trace_ram) {
    return;
  }
  if (!trace_should_log(g_ram_trace_counter, g_trace_burst_ram,
                        g_trace_stride_ram)) {
    return;
  }
  LOG_DEBUG("RAM: %s off=0x%08X val=0x%08X count=%llu", op, offset, value,
            static_cast<unsigned long long>(g_ram_trace_counter));
}
} // namespace

u8 Ram::read8(u32 offset) const {
  const u8 value = data_[offset & (psx::RAM_MAX_SIZE - 1)];
  ram_trace_event("R8", offset, value);
  return value;
}

u16 Ram::read16(u32 offset) const {
  u32 off = offset & (psx::RAM_MAX_SIZE - 1);
  u16 val;
  std::memcpy(&val, &data_[off], sizeof(u16));
  ram_trace_event("R16", offset, val);
  return val;
}

u32 Ram::read32(u32 offset) const {
  u32 off = offset & (psx::RAM_MAX_SIZE - 1);
  u32 val;
  std::memcpy(&val, &data_[off], sizeof(u32));
  ram_trace_event("R32", offset, val);
  return val;
}

void Ram::write8(u32 offset, u8 val) {
  data_[offset & (psx::RAM_MAX_SIZE - 1)] = val;
  ram_trace_event("W8", offset, val);
}

void Ram::write16(u32 offset, u16 val) {
  u32 off = offset & (psx::RAM_MAX_SIZE - 1);
  std::memcpy(&data_[off], &val, sizeof(u16));
  ram_trace_event("W16", offset, val);
}

void Ram::write32(u32 offset, u32 val) {
  u32 off = offset & (psx::RAM_MAX_SIZE - 1);
  std::memcpy(&data_[off], &val, sizeof(u32));
  ram_trace_event("W32", offset, val);
}

u8 Ram::scratch_read8(u32 offset) const {
  const u8 value = scratchpad_[offset & (psx::SCRATCHPAD_SIZE - 1)];
  ram_trace_event("SR8", offset, value);
  return value;
}

u16 Ram::scratch_read16(u32 offset) const {
  u32 off = offset & (psx::SCRATCHPAD_SIZE - 1);
  u16 val;
  std::memcpy(&val, &scratchpad_[off], sizeof(u16));
  ram_trace_event("SR16", offset, val);
  return val;
}

u32 Ram::scratch_read32(u32 offset) const {
  u32 off = offset & (psx::SCRATCHPAD_SIZE - 1);
  u32 val;
  std::memcpy(&val, &scratchpad_[off], sizeof(u32));
  ram_trace_event("SR32", offset, val);
  return val;
}

void Ram::scratch_write8(u32 offset, u8 val) {
  scratchpad_[offset & (psx::SCRATCHPAD_SIZE - 1)] = val;
  ram_trace_event("SW8", offset, val);
}

void Ram::scratch_write16(u32 offset, u16 val) {
  u32 off = offset & (psx::SCRATCHPAD_SIZE - 1);
  std::memcpy(&scratchpad_[off], &val, sizeof(u16));
  ram_trace_event("SW16", offset, val);
}

void Ram::scratch_write32(u32 offset, u32 val) {
  u32 off = offset & (psx::SCRATCHPAD_SIZE - 1);
  std::memcpy(&scratchpad_[off], &val, sizeof(u32));
  ram_trace_event("SW32", offset, val);
}
