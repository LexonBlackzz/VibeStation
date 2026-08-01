#include "system.h"
#include <cstring>

// Serialization helpers
namespace {

struct BufWriter {
  std::vector<u8> &buf;
  void write(const void *data, size_t size) {
    const u8 *src = static_cast<const u8 *>(data);
    buf.insert(buf.end(), src, src + size);
  }
  template <typename T> void val(const T &v) { write(&v, sizeof(T)); }
  void u8v(u8 v) { buf.push_back(v); }
  void u32v(u32 v) { val(v); }
  void boolv(bool b) { buf.push_back(b ? 1u : 0u); }
  void vec_u8(const std::vector<u8> &v) {
    u32v(static_cast<u32>(v.size()));
    if (!v.empty()) {
      write(v.data(), v.size());
    }
  }
  void vec_u16(const std::vector<u16> &v) {
    u32v(static_cast<u32>(v.size()));
    if (!v.empty()) {
      write(v.data(), v.size() * 2);
    }
  }
  void vec_u32(const std::vector<u32> &v) {
    u32v(static_cast<u32>(v.size()));
    if (!v.empty()) {
      write(v.data(), v.size() * 4);
    }
  }
  void deque_u8(const std::deque<u8> &d) {
    u32v(static_cast<u32>(d.size()));
    for (auto b : d) {
      buf.push_back(b);
    }
  }
  void deque_u16(const std::deque<u16> &d) {
    u32v(static_cast<u32>(d.size()));
    for (auto v : d) {
      val(v);
    }
  }
  void deque_u32(const std::deque<u32> &d) {
    u32v(static_cast<u32>(d.size()));
    for (auto v : d) {
      val(v);
    }
  }
};

struct BufReader {
  const u8 *&pos;
  size_t &remaining;
  void read(void *dest, size_t size) {
    if (remaining < size) {
      std::memset(dest, 0, size);
      return;
    }
    std::memcpy(dest, pos, size);
    pos += size;
    remaining -= size;
  }
  template <typename T> void val(T &v) { read(&v, sizeof(T)); }
  u8 u8v() {
    u8 v = 0;
    val(v);
    return v;
  }
  u32 u32v() {
    u32 v = 0;
    val(v);
    return v;
  }
  bool boolv() { return u8v() != 0; }
  void vec_u8(std::vector<u8> &v) {
    u32 count = u32v();
    v.resize(count);
    if (count > 0) {
      read(v.data(), count);
    }
  }
  void vec_u16(std::vector<u16> &v) {
    u32 count = u32v();
    v.resize(count);
    if (count > 0) {
      read(v.data(), count * 2);
    }
  }
  void vec_u32(std::vector<u32> &v) {
    u32 count = u32v();
    v.resize(count);
    if (count > 0) {
      read(v.data(), count * 4);
    }
  }
  void deque_u8(std::deque<u8> &d) {
    u32 count = u32v();
    d.resize(count);
    for (u32 i = 0; i < count; ++i) {
      d[i] = u8v();
    }
  }
  void deque_u16(std::deque<u16> &d) {
    u32 count = u32v();
    d.resize(count);
    for (u32 i = 0; i < count; ++i) {
      val(d[i]);
    }
  }
  void deque_u32(std::deque<u32> &d) {
    u32 count = u32v();
    d.resize(count);
    for (u32 i = 0; i < count; ++i) {
      val(d[i]);
    }
  }
};

} // anonymous namespace

bool System::save_state(SystemSnapshot &out) {
  BufWriter w{out.data};
  out.data.reserve(4 * 1024 * 1024); // Pre-alloc ~4MB

  // Frame ID
  w.val(boot_diag_.frame_counter);

  // CPU state (includes GTE)
  cpu_.save_state(out.data);

  // RAM (2MB main + 1KB scratchpad)
  w.write(ram_.data(), psx::RAM_MAX_SIZE);
  w.write(ram_.scratch_data(), psx::SCRATCHPAD_SIZE);

  // GPU state (includes VRAM)
  gpu_.save_state(out.data);

  // Interrupt controller
  irq_.save_state(out.data);

  // Timers
  timers_.save_state(out.data);

  // DMA controller
  dma_.save_state(out.data);

  // SIO
  sio_.save_state(out.data);

  // CD-ROM
  cdrom_.save_state(out.data);

  // SPU
  spu_.save_state(out.data);

  // MDEC
  mdec_.save_state(out.data);

  // System-level registers
  for (int i = 0; i < 9; ++i) {
    w.val(mem_ctrl_[i]);
  }
  w.val(ram_size_);
  w.val(cache_ctrl_);
  w.val(mdec_command_shadow_);
  w.val(mdec_command_shadow_mask_);
  w.val(mdec_control_shadow_);
  w.val(mdec_control_shadow_mask_);
  w.val(gpu_gp0_shadow_);
  w.val(gpu_gp0_shadow_mask_);
  w.val(gpu_gp1_shadow_);
  w.val(gpu_gp1_shadow_mask_);
  w.val(post_reg_);
  w.val(frame_cycles_);
  w.val(frame_cycle_remainder_);

  return true;
}

bool System::restore_state(const SystemSnapshot &snap) {
  if (snap.data.empty()) {
    return false;
  }

  const u8 *pos = snap.data.data();
  size_t remaining = snap.data.size();
  BufReader r{pos, remaining};

  // Frame ID
  u32 frame_counter = 0;
  r.val(frame_counter);
  boot_diag_.frame_counter = frame_counter;

  // CPU state
  cpu_.restore_state(pos, remaining);

  // RAM
  if (remaining < psx::RAM_MAX_SIZE + psx::SCRATCHPAD_SIZE) {
    return false;
  }
  std::memcpy(ram_.data(), pos, psx::RAM_MAX_SIZE);
  pos += psx::RAM_MAX_SIZE;
  remaining -= psx::RAM_MAX_SIZE;
  std::memcpy(ram_.scratch_data(), pos, psx::SCRATCHPAD_SIZE);
  pos += psx::SCRATCHPAD_SIZE;
  remaining -= psx::SCRATCHPAD_SIZE;

  // GPU
  gpu_.restore_state(pos, remaining);

  // IRQ
  irq_.restore_state(pos, remaining);

  // Timers
  timers_.restore_state(pos, remaining);

  // DMA
  dma_.restore_state(pos, remaining);

  // SIO
  sio_.restore_state(pos, remaining);

  // CD-ROM
  cdrom_.restore_state(pos, remaining);

  // SPU
  spu_.restore_state(pos, remaining);

  // MDEC
  mdec_.restore_state(pos, remaining);

  // System-level registers
  for (int i = 0; i < 9; ++i) {
    r.val(mem_ctrl_[i]);
  }
  r.val(ram_size_);
  r.val(cache_ctrl_);
  r.val(mdec_command_shadow_);
  r.val(mdec_command_shadow_mask_);
  r.val(mdec_control_shadow_);
  r.val(mdec_control_shadow_mask_);
  r.val(gpu_gp0_shadow_);
  r.val(gpu_gp0_shadow_mask_);
  r.val(gpu_gp1_shadow_);
  r.val(gpu_gp1_shadow_mask_);
  r.val(post_reg_);
  r.val(frame_cycles_);
  r.val(frame_cycle_remainder_);

  sio_synced_cpu_cycle_ = cpu_.cycle_count();
  cpu_timing_boundary_requested_ = false;

  return true;
}
