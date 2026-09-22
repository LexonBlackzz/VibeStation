#pragma once

#include "cpu.h"

// Fourth-generation R3000A JIT backend.
//
// V4 is intentionally separate from the legacy optimized backend and V3.
// The interpreter is an early bring-up fallback/oracle only; native dispatch,
// code storage, and execution state live in this backend.
class CpuJitV4Backend {
public:
  explicit CpuJitV4Backend(Cpu &cpu);
  ~CpuJitV4Backend();

  CpuRunSliceResult run_slice(u32 max_cycles, u32 max_instructions);
  void invalidate_range(u32 phys_or_normalized_addr, u32 size_bytes);
  void begin_frame(u32 frame_index);
  void flush();
  CpuBackendStats stats() const;

private:
  struct Impl;

  Cpu &cpu_;
  std::unique_ptr<Impl> impl_;
  CpuBackendStats stats_{};
  u32 current_frame_ = 0;
};
