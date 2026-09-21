#pragma once

#include "cpu.h"

// Third-generation R3000A execution backend.
//
// This backend is intentionally independent from CpuOptimizedBackend. During
// bring-up it may fall back to Cpu::step(), but new dispatch, IR, register
// caching, fastmem and native code generation live here rather than extending
// the legacy JIT's preflight/reduced-helper architecture.
class CpuJitV3Backend {
public:
  explicit CpuJitV3Backend(Cpu &cpu);
  ~CpuJitV3Backend();

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
