#pragma once

#include "cpu.h"

// R3000A dynamic recompiler backend.
//
// The interpreter remains the correctness oracle and fallback for unsupported host platforms; native dispatch, code storage, and execution state live here.
class CpuRecompilerBackend {
public:
  explicit CpuRecompilerBackend(Cpu &cpu);
  ~CpuRecompilerBackend();

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
