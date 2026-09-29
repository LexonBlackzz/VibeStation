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
  // Host address ranges of generated code, for profilers: the resident
  // dispatcher occupies [dispatcher_begin, translations_begin) and translated
  // blocks/fragments [translations_begin, end). False when no code exists.
  bool debug_code_ranges(uintptr_t &dispatcher_begin,
                         uintptr_t &translations_begin, uintptr_t &end) const;

private:
  struct Impl;

  Cpu &cpu_;
  std::unique_ptr<Impl> impl_;
  CpuBackendStats stats_{};
  u32 current_frame_ = 0;
};
