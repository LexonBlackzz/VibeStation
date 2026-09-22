#include "cpu_jit_v4.h"

#include <memory>

struct CpuJitV4Backend::Impl {};

CpuJitV4Backend::CpuJitV4Backend(Cpu &cpu)
    : cpu_(cpu), impl_(std::make_unique<Impl>()) {
  stats_.available = true;
}

CpuJitV4Backend::~CpuJitV4Backend() = default;

CpuRunSliceResult CpuJitV4Backend::run_slice(u32 max_cycles,
                                             u32 max_instructions) {
  CpuRunSliceResult result{};
  stats_.active = true;

  // Phase 0 safety path. Phase 1 replaces normal execution with the V4
  // dispatcher/code-cache loop; Cpu::step() remains only as bring-up fallback.
  while (result.cycles < max_cycles &&
         result.instructions < max_instructions) {
    const u32 consumed = cpu_.step();
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.fallback_instructions;
    ++stats_.interpreter_fallback_steps;
  }
  return result;
}

void CpuJitV4Backend::invalidate_range(u32, u32) {}

void CpuJitV4Backend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV4;
}

void CpuJitV4Backend::flush() { ++stats_.flushes; }

CpuBackendStats CpuJitV4Backend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV4;
  out.native_available = false;
  return out;
}
