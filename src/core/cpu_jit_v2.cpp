#include "cpu_jit_v2.h"

CpuJitV2Backend::CpuJitV2Backend(Cpu &cpu) : cpu_(cpu) {
  stats_.available = true;
}

CpuJitV2Backend::~CpuJitV2Backend() = default;

CpuRunSliceResult CpuJitV2Backend::run_slice(u32 max_cycles,
                                             u32 max_instructions) {
  CpuRunSliceResult result{};
  if (max_cycles == 0u || max_instructions == 0u) {
    return result;
  }

  stats_.available = true;
  stats_.active = true;
  stats_.native_available = false;

  // Foundation phase: preserve the interpreter as the semantic oracle while
  // the V2 dispatcher/IR/native emitter are introduced independently. Native
  // execution will replace this loop incrementally, with the compare runner
  // checking architectural state after every test segment.
  while (result.cycles < max_cycles &&
         result.instructions < max_instructions) {
    const u32 consumed = cpu_.step();
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.interpreter_fallback_steps;
    ++stats_.fallback_instructions;
    stats_.executed_cycles += consumed;

    if (cpu_.sys_ != nullptr && cpu_.sys_->cpu_timing_boundary_requested()) {
      break;
    }
  }

  return result;
}

void CpuJitV2Backend::invalidate_range(u32, u32) {
  // No compiled V2 code exists in the foundation phase. This hook is kept
  // from day one so page generations/invalidation can be added without
  // changing Cpu's public contract later.
}

void CpuJitV2Backend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV2;
}

void CpuJitV2Backend::flush() {
  stats_ = {};
  stats_.available = true;
  current_frame_ = 0;
}

CpuBackendStats CpuJitV2Backend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV2;
  out.native_available = false;
  return out;
}
