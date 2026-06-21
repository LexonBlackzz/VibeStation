#include "cpu_recompiler.h"
#include "system.h"
#include <algorithm>
#include <array>

namespace {
u64 delta_u64(u64 current, u64 previous) {
  return current >= previous ? current - previous : 0;
}

size_t delta_size(size_t current, size_t previous) {
  return current >= previous ? current - previous : 0;
}

CpuBackendStats delta_stats(const CpuBackendStats &current,
                            const CpuBackendStats &previous) {
  CpuBackendStats out{};
  out.decoded_blocks =
      delta_u64(current.decoded_blocks, previous.decoded_blocks);
  out.native_blocks = current.native_blocks;
  out.native_compile_attempts =
      delta_u64(current.native_compile_attempts,
                previous.native_compile_attempts);
  out.native_compile_successes =
      delta_u64(current.native_compile_successes,
                previous.native_compile_successes);
  out.native_blocks_compiled =
      delta_u64(current.native_blocks_compiled,
                previous.native_blocks_compiled);
  out.native_compile_failures =
      delta_u64(current.native_compile_failures,
                previous.native_compile_failures);
  out.native_rejected_unsafe_blocks =
      delta_u64(current.native_rejected_unsafe_blocks,
                previous.native_rejected_unsafe_blocks);
  out.native_to_decoded_fallbacks =
      delta_u64(current.native_to_decoded_fallbacks,
                previous.native_to_decoded_fallbacks);
  out.native_hot_threshold_skips =
      delta_u64(current.native_hot_threshold_skips,
                previous.native_hot_threshold_skips);
  out.native_short_block_skips =
      delta_u64(current.native_short_block_skips,
                previous.native_short_block_skips);
  out.native_reject_branch =
      delta_u64(current.native_reject_branch, previous.native_reject_branch);
  out.native_reject_memory =
      delta_u64(current.native_reject_memory, previous.native_reject_memory);
  out.native_reject_cop0 =
      delta_u64(current.native_reject_cop0, previous.native_reject_cop0);
  out.native_reject_cop2 =
      delta_u64(current.native_reject_cop2, previous.native_reject_cop2);
  out.native_reject_exception_unknown =
      delta_u64(current.native_reject_exception_unknown,
                previous.native_reject_exception_unknown);
  out.native_reject_unsafe_state =
      delta_u64(current.native_reject_unsafe_state,
                previous.native_reject_unsafe_state);
  out.native_reject_budget =
      delta_u64(current.native_reject_budget, previous.native_reject_budget);
  out.native_reject_icache =
      delta_u64(current.native_reject_icache, previous.native_reject_icache);
  out.cache_hits = delta_u64(current.cache_hits, previous.cache_hits);
  out.cache_misses = delta_u64(current.cache_misses, previous.cache_misses);
  out.invalidations =
      delta_u64(current.invalidations, previous.invalidations);
  out.invalidation_queries =
      delta_u64(current.invalidation_queries, previous.invalidation_queries);
  out.invalidation_fast_no_code_page_exits =
      delta_u64(current.invalidation_fast_no_code_page_exits,
                previous.invalidation_fast_no_code_page_exits);
  out.invalidation_blocks_examined =
      delta_u64(current.invalidation_blocks_examined,
                previous.invalidation_blocks_examined);
  out.invalidation_blocks_invalidated =
      delta_u64(current.invalidation_blocks_invalidated,
                previous.invalidation_blocks_invalidated);
  out.flushes = delta_u64(current.flushes, previous.flushes);
  out.decoded_block_entries =
      delta_u64(current.decoded_block_entries,
                previous.decoded_block_entries);
  out.native_block_entries =
      delta_u64(current.native_block_entries, previous.native_block_entries);
  out.optimized_instructions =
      delta_u64(current.optimized_instructions,
                previous.optimized_instructions);
  out.decoded_instructions =
      delta_u64(current.decoded_instructions, previous.decoded_instructions);
  out.native_instructions =
      delta_u64(current.native_instructions, previous.native_instructions);
  out.native_cycles =
      delta_u64(current.native_cycles, previous.native_cycles);
  out.fallback_instructions =
      delta_u64(current.fallback_instructions,
                previous.fallback_instructions);
  out.interpreter_fallback_steps =
      delta_u64(current.interpreter_fallback_steps,
                previous.interpreter_fallback_steps);
  out.forced_interpreter_slices =
      delta_u64(current.forced_interpreter_slices,
                previous.forced_interpreter_slices);
  out.forced_interpreter_instructions =
      delta_u64(current.forced_interpreter_instructions,
                previous.forced_interpreter_instructions);
  out.forced_interpreter_trace =
      delta_u64(current.forced_interpreter_trace,
                previous.forced_interpreter_trace);
  out.forced_interpreter_deep_diagnostics =
      delta_u64(current.forced_interpreter_deep_diagnostics,
                previous.forced_interpreter_deep_diagnostics);
  out.forced_interpreter_fmv =
      delta_u64(current.forced_interpreter_fmv,
                previous.forced_interpreter_fmv);
  out.forced_interpreter_backend_compare =
      delta_u64(current.forced_interpreter_backend_compare,
                previous.forced_interpreter_backend_compare);
  out.forced_interpreter_other =
      delta_u64(current.forced_interpreter_other,
                previous.forced_interpreter_other);
  out.forced_interpreter_last_reason =
      current.forced_interpreter_last_reason;
  out.memory_helper_calls =
      delta_u64(current.memory_helper_calls, previous.memory_helper_calls);
  out.mmio_accesses =
      delta_u64(current.mmio_accesses, previous.mmio_accesses);
  out.exceptions = delta_u64(current.exceptions, previous.exceptions);
  out.interrupt_exits =
      delta_u64(current.interrupt_exits, previous.interrupt_exits);
  out.budget_exits =
      delta_u64(current.budget_exits, previous.budget_exits);
  out.fallback_exits =
      delta_u64(current.fallback_exits, previous.fallback_exits);
  out.pc_mismatch_exits =
      delta_u64(current.pc_mismatch_exits, previous.pc_mismatch_exits);
  out.invalidated_exits =
      delta_u64(current.invalidated_exits, previous.invalidated_exits);
  out.executed_cycles =
      delta_u64(current.executed_cycles, previous.executed_cycles);
  out.code_bytes = current.code_bytes;
  out.native_code_bytes =
      delta_size(current.native_code_bytes, previous.native_code_bytes);
  return out;
}
} // namespace

DecodedBlock::~DecodedBlock() {
  if (destroy_native_context != nullptr && native_context != nullptr) {
    destroy_native_context(native_context);
  }
  native_context = nullptr;
  native_fn = nullptr;
  destroy_native_context = nullptr;
  native_code_bytes = 0;
}

CpuOptimizedBackend::CpuOptimizedBackend(Cpu &cpu) : cpu_(cpu) {
  stats_.available = true;
}

CpuOptimizedBackend::~CpuOptimizedBackend() = default;

CpuRunSliceResult CpuOptimizedBackend::run_slice(
    u32 max_cycles, u32 max_instructions, CpuExecutionMode requested_mode) {
  CpuRunSliceResult total{};
  if (max_cycles == 0 || max_instructions == 0) {
    return total;
  }

  stats_.active = true;
  stats_.available = true;
  const CpuForcedInterpreterReason forced_reason =
      diagnostics_force_interpreter_reason();
  if (forced_reason != CpuForcedInterpreterReason::None) {
    stats_.forced_interpreter_last_reason = forced_reason;
    ++stats_.forced_interpreter_slices;
    warn_forced_interpreter_once(forced_reason);
    u32 forced_instructions = 0;
    while (total.cycles < max_cycles && total.instructions < max_instructions) {
      const u32 consumed = cpu_.step();
      total.cycles += consumed;
      ++total.instructions;
      ++forced_instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
    }
    record_forced_interpreter(forced_reason, forced_instructions);
    return total;
  }
  stats_.forced_interpreter_last_reason = CpuForcedInterpreterReason::None;

  if (requested_mode == CpuExecutionMode::X64Jit && !x64_jit_available()) {
    warn_x64_unavailable_once();
  }

  while (total.cycles < max_cycles && total.instructions < max_instructions) {
    DecodedBlock *block = lookup_or_decode(cpu_.pc_);
    if (block == nullptr ||
        block->interpreter_only_until_frame > current_frame_) {
      const u32 consumed = cpu_.step();
      total.cycles += consumed;
      ++total.instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
      continue;
    }

    ++block->entry_count;
    const u32 cycle_budget = max_cycles - total.cycles;
    const u32 instruction_budget = max_instructions - total.instructions;
    CpuBlockRunResult result{};
    const bool try_native =
        requested_mode == CpuExecutionMode::X64Jit && x64_jit_available();
    if (try_native) {
      if (!ensure_x64_safety_checked(*block)) {
        ++stats_.native_to_decoded_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (block->instruction_count > instruction_budget) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_reject_budget;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else if (!should_attempt_x64_compile(*block)) {
        ++stats_.native_to_decoded_fallbacks;
        result = execute_block(*block, cycle_budget, instruction_budget);
      } else {
        if (!block->native_compile_attempted) {
          (void)compile_x64_block(*block);
        }
        if (block->native_fn != nullptr) {
          result =
              execute_native_block(*block, cycle_budget, instruction_budget);
        } else {
          ++stats_.native_to_decoded_fallbacks;
          result = execute_block(*block, cycle_budget, instruction_budget);
        }
      }
    } else {
      result = execute_block(*block, cycle_budget, instruction_budget);
    }
    record_exit(result.exit_reason);

    if (result.instructions == 0) {
      const u32 consumed = cpu_.step();
      total.cycles += consumed;
      ++total.instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
      continue;
    }

    total.cycles += result.cycles;
    total.instructions += result.instructions;
    stats_.executed_cycles += result.cycles;
  }

  return total;
}

void CpuOptimizedBackend::invalidate_range(u32 phys_or_normalized_addr,
                                           u32 size_bytes) {
  ++stats_.invalidation_queries;
  if (size_bytes == 0 || blocks_.empty() || blocks_by_page_.empty()) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  const u32 start = normalized_code_addr(phys_or_normalized_addr);
  const u64 end64 =
      std::min<u64>(static_cast<u64>(start) + static_cast<u64>(size_bytes),
                    0x100000000ull);
  if (end64 <= start) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  const u32 start_page = code_page_for_normalized_addr(start);
  const u32 end_page =
      code_page_for_normalized_addr(static_cast<u32>(end64 - 1u));
  bool any_code_page = false;
  for (u32 page = start_page; page <= end_page; ++page) {
    if (code_page_maybe_compiled(page)) {
      any_code_page = true;
      break;
    }
  }
  if (!any_code_page) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  u32 query_stamp = ++invalidation_query_stamp_;
  if (query_stamp == 0) {
    for (auto &entry : blocks_) {
      entry.second->last_invalidation_query = 0;
    }
    query_stamp = ++invalidation_query_stamp_;
  }

  const u32 end = static_cast<u32>(end64);
  for (u32 page = start_page; page <= end_page; ++page) {
    if (!code_page_maybe_compiled(page)) {
      continue;
    }
    auto page_blocks = blocks_by_page_.find(page);
    if (page_blocks == blocks_by_page_.end()) {
      clear_code_page_compiled(page);
      continue;
    }

    for (DecodedBlock *block : page_blocks->second) {
      if (block == nullptr || block->last_invalidation_query == query_stamp) {
        continue;
      }
      block->last_invalidation_query = query_stamp;
      if (block->invalidated) {
        continue;
      }

      ++stats_.invalidation_blocks_examined;
      for (u32 i = 0; i < block->tracked_range_count; ++i) {
        const auto &range = block->tracked_ranges[i];
        if (ranges_overlap(start, end, range.first, range.second)) {
          mark_block_invalidated(*block);
          ++stats_.invalidation_blocks_invalidated;
          break;
        }
      }
    }
  }
}

void CpuOptimizedBackend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active = effective_cpu_execution_mode() != CpuExecutionMode::Interpreter;
  stats_.available = true;
  log_periodic_stats();
}

void CpuOptimizedBackend::flush() {
  blocks_.clear();
  blocks_by_page_.clear();
  compiled_code_page_bitmap_.fill(0);
  stats_.block_count = 0;
  stats_.interpreter_only_blocks = 0;
  stats_.code_bytes = 0;
  have_stats_log_snapshot_ = false;
  last_stats_log_ = CpuBackendStats{};
  last_stats_log_frame_ = current_frame_;
  ++stats_.flushes;
}

CpuBackendStats CpuOptimizedBackend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.native_available = x64_jit_available();
  out.active = effective_cpu_execution_mode() != CpuExecutionMode::Interpreter;
  out.block_count = static_cast<u32>(blocks_.size());
  out.interpreter_only_blocks = 0;
  out.code_bytes = 0;
  out.native_blocks = 0;
  out.native_code_bytes = 0;
  for (const auto &entry : blocks_) {
    const DecodedBlock &block = *entry.second;
    if (block.interpreter_only_until_frame > current_frame_) {
      ++out.interpreter_only_blocks;
    }
    out.code_bytes += sizeof(DecodedBlock);
    if (block.native_fn != nullptr) {
      ++out.native_blocks;
      out.native_code_bytes += block.native_code_bytes;
    }
  }
  return out;
}

bool CpuOptimizedBackend::should_attempt_x64_compile(
    const DecodedBlock &block) {
  if (g_cpu_x64_jit_force_compile) {
    return true;
  }
  if (block.instruction_count < g_cpu_x64_jit_min_block_instructions) {
    ++stats_.native_short_block_skips;
    return false;
  }
  if (block.entry_count < g_cpu_x64_jit_hot_block_threshold) {
    ++stats_.native_hot_threshold_skips;
    return false;
  }
  return true;
}

DecodedBlock *CpuOptimizedBackend::lookup_or_decode(u32 pc) {
  const u32 key = normalized_code_addr(pc);
  auto history = block_history_.find(key);
  if (history != block_history_.end() &&
      history->second.interpreter_only_until_frame > current_frame_) {
    return nullptr;
  }

  auto existing = blocks_.find(key);
  if (existing != blocks_.end()) {
    DecodedBlock *block = existing->second.get();
    if (!block->invalidated) {
      ++stats_.cache_hits;
      return block;
    }
    unregister_block_pages(*block);
    blocks_.erase(existing);
  }

  ++stats_.cache_misses;
  return decode_block(pc);
}

DecodedBlock *CpuOptimizedBackend::decode_block(u32 pc) {
  if ((pc & 3u) != 0) {
    return nullptr;
  }
  if (blocks_.size() >= kMaxDecodedBlocks) {
    flush();
  }

  auto block = std::make_unique<DecodedBlock>();
  block->start_pc = pc;
  block->start_key = normalized_code_addr(pc);
  block->compile_frame = current_frame_;

  BlockHistory &history = block_history_[block->start_key];
  if (current_frame_ - history.window_start_frame >
      kInterpreterFallbackFrames) {
    history.window_start_frame = current_frame_;
    history.compile_count_in_window = 0;
    history.invalidation_count_in_window = 0;
  }
  ++history.compile_count_in_window;
  if (history.compile_count_in_window > kInterpreterFallbackRecompiles) {
    history.interpreter_only_until_frame =
        current_frame_ + kInterpreterOnlyFrames;
    return nullptr;
  }
  block->interpreter_only_until_frame = history.interpreter_only_until_frame;

  u32 next_pc = pc;
  for (u32 index = 0; index < kMaxBlockInstructions; ++index) {
    const u32 bits = cpu_.read_instruction_for_backend(next_pc);
    const DecodedInstruction inst = decode_instruction(next_pc, bits);
    if (!append_decoded_instruction(*block, inst)) {
      return nullptr;
    }

    if (inst.must_fallback) {
      break;
    }

    if (is_block_terminator(inst)) {
      block->has_control_flow = true;
      const u32 delay_pc = next_pc + 4u;
      const u32 delay_bits = cpu_.read_instruction_for_backend(delay_pc);
      const DecodedInstruction delay_inst =
          decode_instruction(delay_pc, delay_bits);
      if (!append_decoded_instruction(*block, delay_inst)) {
        return nullptr;
      }
      break;
    }

    next_pc += 4u;
  }

  DecodedBlock *raw = block.get();
  ++stats_.decoded_blocks;
  blocks_[raw->start_key] = std::move(block);
  register_block_pages(*raw);
  stats_.block_count = static_cast<u32>(blocks_.size());
  return raw;
}

bool CpuOptimizedBackend::append_decoded_instruction(
    DecodedBlock &block, const DecodedInstruction &inst) {
  if (block.instruction_count >= block.instructions.size() ||
      block.tracked_range_count >= block.tracked_ranges.size()) {
    return false;
  }

  block.instructions[block.instruction_count++] = inst;
  block.tracked_ranges[block.tracked_range_count++] =
      std::make_pair(inst.normalized_addr, inst.normalized_addr + 4u);
  block.has_fallback = block.has_fallback || inst.must_fallback;
  return true;
}

DecodedInstruction CpuOptimizedBackend::decode_instruction(u32 pc,
                                                           u32 bits) const {
  DecodedInstruction out{};
  out.pc = pc;
  out.bits = bits;
  out.normalized_addr = normalized_code_addr(pc);
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  out.shamt = static_cast<u8>((bits >> 6) & 0x1Fu);
  out.imm = static_cast<u16>(bits & 0xFFFFu);
  out.simm = sign_extend_16(out.imm);
  out.cycles = static_cast<u8>(cpu_.instruction_cycles(bits));

  const u32 primary = (bits >> 26) & 0x3Fu;
  switch (primary) {
  case 0x00: {
    const u32 funct = bits & 0x3Fu;
    switch (funct) {
    case 0x00:
      out.op = (bits == 0u) ? DecodedOp::Nop : DecodedOp::Sll;
      break;
    case 0x02: out.op = DecodedOp::Srl; break;
    case 0x03: out.op = DecodedOp::Sra; break;
    case 0x04: out.op = DecodedOp::Sllv; break;
    case 0x06: out.op = DecodedOp::Srlv; break;
    case 0x07: out.op = DecodedOp::Srav; break;
    case 0x08:
      out.op = DecodedOp::Jr;
      out.is_branch = true;
      out.is_unconditional_branch = true;
      break;
    case 0x09:
      out.op = DecodedOp::Jalr;
      out.is_branch = true;
      out.is_unconditional_branch = true;
      break;
    case 0x0C:
      out.op = DecodedOp::Syscall;
      out.may_raise_exception = true;
      break;
    case 0x0D:
      out.op = DecodedOp::Break;
      out.may_raise_exception = true;
      break;
    case 0x21: out.op = DecodedOp::Addu; break;
    case 0x23: out.op = DecodedOp::Subu; break;
    case 0x24: out.op = DecodedOp::And; break;
    case 0x25: out.op = DecodedOp::Or; break;
    case 0x26: out.op = DecodedOp::Xor; break;
    case 0x27: out.op = DecodedOp::Nor; break;
    case 0x2A: out.op = DecodedOp::Slt; break;
    case 0x2B: out.op = DecodedOp::Sltu; break;
    default:
      out.must_fallback = true;
      break;
    }
    break;
  }
  case 0x01: {
    switch (out.rt) {
    case 0x00: out.op = DecodedOp::Bltz; break;
    case 0x01: out.op = DecodedOp::Bgez; break;
    case 0x10: out.op = DecodedOp::Bltzal; break;
    case 0x11: out.op = DecodedOp::Bgezal; break;
    default:
      out.must_fallback = true;
      break;
    }
    if (!out.must_fallback) {
      out.is_branch = true;
      out.target =
          static_cast<u32>(static_cast<s32>(pc + 4u) + (out.simm << 2));
    }
    break;
  }
  case 0x02:
    out.op = DecodedOp::J;
    out.is_branch = true;
    out.is_unconditional_branch = true;
    out.target = ((pc + 4u) & 0xF0000000u) | ((bits & 0x03FFFFFFu) << 2);
    break;
  case 0x03:
    out.op = DecodedOp::Jal;
    out.is_branch = true;
    out.is_unconditional_branch = true;
    out.target = ((pc + 4u) & 0xF0000000u) | ((bits & 0x03FFFFFFu) << 2);
    break;
  case 0x04: out.op = DecodedOp::Beq; break;
  case 0x05: out.op = DecodedOp::Bne; break;
  case 0x06: out.op = DecodedOp::Blez; break;
  case 0x07: out.op = DecodedOp::Bgtz; break;
  case 0x09: out.op = DecodedOp::Addiu; break;
  case 0x0A: out.op = DecodedOp::Slti; break;
  case 0x0B: out.op = DecodedOp::Sltiu; break;
  case 0x0C: out.op = DecodedOp::Andi; break;
  case 0x0D: out.op = DecodedOp::Ori; break;
  case 0x0E: out.op = DecodedOp::Xori; break;
  case 0x0F: out.op = DecodedOp::Lui; break;
  case 0x10:
    out.op = DecodedOp::Cop0;
    out.must_fallback = true;
    break;
  case 0x12:
    out.op = DecodedOp::Cop2;
    out.must_fallback = true;
    break;
  case 0x20: out.op = DecodedOp::Lb; break;
  case 0x21: out.op = DecodedOp::Lh; break;
  case 0x23: out.op = DecodedOp::Lw; break;
  case 0x24: out.op = DecodedOp::Lbu; break;
  case 0x25: out.op = DecodedOp::Lhu; break;
  case 0x28: out.op = DecodedOp::Sb; break;
  case 0x29: out.op = DecodedOp::Sh; break;
  case 0x2B: out.op = DecodedOp::Sw; break;
  default:
    out.must_fallback = true;
    break;
  }

  switch (out.op) {
  case DecodedOp::Beq:
  case DecodedOp::Bne:
  case DecodedOp::Blez:
  case DecodedOp::Bgtz:
    out.is_branch = true;
    out.target =
        static_cast<u32>(static_cast<s32>(pc + 4u) + (out.simm << 2));
    break;
  case DecodedOp::Lb:
  case DecodedOp::Lh:
  case DecodedOp::Lw:
  case DecodedOp::Lbu:
  case DecodedOp::Lhu:
  case DecodedOp::Sb:
  case DecodedOp::Sh:
  case DecodedOp::Sw:
    out.may_access_memory = true;
    out.may_raise_exception = true;
    break;
  default:
    break;
  }

  return out;
}

CpuBlockRunResult CpuOptimizedBackend::execute_block(
    DecodedBlock &block, u32 max_cycles, u32 max_instructions) {
  CpuBlockRunResult result{};
  if (block.invalidated) {
    result.exit_reason = CpuBlockExitReason::Invalidated;
    return result;
  }

  ++stats_.decoded_block_entries;
  for (u32 i = 0; i < block.instruction_count; ++i) {
    if (result.cycles >= max_cycles || result.instructions >= max_instructions) {
      result.exit_reason = CpuBlockExitReason::Budget;
      break;
    }
    if (block.invalidated) {
      result.exit_reason = CpuBlockExitReason::Invalidated;
      break;
    }

    const DecodedInstruction &inst = block.instructions[i];
    if (cpu_.pc_ != inst.pc) {
      result.exit_reason = CpuBlockExitReason::PcMismatch;
      break;
    }
    if (inst.must_fallback) {
      const u32 consumed = cpu_.step();
      result.cycles += consumed;
      ++result.instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
      result.exit_reason = CpuBlockExitReason::Fallback;
      break;
    }

    if (!prepare_instruction(inst, result)) {
      break;
    }
    const bool decoded_executed = execute_decoded_instruction(inst);
    if (!decoded_executed) {
      cpu_.execute(inst.bits);
      ++stats_.fallback_instructions;
    }
    finish_instruction(inst, result, decoded_executed);

    if (cpu_.exception_raised_) {
      result.exit_reason = CpuBlockExitReason::Exception;
      break;
    }
    if (!decoded_executed) {
      result.exit_reason = CpuBlockExitReason::Fallback;
      break;
    }
    if (inst.is_branch) {
      result.exit_reason = CpuBlockExitReason::Branch;
    }
  }

  if (result.exit_reason == CpuBlockExitReason::None) {
    result.exit_reason = CpuBlockExitReason::Budget;
  }
  return result;
}

bool CpuOptimizedBackend::prepare_instruction(const DecodedInstruction &inst,
                                              CpuBlockRunResult &result) {
  cpu_.executing_step_ = true;
  cpu_.current_pc_ = cpu_.pc_;
  g_diag_current_pc = cpu_.current_pc_;
  cpu_.exception_raised_ = false;
  cpu_.cycle_penalty_ = 0;
  cpu_.in_delay_slot_ = cpu_.pending_delay_slot_;
  cpu_.active_branch_pc_ = cpu_.pending_branch_pc_;
  cpu_.pending_delay_slot_ = false;
  cpu_.pending_branch_taken_ = false;
  cpu_.pending_branch_pc_ = 0;

  if (cpu_.sys_->irq_pending()) {
    cpu_.cop0_cause_ |= (1u << 10);
  } else {
    cpu_.cop0_cause_ &= ~(1u << 10);
  }

  if (!cpu_.in_delay_slot_ && cpu_.check_irq()) {
    cpu_.exception(Exception::Interrupt);
    constexpr u32 irq_cycles = 2;
    cpu_.cycles_ += irq_cycles;
    cpu_.executing_step_ = false;
    result.cycles += irq_cycles;
    ++result.instructions;
    result.exit_reason = CpuBlockExitReason::Interrupt;
    ++stats_.interrupt_exits;
    (void)inst;
    return false;
  }

  (void)cpu_.fetch32(cpu_.pc_);
  if (cpu_.exception_raised_) {
    constexpr u32 fault_cycles = 2;
    cpu_.cycles_ += fault_cycles;
    cpu_.executing_step_ = false;
    result.cycles += fault_cycles;
    ++result.instructions;
    result.exit_reason = CpuBlockExitReason::Exception;
    ++stats_.exceptions;
    return false;
  }

  cpu_.pc_ = cpu_.next_pc_;
  cpu_.next_pc_ += 4;
  return true;
}

void CpuOptimizedBackend::finish_instruction(const DecodedInstruction &inst,
                                             CpuBlockRunResult &result,
                                             bool count_decoded) {
  if (!cpu_.exception_raised_) {
    cpu_.advance_load_delay();
  } else {
    ++stats_.exceptions;
  }

  const u32 consumed_cycles =
      cpu_.instruction_cycles(inst.bits) + cpu_.cycle_penalty_;
  cpu_.cycles_ += consumed_cycles;
  cpu_.executing_step_ = false;

  result.cycles += consumed_cycles;
  ++result.instructions;
  if (count_decoded) {
    ++stats_.optimized_instructions;
    ++stats_.decoded_instructions;
  }
}

bool CpuOptimizedBackend::execute_decoded_instruction(
    const DecodedInstruction &inst) {
  auto reg = [&](u8 index) -> u32 { return cpu_.gpr_[index]; };
  auto set_reg = [&](u8 index, u32 value) { cpu_.set_reg(index, value); };
  auto mem_addr = [&]() -> u32 {
    return reg(inst.rs) + static_cast<u32>(inst.simm);
  };
  auto note_memory = [&](u32 addr) {
    ++stats_.memory_helper_calls;
    if (is_mmio_address(addr)) {
      ++stats_.mmio_accesses;
    }
  };

  switch (inst.op) {
  case DecodedOp::Nop:
    break;
  case DecodedOp::Sll:
    set_reg(inst.rd, reg(inst.rt) << inst.shamt);
    break;
  case DecodedOp::Srl:
    set_reg(inst.rd, reg(inst.rt) >> inst.shamt);
    break;
  case DecodedOp::Sra:
    set_reg(inst.rd,
            static_cast<u32>(static_cast<s32>(reg(inst.rt)) >> inst.shamt));
    break;
  case DecodedOp::Sllv:
    set_reg(inst.rd, reg(inst.rt) << (reg(inst.rs) & 0x1Fu));
    break;
  case DecodedOp::Srlv:
    set_reg(inst.rd, reg(inst.rt) >> (reg(inst.rs) & 0x1Fu));
    break;
  case DecodedOp::Srav:
    set_reg(inst.rd, static_cast<u32>(static_cast<s32>(reg(inst.rt)) >>
                                      (reg(inst.rs) & 0x1Fu)));
    break;
  case DecodedOp::Jr:
    cpu_.begin_branch(true, reg(inst.rs));
    break;
  case DecodedOp::Jalr:
    set_reg(inst.rd, cpu_.next_pc_);
    cpu_.begin_branch(true, reg(inst.rs));
    break;
  case DecodedOp::Syscall:
    cpu_.exception(Exception::Syscall);
    break;
  case DecodedOp::Break:
    cpu_.exception(Exception::Break);
    break;
  case DecodedOp::Addu:
    set_reg(inst.rd, reg(inst.rs) + reg(inst.rt));
    break;
  case DecodedOp::Subu:
    set_reg(inst.rd, reg(inst.rs) - reg(inst.rt));
    break;
  case DecodedOp::And:
    set_reg(inst.rd, reg(inst.rs) & reg(inst.rt));
    break;
  case DecodedOp::Or:
    set_reg(inst.rd, reg(inst.rs) | reg(inst.rt));
    break;
  case DecodedOp::Xor:
    set_reg(inst.rd, reg(inst.rs) ^ reg(inst.rt));
    break;
  case DecodedOp::Nor:
    set_reg(inst.rd, ~(reg(inst.rs) | reg(inst.rt)));
    break;
  case DecodedOp::Slt:
    set_reg(inst.rd, static_cast<s32>(reg(inst.rs)) <
                         static_cast<s32>(reg(inst.rt))
                         ? 1u
                         : 0u);
    break;
  case DecodedOp::Sltu:
    set_reg(inst.rd, reg(inst.rs) < reg(inst.rt) ? 1u : 0u);
    break;
  case DecodedOp::Bltz:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) < 0, inst.target);
    break;
  case DecodedOp::Bgez:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) >= 0, inst.target);
    break;
  case DecodedOp::Bltzal:
    set_reg(31, cpu_.next_pc_);
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) < 0, inst.target);
    break;
  case DecodedOp::Bgezal:
    set_reg(31, cpu_.next_pc_);
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) >= 0, inst.target);
    break;
  case DecodedOp::J:
    cpu_.begin_branch(true, inst.target);
    break;
  case DecodedOp::Jal:
    set_reg(31, cpu_.next_pc_);
    cpu_.begin_branch(true, inst.target);
    break;
  case DecodedOp::Beq:
    cpu_.begin_branch(reg(inst.rs) == reg(inst.rt), inst.target);
    break;
  case DecodedOp::Bne:
    cpu_.begin_branch(reg(inst.rs) != reg(inst.rt), inst.target);
    break;
  case DecodedOp::Blez:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) <= 0, inst.target);
    break;
  case DecodedOp::Bgtz:
    cpu_.begin_branch(static_cast<s32>(reg(inst.rs)) > 0, inst.target);
    break;
  case DecodedOp::Addiu:
    set_reg(inst.rt, reg(inst.rs) + static_cast<u32>(inst.simm));
    break;
  case DecodedOp::Slti:
    set_reg(inst.rt, static_cast<s32>(reg(inst.rs)) < inst.simm ? 1u : 0u);
    break;
  case DecodedOp::Sltiu:
    set_reg(inst.rt, reg(inst.rs) < static_cast<u32>(inst.simm) ? 1u : 0u);
    break;
  case DecodedOp::Andi:
    set_reg(inst.rt, reg(inst.rs) & inst.imm);
    break;
  case DecodedOp::Ori:
    set_reg(inst.rt, reg(inst.rs) | inst.imm);
    break;
  case DecodedOp::Xori:
    set_reg(inst.rt, reg(inst.rs) ^ inst.imm);
    break;
  case DecodedOp::Lui:
    set_reg(inst.rt, static_cast<u32>(inst.imm) << 16);
    break;
  case DecodedOp::Lb: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u8 value = cpu_.load8(addr);
    cpu_.schedule_load(inst.rt, static_cast<u32>(sign_extend_8(value)));
    break;
  }
  case DecodedOp::Lh: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u16 value = cpu_.load16(addr);
    if (!cpu_.exception_raised_) {
      cpu_.schedule_load(inst.rt, static_cast<u32>(sign_extend_16(value)));
    }
    break;
  }
  case DecodedOp::Lw: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u32 value = cpu_.load32(addr);
    if (!cpu_.exception_raised_) {
      cpu_.schedule_load(inst.rt, value);
    }
    break;
  }
  case DecodedOp::Lbu: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.schedule_load(inst.rt, cpu_.load8(addr));
    break;
  }
  case DecodedOp::Lhu: {
    const u32 addr = mem_addr();
    note_memory(addr);
    const u16 value = cpu_.load16(addr);
    if (!cpu_.exception_raised_) {
      cpu_.schedule_load(inst.rt, value);
    }
    break;
  }
  case DecodedOp::Sb: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.store8(addr, static_cast<u8>(reg(inst.rt)));
    break;
  }
  case DecodedOp::Sh: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.store16(addr, static_cast<u16>(reg(inst.rt)));
    break;
  }
  case DecodedOp::Sw: {
    const u32 addr = mem_addr();
    note_memory(addr);
    cpu_.store32(addr, reg(inst.rt));
    break;
  }
  default:
    return false;
  }

  cpu_.gpr_[0] = 0;
  return true;
}

CpuForcedInterpreterReason
CpuOptimizedBackend::diagnostics_force_interpreter_reason() const {
  if (g_trace_cpu) {
    return CpuForcedInterpreterReason::TraceCpu;
  }
  if (g_cpu_deep_diagnostics) {
    return CpuForcedInterpreterReason::DeepDiagnostics;
  }
  if (g_log_fmv_diagnostics) {
    return CpuForcedInterpreterReason::FmvDiagnostics;
  }
  if (g_cpu_backend_compare_test_force_interpreter) {
    return CpuForcedInterpreterReason::BackendCompareTest;
  }
  return CpuForcedInterpreterReason::None;
}

bool CpuOptimizedBackend::is_block_terminator(
    const DecodedInstruction &inst) const {
  return inst.is_branch || inst.op == DecodedOp::Syscall ||
         inst.op == DecodedOp::Break;
}

bool CpuOptimizedBackend::is_mmio_address(u32 addr) const {
  const u32 phys = psx::mask_address(addr);
  return phys >= 0x1F801000u && phys < 0x1F803000u;
}

u32 CpuOptimizedBackend::normalized_code_addr(u32 addr) const {
  const u32 phys = psx::mask_address(addr);
  if (phys < 0x00800000u) {
    return phys & (psx::RAM_SIZE - 1u);
  }
  if (phys >= 0x1F800000u && phys < 0x1F801000u) {
    return phys;
  }
  if (phys >= psx::BIOS_BASE) {
    return phys;
  }
  return phys;
}

u32 CpuOptimizedBackend::code_page_for_normalized_addr(u32 addr) const {
  return addr >> kCodePageShift;
}

bool CpuOptimizedBackend::code_page_maybe_compiled(u32 page) const {
  const u32 word = page >> 6;
  const u32 bit = page & 63u;
  return (compiled_code_page_bitmap_[word] & (1ull << bit)) != 0;
}

void CpuOptimizedBackend::set_code_page_compiled(u32 page) {
  const u32 word = page >> 6;
  const u32 bit = page & 63u;
  compiled_code_page_bitmap_[word] |= (1ull << bit);
}

void CpuOptimizedBackend::clear_code_page_compiled(u32 page) {
  const u32 word = page >> 6;
  const u32 bit = page & 63u;
  compiled_code_page_bitmap_[word] &= ~(1ull << bit);
}

bool CpuOptimizedBackend::register_block_page(DecodedBlock &block, u32 page) {
  for (u32 i = 0; i < block.registered_page_count; ++i) {
    if (block.registered_pages[i] == page) {
      return true;
    }
  }
  if (block.registered_page_count >= block.registered_pages.size()) {
    return false;
  }

  block.registered_pages[block.registered_page_count++] = page;
  blocks_by_page_[page].push_back(&block);
  set_code_page_compiled(page);
  return true;
}

void CpuOptimizedBackend::register_block_pages(DecodedBlock &block) {
  for (u32 i = 0; i < block.tracked_range_count; ++i) {
    const auto &range = block.tracked_ranges[i];
    if (range.second <= range.first) {
      continue;
    }

    const u32 first_page = code_page_for_normalized_addr(range.first);
    const u32 last_page = code_page_for_normalized_addr(range.second - 1u);
    for (u32 page = first_page; page <= last_page; ++page) {
      if (!register_block_page(block, page)) {
        block.invalidated = true;
        return;
      }
    }
  }
}

void CpuOptimizedBackend::unregister_block_pages(DecodedBlock &block) {
  for (u32 i = 0; i < block.registered_page_count; ++i) {
    const u32 page = block.registered_pages[i];
    auto found = blocks_by_page_.find(page);
    if (found == blocks_by_page_.end()) {
      clear_code_page_compiled(page);
      continue;
    }

    auto &page_blocks = found->second;
    page_blocks.erase(std::remove(page_blocks.begin(), page_blocks.end(),
                                  &block),
                      page_blocks.end());
    if (page_blocks.empty()) {
      blocks_by_page_.erase(found);
      clear_code_page_compiled(page);
    }
  }
  block.registered_page_count = 0;
}

bool CpuOptimizedBackend::ranges_overlap(u32 a0, u32 a1, u32 b0,
                                         u32 b1) const {
  return a0 < b1 && b0 < a1;
}

void CpuOptimizedBackend::mark_block_invalidated(DecodedBlock &block) {
  block.invalidated = true;
  ++stats_.invalidations;

  BlockHistory &history = block_history_[block.start_key];
  if (current_frame_ - history.window_start_frame >
      kInterpreterFallbackFrames) {
    history.window_start_frame = current_frame_;
    history.compile_count_in_window = 0;
    history.invalidation_count_in_window = 0;
  }
  ++history.invalidation_count_in_window;

  if (current_frame_ - block.last_invalidate_frame <=
      kInterpreterFallbackFrames) {
    ++block.invalidations_in_window;
  } else {
    block.invalidations_in_window = 1;
  }
  block.last_invalidate_frame = current_frame_;
  ++block.recompiles_in_window;

  if (block.recompiles_in_window >= kInterpreterFallbackRecompiles ||
      block.invalidations_in_window >= kInterpreterFallbackRecompiles ||
      history.compile_count_in_window > kInterpreterFallbackRecompiles ||
      history.invalidation_count_in_window >= kInterpreterFallbackRecompiles) {
    history.interpreter_only_until_frame =
        current_frame_ + kInterpreterOnlyFrames;
    block.interpreter_only_until_frame = history.interpreter_only_until_frame;
  }
}

void CpuOptimizedBackend::record_forced_interpreter(
    CpuForcedInterpreterReason reason, u32 instructions) {
  stats_.forced_interpreter_instructions += instructions;
  switch (reason) {
  case CpuForcedInterpreterReason::TraceCpu:
    stats_.forced_interpreter_trace += instructions;
    break;
  case CpuForcedInterpreterReason::DeepDiagnostics:
    stats_.forced_interpreter_deep_diagnostics += instructions;
    break;
  case CpuForcedInterpreterReason::FmvDiagnostics:
    stats_.forced_interpreter_fmv += instructions;
    break;
  case CpuForcedInterpreterReason::BackendCompareTest:
    stats_.forced_interpreter_backend_compare += instructions;
    break;
  case CpuForcedInterpreterReason::Other:
    stats_.forced_interpreter_other += instructions;
    break;
  case CpuForcedInterpreterReason::None:
  default:
    break;
  }
}

void CpuOptimizedBackend::record_exit(CpuBlockExitReason reason) {
  switch (reason) {
  case CpuBlockExitReason::Budget:
    ++stats_.budget_exits;
    break;
  case CpuBlockExitReason::Fallback:
    ++stats_.fallback_exits;
    break;
  case CpuBlockExitReason::Invalidated:
    ++stats_.invalidated_exits;
    break;
  case CpuBlockExitReason::PcMismatch:
    ++stats_.pc_mismatch_exits;
    break;
  case CpuBlockExitReason::Exception:
  case CpuBlockExitReason::Interrupt:
  case CpuBlockExitReason::Branch:
  case CpuBlockExitReason::None:
  default:
    break;
  }
}

void CpuOptimizedBackend::record_native_reject(
    NativeBlockRejectReason reason) {
  switch (reason) {
  case NativeBlockRejectReason::Branch:
    ++stats_.native_reject_branch;
    break;
  case NativeBlockRejectReason::Memory:
    ++stats_.native_reject_memory;
    break;
  case NativeBlockRejectReason::Cop0:
    ++stats_.native_reject_cop0;
    break;
  case NativeBlockRejectReason::Cop2:
    ++stats_.native_reject_cop2;
    break;
  case NativeBlockRejectReason::ExceptionOrUnknown:
    ++stats_.native_reject_exception_unknown;
    break;
  case NativeBlockRejectReason::None:
  default:
    break;
  }
}

void CpuOptimizedBackend::warn_forced_interpreter_once(
    CpuForcedInterpreterReason reason) {
  if (reason == CpuForcedInterpreterReason::None ||
      last_forced_interpreter_warning_ == reason) {
    return;
  }
  last_forced_interpreter_warning_ = reason;
  LOG_WARN("CPU backend forced to interpreter: %s",
           cpu_forced_interpreter_reason_name(reason));
}

void CpuOptimizedBackend::log_periodic_stats() {
  if (!g_cpu_backend_stats_logging ||
      effective_cpu_execution_mode() == CpuExecutionMode::Interpreter) {
    have_stats_log_snapshot_ = false;
    last_stats_log_frame_ = current_frame_;
    return;
  }

  const CpuBackendStats current = stats();
  if (!have_stats_log_snapshot_) {
    last_stats_log_ = current;
    last_stats_log_frame_ = current_frame_;
    have_stats_log_snapshot_ = true;
    return;
  }

  const u32 cadence = std::max<u32>(1u, g_cpu_backend_stats_log_frames);
  const u32 frame_delta = current_frame_ - last_stats_log_frame_;
  if (frame_delta < cadence) {
    return;
  }

  const CpuBackendStats delta = delta_stats(current, last_stats_log_);
  const u64 instruction_delta = delta.decoded_instructions +
                                delta.native_instructions +
                                delta.fallback_instructions;
  if (instruction_delta != 0 || delta.native_compile_attempts != 0 ||
      delta.native_to_decoded_fallbacks != 0 ||
      delta.forced_interpreter_slices != 0) {
    log_stats_section(current, delta, frame_delta);
  }

  last_stats_log_ = current;
  last_stats_log_frame_ = current_frame_;
}

void CpuOptimizedBackend::log_stats_section(
    const CpuBackendStats &current, const CpuBackendStats &delta,
    u32 frame_delta) const {
  const double decoded_avg =
      delta.decoded_block_entries == 0
          ? 0.0
          : static_cast<double>(delta.decoded_instructions) /
                static_cast<double>(delta.decoded_block_entries);
  const double native_avg =
      delta.native_block_entries == 0
          ? 0.0
          : static_cast<double>(delta.native_instructions) /
                static_cast<double>(delta.native_block_entries);
  const u64 instr_total = delta.decoded_instructions +
                          delta.native_instructions +
                          delta.fallback_instructions;
  const double native_pct =
      instr_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.native_instructions)) /
                static_cast<double>(instr_total);
  const double decoded_pct =
      instr_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.decoded_instructions)) /
                static_cast<double>(instr_total);
  const double fallback_pct =
      instr_total == 0
          ? 0.0
          : (100.0 * static_cast<double>(delta.fallback_instructions)) /
                static_cast<double>(instr_total);

  struct Reason {
    const char *name = "";
    u64 count = 0;
  };
  std::array<Reason, 10> reasons = {{
      {"branch", delta.native_reject_branch},
      {"memory", delta.native_reject_memory},
      {"cop0", delta.native_reject_cop0},
      {"cop2_gte", delta.native_reject_cop2},
      {"exception_unknown", delta.native_reject_exception_unknown},
      {"load_delay_unsafe_state", delta.native_reject_unsafe_state},
      {"budget", delta.native_reject_budget},
      {"icache", delta.native_reject_icache},
      {"cold", delta.native_hot_threshold_skips},
      {"too_short", delta.native_short_block_skips},
  }};
  std::sort(reasons.begin(), reasons.end(),
            [](const Reason &a, const Reason &b) {
              return a.count > b.count;
            });

  LOG_INFO("=== CPU Backend Stats ===");
  LOG_INFO("CPU_BACKEND_STATS frame=%u frames=%u mode=%s blocks=%u native_blocks=%llu code_bytes=%zu native_code_bytes=%zu",
           current_frame_, frame_delta,
           cpu_execution_mode_name(effective_cpu_execution_mode()),
           current.block_count,
           static_cast<unsigned long long>(current.native_blocks),
           current.code_bytes, current.native_code_bytes);
  LOG_INFO("CPU_BACKEND_STATS entries decoded=%llu native=%llu avg_decoded=%.2f avg_native=%.2f",
           static_cast<unsigned long long>(delta.decoded_block_entries),
           static_cast<unsigned long long>(delta.native_block_entries),
           decoded_avg, native_avg);
  LOG_INFO("CPU_BACKEND_STATS instructions decoded=%llu native=%llu fallback=%llu native_pct=%.2f decoded_pct=%.2f fallback_pct=%.2f",
           static_cast<unsigned long long>(delta.decoded_instructions),
           static_cast<unsigned long long>(delta.native_instructions),
           static_cast<unsigned long long>(delta.fallback_instructions),
           native_pct, decoded_pct, fallback_pct);
  LOG_INFO("CPU_BACKEND_STATS forced_interpreter reason=%s slices=%llu instructions=%llu trace=%llu deep_diagnostics=%llu fmv=%llu backend_compare=%llu other=%llu",
           cpu_forced_interpreter_reason_name(
               current.forced_interpreter_last_reason),
           static_cast<unsigned long long>(delta.forced_interpreter_slices),
           static_cast<unsigned long long>(
               delta.forced_interpreter_instructions),
           static_cast<unsigned long long>(delta.forced_interpreter_trace),
           static_cast<unsigned long long>(
               delta.forced_interpreter_deep_diagnostics),
           static_cast<unsigned long long>(delta.forced_interpreter_fmv),
           static_cast<unsigned long long>(
               delta.forced_interpreter_backend_compare),
           static_cast<unsigned long long>(delta.forced_interpreter_other));
  LOG_INFO("CPU_BACKEND_STATS native attempts=%llu successes=%llu compiled=%llu failures=%llu unsafe=%llu decoded_fallbacks=%llu",
           static_cast<unsigned long long>(delta.native_compile_attempts),
           static_cast<unsigned long long>(delta.native_compile_successes),
           static_cast<unsigned long long>(delta.native_blocks_compiled),
           static_cast<unsigned long long>(delta.native_compile_failures),
           static_cast<unsigned long long>(delta.native_rejected_unsafe_blocks),
           static_cast<unsigned long long>(delta.native_to_decoded_fallbacks));
  LOG_INFO("CPU_BACKEND_STATS native_gating hot_threshold=%u min_block=%u force=%u cold_skips=%llu short_skips=%llu",
           g_cpu_x64_jit_hot_block_threshold,
           g_cpu_x64_jit_min_block_instructions,
           g_cpu_x64_jit_force_compile ? 1u : 0u,
           static_cast<unsigned long long>(delta.native_hot_threshold_skips),
           static_cast<unsigned long long>(delta.native_short_block_skips));
  LOG_INFO("CPU_BACKEND_STATS top_rejections %s=%llu %s=%llu %s=%llu",
           reasons[0].name, static_cast<unsigned long long>(reasons[0].count),
           reasons[1].name, static_cast<unsigned long long>(reasons[1].count),
           reasons[2].name, static_cast<unsigned long long>(reasons[2].count));
  LOG_INFO("CPU_BACKEND_STATS rejection_counts branch=%llu memory=%llu cop0=%llu cop2_gte=%llu exception_unknown=%llu load_delay_unsafe_state=%llu budget=%llu icache=%llu",
           static_cast<unsigned long long>(delta.native_reject_branch),
           static_cast<unsigned long long>(delta.native_reject_memory),
           static_cast<unsigned long long>(delta.native_reject_cop0),
           static_cast<unsigned long long>(delta.native_reject_cop2),
           static_cast<unsigned long long>(
               delta.native_reject_exception_unknown),
           static_cast<unsigned long long>(delta.native_reject_unsafe_state),
           static_cast<unsigned long long>(delta.native_reject_budget),
           static_cast<unsigned long long>(delta.native_reject_icache));
  LOG_INFO("CPU_BACKEND_STATS cache hits=%llu misses=%llu invalidations=%llu invalidation_queries=%llu no_code_page=%llu examined=%llu invalidated=%llu flushes=%llu",
           static_cast<unsigned long long>(delta.cache_hits),
           static_cast<unsigned long long>(delta.cache_misses),
           static_cast<unsigned long long>(delta.invalidations),
           static_cast<unsigned long long>(delta.invalidation_queries),
           static_cast<unsigned long long>(
               delta.invalidation_fast_no_code_page_exits),
           static_cast<unsigned long long>(delta.invalidation_blocks_examined),
           static_cast<unsigned long long>(
               delta.invalidation_blocks_invalidated),
           static_cast<unsigned long long>(delta.flushes));
  LOG_INFO("=== End CPU Backend Stats ===");
}

void CpuOptimizedBackend::warn_x64_unavailable_once() {
  static bool process_warned = false;
  if (x64_unavailable_warned_ || process_warned) {
    return;
  }
  x64_unavailable_warned_ = true;
  process_warned = true;
  LOG_WARN(
      "CPU: x64 JIT mode requested, but no real native emitter is enabled yet; using decoded block interpreter.");
}
