#include "cpu_recompiler.h"
#include "system.h"
#include <algorithm>

DecodedBlock::~DecodedBlock() = default;

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
  if (diagnostics_force_interpreter()) {
    while (total.cycles < max_cycles && total.instructions < max_instructions) {
      const u32 consumed = cpu_.step();
      total.cycles += consumed;
      ++total.instructions;
      ++stats_.interpreter_fallback_steps;
      ++stats_.fallback_instructions;
    }
    return total;
  }

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

    const u32 cycle_budget = max_cycles - total.cycles;
    const u32 instruction_budget = max_instructions - total.instructions;
    CpuBlockRunResult result =
        execute_block(*block, cycle_budget, instruction_budget);
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
}

void CpuOptimizedBackend::flush() {
  blocks_.clear();
  blocks_by_page_.clear();
  compiled_code_page_bitmap_.fill(0);
  stats_.block_count = 0;
  stats_.interpreter_only_blocks = 0;
  stats_.code_bytes = 0;
  ++stats_.flushes;
}

CpuBackendStats CpuOptimizedBackend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active = effective_cpu_execution_mode() != CpuExecutionMode::Interpreter;
  out.block_count = static_cast<u32>(blocks_.size());
  out.interpreter_only_blocks = 0;
  out.code_bytes = 0;
  for (const auto &entry : blocks_) {
    const DecodedBlock &block = *entry.second;
    if (block.interpreter_only_until_frame > current_frame_) {
      ++out.interpreter_only_blocks;
    }
    out.code_bytes += sizeof(DecodedBlock);
  }
  return out;
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

bool CpuOptimizedBackend::diagnostics_force_interpreter() const {
  return g_trace_cpu || g_cpu_deep_diagnostics || g_log_fmv_diagnostics;
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

void CpuOptimizedBackend::warn_x64_unavailable_once() {
  if (x64_unavailable_warned_) {
    return;
  }
  x64_unavailable_warned_ = true;
  LOG_WARN(
      "CPU: x64 JIT mode requested, but no real native emitter is enabled yet; using decoded block interpreter.");
}
