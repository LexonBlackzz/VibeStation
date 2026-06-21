#include "cpu_recompiler.h"
#include "system.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>

#if defined(VIBESTATION_ENABLE_X64_JIT) && \
    (defined(_M_X64) || defined(__x86_64__))
#include <xbyak/xbyak.h>
#define VIBESTATION_X64_JIT_SUPPORTED 1
#else
#define VIBESTATION_X64_JIT_SUPPORTED 0
#endif

namespace {
struct X64NativeContext {
#if VIBESTATION_X64_JIT_SUPPORTED
  explicit X64NativeContext(size_t code_size) : code(code_size) {}
  Xbyak::CodeGenerator code;
#endif
  u32 *gpr = nullptr;
  u32 *pc = nullptr;
  u32 *next_pc = nullptr;
  u32 *current_pc = nullptr;
  u64 *cycles = nullptr;
  u32 end_pc = 0;
  u32 end_next_pc = 0;
  u32 last_pc = 0;
  u32 instruction_count = 0;
  u32 base_cycles = 0;
  u32 cycles_to_add = 0;
};

void destroy_x64_native_context(void *context) {
  delete static_cast<X64NativeContext *>(context);
}

bool is_x64_stage1_op(DecodedOp op) {
  switch (op) {
  case DecodedOp::Nop:
  case DecodedOp::Sll:
  case DecodedOp::Srl:
  case DecodedOp::Sra:
  case DecodedOp::Sllv:
  case DecodedOp::Srlv:
  case DecodedOp::Srav:
  case DecodedOp::Addu:
  case DecodedOp::Subu:
  case DecodedOp::And:
  case DecodedOp::Or:
  case DecodedOp::Xor:
  case DecodedOp::Nor:
  case DecodedOp::Slt:
  case DecodedOp::Sltu:
  case DecodedOp::Addiu:
  case DecodedOp::Slti:
  case DecodedOp::Sltiu:
  case DecodedOp::Andi:
  case DecodedOp::Ori:
  case DecodedOp::Xori:
  case DecodedOp::Lui:
    return true;
  default:
    return false;
  }
}

#if VIBESTATION_X64_JIT_SUPPORTED
void emit_read_gpr(Xbyak::CodeGenerator &code, const Xbyak::Reg32 &dst,
                   const Xbyak::Reg64 &gpr, u8 index) {
  if (index == 0) {
    code.xor_(dst, dst);
  } else {
    code.mov(dst, code.dword[gpr + static_cast<int>(index) * 4]);
  }
}

void emit_write_gpr(Xbyak::CodeGenerator &code, const Xbyak::Reg64 &gpr,
                    u8 index, const Xbyak::Reg32 &src) {
  if (index != 0) {
    code.mov(code.dword[gpr + static_cast<int>(index) * 4], src);
  }
}

void emit_mov_imm32(Xbyak::CodeGenerator &code, const Xbyak::Reg32 &dst,
                    u32 value) {
  code.mov(dst, value);
}

void emit_x64_instruction(Xbyak::CodeGenerator &code,
                          const DecodedInstruction &inst,
                          const Xbyak::Reg64 &gpr) {
  using namespace Xbyak;
  switch (inst.op) {
  case DecodedOp::Nop:
    break;
  case DecodedOp::Sll:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    if (inst.shamt != 0) {
      code.shl(code.eax, inst.shamt);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Srl:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    if (inst.shamt != 0) {
      code.shr(code.eax, inst.shamt);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Sra:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    if (inst.shamt != 0) {
      code.sar(code.eax, inst.shamt);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Sllv:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    emit_read_gpr(code, code.ecx, gpr, inst.rs);
    code.and_(code.ecx, 0x1F);
    code.shl(code.eax, code.cl);
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Srlv:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    emit_read_gpr(code, code.ecx, gpr, inst.rs);
    code.and_(code.ecx, 0x1F);
    code.shr(code.eax, code.cl);
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Srav:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    emit_read_gpr(code, code.ecx, gpr, inst.rs);
    code.and_(code.ecx, 0x1F);
    code.sar(code.eax, code.cl);
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Addu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.add(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Subu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.sub(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::And:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt == 0) {
      code.xor_(code.eax, code.eax);
    } else {
      code.and_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Or:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.or_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Xor:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.xor_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Nor:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.or_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    code.not_(code.eax);
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Slt:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt == 0) {
      code.cmp(code.eax, 0);
    } else {
      code.cmp(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    code.setl(code.al);
    code.movzx(code.eax, code.al);
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Sltu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt == 0) {
      code.cmp(code.eax, 0);
    } else {
      code.cmp(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    code.setb(code.al);
    code.movzx(code.eax, code.al);
    emit_write_gpr(code, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Addiu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.simm != 0) {
      code.add(code.eax, static_cast<u32>(inst.simm));
    }
    emit_write_gpr(code, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Slti:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    emit_mov_imm32(code, code.r10d, static_cast<u32>(inst.simm));
    code.cmp(code.eax, code.r10d);
    code.setl(code.al);
    code.movzx(code.eax, code.al);
    emit_write_gpr(code, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Sltiu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    emit_mov_imm32(code, code.r10d, static_cast<u32>(inst.simm));
    code.cmp(code.eax, code.r10d);
    code.setb(code.al);
    code.movzx(code.eax, code.al);
    emit_write_gpr(code, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Andi:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    code.and_(code.eax, static_cast<u32>(inst.imm));
    emit_write_gpr(code, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Ori:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.imm != 0) {
      code.or_(code.eax, static_cast<u32>(inst.imm));
    }
    emit_write_gpr(code, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Xori:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.imm != 0) {
      code.xor_(code.eax, static_cast<u32>(inst.imm));
    }
    emit_write_gpr(code, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Lui:
    emit_mov_imm32(code, code.eax, static_cast<u32>(inst.imm) << 16);
    emit_write_gpr(code, gpr, inst.rt, code.eax);
    break;
  default:
    break;
  }
}

void emit_x64_block(X64NativeContext &context, const DecodedBlock &block) {
  using namespace Xbyak;
  CodeGenerator &code = context.code;
#if defined(_WIN32)
  const Reg64 ctx = code.r11;
  const Reg64 result = code.rdx;
  code.mov(ctx, code.rcx);
#else
  const Reg64 ctx = code.r11;
  const Reg64 result = code.rsi;
  code.mov(ctx, code.rdi);
#endif
  const Reg64 gpr = code.r8;
  code.mov(gpr, code.ptr[ctx + offsetof(X64NativeContext, gpr)]);

  for (u32 i = 0; i < block.instruction_count; ++i) {
    emit_x64_instruction(code, block.instructions[i], gpr);
  }

  code.mov(code.dword[gpr], 0);

  code.mov(code.r9, code.ptr[ctx + offsetof(X64NativeContext, pc)]);
  code.mov(code.r10d, code.dword[ctx + offsetof(X64NativeContext, end_pc)]);
  code.mov(code.dword[code.r9], code.r10d);

  code.mov(code.r9, code.ptr[ctx + offsetof(X64NativeContext, next_pc)]);
  code.mov(code.r10d,
           code.dword[ctx + offsetof(X64NativeContext, end_next_pc)]);
  code.mov(code.dword[code.r9], code.r10d);

  code.mov(code.r9, code.ptr[ctx + offsetof(X64NativeContext, current_pc)]);
  code.mov(code.r10d, code.dword[ctx + offsetof(X64NativeContext, last_pc)]);
  code.mov(code.dword[code.r9], code.r10d);

  code.mov(code.r9, code.ptr[ctx + offsetof(X64NativeContext, cycles)]);
  code.mov(code.rax, code.ptr[code.r9]);
  code.mov(code.r10d,
           code.dword[ctx + offsetof(X64NativeContext, cycles_to_add)]);
  code.add(code.rax, code.r10);
  code.mov(code.ptr[code.r9], code.rax);

  code.mov(code.r10d,
           code.dword[ctx + offsetof(X64NativeContext, cycles_to_add)]);
  code.mov(code.dword[result + offsetof(CpuBlockRunResult, cycles)],
           code.r10d);
  code.mov(code.r10d,
           code.dword[ctx + offsetof(X64NativeContext, instruction_count)]);
  code.mov(code.dword[result + offsetof(CpuBlockRunResult, instructions)],
           code.r10d);
  code.mov(code.byte[result + offsetof(CpuBlockRunResult, exit_reason)],
           static_cast<uint8_t>(CpuBlockExitReason::None));
  code.ret();
  code.ready();
}
#endif
} // namespace

bool CpuOptimizedBackend::x64_jit_available() const {
  return VIBESTATION_X64_JIT_SUPPORTED != 0;
}

NativeBlockRejectReason CpuOptimizedBackend::classify_x64_reject_reason(
    const DecodedBlock &block) const {
  if (block.instruction_count == 0) {
    return NativeBlockRejectReason::ExceptionOrUnknown;
  }
  for (u32 i = 0; i < block.instruction_count; ++i) {
    const DecodedInstruction &inst = block.instructions[i];
    if (inst.is_branch) {
      return NativeBlockRejectReason::Branch;
    }
    if (inst.may_access_memory) {
      return NativeBlockRejectReason::Memory;
    }
    if (inst.op == DecodedOp::Cop0) {
      return NativeBlockRejectReason::Cop0;
    }
    if (inst.op == DecodedOp::Cop2) {
      return NativeBlockRejectReason::Cop2;
    }
    if (inst.must_fallback || inst.may_raise_exception ||
        !is_x64_stage1_op(inst.op)) {
      return NativeBlockRejectReason::ExceptionOrUnknown;
    }
  }
  if (block.has_control_flow) {
    return NativeBlockRejectReason::Branch;
  }
  if (block.has_fallback) {
    return NativeBlockRejectReason::ExceptionOrUnknown;
  }
  return NativeBlockRejectReason::None;
}

bool CpuOptimizedBackend::ensure_x64_safety_checked(DecodedBlock &block) {
  if (block.native_safety_checked) {
    return block.native_stage1_safe;
  }

  block.native_safety_checked = true;
  block.native_reject_reason = classify_x64_reject_reason(block);
  block.native_stage1_safe =
      block.native_reject_reason == NativeBlockRejectReason::None;
  if (!block.native_stage1_safe) {
    block.native_rejected_unsafe = true;
    ++stats_.native_rejected_unsafe_blocks;
    record_native_reject(block.native_reject_reason);
    return false;
  }
  return true;
}

bool CpuOptimizedBackend::compile_x64_block(DecodedBlock &block) {
  if (block.native_compile_attempted) {
    return block.native_fn != nullptr;
  }
  block.native_compile_attempted = true;
  ++stats_.native_compile_attempts;

  if (!x64_jit_available()) {
    ++stats_.native_compile_failures;
    return false;
  }

  if (!ensure_x64_safety_checked(block)) {
    return false;
  }

#if VIBESTATION_X64_JIT_SUPPORTED
  try {
    auto context = std::make_unique<X64NativeContext>(4096);
    context->gpr = cpu_.gpr_;
    context->pc = &cpu_.pc_;
    context->next_pc = &cpu_.next_pc_;
    context->current_pc = &cpu_.current_pc_;
    context->cycles = &cpu_.cycles_;
    context->instruction_count = block.instruction_count;
    context->end_pc = block.start_pc + block.instruction_count * 4u;
    context->end_next_pc = context->end_pc + 4u;
    context->last_pc = context->end_pc - 4u;
    for (u32 i = 0; i < block.instruction_count; ++i) {
      context->base_cycles += block.instructions[i].cycles;
    }

    emit_x64_block(*context, block);
    block.native_fn = context->code.getCode<DecodedBlock::NativeFn>();
    block.native_code_bytes = context->code.getSize();
    block.native_context = context.release();
    block.destroy_native_context = destroy_x64_native_context;
    ++stats_.native_compile_successes;
    ++stats_.native_blocks_compiled;
    stats_.native_code_bytes += block.native_code_bytes;
    return true;
  } catch (const std::exception &) {
    ++stats_.native_compile_failures;
    return false;
  }
#else
  ++stats_.native_compile_failures;
  return false;
#endif
}

CpuBlockRunResult CpuOptimizedBackend::execute_native_block(
    DecodedBlock &block, u32 max_cycles, u32 max_instructions) {
  CpuBlockRunResult result{};
  if (block.invalidated) {
    result.exit_reason = CpuBlockExitReason::Invalidated;
    return result;
  }

  if (block.native_fn == nullptr || block.native_context == nullptr) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_unsafe_state;
    return execute_block(block, max_cycles, max_instructions);
  }

  auto *context = static_cast<X64NativeContext *>(block.native_context);
  if (block.instruction_count > max_instructions) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_budget;
    return execute_block(block, max_cycles, max_instructions);
  }
  if (cpu_.pc_ != block.start_pc || cpu_.next_pc_ != block.start_pc + 4u ||
      cpu_.pending_delay_slot_ || cpu_.in_delay_slot_ ||
      cpu_.load_.reg != 0 || cpu_.next_load_.reg != 0) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_unsafe_state;
    return execute_block(block, max_cycles, max_instructions);
  }

  if (cpu_.sys_->irq_pending()) {
    cpu_.cop0_cause_ |= (1u << 10);
  } else {
    cpu_.cop0_cause_ &= ~(1u << 10);
  }
  if (cpu_.check_irq()) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_unsafe_state;
    return execute_block(block, max_cycles, max_instructions);
  }

  struct FetchFill {
    u32 index = 0;
    u32 tag = 0;
    u32 base = 0;
  };
  std::array<FetchFill, DecodedBlock::kMaxInstructions> fills{};
  u32 fill_count = 0;
  u32 fetch_penalty = 0;

  for (u32 i = 0; i < block.instruction_count; ++i) {
    const u32 pc = block.instructions[i].pc;
    if ((pc & 3u) != 0) {
      ++stats_.native_to_decoded_fallbacks;
      ++stats_.native_reject_icache;
      return execute_block(block, max_cycles, max_instructions);
    }

    if (!cpu_.instruction_cacheable(pc)) {
      continue;
    }

    const u32 index = (pc >> 4) & 0xFFu;
    const u32 tag = psx::mask_address(pc) & ~0x0Fu;
    bool hit = cpu_.icache_[index].valid && cpu_.icache_[index].tag == tag;
    for (u32 fill = 0; fill < fill_count; ++fill) {
      if (fills[fill].index == index && fills[fill].tag == tag) {
        hit = true;
        break;
      }
    }
    if (!hit) {
      fetch_penalty += 4u;
      if (fill_count >= fills.size()) {
        ++stats_.native_to_decoded_fallbacks;
        ++stats_.native_reject_icache;
        return execute_block(block, max_cycles, max_instructions);
      }
      fills[fill_count++] = {index, tag, pc & ~0x0Fu};
    }
  }

  const u32 total_cycles = context->base_cycles + fetch_penalty;
  if (total_cycles > max_cycles) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_budget;
    return execute_block(block, max_cycles, max_instructions);
  }

  for (u32 fill = 0; fill < fill_count; ++fill) {
    auto &line = cpu_.icache_[fills[fill].index];
    line.tag = fills[fill].tag;
    for (u32 word = 0; word < 4u; ++word) {
      line.words[word] =
          cpu_.sys_->read32_instruction(fills[fill].base + word * 4u);
    }
    line.valid = true;
  }

  context->cycles_to_add = total_cycles;
  block.native_fn(block.native_context, &result);

  cpu_.cycle_penalty_ = 0;
  cpu_.executing_step_ = false;
  cpu_.exception_raised_ = false;
  cpu_.in_delay_slot_ = false;
  cpu_.pending_delay_slot_ = false;
  cpu_.pending_branch_taken_ = false;
  cpu_.pending_branch_pc_ = 0;
  cpu_.active_branch_pc_ = 0;
  g_diag_current_pc = cpu_.current_pc_;

  ++stats_.native_block_entries;
  stats_.native_instructions += result.instructions;
  stats_.native_cycles += result.cycles;
  stats_.optimized_instructions += result.instructions;
  return result;
}
