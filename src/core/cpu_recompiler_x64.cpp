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
  CpuOptimizedBackend *backend = nullptr;
  Cpu *cpu = nullptr;
  DecodedBlock *block = nullptr;
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
  u32 max_cycles = 0;
  u32 max_instructions = 0;
  bool uses_instruction_helpers = false;
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

bool is_x64_memory_op(DecodedOp op) {
  switch (op) {
  case DecodedOp::Lb:
  case DecodedOp::Lh:
  case DecodedOp::Lw:
  case DecodedOp::Lbu:
  case DecodedOp::Lhu:
  case DecodedOp::Sb:
  case DecodedOp::Sh:
  case DecodedOp::Sw:
    return true;
  default:
    return false;
  }
}

bool is_x64_stage2_op(DecodedOp op) {
  return is_x64_stage1_op(op) || is_x64_memory_op(op);
}

bool block_uses_instruction_helpers(const DecodedBlock &block) {
  for (u32 i = 0; i < block.instruction_count; ++i) {
    if (is_x64_memory_op(block.instructions[i].op)) {
      return true;
    }
  }
  return false;
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

void emit_absolute_call(Xbyak::CodeGenerator &code, uintptr_t fn) {
  code.mov(code.rax, fn);
  code.call(code.rax);
}

void emit_prepare_helper_call(Xbyak::CodeGenerator &code,
                              const Xbyak::Reg64 &ctx,
                              const Xbyak::Reg64 &result, u32 index,
                              uintptr_t prepare_fn) {
#if defined(_WIN32)
  code.mov(code.rcx, ctx);
  code.mov(code.edx, index);
  code.mov(code.r8, result);
#else
  code.mov(code.rdi, ctx);
  code.mov(code.esi, index);
  code.mov(code.rdx, result);
#endif
  emit_absolute_call(code, prepare_fn);
}

void emit_finish_helper_call(Xbyak::CodeGenerator &code,
                             const Xbyak::Reg64 &ctx,
                             const Xbyak::Reg64 &result, u32 index,
                             bool memory_instruction, uintptr_t finish_fn) {
#if defined(_WIN32)
  code.mov(code.rcx, ctx);
  code.mov(code.edx, index);
  code.mov(code.r8, result);
  code.mov(code.r9d, memory_instruction ? 1u : 0u);
#else
  code.mov(code.rdi, ctx);
  code.mov(code.esi, index);
  code.mov(code.rdx, result);
  code.mov(code.ecx, memory_instruction ? 1u : 0u);
#endif
  emit_absolute_call(code, finish_fn);
}

void emit_memory_helper_call(Xbyak::CodeGenerator &code,
                             const Xbyak::Reg64 &ctx,
                             const Xbyak::Reg64 &gpr,
                             const DecodedInstruction &inst,
                             uintptr_t memory_fn) {
  emit_read_gpr(code, code.r10d, gpr, inst.rs);
  if (inst.simm != 0) {
    code.add(code.r10d, static_cast<u32>(inst.simm));
  }

  switch (inst.op) {
  case DecodedOp::Sb:
  case DecodedOp::Sh:
  case DecodedOp::Sw:
    emit_read_gpr(code, code.r11d, gpr, inst.rt);
    break;
  default:
    code.mov(code.r11d, static_cast<u32>(inst.rt));
    break;
  }

#if defined(_WIN32)
  code.mov(code.rcx, ctx);
  code.mov(code.edx, static_cast<u32>(inst.op));
  code.mov(code.r8d, code.r11d);
  code.mov(code.r9d, code.r10d);
#else
  code.mov(code.rdi, ctx);
  code.mov(code.esi, static_cast<u32>(inst.op));
  code.mov(code.edx, code.r11d);
  code.mov(code.ecx, code.r10d);
#endif
  emit_absolute_call(code, memory_fn);
}

void emit_x64_direct_block(X64NativeContext &context,
                           const DecodedBlock &block) {
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

void emit_x64_helper_block(X64NativeContext &context,
                           const DecodedBlock &block, uintptr_t prepare_fn,
                           uintptr_t memory_fn, uintptr_t finish_fn) {
  using namespace Xbyak;
  CodeGenerator &code = context.code;
  Label done;

  code.push(code.r12);
  code.push(code.r13);
  code.push(code.r14);
  code.sub(code.rsp, 32);

  const Reg64 ctx = code.r12;
  const Reg64 result = code.r13;
  const Reg64 gpr = code.r14;
#if defined(_WIN32)
  code.mov(ctx, code.rcx);
  code.mov(result, code.rdx);
#else
  code.mov(ctx, code.rdi);
  code.mov(result, code.rsi);
#endif
  code.mov(gpr, code.ptr[ctx + offsetof(X64NativeContext, gpr)]);

  for (u32 i = 0; i < block.instruction_count; ++i) {
    const DecodedInstruction &inst = block.instructions[i];
    const bool memory_instruction = is_x64_memory_op(inst.op);

    emit_prepare_helper_call(code, ctx, result, i, prepare_fn);
    code.test(code.al, code.al);
    code.jz(done, CodeGenerator::T_NEAR);

    if (memory_instruction) {
      emit_memory_helper_call(code, ctx, gpr, inst, memory_fn);
      code.test(code.al, code.al);
      code.jz(done, CodeGenerator::T_NEAR);
    } else {
      emit_x64_instruction(code, inst, gpr);
    }

    code.mov(code.dword[gpr], 0);
    emit_finish_helper_call(code, ctx, result, i, memory_instruction,
                            finish_fn);
    code.test(code.al, code.al);
    code.jz(done, CodeGenerator::T_NEAR);
  }

  code.L(done);
  code.add(code.rsp, 32);
  code.pop(code.r14);
  code.pop(code.r13);
  code.pop(code.r12);
  code.ret();
  code.ready();
}

void emit_x64_block(X64NativeContext &context, const DecodedBlock &block,
                    uintptr_t prepare_fn, uintptr_t memory_fn,
                    uintptr_t finish_fn) {
  if (context.uses_instruction_helpers) {
    emit_x64_helper_block(context, block, prepare_fn, memory_fn, finish_fn);
  } else {
    emit_x64_direct_block(context, block);
  }
}
#endif
} // namespace

bool CpuOptimizedBackend::x64_jit_available() const {
  return VIBESTATION_X64_JIT_SUPPORTED != 0;
}

bool CpuOptimizedBackend::x64_native_prepare_instruction(
    void *context_ptr, u32 index, CpuBlockRunResult *result) {
  auto *context = static_cast<X64NativeContext *>(context_ptr);
  if (context == nullptr || context->backend == nullptr ||
      context->block == nullptr || result == nullptr ||
      index >= context->block->instruction_count) {
    if (result != nullptr) {
      result->exit_reason = CpuBlockExitReason::Fallback;
    }
    return false;
  }

  if (result->cycles >= context->max_cycles ||
      result->instructions >= context->max_instructions ||
      context->block->invalidated) {
    result->exit_reason = context->block->invalidated
                              ? CpuBlockExitReason::Invalidated
                              : CpuBlockExitReason::Budget;
    return false;
  }

  return context->backend->prepare_instruction(context->block->instructions[index],
                                               *result);
}

bool CpuOptimizedBackend::x64_native_memory_instruction(void *context_ptr,
                                                        u32 op_value,
                                                        u32 rt_or_value,
                                                        u32 addr) {
  auto *context = static_cast<X64NativeContext *>(context_ptr);
  if (context == nullptr || context->backend == nullptr ||
      context->cpu == nullptr) {
    return false;
  }

  CpuOptimizedBackend &backend = *context->backend;
  Cpu &cpu = *context->cpu;
  const DecodedOp op = static_cast<DecodedOp>(op_value);

  ++backend.stats_.memory_helper_calls;
  ++backend.stats_.native_memory_helper_calls;
  if (backend.is_mmio_address(addr)) {
    ++backend.stats_.mmio_accesses;
  }

  switch (op) {
  case DecodedOp::Lh:
  case DecodedOp::Lhu:
  case DecodedOp::Sh:
    if ((addr & 1u) != 0) {
      ++backend.stats_.native_reject_unaligned;
    }
    break;
  case DecodedOp::Lw:
  case DecodedOp::Sw:
    if ((addr & 3u) != 0) {
      ++backend.stats_.native_reject_unaligned;
    }
    break;
  default:
    break;
  }

  switch (op) {
  case DecodedOp::Lb: {
    const u8 value = cpu.load8(addr);
    cpu.schedule_load(rt_or_value, static_cast<u32>(sign_extend_8(value)));
    break;
  }
  case DecodedOp::Lbu:
    cpu.schedule_load(rt_or_value, cpu.load8(addr));
    break;
  case DecodedOp::Lh: {
    const u16 value = cpu.load16(addr);
    if (!cpu.exception_raised_) {
      cpu.schedule_load(rt_or_value, static_cast<u32>(sign_extend_16(value)));
    }
    break;
  }
  case DecodedOp::Lhu: {
    const u16 value = cpu.load16(addr);
    if (!cpu.exception_raised_) {
      cpu.schedule_load(rt_or_value, value);
    }
    break;
  }
  case DecodedOp::Lw: {
    const u32 value = cpu.load32(addr);
    if (!cpu.exception_raised_) {
      cpu.schedule_load(rt_or_value, value);
    }
    break;
  }
  case DecodedOp::Sb:
    cpu.store8(addr, static_cast<u8>(rt_or_value));
    break;
  case DecodedOp::Sh:
    cpu.store16(addr, static_cast<u16>(rt_or_value));
    break;
  case DecodedOp::Sw:
    cpu.store32(addr, rt_or_value);
    break;
  default:
    return false;
  }

  return true;
}

bool CpuOptimizedBackend::x64_native_finish_instruction(
    void *context_ptr, u32 index, CpuBlockRunResult *result,
    u32 memory_instruction) {
  auto *context = static_cast<X64NativeContext *>(context_ptr);
  if (context == nullptr || context->backend == nullptr ||
      context->cpu == nullptr || context->block == nullptr ||
      result == nullptr || index >= context->block->instruction_count) {
    if (result != nullptr) {
      result->exit_reason = CpuBlockExitReason::Fallback;
    }
    return false;
  }

  Cpu &cpu = *context->cpu;
  context->backend->finish_instruction(context->block->instructions[index],
                                       *result, false);

  if (cpu.exception_raised_) {
    result->exit_reason = CpuBlockExitReason::Exception;
    if (memory_instruction != 0) {
      ++context->backend->stats_.native_memory_exception_exits;
    }
    return false;
  }

  if (context->block->invalidated) {
    result->exit_reason = CpuBlockExitReason::Invalidated;
    return false;
  }

  if (index + 1u < context->block->instruction_count &&
      (result->cycles >= context->max_cycles ||
       result->instructions >= context->max_instructions)) {
    result->exit_reason = CpuBlockExitReason::Budget;
    return false;
  }

  return true;
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
      if (!is_x64_memory_op(inst.op)) {
        return NativeBlockRejectReason::Memory;
      }
      continue;
    }
    if (inst.op == DecodedOp::Cop0) {
      return NativeBlockRejectReason::Cop0;
    }
    if (inst.op == DecodedOp::Cop2) {
      return NativeBlockRejectReason::Cop2;
    }
    if (inst.must_fallback || inst.may_raise_exception ||
        !is_x64_stage2_op(inst.op)) {
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
    ++stats_.native_rejected_block_count;
    stats_.native_rejected_block_instructions += block.instruction_count;
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
    auto context = std::make_unique<X64NativeContext>(8192);
    context->backend = this;
    context->cpu = &cpu_;
    context->block = &block;
    context->gpr = cpu_.gpr_;
    context->pc = &cpu_.pc_;
    context->next_pc = &cpu_.next_pc_;
    context->current_pc = &cpu_.current_pc_;
    context->cycles = &cpu_.cycles_;
    context->instruction_count = block.instruction_count;
    context->uses_instruction_helpers = block_uses_instruction_helpers(block);
    context->end_pc = block.start_pc + block.instruction_count * 4u;
    context->end_next_pc = context->end_pc + 4u;
    context->last_pc = context->end_pc - 4u;
    for (u32 i = 0; i < block.instruction_count; ++i) {
      context->base_cycles += block.instructions[i].cycles;
    }

    emit_x64_block(
        *context, block,
        reinterpret_cast<uintptr_t>(
            &CpuOptimizedBackend::x64_native_prepare_instruction),
        reinterpret_cast<uintptr_t>(
            &CpuOptimizedBackend::x64_native_memory_instruction),
        reinterpret_cast<uintptr_t>(
            &CpuOptimizedBackend::x64_native_finish_instruction));
    block.native_fn = context->code.getCode<DecodedBlock::NativeFn>();
    block.native_code_bytes = context->code.getSize();
    block.native_context = context.release();
    block.destroy_native_context = destroy_x64_native_context;
    ++stats_.native_compile_successes;
    ++stats_.native_blocks_compiled;
    stats_.native_code_bytes += block.native_code_bytes;
    return true;
  } catch (const std::exception &e) {
    ++stats_.native_compile_failures;
    LOG_WARN("CPU x64 JIT compile failed at pc=0x%08X: %s", block.start_pc,
             e.what());
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
  auto record_rejected_block = [&](NativeBlockRejectDetail detail) {
    ++stats_.native_rejected_block_count;
    stats_.native_rejected_block_instructions += block.instruction_count;
    record_native_block_rejection(block, detail);
  };
  auto reject_to_decoded = [&](u64 &specific_counter,
                               NativeBlockRejectDetail detail) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_unsafe_state;
    ++specific_counter;
    record_rejected_block(detail);
    return execute_block(block, max_cycles, max_instructions);
  };

  if (block.invalidated) {
    ++stats_.native_reject_invalidated_state;
    ++stats_.native_reject_stale_invalid_block_state;
    record_rejected_block(NativeBlockRejectDetail::InvalidatedState);
    result.exit_reason = CpuBlockExitReason::Invalidated;
    return result;
  }

  if (block.native_fn == nullptr || block.native_context == nullptr) {
    ++stats_.native_reject_stale_invalid_block_state;
    return reject_to_decoded(stats_.native_reject_other_state,
                             NativeBlockRejectDetail::StaleInvalidBlockState);
  }

  auto *context = static_cast<X64NativeContext *>(block.native_context);
  if (block.instruction_count > max_instructions) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_budget;
    record_rejected_block(NativeBlockRejectDetail::Budget);
    return execute_block(block, max_cycles, max_instructions);
  }
  if (cpu_.pc_ != block.start_pc || cpu_.next_pc_ != block.start_pc + 4u) {
    const NativeBlockRejectDetail detail =
        classify_native_pc_state_reject(block);
    record_native_pc_state_subreason(detail);
    return reject_to_decoded(stats_.native_reject_pc_state, detail);
  }
  if (cpu_.pending_delay_slot_ || cpu_.in_delay_slot_ ||
      cpu_.pending_branch_taken_ || cpu_.pending_branch_pc_ != 0u) {
    const NativeBlockRejectDetail detail =
        record_native_branch_delay_subreasons();
    return reject_to_decoded(stats_.native_reject_branch_delay_state, detail);
  }

  const bool active_load_delay =
      cpu_.load_.reg != 0 || cpu_.next_load_.reg != 0;
  if (active_load_delay && !context->uses_instruction_helpers) {
    ++stats_.native_helper_load_delay_fallbacks;
    return reject_to_decoded(stats_.native_reject_load_delay_state,
                             NativeBlockRejectDetail::LoadDelayState);
  }

  if (cpu_.sys_->irq_pending()) {
    cpu_.cop0_cause_ |= (1u << 10);
  } else {
    cpu_.cop0_cause_ &= ~(1u << 10);
  }
  if (cpu_.check_irq()) {
    return reject_to_decoded(stats_.native_reject_irq_state,
                             NativeBlockRejectDetail::IrqState);
  }

  if (active_load_delay && context->uses_instruction_helpers) {
    ++stats_.native_helper_load_delay_entries;
  }

  context->max_cycles = max_cycles;
  context->max_instructions = max_instructions;

  if (!context->uses_instruction_helpers) {
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
      record_rejected_block(NativeBlockRejectDetail::ICache);
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
        record_rejected_block(NativeBlockRejectDetail::ICache);
        return execute_block(block, max_cycles, max_instructions);
      }
      fills[fill_count++] = {index, tag, pc & ~0x0Fu};
    }
  }

  const u32 total_cycles = context->base_cycles + fetch_penalty;
  if (total_cycles > max_cycles) {
    ++stats_.native_to_decoded_fallbacks;
    ++stats_.native_reject_budget;
    record_rejected_block(NativeBlockRejectDetail::Budget);
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
  } else {
    context->cycles_to_add = 0;
  }

  block.native_fn(block.native_context, &result);

  if (active_load_delay && context->uses_instruction_helpers) {
    if (result.instructions != 0) {
      ++stats_.native_helper_load_delay_passes;
    } else {
      ++stats_.native_helper_load_delay_fallbacks;
    }
  }

  cpu_.cycle_penalty_ = 0;
  cpu_.executing_step_ = false;
  if (result.exit_reason != CpuBlockExitReason::Exception) {
    cpu_.exception_raised_ = false;
  }
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
