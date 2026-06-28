#include "cpu_recompiler.h"
#include "system.h"
#include <algorithm>
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
  u32 *load_reg = nullptr;
  u32 *load_value = nullptr;
  u32 *next_load_reg = nullptr;
  u32 *next_load_value = nullptr;
  u32 *cycle_penalty = nullptr;
  const u8 *ram_data = nullptr;
  u64 *ram_load_fastpath_counter = nullptr;
  u64 *branch_tail_ram_load_fastpath_counter = nullptr;
  u32 end_pc = 0;
  u32 end_next_pc = 0;
  u32 last_pc = 0;
  u32 instruction_count = 0;
  u32 base_cycles = 0;
  u32 cycles_to_add = 0;
  u32 max_cycles = 0;
  u32 max_instructions = 0;
  u32 current_instruction_index = 0;
  u32 branch_instruction_index = 0;
  bool uses_instruction_helpers = false;
  bool is_branch_tail = false;
  bool branch_taken = false;
  bool delay_slot_used_memory = false;
  bool delay_slot_used_mmio = false;
  bool memory_filter_exit = false;
  bool memory_trace_current = false;
  bool ram_load_fastpath_enabled = false;
  u64 memory_trace_sequence = 0;
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
  case DecodedOp::Movz:
  case DecodedOp::Movn:
  case DecodedOp::Sync:
  case DecodedOp::Clear:
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

bool is_x64_load_op(DecodedOp op) {
  return op == DecodedOp::Lb || op == DecodedOp::Lh ||
         op == DecodedOp::Lw || op == DecodedOp::Lbu ||
         op == DecodedOp::Lhu;
}

const char *x64_memory_op_name(DecodedOp op) {
  switch (op) {
  case DecodedOp::Lb: return "LB";
  case DecodedOp::Lh: return "LH";
  case DecodedOp::Lw: return "LW";
  case DecodedOp::Lbu: return "LBU";
  case DecodedOp::Lhu: return "LHU";
  case DecodedOp::Sb: return "SB";
  case DecodedOp::Sh: return "SH";
  case DecodedOp::Sw: return "SW";
  default: return "UNKNOWN";
  }
}

const char *x64_memory_region(u32 addr) {
  return native_memory_region_name(classify_native_memory_region(addr));
}

bool is_x64_stage2_op(DecodedOp op) {
  return is_x64_stage1_op(op) || is_x64_memory_op(op);
}

bool is_x64_branch_tail_op(DecodedOp op) {
  return op == DecodedOp::Bne || op == DecodedOp::Beq ||
         op == DecodedOp::Bgtz || op == DecodedOp::Blez ||
         op == DecodedOp::Bltz || op == DecodedOp::Bgez;
}

u8 x64_native_gpr_write_reg(const DecodedInstruction &inst) {
  switch (inst.op) {
  case DecodedOp::Sll:
  case DecodedOp::Srl:
  case DecodedOp::Sra:
  case DecodedOp::Sllv:
  case DecodedOp::Srlv:
  case DecodedOp::Srav:
  case DecodedOp::Movz:
  case DecodedOp::Movn:
  case DecodedOp::Clear:
  case DecodedOp::Addu:
  case DecodedOp::Subu:
  case DecodedOp::And:
  case DecodedOp::Or:
  case DecodedOp::Xor:
  case DecodedOp::Nor:
  case DecodedOp::Slt:
  case DecodedOp::Sltu:
    return inst.rd;
  case DecodedOp::Addiu:
  case DecodedOp::Slti:
  case DecodedOp::Sltiu:
  case DecodedOp::Andi:
  case DecodedOp::Ori:
  case DecodedOp::Xori:
  case DecodedOp::Lui:
    return inst.rt;
  default:
    return 0;
  }
}

bool x64_native_conditional_gpr_write(const DecodedInstruction &inst) {
  return inst.op == DecodedOp::Movz || inst.op == DecodedOp::Movn;
}

bool is_x64_reduced_helper_ram_load_block(
    const DecodedBlock &block, NativeBlockRejectDetail &reject_detail) {
  reject_detail = NativeBlockRejectDetail::None;
  if (!block.has_load) {
    return false;
  }
  if (block.has_store) {
    reject_detail = NativeBlockRejectDetail::ReducedHelperStore;
    return false;
  }
  if (block.has_control_flow || block.has_fallback) {
    reject_detail = NativeBlockRejectDetail::ReducedHelperUnsupportedMemory;
    return false;
  }

  std::array<bool, 32> written{};
  for (u32 i = 0; i < block.instruction_count; ++i) {
    const DecodedInstruction &inst = block.instructions[i];
    if (is_x64_load_op(inst.op)) {
      if (inst.rs != 0u && written[inst.rs]) {
        reject_detail = NativeBlockRejectDetail::ReducedHelperLoadBaseWritten;
        return false;
      }
      if (inst.rt != 0u) {
        written[inst.rt] = true;
      }
      continue;
    }
    if (inst.may_access_memory || !is_x64_stage1_op(inst.op) ||
        inst.must_fallback || inst.may_raise_exception) {
      reject_detail = NativeBlockRejectDetail::ReducedHelperUnsupportedMemory;
      return false;
    }
    const u8 write_reg = x64_native_gpr_write_reg(inst);
    if (write_reg != 0u) {
      written[write_reg] = true;
    }
  }
  return true;
}

bool is_canonical_ram_alias(u32 addr) {
  const u32 segment = addr & 0xE0000000u;
  if (segment != 0u && segment != 0x80000000u &&
      segment != 0xA0000000u) {
    return false;
  }
  return (addr & 0x1FFFFFFFu) < psx::RAM_SIZE;
}

bool is_x64_branch_tail_block(const DecodedBlock &block,
                              NativeBlockRejectDetail &reject_detail) {
  reject_detail = NativeBlockRejectDetail::None;
  if (!block.has_control_flow) {
    return false;
  }
  if (block.instruction_count < 2u) {
    reject_detail = NativeBlockRejectDetail::BranchTailMissingDelaySlot;
    return false;
  }

  const u32 branch_index = block.instruction_count - 2u;
  const DecodedInstruction &branch = block.instructions[branch_index];
  const DecodedInstruction &delay = block.instructions[branch_index + 1u];
  if (!branch.is_branch) {
    reject_detail = NativeBlockRejectDetail::BranchTailMissingBranch;
    return false;
  }
  if (!is_x64_branch_tail_op(branch.op)) {
    reject_detail = NativeBlockRejectDetail::BranchTailUnsupportedBranch;
    return false;
  }
  if (delay.pc != branch.pc + 4u) {
    reject_detail = NativeBlockRejectDetail::BranchTailMissingDelaySlot;
    return false;
  }

  for (u32 i = 0; i < branch_index; ++i) {
    const DecodedInstruction &inst = block.instructions[i];
    if (inst.is_branch || inst.must_fallback ||
        (!is_x64_stage2_op(inst.op) && !inst.may_access_memory)) {
      reject_detail = NativeBlockRejectDetail::BranchTailUnsupportedBody;
      return false;
    }
    if (inst.may_access_memory && !is_x64_memory_op(inst.op)) {
      reject_detail = NativeBlockRejectDetail::BranchTailUnsupportedBody;
      return false;
    }
    if (inst.may_raise_exception && !is_x64_memory_op(inst.op)) {
      reject_detail = NativeBlockRejectDetail::BranchTailUnsupportedBody;
      return false;
    }
  }

  if (delay.is_branch) {
    reject_detail = NativeBlockRejectDetail::BranchTailNestedDelayBranch;
    return false;
  }
  if (delay.must_fallback ||
      (!is_x64_stage2_op(delay.op) && !delay.may_access_memory) ||
      (delay.may_access_memory && !is_x64_memory_op(delay.op)) ||
      (delay.may_raise_exception && !is_x64_memory_op(delay.op))) {
    reject_detail = NativeBlockRejectDetail::BranchTailUnsupportedDelaySlot;
    return false;
  }
  return true;
}

bool block_uses_instruction_helpers(const DecodedBlock &block) {
  for (u32 i = 0; i < block.instruction_count; ++i) {
    if (is_x64_memory_op(block.instructions[i].op)) {
      return true;
    }
  }
  return false;
}

NativeBlockShape classify_x64_block_shape(const DecodedBlock &block) {
  bool contains_cop0 = false;
  bool contains_cop2 = false;
  bool contains_unsafe = block.has_fallback;
  for (u32 i = 0; i < block.instruction_count; ++i) {
    const DecodedInstruction &inst = block.instructions[i];
    contains_cop0 = contains_cop0 || inst.op == DecodedOp::Cop0;
    contains_cop2 = contains_cop2 || inst.op == DecodedOp::Cop2;
    contains_unsafe = contains_unsafe || inst.must_fallback ||
                      (inst.may_raise_exception && !inst.may_access_memory);
  }
  if (contains_cop0) {
    return NativeBlockShape::Cop0;
  }
  if (contains_cop2) {
    return NativeBlockShape::Cop2;
  }
  if (block.has_control_flow) {
    return block.native_branch_tail ? NativeBlockShape::BranchTail
                                    : NativeBlockShape::FallbackControl;
  }
  if (contains_unsafe) {
    return NativeBlockShape::ExceptionUnsafe;
  }
  if (block.has_store) {
    return NativeBlockShape::StraightAluRamStoreCandidate;
  }
  if (block.has_load) {
    return NativeBlockShape::StraightAluRamLoadCandidate;
  }
  if (block.has_memory) {
    return NativeBlockShape::HelperSafeMemory;
  }
  return NativeBlockShape::StraightAlu;
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

void emit_write_gpr(Xbyak::CodeGenerator &code, const Xbyak::Reg64 &ctx,
                    const Xbyak::Reg64 &gpr, u8 index,
                    const Xbyak::Reg32 &src) {
  if (index != 0) {
    Xbyak::Label no_cancel;
    code.mov(code.r9,
             code.ptr[ctx + offsetof(X64NativeContext, load_reg)]);
    code.cmp(code.dword[code.r9], static_cast<u32>(index));
    code.jne(no_cancel, Xbyak::CodeGenerator::T_NEAR);
    code.mov(code.dword[code.r9], 0u);
    code.L(no_cancel);
    code.mov(code.dword[gpr + static_cast<int>(index) * 4], src);
  }
}

void emit_mov_imm32(Xbyak::CodeGenerator &code, const Xbyak::Reg32 &dst,
                    u32 value) {
  code.mov(dst, value);
}

void emit_x64_instruction(Xbyak::CodeGenerator &code,
                          const DecodedInstruction &inst,
                          const Xbyak::Reg64 &ctx,
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
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Srl:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    if (inst.shamt != 0) {
      code.shr(code.eax, inst.shamt);
    }
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Sra:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    if (inst.shamt != 0) {
      code.sar(code.eax, inst.shamt);
    }
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Sllv:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    emit_read_gpr(code, code.ecx, gpr, inst.rs);
    code.and_(code.ecx, 0x1F);
    code.shl(code.eax, code.cl);
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Srlv:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    emit_read_gpr(code, code.ecx, gpr, inst.rs);
    code.and_(code.ecx, 0x1F);
    code.shr(code.eax, code.cl);
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Srav:
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    emit_read_gpr(code, code.ecx, gpr, inst.rs);
    code.and_(code.ecx, 0x1F);
    code.sar(code.eax, code.cl);
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Movz: {
    Label skip_write;
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    code.test(code.eax, code.eax);
    code.jnz(skip_write, CodeGenerator::T_NEAR);
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    code.L(skip_write);
    break;
  }
  case DecodedOp::Movn: {
    Label skip_write;
    emit_read_gpr(code, code.eax, gpr, inst.rt);
    code.test(code.eax, code.eax);
    code.jz(skip_write, CodeGenerator::T_NEAR);
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    code.L(skip_write);
    break;
  }
  case DecodedOp::Sync:
    break;
  case DecodedOp::Clear:
    code.xor_(code.eax, code.eax);
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Addu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.add(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Subu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.sub(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::And:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt == 0) {
      code.xor_(code.eax, code.eax);
    } else {
      code.and_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Or:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.or_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Xor:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.xor_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Nor:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.rt != 0) {
      code.or_(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
    }
    code.not_(code.eax);
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
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
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
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
    emit_write_gpr(code, ctx, gpr, inst.rd, code.eax);
    break;
  case DecodedOp::Addiu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.simm != 0) {
      code.add(code.eax, static_cast<u32>(inst.simm));
    }
    emit_write_gpr(code, ctx, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Slti:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    emit_mov_imm32(code, code.r10d, static_cast<u32>(inst.simm));
    code.cmp(code.eax, code.r10d);
    code.setl(code.al);
    code.movzx(code.eax, code.al);
    emit_write_gpr(code, ctx, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Sltiu:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    emit_mov_imm32(code, code.r10d, static_cast<u32>(inst.simm));
    code.cmp(code.eax, code.r10d);
    code.setb(code.al);
    code.movzx(code.eax, code.al);
    emit_write_gpr(code, ctx, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Andi:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    code.and_(code.eax, static_cast<u32>(inst.imm));
    emit_write_gpr(code, ctx, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Ori:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.imm != 0) {
      code.or_(code.eax, static_cast<u32>(inst.imm));
    }
    emit_write_gpr(code, ctx, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Xori:
    emit_read_gpr(code, code.eax, gpr, inst.rs);
    if (inst.imm != 0) {
      code.xor_(code.eax, static_cast<u32>(inst.imm));
    }
    emit_write_gpr(code, ctx, gpr, inst.rt, code.eax);
    break;
  case DecodedOp::Lui:
    emit_mov_imm32(code, code.eax, static_cast<u32>(inst.imm) << 16);
    emit_write_gpr(code, ctx, gpr, inst.rt, code.eax);
    break;
  default:
    break;
  }
}

void emit_advance_load_delay(Xbyak::CodeGenerator &code,
                             const Xbyak::Reg64 &ctx,
                             const Xbyak::Reg64 &gpr) {
  using namespace Xbyak;
  Label no_commit;

  code.mov(code.r9,
           code.ptr[ctx + offsetof(X64NativeContext, load_reg)]);
  code.mov(code.r10d, code.dword[code.r9]);
  code.test(code.r10d, code.r10d);
  code.jz(no_commit, CodeGenerator::T_NEAR);
  code.mov(code.rax,
           code.ptr[ctx + offsetof(X64NativeContext, load_value)]);
  code.mov(code.eax, code.dword[code.rax]);
  code.mov(code.dword[gpr + code.r10 * 4], code.eax);
  code.L(no_commit);

  code.mov(code.rax,
           code.ptr[ctx + offsetof(X64NativeContext, next_load_reg)]);
  code.mov(code.r10d, code.dword[code.rax]);
  code.mov(code.dword[code.r9], code.r10d);

  code.mov(code.r9,
           code.ptr[ctx + offsetof(X64NativeContext, load_value)]);
  code.mov(code.rax,
           code.ptr[ctx + offsetof(X64NativeContext, next_load_value)]);
  code.mov(code.r10d, code.dword[code.rax]);
  code.mov(code.dword[code.r9], code.r10d);

  code.mov(code.r9,
           code.ptr[ctx + offsetof(X64NativeContext, next_load_reg)]);
  code.mov(code.dword[code.r9], 0u);
  code.mov(code.r9,
           code.ptr[ctx + offsetof(X64NativeContext, next_load_value)]);
  code.mov(code.dword[code.r9], 0u);
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

void emit_ram_load_fastpath_or_memory_helper(
    Xbyak::CodeGenerator &code, const Xbyak::Reg64 &ctx,
    const Xbyak::Reg64 &gpr, const DecodedInstruction &inst,
    uintptr_t memory_fn, Xbyak::Label &block_done,
    bool count_branch_tail_fastpath_hits) {
  using namespace Xbyak;
  Label slow, mapped, completed;

  code.cmp(code.byte[ctx + offsetof(X64NativeContext,
                                    ram_load_fastpath_enabled)],
           0);
  code.je(slow, CodeGenerator::T_NEAR);

  emit_read_gpr(code, code.r10d, gpr, inst.rs);
  if (inst.simm != 0) {
    code.add(code.r10d, static_cast<u32>(inst.simm));
  }
  if (inst.op == DecodedOp::Lh || inst.op == DecodedOp::Lhu) {
    code.test(code.r10d, 1u);
    code.jnz(slow, CodeGenerator::T_NEAR);
  } else if (inst.op == DecodedOp::Lw) {
    code.test(code.r10d, 3u);
    code.jnz(slow, CodeGenerator::T_NEAR);
  }

  // Accept only the canonical 2 MiB KUSEG/KSEG0/KSEG1 RAM aliases. KSEG2,
  // upper RAM-window mirrors, scratchpad, BIOS, and I/O all stay slow.
  code.mov(code.eax, code.r10d);
  code.and_(code.eax, 0x1FFFFFFFu);
  code.cmp(code.eax, psx::RAM_SIZE);
  code.jae(slow, CodeGenerator::T_NEAR);
  code.mov(code.edx, code.r10d);
  code.and_(code.edx, 0xE0000000u);
  code.cmp(code.edx, 0u);
  code.je(mapped, CodeGenerator::T_NEAR);
  code.cmp(code.edx, 0x80000000u);
  code.je(mapped, CodeGenerator::T_NEAR);
  code.cmp(code.edx, 0xA0000000u);
  code.jne(slow, CodeGenerator::T_NEAR);

  code.L(mapped);
  code.mov(code.r11, code.ptr[ctx + offsetof(X64NativeContext, ram_data)]);
  switch (inst.op) {
  case DecodedOp::Lb:
    code.movsx(code.edx, code.byte[code.r11 + code.rax]);
    break;
  case DecodedOp::Lbu:
    code.movzx(code.edx, code.byte[code.r11 + code.rax]);
    break;
  case DecodedOp::Lh:
    code.movsx(code.edx, code.word[code.r11 + code.rax]);
    break;
  case DecodedOp::Lhu:
    code.movzx(code.edx, code.word[code.r11 + code.rax]);
    break;
  case DecodedOp::Lw:
    code.mov(code.edx, code.dword[code.r11 + code.rax]);
    break;
  default:
    code.jmp(slow, CodeGenerator::T_NEAR);
    break;
  }

  code.mov(code.rax,
           code.ptr[ctx + offsetof(X64NativeContext, cycle_penalty)]);
  code.add(code.dword[code.rax], 4u);
  code.mov(code.rax,
           code.ptr[ctx +
                     offsetof(X64NativeContext, ram_load_fastpath_counter)]);
  code.inc(code.qword[code.rax]);
  if (count_branch_tail_fastpath_hits) {
    code.mov(
        code.rax,
        code.ptr[ctx + offsetof(X64NativeContext,
                                branch_tail_ram_load_fastpath_counter)]);
    code.inc(code.qword[code.rax]);
  }

  if (inst.rt == 0u) {
    code.mov(code.rax,
             code.ptr[ctx + offsetof(X64NativeContext, next_load_reg)]);
    code.mov(code.dword[code.rax], 0u);
    code.mov(code.rax,
             code.ptr[ctx + offsetof(X64NativeContext, next_load_value)]);
    code.mov(code.dword[code.rax], 0u);
  } else {
    code.mov(code.rax, code.ptr[ctx + offsetof(X64NativeContext, load_reg)]);
    code.cmp(code.dword[code.rax], static_cast<u32>(inst.rt));
    Label no_cancel;
    code.jne(no_cancel, CodeGenerator::T_NEAR);
    code.mov(code.dword[code.rax], 0u);
    code.L(no_cancel);
    code.mov(code.rax,
             code.ptr[ctx + offsetof(X64NativeContext, next_load_reg)]);
    code.mov(code.dword[code.rax], static_cast<u32>(inst.rt));
    code.mov(code.rax,
             code.ptr[ctx + offsetof(X64NativeContext, next_load_value)]);
    code.mov(code.dword[code.rax], code.edx);
  }
  code.jmp(completed, CodeGenerator::T_NEAR);

  code.L(slow);
  emit_memory_helper_call(code, ctx, gpr, inst, memory_fn);
  code.test(code.al, code.al);
  code.jz(block_done, CodeGenerator::T_NEAR);
  code.L(completed);
}

void emit_reduced_helper_ram_load(Xbyak::CodeGenerator &code,
                                  const Xbyak::Reg64 &ctx,
                                  const Xbyak::Reg64 &gpr,
                                  const DecodedInstruction &inst) {
  emit_read_gpr(code, code.r10d, gpr, inst.rs);
  if (inst.simm != 0) {
    code.add(code.r10d, static_cast<u32>(inst.simm));
  }
  code.mov(code.eax, code.r10d);
  code.and_(code.eax, 0x1FFFFFFFu);
  code.mov(code.r9, code.ptr[ctx + offsetof(X64NativeContext, ram_data)]);
  switch (inst.op) {
  case DecodedOp::Lb:
    code.movsx(code.r10d, code.byte[code.r9 + code.rax]);
    break;
  case DecodedOp::Lbu:
    code.movzx(code.r10d, code.byte[code.r9 + code.rax]);
    break;
  case DecodedOp::Lh:
    code.movsx(code.r10d, code.word[code.r9 + code.rax]);
    break;
  case DecodedOp::Lhu:
    code.movzx(code.r10d, code.word[code.r9 + code.rax]);
    break;
  case DecodedOp::Lw:
    code.mov(code.r10d, code.dword[code.r9 + code.rax]);
    break;
  default:
    code.xor_(code.r10d, code.r10d);
    break;
  }

  code.mov(code.rax,
           code.ptr[ctx +
                    offsetof(X64NativeContext, ram_load_fastpath_counter)]);
  code.inc(code.qword[code.rax]);

  if (inst.rt == 0u) {
    code.mov(code.rax,
             code.ptr[ctx + offsetof(X64NativeContext, next_load_reg)]);
    code.mov(code.dword[code.rax], 0u);
    code.mov(code.rax,
             code.ptr[ctx + offsetof(X64NativeContext, next_load_value)]);
    code.mov(code.dword[code.rax], 0u);
    return;
  }

  code.mov(code.rax, code.ptr[ctx + offsetof(X64NativeContext, load_reg)]);
  code.cmp(code.dword[code.rax], static_cast<u32>(inst.rt));
  Xbyak::Label no_cancel;
  code.jne(no_cancel, Xbyak::CodeGenerator::T_NEAR);
  code.mov(code.dword[code.rax], 0u);
  code.L(no_cancel);
  code.mov(code.rax,
           code.ptr[ctx + offsetof(X64NativeContext, next_load_reg)]);
  code.mov(code.dword[code.rax], static_cast<u32>(inst.rt));
  code.mov(code.rax,
           code.ptr[ctx + offsetof(X64NativeContext, next_load_value)]);
  code.mov(code.dword[code.rax], code.r10d);
}

void emit_branch_helper_call(Xbyak::CodeGenerator &code,
                             const Xbyak::Reg64 &ctx,
                             const Xbyak::Reg64 &gpr,
                             const DecodedInstruction &inst,
                             uintptr_t branch_fn) {
  emit_read_gpr(code, code.eax, gpr, inst.rs);
  if (inst.op == DecodedOp::Bgtz || inst.op == DecodedOp::Blez ||
      inst.op == DecodedOp::Bltz || inst.op == DecodedOp::Bgez) {
    code.cmp(code.eax, 0);
  } else if (inst.rt == 0) {
    code.cmp(code.eax, 0);
  } else {
    code.cmp(code.eax, code.dword[gpr + static_cast<int>(inst.rt) * 4]);
  }
  if (inst.op == DecodedOp::Bne) {
    code.setne(code.al);
  } else if (inst.op == DecodedOp::Beq) {
    code.sete(code.al);
  } else if (inst.op == DecodedOp::Bgtz) {
    code.setg(code.al);
  } else if (inst.op == DecodedOp::Bltz) {
    code.setl(code.al);
  } else if (inst.op == DecodedOp::Bgez) {
    code.setge(code.al);
  } else {
    code.setle(code.al);
  }
  code.movzx(code.r10d, code.al);
#if defined(_WIN32)
  code.mov(code.rcx, ctx);
  code.mov(code.edx, code.r10d);
#else
  code.mov(code.rdi, ctx);
  code.mov(code.esi, code.r10d);
#endif
  emit_absolute_call(code, branch_fn);
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
    const DecodedInstruction &inst = block.instructions[i];
    if (block.native_reduced_helper_ram_load && is_x64_load_op(inst.op)) {
      emit_reduced_helper_ram_load(code, ctx, gpr, inst);
    } else {
      emit_x64_instruction(code, inst, ctx, gpr);
    }
    emit_advance_load_delay(code, ctx, gpr);
    code.mov(code.dword[gpr], 0);
  }

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
                           uintptr_t memory_fn, uintptr_t branch_fn,
                           uintptr_t finish_fn) {
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

    if (is_x64_branch_tail_op(inst.op)) {
      emit_branch_helper_call(code, ctx, gpr, inst, branch_fn);
    } else if (memory_instruction) {
      if (is_x64_load_op(inst.op)) {
        emit_ram_load_fastpath_or_memory_helper(code, ctx, gpr, inst,
                                                memory_fn, done,
                                                context.is_branch_tail);
      } else {
        emit_memory_helper_call(code, ctx, gpr, inst, memory_fn);
        code.test(code.al, code.al);
        code.jz(done, CodeGenerator::T_NEAR);
      }
    } else {
      emit_x64_instruction(code, inst, ctx, gpr);
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
                    uintptr_t branch_fn, uintptr_t finish_fn) {
  if (context.uses_instruction_helpers) {
    emit_x64_helper_block(context, block, prepare_fn, memory_fn, branch_fn,
                          finish_fn);
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

  ++context->backend->stats_.native_prepare_helper_calls;
  ++context->block->native_prepare_helper_call_count;
  if (context->is_branch_tail) {
    ++context->backend->stats_.native_branch_tail_prepare_helper_calls;
  }

  context->current_instruction_index = index;
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
  if (context->block == nullptr ||
      context->current_instruction_index >= context->block->instruction_count) {
    return false;
  }
  const DecodedInstruction &inst =
      context->block->instructions[context->current_instruction_index];
  ++backend.stats_.native_memory_helper_calls;
  ++context->block->native_memory_helper_call_count;
  if (context->is_branch_tail) {
    ++backend.stats_.native_branch_tail_memory_helper_calls;
  }
  const u32 expected_addr =
      cpu.gpr_[inst.rs] + static_cast<u32>(inst.simm);
  const u32 expected_arg =
      is_x64_load_op(inst.op) ? static_cast<u32>(inst.rt)
                              : cpu.gpr_[inst.rt];
  const bool load_op = is_x64_load_op(inst.op);
  CpuBackendStats &stats = backend.stats_;
  if (load_op) {
    ++stats.native_memory_helper_load_calls;
    if (context->is_branch_tail) {
      ++stats.native_branch_tail_memory_helper_load_calls;
    }
  } else {
    ++stats.native_memory_helper_store_calls;
    if (context->is_branch_tail) {
      ++stats.native_branch_tail_memory_helper_store_calls;
    }
  }
  const bool operands_match =
      op == inst.op && addr == expected_addr && rt_or_value == expected_arg;

  ++backend.stats_.native_memory_shared_decoded_calls;
  if (!operands_match) {
    ++backend.stats_.native_memory_operand_mismatches;
    LOG_WARN("CPU_NATIVE_MEMORY_OPERAND_MISMATCH block=0x%08X pc=0x%08X emitted_op=%u decoded_op=%u emitted_addr=0x%08X decoded_addr=0x%08X emitted_arg=0x%08X decoded_arg=0x%08X",
             context->block->start_pc, inst.pc, op_value,
             static_cast<u32>(inst.op), addr, expected_addr, rt_or_value,
             expected_arg);
  }
  if (context->is_branch_tail &&
      context->current_instruction_index ==
          context->branch_instruction_index + 1u) {
    ++backend.stats_.native_branch_delay_slot_memory_helpers;
    context->delay_slot_used_memory = true;
  }
  const NativeMemoryRegion region =
      classify_native_memory_region(expected_addr);
  switch (region) {
  case NativeMemoryRegion::Ram:
    ++backend.stats_.native_memory_helper_ram_calls;
    break;
  case NativeMemoryRegion::Scratchpad:
    ++backend.stats_.native_memory_helper_scratchpad_calls;
    break;
  case NativeMemoryRegion::BiosReadOnly:
    ++backend.stats_.native_memory_helper_bios_calls;
    break;
  case NativeMemoryRegion::Mmio:
    ++backend.stats_.native_memory_helper_mmio_calls;
    break;
  case NativeMemoryRegion::UnknownSlow:
  default:
    ++backend.stats_.native_memory_helper_unknown_calls;
    break;
  }
  const bool mmio = region == NativeMemoryRegion::Mmio;
  const u32 phys = psx::mask_address(expected_addr);
  const bool ram = phys < 0x00800000u;
  if (mmio) {
    if (context->is_branch_tail &&
        context->current_instruction_index ==
            context->branch_instruction_index + 1u) {
      context->delay_slot_used_mmio = true;
    }
  }

  if ((mmio && g_cpu_x64_jit_disable_native_mmio) ||
      (ram && g_cpu_x64_jit_disable_native_ram)) {
    context->block->native_memory_runtime_filter_reason = mmio ? 1u : 2u;
    context->memory_filter_exit = true;
  }

  const bool unaligned_load =
      load_op &&
      (((inst.op == DecodedOp::Lh || inst.op == DecodedOp::Lhu) &&
        (expected_addr & 1u) != 0u) ||
       (inst.op == DecodedOp::Lw && (expected_addr & 3u) != 0u));
  if (load_op) {
    const bool trace_blocks_fastpath =
        g_cpu_x64_jit_memory_trace || g_trace_ram || g_trace_bus;
    if (!context->ram_load_fastpath_enabled) {
      ++stats.native_memory_fastpath_load_misses;
      if (context->is_branch_tail) {
        ++stats.native_branch_tail_ram_load_fastpath_load_misses;
      }
      if (trace_blocks_fastpath) {
        ++stats.native_memory_fastpath_load_miss_trace;
        if (context->is_branch_tail) {
          ++stats.native_branch_tail_ram_load_fastpath_load_miss_trace;
        }
      } else {
        ++stats.native_memory_fastpath_load_miss_disabled;
        if (context->is_branch_tail) {
          ++stats.native_branch_tail_ram_load_fastpath_load_miss_disabled;
        }
      }
    } else if (unaligned_load) {
      ++stats.native_memory_fastpath_load_misses;
      ++stats.native_memory_fastpath_load_miss_unaligned;
      if (context->is_branch_tail) {
        ++stats.native_branch_tail_ram_load_fastpath_load_misses;
        ++stats.native_branch_tail_ram_load_fastpath_load_miss_unaligned;
      }
    } else if (!is_canonical_ram_alias(expected_addr)) {
      ++stats.native_memory_fastpath_load_misses;
      ++stats.native_memory_fastpath_load_miss_non_ram;
      if (context->is_branch_tail) {
        ++stats.native_branch_tail_ram_load_fastpath_load_misses;
        ++stats.native_branch_tail_ram_load_fastpath_load_miss_non_ram;
      }
    }
  }

  const bool trace =
      g_cpu_x64_jit_memory_trace &&
      backend.native_memory_trace_emitted_ <
          static_cast<u64>(std::max<u32>(1u,
                                        g_cpu_x64_jit_memory_trace_count)) &&
      (!g_cpu_x64_jit_memory_trace_pc_set ||
       g_cpu_x64_jit_memory_trace_pc == context->block->start_pc ||
       g_cpu_x64_jit_memory_trace_pc == inst.pc);
  u64 trace_sequence = 0;
  context->memory_trace_current = false;
  if (trace) {
    trace_sequence = ++backend.native_memory_trace_emitted_;
    context->memory_trace_current = true;
    context->memory_trace_sequence = trace_sequence;
    const CdRom &cd = cpu.sys_->cdrom();
    LOG_INFO("CPU_NATIVE_MEMORY_PRE seq=%llu block=0x%08X pc=0x%08X op=%s addr=0x%08X arg=0x%08X decoded_addr=0x%08X decoded_arg=0x%08X operands_match=%u region=%s current_pc=0x%08X cpu_pc=0x%08X next_pc=0x%08X executing=%u delay=%u pending_delay=%u pending_taken=%u pending_pc=0x%08X active_branch=0x%08X load=%u:0x%08X next_load=%u:0x%08X exception=%u penalty=%u cycles=%llu irq_stat=0x%08X irq_mask=0x%08X cd_sector=%llu cd_read_lba=%d cd_active_lba=%d cd_irq=%u cd_resp=%zu cd_data_index=%d cd_busy=%d dma_dpcr=0x%08X dma_dicr=0x%08X",
             static_cast<unsigned long long>(trace_sequence),
             context->block->start_pc, inst.pc, x64_memory_op_name(inst.op),
             addr, rt_or_value, expected_addr, expected_arg,
             operands_match ? 1u : 0u, x64_memory_region(expected_addr),
             cpu.current_pc_, cpu.pc_, cpu.next_pc_,
             cpu.executing_step_ ? 1u : 0u, cpu.in_delay_slot_ ? 1u : 0u,
             cpu.pending_delay_slot_ ? 1u : 0u,
             cpu.pending_branch_taken_ ? 1u : 0u, cpu.pending_branch_pc_,
             cpu.active_branch_pc_, cpu.load_.reg, cpu.load_.value,
             cpu.next_load_.reg, cpu.next_load_.value,
             cpu.exception_raised_ ? 1u : 0u, cpu.cycle_penalty_,
             static_cast<unsigned long long>(cpu.cycles_),
             cpu.sys_->irq().stat(), cpu.sys_->irq().mask(),
             static_cast<unsigned long long>(cd.sector_count()),
             cd.current_read_lba(), cd.active_data_lba(), cd.last_irq_code(),
             cd.response_fifo_size(), cd.dma_data_index(),
             cd.busy_cycles_remaining(), cpu.sys_->debug_dma_read(0x70u),
             cpu.sys_->debug_dma_read(0x74u));
  }

  switch (inst.op) {
  case DecodedOp::Lh:
  case DecodedOp::Lhu:
  case DecodedOp::Sh:
    if ((expected_addr & 1u) != 0) {
      ++backend.stats_.native_reject_unaligned;
      ++backend.stats_.native_memory_helper_unaligned_calls;
    }
    break;
  case DecodedOp::Lw:
  case DecodedOp::Sw:
    if ((expected_addr & 3u) != 0) {
      ++backend.stats_.native_reject_unaligned;
      ++backend.stats_.native_memory_helper_unaligned_calls;
    }
    break;
  default:
    break;
  }

  const bool executed = backend.execute_decoded_instruction(inst);
  if (!executed) {
    return false;
  }

  if (trace) {
    const CdRom &cd = cpu.sys_->cdrom();
    const bool load_result_valid =
        is_x64_load_op(inst.op) && cpu.next_load_.reg == inst.rt;
    const u32 result_value =
        load_result_valid ? cpu.next_load_.value : expected_arg;
    LOG_INFO("CPU_NATIVE_MEMORY_POST seq=%llu block=0x%08X pc=0x%08X op=%s addr=0x%08X result=0x%08X result_valid=%u region=%s current_pc=0x%08X cpu_pc=0x%08X next_pc=0x%08X executing=%u delay=%u pending_delay=%u pending_taken=%u pending_pc=0x%08X active_branch=0x%08X load=%u:0x%08X next_load=%u:0x%08X exception=%u penalty=%u cycles=%llu irq_stat=0x%08X irq_mask=0x%08X cd_sector=%llu cd_read_lba=%d cd_active_lba=%d cd_irq=%u cd_resp=%zu cd_data_index=%d cd_busy=%d dma_dpcr=0x%08X dma_dicr=0x%08X filter_exit=%u",
             static_cast<unsigned long long>(trace_sequence),
             context->block->start_pc, inst.pc, x64_memory_op_name(inst.op),
             expected_addr, result_value, load_result_valid ? 1u : 0u,
             x64_memory_region(expected_addr), cpu.current_pc_, cpu.pc_,
             cpu.next_pc_, cpu.executing_step_ ? 1u : 0u,
             cpu.in_delay_slot_ ? 1u : 0u,
             cpu.pending_delay_slot_ ? 1u : 0u,
             cpu.pending_branch_taken_ ? 1u : 0u, cpu.pending_branch_pc_,
             cpu.active_branch_pc_, cpu.load_.reg, cpu.load_.value,
             cpu.next_load_.reg, cpu.next_load_.value,
             cpu.exception_raised_ ? 1u : 0u, cpu.cycle_penalty_,
             static_cast<unsigned long long>(cpu.cycles_),
             cpu.sys_->irq().stat(), cpu.sys_->irq().mask(),
             static_cast<unsigned long long>(cd.sector_count()),
             cd.current_read_lba(), cd.active_data_lba(), cd.last_irq_code(),
             cd.response_fifo_size(), cd.dma_data_index(),
             cd.busy_cycles_remaining(), cpu.sys_->debug_dma_read(0x70u),
             cpu.sys_->debug_dma_read(0x74u),
             context->memory_filter_exit ? 1u : 0u);
  }

  return true;
}

void CpuOptimizedBackend::x64_native_branch_instruction(void *context_ptr,
                                                        u32 taken) {
  auto *context = static_cast<X64NativeContext *>(context_ptr);
  if (context == nullptr || context->backend == nullptr ||
      context->cpu == nullptr || context->block == nullptr ||
      !context->is_branch_tail) {
    return;
  }

  ++context->backend->stats_.native_branch_helper_calls;
  ++context->block->native_branch_helper_call_count;
  ++context->backend->stats_.native_branch_tail_branch_helper_calls;

  Cpu &cpu = *context->cpu;
  context->branch_taken = taken != 0u;
  const DecodedInstruction &branch =
      context->block->instructions[context->branch_instruction_index];
  cpu.begin_branch(context->branch_taken, branch.target);
  if (context->branch_taken) {
    ++context->backend->stats_.native_branch_taken;
  } else {
    ++context->backend->stats_.native_branch_not_taken;
  }
  if (branch.op == DecodedOp::Bgtz) {
    if (context->branch_taken) {
      ++context->backend->stats_.native_branch_tail_bgtz_taken;
    } else {
      ++context->backend->stats_.native_branch_tail_bgtz_not_taken;
    }
  } else if (branch.op == DecodedOp::Blez) {
    if (context->branch_taken) {
      ++context->backend->stats_.native_branch_tail_blez_taken;
    } else {
      ++context->backend->stats_.native_branch_tail_blez_not_taken;
    }
  }
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

  ++context->backend->stats_.native_finish_helper_calls;
  ++context->block->native_finish_helper_call_count;
  if (context->is_branch_tail) {
    ++context->backend->stats_.native_branch_tail_finish_helper_calls;
  }

  Cpu &cpu = *context->cpu;
  const DecodedInstruction &inst = context->block->instructions[index];
  const u8 write_reg = x64_native_gpr_write_reg(inst);
  if (write_reg != 0u && !x64_native_conditional_gpr_write(inst)) {
    // The native ALU instruction has already written its result. Reuse the
    // CPU register-write semantic helper so a write to the destination of a
    // pending load cancels that load exactly as decoded execution does.
    cpu.set_reg(write_reg, cpu.gpr_[write_reg]);
  }
  context->backend->finish_instruction(inst, *result, false);

  if (memory_instruction != 0u && context->memory_trace_current) {
    const CdRom &cd = cpu.sys_->cdrom();
    LOG_INFO("CPU_NATIVE_MEMORY_FINISH seq=%llu block=0x%08X pc=0x%08X current_pc=0x%08X cpu_pc=0x%08X next_pc=0x%08X executing=%u delay=%u pending_delay=%u pending_taken=%u pending_pc=0x%08X active_branch=0x%08X load=%u:0x%08X next_load=%u:0x%08X exception=%u penalty=%u cycles=%llu result_cycles=%u result_instructions=%u irq_stat=0x%08X irq_mask=0x%08X cd_sector=%llu cd_read_lba=%d cd_active_lba=%d cd_irq=%u cd_resp=%zu cd_data_index=%d cd_busy=%d dma_dpcr=0x%08X dma_dicr=0x%08X",
             static_cast<unsigned long long>(context->memory_trace_sequence),
             context->block->start_pc, inst.pc, cpu.current_pc_, cpu.pc_,
             cpu.next_pc_, cpu.executing_step_ ? 1u : 0u,
             cpu.in_delay_slot_ ? 1u : 0u,
             cpu.pending_delay_slot_ ? 1u : 0u,
             cpu.pending_branch_taken_ ? 1u : 0u, cpu.pending_branch_pc_,
             cpu.active_branch_pc_, cpu.load_.reg, cpu.load_.value,
             cpu.next_load_.reg, cpu.next_load_.value,
             cpu.exception_raised_ ? 1u : 0u, cpu.cycle_penalty_,
             static_cast<unsigned long long>(cpu.cycles_), result->cycles,
             result->instructions, cpu.sys_->irq().stat(),
             cpu.sys_->irq().mask(),
             static_cast<unsigned long long>(cd.sector_count()),
             cd.current_read_lba(), cd.active_data_lba(), cd.last_irq_code(),
             cd.response_fifo_size(), cd.dma_data_index(),
             cd.busy_cycles_remaining(), cpu.sys_->debug_dma_read(0x70u),
             cpu.sys_->debug_dma_read(0x74u));
    context->memory_trace_current = false;
  }

  const bool completed_delay_slot =
      context->is_branch_tail &&
      index == context->branch_instruction_index + 1u;
  if (completed_delay_slot) {
    ++context->backend->stats_.native_branch_tail_delay_slot_finish_helper_calls;
    context->backend->record_native_branch_tail_trace(
        *context->block, context->branch_taken,
        context->delay_slot_used_memory, context->delay_slot_used_mmio,
        cpu.exception_raised_);
  }

  if (cpu.exception_raised_) {
    result->exit_reason = CpuBlockExitReason::Exception;
    if (memory_instruction != 0) {
      ++context->backend->stats_.native_memory_exception_exits;
    }
    return false;
  }

  if (context->memory_filter_exit) {
    result->exit_reason = CpuBlockExitReason::Fallback;
    return false;
  }

  if (completed_delay_slot) {
    result->exit_reason = CpuBlockExitReason::Branch;
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
  NativeBlockRejectDetail branch_tail_detail = NativeBlockRejectDetail::None;
  if (is_x64_branch_tail_block(block, branch_tail_detail)) {
    return NativeBlockRejectReason::None;
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

NativeBlockRejectDetail CpuOptimizedBackend::classify_x64_reject_detail(
    const DecodedBlock &block) const {
  if (block.instruction_count == 0) {
    return NativeBlockRejectDetail::UnsupportedInstruction;
  }
  for (u32 i = 0; i < block.instruction_count; ++i) {
    const DecodedInstruction &inst = block.instructions[i];
    if (inst.is_branch) {
      return NativeBlockRejectDetail::Branch;
    }
    if (inst.may_access_memory) {
      if (!is_x64_memory_op(inst.op)) {
        return NativeBlockRejectDetail::Memory;
      }
      continue;
    }
    if (inst.op == DecodedOp::Cop0) {
      return NativeBlockRejectDetail::Cop0;
    }
    if (inst.op == DecodedOp::Cop2) {
      return NativeBlockRejectDetail::Cop2;
    }
    if (inst.must_fallback) {
      return NativeBlockRejectDetail::FallbackInstruction;
    }
    if (inst.may_raise_exception) {
      return NativeBlockRejectDetail::ExceptionRisk;
    }
    if (!is_x64_stage2_op(inst.op)) {
      return NativeBlockRejectDetail::UnsupportedInstruction;
    }
  }
  if (block.has_control_flow) {
    return NativeBlockRejectDetail::Branch;
  }
  if (block.has_fallback) {
    return NativeBlockRejectDetail::FallbackInstruction;
  }
  return native_reject_detail_for_reason(block.native_reject_reason);
}

bool CpuOptimizedBackend::ensure_x64_safety_checked(DecodedBlock &block) {
  if (block.native_safety_checked) {
    return block.native_stage1_safe;
  }

  block.native_safety_checked = true;
  NativeBlockRejectDetail branch_tail_detail = NativeBlockRejectDetail::None;
  block.native_branch_tail =
      is_x64_branch_tail_block(block, branch_tail_detail);
  block.native_shape = classify_x64_block_shape(block);
  block.native_reject_reason = classify_x64_reject_reason(block);
  block.native_stage1_safe =
      block.native_reject_reason == NativeBlockRejectReason::None;
  NativeBlockRejectDetail reduced_helper_detail =
      NativeBlockRejectDetail::None;
  const bool reduced_helper_ram_load_candidate =
      is_x64_reduced_helper_ram_load_block(block, reduced_helper_detail);
  block.native_reduced_helper_ram_load =
      block.native_stage1_safe && !block.native_branch_tail &&
      reduced_helper_ram_load_candidate &&
      g_cpu_x64_jit_ram_load_fastpath_enabled;
  if (reduced_helper_ram_load_candidate &&
      !g_cpu_x64_jit_ram_load_fastpath_enabled) {
    reduced_helper_detail =
        NativeBlockRejectDetail::ReducedHelperPreflightDisabled;
  }
  block.native_reduced_helper =
      block.native_stage1_safe && !block.native_branch_tail &&
      (!block.has_memory || block.native_reduced_helper_ram_load);
  block.native_reduced_helper_reject_detail = reduced_helper_detail;
  block.native_reject_detail =
      branch_tail_detail != NativeBlockRejectDetail::None
          ? branch_tail_detail
          : classify_x64_reject_detail(block);

  switch (block.native_shape) {
  case NativeBlockShape::StraightAlu:
    ++stats_.native_shape_straight_alu;
    break;
  case NativeBlockShape::StraightAluRamLoadCandidate:
    ++stats_.native_shape_straight_alu_ram_load;
    break;
  case NativeBlockShape::StraightAluRamStoreCandidate:
    ++stats_.native_shape_straight_alu_ram_store;
    break;
  case NativeBlockShape::HelperSafeMemory:
    ++stats_.native_shape_helper_safe_memory;
    break;
  case NativeBlockShape::BranchTail:
    ++stats_.native_shape_branch_tail;
    break;
  case NativeBlockShape::FallbackControl:
    ++stats_.native_shape_fallback_control;
    break;
  case NativeBlockShape::Cop0:
    ++stats_.native_shape_cop0;
    break;
  case NativeBlockShape::Cop2:
    ++stats_.native_shape_cop2;
    break;
  case NativeBlockShape::Mmio:
    break;
  case NativeBlockShape::ExceptionUnsafe:
    ++stats_.native_shape_exception_unsafe;
    break;
  }
  if (block.native_reduced_helper) {
    ++stats_.native_reduced_helper_candidate_blocks;
  } else {
    ++stats_.native_reduced_helper_rejected_blocks;
    stats_.native_reduced_helper_rejected_instructions +=
        block.instruction_count;
    if (block.native_stage1_safe) {
      ++stats_.native_direct_helper_candidate_blocks;
    }
    switch (block.native_reduced_helper_reject_detail) {
    case NativeBlockRejectDetail::ReducedHelperStore:
      ++stats_.native_reduced_helper_reject_stores;
      break;
    case NativeBlockRejectDetail::ReducedHelperLoadBaseWritten:
      ++stats_.native_reduced_helper_reject_load_base_written;
      break;
    case NativeBlockRejectDetail::ReducedHelperUnsupportedMemory:
      ++stats_.native_reduced_helper_reject_unsupported_memory;
      break;
    default:
      break;
    }
  }
  if (!block.native_stage1_safe) {
    if (block.has_control_flow) {
      ++stats_.native_branch_tail_rejects;
      DecodedOp rejected_branch_op = DecodedOp::Unsupported;
      for (u32 i = 0; i < block.instruction_count; ++i) {
        if (block.instructions[i].is_branch) {
          rejected_branch_op = block.instructions[i].op;
          break;
        }
      }
      switch (rejected_branch_op) {
      case DecodedOp::Bne: ++stats_.native_branch_tail_reject_bne; break;
      case DecodedOp::Beq: ++stats_.native_branch_tail_reject_beq; break;
      case DecodedOp::Bgtz: ++stats_.native_branch_tail_reject_bgtz; break;
      case DecodedOp::Blez: ++stats_.native_branch_tail_reject_blez; break;
      default: ++stats_.native_branch_tail_reject_other_opcode; break;
      }
    }
    block.native_rejected_unsafe = true;
    ++stats_.native_rejected_unsafe_blocks;
    ++stats_.native_rejected_block_count;
    stats_.native_rejected_block_instructions += block.instruction_count;
    record_native_reject(block.native_reject_reason);
    switch (block.native_reject_detail) {
    case NativeBlockRejectDetail::ExceptionRisk:
      ++stats_.native_reject_exception_risk;
      break;
    case NativeBlockRejectDetail::FallbackInstruction:
      ++stats_.native_reject_fallback_instruction;
      break;
    case NativeBlockRejectDetail::UnsupportedInstruction:
      ++stats_.native_reject_unsupported_instruction;
      break;
    default:
      break;
    }
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

  if (block.native_reduced_helper) {
    ++stats_.native_reduced_helper_compile_attempts;
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
    context->load_reg = &cpu_.load_.reg;
    context->load_value = &cpu_.load_.value;
    context->next_load_reg = &cpu_.next_load_.reg;
    context->next_load_value = &cpu_.next_load_.value;
    context->cycle_penalty = &cpu_.cycle_penalty_;
    context->ram_data = cpu_.sys_->jit_main_ram_data();
    context->ram_load_fastpath_counter =
        &stats_.native_memory_fastpath_loads;
    context->branch_tail_ram_load_fastpath_counter =
        &stats_.native_branch_tail_ram_load_fastpath_loads;
    context->instruction_count = block.instruction_count;
    context->is_branch_tail = block.native_branch_tail;
    context->uses_instruction_helpers =
        context->is_branch_tail ||
        (block_uses_instruction_helpers(block) &&
         !block.native_reduced_helper);
    if (context->is_branch_tail) {
      context->branch_instruction_index = block.instruction_count - 2u;
    }
    context->end_pc = block.start_pc + block.instruction_count * 4u;
    context->end_next_pc = context->end_pc + 4u;
    context->last_pc = context->end_pc - 4u;
    for (u32 i = 0; i < block.instruction_count; ++i) {
      context->base_cycles += block.instructions[i].cycles;
      if (block.native_reduced_helper_ram_load &&
          is_x64_load_op(block.instructions[i].op)) {
        context->base_cycles += 4u;
      }
    }

    emit_x64_block(
        *context, block,
        reinterpret_cast<uintptr_t>(
            &CpuOptimizedBackend::x64_native_prepare_instruction),
        reinterpret_cast<uintptr_t>(
            &CpuOptimizedBackend::x64_native_memory_instruction),
        reinterpret_cast<uintptr_t>(
            &CpuOptimizedBackend::x64_native_branch_instruction),
        reinterpret_cast<uintptr_t>(
            &CpuOptimizedBackend::x64_native_finish_instruction));
    block.native_fn = context->code.getCode<DecodedBlock::NativeFn>();
    block.native_code_bytes = context->code.getSize();
    block.native_context = context.release();
    block.destroy_native_context = destroy_x64_native_context;
    ++stats_.native_compile_successes;
    ++stats_.native_blocks_compiled;
    if (block.native_reduced_helper) {
      ++stats_.native_reduced_helper_compile_successes;
      ++stats_.native_reduced_helper_blocks_compiled;
      if (block.native_reduced_helper_ram_load) {
        ++stats_.native_reduced_helper_ram_load_blocks_compiled;
      }
    }
    if (block.native_branch_tail) {
      ++stats_.native_branch_tail_blocks_compiled;
    } else if (block.has_memory) {
      ++stats_.native_memory_blocks_compiled;
    } else {
      ++stats_.native_alu_blocks_compiled;
    }
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
    if (block.native_branch_tail) {
      ++stats_.native_branch_tail_to_decoded_fallbacks;
    }
    ++stats_.native_reject_unsafe_state;
    ++specific_counter;
    if (block.native_reduced_helper) {
      ++stats_.native_reduced_helper_fallbacks;
    }
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
  if (block.instruction_count > max_instructions &&
      !(context->is_branch_tail &&
        g_cpu_backend_compare_allow_partial_branch_tail) &&
      !(block.has_memory &&
        g_cpu_backend_compare_allow_partial_memory_helper)) {
    ++stats_.native_to_decoded_fallbacks;
    if (block.native_branch_tail) {
      ++stats_.native_branch_tail_to_decoded_fallbacks;
    }
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
  const bool completed_delay_slot_state =
      cpu_.in_delay_slot_ && !cpu_.pending_delay_slot_ &&
      !cpu_.pending_branch_taken_ && cpu_.pending_branch_pc_ == 0u &&
      cpu_.current_pc_ == cpu_.active_branch_pc_ + 4u;
  if (cpu_.pending_delay_slot_ ||
      (cpu_.in_delay_slot_ && !completed_delay_slot_state) ||
      cpu_.pending_branch_taken_ || cpu_.pending_branch_pc_ != 0u) {
    const NativeBlockRejectDetail detail =
        record_native_branch_delay_subreasons();
    return reject_to_decoded(stats_.native_reject_branch_delay_state, detail);
  }
  if (completed_delay_slot_state) {
    cpu_.in_delay_slot_ = false;
    cpu_.active_branch_pc_ = 0u;
  }

  if (block.native_reduced_helper_ram_load) {
    NativeBlockRejectDetail preflight_detail =
        NativeBlockRejectDetail::None;
    if (!g_cpu_x64_jit_ram_load_fastpath_enabled ||
        g_cpu_x64_jit_memory_trace || g_trace_ram || g_trace_bus ||
        g_cpu_x64_jit_disable_native_ram || context->ram_data == nullptr) {
      preflight_detail =
          NativeBlockRejectDetail::ReducedHelperPreflightDisabled;
    } else {
      for (u32 i = 0; i < block.instruction_count; ++i) {
        const DecodedInstruction &inst = block.instructions[i];
        if (!is_x64_load_op(inst.op)) {
          continue;
        }
        const u32 addr =
            cpu_.gpr_[inst.rs] + static_cast<u32>(inst.simm);
        if (((inst.op == DecodedOp::Lh || inst.op == DecodedOp::Lhu) &&
             (addr & 1u) != 0u) ||
            (inst.op == DecodedOp::Lw && (addr & 3u) != 0u)) {
          preflight_detail = NativeBlockRejectDetail::Unaligned;
          break;
        }
        if (!is_canonical_ram_alias(addr)) {
          preflight_detail =
              classify_native_memory_region(addr) == NativeMemoryRegion::Mmio
                  ? NativeBlockRejectDetail::Mmio
                  : NativeBlockRejectDetail::Memory;
          break;
        }
      }
    }

    if (preflight_detail != NativeBlockRejectDetail::None) {
      ++stats_.native_to_decoded_fallbacks;
      ++stats_.native_reduced_helper_fallbacks;
      ++stats_.native_reduced_helper_ram_load_preflight_fallbacks;
      if (preflight_detail == NativeBlockRejectDetail::Mmio) {
        ++stats_.native_reduced_helper_ram_load_preflight_mmio;
        ++stats_.native_reject_mmio;
      } else if (preflight_detail == NativeBlockRejectDetail::Unaligned) {
        ++stats_.native_reduced_helper_ram_load_preflight_unaligned;
        ++stats_.native_reject_unaligned;
      } else if (preflight_detail == NativeBlockRejectDetail::Memory) {
        ++stats_.native_reduced_helper_ram_load_preflight_non_ram;
        ++stats_.native_reject_memory;
      } else {
        ++stats_.native_reduced_helper_ram_load_preflight_disabled;
        ++stats_.native_ram_disabled_fallbacks;
      }
      record_rejected_block(preflight_detail);
      return execute_block(block, max_cycles, max_instructions);
    }
  }

  const bool active_load_delay =
      cpu_.load_.reg != 0 || cpu_.next_load_.reg != 0;
  if (active_load_delay && !context->uses_instruction_helpers &&
      !block.native_reduced_helper) {
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
  } else if (active_load_delay && block.native_reduced_helper) {
    ++stats_.native_reduced_helper_load_delay_entries;
  }

  context->max_cycles = max_cycles;
  context->max_instructions = max_instructions;
  context->branch_taken = false;
  context->memory_filter_exit = false;
  context->memory_trace_current = false;
  context->ram_load_fastpath_enabled =
      g_cpu_x64_jit_ram_load_fastpath_enabled &&
      !g_cpu_x64_jit_memory_trace && !g_trace_ram && !g_trace_bus &&
      !g_cpu_x64_jit_disable_native_ram;
  context->delay_slot_used_memory =
      context->is_branch_tail &&
      block.instructions[block.instruction_count - 1u].may_access_memory;
  context->delay_slot_used_mmio = false;

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

  if (context->is_branch_tail) {
    ++stats_.native_branch_tail_entries;
    ++block.native_branch_tail_entry_count;
    const DecodedOp branch_op =
        block.instructions[context->branch_instruction_index].op;
    if (branch_op == DecodedOp::Bgtz) {
      ++stats_.native_branch_tail_bgtz_entries;
    } else if (branch_op == DecodedOp::Blez) {
      ++stats_.native_branch_tail_blez_entries;
    }
  } else if (block.has_memory) {
    ++stats_.native_memory_block_entries;
  } else {
    ++stats_.native_alu_block_entries;
  }
  ++block.native_entry_count;
  if (block.native_reduced_helper) {
    ++stats_.native_reduced_helper_entries;
    if (block.native_reduced_helper_ram_load) {
      ++stats_.native_reduced_helper_ram_load_entries;
    }
  }
  block.native_fn(block.native_context, &result);
  if (context->is_branch_tail) {
    switch (result.exit_reason) {
    case CpuBlockExitReason::Branch:
      ++stats_.native_branch_tail_completed_exits;
      break;
    case CpuBlockExitReason::Budget:
      ++stats_.native_branch_tail_budget_exits;
      break;
    case CpuBlockExitReason::Fallback:
      ++stats_.native_branch_tail_fallback_exits;
      break;
    case CpuBlockExitReason::Exception:
      ++stats_.native_branch_tail_exception_exits;
      break;
    default:
      break;
    }
  }

  if (active_load_delay && context->uses_instruction_helpers) {
    if (result.instructions != 0) {
      ++stats_.native_helper_load_delay_passes;
    } else {
      ++stats_.native_helper_load_delay_fallbacks;
    }
  }

  g_diag_current_pc = cpu_.current_pc_;

  ++stats_.native_block_entries;
  stats_.native_instructions += result.instructions;
  if (block.native_reduced_helper) {
    stats_.native_reduced_helper_instructions += result.instructions;
  }
  stats_.native_cycles += result.cycles;
  stats_.optimized_instructions += result.instructions;
  return result;
}
