#include "cpu_jit_v2.h"
#include "system.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#if defined(VIBESTATION_ENABLE_X64_JIT) && \
    (defined(_M_X64) || defined(__x86_64__))
#include <xbyak/xbyak.h>
#define VIBESTATION_JIT_V2_X64 1
#else
#define VIBESTATION_JIT_V2_X64 0
#endif

namespace {

enum class V2AluOp : u8 {
  Nop,
  Sll,
  Srl,
  Sra,
  Addu,
  Subu,
  And,
  Or,
  Xor,
  Nor,
  Slt,
  Sltu,
  Addiu,
  Slti,
  Sltiu,
  Andi,
  Ori,
  Xori,
  Lui,
  Clear,
  Sw,
  Beq,
  Bne,
};

struct V2DecodedInstruction {
  V2AluOp op = V2AluOp::Nop;
  u8 rs = 0;
  u8 rt = 0;
  u8 rd = 0;
  u8 shamt = 0;
  u16 imm = 0;
  s32 simm = 0;
};

bool decode_v2_alu(u32 bits, V2DecodedInstruction &out) {
  out = {};
  out.rs = static_cast<u8>((bits >> 21) & 0x1Fu);
  out.rt = static_cast<u8>((bits >> 16) & 0x1Fu);
  out.rd = static_cast<u8>((bits >> 11) & 0x1Fu);
  out.shamt = static_cast<u8>((bits >> 6) & 0x1Fu);
  out.imm = static_cast<u16>(bits & 0xFFFFu);
  out.simm = sign_extend_16(out.imm);

  const u32 primary = (bits >> 26) & 0x3Fu;
  if (primary == 0u) {
    if (bits == 0u) {
      out.op = V2AluOp::Nop;
      return true;
    }
    switch (bits & 0x3Fu) {
    case 0x00: out.op = V2AluOp::Sll; return true;
    case 0x02: out.op = V2AluOp::Srl; return true;
    case 0x03: out.op = V2AluOp::Sra; return true;
    case 0x0F: out.op = V2AluOp::Nop; return true; // SYNC
    case 0x14:
    case 0x1C:
    case 0x28:
    case 0x29:
      out.op = V2AluOp::Nop;
      return true;
    case 0x21:
    case 0x2D:
      out.op = V2AluOp::Addu;
      return true;
    case 0x23:
    case 0x2F:
      out.op = V2AluOp::Subu;
      return true;
    case 0x24: out.op = V2AluOp::And; return true;
    case 0x25: out.op = V2AluOp::Or; return true;
    case 0x26: out.op = V2AluOp::Xor; return true;
    case 0x27: out.op = V2AluOp::Nor; return true;
    case 0x2A: out.op = V2AluOp::Slt; return true;
    case 0x2B: out.op = V2AluOp::Sltu; return true;
    case 0x38: out.op = V2AluOp::Clear; return true;
    default: return false;
    }
  }

  switch (primary) {
  case 0x04: out.op = V2AluOp::Beq; return true;
  case 0x05: out.op = V2AluOp::Bne; return true;
  case 0x09: out.op = V2AluOp::Addiu; return true;
  case 0x0A: out.op = V2AluOp::Slti; return true;
  case 0x0B: out.op = V2AluOp::Sltiu; return true;
  case 0x0C: out.op = V2AluOp::Andi; return true;
  case 0x0D: out.op = V2AluOp::Ori; return true;
  case 0x0E: out.op = V2AluOp::Xori; return true;
  case 0x0F: out.op = V2AluOp::Lui; return true;
  case 0x2B: out.op = V2AluOp::Sw; return true;
  default: return false;
  }
}

u32 read_mask(const V2DecodedInstruction &inst) {
  auto reg = [](u8 r) { return r == 0u ? 0u : (1u << r); };
  switch (inst.op) {
  case V2AluOp::Sll:
  case V2AluOp::Srl:
  case V2AluOp::Sra:
    return reg(inst.rt);
  case V2AluOp::Addu:
  case V2AluOp::Subu:
  case V2AluOp::And:
  case V2AluOp::Or:
  case V2AluOp::Xor:
  case V2AluOp::Nor:
  case V2AluOp::Slt:
  case V2AluOp::Sltu:
    return reg(inst.rs) | reg(inst.rt);
  case V2AluOp::Addiu:
  case V2AluOp::Slti:
  case V2AluOp::Sltiu:
  case V2AluOp::Andi:
  case V2AluOp::Ori:
  case V2AluOp::Xori:
    return reg(inst.rs);
  case V2AluOp::Sw:
  case V2AluOp::Beq:
  case V2AluOp::Bne:
    return reg(inst.rs) | reg(inst.rt);
  case V2AluOp::Nop:
  case V2AluOp::Lui:
  case V2AluOp::Clear:
    return 0u;
  }
  return 0u;
}

u8 write_reg(const V2DecodedInstruction &inst) {
  switch (inst.op) {
  case V2AluOp::Sll:
  case V2AluOp::Srl:
  case V2AluOp::Sra:
  case V2AluOp::Addu:
  case V2AluOp::Subu:
  case V2AluOp::And:
  case V2AluOp::Or:
  case V2AluOp::Xor:
  case V2AluOp::Nor:
  case V2AluOp::Slt:
  case V2AluOp::Sltu:
  case V2AluOp::Clear:
    return inst.rd;
  case V2AluOp::Addiu:
  case V2AluOp::Slti:
  case V2AluOp::Sltiu:
  case V2AluOp::Andi:
  case V2AluOp::Ori:
  case V2AluOp::Xori:
  case V2AluOp::Lui:
    return inst.rt;
  case V2AluOp::Nop:
  case V2AluOp::Sw:
  case V2AluOp::Beq:
  case V2AluOp::Bne:
    return 0u;
  }
  return 0u;
}

bool is_v2_branch(V2AluOp op) {
  return op == V2AluOp::Beq || op == V2AluOp::Bne;
}

bool is_v2_alu_only(V2AluOp op) {
  return op != V2AluOp::Sw && !is_v2_branch(op);
}

std::array<u8, 6> choose_cached_regs(
    const std::vector<V2DecodedInstruction> &instructions) {
  std::array<u8, 32> score{};
  for (const auto &inst : instructions) {
    const u32 reads = read_mask(inst);
    for (u32 reg = 1; reg < 32; ++reg) {
      if ((reads & (1u << reg)) != 0u) {
        score[reg] = static_cast<u8>(
            std::min<u32>(255u, static_cast<u32>(score[reg]) + 2u));
      }
    }
    const u8 dst = write_reg(inst);
    if (dst != 0u) {
      score[dst] = static_cast<u8>(
          std::min<u32>(255u, static_cast<u32>(score[dst]) + 1u));
    }
  }

  std::array<u8, 6> selected{};
  for (u32 slot = 0; slot < selected.size(); ++slot) {
    u8 best_reg = 0u;
    u8 best_score = 0u;
    for (u32 reg = 1; reg < 32; ++reg) {
      if (score[reg] <= best_score) {
        continue;
      }
      bool used = false;
      for (u32 prior = 0; prior < slot; ++prior) {
        used = used || selected[prior] == reg;
      }
      if (!used) {
        best_reg = static_cast<u8>(reg);
        best_score = score[reg];
      }
    }
    if (best_reg == 0u) {
      break;
    }
    selected[slot] = best_reg;
  }
  return selected;
}

#if VIBESTATION_JIT_V2_X64

struct V2NativeRuntime {
  std::array<u8 *, 8> store_ptrs{};
};

using V2NativeFn = u32 (*)(u32 *, const V2NativeRuntime *);

int cache_slot(const std::array<u8, 6> &cached, u8 guest_reg) {
  for (int i = 0; i < static_cast<int>(cached.size()); ++i) {
    if (cached[static_cast<size_t>(i)] == guest_reg && guest_reg != 0u) {
      return i;
    }
  }
  return -1;
}

Xbyak::Reg32 cache_host_reg(Xbyak::CodeGenerator &code, int slot) {
  switch (slot) {
  case 0: return code.r8d;
  case 1: return code.r9d;
  case 2: return code.r10d;
  case 3: return code.r11d;
  case 4: return code.r12d;
  default: return code.r13d;
  }
}

void emit_read_guest(Xbyak::CodeGenerator &code, const Xbyak::Reg32 &dst,
                     const std::array<u8, 6> &cached, u8 guest_reg) {
  using namespace Xbyak;
  if (guest_reg == 0u) {
    code.xor_(dst, dst);
    return;
  }
  const int slot = cache_slot(cached, guest_reg);
  if (slot >= 0) {
    const Reg32 src = cache_host_reg(code, slot);
    if (src.getIdx() != dst.getIdx()) {
      code.mov(dst, src);
    }
    return;
  }
  code.mov(dst, code.dword[code.r15 + static_cast<int>(guest_reg) * 4]);
}

void emit_write_guest(Xbyak::CodeGenerator &code,
                      const std::array<u8, 6> &cached, u8 guest_reg,
                      const Xbyak::Reg32 &src,
                      std::array<bool, 6> &dirty) {
  using namespace Xbyak;
  if (guest_reg == 0u) {
    return;
  }
  const int slot = cache_slot(cached, guest_reg);
  if (slot >= 0) {
    const Reg32 dst = cache_host_reg(code, slot);
    if (dst.getIdx() != src.getIdx()) {
      code.mov(dst, src);
    }
    dirty[static_cast<size_t>(slot)] = true;
    return;
  }
  code.mov(code.dword[code.r15 + static_cast<int>(guest_reg) * 4], src);
}

std::unique_ptr<Xbyak::CodeGenerator> compile_native_alu(
    const std::vector<V2DecodedInstruction> &instructions,
    const std::array<u8, 6> &cached) {
  using namespace Xbyak;
  auto code = std::make_unique<CodeGenerator>(4096);

  // V2 deliberately keeps architectural state virtual inside a block.
  // r15 = guest GPR base, r14 = runtime/preflight data, rbx = branch result.
  // r8-r13 are the six guest-register cache slots.
  code->push(code->rbx);
  code->push(code->r12);
  code->push(code->r13);
  code->push(code->r14);
  code->push(code->r15);

#if defined(_WIN32)
  code->mov(code->r15, code->rcx);
  code->mov(code->r14, code->rdx);
#else
  code->mov(code->r15, code->rdi);
  code->mov(code->r14, code->rsi);
#endif
  code->xor_(code->ebx, code->ebx);

  for (size_t slot = 0; slot < cached.size(); ++slot) {
    if (cached[slot] == 0u) {
      continue;
    }
    code->mov(cache_host_reg(*code, static_cast<int>(slot)),
              code->dword[code->r15 + static_cast<int>(cached[slot]) * 4]);
  }

  std::array<bool, 6> dirty{};
  u32 store_index = 0;

  for (const auto &inst : instructions) {
    const u8 dst = write_reg(inst);
    switch (inst.op) {
    case V2AluOp::Nop:
      break;

    case V2AluOp::Sll:
    case V2AluOp::Srl:
    case V2AluOp::Sra:
      emit_read_guest(*code, code->eax, cached, inst.rt);
      if (inst.shamt != 0u) {
        if (inst.op == V2AluOp::Sll) code->shl(code->eax, inst.shamt);
        else if (inst.op == V2AluOp::Srl) code->shr(code->eax, inst.shamt);
        else code->sar(code->eax, inst.shamt);
      }
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V2AluOp::Addu:
    case V2AluOp::Subu:
    case V2AluOp::And:
    case V2AluOp::Or:
    case V2AluOp::Xor:
    case V2AluOp::Nor:
    case V2AluOp::Slt:
    case V2AluOp::Sltu:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      emit_read_guest(*code, code->ecx, cached, inst.rt);
      switch (inst.op) {
      case V2AluOp::Addu: code->add(code->eax, code->ecx); break;
      case V2AluOp::Subu: code->sub(code->eax, code->ecx); break;
      case V2AluOp::And: code->and_(code->eax, code->ecx); break;
      case V2AluOp::Or: code->or_(code->eax, code->ecx); break;
      case V2AluOp::Xor: code->xor_(code->eax, code->ecx); break;
      case V2AluOp::Nor:
        code->or_(code->eax, code->ecx);
        code->not_(code->eax);
        break;
      case V2AluOp::Slt:
        code->cmp(code->eax, code->ecx);
        code->setl(code->al);
        code->movzx(code->eax, code->al);
        break;
      case V2AluOp::Sltu:
        code->cmp(code->eax, code->ecx);
        code->setb(code->al);
        code->movzx(code->eax, code->al);
        break;
      default: break;
      }
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V2AluOp::Addiu:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      if (inst.simm != 0) {
        code->add(code->eax, static_cast<u32>(inst.simm));
      }
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V2AluOp::Slti:
    case V2AluOp::Sltiu:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      code->mov(code->ecx, static_cast<u32>(inst.simm));
      code->cmp(code->eax, code->ecx);
      if (inst.op == V2AluOp::Slti) code->setl(code->al);
      else code->setb(code->al);
      code->movzx(code->eax, code->al);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V2AluOp::Andi:
    case V2AluOp::Ori:
    case V2AluOp::Xori:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      if (inst.op == V2AluOp::Andi) code->and_(code->eax, inst.imm);
      else if (inst.op == V2AluOp::Ori) code->or_(code->eax, inst.imm);
      else code->xor_(code->eax, inst.imm);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V2AluOp::Lui:
      code->mov(code->eax, static_cast<u32>(inst.imm) << 16u);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V2AluOp::Clear:
      code->xor_(code->eax, code->eax);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V2AluOp::Sw: {
      emit_read_guest(*code, code->ecx, cached, inst.rt);
      const size_t ptr_offset =
          offsetof(V2NativeRuntime, store_ptrs) +
          static_cast<size_t>(store_index) * sizeof(u8 *);
      code->mov(code->rax, code->ptr[code->r14 + static_cast<int>(ptr_offset)]);
      code->mov(code->dword[code->rax], code->ecx);
      ++store_index;
      break;
    }

    case V2AluOp::Beq:
    case V2AluOp::Bne:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      emit_read_guest(*code, code->ecx, cached, inst.rt);
      code->cmp(code->eax, code->ecx);
      if (inst.op == V2AluOp::Beq) code->sete(code->bl);
      else code->setne(code->bl);
      code->movzx(code->ebx, code->bl);
      break;
    }
  }

  for (size_t slot = 0; slot < cached.size(); ++slot) {
    if (cached[slot] == 0u || !dirty[slot]) {
      continue;
    }
    code->mov(code->dword[code->r15 + static_cast<int>(cached[slot]) * 4],
              cache_host_reg(*code, static_cast<int>(slot)));
  }

  code->mov(code->dword[code->r15], 0u);
  code->mov(code->eax, code->ebx);
  code->pop(code->r15);
  code->pop(code->r14);
  code->pop(code->r13);
  code->pop(code->r12);
  code->pop(code->rbx);
  code->ret();
  code->ready();
  return code;
}

#endif

} // namespace

struct CpuJitV2Backend::Impl {
  struct Block {
    u32 start_pc = 0;
    u32 phys_start = 0;
    u32 phys_end = 0;
    u32 instruction_count = 0;
    u32 base_cycles = 0;
    u32 store_count = 0;
    u32 branch_index = 0;
    u32 branch_target = 0;
    bool has_store = false;
    bool has_branch = false;
    std::array<u32, 16> words{};
    std::array<u8, 6> cached_regs{};
    std::array<u8, 8> store_rs{};
    std::array<s32, 8> store_simm{};
    std::array<u8, 8> store_instruction_index{};
#if VIBESTATION_JIT_V2_X64
    std::unique_ptr<Xbyak::CodeGenerator> code;
    V2NativeFn fn = nullptr;
#endif
  };

  std::unordered_map<u32, Block> blocks;
  // Conservative ever-compiled page set. Entries are intentionally retained
  // until flush: stale positives cost only an occasional scan, while there can
  // never be a false negative that misses self-modifying code.
  std::unordered_set<u32> code_pages;
};

CpuJitV2Backend::CpuJitV2Backend(Cpu &cpu)
    : cpu_(cpu), impl_(std::make_unique<Impl>()) {
  stats_.available = true;
  stats_.native_available = VIBESTATION_JIT_V2_X64 != 0;
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
  stats_.native_available = VIBESTATION_JIT_V2_X64 != 0;

  auto interpreter_step = [&]() {
    const u32 consumed = cpu_.step();
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.interpreter_fallback_steps;
    ++stats_.fallback_instructions;
    stats_.executed_cycles += consumed;
  };

#if VIBESTATION_JIT_V2_X64
  auto state_allows_native = [&]() {
    if (g_trace_cpu || g_cpu_deep_diagnostics || g_log_fmv_diagnostics) {
      return false;
    }

    // After a branch delay slot retires, Cpu::step() leaves a small amount of
    // descriptive state behind until the next instruction begins. Normalize
    // that exact completed state here just as the next interpreter step would.
    const bool completed_delay_slot =
        cpu_.in_delay_slot_ && !cpu_.pending_delay_slot_ &&
        !cpu_.pending_branch_taken_ && cpu_.pending_branch_pc_ == 0u &&
        cpu_.active_branch_pc_ != 0u &&
        cpu_.current_pc_ == cpu_.active_branch_pc_ + 4u;
    if (completed_delay_slot) {
      cpu_.in_delay_slot_ = false;
      cpu_.active_branch_pc_ = 0u;
    }

    if (cpu_.in_delay_slot_ || cpu_.pending_delay_slot_ ||
        cpu_.pending_branch_taken_ || cpu_.pending_branch_pc_ != 0u) {
      ++stats_.native_reject_branch_delay_state;
      return false;
    }
    if (cpu_.load_.reg != 0u || cpu_.next_load_.reg != 0u) {
      ++stats_.native_reject_load_delay_state;
      return false;
    }
    if (cpu_.next_pc_ != cpu_.pc_ + 4u) {
      ++stats_.native_reject_pc_state;
      return false;
    }
    return true;
  };

  auto try_native = [&]() -> bool {
    if (!state_allows_native()) {
      return false;
    }

    // Match the interpreter's between-instruction hardware IRQ sampling.
    if (cpu_.sys_->irq_pending()) {
      cpu_.cop0_cause_ |= (1u << 10);
    } else {
      cpu_.cop0_cause_ &= ~(1u << 10);
    }
    if (cpu_.check_irq()) {
      ++stats_.native_reject_irq_state;
      return false;
    }

    const u32 start_pc = cpu_.pc_;
    if (!cpu_.instruction_cacheable(start_pc)) {
      ++stats_.native_reject_icache;
      return false;
    }

    const u32 index = (start_pc >> 4u) & 0xFFu;
    const u32 word_index = (start_pc >> 2u) & 0x03u;
    const u32 expected_tag = psx::mask_address(start_pc) & ~0x0Fu;
    auto &line = cpu_.icache_[index];
    if (!line.valid || line.tag != expected_tag) {
      ++stats_.native_reject_icache;
      return false;
    }

    auto found = impl_->blocks.find(start_pc);
    if (found != impl_->blocks.end()) {
      Impl::Block &block = found->second;
      bool coherent = true;
      for (u32 i = 0; i < block.instruction_count; ++i) {
        const u32 inst_pc = start_pc + i * 4u;
        if (!cpu_.instruction_cacheable(inst_pc)) {
          coherent = false;
          break;
        }
        const u32 inst_index = (inst_pc >> 4u) & 0xFFu;
        const u32 inst_word = (inst_pc >> 2u) & 0x03u;
        const u32 inst_tag = psx::mask_address(inst_pc) & ~0x0Fu;
        const auto &inst_line = cpu_.icache_[inst_index];
        if (!inst_line.valid || inst_line.tag != inst_tag ||
            inst_line.words[inst_word] != block.words[i]) {
          coherent = false;
          break;
        }
      }
      if (!coherent) {
        impl_->blocks.erase(found);
        found = impl_->blocks.end();
      }
    }

    if (found == impl_->blocks.end()) {
      ++stats_.cache_misses;
      ++stats_.native_compile_attempts;

      std::vector<V2DecodedInstruction> decoded;
      std::array<u32, 16> words{};
      constexpr u32 kMaxV2Instructions = 16u;
      u32 written_mask = 0u;
      u32 store_count = 0u;
      bool has_store = false;
      bool has_branch = false;
      u32 branch_index = 0u;
      s32 branch_simm = 0;
      std::array<u8, 8> store_rs{};
      std::array<s32, 8> store_simm{};
      std::array<u8, 8> store_instruction_index{};

      auto fetch_decoded = [&](u32 i, V2DecodedInstruction &inst,
                               u32 &bits) -> bool {
        const u32 inst_pc = start_pc + i * 4u;
        if (!cpu_.instruction_cacheable(inst_pc)) {
          return false;
        }
        const u32 inst_index = (inst_pc >> 4u) & 0xFFu;
        const u32 inst_word = (inst_pc >> 2u) & 0x03u;
        const u32 inst_tag = psx::mask_address(inst_pc) & ~0x0Fu;
        const auto &inst_line = cpu_.icache_[inst_index];
        if (!inst_line.valid || inst_line.tag != inst_tag) {
          return false;
        }
        bits = inst_line.words[inst_word];
        return decode_v2_alu(bits, inst);
      };

      for (u32 i = 0; i < kMaxV2Instructions; ++i) {
        V2DecodedInstruction inst{};
        u32 bits = 0u;
        if (!fetch_decoded(i, inst, bits)) {
          break;
        }

        if (is_v2_branch(inst.op)) {
          // V2 branch blocks always include the architectural delay slot and
          // end immediately after it. Keep the delay slot ALU-only for the
          // first branch tier; memory delay slots remain interpreter territory.
          if (i + 1u >= kMaxV2Instructions) {
            break;
          }
          V2DecodedInstruction delay{};
          u32 delay_bits = 0u;
          if (!fetch_decoded(i + 1u, delay, delay_bits) ||
              !is_v2_alu_only(delay.op)) {
            break;
          }

          has_branch = true;
          branch_index = static_cast<u32>(decoded.size());
          branch_simm = inst.simm;
          decoded.push_back(inst);
          words[branch_index] = bits;
          decoded.push_back(delay);
          words[branch_index + 1u] = delay_bits;
          break;
        }

        if (inst.op == V2AluOp::Sw) {
          if (store_count >= store_rs.size()) {
            break;
          }
          // Runtime address preflight uses the block-entry GPR value. If an
          // earlier instruction rewrites the base, stop before this store.
          if (inst.rs != 0u && (written_mask & (1u << inst.rs)) != 0u) {
            break;
          }
          has_store = true;
          store_rs[store_count] = inst.rs;
          store_simm[store_count] = inst.simm;
          store_instruction_index[store_count] =
              static_cast<u8>(decoded.size());
          ++store_count;
        }

        decoded.push_back(inst);
        words[decoded.size() - 1u] = bits;
        const u8 dst = write_reg(inst);
        if (dst != 0u) {
          written_mask |= 1u << dst;
        }
      }

      if (decoded.size() < 2u) {
        ++stats_.native_compile_failures;
        return false;
      }

      Impl::Block block{};
      block.start_pc = start_pc;
      block.instruction_count = static_cast<u32>(decoded.size());
      block.phys_start = psx::mask_address(start_pc);
      block.phys_end =
          psx::mask_address(start_pc + block.instruction_count * 4u - 1u);
      block.words = words;
      block.cached_regs = choose_cached_regs(decoded);
      block.has_store = has_store;
      block.has_branch = has_branch;
      block.store_count = store_count;
      block.store_rs = store_rs;
      block.store_simm = store_simm;
      block.store_instruction_index = store_instruction_index;
      block.branch_index = branch_index;
      if (has_branch) {
        const u32 branch_pc = start_pc + branch_index * 4u;
        block.branch_target =
            branch_pc + 4u + (static_cast<u32>(branch_simm) << 2u);
      }
      for (const auto &inst : decoded) {
        if (inst.op == V2AluOp::Sw) {
          block.base_cycles += 2u;
        } else {
          // Conditional branches cost one cycle when not taken and gain one
          // more dynamically when taken. All currently-native ALU ops cost 1.
          block.base_cycles += 1u;
        }
      }

      block.code = compile_native_alu(decoded, block.cached_regs);
      if (!block.code) {
        ++stats_.native_compile_failures;
        return false;
      }
      block.fn = block.code->getCode<V2NativeFn>();
      const u32 block_phys_first = block.phys_start;
      const u32 block_phys_last = block.phys_end;
      for (u32 page = block_phys_first >> 12u;
           page <= (block_phys_last >> 12u); ++page) {
        impl_->code_pages.insert(page);
      }
      auto inserted = impl_->blocks.emplace(start_pc, std::move(block));
      found = inserted.first;
      ++stats_.native_compile_successes;
      ++stats_.native_blocks_compiled;
      if (has_branch) {
        ++stats_.native_branch_tail_blocks_compiled;
      } else if (has_store) {
        ++stats_.native_memory_blocks_compiled;
      } else {
        ++stats_.native_alu_blocks_compiled;
      }
    } else {
      ++stats_.cache_hits;
    }

    Impl::Block &block = found->second;
    if (block.fn == nullptr || block.instruction_count == 0u) {
      return false;
    }

    if (block.has_branch && g_cpu_backend_compare_irq_on_branch) {
      ++stats_.native_reject_irq_state;
      return false;
    }

    V2NativeRuntime runtime{};
    std::array<u32, 8> store_addrs{};
    u32 main_ram_store_count = 0u;

    if (block.has_store) {
      // Direct writes intentionally bypass System::write32. Anything that
      // requires tracing, watchpoints, MMIO semantics or isolated-cache store
      // behavior leaves the native tier before generated code is entered.
      if (g_trace_ram || g_trace_bus || g_ram_watch_diagnostics ||
          (cpu_.cop0_sr_ & (1u << 16)) != 0u) {
        ++stats_.native_reject_unsafe_state;
        return false;
      }

      u8 *const main_ram = cpu_.sys_->jit_main_ram_data_mut();
      u8 *const scratchpad = cpu_.sys_->jit_scratchpad_data_mut();
      if (main_ram == nullptr || scratchpad == nullptr) {
        ++stats_.native_reject_memory;
        return false;
      }

      for (u32 i = 0; i < block.store_count; ++i) {
        const u8 base_reg = block.store_rs[i];
        const u32 base = base_reg == 0u ? 0u : cpu_.gpr_[base_reg];
        const u32 addr = base + static_cast<u32>(block.store_simm[i]);
        store_addrs[i] = addr;

        if ((addr & 3u) != 0u) {
          ++stats_.native_reject_unaligned;
          return false;
        }

        const u32 phys = psx::mask_address(addr);
        const u64 store_end = static_cast<u64>(phys) + 3u;
        const bool touches_current_code =
            phys <= block.phys_end &&
            store_end >= static_cast<u64>(block.phys_start);
        if (touches_current_code) {
          ++stats_.native_reject_icache;
          return false;
        }

        if (phys < psx::RAM_SIZE) {
          runtime.store_ptrs[i] = main_ram + phys;
          ++main_ram_store_count;
          continue;
        }

        if (phys >= 0x1F800000u && phys < 0x1F801000u) {
          const u32 scratch_off =
              (phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
          runtime.store_ptrs[i] = scratchpad + scratch_off;
          continue;
        }

        if (phys >= 0x1F801000u && phys < 0x1F803000u) {
          ++stats_.native_reject_mmio;
        } else {
          ++stats_.native_reject_memory;
        }
        return false;
      }
    }

    struct SimIcacheLine {
      u32 index = 0;
      u32 tag = 0;
      bool valid = false;
      bool refilled = false;
    };
    std::array<SimIcacheLine, 24> sim_icache{};
    u32 sim_icache_count = 0u;
    auto sim_line = [&](u32 index) -> SimIcacheLine & {
      for (u32 i = 0; i < sim_icache_count; ++i) {
        if (sim_icache[i].index == index) {
          return sim_icache[i];
        }
      }
      SimIcacheLine &state = sim_icache[sim_icache_count++];
      state.index = index;
      state.tag = cpu_.icache_[index].tag;
      state.valid = cpu_.icache_[index].valid;
      state.refilled = false;
      return state;
    };

    // Model the interpreter's direct-mapped I-cache effects instruction by
    // instruction. A data store invalidates by cache index, so an unrelated
    // scratchpad/RAM address can evict a code line and force a 4-cycle refill
    // before the next guest instruction in this same native block.
    u32 fetch_penalty = 0u;
    if (block.has_store) {
      u32 store_cursor = 0u;
      for (u32 i = 0; i < block.instruction_count; ++i) {
        const u32 inst_pc = start_pc + i * 4u;
        const u32 inst_index = (inst_pc >> 4u) & 0xFFu;
        const u32 inst_tag = psx::mask_address(inst_pc) & ~0x0Fu;
        SimIcacheLine &fetch_state = sim_line(inst_index);
        if (!fetch_state.valid || fetch_state.tag != inst_tag) {
          fetch_penalty += 4u;
          fetch_state.valid = true;
          fetch_state.tag = inst_tag;
          fetch_state.refilled = true;
        }

        if (store_cursor < block.store_count &&
            block.store_instruction_index[store_cursor] == i) {
          const u32 store_phys = psx::mask_address(store_addrs[store_cursor]);
          const u32 store_index = (store_phys >> 4u) & 0xFFu;
          SimIcacheLine &store_state = sim_line(store_index);
          store_state.valid = false;
          store_state.refilled = false;
          ++store_cursor;
        }
      }
    }

    const u32 remaining_cycles = max_cycles - result.cycles;
    const u32 remaining_instructions = max_instructions - result.instructions;
    const u32 worst_cycles =
        block.base_cycles + main_ram_store_count + fetch_penalty +
        (block.has_branch ? 1u : 0u);
    if (block.instruction_count > remaining_instructions ||
        worst_cycles > remaining_cycles) {
      ++stats_.native_reject_budget;
      return false;
    }

    cpu_.executing_step_ = true;
    cpu_.exception_raised_ = false;
    cpu_.cycle_penalty_ = 0u;
    cpu_.in_delay_slot_ = false;
    cpu_.active_branch_pc_ = 0u;
    cpu_.pending_delay_slot_ = false;
    cpu_.pending_branch_taken_ = false;
    cpu_.pending_branch_pc_ = 0u;

    const bool branch_taken = block.fn(cpu_.gpr_, &runtime) != 0u;

    const u32 count = block.instruction_count;
    const u32 consumed_cycles =
        block.base_cycles + main_ram_store_count + fetch_penalty +
        ((block.has_branch && branch_taken) ? 1u : 0u);

    if (block.has_branch) {
      const u32 branch_pc = start_pc + block.branch_index * 4u;
      const u32 delay_pc = branch_pc + 4u;
      cpu_.current_pc_ = delay_pc;
      cpu_.pc_ = branch_taken ? block.branch_target : (branch_pc + 8u);
      cpu_.next_pc_ = cpu_.pc_ + 4u;
      cpu_.in_delay_slot_ = true;
      cpu_.active_branch_pc_ = branch_pc;
      ++stats_.native_branch_tail_entries;
      if (branch_taken) {
        ++stats_.native_branch_taken;
      } else {
        ++stats_.native_branch_not_taken;
      }
    } else {
      cpu_.current_pc_ = start_pc + (count - 1u) * 4u;
      cpu_.pc_ = start_pc + count * 4u;
      cpu_.next_pc_ = cpu_.pc_ + 4u;
    }

    cpu_.cycles_ += consumed_cycles;
    cpu_.executing_step_ = false;
    g_diag_current_pc = cpu_.current_pc_;
    cpu_.rr4_diag_state_.prev_pc_for_diag = cpu_.current_pc_;

    result.cycles += consumed_cycles;
    result.instructions += count;
    ++stats_.native_block_entries;
    if (block.has_branch) {
      // Branch-tail blocks can also contain fast stores.
    } else if (block.has_store) {
      ++stats_.native_memory_block_entries;
    } else {
      ++stats_.native_alu_block_entries;
    }
    if (block.has_store) {
      stats_.native_memory_fastpath_stores += block.store_count;
    }
    stats_.native_instructions += count;
    stats_.optimized_instructions += count;
    stats_.native_cycles += consumed_cycles;
    stats_.executed_cycles += consumed_cycles;

    // Commit the I-cache state that the interpreter would have produced
    // while fetching instructions between these stores.
    for (u32 i = 0; i < sim_icache_count; ++i) {
      const SimIcacheLine &state = sim_icache[i];
      auto &line = cpu_.icache_[state.index];
      if (!state.valid) {
        line.valid = false;
        continue;
      }
      if (state.refilled || !line.valid || line.tag != state.tag) {
        line.tag = state.tag;
        for (u32 word = 0; word < 4u; ++word) {
          line.words[word] =
              cpu_.sys_->read32_instruction(state.tag + word * 4u);
        }
      }
      line.valid = true;
    }

    // Direct memory writes still invalidate any compiled code that physically
    // overlaps their targets. The CPU I-cache was modeled above, so use the
    // backend-only hook here rather than invalidating it a second time.
    const u32 completed_store_count = block.store_count;
    if (completed_store_count != 0u) {
      for (u32 i = 0; i < completed_store_count; ++i) {
        cpu_.notify_jit_code_write_only(store_addrs[i], 4u);
      }
    }
    return true;
  };
#endif

  while (result.cycles < max_cycles &&
         result.instructions < max_instructions) {
#if VIBESTATION_JIT_V2_X64
    if (!try_native()) {
      interpreter_step();
    }
#else
    interpreter_step();
#endif

    if (cpu_.sys_ != nullptr && cpu_.sys_->cpu_timing_boundary_requested()) {
      break;
    }
  }

  return result;
}

void CpuJitV2Backend::invalidate_range(u32 phys_or_normalized_addr,
                                       u32 size_bytes) {
  ++stats_.invalidation_queries;
  if (size_bytes == 0u || impl_->blocks.empty()) {
    return;
  }

  const u32 first = psx::mask_address(phys_or_normalized_addr);
  const u32 last =
      psx::mask_address(phys_or_normalized_addr + size_bytes - 1u);
  const u32 first_line = first & ~0x0Fu;
  const u32 last_line = last & ~0x0Fu;

  bool maybe_code = false;
  for (u32 page = first >> 12u; page <= (last >> 12u); ++page) {
    if (impl_->code_pages.find(page) != impl_->code_pages.end()) {
      maybe_code = true;
      break;
    }
  }
  if (!maybe_code) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  for (auto it = impl_->blocks.begin(); it != impl_->blocks.end();) {
    const u32 block_first = it->second.phys_start & ~0x0Fu;
    const u32 block_last = it->second.phys_end & ~0x0Fu;
    const bool overlaps = block_first <= last_line && block_last >= first_line;
    if (overlaps) {
      it = impl_->blocks.erase(it);
      ++stats_.invalidations;
      ++stats_.invalidation_blocks_invalidated;
    } else {
      ++it;
    }
  }
}

void CpuJitV2Backend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV2;
}

void CpuJitV2Backend::flush() {
  impl_->blocks.clear();
  impl_->code_pages.clear();
  stats_ = {};
  stats_.available = true;
  stats_.native_available = VIBESTATION_JIT_V2_X64 != 0;
  current_frame_ = 0;
}

CpuBackendStats CpuJitV2Backend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV2;
  out.native_available = VIBESTATION_JIT_V2_X64 != 0;
  out.block_count = static_cast<u32>(impl_->blocks.size());
  out.native_blocks = static_cast<u64>(impl_->blocks.size());
  out.native_code_bytes = impl_->blocks.size() * 4096u;
  return out;
}
