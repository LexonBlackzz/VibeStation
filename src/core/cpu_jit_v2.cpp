#include "cpu_jit_v2.h"
#include "system.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
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
#if VIBESTATION_JIT_V2_X64
    std::unique_ptr<Xbyak::CodeGenerator> code;
    V2NativeFn fn = nullptr;
#endif
  };

  std::unordered_map<u32, Block> blocks;
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
    if (cpu_.pending_delay_slot_ || cpu_.pending_branch_taken_ ||
        cpu_.pending_branch_pc_ != 0u) {
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
      constexpr u32 kMaxV2AluInstructions = 16u;
      for (u32 i = 0; i < kMaxV2AluInstructions; ++i) {
        const u32 inst_pc = start_pc + i * 4u;
        if (!cpu_.instruction_cacheable(inst_pc)) {
          break;
        }
        const u32 inst_index = (inst_pc >> 4u) & 0xFFu;
        const u32 inst_word = (inst_pc >> 2u) & 0x03u;
        const u32 inst_tag = psx::mask_address(inst_pc) & ~0x0Fu;
        const auto &inst_line = cpu_.icache_[inst_index];
        if (!inst_line.valid || inst_line.tag != inst_tag) {
          break;
        }

        V2DecodedInstruction inst{};
        const u32 bits = inst_line.words[inst_word];
        if (!decode_v2_alu(bits, inst)) {
          break;
        }
        decoded.push_back(inst);
        words[i] = bits;
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
      block.code = compile_native_alu(decoded, block.cached_regs);
      if (!block.code) {
        ++stats_.native_compile_failures;
        return false;
      }
      block.fn = block.code->getCode<V2NativeFn>();
      auto inserted = impl_->blocks.emplace(start_pc, std::move(block));
      found = inserted.first;
      ++stats_.native_compile_successes;
      ++stats_.native_blocks_compiled;
      ++stats_.native_alu_blocks_compiled;
    } else {
      ++stats_.cache_hits;
    }

    Impl::Block &block = found->second;
    if (block.fn == nullptr || block.instruction_count == 0u) {
      return false;
    }

    const u32 remaining_cycles = max_cycles - result.cycles;
    const u32 remaining_instructions = max_instructions - result.instructions;
    if (block.instruction_count > remaining_cycles ||
        block.instruction_count > remaining_instructions) {
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

    block.fn(cpu_.gpr_);

    const u32 count = block.instruction_count;
    cpu_.current_pc_ = start_pc + (count - 1u) * 4u;
    cpu_.pc_ = start_pc + count * 4u;
    cpu_.next_pc_ = cpu_.pc_ + 4u;
    cpu_.cycles_ += count;
    cpu_.executing_step_ = false;
    g_diag_current_pc = cpu_.current_pc_;
    cpu_.rr4_diag_state_.prev_pc_for_diag = cpu_.current_pc_;

    result.cycles += count;
    result.instructions += count;
    ++stats_.native_block_entries;
    ++stats_.native_alu_block_entries;
    stats_.native_instructions += count;
    stats_.optimized_instructions += count;
    stats_.native_cycles += count;
    stats_.executed_cycles += count;
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
