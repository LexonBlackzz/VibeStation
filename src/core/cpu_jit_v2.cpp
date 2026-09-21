#include "cpu_jit_v2.h"
#include "cpu_recompiler.h"
#include "system.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#if defined(VIBESTATION_ENABLE_X64_JIT) && \
    (defined(_M_X64) || defined(__x86_64__))
#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif
#include <xbyak/xbyak.h>
#define VIBESTATION_JIT_V2_X64 1
#if defined(_WIN32)
#include <windows.h>
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#else
#include <sys/mman.h>
#include <unistd.h>
#endif
#else
#define VIBESTATION_JIT_V2_X64 0
#endif

namespace {

enum class V2BlockKind : u8 {
  Inline,
  StepHelper,
};

enum class V2HelperReason : u8 {
  State,
  Icache,
  Irq,
  Unsupported,
  Memory,
  Budget,
  Internal,
};

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
  J,
  Jal,
  Jr,
  Jalr,
  Lw,
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
  u32 link_value = 0u;
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
    case 0x08: out.op = V2AluOp::Jr; return true;
    case 0x09: out.op = V2AluOp::Jalr; return true;
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
  case 0x02: out.op = V2AluOp::J; return true;
  case 0x03: out.op = V2AluOp::Jal; return true;
  case 0x04: out.op = V2AluOp::Beq; return true;
  case 0x05: out.op = V2AluOp::Bne; return true;
  case 0x09: out.op = V2AluOp::Addiu; return true;
  case 0x0A: out.op = V2AluOp::Slti; return true;
  case 0x0B: out.op = V2AluOp::Sltiu; return true;
  case 0x0C: out.op = V2AluOp::Andi; return true;
  case 0x0D: out.op = V2AluOp::Ori; return true;
  case 0x0E: out.op = V2AluOp::Xori; return true;
  case 0x0F: out.op = V2AluOp::Lui; return true;
  case 0x23: out.op = V2AluOp::Lw; return true;
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
  case V2AluOp::Jr:
  case V2AluOp::Jalr:
    return reg(inst.rs);
  case V2AluOp::Addiu:
  case V2AluOp::Slti:
  case V2AluOp::Sltiu:
  case V2AluOp::Andi:
  case V2AluOp::Ori:
  case V2AluOp::Xori:
  case V2AluOp::Lw:
    return reg(inst.rs);
  case V2AluOp::Sw:
  case V2AluOp::Beq:
  case V2AluOp::Bne:
    return reg(inst.rs) | reg(inst.rt);
  case V2AluOp::Nop:
  case V2AluOp::Lui:
  case V2AluOp::Clear:
  case V2AluOp::J:
  case V2AluOp::Jal:
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
  case V2AluOp::Jal:
    return 31u;
  case V2AluOp::Jalr:
    return inst.rd;
  case V2AluOp::Nop:
  case V2AluOp::Jr:
  case V2AluOp::J:
  case V2AluOp::Lw:
  case V2AluOp::Sw:
  case V2AluOp::Beq:
  case V2AluOp::Bne:
    return 0u;
  }
  return 0u;
}

void count_v2_unsupported_opcode(CpuBackendStats &stats, u32 bits) {
  const u32 primary = bits >> 26u;
  switch (primary) {
  case 0x23:
    ++stats.jit_v2_unsupported_lw;
    return;
  case 0x20: case 0x21: case 0x22: case 0x24: case 0x25: case 0x26:
    ++stats.jit_v2_unsupported_other_load;
    return;
  case 0x12:
    ++stats.jit_v2_unsupported_cop2;
    return;
  case 0x10:
    ++stats.jit_v2_unsupported_cop0;
    return;
  case 0x02: case 0x03:
    ++stats.jit_v2_unsupported_jump;
    return;
  case 0x01: case 0x06: case 0x07:
    ++stats.jit_v2_unsupported_other_branch;
    return;
  case 0x28: case 0x29: case 0x2A: case 0x2E:
    ++stats.jit_v2_unsupported_store;
    return;
  case 0x00: {
    const u32 funct = bits & 0x3Fu;
    if (funct == 0x08 || funct == 0x09 || funct == 0x0C || funct == 0x0D) {
      ++stats.jit_v2_unsupported_special_control;
      return;
    }
    if (funct == 0x10 || funct == 0x11 || funct == 0x12 || funct == 0x13 ||
        funct == 0x18 || funct == 0x19 || funct == 0x1A || funct == 0x1B) {
      ++stats.jit_v2_unsupported_muldiv;
      return;
    }
    break;
  }
  default:
    break;
  }
  ++stats.jit_v2_unsupported_other;
}

bool is_v2_branch(V2AluOp op) {
  return op == V2AluOp::Beq || op == V2AluOp::Bne;
}

bool is_v2_fixed_jump(V2AluOp op) {
  return op == V2AluOp::J || op == V2AluOp::Jal;
}

bool is_v2_dynamic_jump(V2AluOp op) {
  return op == V2AluOp::Jr || op == V2AluOp::Jalr;
}

bool is_v2_alu_only(V2AluOp op) {
  return op != V2AluOp::Lw && op != V2AluOp::Sw &&
         !is_v2_branch(op) && !is_v2_fixed_jump(op) &&
         !is_v2_dynamic_jump(op);
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
  u8 *load_ptr = nullptr;
  u32 load_value = 0u;
  u32 dynamic_target = 0u;
};

using V2NativeFn = u32 (*)(u32 *, const V2NativeRuntime *);
using V2StepHelperFn = u32 (*)(Cpu *);

class V2CodeArena {
public:
  V2CodeArena() = default;
  ~V2CodeArena() { reset(); }

  V2CodeArena(const V2CodeArena &) = delete;
  V2CodeArena &operator=(const V2CodeArena &) = delete;

  void *copy_code(const void *src, size_t size) {
    if (src == nullptr || size == 0u) {
      return nullptr;
    }

    constexpr size_t kAlignment = 16u;
    constexpr size_t kDefaultChunk = 4u * 1024u * 1024u;

    if (chunks_.empty() ||
        align_up(chunks_.back().used, kAlignment) + size >
            chunks_.back().capacity) {
      const size_t page_aligned =
          align_up(size, static_cast<size_t>(4096u));
      const size_t requested =
          page_aligned > kDefaultChunk ? page_aligned : kDefaultChunk;
      Chunk chunk{};
      chunk.base = allocate_executable(requested);
      if (chunk.base == nullptr) {
        return nullptr;
      }
      chunk.capacity = requested;
      chunks_.push_back(chunk);
    }

    Chunk &chunk = chunks_.back();
    const size_t offset = align_up(chunk.used, kAlignment);
    u8 *dst = chunk.base + offset;
    std::memcpy(dst, src, size);
    chunk.used = offset + size;
    bytes_used_ += size;

#if defined(_WIN32)
    FlushInstructionCache(GetCurrentProcess(), dst, size);
#elif defined(__GNUC__) || defined(__clang__)
    __builtin___clear_cache(reinterpret_cast<char *>(dst),
                            reinterpret_cast<char *>(dst + size));
#endif
    return dst;
  }

  void reset() {
    for (Chunk &chunk : chunks_) {
      free_executable(chunk.base, chunk.capacity);
    }
    chunks_.clear();
    bytes_used_ = 0u;
  }

  size_t bytes_used() const { return bytes_used_; }

private:
  struct Chunk {
    u8 *base = nullptr;
    size_t capacity = 0u;
    size_t used = 0u;
  };

  static size_t align_up(size_t value, size_t alignment) {
    return (value + alignment - 1u) & ~(alignment - 1u);
  }

  static u8 *allocate_executable(size_t size) {
#if defined(_WIN32)
    return static_cast<u8 *>(VirtualAlloc(nullptr, size,
                                         MEM_RESERVE | MEM_COMMIT,
                                         PAGE_EXECUTE_READWRITE));
#else
    void *ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE | PROT_EXEC,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    return ptr == MAP_FAILED ? nullptr : static_cast<u8 *>(ptr);
#endif
  }

  static void free_executable(u8 *ptr, size_t size) {
    if (ptr == nullptr) {
      return;
    }
#if defined(_WIN32)
    (void)size;
    VirtualFree(ptr, 0, MEM_RELEASE);
#else
    munmap(ptr, size);
#endif
  }

  std::vector<Chunk> chunks_;
  size_t bytes_used_ = 0u;
};

u32 v2_step_helper(Cpu *cpu) {
  return cpu != nullptr ? cpu->step() : 0u;
}

std::unique_ptr<Xbyak::CodeGenerator> compile_step_helper_trampoline() {
  auto code = std::make_unique<Xbyak::CodeGenerator>(256);
#if defined(_WIN32)
  code->sub(code->rsp, 40);
#else
  code->sub(code->rsp, 8);
#endif
  code->mov(code->rax,
            reinterpret_cast<size_t>(&v2_step_helper));
  code->call(code->rax);
#if defined(_WIN32)
  code->add(code->rsp, 40);
#else
  code->add(code->rsp, 8);
#endif
  code->ret();
  code->ready();
  return code;
}

V2StepHelperFn install_step_helper(V2CodeArena &arena) {
  auto code = compile_step_helper_trampoline();
  if (!code) {
    return nullptr;
  }
  void *entry = arena.copy_code(code->getCode(), code->getSize());
  return reinterpret_cast<V2StepHelperFn>(entry);
}


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

    case V2AluOp::J:
      break;

    case V2AluOp::Jal:
      code->mov(code->eax, inst.link_value);
      emit_write_guest(*code, cached, 31u, code->eax, dirty);
      break;

    case V2AluOp::Jr:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      code->mov(code->dword[code->r14 +
          static_cast<int>(offsetof(V2NativeRuntime, dynamic_target))],
          code->eax);
      break;

    case V2AluOp::Jalr:
      // Capture the target before writing the link register. This matters for
      // the legal but nasty rd == rs case.
      emit_read_guest(*code, code->eax, cached, inst.rs);
      code->mov(code->dword[code->r14 +
          static_cast<int>(offsetof(V2NativeRuntime, dynamic_target))],
          code->eax);
      code->mov(code->eax, inst.link_value);
      emit_write_guest(*code, cached, inst.rd, code->eax, dirty);
      break;

    case V2AluOp::Lw: {
      code->mov(code->rax, code->ptr[code->r14 +
          static_cast<int>(offsetof(V2NativeRuntime, load_ptr))]);
      code->mov(code->eax, code->dword[code->rax]);
      code->mov(code->dword[code->r14 +
          static_cast<int>(offsetof(V2NativeRuntime, load_value))], code->eax);
      break;
    }

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
    V2BlockKind kind = V2BlockKind::Inline;
    u32 start_pc = 0;
    u32 phys_start = 0;
    u32 phys_end = 0;
    u32 instruction_count = 0;
    u32 base_cycles = 0;
    u32 store_count = 0;
    u8 load_rs = 0u;
    u8 load_rt = 0u;
    s32 load_simm = 0;
    u32 load_index = 0u;
    u32 branch_index = 0;
    u32 branch_target = 0;
    u32 jump_index = 0;
    u32 jump_target = 0;
    bool jump_dynamic = false;
    bool has_store = false;
    bool has_load = false;
    bool has_branch = false;
    bool has_jump = false;
    std::array<u32, 16> words{};
    std::array<u8, 6> cached_regs{};
    std::array<u8, 8> store_rs{};
    std::array<s32, 8> store_simm{};
    std::array<u8, 8> store_instruction_index{};
#if VIBESTATION_JIT_V2_X64
    V2NativeFn fn = nullptr;
    size_t code_size = 0u;
#endif
  };

  static constexpr size_t kDispatchCacheSize = 1u << 16u;
  struct DispatchCacheEntry {
    u32 pc = 0u;
    Block *block = nullptr;
  };

  Block *lookup_dispatch(u32 pc) {
    DispatchCacheEntry &entry =
        dispatch_cache[(pc >> 2u) & (kDispatchCacheSize - 1u)];
    return entry.block != nullptr && entry.pc == pc ? entry.block : nullptr;
  }

  void remember_dispatch(Block &block) {
    DispatchCacheEntry &entry =
        dispatch_cache[(block.start_pc >> 2u) & (kDispatchCacheSize - 1u)];
    entry.pc = block.start_pc;
    entry.block = &block;
  }

  void forget_dispatch(u32 pc) {
    DispatchCacheEntry &entry =
        dispatch_cache[(pc >> 2u) & (kDispatchCacheSize - 1u)];
    if (entry.block != nullptr && entry.pc == pc) {
      entry = {};
    }
  }

  void clear_dispatch() { dispatch_cache.fill({}); }

  std::array<DispatchCacheEntry, kDispatchCacheSize> dispatch_cache{};

#if VIBESTATION_JIT_V2_X64
  V2CodeArena arena;
  V2StepHelperFn step_helper_fn = nullptr;
#endif
  std::unordered_map<u32, Block> blocks;
  // PCs which cannot currently form even the minimum V2 native block.
  // These are invalidated with code writes instead of being recompiled on
  // every execution.
  std::unordered_set<u32> rejected_pcs;
  std::unordered_set<u32> rejected_pages;
  // Conservative ever-compiled page set. Entries are intentionally retained
  // until flush: stale positives cost only an occasional scan, while there can
  // never be a false negative that misses self-modifying code.
  std::unordered_set<u32> code_pages;
};

CpuJitV2Backend::CpuJitV2Backend(Cpu &cpu)
    : cpu_(cpu), impl_(std::make_unique<Impl>()) {
  stats_.available = true;
#if VIBESTATION_JIT_V2_X64
  impl_->step_helper_fn = install_step_helper(impl_->arena);
  stats_.native_available = impl_->step_helper_fn != nullptr;
#else
  stats_.native_available = false;
#endif
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
#if VIBESTATION_JIT_V2_X64
  stats_.native_available = impl_->step_helper_fn != nullptr;
#else
  stats_.native_available = false;
#endif

  auto interpreter_step = [&]() {
    const u32 consumed = cpu_.step();
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.interpreter_fallback_steps;
    ++stats_.fallback_instructions;
    stats_.executed_cycles += consumed;
  };


#if VIBESTATION_JIT_V2_X64
  auto helper_step = [&](V2HelperReason reason) -> bool {
    if (impl_->step_helper_fn == nullptr ||
        result.instructions >= max_instructions ||
        result.cycles >= max_cycles) {
      return false;
    }

    const u32 consumed = impl_->step_helper_fn(&cpu_);
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.native_block_entries;
    ++stats_.jit_v2_helper_entries;
    ++stats_.jit_v2_helper_instructions;
    switch (reason) {
    case V2HelperReason::State: ++stats_.jit_v2_helper_state; break;
    case V2HelperReason::Icache: ++stats_.jit_v2_helper_icache; break;
    case V2HelperReason::Irq: ++stats_.jit_v2_helper_irq; break;
    case V2HelperReason::Unsupported: ++stats_.jit_v2_helper_unsupported; break;
    case V2HelperReason::Memory: ++stats_.jit_v2_helper_memory; break;
    case V2HelperReason::Budget: ++stats_.jit_v2_helper_budget; break;
    case V2HelperReason::Internal: ++stats_.jit_v2_helper_internal; break;
    }
    ++stats_.native_instructions;
    ++stats_.optimized_instructions;
    stats_.native_cycles += consumed;
    stats_.executed_cycles += consumed;
    return true;
  };

  auto state_allows_native = [&]() {
    if (g_trace_cpu || g_cpu_deep_diagnostics || g_log_fmv_diagnostics) {
      ++stats_.jit_v2_state_diagnostics;
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
      ++stats_.jit_v2_state_branch_delay;
      return false;
    }
    if (cpu_.load_.reg != 0u || cpu_.next_load_.reg != 0u) {
      ++stats_.jit_v2_state_load_delay;
      return false;
    }
    if (cpu_.next_pc_ != cpu_.pc_ + 4u) {
      ++stats_.jit_v2_state_pc;
      return false;
    }
    return true;
  };

  auto try_native = [&]() -> bool {
    if (!state_allows_native()) {
      return helper_step(V2HelperReason::State);
    }

    // Match the interpreter's between-instruction hardware IRQ sampling.
    if (cpu_.sys_->irq_pending()) {
      cpu_.cop0_cause_ |= (1u << 10);
    } else {
      cpu_.cop0_cause_ &= ~(1u << 10);
    }
    if (cpu_.check_irq()) {
      return helper_step(V2HelperReason::Irq);
    }

    const u32 start_pc = cpu_.pc_;
    if (!cpu_.instruction_cacheable(start_pc)) {
      return helper_step(V2HelperReason::Icache);
    }

    const u32 index = (start_pc >> 4u) & 0xFFu;
    const u32 word_index = (start_pc >> 2u) & 0x03u;
    const u32 expected_tag = psx::mask_address(start_pc) & ~0x0Fu;
    auto &line = cpu_.icache_[index];
    if (!line.valid || line.tag != expected_tag) {
      return helper_step(V2HelperReason::Icache);
    }

    if (impl_->rejected_pcs.find(start_pc) != impl_->rejected_pcs.end()) {
      ++stats_.cache_hits;
      return helper_step(V2HelperReason::Unsupported);
    }

    Impl::Block *block_ptr = impl_->lookup_dispatch(start_pc);
    if (block_ptr != nullptr) {
      ++stats_.cache_hits;
    } else {
      auto found = impl_->blocks.find(start_pc);
      if (found != impl_->blocks.end()) {
        block_ptr = &found->second;
        impl_->remember_dispatch(*block_ptr);
        ++stats_.cache_hits;
      }
    }

    if (block_ptr != nullptr) {
      bool coherent = true;
      for (u32 i = 0; i < block_ptr->instruction_count; ++i) {
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
            inst_line.words[inst_word] != block_ptr->words[i]) {
          coherent = false;
          break;
        }
      }
      if (!coherent) {
        impl_->forget_dispatch(start_pc);
        impl_->blocks.erase(start_pc);
        block_ptr = nullptr;
      }
    }

    if (block_ptr == nullptr) {
      ++stats_.cache_misses;
      ++stats_.native_compile_attempts;

      std::vector<V2DecodedInstruction> decoded;
      std::array<u32, 16> words{};
      constexpr u32 kMaxV2Instructions = 16u;
      u32 written_mask = 0u;
      u32 store_count = 0u;
      bool has_store = false;
      bool has_load = false;
      u8 load_rs = 0u;
      u8 load_rt = 0u;
      s32 load_simm = 0;
      u32 load_index = 0u;
      bool has_branch = false;
      u32 branch_index = 0u;
      s32 branch_simm = 0;
      bool has_jump = false;
      bool jump_dynamic = false;
      u32 jump_index = 0u;
      u32 jump_target = 0u;
      std::array<u8, 8> store_rs{};
      std::array<s32, 8> store_simm{};
      std::array<u8, 8> store_instruction_index{};
      bool fetch_stopped_on_unsupported = false;
      u32 unsupported_bits = 0u;

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
        if (!decode_v2_alu(bits, inst)) {
          fetch_stopped_on_unsupported = true;
          unsupported_bits = bits;
          return false;
        }
        return true;
      };

      for (u32 i = 0; i < kMaxV2Instructions; ++i) {
        V2DecodedInstruction inst{};
        u32 bits = 0u;
        if (!fetch_decoded(i, inst, bits)) {
          break;
        }

        if (is_v2_fixed_jump(inst.op) || is_v2_dynamic_jump(inst.op)) {
          // J/JAL/JR/JALR are always taken and include the architectural delay
          // slot. Link instructions write their destination before the delay
          // slot, while JR/JALR capture the dynamic target before that slot.
          if (i + 1u >= kMaxV2Instructions) {
            break;
          }
          V2DecodedInstruction delay{};
          u32 delay_bits = 0u;
          if (!fetch_decoded(i + 1u, delay, delay_bits) ||
              !is_v2_alu_only(delay.op)) {
            break;
          }

          const u32 jump_pc = start_pc + i * 4u;
          inst.link_value = jump_pc + 8u;
          has_jump = true;
          jump_dynamic = is_v2_dynamic_jump(inst.op);
          jump_index = static_cast<u32>(decoded.size());
          if (!jump_dynamic) {
            jump_target = ((jump_pc + 4u) & 0xF0000000u) |
                          ((bits & 0x03FFFFFFu) << 2u);
          }
          decoded.push_back(inst);
          words[jump_index] = bits;
          decoded.push_back(delay);
          words[jump_index + 1u] = delay_bits;
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

        if (inst.op == V2AluOp::Lw) {
          // First load tier: execute one aligned direct load as the final
          // instruction in the block. Materialize the R3000A pending load at
          // block exit so the following guest instruction still observes the
          // old register value.
          if (inst.rs != 0u && (written_mask & (1u << inst.rs)) != 0u) {
            break;
          }
          has_load = true;
          load_rs = inst.rs;
          load_rt = inst.rt;
          load_simm = inst.simm;
          load_index = static_cast<u32>(decoded.size());
          decoded.push_back(inst);
          words[decoded.size() - 1u] = bits;
          break;
        }

        if (inst.op == V2AluOp::Sw || inst.op == V2AluOp::Lw) {
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

      if (decoded.empty()) {
        if (!fetch_stopped_on_unsupported) {
          return helper_step(V2HelperReason::Icache);
        }

        // Universal baseline: an instruction V2 does not lower yet still gets
        // a compiled block identity. Its entry is the single shared generated
        // Cpu::step() trampoline, so ordinary executable code can no longer
        // fail compilation merely because an opcode is not inline-native yet.
        Impl::Block helper_block{};
        helper_block.kind = V2BlockKind::StepHelper;
        helper_block.start_pc = start_pc;
        helper_block.phys_start = psx::mask_address(start_pc);
        helper_block.phys_end = helper_block.phys_start + 3u;
        helper_block.instruction_count = 1u;
        helper_block.words[0] = line.words[word_index];
        impl_->code_pages.insert(helper_block.phys_start >> 12u);
        auto inserted =
            impl_->blocks.emplace(start_pc, std::move(helper_block));
        block_ptr = &inserted.first->second;
        impl_->remember_dispatch(*block_ptr);
        ++stats_.native_compile_successes;
        ++stats_.native_blocks_compiled;
        ++stats_.jit_v2_helper_blocks_compiled;
        (void)unsupported_bits;
      }

      if (block_ptr == nullptr) {
      Impl::Block block{};
      block.start_pc = start_pc;
      block.instruction_count = static_cast<u32>(decoded.size());
      block.phys_start = psx::mask_address(start_pc);
      block.phys_end =
          psx::mask_address(start_pc + block.instruction_count * 4u - 1u);
      block.words = words;
      block.cached_regs = choose_cached_regs(decoded);
      block.has_store = has_store;
      block.has_load = has_load;
      block.has_branch = has_branch;
      block.has_jump = has_jump;
      block.store_count = store_count;
      block.load_rs = load_rs;
      block.load_rt = load_rt;
      block.load_simm = load_simm;
      block.load_index = load_index;
      block.store_rs = store_rs;
      block.store_simm = store_simm;
      block.store_instruction_index = store_instruction_index;
      block.branch_index = branch_index;
      block.jump_index = jump_index;
      block.jump_target = jump_target;
      block.jump_dynamic = jump_dynamic;
      if (has_branch) {
        const u32 branch_pc = start_pc + branch_index * 4u;
        block.branch_target =
            branch_pc + 4u + (static_cast<u32>(branch_simm) << 2u);
      }
      for (const auto &inst : decoded) {
        if (inst.op == V2AluOp::Sw || inst.op == V2AluOp::Lw ||
            inst.op == V2AluOp::J || inst.op == V2AluOp::Jal ||
            inst.op == V2AluOp::Jr || inst.op == V2AluOp::Jalr) {
          block.base_cycles += 2u;
        } else {
          // Conditional branches cost one cycle when not taken and gain one
          // more dynamically when taken. All currently-native ALU ops cost 1.
          block.base_cycles += 1u;
        }
      }

      auto generated = compile_native_alu(decoded, block.cached_regs);
      if (!generated) {
        ++stats_.native_compile_failures;
        impl_->rejected_pcs.insert(start_pc);
        impl_->rejected_pages.insert(psx::mask_address(start_pc) >> 12u);
        return false;
      }
      block.code_size = generated->getSize();
      void *entry =
          impl_->arena.copy_code(generated->getCode(), block.code_size);
      if (entry == nullptr) {
        ++stats_.native_compile_failures;
        impl_->rejected_pcs.insert(start_pc);
        impl_->rejected_pages.insert(psx::mask_address(start_pc) >> 12u);
        return false;
      }
      block.fn = reinterpret_cast<V2NativeFn>(entry);
      const u32 block_phys_first = block.phys_start;
      const u32 block_phys_last = block.phys_end;
      for (u32 page = block_phys_first >> 12u;
           page <= (block_phys_last >> 12u); ++page) {
        impl_->code_pages.insert(page);
      }
      auto inserted = impl_->blocks.emplace(start_pc, std::move(block));
      block_ptr = &inserted.first->second;
      impl_->remember_dispatch(*block_ptr);
      ++stats_.native_compile_successes;
      ++stats_.native_blocks_compiled;
      if (has_branch || has_jump) {
        ++stats_.native_branch_tail_blocks_compiled;
      } else if (has_store || has_load) {
        ++stats_.native_memory_blocks_compiled;
      } else {
        ++stats_.native_alu_blocks_compiled;
      }
      }
    }

    if (block_ptr == nullptr) {
      return helper_step(V2HelperReason::Internal);
    }
    Impl::Block &block = *block_ptr;
    if (block.instruction_count == 0u) {
      return helper_step(V2HelperReason::Internal);
    }
    if (block.kind == V2BlockKind::StepHelper) {
      count_v2_unsupported_opcode(stats_, block.words[0]);
      return helper_step(V2HelperReason::Unsupported);
    }
    if (block.fn == nullptr) {
      return helper_step(V2HelperReason::Internal);
    }

    if ((block.has_branch || block.has_jump) &&
        g_cpu_backend_compare_irq_on_branch) {
      return helper_step(V2HelperReason::Irq);
    }

    V2NativeRuntime runtime{};
    std::array<u32, 8> store_addrs{};
    u32 main_ram_store_count = 0u;
    u32 load_addr = 0u;
    u32 main_ram_load_count = 0u;

    if (block.has_load) {
      const u32 base = block.load_rs == 0u ? 0u : cpu_.gpr_[block.load_rs];
      load_addr = base + static_cast<u32>(block.load_simm);
      if ((load_addr & 3u) != 0u) {
        return helper_step(V2HelperReason::Memory);
      }
      const u32 phys = psx::mask_address(load_addr);
      u8 *const main_ram = cpu_.sys_->jit_main_ram_data_mut();
      u8 *const scratchpad = cpu_.sys_->jit_scratchpad_data_mut();
      if (phys < psx::RAM_SIZE && main_ram != nullptr) {
        runtime.load_ptr = main_ram + phys;
        main_ram_load_count = 1u;
      } else if (phys >= 0x1F800000u && phys < 0x1F801000u &&
                 scratchpad != nullptr) {
        runtime.load_ptr = scratchpad +
            ((phys - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u));
      } else {
        return helper_step(V2HelperReason::Memory);
      }
    }

    if (block.has_store) {
      // Direct writes intentionally bypass System::write32. Anything that
      // requires tracing, watchpoints, MMIO semantics or isolated-cache store
      // behavior leaves the native tier before generated code is entered.
      if (g_trace_ram || g_trace_bus || g_ram_watch_diagnostics ||
          (cpu_.cop0_sr_ & (1u << 16)) != 0u) {
        return helper_step(V2HelperReason::Memory);
      }

      u8 *const main_ram = cpu_.sys_->jit_main_ram_data_mut();
      u8 *const scratchpad = cpu_.sys_->jit_scratchpad_data_mut();
      if (main_ram == nullptr || scratchpad == nullptr) {
        return helper_step(V2HelperReason::Memory);
      }

      for (u32 i = 0; i < block.store_count; ++i) {
        const u8 base_reg = block.store_rs[i];
        const u32 base = base_reg == 0u ? 0u : cpu_.gpr_[base_reg];
        const u32 addr = base + static_cast<u32>(block.store_simm[i]);
        store_addrs[i] = addr;

        if ((addr & 3u) != 0u) {
          return helper_step(V2HelperReason::Memory);
        }

        const u32 phys = psx::mask_address(addr);
        const u64 store_end = static_cast<u64>(phys) + 3u;
        const bool touches_current_code =
            phys <= block.phys_end &&
            store_end >= static_cast<u64>(block.phys_start);
        if (touches_current_code) {
          return helper_step(V2HelperReason::Memory);
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

        return helper_step(V2HelperReason::Memory);
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
        block.base_cycles + main_ram_store_count + main_ram_load_count * 4u +
        fetch_penalty + (block.has_branch ? 1u : 0u);
    if (block.instruction_count > remaining_instructions ||
        worst_cycles > remaining_cycles) {
      return helper_step(V2HelperReason::Budget);
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
        block.base_cycles + main_ram_store_count + main_ram_load_count * 4u +
        fetch_penalty +
        ((block.has_branch && branch_taken) ? 1u : 0u);

    if (block.has_jump) {
      const u32 jump_pc = start_pc + block.jump_index * 4u;
      const u32 delay_pc = jump_pc + 4u;
      cpu_.current_pc_ = delay_pc;
      cpu_.pc_ = block.jump_dynamic ? runtime.dynamic_target
                                    : block.jump_target;
      cpu_.next_pc_ = cpu_.pc_ + 4u;
      cpu_.in_delay_slot_ = true;
      cpu_.active_branch_pc_ = jump_pc;
      ++stats_.native_branch_tail_entries;
      ++stats_.native_branch_taken;
    } else if (block.has_branch) {
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

    if (block.has_load) {
      if (block.load_rt != 0u) {
        const u32 load_pc = start_pc + block.load_index * 4u;
        cpu_.load_ = {block.load_rt, runtime.load_value, load_pc, load_addr};
      } else {
        cpu_.load_ = {};
      }
      cpu_.next_load_ = {};
      ++stats_.native_memory_fastpath_loads;
    }

    cpu_.cycles_ += consumed_cycles;
    cpu_.executing_step_ = false;
    g_diag_current_pc = cpu_.current_pc_;
    cpu_.rr4_diag_state_.prev_pc_for_diag = cpu_.current_pc_;

    result.cycles += consumed_cycles;
    result.instructions += count;
    ++stats_.native_block_entries;
    if (block.has_branch || block.has_jump) {
      // Control-flow blocks can also contain fast memory operations.
    } else if (block.has_store || block.has_load) {
      ++stats_.native_memory_block_entries;
    } else {
      ++stats_.native_alu_block_entries;
    }
    if (block.has_store) {
      stats_.native_memory_fastpath_stores += block.store_count;
    }
    stats_.native_instructions += count;
    stats_.jit_v2_inline_instructions += count;
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
      // Universal V2 should only get here for an internal codegen/allocation
      // failure. Keep correctness with the generated step trampoline when
      // available; the raw interpreter is now the last-resort safety net.
      if (!helper_step(V2HelperReason::Internal)) {
        interpreter_step();
      }
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
  if (size_bytes == 0u) {
    return;
  }

  const u32 first = psx::mask_address(phys_or_normalized_addr);
  const u32 last =
      psx::mask_address(phys_or_normalized_addr + size_bytes - 1u);
  const u32 first_page = first >> 12u;
  const u32 last_page = last >> 12u;

  bool maybe_rejected_code = false;
  for (u32 page = first_page; page <= last_page; ++page) {
    if (impl_->rejected_pages.find(page) != impl_->rejected_pages.end()) {
      maybe_rejected_code = true;
      break;
    }
  }
  if (maybe_rejected_code) {
    for (auto it = impl_->rejected_pcs.begin();
         it != impl_->rejected_pcs.end();) {
      const u32 page = psx::mask_address(*it) >> 12u;
      if (page >= first_page && page <= last_page) {
        it = impl_->rejected_pcs.erase(it);
      } else {
        ++it;
      }
    }
    for (u32 page = first_page; page <= last_page; ++page) {
      impl_->rejected_pages.erase(page);
    }
  }

  if (impl_->blocks.empty()) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  const u32 first_line = first & ~0x0Fu;
  const u32 last_line = last & ~0x0Fu;

  bool maybe_code = false;
  for (u32 page = first_page; page <= last_page; ++page) {
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
      impl_->forget_dispatch(it->second.start_pc);
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
  impl_->clear_dispatch();
  impl_->blocks.clear();
  impl_->rejected_pcs.clear();
  impl_->rejected_pages.clear();
  impl_->code_pages.clear();
#if VIBESTATION_JIT_V2_X64
  impl_->arena.reset();
  impl_->step_helper_fn = install_step_helper(impl_->arena);
#endif
  stats_ = {};
  stats_.available = true;
#if VIBESTATION_JIT_V2_X64
  stats_.native_available = impl_->step_helper_fn != nullptr;
#else
  stats_.native_available = false;
#endif
  current_frame_ = 0;
}

CpuBackendStats CpuJitV2Backend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV2;
#if VIBESTATION_JIT_V2_X64
  out.native_available = impl_->step_helper_fn != nullptr;
#else
  out.native_available = false;
#endif
  out.block_count = static_cast<u32>(impl_->blocks.size());
  out.native_blocks = static_cast<u64>(impl_->blocks.size());
#if VIBESTATION_JIT_V2_X64
  out.native_code_bytes = impl_->arena.bytes_used();
#else
  out.native_code_bytes = 0u;
#endif
  return out;
}
