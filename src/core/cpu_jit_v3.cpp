#include "cpu_jit_v3.h"
#include "cpu_recompiler.h"
#include "jit_code_page_bitmap.h"
#include "system.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <exception>
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
#define VIBESTATION_JIT_V3_X64 1
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
#define VIBESTATION_JIT_V3_X64 0
#endif

namespace {

enum class V3BlockKind : u8 {
  Inline,
  StepHelper,
};

enum class V3HelperReason : u8 {
  State,
  Icache,
  Irq,
  Unsupported,
  Memory,
  Budget,
  Internal,
};

enum class V3AluOp : u8 {
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
  Lb,
  Lbu,
  Lh,
  Lhu,
  Lw,
  Sw,
  Beq,
  Bne,
};

struct V3DecodedInstruction {
  V3AluOp op = V3AluOp::Nop;
  u8 rs = 0;
  u8 rt = 0;
  u8 rd = 0;
  u8 shamt = 0;
  u16 imm = 0;
  s32 simm = 0;
  u32 link_value = 0u;
};

bool decode_v3_alu(u32 bits, V3DecodedInstruction &out) {
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
      out.op = V3AluOp::Nop;
      return true;
    }
    switch (bits & 0x3Fu) {
    case 0x00: out.op = V3AluOp::Sll; return true;
    case 0x02: out.op = V3AluOp::Srl; return true;
    case 0x03: out.op = V3AluOp::Sra; return true;
    case 0x08: out.op = V3AluOp::Jr; return true;
    case 0x09: out.op = V3AluOp::Jalr; return true;
    case 0x0F: out.op = V3AluOp::Nop; return true; // SYNC
    case 0x14:
    case 0x1C:
    case 0x28:
    case 0x29:
      out.op = V3AluOp::Nop;
      return true;
    case 0x21:
    case 0x2D:
      out.op = V3AluOp::Addu;
      return true;
    case 0x23:
    case 0x2F:
      out.op = V3AluOp::Subu;
      return true;
    case 0x24: out.op = V3AluOp::And; return true;
    case 0x25: out.op = V3AluOp::Or; return true;
    case 0x26: out.op = V3AluOp::Xor; return true;
    case 0x27: out.op = V3AluOp::Nor; return true;
    case 0x2A: out.op = V3AluOp::Slt; return true;
    case 0x2B: out.op = V3AluOp::Sltu; return true;
    case 0x38: out.op = V3AluOp::Clear; return true;
    default: return false;
    }
  }

  switch (primary) {
  case 0x02: out.op = V3AluOp::J; return true;
  case 0x03: out.op = V3AluOp::Jal; return true;
  case 0x04: out.op = V3AluOp::Beq; return true;
  case 0x05: out.op = V3AluOp::Bne; return true;
  case 0x09: out.op = V3AluOp::Addiu; return true;
  case 0x0A: out.op = V3AluOp::Slti; return true;
  case 0x0B: out.op = V3AluOp::Sltiu; return true;
  case 0x0C: out.op = V3AluOp::Andi; return true;
  case 0x0D: out.op = V3AluOp::Ori; return true;
  case 0x0E: out.op = V3AluOp::Xori; return true;
  case 0x0F: out.op = V3AluOp::Lui; return true;
  case 0x20: out.op = V3AluOp::Lb; return true;
  case 0x21: out.op = V3AluOp::Lh; return true;
  case 0x23: out.op = V3AluOp::Lw; return true;
  case 0x24: out.op = V3AluOp::Lbu; return true;
  case 0x25: out.op = V3AluOp::Lhu; return true;
  case 0x2B: out.op = V3AluOp::Sw; return true;
  default: return false;
  }
}

bool is_v3_load(V3AluOp op) {
  return op == V3AluOp::Lb || op == V3AluOp::Lbu ||
         op == V3AluOp::Lh || op == V3AluOp::Lhu ||
         op == V3AluOp::Lw;
}

u32 v3_load_alignment_mask(V3AluOp op) {
  switch (op) {
  case V3AluOp::Lh:
  case V3AluOp::Lhu:
    return 1u;
  case V3AluOp::Lw:
    return 3u;
  default:
    return 0u;
  }
}

u32 read_mask(const V3DecodedInstruction &inst) {
  auto reg = [](u8 r) { return r == 0u ? 0u : (1u << r); };
  switch (inst.op) {
  case V3AluOp::Sll:
  case V3AluOp::Srl:
  case V3AluOp::Sra:
    return reg(inst.rt);
  case V3AluOp::Addu:
  case V3AluOp::Subu:
  case V3AluOp::And:
  case V3AluOp::Or:
  case V3AluOp::Xor:
  case V3AluOp::Nor:
  case V3AluOp::Slt:
  case V3AluOp::Sltu:
    return reg(inst.rs) | reg(inst.rt);
  case V3AluOp::Jr:
  case V3AluOp::Jalr:
    return reg(inst.rs);
  case V3AluOp::Addiu:
  case V3AluOp::Slti:
  case V3AluOp::Sltiu:
  case V3AluOp::Andi:
  case V3AluOp::Ori:
  case V3AluOp::Xori:
  case V3AluOp::Lb:
  case V3AluOp::Lbu:
  case V3AluOp::Lh:
  case V3AluOp::Lhu:
  case V3AluOp::Lw:
    return reg(inst.rs);
  case V3AluOp::Sw:
  case V3AluOp::Beq:
  case V3AluOp::Bne:
    return reg(inst.rs) | reg(inst.rt);
  case V3AluOp::Nop:
  case V3AluOp::Lui:
  case V3AluOp::Clear:
  case V3AluOp::J:
  case V3AluOp::Jal:
    return 0u;
  }
  return 0u;
}

u8 write_reg(const V3DecodedInstruction &inst) {
  switch (inst.op) {
  case V3AluOp::Sll:
  case V3AluOp::Srl:
  case V3AluOp::Sra:
  case V3AluOp::Addu:
  case V3AluOp::Subu:
  case V3AluOp::And:
  case V3AluOp::Or:
  case V3AluOp::Xor:
  case V3AluOp::Nor:
  case V3AluOp::Slt:
  case V3AluOp::Sltu:
  case V3AluOp::Clear:
    return inst.rd;
  case V3AluOp::Addiu:
  case V3AluOp::Slti:
  case V3AluOp::Sltiu:
  case V3AluOp::Andi:
  case V3AluOp::Ori:
  case V3AluOp::Xori:
  case V3AluOp::Lui:
    return inst.rt;
  case V3AluOp::Jal:
    return 31u;
  case V3AluOp::Jalr:
    return inst.rd;
  case V3AluOp::Nop:
  case V3AluOp::Jr:
  case V3AluOp::J:
  case V3AluOp::Lb:
  case V3AluOp::Lbu:
  case V3AluOp::Lh:
  case V3AluOp::Lhu:
  case V3AluOp::Lw:
  case V3AluOp::Sw:
  case V3AluOp::Beq:
  case V3AluOp::Bne:
    return 0u;
  }
  return 0u;
}

void count_v3_unsupported_opcode(CpuBackendStats &stats, u32 bits) {
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

bool is_v3_branch(V3AluOp op) {
  return op == V3AluOp::Beq || op == V3AluOp::Bne;
}

bool is_v3_fixed_jump(V3AluOp op) {
  return op == V3AluOp::J || op == V3AluOp::Jal;
}

bool is_v3_dynamic_jump(V3AluOp op) {
  return op == V3AluOp::Jr || op == V3AluOp::Jalr;
}

bool is_v3_alu_only(V3AluOp op) {
  return !is_v3_load(op) && op != V3AluOp::Sw &&
         !is_v3_branch(op) && !is_v3_fixed_jump(op) &&
         !is_v3_dynamic_jump(op);
}

std::array<u8, 6> choose_cached_regs(
    const std::vector<V3DecodedInstruction> &instructions) {
  // V3 commonly emits very short blocks. Loading and spilling a six-register
  // host cache around a 1-3 instruction block costs more than direct GPR
  // accesses, especially before resident block linking removes the call edge.
  if (instructions.size() <= 3u) {
    return {};
  }
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

#if VIBESTATION_JIT_V3_X64

struct V3NativeRuntime {
  std::array<u8 *, 8> store_ptrs{};
  u8 *load_ptr = nullptr;
  u32 load_value = 0u;
  u32 dynamic_target = 0u;
  u32 incoming_load_reg = 0u;
  u32 incoming_load_value = 0u;
  u32 incoming_load_pc = 0u;
  u32 incoming_load_addr = 0u;
  u32 load_addr = 0u;
};

using V3NativeFn = u32 (*)(u32 *, const V3NativeRuntime *);
using V3StepHelperFn = u32 (*)(Cpu *);

// Link cells have stable addresses. Invalidation clears a cell before erasing
// its block, so native code never follows a pointer into discarded metadata.
struct V3ResidentBlock;
struct V3ResidentSlot {
  void *entry = nullptr;
  V3ResidentBlock *block = nullptr;
};

struct V3ResidentBlock {
  V3NativeFn fn = nullptr;
  void *linked_fn = nullptr;
  V3ResidentSlot *successor = nullptr;
  V3ResidentSlot *taken_successor = nullptr;
  u32 start_pc = 0;
  u32 next_pc = 0;
  u32 taken_pc = 0;
  u32 instruction_count = 0;
  u32 base_cycles = 0;
  u32 tail_index = 0;
  u32 kind = 0; // 0: sequential, 1: conditional branch, 2: fixed jump
  u32 has_load = 0;
  u32 load_retired = 0;
  u32 load_rs = 0;
  u32 load_rt = 0;
  u32 load_index = 0;
  s32 load_simm = 0;
  u32 load_alignment_mask = 0;
  u32 icache_line_count = 0;
  std::array<u8, 5> icache_indices{};
  std::array<u32, 5> icache_tags{};
};

struct V3ResidentContext {
  V3ResidentBlock *first = nullptr;
  u32 *gpr = nullptr;
  V3NativeRuntime *runtime = nullptr;
  const u8 *icache = nullptr;
  u8 *main_ram = nullptr;
  u8 *scratchpad = nullptr;
  u32 direct_icache_refill = 1u;
  u32 cycle_budget = 0;
  u32 instruction_budget = 0;
  u32 branch_allowed = 1;
  u32 entry_limit = 0xFFFFFFFFu;
  u32 cycles = 0;
  u32 instructions = 0;
  u32 entries = 0;
  u32 alu_entries = 0;
  u32 memory_entries = 0;
  u32 branch_entries = 0;
  u32 branch_taken = 0;
  u32 branch_not_taken = 0;
  u32 final_pc = 0;
  u32 last_taken = 0;
  V3ResidentBlock *last = nullptr;
};

using V3ResidentFn = void (*)(V3ResidentContext *);

// The linked ABI keeps the context in rbx, GPRs in r10, runtime in r11,
// I-cache in r15, and totals in r12d/r13d/r14d. r8d is the next PC and rdi
// identifies the last completed block. Only the chain exit writes totals.
struct V3LinkedDispatch {
  V3ResidentFn entry = nullptr;
  void *exit = nullptr;
};

class V3CodeArena {
public:
  V3CodeArena() = default;
  ~V3CodeArena() { reset(); }

  V3CodeArena(const V3CodeArena &) = delete;
  V3CodeArena &operator=(const V3CodeArena &) = delete;

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

u32 v3_step_helper(Cpu *cpu) {
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
            reinterpret_cast<size_t>(&v3_step_helper));
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

V3StepHelperFn install_step_helper(V3CodeArena &arena) {
  auto code = compile_step_helper_trampoline();
  if (!code) {
    return nullptr;
  }
  void *entry = arena.copy_code(code->getCode(), code->getSize());
  return reinterpret_cast<V3StepHelperFn>(entry);
}

V3ResidentFn install_resident_dispatch(V3CodeArena &arena) {
  using namespace Xbyak;
  auto code = std::make_unique<CodeGenerator>(4096);
  code->setDefaultJmpNEAR(true);
  Label execute, validate, done, sequential, branch_not_taken, choose_next;
  Label selected_slot, no_alu, conditional_branch;
  Label validation_line;
  code->push(code->rbx);
  code->push(code->rbp);
  code->push(code->r12);
  code->push(code->r13);
  code->push(code->r14);
  code->push(code->r15);
#if defined(_WIN32)
  code->sub(code->rsp, 40);
  code->mov(code->rbx, code->rcx);
#else
  code->sub(code->rsp, 8);
  code->mov(code->rbx, code->rdi);
#endif
  code->mov(code->r12, code->ptr[code->rbx + offsetof(V3ResidentContext, first)]);
  code->mov(code->r13, code->ptr[code->rbx + offsetof(V3ResidentContext, gpr)]);
  code->mov(code->r14, code->ptr[code->rbx + offsetof(V3ResidentContext, runtime)]);
  code->mov(code->r15, code->ptr[code->rbx + offsetof(V3ResidentContext, icache)]);
  code->jmp(execute); // The C++ entry path has already validated the first block.

  code->L(validate);
  code->test(code->r12, code->r12);
  code->jz(done);
  code->xor_(code->ecx, code->ecx);
  code->L(validation_line);
  code->cmp(code->ecx, code->dword[code->r12 + offsetof(V3ResidentBlock, icache_line_count)]);
  code->jae(execute);
  code->movzx(code->eax, code->byte[code->r12 + offsetof(V3ResidentBlock, icache_indices) + code->rcx]);
  code->imul(code->eax, code->eax, 24); // Cpu::ICacheLine stride
  code->lea(code->rdx, code->ptr[code->r15 + code->rax]);
  code->cmp(code->byte[code->rdx + 20], 0); // Cpu::ICacheLine::valid
  code->je(done);
  code->mov(code->eax, code->dword[code->rdx]);
  code->cmp(code->eax, code->dword[code->r12 + offsetof(V3ResidentBlock, icache_tags) + code->rcx * 4]);
  code->jne(done);
  code->inc(code->ecx);
  code->jmp(validation_line);

  code->L(execute);
  Label branch_ok;
  code->cmp(code->dword[code->rbx + offsetof(V3ResidentContext, branch_allowed)], 0);
  code->jne(branch_ok);
  code->cmp(code->dword[code->r12 + offsetof(V3ResidentBlock, kind)], 0);
  code->jne(done);
  code->L(branch_ok);
  code->mov(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, instructions)]);
  code->add(code->eax, code->dword[code->r12 + offsetof(V3ResidentBlock, instruction_count)]);
  code->cmp(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, instruction_budget)]);
  code->ja(done);
  code->mov(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, cycles)]);
  code->add(code->eax, code->dword[code->r12 + offsetof(V3ResidentBlock, base_cycles)]);
  code->cmp(code->dword[code->r12 + offsetof(V3ResidentBlock, kind)], 1);
  code->jne(sequential);
  code->inc(code->eax); // Worst case: conditional branch is taken.
  code->L(sequential);
  code->cmp(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, cycle_budget)]);
  code->ja(done);
#if defined(_WIN32)
  code->mov(code->rcx, code->r13);
  code->mov(code->rdx, code->r14);
#else
  code->mov(code->rdi, code->r13);
  code->mov(code->rsi, code->r14);
#endif
  code->call(code->ptr[code->r12 + offsetof(V3ResidentBlock, fn)]);
  code->mov(code->ebp, code->eax);
  code->mov(code->dword[code->r14 + offsetof(V3NativeRuntime, incoming_load_reg)], 0);
  code->mov(code->ptr[code->rbx + offsetof(V3ResidentContext, last)], code->r12);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, last_taken)], code->ebp);
  code->mov(code->eax, code->dword[code->r12 + offsetof(V3ResidentBlock, instruction_count)]);
  code->add(code->dword[code->rbx + offsetof(V3ResidentContext, instructions)], code->eax);
  code->mov(code->eax, code->dword[code->r12 + offsetof(V3ResidentBlock, base_cycles)]);
  code->add(code->dword[code->rbx + offsetof(V3ResidentContext, cycles)], code->eax);
  code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, entries)]);
  code->mov(code->eax, code->dword[code->r12 + offsetof(V3ResidentBlock, next_pc)]);
  code->cmp(code->dword[code->r12 + offsetof(V3ResidentBlock, kind)], 0);
  code->je(choose_next);
  code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, branch_entries)]);
  code->cmp(code->dword[code->r12 + offsetof(V3ResidentBlock, kind)], 1);
  code->je(conditional_branch);
  code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, branch_taken)]);
  code->jmp(choose_next);
  code->L(conditional_branch);
  code->test(code->ebp, code->ebp);
  code->jz(branch_not_taken);
  code->mov(code->eax, code->dword[code->r12 + offsetof(V3ResidentBlock, taken_pc)]);
  code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, cycles)]);
  code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, branch_taken)]);
  code->jmp(choose_next);
  code->L(branch_not_taken);
  code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, branch_not_taken)]);
  code->L(choose_next);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, final_pc)], code->eax);
  code->cmp(code->dword[code->r12 + offsetof(V3ResidentBlock, kind)], 0);
  code->jne(no_alu);
  code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, alu_entries)]);
  code->L(no_alu);
  code->mov(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, entries)]);
  code->cmp(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, entry_limit)]);
  code->jae(done);
  code->mov(code->rax, code->ptr[code->r12 + offsetof(V3ResidentBlock, successor)]);
  code->cmp(code->dword[code->r12 + offsetof(V3ResidentBlock, kind)], 1);
  code->jne(selected_slot);
  code->test(code->ebp, code->ebp);
  code->jz(selected_slot);
  code->mov(code->rax, code->ptr[code->r12 + offsetof(V3ResidentBlock, taken_successor)]);
  code->L(selected_slot);
  code->test(code->rax, code->rax);
  code->jz(done);
  code->mov(code->r12, code->ptr[code->rax + offsetof(V3ResidentSlot, block)]);
  code->jmp(validate);
  code->L(done);
#if defined(_WIN32)
  code->add(code->rsp, 40);
#else
  code->add(code->rsp, 8);
#endif
  code->pop(code->r15);
  code->pop(code->r14);
  code->pop(code->r13);
  code->pop(code->r12);
  code->pop(code->rbp);
  code->pop(code->rbx);
  code->ret();
  code->ready();
  return reinterpret_cast<V3ResidentFn>(arena.copy_code(code->getCode(), code->getSize()));
}

V3LinkedDispatch install_linked_dispatch(V3CodeArena &arena) {
  using namespace Xbyak;
  auto code = std::make_unique<CodeGenerator>(512);
  code->push(code->rbx);
  code->push(code->rbp);
  code->push(code->rsi);
  code->push(code->rdi);
  code->push(code->r12);
  code->push(code->r13);
  code->push(code->r14);
  code->push(code->r15);
#if defined(_WIN32)
  code->mov(code->rbx, code->rcx);
#else
  code->mov(code->rbx, code->rdi);
#endif
  code->mov(code->r10, code->ptr[code->rbx + offsetof(V3ResidentContext, gpr)]);
  code->mov(code->r11, code->ptr[code->rbx + offsetof(V3ResidentContext, runtime)]);
  code->mov(code->r15, code->ptr[code->rbx + offsetof(V3ResidentContext, icache)]);
  code->xor_(code->r12d, code->r12d);
  code->xor_(code->r13d, code->r13d);
  code->xor_(code->r14d, code->r14d);
  code->xor_(code->r8d, code->r8d);
  code->xor_(code->r9d, code->r9d);
  code->xor_(code->esi, code->esi);
  code->xor_(code->ebp, code->ebp);
  code->xor_(code->edi, code->edi);
  code->mov(code->rax, code->ptr[code->rbx + offsetof(V3ResidentContext, first)]);
  code->jmp(code->ptr[code->rax + offsetof(V3ResidentBlock, linked_fn)]);
  const size_t exit_offset = code->getSize();
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, cycles)], code->r12d);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, instructions)], code->r13d);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, entries)], code->r14d);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, branch_entries)], code->ebp);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, branch_taken)], code->r9d);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, branch_not_taken)], code->esi);
  code->mov(code->eax, code->r14d);
  code->sub(code->eax, code->ebp);
  code->sub(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, memory_entries)]);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, alu_entries)], code->eax);
  code->mov(code->dword[code->rbx + offsetof(V3ResidentContext, final_pc)], code->r8d);
  code->mov(code->ptr[code->rbx + offsetof(V3ResidentContext, last)], code->rdi);
  code->pop(code->r15);
  code->pop(code->r14);
  code->pop(code->r13);
  code->pop(code->r12);
  code->pop(code->rdi);
  code->pop(code->rsi);
  code->pop(code->rbp);
  code->pop(code->rbx);
  code->ret();
  code->ready();
  auto *entry = static_cast<u8 *>(arena.copy_code(code->getCode(), code->getSize()));
  return {reinterpret_cast<V3ResidentFn>(entry), entry ? entry + exit_offset : nullptr};
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
  case 2: return code.r12d;
  case 3: return code.r13d;
  case 4: return code.r14d;
  default: return code.r15d;
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
  code.mov(dst, code.dword[code.r10 + static_cast<int>(guest_reg) * 4]);
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
  code.mov(code.dword[code.r10 + static_cast<int>(guest_reg) * 4], src);
}


// Commit a load produced by the previous guest instruction *after* the first
// instruction of this block has consumed its operands. R3000A load delay makes
// this ordering observable. If the first instruction writes/schedules the same
// register, that write cancels the pending load instead.
void emit_commit_incoming_load(Xbyak::CodeGenerator &code,
                               const std::array<u8, 6> &cached,
                               u8 cancel_reg) {
  using namespace Xbyak;
  Label done;
  code.mov(code.eax, code.dword[code.r11 +
      static_cast<int>(offsetof(V3NativeRuntime, incoming_load_reg))]);
  code.test(code.eax, code.eax);
  code.jz(done);
  if (cancel_reg != 0u) {
    code.cmp(code.eax, static_cast<u32>(cancel_reg));
    code.je(done);
  }
  code.mov(code.ecx, code.dword[code.r11 +
      static_cast<int>(offsetof(V3NativeRuntime, incoming_load_value))]);
  code.mov(code.dword[code.r10 + code.rax * 4], code.ecx);
  for (size_t slot = 0; slot < cached.size(); ++slot) {
    if (cached[slot] == 0u) {
      continue;
    }
    Label next;
    code.cmp(code.eax, static_cast<u32>(cached[slot]));
    code.jne(next);
    code.mov(cache_host_reg(code, static_cast<int>(slot)), code.ecx);
    code.L(next);
  }
  code.L(done);
}

std::unique_ptr<Xbyak::CodeGenerator> compile_native_alu(
    const std::vector<V3DecodedInstruction> &instructions,
    const std::array<u8, 6> &cached, bool load_retired,
    u32 load_index, u8 load_rt,
    const V3ResidentBlock *linked = nullptr, void *linked_exit = nullptr) {
  using namespace Xbyak;
  auto code = std::make_unique<CodeGenerator>(4096);
  code->setDefaultJmpNEAR(true);
  Label linked_done, linked_icache_miss, linked_after_icache;
  if (linked != nullptr) {
    Label load_ram, load_ready;
    for (u32 line = 0; line < linked->icache_line_count; ++line) {
      const int offset = static_cast<int>(linked->icache_indices[line]) * 24;
      code->cmp(code->byte[code->r15 + offset + 20], 0);
      code->je(linked_icache_miss);
      code->cmp(code->dword[code->r15 + offset], linked->icache_tags[line]);
      code->jne(linked_icache_miss);
    }
    code->L(linked_after_icache);
    if (linked->kind != 0u) {
      code->cmp(code->dword[code->rbx + offsetof(V3ResidentContext, branch_allowed)], 0);
      code->je(linked_done);
    }
    if (linked->has_load != 0u) {
      if (linked->load_index != 0u && linked->load_rs != 0u) {
        code->cmp(code->dword[code->r11 + offsetof(V3NativeRuntime, incoming_load_reg)],
                  linked->load_rs);
        code->je(linked_done);
      }
      if (linked->load_rs == 0u) {
        code->xor_(code->eax, code->eax);
      } else {
        code->mov(code->eax, code->dword[code->r10 + linked->load_rs * 4u]);
      }
      if (linked->load_simm != 0) {
        code->add(code->eax, static_cast<u32>(linked->load_simm));
      }
      code->mov(code->dword[code->r11 + offsetof(V3NativeRuntime, load_addr)], code->eax);
      if (linked->load_alignment_mask != 0u) {
        code->test(code->eax, linked->load_alignment_mask);
        code->jnz(linked_done);
      }
      code->and_(code->eax, 0x1FFFFFFF);
      code->xor_(code->edx, code->edx);
      code->cmp(code->eax, psx::RAM_SIZE);
      code->jb(load_ram);
      code->cmp(code->eax, 0x1F800000u);
      code->jb(linked_done);
      code->cmp(code->eax, 0x1F801000u);
      code->jae(linked_done);
      code->sub(code->eax, 0x1F800000u);
      code->mov(code->rcx, code->ptr[code->rbx + offsetof(V3ResidentContext, scratchpad)]);
      code->test(code->rcx, code->rcx);
      code->jz(linked_done);
      code->lea(code->rax, code->ptr[code->rcx + code->rax]);
      code->jmp(load_ready);
      code->L(load_ram);
      code->mov(code->rcx, code->ptr[code->rbx + offsetof(V3ResidentContext, main_ram)]);
      code->test(code->rcx, code->rcx);
      code->jz(linked_done);
      code->lea(code->rax, code->ptr[code->rcx + code->rax]);
      code->mov(code->edx, 4);
      code->L(load_ready);
      code->mov(code->ptr[code->r11 + offsetof(V3NativeRuntime, load_ptr)], code->rax);
    }
    code->mov(code->eax, code->r13d);
    code->add(code->eax, linked->instruction_count);
    code->cmp(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, instruction_budget)]);
    code->ja(linked_done);
    code->mov(code->eax, code->r12d);
    code->add(code->eax, linked->base_cycles + (linked->kind == 1u ? 1u : 0u));
    if (linked->has_load != 0u) code->add(code->eax, code->edx);
    code->cmp(code->eax, code->dword[code->rbx + offsetof(V3ResidentContext, cycle_budget)]);
    code->ja(linked_done);
    if (linked->has_load != 0u) code->add(code->r12d, code->edx);
  }

  // V3 deliberately keeps architectural state virtual inside a block.
  // r10 = guest GPR base, r11 = runtime/preflight data, edx = branch result.
  // Both pointer registers are caller-saved on x64, so tiny blocks need no
  // stack traffic at all. Cache slots 2-5 use callee-saved registers and are
  // preserved only when a longer block actually populates them.
  const bool save_r12 = cached[2] != 0u;
  const bool save_r13 = cached[3] != 0u;
  const bool save_r14 = cached[4] != 0u;
  const bool save_r15 = cached[5] != 0u;
  if (linked == nullptr) {
    if (save_r12) code->push(code->r12);
    if (save_r13) code->push(code->r13);
    if (save_r14) code->push(code->r14);
    if (save_r15) code->push(code->r15);

#if defined(_WIN32)
  code->mov(code->r10, code->rcx);
  code->mov(code->r11, code->rdx);
#else
  code->mov(code->r10, code->rdi);
  code->mov(code->r11, code->rsi);
#endif
  }
  code->xor_(code->edx, code->edx);

  for (size_t slot = 0; slot < cached.size(); ++slot) {
    if (cached[slot] == 0u) {
      continue;
    }
    code->mov(cache_host_reg(*code, static_cast<int>(slot)),
              code->dword[code->r10 + static_cast<int>(cached[slot]) * 4]);
  }

  std::array<bool, 6> dirty{};
  u32 store_index = 0;

  for (size_t instruction_index = 0; instruction_index < instructions.size();
       ++instruction_index) {
    const auto &inst = instructions[instruction_index];
    const u8 dst = write_reg(inst);
    switch (inst.op) {
    case V3AluOp::Nop:
      break;

    case V3AluOp::Sll:
    case V3AluOp::Srl:
    case V3AluOp::Sra:
      emit_read_guest(*code, code->eax, cached, inst.rt);
      if (inst.shamt != 0u) {
        if (inst.op == V3AluOp::Sll) code->shl(code->eax, inst.shamt);
        else if (inst.op == V3AluOp::Srl) code->shr(code->eax, inst.shamt);
        else code->sar(code->eax, inst.shamt);
      }
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V3AluOp::Addu:
    case V3AluOp::Subu:
    case V3AluOp::And:
    case V3AluOp::Or:
    case V3AluOp::Xor:
    case V3AluOp::Nor:
    case V3AluOp::Slt:
    case V3AluOp::Sltu:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      emit_read_guest(*code, code->ecx, cached, inst.rt);
      switch (inst.op) {
      case V3AluOp::Addu: code->add(code->eax, code->ecx); break;
      case V3AluOp::Subu: code->sub(code->eax, code->ecx); break;
      case V3AluOp::And: code->and_(code->eax, code->ecx); break;
      case V3AluOp::Or: code->or_(code->eax, code->ecx); break;
      case V3AluOp::Xor: code->xor_(code->eax, code->ecx); break;
      case V3AluOp::Nor:
        code->or_(code->eax, code->ecx);
        code->not_(code->eax);
        break;
      case V3AluOp::Slt:
        code->cmp(code->eax, code->ecx);
        code->setl(code->al);
        code->movzx(code->eax, code->al);
        break;
      case V3AluOp::Sltu:
        code->cmp(code->eax, code->ecx);
        code->setb(code->al);
        code->movzx(code->eax, code->al);
        break;
      default: break;
      }
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V3AluOp::Addiu:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      if (inst.simm != 0) {
        code->add(code->eax, static_cast<u32>(inst.simm));
      }
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V3AluOp::Slti:
    case V3AluOp::Sltiu:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      code->mov(code->ecx, static_cast<u32>(inst.simm));
      code->cmp(code->eax, code->ecx);
      if (inst.op == V3AluOp::Slti) code->setl(code->al);
      else code->setb(code->al);
      code->movzx(code->eax, code->al);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V3AluOp::Andi:
    case V3AluOp::Ori:
    case V3AluOp::Xori:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      if (inst.op == V3AluOp::Andi) code->and_(code->eax, inst.imm);
      else if (inst.op == V3AluOp::Ori) code->or_(code->eax, inst.imm);
      else code->xor_(code->eax, inst.imm);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V3AluOp::Lui:
      code->mov(code->eax, static_cast<u32>(inst.imm) << 16u);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V3AluOp::Clear:
      code->xor_(code->eax, code->eax);
      emit_write_guest(*code, cached, dst, code->eax, dirty);
      break;

    case V3AluOp::J:
      break;

    case V3AluOp::Jal:
      code->mov(code->eax, inst.link_value);
      emit_write_guest(*code, cached, 31u, code->eax, dirty);
      break;

    case V3AluOp::Jr:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      code->mov(code->dword[code->r11 +
          static_cast<int>(offsetof(V3NativeRuntime, dynamic_target))],
          code->eax);
      break;

    case V3AluOp::Jalr:
      // Capture the target before writing the link register. This matters for
      // the legal but nasty rd == rs case.
      emit_read_guest(*code, code->eax, cached, inst.rs);
      code->mov(code->dword[code->r11 +
          static_cast<int>(offsetof(V3NativeRuntime, dynamic_target))],
          code->eax);
      code->mov(code->eax, inst.link_value);
      emit_write_guest(*code, cached, inst.rd, code->eax, dirty);
      break;

    case V3AluOp::Lb:
    case V3AluOp::Lbu:
    case V3AluOp::Lh:
    case V3AluOp::Lhu:
    case V3AluOp::Lw: {
      code->mov(code->rax, code->ptr[code->r11 +
          static_cast<int>(offsetof(V3NativeRuntime, load_ptr))]);
      switch (inst.op) {
      case V3AluOp::Lb:
        code->movsx(code->eax, code->byte[code->rax]);
        break;
      case V3AluOp::Lbu:
        code->movzx(code->eax, code->byte[code->rax]);
        break;
      case V3AluOp::Lh:
        code->movsx(code->eax, code->word[code->rax]);
        break;
      case V3AluOp::Lhu:
        code->movzx(code->eax, code->word[code->rax]);
        break;
      case V3AluOp::Lw:
        code->mov(code->eax, code->dword[code->rax]);
        break;
      default:
        break;
      }
      code->mov(code->dword[code->r11 +
          static_cast<int>(offsetof(V3NativeRuntime, load_value))], code->eax);
      break;
    }

    case V3AluOp::Sw: {
      emit_read_guest(*code, code->ecx, cached, inst.rt);
      const size_t ptr_offset =
          offsetof(V3NativeRuntime, store_ptrs) +
          static_cast<size_t>(store_index) * sizeof(u8 *);
      code->mov(code->rax, code->ptr[code->r11 + static_cast<int>(ptr_offset)]);
      code->mov(code->dword[code->rax], code->ecx);
      ++store_index;
      break;
    }

    case V3AluOp::Beq:
    case V3AluOp::Bne:
      emit_read_guest(*code, code->eax, cached, inst.rs);
      emit_read_guest(*code, code->ecx, cached, inst.rt);
      code->cmp(code->eax, code->ecx);
      if (inst.op == V3AluOp::Beq) code->sete(code->dl);
      else code->setne(code->dl);
      code->movzx(code->edx, code->dl);
      break;
    }

    if (instruction_index == 0u) {
      u8 cancel_reg = dst;
      if (is_v3_load(inst.op)) {
        cancel_reg = inst.rt;
      }
      emit_commit_incoming_load(*code, cached, cancel_reg);
    }
    // A load becomes visible after exactly one following instruction. That
    // instruction reads the old value above; its own write cancels the load.
    if (load_retired && instruction_index == load_index + 1u &&
        load_rt != 0u && dst != load_rt) {
      code->mov(code->eax, code->dword[code->r11 +
          static_cast<int>(offsetof(V3NativeRuntime, load_value))]);
      emit_write_guest(*code, cached, load_rt, code->eax, dirty);
    }
  }

  for (size_t slot = 0; slot < cached.size(); ++slot) {
    if (cached[slot] == 0u || !dirty[slot]) {
      continue;
    }
    code->mov(code->dword[code->r10 + static_cast<int>(cached[slot]) * 4],
              cache_host_reg(*code, static_cast<int>(slot)));
  }

  code->mov(code->dword[code->r10], 0u);
  if (linked != nullptr) {
    Label not_taken, next;
    code->mov(code->rdi, reinterpret_cast<size_t>(linked));
    if (linked->has_load != 0u && linked->load_retired == 0u) {
      code->mov(code->eax, code->dword[code->r11 + offsetof(V3NativeRuntime, load_value)]);
      code->mov(code->dword[code->r11 + offsetof(V3NativeRuntime, incoming_load_value)], code->eax);
      code->mov(code->eax, code->dword[code->r11 + offsetof(V3NativeRuntime, load_addr)]);
      code->mov(code->dword[code->r11 + offsetof(V3NativeRuntime, incoming_load_addr)], code->eax);
      code->mov(code->dword[code->r11 + offsetof(V3NativeRuntime, incoming_load_pc)],
                linked->start_pc + linked->load_index * 4u);
      code->mov(code->dword[code->r11 + offsetof(V3NativeRuntime, incoming_load_reg)],
                static_cast<u32>(linked->load_rt));
    } else {
      code->mov(code->dword[code->r11 + offsetof(V3NativeRuntime, incoming_load_reg)], 0);
    }
    code->add(code->r13d, linked->instruction_count);
    code->add(code->r12d, linked->base_cycles);
    code->inc(code->r14d);
    code->mov(code->r8d, linked->next_pc);
    if (linked->kind == 1u) {
      code->inc(code->ebp);
      code->test(code->edx, code->edx);
      code->jz(not_taken);
      code->mov(code->r8d, linked->taken_pc);
      code->inc(code->r12d);
      code->inc(code->r9d);
      code->mov(code->rax, reinterpret_cast<size_t>(linked->taken_successor));
      code->jmp(next);
      code->L(not_taken);
      code->inc(code->esi);
    } else if (linked->kind == 2u) {
      code->inc(code->ebp);
      code->inc(code->r9d);
    } else if (linked->has_load != 0u) {
      code->inc(code->dword[code->rbx + offsetof(V3ResidentContext, memory_entries)]);
    }
    code->mov(code->rax, reinterpret_cast<size_t>(linked->successor));
    code->L(next);
    code->jmp(code->ptr[code->rax + offsetof(V3ResidentSlot, entry)]);

    // Cold path for a normal direct-mapped R3000A I-cache alias/eviction.
    // Code writes separately invalidate the compiled block/link cell, so a
    // tag miss here means the compiled guest code is still valid: refill the
    // architectural cache line, charge the exact four-cycle fill penalty, and
    // remain inside the native chain.
    code->L(linked_icache_miss);
    code->cmp(code->dword[code->rbx +
                  offsetof(V3ResidentContext, direct_icache_refill)], 0);
    code->je(linked_done);

    // Do not mutate cache/timing state unless the complete block can fit even
    // in the worst refill case. This keeps slice-boundary behavior on the
    // existing precise exit path.
    code->mov(code->eax, code->r13d);
    code->add(code->eax, linked->instruction_count);
    code->cmp(code->eax,
              code->dword[code->rbx +
                  offsetof(V3ResidentContext, instruction_budget)]);
    code->ja(linked_done);
    code->mov(code->eax, code->r12d);
    code->add(code->eax,
              linked->base_cycles + (linked->kind == 1u ? 1u : 0u) +
                  (linked->has_load != 0u ? 4u : 0u) +
                  linked->icache_line_count * 4u);
    code->cmp(code->eax,
              code->dword[code->rbx +
                  offsetof(V3ResidentContext, cycle_budget)]);
    code->ja(linked_done);

    for (u32 line = 0; line < linked->icache_line_count; ++line) {
      Label refill, line_ready;
      const u32 index = linked->icache_indices[line];
      const u32 tag = linked->icache_tags[line];
      const int offset = static_cast<int>(index) * 24;

      code->cmp(code->byte[code->r15 + offset + 20], 0);
      code->je(refill);
      code->cmp(code->dword[code->r15 + offset], tag);
      code->je(line_ready);
      code->L(refill);

      if (tag < 0x00800000u) {
        const u32 ram_offset = tag & 0x001FFFFFu;
        code->mov(code->rax,
                  code->ptr[code->rbx +
                      offsetof(V3ResidentContext, main_ram)]);
        code->test(code->rax, code->rax);
        code->jz(linked_done);
        code->movups(code->xmm0, code->xword[code->rax + ram_offset]);
      } else if (tag >= 0x1F800000u && tag < 0x1F801000u) {
        const u32 scratch_offset =
            (tag - 0x1F800000u) & (psx::SCRATCHPAD_SIZE - 1u);
        code->mov(code->rax,
                  code->ptr[code->rbx +
                      offsetof(V3ResidentContext, scratchpad)]);
        code->test(code->rax, code->rax);
        code->jz(linked_done);
        code->movups(code->xmm0, code->xword[code->rax + scratch_offset]);
      } else {
        code->jmp(linked_done);
      }

      code->mov(code->dword[code->r15 + offset], tag);
      code->movups(code->xword[code->r15 + offset + 4], code->xmm0);
      code->mov(code->byte[code->r15 + offset + 20], 1);
      code->add(code->r12d, 4u);
      code->L(line_ready);
    }
    code->jmp(linked_after_icache);

    code->L(linked_done);
    code->mov(code->rax, reinterpret_cast<size_t>(linked_exit));
    code->jmp(code->rax);
    code->ready();
    return code;
  }
  code->mov(code->eax, code->edx);
  if (save_r15) code->pop(code->r15);
  if (save_r14) code->pop(code->r14);
  if (save_r13) code->pop(code->r13);
  if (save_r12) code->pop(code->r12);
  code->ret();
  code->ready();
  return code;
}

#endif

} // namespace

struct CpuJitV3Backend::Impl {
  struct Block {
    V3BlockKind kind = V3BlockKind::Inline;
    u32 start_pc = 0;
    u32 phys_start = 0;
    u32 phys_end = 0;
    u32 instruction_count = 0;
    u32 base_cycles = 0;
    u32 store_count = 0;
    u8 load_rs = 0u;
    u8 load_rt = 0u;
    s32 load_simm = 0;
    u32 load_alignment_mask = 0u;
    u32 load_index = 0u;
    u32 branch_index = 0;
    u32 branch_target = 0;
    u32 jump_index = 0;
    u32 jump_target = 0;
    bool jump_dynamic = false;
    bool has_store = false;
    bool has_load = false;
    bool load_retired = false;
    bool has_branch = false;
    bool has_jump = false;
    std::array<u32, 16> words{};
    u8 icache_line_count = 0u;
    std::array<u8, 5> icache_indices{};
    std::array<u32, 5> icache_tags{};
    std::array<u8, 6> cached_regs{};
    std::array<u8, 8> store_rs{};
    std::array<s32, 8> store_simm{};
    std::array<u8, 8> store_instruction_index{};
#if VIBESTATION_JIT_V3_X64
    V3NativeFn fn = nullptr;
    std::array<V3NativeFn, 17> prefix_fns{};
    V3ResidentBlock resident{};
    size_t code_size = 0u;
#endif
  };

  // A page is indexed by the low 16 bits of the guest PC. The top table keeps
  // KUSEG/KSEG0/KSEG1 aliases distinct without allocating a 4 GB flat map.
  struct DispatchEntry {
    Block *block = nullptr;
#if VIBESTATION_JIT_V3_X64
    V3ResidentSlot resident;
#endif
  };
  struct DispatchPage {
    std::array<DispatchEntry, 1u << 14u> entries{};
  };

  DispatchEntry *dispatch_entry(u32 pc, bool create) {
    auto &page = dispatch_pages[pc >> 16u];
    if (!page) {
      if (!create) return nullptr;
      page = std::make_unique<DispatchPage>();
    }
    return &page->entries[(pc >> 2u) & 0x3FFFu];
  }

  Block *lookup_dispatch(u32 pc) {
    DispatchEntry *entry = dispatch_entry(pc, false);
    return entry != nullptr ? entry->block : nullptr;
  }

  void remember_dispatch(Block &block) {
    dispatch_entry(block.start_pc, true)->block = &block;
  }

  void forget_dispatch(u32 pc) {
    DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry != nullptr) entry->block = nullptr;
  }

  void clear_dispatch() {
    for (auto &page : dispatch_pages) page.reset();
  }

  std::array<std::unique_ptr<DispatchPage>, 1u << 16u> dispatch_pages{};

#if VIBESTATION_JIT_V3_X64
  struct DelaySlotCode {
    u32 word = 0u;
    V3NativeFn fn = nullptr;
  };
  V3CodeArena arena;
  std::unordered_map<u32, DelaySlotCode> delay_slot_code;
  V3StepHelperFn step_helper_fn = nullptr;
  V3ResidentFn resident_fn = nullptr;
  V3LinkedDispatch linked_dispatch{};
  bool use_linked = true;
  bool link_pending_lw = true;
  V3ResidentSlot *resident_slot(u32 pc) {
    V3ResidentSlot *slot = &dispatch_entry(pc, true)->resident;
    if (slot->entry == nullptr) slot->entry = linked_dispatch.exit;
    return slot;
  }

  void unlink_resident(u32 pc) {
    DispatchEntry *entry = dispatch_entry(pc, false);
    if (entry != nullptr) {
      entry->resident.entry = linked_dispatch.exit;
      entry->resident.block = nullptr;
    }
  }
#endif
  std::unordered_map<u32, Block> blocks;
  // PCs which cannot currently form even the minimum V3 native block.
  // These are invalidated with code writes instead of being recompiled on
  // every execution.
  std::unordered_set<u32> rejected_pcs;
  std::unordered_set<u32> rejected_pages;
  // Conservative ever-compiled page bitmap. Entries are intentionally retained
  // until flush: stale positives only cost an uncommon invalidation check,
  // while there can never be a false negative that misses self-modifying code.
  // The same architecture-neutral primitive can be reused by future EE/IOP
  // dynarecs with their own address width.
  JitCodePageBitmap<29u, 12u> code_pages;
};

CpuJitV3Backend::CpuJitV3Backend(Cpu &cpu)
    : cpu_(cpu), impl_(std::make_unique<Impl>()) {
  stats_.available = true;
#if VIBESTATION_JIT_V3_X64
  impl_->use_linked = std::getenv("VIBESTATION_V3_LEGACY_RESIDENT") == nullptr;
  impl_->link_pending_lw = std::getenv("VIBESTATION_V3_NO_PENDING_LW") == nullptr;
  static_assert(sizeof(Cpu::ICacheLine) == 24u, "resident I-cache stride");
  static_assert(offsetof(Cpu::ICacheLine, valid) == 20u,
                "resident I-cache valid offset");
  impl_->step_helper_fn = install_step_helper(impl_->arena);
  try {
    impl_->resident_fn = install_resident_dispatch(impl_->arena);
    impl_->linked_dispatch = install_linked_dispatch(impl_->arena);
  } catch (const std::exception &e) {
    std::fprintf(stderr, "V3 resident dispatch codegen: %s\n", e.what());
  }
  stats_.native_available = impl_->step_helper_fn != nullptr;
#else
  stats_.native_available = false;
#endif
}

CpuJitV3Backend::~CpuJitV3Backend() = default;

CpuRunSliceResult CpuJitV3Backend::run_slice(u32 max_cycles,
                                             u32 max_instructions) {
  CpuRunSliceResult result{};
  if (max_cycles == 0u || max_instructions == 0u) {
    return result;
  }

  stats_.available = true;
  stats_.active = true;
#if VIBESTATION_JIT_V3_X64
  stats_.native_available = impl_->step_helper_fn != nullptr;
#else
  stats_.native_available = false;
#endif

  bool native_streak = false;

  auto interpreter_step = [&]() {
    native_streak = false;
    const u32 consumed = cpu_.step();
    result.cycles += consumed;
    ++result.instructions;
    ++stats_.interpreter_fallback_steps;
    ++stats_.fallback_instructions;
    stats_.executed_cycles += consumed;
  };


#if VIBESTATION_JIT_V3_X64
  auto helper_step = [&](V3HelperReason reason) -> bool {
    native_streak = false;
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
    case V3HelperReason::State: ++stats_.jit_v2_helper_state; break;
    case V3HelperReason::Icache: ++stats_.jit_v2_helper_icache; break;
    case V3HelperReason::Irq: ++stats_.jit_v2_helper_irq; break;
    case V3HelperReason::Unsupported: ++stats_.jit_v2_helper_unsupported; break;
    case V3HelperReason::Memory: ++stats_.jit_v2_helper_memory; break;
    case V3HelperReason::Budget: ++stats_.jit_v2_helper_budget; break;
    case V3HelperReason::Internal: ++stats_.jit_v2_helper_internal; break;
    }
    ++stats_.native_instructions;
    ++stats_.optimized_instructions;
    stats_.native_cycles += consumed;
    stats_.executed_cycles += consumed;
    return true;
  };

  auto try_native_pending_delay_slot = [&]() -> bool {
    // Cpu::step() leaves a branch's architectural delay slot pending after a
    // helper executes the branch itself. Most of these slots are ordinary
    // ALU/NOP instructions. Execute those directly instead of forcing a
    // second Cpu::step(), but deliberately leave memory/control slots on the
    // precise helper path for now.
    if (!cpu_.pending_delay_slot_ || cpu_.in_delay_slot_ ||
        cpu_.pc_ != cpu_.pending_branch_pc_ + 4u ||
        cpu_.next_load_.reg != 0u ||
        g_trace_cpu || g_cpu_deep_diagnostics || g_log_fmv_diagnostics) {
      return false;
    }
    if (result.instructions >= max_instructions || result.cycles >= max_cycles) {
      return false;
    }

    const u32 delay_pc = cpu_.pc_;
    if (!cpu_.instruction_cacheable(delay_pc)) {
      return false;
    }
    const u32 index = (delay_pc >> 4u) & 0xFFu;
    const u32 word_index = (delay_pc >> 2u) & 0x03u;
    const u32 expected_tag = psx::mask_address(delay_pc) & ~0x0Fu;
    const auto &line = cpu_.icache_[index];
    if (!line.valid || line.tag != expected_tag) {
      return false;
    }

    const u32 bits = line.words[word_index];
    V3DecodedInstruction inst{};
    if (!decode_v3_alu(bits, inst) || !is_v3_alu_only(inst.op)) {
      return false;
    }

    auto &entry = impl_->delay_slot_code[delay_pc];
    if (entry.fn == nullptr || entry.word != bits) {
      std::vector<V3DecodedInstruction> one{inst};
      try {
        auto generated = compile_native_alu(one, {}, false, 0u, 0u);
        entry.fn = reinterpret_cast<V3NativeFn>(impl_->arena.copy_code(
            generated->getCode(), generated->getSize()));
        entry.word = bits;
      } catch (const std::exception &e) {
        std::fprintf(stderr, "V3 native delay-slot codegen: %s\n", e.what());
        entry.fn = nullptr;
      }
    }
    if (entry.fn == nullptr) {
      return false;
    }

    V3NativeRuntime runtime{};
    runtime.incoming_load_reg = cpu_.load_.reg;
    runtime.incoming_load_value = cpu_.load_.value;
    runtime.incoming_load_pc = cpu_.load_.source_pc;
    runtime.incoming_load_addr = cpu_.load_.source_addr;

    const u32 resume_pc = cpu_.next_pc_;
    const u32 branch_pc = cpu_.pending_branch_pc_;

    // Cpu::step() samples the external IRQ line at every instruction boundary,
    // including immediately before a branch delay slot. The interrupt itself
    // is deferred until the delay slot retires, but Cause.IP2 must already
    // reflect the sampled line in the architectural state.
    if (cpu_.sys_->irq_pending()) {
      cpu_.cop0_cause_ |= (1u << 10);
    } else {
      cpu_.cop0_cause_ &= ~(1u << 10);
    }

    cpu_.executing_step_ = true;
    cpu_.exception_raised_ = false;
    cpu_.cycle_penalty_ = 0u;
    entry.fn(cpu_.gpr_, &runtime);
    cpu_.executing_step_ = false;

    cpu_.current_pc_ = delay_pc;
    cpu_.pc_ = resume_pc;
    cpu_.next_pc_ = resume_pc + 4u;
    cpu_.in_delay_slot_ = true;
    cpu_.active_branch_pc_ = branch_pc;
    cpu_.pending_delay_slot_ = false;
    cpu_.pending_branch_taken_ = false;
    cpu_.pending_branch_pc_ = 0u;
    cpu_.load_ = {};
    cpu_.next_load_ = {};
    cpu_.cycles_ += 1u;
    g_diag_current_pc = cpu_.current_pc_;
    cpu_.rr4_diag_state_.prev_pc_for_diag = cpu_.current_pc_;

    ++result.cycles;
    ++result.instructions;
    ++stats_.native_block_entries;
    ++stats_.native_alu_block_entries;
    ++stats_.native_instructions;
    ++stats_.jit_v2_inline_instructions;
    ++stats_.optimized_instructions;
    ++stats_.native_cycles;
    ++stats_.executed_cycles;

    // The next architectural boundary must resample IRQs. Cpu::step() defers
    // them for the delay slot itself, but not for the target/fallthrough.
    native_streak = false;
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
    // V3 carries an active one-instruction R3000A load delay through native
    // blocks. next_load_ exists only while Cpu::step() is executing, so seeing
    // it here still means the architectural state is not safe to enter.
    if (cpu_.next_load_.reg != 0u) {
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
    if (!native_streak && cpu_.pending_delay_slot_ &&
        try_native_pending_delay_slot()) {
      return true;
    }
    if (!native_streak) {
      if (!state_allows_native()) {
        return helper_step(V3HelperReason::State);
      }

      // IRQ can only change when we cross back through a helper/MMIO/event
      // boundary. Pure native blocks touch GPRs, RAM/scratchpad and local CPU
      // timing state only, so resampling it for every 1-2 instruction block is
      // redundant. The outer slice still tests timing boundaries after every
      // block.
      if (cpu_.sys_->irq_pending()) {
        cpu_.cop0_cause_ |= (1u << 10);
      } else {
        cpu_.cop0_cause_ &= ~(1u << 10);
      }
      if (cpu_.check_irq()) {
        return helper_step(V3HelperReason::Irq);
      }
    } else {
      // A native branch block has already retired its architectural delay slot.
      // Cpu::step() would clear this descriptive state at the beginning of the
      // next instruction; do exactly that without re-running the full state
      // predicate.
      if (cpu_.in_delay_slot_) {
        cpu_.in_delay_slot_ = false;
        cpu_.active_branch_pc_ = 0u;
      }
    }

    const u32 start_pc = cpu_.pc_;
    if (!cpu_.instruction_cacheable(start_pc)) {
      return helper_step(V3HelperReason::Icache);
    }

    const u32 index = (start_pc >> 4u) & 0xFFu;
    const u32 word_index = (start_pc >> 2u) & 0x03u;
    const u32 expected_tag = psx::mask_address(start_pc) & ~0x0Fu;
    auto &line = cpu_.icache_[index];
    if (!line.valid || line.tag != expected_tag) {
      return helper_step(V3HelperReason::Icache);
    }

    if (impl_->rejected_pcs.find(start_pc) != impl_->rejected_pcs.end()) {
      ++stats_.cache_hits;
      return helper_step(V3HelperReason::Unsupported);
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
      // A block's instruction words cannot change without a code-write
      // invalidation. The hot path only needs to prove that the direct-mapped
      // I-cache still contains the same physical lines. Check each line once
      // instead of cacheability/tag/word for every guest instruction.
      for (u32 i = 0; i < block_ptr->icache_line_count; ++i) {
        const auto &inst_line = cpu_.icache_[block_ptr->icache_indices[i]];
        if (!inst_line.valid || inst_line.tag != block_ptr->icache_tags[i]) {
          coherent = false;
          break;
        }
      }
      if (!coherent) {
        impl_->forget_dispatch(start_pc);
        impl_->unlink_resident(start_pc);
        impl_->blocks.erase(start_pc);
        block_ptr = nullptr;
      }
    }

    if (block_ptr == nullptr) {
      ++stats_.cache_misses;
      ++stats_.native_compile_attempts;

      std::vector<V3DecodedInstruction> decoded;
      std::array<u32, 16> words{};
      constexpr u32 kMaxV3Instructions = 16u;
      // Keep ordinary translated blocks within one architectural 16-byte
      // R3000A I-cache line. Tail linking makes these extra host block
      // boundaries cheap, while a future cache-miss refill can now happen at
      // exactly the guest fetch boundary instead of prefetching later lines.
      const u32 line_instruction_limit =
          std::min<u32>(kMaxV3Instructions, 4u - word_index);
      u32 written_mask = 0u;
      u32 store_count = 0u;
      bool has_store = false;
      bool has_load = false;
      u8 load_rs = 0u;
      u8 load_rt = 0u;
      s32 load_simm = 0;
      u32 load_alignment_mask = 0u;
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

      auto fetch_decoded = [&](u32 i, V3DecodedInstruction &inst,
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
        if (!decode_v3_alu(bits, inst)) {
          fetch_stopped_on_unsupported = true;
          unsupported_bits = bits;
          return false;
        }
        return true;
      };

      for (u32 i = 0; i < line_instruction_limit; ++i) {
        V3DecodedInstruction inst{};
        u32 bits = 0u;
        if (!fetch_decoded(i, inst, bits)) {
          break;
        }

        if (is_v3_fixed_jump(inst.op) || is_v3_dynamic_jump(inst.op)) {
          // J/JAL/JR/JALR are always taken and include the architectural delay
          // slot. Link instructions write their destination before the delay
          // slot, while JR/JALR capture the dynamic target before that slot.
          // If a control instruction is itself the last word in an I-cache
          // line, it still owns the architectural delay slot in the next line.
          // Otherwise stop before a cross-line control op and let the next
          // tail-linked block begin at that instruction.
          if (i + 1u >= line_instruction_limit && !decoded.empty()) {
            break;
          }
          V3DecodedInstruction delay{};
          u32 delay_bits = 0u;
          if (!fetch_decoded(i + 1u, delay, delay_bits) ||
              !is_v3_alu_only(delay.op)) {
            break;
          }

          const u32 jump_pc = start_pc + i * 4u;
          inst.link_value = jump_pc + 8u;
          has_jump = true;
          jump_dynamic = is_v3_dynamic_jump(inst.op);
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

        if (is_v3_branch(inst.op)) {
          // V3 branch blocks always include the architectural delay slot and
          // end immediately after it. Keep the delay slot ALU-only for the
          // first branch tier; memory delay slots remain interpreter territory.
          if (i + 1u >= line_instruction_limit && !decoded.empty()) {
            break;
          }
          V3DecodedInstruction delay{};
          u32 delay_bits = 0u;
          if (!fetch_decoded(i + 1u, delay, delay_bits) ||
              !is_v3_alu_only(delay.op)) {
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

        if (is_v3_load(inst.op)) {
          // Only one load is preflighted per block. The following instruction
          // may execute in the same block with the old register value.
          if (has_load) {
            break;
          }
          if (inst.rs != 0u && (written_mask & (1u << inst.rs)) != 0u) {
            break;
          }
          has_load = true;
          load_rs = inst.rs;
          load_rt = inst.rt;
          load_simm = inst.simm;
          load_alignment_mask = v3_load_alignment_mask(inst.op);
          load_index = static_cast<u32>(decoded.size());
          decoded.push_back(inst);
          words[decoded.size() - 1u] = bits;
          continue;
        }

        if (inst.op == V3AluOp::Sw) {
          if (has_load) {
            break;
          }
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
          return helper_step(V3HelperReason::Icache);
        }

        // Universal baseline: an instruction V3 does not lower yet still gets
        // a compiled block identity. Its entry is the single shared generated
        // Cpu::step() trampoline, so ordinary executable code can no longer
        // fail compilation merely because an opcode is not inline-native yet.
        Impl::Block helper_block{};
        helper_block.kind = V3BlockKind::StepHelper;
        helper_block.start_pc = start_pc;
        helper_block.phys_start = psx::mask_address(start_pc);
        helper_block.phys_end = helper_block.phys_start + 3u;
        helper_block.instruction_count = 1u;
        helper_block.words[0] = line.words[word_index];
        helper_block.icache_line_count = 1u;
        helper_block.icache_indices[0] = static_cast<u8>(index);
        helper_block.icache_tags[0] = expected_tag;
        impl_->code_pages.mark_page(helper_block.phys_start >> 12u);
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
      // Sequential blocks span at most five 16-byte I-cache lines at the
      // current 16-instruction cap. Precompute their identities once.
      u32 previous_index = 0xFFFFFFFFu;
      u32 previous_tag = 0xFFFFFFFFu;
      for (u32 i = 0; i < block.instruction_count; ++i) {
        const u32 inst_pc = start_pc + i * 4u;
        const u32 inst_index = (inst_pc >> 4u) & 0xFFu;
        const u32 inst_tag = psx::mask_address(inst_pc) & ~0x0Fu;
        if (i != 0u && inst_index == previous_index && inst_tag == previous_tag) {
          continue;
        }
        const u32 slot = block.icache_line_count++;
        block.icache_indices[slot] = static_cast<u8>(inst_index);
        block.icache_tags[slot] = inst_tag;
        previous_index = inst_index;
        previous_tag = inst_tag;
      }
      block.cached_regs = choose_cached_regs(decoded);
      block.has_store = has_store;
      block.has_load = has_load;
      block.load_retired = has_load && decoded.size() > load_index + 1u;
      block.has_branch = has_branch;
      block.has_jump = has_jump;
      block.store_count = store_count;
      block.load_rs = load_rs;
      block.load_rt = load_rt;
      block.load_simm = load_simm;
      block.load_alignment_mask = load_alignment_mask;
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
        if (inst.op == V3AluOp::Sw || is_v3_load(inst.op) ||
            inst.op == V3AluOp::J || inst.op == V3AluOp::Jal ||
            inst.op == V3AluOp::Jr || inst.op == V3AluOp::Jalr) {
          block.base_cycles += 2u;
        } else {
          // Conditional branches cost one cycle when not taken and gain one
          // more dynamically when taken. All currently-native ALU ops cost 1.
          block.base_cycles += 1u;
        }
      }

      auto generated = compile_native_alu(decoded, block.cached_regs,
                                          block.load_retired, load_index,
                                          load_rt);
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
      block.fn = reinterpret_cast<V3NativeFn>(entry);
      if (!has_store &&
          (!has_load || (impl_->use_linked &&
                         (block.load_retired || impl_->link_pending_lw))) &&
          !jump_dynamic) {
        auto &resident = block.resident;
        resident.fn = block.fn;
        resident.start_pc = start_pc;
        resident.instruction_count = block.instruction_count;
        resident.base_cycles = block.base_cycles;
        resident.has_load = block.has_load ? 1u : 0u;
        resident.load_retired = block.load_retired ? 1u : 0u;
        resident.load_rs = block.load_rs;
        resident.load_rt = block.load_rt;
        resident.load_index = block.load_index;
        resident.load_simm = block.load_simm;
        resident.load_alignment_mask = block.load_alignment_mask;
        resident.icache_line_count = block.icache_line_count;
        resident.icache_indices = block.icache_indices;
        resident.icache_tags = block.icache_tags;
        if (has_branch) {
          const u32 branch_pc = start_pc + block.branch_index * 4u;
          resident.kind = 1u;
          resident.tail_index = block.branch_index;
          resident.next_pc = branch_pc + 8u;
          resident.taken_pc = block.branch_target;
          resident.successor = impl_->resident_slot(resident.next_pc);
          resident.taken_successor = impl_->resident_slot(resident.taken_pc);
        } else if (has_jump) {
          resident.kind = 2u;
          resident.tail_index = block.jump_index;
          resident.next_pc = block.jump_target;
          resident.successor = impl_->resident_slot(resident.next_pc);
        } else {
          resident.next_pc = start_pc + block.instruction_count * 4u;
          resident.successor = impl_->resident_slot(resident.next_pc);
        }
      }
      const u32 block_phys_first = block.phys_start;
      const u32 block_phys_last = block.phys_end;
      for (u32 page = block_phys_first >> 12u;
           page <= (block_phys_last >> 12u); ++page) {
        impl_->code_pages.mark_page(page);
      }
      auto inserted = impl_->blocks.emplace(start_pc, std::move(block));
      block_ptr = &inserted.first->second;
      impl_->remember_dispatch(*block_ptr);
      if (block_ptr->resident.fn != nullptr) {
        if (impl_->linked_dispatch.exit != nullptr) {
          try {
            auto linked = compile_native_alu(decoded, {}, block_ptr->load_retired,
                                             load_index, load_rt,
                                             &block_ptr->resident,
                                             impl_->linked_dispatch.exit);
            block_ptr->resident.linked_fn =
                impl_->arena.copy_code(linked->getCode(), linked->getSize());
            block_ptr->code_size += linked->getSize();
          } catch (const std::exception &e) {
            std::fprintf(stderr, "V3 linked block codegen: %s\n", e.what());
          }
        }
        if (block_ptr->has_load && block_ptr->resident.linked_fn == nullptr)
          block_ptr->resident.fn = nullptr;
        V3ResidentSlot *slot = impl_->resident_slot(start_pc);
        slot->block = block_ptr->resident.fn != nullptr && !block_ptr->has_load
                          ? &block_ptr->resident : nullptr;
        if (block_ptr->resident.linked_fn != nullptr)
          slot->entry = block_ptr->resident.linked_fn;
      }
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
      return helper_step(V3HelperReason::Internal);
    }
    Impl::Block &block = *block_ptr;
    if (block.instruction_count == 0u) {
      return helper_step(V3HelperReason::Internal);
    }
    if (block.kind == V3BlockKind::StepHelper) {
      count_v3_unsupported_opcode(stats_, block.words[0]);
      return helper_step(V3HelperReason::Unsupported);
    }
    if (block.fn == nullptr) {
      return helper_step(V3HelperReason::Internal);
    }

    if ((block.has_branch || block.has_jump) &&
        g_cpu_backend_compare_irq_on_branch) {
      return helper_step(V3HelperReason::Irq);
    }

    V3NativeRuntime runtime{};
    const Cpu::PendingLoad incoming_load = cpu_.load_;
    runtime.incoming_load_reg = incoming_load.reg;
    runtime.incoming_load_value = incoming_load.value;
    runtime.incoming_load_pc = incoming_load.source_pc;
    runtime.incoming_load_addr = incoming_load.source_addr;
    std::array<u32, 8> store_addrs{};
    u32 main_ram_store_count = 0u;
    u32 load_addr = 0u;
    u32 main_ram_load_count = 0u;

    if (block.has_load) {
      if (incoming_load.reg != 0u && block.load_index > 0u &&
          block.load_rs == incoming_load.reg) {
        return helper_step(V3HelperReason::State);
      }
      const u32 base = block.load_rs == 0u ? 0u : cpu_.gpr_[block.load_rs];
      load_addr = base + static_cast<u32>(block.load_simm);
      if ((load_addr & block.load_alignment_mask) != 0u) {
        return helper_step(V3HelperReason::Memory);
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
        return helper_step(V3HelperReason::Memory);
      }
    }

    if (block.has_store) {
      // Direct writes intentionally bypass System::write32. Anything that
      // requires tracing, watchpoints, MMIO semantics or isolated-cache store
      // behavior leaves the native tier before generated code is entered.
      if (g_trace_ram || g_trace_bus || g_ram_watch_diagnostics ||
          (cpu_.cop0_sr_ & (1u << 16)) != 0u) {
        return helper_step(V3HelperReason::Memory);
      }

      u8 *const main_ram = cpu_.sys_->jit_main_ram_data_mut();
      u8 *const scratchpad = cpu_.sys_->jit_scratchpad_data_mut();
      if (main_ram == nullptr || scratchpad == nullptr) {
        return helper_step(V3HelperReason::Memory);
      }

      for (u32 i = 0; i < block.store_count; ++i) {
        const u8 base_reg = block.store_rs[i];
        if (incoming_load.reg != 0u &&
            block.store_instruction_index[i] > 0u &&
            base_reg == incoming_load.reg) {
          return helper_step(V3HelperReason::State);
        }
        const u32 base = base_reg == 0u ? 0u : cpu_.gpr_[base_reg];
        const u32 addr = base + static_cast<u32>(block.store_simm[i]);
        store_addrs[i] = addr;

        if ((addr & 3u) != 0u) {
          return helper_step(V3HelperReason::Memory);
        }

        const u32 phys = psx::mask_address(addr);
        const u64 store_end = static_cast<u64>(phys) + 3u;
        const bool touches_current_code =
            phys <= block.phys_end &&
            store_end >= static_cast<u64>(block.phys_start);
        if (touches_current_code) {
          return helper_step(V3HelperReason::Memory);
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

        return helper_step(V3HelperReason::Memory);
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
      // A slice may end inside a longer native block. Execute its safe ALU
      // prefix natively, leaving the remaining PC for the next slice. This
      // preserves the R3000A pending-load commit in the first instruction.
      const u32 limit = std::min({remaining_instructions, remaining_cycles,
                                  block.instruction_count - 1u});
      std::vector<V3DecodedInstruction> prefix;
      prefix.reserve(limit);
      for (u32 i = 0; i < limit; ++i) {
        V3DecodedInstruction inst{};
        if (!decode_v3_alu(block.words[i], inst) ||
            !is_v3_alu_only(inst.op)) {
          break;
        }
        prefix.push_back(inst);
      }
      const u32 count = static_cast<u32>(prefix.size());
      if (count != 0u) {
        V3NativeFn &partial = block.prefix_fns[count];
        if (partial == nullptr) {
          try {
            auto generated = compile_native_alu(prefix, {}, false, 0u, 0u);
            partial = reinterpret_cast<V3NativeFn>(impl_->arena.copy_code(
                generated->getCode(), generated->getSize()));
          } catch (const std::exception &e) {
            std::fprintf(stderr, "V3 native prefix codegen: %s\n", e.what());
          }
        }
        if (partial != nullptr) {
          cpu_.executing_step_ = true;
          cpu_.exception_raised_ = false;
          cpu_.cycle_penalty_ = 0u;
          partial(cpu_.gpr_, &runtime);
          cpu_.executing_step_ = false;
          cpu_.pc_ = start_pc + count * 4u;
          cpu_.next_pc_ = cpu_.pc_ + 4u;
          cpu_.current_pc_ = cpu_.pc_ - 4u;
          cpu_.in_delay_slot_ = false;
          cpu_.active_branch_pc_ = 0u;
          cpu_.pending_delay_slot_ = false;
          cpu_.pending_branch_taken_ = false;
          cpu_.pending_branch_pc_ = 0u;
          cpu_.load_ = {};
          cpu_.next_load_ = {};
          cpu_.cycles_ += count;
          g_diag_current_pc = cpu_.current_pc_;
          cpu_.rr4_diag_state_.prev_pc_for_diag = cpu_.current_pc_;
          result.cycles += count;
          result.instructions += count;
          ++stats_.native_block_entries;
          ++stats_.native_alu_block_entries;
          stats_.native_instructions += count;
          stats_.jit_v2_inline_instructions += count;
          stats_.optimized_instructions += count;
          stats_.native_cycles += count;
          stats_.executed_cycles += count;
          native_streak = true;
          return true;
        }
      }
      return helper_step(V3HelperReason::Budget);
    }

    if (block.resident.fn != nullptr && impl_->resident_fn != nullptr) {
      V3ResidentContext resident{};
      resident.first = &block.resident;
      resident.gpr = cpu_.gpr_;
      resident.runtime = &runtime;
      resident.icache = reinterpret_cast<const u8 *>(cpu_.icache_.data());
      resident.main_ram = cpu_.sys_->jit_main_ram_data_mut();
      resident.scratchpad = cpu_.sys_->jit_scratchpad_data_mut();
      // read32_instruction() only takes the slower observable RAM path when
      // RAM tracing is enabled. Preserve that behavior by leaving refills on
      // the old C++ path in tracing sessions.
      resident.direct_icache_refill = g_trace_ram ? 0u : 1u;
      resident.cycle_budget = remaining_cycles;
      resident.instruction_budget = remaining_instructions;
      resident.branch_allowed = g_cpu_backend_compare_irq_on_branch ? 0u : 1u;
      cpu_.executing_step_ = true;
      cpu_.exception_raised_ = false;
      cpu_.cycle_penalty_ = 0u;
      if (impl_->use_linked && block.resident.linked_fn != nullptr &&
          impl_->linked_dispatch.entry != nullptr) {
        impl_->linked_dispatch.entry(&resident);
      } else {
        impl_->resident_fn(&resident);
      }
      cpu_.executing_step_ = false;
      if (resident.last == nullptr) {
        return helper_step(V3HelperReason::Budget);
      }

      const V3ResidentBlock &last = *resident.last;
      cpu_.pc_ = resident.final_pc;
      cpu_.next_pc_ = cpu_.pc_ + 4u;
      if (last.kind != 0u) {
        const u32 tail_pc = last.start_pc + last.tail_index * 4u;
        cpu_.current_pc_ = tail_pc + 4u;
        cpu_.in_delay_slot_ = true;
        cpu_.active_branch_pc_ = tail_pc;
      } else {
        cpu_.current_pc_ = last.start_pc +
                           (last.instruction_count - 1u) * 4u;
        cpu_.in_delay_slot_ = false;
        cpu_.active_branch_pc_ = 0u;
      }
      cpu_.pending_delay_slot_ = false;
      cpu_.pending_branch_taken_ = false;
      cpu_.pending_branch_pc_ = 0u;
      if (impl_->use_linked && block.resident.linked_fn != nullptr &&
          impl_->linked_dispatch.entry != nullptr &&
          runtime.incoming_load_reg != 0u) {
        cpu_.load_ = {static_cast<u8>(runtime.incoming_load_reg),
                      runtime.incoming_load_value, runtime.incoming_load_pc,
                      runtime.incoming_load_addr};
      } else {
        cpu_.load_ = {};
      }
      cpu_.next_load_ = {};
      cpu_.cycles_ += resident.cycles;
      g_diag_current_pc = cpu_.current_pc_;
      cpu_.rr4_diag_state_.prev_pc_for_diag = cpu_.current_pc_;

      result.cycles += resident.cycles;
      result.instructions += resident.instructions;
      stats_.native_block_entries += resident.entries;
      if (impl_->use_linked && block.resident.linked_fn != nullptr &&
          impl_->linked_dispatch.entry != nullptr) {
        ++stats_.native_chain_entries;
        stats_.native_linked_transitions += resident.entries - 1u;
        stats_.native_chain_max_blocks = std::max<u64>(
            stats_.native_chain_max_blocks, resident.entries);
      }
      stats_.native_alu_block_entries += resident.alu_entries;
      stats_.native_memory_block_entries += resident.memory_entries;
      stats_.native_branch_tail_entries += resident.branch_entries;
      stats_.native_branch_taken += resident.branch_taken;
      stats_.native_branch_not_taken += resident.branch_not_taken;
      stats_.native_instructions += resident.instructions;
      stats_.jit_v2_inline_instructions += resident.instructions;
      stats_.optimized_instructions += resident.instructions;
      stats_.native_cycles += resident.cycles;
      stats_.executed_cycles += resident.cycles;
      native_streak = true;
      return true;
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

    // The incoming pending load retired after this block's first instruction
    // inside generated code. A final native load becomes the new pending load.
    cpu_.load_ = {};
    cpu_.next_load_ = {};
    if (block.has_load && !block.load_retired) {
      if (block.load_rt != 0u) {
        const u32 load_pc = start_pc + block.load_index * 4u;
        cpu_.load_ = {block.load_rt, runtime.load_value, load_pc, load_addr};
      }
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
    native_streak = true;
    return true;
  };
#endif

  while (result.cycles < max_cycles &&
         result.instructions < max_instructions) {
#if VIBESTATION_JIT_V3_X64
    if (!try_native()) {
      // Universal V3 should only get here for an internal codegen/allocation
      // failure. Keep correctness with the generated step trampoline when
      // available; the raw interpreter is now the last-resort safety net.
      if (!helper_step(V3HelperReason::Internal)) {
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

void CpuJitV3Backend::invalidate_range(u32 phys_or_normalized_addr,
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

  if (!impl_->code_pages.any_page(first_page, last_page)) {
    ++stats_.invalidation_fast_no_code_page_exits;
    return;
  }

  for (auto it = impl_->blocks.begin(); it != impl_->blocks.end();) {
    const u32 block_first = it->second.phys_start & ~0x0Fu;
    const u32 block_last = it->second.phys_end & ~0x0Fu;
    const bool overlaps = block_first <= last_line && block_last >= first_line;
    if (overlaps) {
      impl_->forget_dispatch(it->second.start_pc);
#if VIBESTATION_JIT_V3_X64
      impl_->unlink_resident(it->second.start_pc);
#endif
      it = impl_->blocks.erase(it);
      ++stats_.invalidations;
      ++stats_.invalidation_blocks_invalidated;
    } else {
      ++it;
    }
  }
}

void CpuJitV3Backend::begin_frame(u32 frame_index) {
  current_frame_ = frame_index;
  stats_.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV3;
}

void CpuJitV3Backend::flush() {
  impl_->clear_dispatch();
  impl_->blocks.clear();
#if VIBESTATION_JIT_V3_X64
  impl_->delay_slot_code.clear();
#endif
  impl_->rejected_pcs.clear();
  impl_->rejected_pages.clear();
  impl_->code_pages.clear();
#if VIBESTATION_JIT_V3_X64
  impl_->arena.reset();
  impl_->step_helper_fn = install_step_helper(impl_->arena);
  impl_->resident_fn = install_resident_dispatch(impl_->arena);
  impl_->linked_dispatch = install_linked_dispatch(impl_->arena);
#endif
  stats_ = {};
  stats_.available = true;
#if VIBESTATION_JIT_V3_X64
  stats_.native_available = impl_->step_helper_fn != nullptr;
#else
  stats_.native_available = false;
#endif
  current_frame_ = 0;
}

CpuBackendStats CpuJitV3Backend::stats() const {
  CpuBackendStats out = stats_;
  out.available = true;
  out.active =
      effective_cpu_execution_mode() == CpuExecutionMode::X64JitV3;
#if VIBESTATION_JIT_V3_X64
  out.native_available = impl_->step_helper_fn != nullptr;
#else
  out.native_available = false;
#endif
  out.block_count = static_cast<u32>(impl_->blocks.size());
  out.native_blocks = static_cast<u64>(impl_->blocks.size());
#if VIBESTATION_JIT_V3_X64
  out.native_code_bytes = impl_->arena.bytes_used();
#else
  out.native_code_bytes = 0u;
#endif
  return out;
}
